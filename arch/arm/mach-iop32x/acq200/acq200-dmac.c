/* ------------------------------------------------------------------------- */
/* acq200-dmac.c driver for acq200/iop321 DMA controller                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of Version 2 of the GNU General Public License 
    as published by the Free Software Foundation;

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */


#define VERID "$Revision: 1.7 $ Build 1003 " __DATE__

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>

#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <asm/arch/iop321.h>
#include <asm/arch/iop321-irqs.h>
#include <asm/arch/iop321-dma.h>
#include <asm/arch-iop32x/acq200.h>

#include "acq200.h"
#include "acq200_minors.h"


#include "acq200-dmac.h"

#include "ringbuffer.h"

#include "acq200-inline-dma.h"


char* verid = VERID;
module_param(verid, charp, 0444);

int schedule_policy = DMA_CHANNEL_POLL_EZ;
module_param(schedule_policy, int, 0644);

/* FOUND emprically - this appears to be a Linux limit */
#define MAX_ALLOC 32768    

static struct DMACPOOL {
	struct dma_pool *pool;                 /* pool of iop321_dma_desc */
	int pool_alloc;
	int max_alloc;	
	struct u32_ringbuffer rb;
} DP;

static struct DmaChannelSync acq200_is_dma[2];


#define DMA_REGS_LEN (DMA_DCR+4)
#define DMA_REGS_CNT (DMA_REGS_LEN/sizeof(u32))

static struct DmaErrBuf {
	int channel;
	int ecount;
	struct DmaErr {
		u32 gtsr;
		u32 flags;
		u32 atusr;
		u32 atuisr;
		u32 pcsr;
		u32 dma_regs[DMA_REGS_CNT];		
	}
		buffers[2];   /* first, last - poorman's ring buffer */
} D_ERRB[2] = {
	{ .channel = 0 },
	{ .channel = 1 }
};
	       

struct DmaChannelSync* acq200_dma_getDmaChannelSync(void)
{
	return acq200_is_dma;
}

int acq200_dma_error_count(int channel)
{
	return D_ERRB[channel&1].ecount;
}

void acq200_init_interrupt_hook(
	int irq, struct InterruptSync *is, volatile u32* regs)
{
	is->irq = irq;
	is->interrupted = 0;
	init_waitqueue_head(&is->waitq);
	is->regs = regs;
	spin_lock_init(&is->bh_lock);
}

int acq200_dma_init_interrupt_hook(
	struct InterruptSync* is,
	unsigned int irq,
	irq_handler_t handler,
	const char * devname,
	volatile u32 *regs
	)
{
#define FLAGS 0
	acq200_init_interrupt_hook(irq, is, regs);

	if (is->requested==0){
		if (request_irq(irq, handler, FLAGS, devname, is) != 0){
			err( "request_irq( %d %s )", irq, devname);
			return -1;
		}else{
			is->enabled = 1;
			is->requested = 1;
		}
	}

	return 0;
#undef FLAGS
}


/*
 * PUBLIC INTERFACE IS HERE:
 * acq200_dmad_alloc() is a constant size, constant time allocator.
 * underlying dmad_pool_alloc() func prone to inconvenient sleeps,
 * and hence unsuited to RT.
 */

struct iop321_dma_desc* acq200_dmad_alloc(void)
{
	u32 item = 0;

	if (likely(u32rb_get(&DP.rb, &item))){
		return (struct iop321_dma_desc *)item;
	}else{
		BUG();
		return 0;
	}
}



void acq200_dmad_free(struct iop321_dma_desc* dmad)
{
	if (likely(u32rb_put(&DP.rb, (u32)dmad))){
		return;
	}else{
		BUG();
	}		       
}

void acq200_dmad_clear(void)
{
	DP.max_alloc = 0;
}


/*
 * inter module hook to TEE a DMA request 
 */



static int G_poll_caf;
static int G_calls;
static int G_poll_calls;

int acq200_post_dmac_request( 
	int channel, 
	u32 laddr,
	u32 offset,
	u32 remaddr, 
	u32 bc, 
	int incoming
)
{
	struct iop321_dma_desc *dmad;
	int timeout;
	int chn = channel&1;
	int poll = 
		(channel&DMA_CHANNEL_POLL) != 0 || 
		!acq200_is_dma[chn].eoc.isr_cb; 
	u32 ie = poll? 0: IOP321_DCR_IE;
	int poll_calls = 0;

	if (chn != 1) return -ENODEV;

	++G_calls;

	if (channel&DMA_CHANNEL_NOBLOCK){
		while(*IOP321_DMA1_CSR & IOP321_CSR_CAF){
			++G_poll_caf;
			poll_calls++;
		}
	}

	if (poll_calls){
		++G_poll_calls;
	}
	
	acq200_is_dma[chn].eoc.interrupted = 0;
	*IOP321_DMA1_CCR &= ~IOP321_CCR_CE;

	dmad = acq200_dmad_alloc();
	if (dmad ==0){
		return -ENOMEM;
	}

	dmad->NDA = 0;
	dmad->PUAD = 0;
	dmad->BC = bc;

	if (channel&DMA_CHANNEL_MEM2MEM){
		if (incoming){
			dmad->PDA = remaddr;
			dmad->LAD = laddr + offset;
		}else{
			dmad->PDA = laddr + offset;
			dmad->LAD = remaddr;
		}
		dmad->DC = DMA_DCR_MEM2MEM|ie;
	}else{
		dmad->PDA = remaddr;
		dmad->LAD = laddr + offset;
		dmad->DC = (incoming? DMA_DCR_PCI_MR: DMA_DCR_PCI_MW)|ie;
	}


	acq200_is_dma[chn].eoc.interrupted = 0;

	*IOP321_DMA1_NDAR = dmad->pa;
	*IOP321_DMA1_CCR = IOP321_CCR_CE;

	if (channel&DMA_CHANNEL_NOBLOCK){
		timeout = 0;
	}else if (poll){
		u32 status;
/* let me poll it ... */
		while(((status = *IOP321_DMA1_CSR)&IOP321_CSR_CAF) != 0){
			if (channel&DMA_CHANNEL_POLL_EZ){
				schedule();
			}
		}
		timeout = (status&IOP321_CSR_ERR) != 0;
	}else{
		timeout = !wait_event_interruptible_timeout(
			acq200_is_dma[chn].eoc.waitq,
			acq200_is_dma[chn].eoc.interrupted,
			100);
	}


	acq200_dmad_free(dmad);
	if (timeout) err( "TIMEOUT" );
	return timeout? -ETIMEDOUT: 0;
}


/*
 * PRIVATE FUNCTIONS - internal USE ONLY
 */
static struct iop321_dma_desc* _acq200_dmad_alloc(void)
{
#ifdef PGOMCOMOUT
	dbg(1, "pool %p pool_alloc %d max_alloc %d MAX_ALLOC %d",
	    DP.pool, DP.pool_alloc, DP.max_alloc, MAX_ALLOC);
#endif
	if (DP.pool_alloc >= MAX_ALLOC){
		return 0;
	}else{
		dma_addr_t pa;
		struct iop321_dma_desc *dmad = 
			dma_pool_alloc(DP.pool, GFP_ATOMIC, &pa);

		if (dmad){
			memset(dmad, 0, sizeof(struct iop321_dma_desc));
			dmad->pa = pa;

			DP.pool_alloc++;
			if (DP.pool_alloc > DP.max_alloc){
				DP.max_alloc = DP.pool_alloc;
			}
		}
#ifdef PGMCOMOUT
		dbg(1, "dmad %p", dmad);
		dbg(2, "dmad->pa 0x%08x dmad->vac 0x%p", dmad->pa, dmad->vac);
#endif
		return dmad;
	}
}





#define NO_ALLOC_RESTRICTION 0


static int __devinit map_local_resource(struct device * dev)
{
	DP.pool = dma_pool_create( 
		"acq200-dmad-pool", dev, 
		sizeof(struct iop321_dma_desc),
		sizeof(struct iop321_dma_desc),
		NO_ALLOC_RESTRICTION );

	if ( DP.pool == 0 ){
		err( "failed to allocate pool" );
		return -ENOMEM;
	}
	dbg( 1, "dma_pool_create() set DP.pool %p items %d\n", 
	     DP.pool, sizeof(struct iop321_dma_desc));

	u32rb_init(&DP.rb, MAX_ALLOC);

	while(!u32rb_is_full(&DP.rb)){
		u32 item = (u32)_acq200_dmad_alloc();
		if (item){
			u32rb_put(&DP.rb, item);
		}
	}
	info("acq200-dmac put %d descriptors", DP.rb.nput);
	DP.rb.nput = DP.rb.nget = 0;
	return 0;
}

static void destroy_local_resource(struct device * dev)
{
	u32rb_destroy(&DP.rb);
	dma_pool_destroy(DP.pool);
}


static char* dumpCSRbits(u32 regval)
{
	static char buf[32];

	buf[0] = '\0';

	if (regval&IOP321_CSR_ERR){
		strcat(buf, "ERR ");
	}
	if (regval&IOP321_CSR_EOCIF){
		strcat(buf, "EOC ");
	}
	if (regval&IOP321_CSR_EOTIF){
		strcat(buf, "EOT ");
	}
	if (regval&IOP321_CSR_CAF){
		strcat(buf, "CAF ");
	}
	return buf;
}

static char* dumpEmpty(u32 regval)
{
	return "";
}
static char* dumpCCRbits(u32 regval)
{
	static char buf [8];
	
	buf[0] = '\0';
	if (regval&IOP321_CCR_CE){
		strcat(buf, "CE ");
	}
	if (regval&IOP321_CCR_CR){
		strcat(buf, "CR ");
	}
	return buf;
}
static char* dumpDCRbits(u32 regval)
{
	static char buf[32];

	buf[0] = '\0';
	
	if (regval&IOP321_DCR_MMTE){
		strcat(buf, "MMTE ");
	}
	if (regval&IOP321_DCR_IE){
		strcat(buf, "IE ");
	}
	if (regval&IOP321_DCR_PCITR){
		strcat(buf, regval&1? "PCIW ": "PCIR ");
	}
	return buf;
}

#define _DMA_REG(base, boffset) *(volatile u32*)((char*)(base)+(boffset))

static irqreturn_t dma_irq_err(int irq, void *dev_id)
{
	struct InterruptSync* is = (struct InterruptSync*)dev_id;
	u32 flags = _DMA_REG(is->regs,DMA_CSR);
	_DMA_REG(is->regs,DMA_CSR) = flags;

	if (is->isr_cb) is->isr_cb( is, flags );
	
	return IRQ_HANDLED;
} 


#define ATUSR_ERR  0xf900
#define PCSR_ADDR_PARITY_ERR (1<<18)

static void dma_err_callback(struct InterruptSync *self, u32 flags)
{
	int ichan = self->channel;
	int ibuf = D_ERRB[ichan].ecount++ > 0;
	struct DmaErr *dmaErr = &D_ERRB[ichan].buffers[ibuf];
	volatile void* dma_regs = ichan? IOP321_DMA1_CCR: IOP321_DMA0_CCR;

	dmaErr->gtsr   = *IOP321_GTSR;
	dmaErr->flags  = flags;
	dmaErr->atusr  = *IOP321_ATUSR;
	dmaErr->pcsr   = *IOP321_PCSR;
	dmaErr->atuisr = *IOP321_ATUISR;

	if ((dmaErr->atusr&ATUSR_ERR) != 0){
		*IOP321_ATUSR |= ATUSR_ERR;
	}
	if ((dmaErr->pcsr&PCSR_ADDR_PARITY_ERR) != 0){
		*IOP321_PCSR |= PCSR_ADDR_PARITY_ERR;
	}
	if (dmaErr->atuisr){
		*IOP321_ATUISR = dmaErr->atuisr;
	}

	memcpy(dmaErr->dma_regs, (void*)dma_regs, DMA_REGS_LEN);
}

static 	ssize_t show_pci_errors(
	struct device *dev, char * buf, int ichannel, int ibuf)
{
	struct DmaErr *dmaErr = &D_ERRB[ichannel].buffers[ibuf];
	int len = 0;
#define PRINTF(fmt, arg...) len += sprintf(buf+len, fmt, ##arg)
	const char* parity_err = "";
	const char* atu_stat = "";

	if ((dmaErr->pcsr&PCSR_ADDR_PARITY_ERR) != 0){
		parity_err = "PARITY ERR";
	}

	PRINTF("DMA ERROR DMA%d_CSR:0x%08x"
		     " atusr:%08x"
		     " pcsr:%08x %s"
		     " atuisr:%08x %s\n", 		   
		     ichannel, dmaErr->flags,
		     (dmaErr->atusr & ATUSR_ERR),
		     dmaErr->pcsr, parity_err,
		     dmaErr->atuisr, atu_stat 
		);

	return len;
#undef PRINTF
}

static ssize_t show_dmac_regs(
	struct device *dev, char * buf, volatile u32* base)
{
	int len = 0;
#define DMAREG(boff) (base+((boff)/4))
#define DPRINTF(boff, flagger)						    \
        do {								    \
		u32 regval = *DMAREG(boff);				    \
		len += sprintf(buf+len, 				    \
			       "%p %10s 0x%08x %s\n",			    \
		               DMAREG(boff), #boff, regval, flagger(regval));\
	} while(0)

	DPRINTF(DMA_CCR, dumpCCRbits);
	DPRINTF(DMA_CSR, dumpCSRbits);
	DPRINTF(DMA_DAR, dumpEmpty);
	DPRINTF(DMA_NDAR,dumpEmpty);
	DPRINTF(DMA_PADR,dumpEmpty);
	DPRINTF(DMA_PUADR,dumpEmpty);
	DPRINTF(DMA_LADR,dumpEmpty);
	DPRINTF(DMA_BCR, dumpEmpty);
	DPRINTF(DMA_DCR, dumpDCRbits);
#undef DPRINTF
#undef DMAREG
	return len;
}



static ssize_t show_dmac0_regs(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return show_dmac_regs(dev, buf, IOP321_DMA0_CCR);
}
static DEVICE_ATTR(dmac0_regs, S_IRUGO, show_dmac0_regs, 0);

static ssize_t show_dmac1_regs(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return show_dmac_regs(dev, buf, IOP321_DMA1_CCR);
}
static DEVICE_ATTR(dmac1_regs, S_IRUGO, show_dmac1_regs, 0);


static ssize_t show_rb(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return u32rb_printf(buf, &DP.rb);
}
static DEVICE_ATTR(rb_state, S_IRUGO, show_rb, 0);



static ssize_t show_dmac_errs(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf, int ichan)
{
	struct InterruptSync *is = &acq200_is_dma[ichan].err;
	struct DmaErrBuf deb;
	int len = 0;
	unsigned long flags;
	
	spin_lock_irqsave(&is->bh_lock, flags);
	memcpy(&deb, &D_ERRB[ichan], sizeof(struct DmaErrBuf));
	D_ERRB[ichan].ecount = 0;
	spin_unlock_irqrestore(&is->bh_lock, flags);

	if (deb.ecount){
		u32* dma_regs = deb.buffers[0].dma_regs;
		len += sprintf(buf, "dma channel %d errcount %d\n", 
			       ichan, deb.ecount);

		len += show_pci_errors(dev, buf+len, ichan, 0);
		len += show_dmac_regs(dev, buf+len, dma_regs);

		if (deb.ecount > 1){
			dma_regs = deb.buffers[1].dma_regs;
			len += show_pci_errors(dev, buf+len, ichan, 1);
			len += show_dmac_regs(dev, buf+len, dma_regs);
		}
		return len;
	}else{
		return 0;
	}
}

static ssize_t show_dmac0_errs(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return show_dmac_errs(dev, attr, buf, 0);
}

static DEVICE_ATTR(dmac0_errs, S_IRUGO, show_dmac0_errs, 0);

static ssize_t show_dmac1_errs(
	struct device *dev, 
	struct device_attribute *attr, 
	char *buf)
{
	return show_dmac_errs(dev, attr, buf, 0);
}
static DEVICE_ATTR(dmac1_errs, S_IRUGO, show_dmac1_errs, 0);

static ssize_t show_dmac_state(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)

	len += PRINTF("%15s: %p\n", "pool", DP.pool);
	len += PRINTF("%15s: %d\n", "pool_alloc", DP.pool_alloc);
	len += PRINTF("%15s: %d\n", "max_alloc", DP.max_alloc);

	len += PRINTF("%15s: %d\n", "calls", G_calls);
	len += PRINTF("%15s: %d\n", "poll_calls", G_poll_calls);
	len += PRINTF("%15s: %d\n", "waiting CAF", G_poll_caf);
	return len;
#undef PRINTF
}

static DEVICE_ATTR(dmac_state,S_IRUGO,show_dmac_state,0);


/**
 *   app hook to run a oneshot DMA
 */

static struct OneShot {
	int channel;
	unsigned laddr, remaddr, bc;
	int result;
	int ident;
	unsigned throttle;
} oneshot;


static void run_oneshot_throttle(struct OneShot* ot, int incoming)
{
	unsigned offset;
	unsigned block_len;
	int rc;

	for (offset = 0; 
	     (block_len = min(ot->throttle, ot->bc - offset)) > 0; 
	     offset += block_len){		
		rc = acq200_post_dmac_request(
			ot->channel|schedule_policy,
			ot->laddr+offset, 0,
			ot->remaddr+offset, 
			block_len,
			incoming);
	}
}
static ssize_t run_oneshot(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count, int channel, int incoming)
{
	int nargs = sscanf(buf, "%x %x %x %d", 
			   &oneshot.laddr, &oneshot.remaddr, &oneshot.bc, 
			   &oneshot.throttle);
	oneshot.channel = channel;
	if (nargs == 3){
		oneshot.ident++;
		oneshot.result = acq200_post_dmac_request(
			channel, 
			oneshot.laddr, 0, oneshot.remaddr, 
			oneshot.bc, incoming);
	}else if (nargs == 4){
		run_oneshot_throttle(&oneshot, incoming);
	}

	return strlen(buf);
}

static ssize_t show_oneshot(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "0x%08x 0x%08x 0x%x [%d] result %d\n",
		       oneshot.laddr, oneshot.remaddr, oneshot.bc, 
		       oneshot.ident, oneshot.result);
}

static DEVICE_ATTR(oneshot_result, S_IRUGO, show_oneshot, 0);

#define DECLARE_CHANNEL_ACCESS(name, channel, incoming, mem2mem)	\
static ssize_t oneshot_##name(						\
	struct device* dev,						\
	struct device_attribute *attr,					\
	const char* buf, size_t count )					\
{									\
	return run_oneshot(dev, attr, buf, count,			\
			   channel|mem2mem|DMA_CHANNEL_POLL, incoming);	\
}									\
static DEVICE_ATTR(oneshot_##name, S_IWUGO, 0, oneshot_##name);

#define DECLARE_CHANNEL_ACCESS_GROUP(channel)			\
DECLARE_CHANNEL_ACCESS(channel##_I_m2m, channel, 1, DMA_CHANNEL_MEM2MEM); \
DECLARE_CHANNEL_ACCESS(channel##_O_m2m, channel, 0, DMA_CHANNEL_MEM2MEM); \
DECLARE_CHANNEL_ACCESS(channel##_I_pci, channel, 1, 0);	\
DECLARE_CHANNEL_ACCESS(channel##_O_pci, channel, 0, 0);

#define DEVICE_CREATE_CHANNEL_ACCESS_GROUP(dev, channel)	\
DEVICE_CREATE_FILE(dev, &dev_attr_oneshot_##channel##_I_m2m);		\
DEVICE_CREATE_FILE(dev, &dev_attr_oneshot_##channel##_O_m2m);		\
DEVICE_CREATE_FILE(dev, &dev_attr_oneshot_##channel##_I_pci);		\
DEVICE_CREATE_FILE(dev, &dev_attr_oneshot_##channel##_O_pci);

#define DEVICE_REMOVE_CHANNEL_ACCESS_GROUP(dev, channel)	\
device_remove_file(dev, &dev_attr_oneshot_##channel##_I_m2m);		\
device_remove_file(dev, &dev_attr_oneshot_##channel##_O_m2m);		\
device_remove_file(dev, &dev_attr_oneshot_##channel##_I_pci);		\
device_remove_file(dev, &dev_attr_oneshot_##channel##_O_pci);

DECLARE_CHANNEL_ACCESS_GROUP(0);
DECLARE_CHANNEL_ACCESS_GROUP(1);

static void mk_sysfs(struct device *dev)
{
	dbg(1,  "calling DEVICE_CREATE_FILE dev %p", dev );

	DEVICE_CREATE_FILE(dev, &dev_attr_dmac_state);
	DEVICE_CREATE_FILE(dev, &dev_attr_dmac0_regs);
	DEVICE_CREATE_FILE(dev, &dev_attr_dmac1_regs);
	DEVICE_CREATE_FILE(dev, &dev_attr_rb_state);

	DEVICE_CREATE_CHANNEL_ACCESS_GROUP(dev, 0);
	DEVICE_CREATE_CHANNEL_ACCESS_GROUP(dev, 1);
	DEVICE_CREATE_FILE(dev, &dev_attr_oneshot_result);

	DEVICE_CREATE_FILE(dev, &dev_attr_dmac0_errs);
	DEVICE_CREATE_FILE(dev, &dev_attr_dmac1_errs);
}

static void rm_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_dmac_state);
	device_remove_file(dev, &dev_attr_dmac0_regs);
	device_remove_file(dev, &dev_attr_dmac1_regs);
	device_remove_file(dev, &dev_attr_rb_state);

	DEVICE_REMOVE_CHANNEL_ACCESS_GROUP(dev, 0);
	DEVICE_REMOVE_CHANNEL_ACCESS_GROUP(dev, 1);
	device_remove_file(dev, &dev_attr_oneshot_result);

	device_remove_file(dev, &dev_attr_dmac0_errs);
	device_remove_file(dev, &dev_attr_dmac1_errs);
}


static int acq200_dmac_probe(struct device * dev)
{
	info("acq200dmac %s", VERID);
	map_local_resource(dev);
	mk_sysfs(dev);
	return 0;
}



static int acq200_dmac_remove(struct device * dev)
{
	rm_sysfs(dev);
	destroy_local_resource(dev);
	return 0;
}





static struct device_driver dmac_device_driver = {
	.name		= "acq200dmac",
	.bus		= &platform_bus_type,
	.probe          = acq200_dmac_probe,
	.remove         = acq200_dmac_remove,
};

static void acq200_dmac_dev_release(struct device * dev)
{

}

static u64 dma_mask = ~(u32)0;

static struct platform_device dmac_device = {
	.name		= "acq200dmac",
	.id		= 0,
	.dev = {
		.kobj.name	= "acq200dmac",
		.release        = acq200_dmac_dev_release,
		.dma_mask       = &dma_mask,
		.coherent_dma_mask = ~(u32)0
	}
};


static int __init acq200_dmac_init( void )
{
	int rc = driver_register(&dmac_device_driver);

	if (rc != 0){
		return rc;
	}

	DMA_REQUEST_IRQ(acq200_is_dma, ERR, 0, err);
	DMA_REQUEST_IRQ(acq200_is_dma, ERR, 1, err);

	acq200_is_dma[0].err.isr_cb = dma_err_callback;
	acq200_is_dma[1].err.isr_cb = dma_err_callback;
	
	return platform_device_register(&dmac_device);
}


static void __exit
acq200_dmac_exit_module(void)
// Remove DRIVER resources on module exit
{
	DMA_FREE_IRQ(acq200_is_dma, err, 0);
	DMA_FREE_IRQ(acq200_is_dma, err, 1);
	platform_device_unregister(&dmac_device);	
	driver_unregister(&dmac_device_driver);
}

module_init(acq200_dmac_init);
module_exit(acq200_dmac_exit_module);


EXPORT_SYMBOL_GPL(acq200_init_interrupt_hook);
EXPORT_SYMBOL_GPL(acq200_dma_init_interrupt_hook);

EXPORT_SYMBOL_GPL(acq200_post_dmac_request);
EXPORT_SYMBOL_GPL(acq200_dma_getDmaChannelSync);
EXPORT_SYMBOL_GPL(acq200_dmad_alloc);
EXPORT_SYMBOL_GPL(acq200_dmad_free);
EXPORT_SYMBOL_GPL(acq200_dmad_clear);
EXPORT_SYMBOL_GPL(acq200_dma_error_count);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 DMA Controller");

