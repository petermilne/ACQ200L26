/* ------------------------------------------------------------------------- */
/* acq200-mu.c driver for acq200/iop321 message unit (MU)                    */
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
/* 
 * Mailboxes: not done. Use mapping from acq200-core.
 *            WORKTOO: hook inbound mailbox interrupt.
 *
 * Circular Queues: MUQ
 *            Set up MUQ memory on load.
 *            The buffers are provided by external app (how)
 *
 *            services:
 *                   write_outbound_post
 *                   read_outbound_free
 *                   read_inbound_post
 *                   write_inbound_free
 *            all read/writes are in 4 byte units
 *            all block until data is available.
 *
 * We have two memory areas
 * mumem   - local memory used to implement circular queues
 * hostmem - host memory slaved on pci visable through a constant addr window
 *
 * hostmem is divvied up into
 * RMA's - variable host side bufs
 * MFA's - message frame address - fixed 1K host bufs
 *
 * Both RMA and MFA are bytes offsets from hostmem_base
 *         
 * mumem is dvided into the 4 circular message queues
 * Q entries in mumem are MFA's. 
 * Q pointer values are byte indexes into mumem;
 */

#define USE_INTERRUPTS 1
#define USE_MAILBOX_INTERRUPTS 0

/* lazy alloc - in principle, host can define buf len. In practise, no */
#define MU_LAZY_ALLOC	1

static const char* VERID =
	"$Revision: 1.6 $ Build 1003 " __DATE__ "\n"
	"Features:"
#if (MU_LAZY_ALLOC)
	"MU_LAZY_ALLOC "
#endif
#if (USE_INTERRUPTS)
	"USE_INTERRUPTS "
#endif
	;


#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
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


int acq200_debug;

#include "acq32busprot.h"
#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"
#include "acq200_minors.h"
#include "ringbuffer.h"
#include "acq200-dmac.h"
#include "acq200-mu.h"

#include "acq200-mu-app.h"
#define MAXCHAIN MAX_RMA_GROUP
#include "acq200-inline-dma.h"




#include <asm/arch/iop321.h>
#include <asm/arch/iop321-irqs.h>

#define QLEN
#define QMASK 0xfff

#define ACQ216_BRIDGE_WINDOW_BASE 0x70000000
#define ACQ216_BRIDGE_WINDOW_SIZE 0x01000000
#define ACQ216_BRIDGE_WINDOW_MASK (ACQ216_BRIDGE_WINDOW_SIZE-1)

int acq200_databuf_debug = 0;
module_param(acq200_databuf_debug, int, 0664);

module_param(acq200_debug, int, 0664);

int HBPHYS;
module_param(HBPHYS, int, 0444);

int HBLEN = _HBLEN24;
module_param(HBLEN, int, 0444);

int HBBLOCK = _HBBLOCK24;
module_param(HBBLOCK, int, 0444);

int downstream_is_set = 0;
module_param(downstream_is_set, int, 0444);


int dma1_IE = 1;
module_param(dma1_IE, int, 0644);


unsigned max_dma = 0;			/* 0 => no maximum */
module_param(max_dma, int, 0644);

#define EOT_TO_TICKS (HZ/10)
int eot_to_ticks = EOT_TO_TICKS;
module_param(eot_to_ticks, int, 0644);


/* for testing ONLY: truncates data dma bursts */
int debug_dma_chunk_clip = INT_MAX;
module_param(debug_dma_chunk_clip, int, 0644);

#define DBDBG(lvl, format, args...) \
        if (acq200_databuf_debug>lvl ) info( "DBDBG:" format, ##args)

#define INCR(qp) (((qp+4)&(ACQ200_MU_QSZ-1))|((qp)&~(ACQ200_MU_QSZ-1)))


#define machine_has_bridge() machine_is_acq200()


static inline int isEmpty( MFA tail, MFA head )
{
	return tail == head;
}

static inline int isFull(MFA tail, MFA head)
{
	return INCR(head) == tail;
}

unsigned acq200mu_get_hb_phys(void) {
	return HBPHYS;
}

unsigned acq200mu_get_hb_len(void) {
	return HBLEN;
}

static int qCount(MFA tail, MFA head)
/* NB: this is expensive */
{
	int count = 0;
	int tp;

	for( tp = tail; tp != head; tp = INCR(tp), ++count ){
		;
	}
	return count;
}

struct MU_PATH_DESCRIPTOR {
	unsigned lbuf[MSQSIZE/sizeof(unsigned)];
};
#define MUPD(file) ((struct MU_PATH_DESCRIPTOR*)((file)->private_data))
#define LBUF(file) (MUPD(file)->lbuf)


int acq200_mu_major;

static struct ACQ200_MU_DEVICE {
	struct device *dev;
	u32 host_base_pa;
	char* host_base_va;
	int host_window_offset;
	struct resource mumem;
	dma_addr_t mumem_dmad;
	u32 host_base_len;
	wait_queue_head_t ip_waitq;
	wait_queue_head_t mb_waitq;
	int mb_interrupt_rx;
	int mailbox_inbound_active;
	int mb_count;
	struct PCI_DMA_BUFFER *databufs;
	struct STATS {
		unsigned n_ops;
	}
	IP, IF, OP, OF;

	struct resource bigbuf;
	u32 rma_base;                     /* remote PCI base bus addr */
} mug = {
	.host_base_pa = ACQ200_PCIMEM,    /* WORKTODO: FIXME PLEASE */
	.host_base_len= 0x01000000,
};

#define DATABUFS_SZ	(sizeof(struct PCI_DMA_BUFFER)*NDATABUFS)

static inline int mumem_len(void)
{
	return mug.mumem.end - mug.mumem.start;
}
static inline u32 mumem_pa(void)
{
	return  virt_to_phys((void*)mug.mumem.start);
}
static inline void* mumem_va(void)
{
	return (void*)mug.mumem.start;
}


u32* mfa2va(MFA mfa)
{
	return (u32*)((u32)mug.host_base_va +
		      mug.host_window_offset + (mfa&MFA_MASK));
}
u32 mfa2pa(MFA mfa)
{
	return mug.host_base_pa + mug.host_window_offset + (mfa&MFA_MASK);
}
void* rma2va( RMA rma )
{
	return (u32*)((u32)mug.host_base_va +
		      mug.host_window_offset + (rma&MFA_MASK));
}
u32 mfa2pci(MFA mfa)
/* WARNING: assumes pa == bus */
{
	return mug.host_base_pa+mug.host_window_offset+(mfa&MFA_MASK);
}
u32 rma2pci( RMA rma )
{
	return mug.host_base_pa+mug.host_window_offset+(rma&MFA_MASK);
}

static inline u32* qp2va( u32 qp )
{
	return (u32*)((u32)mumem_va()+(qp&QP_MASK));
}

void* bbva(u32 offset)
{
	return (void*)(mug.bigbuf.start + offset);
}

static void msgq_fill( 
	unsigned long va, unsigned id, unsigned v1, unsigned nmsgs )
{
	u32 *pbuf = (u32*)va;
	
	while( nmsgs-- ){
		*pbuf++ = id | v1;
		v1 += MSQSIZE;
	}
}



static ssize_t show_queue_state(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf
	)
{
	int len = 0;
#define HEAD(field) (IOP321_##field##HPR)
#define TAIL(field) (IOP321_##field##TPR)

#define QDB(field) \
        sprintf( buf+len, \
		 "%10lu %2s T: %08x [%08x] H: %08x [%08x] %4d %10d\n",\
		 jiffies, #field, \
		 *TAIL(field), *qp2va(*TAIL(field)),\
		 *HEAD(field), *qp2va(*HEAD(field)),\
		 qCount(*TAIL(field), *HEAD(field)),\
		mug. field . n_ops)

	len += QDB( IF );
	len += QDB( IP );
	len += QDB( OF );
	len += QDB( OP );
	return len;
}
static DEVICE_ATTR(queue_state, S_IRUGO|S_IWUGO, show_queue_state, 0);


static ssize_t _show_queue_entries(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf, int qfirst, int nshow )
{
#define SQE sprintf(buf+len, "0x%08x [0x%08x]\n", \
        mumem_pa()+qfirst+ishow*4, qbase[ishow] )
		   
	u32* qbase = (u32*)((char*)mumem_va()+qfirst);
	int len = 0;
	int ishow;

	for ( ishow = 0; ishow != nshow; ++ishow ){
		len += SQE;
	}
	return len;
}


static ssize_t show_ipq_entries(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return _show_queue_entries(dev, attr, buf, ACQ200_MU_IPQ_FIRST, 8);
}
static DEVICE_ATTR(ipq_entries, S_IRUGO,show_ipq_entries,0);

static ssize_t show_ifq_entries(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return _show_queue_entries(dev, attr, buf, ACQ200_MU_IFQ_FIRST, 8);
}
static DEVICE_ATTR(ifq_entries, S_IRUGO,show_ifq_entries,0);

static ssize_t show_opq_entries(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return _show_queue_entries(dev, attr, buf, ACQ200_MU_OPQ_FIRST, 8);
}
static DEVICE_ATTR(opq_entries, S_IRUGO,show_opq_entries,0);

static ssize_t show_ofq_entries(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return _show_queue_entries(dev, attr, buf, ACQ200_MU_OFQ_FIRST, 8);
}
static DEVICE_ATTR(ofq_entries, S_IRUGO,show_ofq_entries,0);



static ssize_t show_host_base(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "stub - mostly harmless\n" );
}

static ssize_t set_host_base(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	return strlen(buf);
}


static DEVICE_ATTR(host_base, S_IRUGO|S_IWUGO,
		   show_host_base, set_host_base);





static int acq200_mu_ipq_int_enable = 1;

static ssize_t show_mu_ipq_int_enable(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "%d\n", acq200_mu_ipq_int_enable);
}

static ssize_t set_mu_ipq_int_enable(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf( buf, "%d", &acq200_mu_ipq_int_enable ) == 1){
		if (acq200_mu_ipq_int_enable){
			acq200_unmask_irq(IRQ_IOP321_MU_IPQ);
		}else{
			acq200_mask_irq(IRQ_IOP321_MU_IPQ);
		}
	}
	return strlen(buf);
}

static DEVICE_ATTR(mu_ipq_int_enable, S_IRUGO|S_IWUGO,
		   show_mu_ipq_int_enable, set_mu_ipq_int_enable);

static ssize_t show_OIMR(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "0x%x\n", *IOP321_OIMR);
}

static ssize_t set_OIMR(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned mask;

	if (sscanf( buf, "0x%x", &mask ) == 1 ){
		*IOP321_OIMR = mask;
	}
	return strlen(buf);
}


static DEVICE_ATTR(OIMR, S_IRUGO|S_IWUSR, show_OIMR, set_OIMR);

static unsigned dmad_alloc_count;
static unsigned dmad_free_count;

typedef struct iop321_dma_desc *DMAD;

static inline struct iop321_dma_desc *dmad_alloc(void)
{
	++dmad_alloc_count;
	return acq200_dmad_alloc();
}

void dmad_free(struct iop321_dma_desc* dmad)
{
	++dmad_free_count;
	acq200_dmad_free(dmad);
}
static ssize_t show_dmad_counts(	
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "dmad: a: %u p: %u out:%u\n",
			dmad_alloc_count, dmad_free_count, 
			dmad_alloc_count - dmad_free_count);
}

static DEVICE_ATTR(dmad_counts, S_IRUGO|S_IWUSR, show_dmad_counts, 0);


static ssize_t show_version(
	struct device_driver *driver, char * buf)
{
        return sprintf(buf,"%s\n", VERID);
}
static DRIVER_ATTR(version,S_IRUGO,show_version,0);

struct EOT_stats {
	unsigned new_chain;
	unsigned interrupted;
	unsigned timeout;
	unsigned uptodate;
	unsigned reload;
	unsigned fire;
} S_EOT_stats;


static ssize_t set_EOT_stats(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned mask;

	if (sscanf( buf, "0x%x", &mask ) == 1 && mask ==1){
		memset(&S_EOT_stats, 0, sizeof(S_EOT_stats));		
	}
	return strlen(buf);
}

static ssize_t show_EOT_stats(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf
	)
{
#define UF	"%5u"
	return sprintf(buf, UF "," UF "," UF "," UF "," UF "," UF "\n",
		       S_EOT_stats.new_chain,	
		       S_EOT_stats.interrupted,	
		       S_EOT_stats.timeout,	
		       S_EOT_stats.uptodate,
		       S_EOT_stats.reload,	
		       S_EOT_stats.fire);
}

static DEVICE_ATTR(EOT_stats, S_IRUGO|S_IWUGO, show_EOT_stats, set_EOT_stats);


#define UPSTATS(field, bf)  if (bf){ S_EOT_stats . field ++; }


extern int iop32x_pci_bus_speed(void);

static ssize_t show_pci_bus_speed(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf
	)
{
	return sprintf(buf, "%d\n", iop32x_pci_bus_speed()/1000000);
}

static DEVICE_ATTR(pci_bus_speed, S_IRUGO, show_pci_bus_speed, 0);

extern int iop32x_pbi_bus_speed(void);

static ssize_t show_pbi_bus_speed(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf
	)
{
	return sprintf(buf, "%d\n", iop32x_pbi_bus_speed());
}

static DEVICE_ATTR(pbi_bus_speed, S_IRUGO, show_pbi_bus_speed, 0);



static void mk_sysfs(struct device *dev)
{
	dbg(1,  "calling device_create_file dev %p", dev );


	DEVICE_CREATE_FILE(dev, &dev_attr_host_base);
	DEVICE_CREATE_FILE(dev, &dev_attr_queue_state);
	DEVICE_CREATE_FILE(dev, &dev_attr_ipq_entries);
	DEVICE_CREATE_FILE(dev, &dev_attr_ifq_entries);
	DEVICE_CREATE_FILE(dev, &dev_attr_opq_entries);
	DEVICE_CREATE_FILE(dev, &dev_attr_ofq_entries);
	DEVICE_CREATE_FILE(dev, &dev_attr_mu_ipq_int_enable);
	DEVICE_CREATE_FILE(dev, &dev_attr_OIMR);
	DEVICE_CREATE_FILE(dev, &dev_attr_dmad_counts);
	DEVICE_CREATE_FILE(dev, &dev_attr_EOT_stats);
	DEVICE_CREATE_FILE(dev, &dev_attr_pci_bus_speed);
	DEVICE_CREATE_FILE(dev, &dev_attr_pbi_bus_speed);

	DRIVER_CREATE_FILE(dev->driver, &driver_attr_version);
}

static void rm_sysfs(struct device *dev)
{

}



static irqreturn_t acq200_mb_isr(int irq, void *dev_id)
{
	u32 iisr = *IOP321_IISR;

	iisr &= ~IOP321_IIxR_IPQ;
	*IOP321_IISR = iisr;

	if (iisr&IOP321_IIxR_IM0){		
		iisr &= ~IOP321_IIxR_IM0;
		mug.mb_interrupt_rx = 1;
		mug.mb_count++;
		wake_up_interruptible_all(&mug.mb_waitq);
	}
	if (iisr){
		dbg(1, "Attempting to clear SPURIOUS interrupt status 0x%02x", 
		       iisr );
	}
	return IRQ_HANDLED;
}

static irqreturn_t acq200_mu_ipq_isr(int irq, void *dev_id)
{
	u32 iisr = *IOP321_IISR;

	
	if ( iisr&IOP321_IIxR_IPQ ){
		*IOP321_IISR = IOP321_IIxR_IPQ;
		iisr &= ~IOP321_IIxR_IPQ;
		wake_up_interruptible_all(&mug.ip_waitq);
	}

	return IRQ_HANDLED;
}


#ifdef PGMCOMOUT
/* acq32-drv asserts on HIBYTES != 0 */
#define ID_IN  0xd1000000
#define ID_OUT 0xd0000000
#else
#define ID_IN  0
#define ID_OUT 0
#define ID_NFG 0xbf000000
#endif


#define BIGSYNC \
        wmb(); \
        dma_sync_single(0, mug.mumem_dmad, mumem_len(), DMA_BIDIRECTIONAL);\


#define LILSYNCI(qp) \
        dma_sync_single(0, mug.mumem_dmad+((qp)&QP_MASK), 4, DMA_FROM_DEVICE)

#define LILSYNCO(qp)							   \
        do {								   \
	        wmb();							   \
                dma_sync_single(					   \
                        0, mug.mumem_dmad+((qp)&QP_MASK), 4, DMA_TO_DEVICE); \
        }while(0)

static void init_mu_queues(void)
{
	unsigned long vaddr = (unsigned long)mumem_va();
	unsigned paddr = mumem_pa();

	dbg(1,  "MUMEM: va %p pa 0x%08x len 0x%08x",
	      mumem_va(), mumem_pa(), mumem_len() );

	assert((paddr&(ACQ200_MU_QALIGN-1))==0);  /* MUST BE 1MB aligned */

	/* IFQ Full start at empty, then full afterawards*/

	*IOP321_IFHPR = ACQ200_MU_IFQ_FIRST;
	*IOP321_IFTPR = ACQ200_MU_IFQ_FIRST;
	/* IPQ Empty */
	*IOP321_IPHPR = ACQ200_MU_IPQ_FIRST;
	*IOP321_IPTPR = ACQ200_MU_IPQ_FIRST;
	/* OFQ Full */
	*IOP321_OFHPR = ACQ200_MU_OFQ_FIRST+ACQ200_MU_ENTRY(NOBMSGS);
	*IOP321_OFTPR = ACQ200_MU_OFQ_FIRST;
	/* OPQ Empty */
	*IOP321_OPHPR = ACQ200_MU_OPQ_FIRST;
	*IOP321_OPTPR = ACQ200_MU_OPQ_FIRST;

	msgq_fill(vaddr+ACQ200_MU_IFQ_FIRST, ID_IN,  MSGQBASE_IN,  NIBMSGS);
	msgq_fill(vaddr+ACQ200_MU_IPQ_FIRST, ID_NFG, MSGQBASE_IN,  NIBMSGS);
	msgq_fill(vaddr+ACQ200_MU_OFQ_FIRST, ID_OUT, MSGQBASE_OUT, NOBMSGS);
	msgq_fill(vaddr+ACQ200_MU_OPQ_FIRST, ID_NFG, MSGQBASE_OUT, NOBMSGS);
	
	BIGSYNC;

	*IOP321_QBAR = paddr;
	*IOP321_MUCR = 0x3; /* ENABLE, min size = 16K */

	/* force MU to preload IFT */
	*IOP321_IFHPR = ACQ200_MU_IFQ_FIRST+ACQ200_MU_ENTRY(NIBMSGS);
}



/*
 * This is actually the functionality required. 
 * Policy: NO WAITING, NO POLLING, return 0 on fail
 */

MFA acq200mu_get_free_ob(void)
/* return 0 on empty */
{
	u32 qp;
	MFA mfa = 0;

	if (downstream_is_set && !isEmpty(qp = *IOP321_OFTPR, *IOP321_OFHPR)){
		mfa = *qp2va(qp);
		LILSYNCI(qp);      //@@bogus		BIGSYNC;
		*IOP321_OFTPR = INCR(qp);
		mug.OF.n_ops++;
	}

	return mfa;
}

int acq200mu_post_ob(MFA mfa)
{
	u32 qp;

	if (downstream_is_set && !isFull(*IOP321_OPTPR, qp = *IOP321_OPHPR)){
		*qp2va(qp) = mfa;
		LILSYNCO(qp);
		*IOP321_OPHPR = INCR(qp);
		mug.OP.n_ops++;
		return mfa;
	}

	return 0;
}

MFA acq200mu_get_ib(void)
/* return next inbound message, or block until one arrives. return 0 on fail */
{
	u32 qp = 0;
	MFA mfa = 0;

	if (downstream_is_set){
		dbg(2, "01: downstream is set");
	}

	if (downstream_is_set && !isEmpty(qp = *IOP321_IPTPR, *IOP321_IPHPR)){
		dbg(2, "qp 0x%08x va %p", qp, qp2va(qp));
		LILSYNCI(qp);
		mfa = *qp2va(qp);
		*IOP321_IPTPR = INCR(qp);
		mug.IP.n_ops++;
	}

	dbg( 1, "mfa %08x qp %08x", mfa, qp );
	return mfa;
}

int acq200mu_return_free_ib(MFA mfa)
{
	u32 qp;

	if (downstream_is_set && !isFull(*IOP321_IFTPR, qp = *IOP321_IFHPR)){
		*qp2va(qp) = mfa;
		LILSYNCO(qp);            //@@todo bogus		BIGSYNC;
		*IOP321_IFHPR = INCR(qp);
		mug.IF.n_ops++;
		return 1;
	}else{
		return 0;
	}
}

static int acq200_mu_release(
	struct inode *inode, struct file *file)
{
	kfree(MUPD(file));
	return 0;
}



int acq200_mu_write_rma(RMA rma, char* buf, int len)
{
	return 0;
}

int acq200_mu_read_rma(RMA rma, char* buf, int len)
{
	return 0;
}


static ssize_t acq200_mailbox_inbound_read(
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	int rc;

	len = min(len, (size_t)4*sizeof(u32));

	if ((file->f_flags & O_NONBLOCK) == 0){
		int old_count = mug.mb_count;
		wait_event_interruptible(mug.mb_waitq, 
						old_count != mug.mb_count);
	}

	rc = copy_to_user(buf, (u32*)IOP321_IMR0, len);	
	return len;	
}

unsigned int acq200_mailbox_inbound_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	dbg(4, "01");
	poll_wait(file, &mug.mb_waitq, poll_table );
	if (mug.mb_interrupt_rx){
		/* @todo RACE */
		mug.mb_interrupt_rx = 0;
		dbg(3, "POLLIN | POLLRDNORM");
		return POLLIN | POLLRDNORM;
	}else{
		dbg(4, "99");
		return 0;
	}
}

static int acq200_mailbox_inbound_open(struct inode *inode, struct file *file)
{
	dbg(1,"");
#if (USE_MAILBOX_INTERRUPTS)
	*IOP321_IIMR &= ~IOP321_IIxR_IM0;
#endif
	mug.mailbox_inbound_active = 1;
	return 0;
}

static int acq200_mailbox_inbound_release(
	struct inode *inode, struct file *file)
{
	dbg(1,"");
	*IOP321_IIMR |= IOP321_IIxR_IM0;
	mug.mailbox_inbound_active = 0;
	return acq200_mu_release(inode, file);
}
/*
 * hook on read(0 write() interface.
 * Question: at what level does the app want to interface
 * App doesn't wanna know about MFA's etc.
 * on the other hand, forcing a buffer copy in/out will be inefficient
 * how about: read/write deals in cookies
 * and then the app can choos to either use the cookies to map
 * into a mmapped area or to use DMAC to fetch them.
 * This also implies that inbound cookies can migrate to the outbound Q.
 * is this good or bad?. Do we care? - yes, because an app can deadlock
 * the system by holding up or transferring too many cookies.
 *
 * Decision: always copy the MFS's, to the length indicated by the
 * Message. Read will return exactly one Message.
 * MFA's will be strictly recycled within the driver.
 * But cookies can contain other cookes which are indexes into data buffer
 * It's up to the application to decide how to arbitrate D/bufs.
 * So data exchange can lock up, but message exchange cannot.
 * SOUNDS GOOD TO ME

 * And - it gets better - then 
 * inbound is READ ONLY,
 * Outbound is WRITE only
 */
static ssize_t acq200_mu_inbound_read(
	struct file *file, char *buf, size_t len, loff_t *offset
	)
/*
 * Outputs one message.
 * block until message available
 * dump the MF into buf
 * replace MFA into inbound free Q.
 * currently this is NON-BLOCKING
 */
{
	MFA mfa = acq200mu_get_ib();

	dbg(1,"here with mfa %08x", mfa );

	if ( mfa == 0 ){
		if (file->f_flags & O_NONBLOCK){
			return -EAGAIN;
		}else{
			info( "workto do: block until interrupt\n" );
		}		
		return -EIO;
	}else{
		MessageHeader messageHeader;

		dbg(1,"dma_sync mfa %08x pa 0x%08x va %p", 
		    mfa, mfa2pa(mfa), mfa2va(mfa));

		memcpy_fromio(&messageHeader,mfa2va(mfa),MESSAGE_HEADER_SIZE);

		dbg(2, "MessageHeader id:0x%04x len:0x%04x type:0x%04x\n",
		    messageHeader.id, messageHeader.length,messageHeader.type);

		len = min( (int)len, (int)messageHeader.length);

		if (len == 0){
			return -EIO;
		}

		dbg(2, "memcpy_fromio( %p, %p, %d )", 
		    LBUF(file), mfa2va(mfa), len);

		memcpy_fromio(LBUF(file), mfa2va(mfa), len);

		dbg(2, "copy_to_user( %p, %p, %d)", buf, LBUF(file), len);

		if (copy_to_user(buf, LBUF(file), len)){
			return -EFAULT;
		}
		acq200mu_return_free_ib(mfa);
		return len;
	}
}

static int ipq_waiting(void)
{
	return downstream_is_set && !isEmpty(*IOP321_IPTPR, *IOP321_IPHPR);
}

unsigned int acq200_mu_inbound_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	dbg(4, "01");
	if (!ipq_waiting()){
		poll_wait(file, &mug.ip_waitq, poll_table );
	}
	if (ipq_waiting()){
		dbg(3, "POLLIN | POLLRDNORM");
		return POLLIN | POLLRDNORM;
	}else{
		dbg(4, "99");
		return 0;
	}
}


static MFA copy_user_to_mfa(
	struct file *file, const char *buf, size_t len
	)
/* get and MFA and copy user data to it. return the MFA */
{
	dma_addr_t handle;
	MFA mfa = acq200mu_get_free_ob();

	dbg(2,  "got mfa 0x%08x", mfa );

	dbg(2,  "copy_from_user( %p %p %d )", LBUF(file), buf, len);

	if (copy_from_user(LBUF(file), buf, len)){
		return -EFAULT;
	}

	handle = dma_map_single(mug.dev, LBUF(file), len, DMA_TO_DEVICE);

	dbg(2,  "memcpy_toio( v:%p [p:0x%08x] %p %d )", 
	      mfa2va(mfa), mfa2pa(mfa), buf, len);
	memcpy_toio(mfa2va(mfa), LBUF(file), len);

	dma_unmap_single(mug.dev, handle, len, DMA_TO_DEVICE);
	return mfa;
}
static ssize_t acq200_mu_outbound_write(
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
/*
 * Writes one message
 * block until free MFA available
 * copy the buffer to the MF
 * Post the MF
 */
{
	MFA mfa = copy_user_to_mfa(file, buf, len);
	dbg(2,  "now post mfa 0x%08x", mfa );
	acq200mu_post_ob(mfa);
	return len;
}

static struct mu_rma _rma_pool;

static struct mu_rma *getRma(void)
{
	return &_rma_pool;
}
static void putRma(struct mu_rma *rma)
{

}

#define DMA_CHANNEL (1|DMA_CHANNEL_NOBLOCK)

#define BUF_OFFSET(rma)       
#define REMOTE_ADDR(rma)      (rma->buffer_offset+mug.rma_base) /* WORKTODO! */
#define LOCAL_BUF_INDEX(rma)  (rma->buffer_offset/DATABUFSZ)
#define LOCAL_BUF_OFFSET(rma) (rma->buffer_offset%DATABUFSZ)



int acq200_mu_remote_read(
	struct file *file, char *buf, size_t len, loff_t *offset
	)
/* worktodo - potentially read  completed RMA's */
{
	return 0;
}


static void adump( const char* label, char* buf, int maxbuf )
{
	char aline[33];
	int iline, ibuf;

	aline[32] = '\0';

	for (ibuf = 0; ibuf != maxbuf; ){
		for (iline = 0; 
		     iline != 32 && ibuf != maxbuf; 
		     ++iline, ++ibuf){
			char achar = buf[ibuf];

			aline[iline] = achar>=' '&&achar<='}'? achar: '.';
		}
		
		info( "%10s: %s", label, aline );
	}
}


#define ADUMP(a,b,c) if(acq200_databuf_debug>2)adump(a,b,c)

#define INCOMING 1
#define OUTGOING 0
static int post_dmac_incoming_request(
	struct PCI_DMA_BUFFER *buf,
	unsigned offset,
	u32 remaddr,
	u32 bc
	)
{
	int rc;

	dbg( 1, "calling acq200_post_dmac_request" );
	rc = acq200_post_dmac_request(
		DMA_CHANNEL, buf->laddr, offset, remaddr, bc, INCOMING);

	dma_sync_single_for_cpu(mug.dev, buf->laddr, bc, DMA_FROM_DEVICE);

	ADUMP( FN, (char*)buf->va, 80 );
	return rc;
}

static int post_dmac_outgoing_request(
	struct PCI_DMA_BUFFER *buf,
	unsigned offset,
	u32 remaddr,
	u32 bc
	)
{
	
	ADUMP( FN, (char*)buf->va, 80 );				

	dma_sync_single_for_device(mug.dev, buf->laddr, bc, DMA_TO_DEVICE);

	dbg(2, "acq200_post_dmac_request(%d, 0x%08x, 0x%08x, 0x%08x, %d)",
	    DMA_CHANNEL, buf->laddr, offset, remaddr, bc);

	return acq200_post_dmac_request(
		DMA_CHANNEL, buf->laddr, offset, remaddr, bc, OUTGOING );
}



static int print_chain(struct DmaChannel* channel, char* buf)
{
	int ic;
	int len = 0;
	struct iop321_dma_desc* desc = channel->dmad[0];
	
	len += sprintf(buf+len, "[ ] %8s %8s %8s %8s %8s %8s\n",
		       "NDA", "PDA", "PUAD", "LAD", "BC", "DC");

	for (ic = 0; ic < channel->nchain; ++ic, ++desc){
		len += sprintf(buf+len, 
			       "[%d] %08x %08x %08x %08x %08x %08x %s\n",
			       ic, 
			       desc->NDA, desc->PDA, desc->PUAD,
			       desc->LAD, desc->BC, desc->DC,
			       channel->description[ic]);
	}	
	return len;
}
/** chaining DMA operation. Single work task does all, then no locking issues*/

#define DMA_ISYNC acq200_dma_getDmaChannelSync()[1].eoc



DEFINE_DMA_CHANNEL(dma_channel, 1);
static struct u32_ringbuffer DMADQ;
static int new_chain_ready;
static spinlock_t eot_lock = SPIN_LOCK_UNLOCKED;

static DECLARE_WAIT_QUEUE_HEAD(show_dma_waitq);
static enum DMA_START_MODE { D_OFF, D_FIRE, D_RELOAD } dma_start_mode;

static ssize_t show_dma_channel(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	enum DMA_START_MODE mode;

	if (wait_event_interruptible_timeout(
		    show_dma_waitq, (mode = dma_start_mode) != D_OFF, 2*HZ)){
		dma_start_mode = D_OFF;
		sprintf(buf, "DMA_START %s\n", mode==D_FIRE? "D_FIRE":
					mode==D_RELOAD? "D_RELOAD": "???");
		print_chain(&dma_channel, buf+strlen(buf));
		return strlen(buf);		
	}else{
		return -ETIMEDOUT;
	}
}

static DEVICE_ATTR(dma_chain, S_IRUGO, show_dma_channel, 0);

static int acq200mu_EOT_task(void *nothing)
{
	struct InterruptSync * isync = &DMA_ISYNC;
	struct DmaChannel *ch = &dma_channel;
	unsigned long flags;
	u32 stat;

	unsigned iter = 0;

/* this is going to be the top RT process */
	struct sched_param param = { .sched_priority = 12 };

	sched_setscheduler(current, SCHED_FIFO, &param);

	while(!kthread_should_stop()) {
		int new_chain_count;
		int was_interrupted;
		int timeout = wait_event_interruptible_timeout(
			isync->waitq, 
			isync->interrupted || new_chain_ready ||
				kthread_should_stop(),
			eot_to_ticks) == 0;
		int uptodate = 0;
		u32 dar = DMA_REG(dma_channel, DMA_DAR);

		dbg(timeout==0? 1: 3, "done wait [%8u] : %s %s %s %s",
		    ++iter,
		    isync->interrupted? "EOC": "",
		    new_chain_ready? "NEW": "",
		    timeout? "TIMEOUT": "",
		    kthread_should_stop()? "STOP": "");


		/** first, update dma_channel chain to reflect DMA state */

		spin_lock_irqsave(&eot_lock, flags);
		if ((was_interrupted = isync->interrupted) != 0){
			isync->interrupted = 0;
		}
		spin_unlock_irqrestore(&eot_lock, flags);

		UPSTATS(new_chain, new_chain_ready);
		UPSTATS(timeout, timeout);
		UPSTATS(interrupted, was_interrupted);


#define IMARK	dbg(2, "%d", __LINE__)

		if (was_interrupted || DMA_DONE(dma_channel, stat)){
			int ic = 0;
			for (; !uptodate && ic < dma_channel.nchain; ++ic){
				DMAD dmad = dma_channel.dmad[ic];

				if (dmad == 0){
					IMARK;
					continue;
				}else if (dar == dmad->pa){
					if (DMA_DONE(dma_channel, stat)){
						IMARK;
						uptodate = 1;
						UPSTATS(uptodate, uptodate);
					}else{
						IMARK;
						goto no_reset_chain;
					}
				}
				/* else != pa - DMAC has gone past, free it */

				/* here with completed dmad ... cleanup */
				if (dmad->clidat){
					dbg(2, "dmad %p POST %p", 
							dmad, dmad->clidat);

					acq200mu_post_ob((MFA)dmad->clidat);
					/* critical: */
					dmad->clidat = 0;
				}
				IMARK;
				dmad_free(dmad);
				dma_channel.dmad[ic] = 0;
			}
/*	reset_chain: */
			if (ic == dma_channel.nchain){
				dma_channel.nchain = 0;
			}
			dar = 0;			/* NO RELOAD */
		no_reset_chain:
			;
		}

		spin_lock(&eot_lock);
		if ((new_chain_count = new_chain_ready) != 0){
			new_chain_ready = 0;
		}
		spin_unlock(&eot_lock);
		/* now handle new requests */		

		if (new_chain_count && 
			MAXCHAIN - dma_channel.nchain > new_chain_count){
			DMAD dmad;

			while (u32rb_get(&DMADQ, (u32*)&dmad)){
				dbg(2, "dmad append %p M:%p", 
						dmad, dmad->clidat);
				dma_append_chain(ch, dmad, "data");
			}
			ch->dmad[ch->nchain-1]->DC |= IOP321_DCR_IE;
				
			if (dar == 0){
				IMARK;
				DMA_ARM(dma_channel);
				DMA_FIRE(dma_channel);
				IMARK;
				UPSTATS(fire, 1);
				dma_start_mode = D_FIRE;
			}else{
				IMARK;
				DMA_RELOAD(dma_channel);
				UPSTATS(reload, 1);
				dma_start_mode = D_RELOAD;
			}
			wake_up_interruptible(&show_dma_waitq);
		}else{
			IMARK;
			continue; /* no room, try later */
		}
	}

#undef IMARK

	err("requested to stop");
	return 0;
}


static int acq200mu_queue_dma_desc(struct iop321_dma_desc* dmad)
{
/* Q the dmad (MFA in clidat), and post when the DMA transfer is DONE.
 * problem :: which dma transfer?. 
 * answer, Q the DAR as well, then post all MFA' up to last DAR.
 * risk is, DAR has moved on ... in that case, just post the next one?.
 */

	return u32rb_put(&DMADQ, (u32)dmad);
}
	
	

static int queue_data_dmad_remote_write(struct mu_rma* rma)
{
	struct iop321_dma_desc* dmad = 0;
	struct PCI_DMA_BUFFER dma_buf;
	int dmad_count = 0;
	unsigned pci_addr = rma->bb_remote_pci_offset;

	int cursor;
	int chunk_len;
	int remainder;
	int _max_dma = max_dma;
	unsigned dc;

	dma_buf.va = bbva(rma->buffer_offset);
	if (MU_RMA_IS_PCI_REL(rma)){			
		pci_addr += mug.rma_base; /* rma offset */
	}

	if (MU_RMA_IS_HOSTBOUND(rma)){
		dma_buf.direction = DMA_TO_DEVICE;
		dc = DMA_DCR_PCI_MW;
	}else{
		dma_buf.direction = DMA_FROM_DEVICE;
		dc = DMA_DCR_PCI_MR;
	}

	dma_buf.laddr = dma_map_single(mug.dev, dma_buf.va, 
					rma->length, dma_buf.direction);

	if (_max_dma == 0){
		_max_dma = rma->length;
	}

	for (cursor = 0, remainder = rma->length; 
	     (chunk_len = min(_max_dma, remainder)) > 0;
	     remainder -= chunk_len, cursor += chunk_len) {

		dmad = dmad_alloc();

		dmad->NDA	= 0;
		dmad->PDA	= pci_addr+cursor;
		dmad->PUAD	= 0;
		dmad->LAD	= dma_buf.laddr+cursor;
		dmad->BC	= min(chunk_len, debug_dma_chunk_clip);
		dmad->DC	= dc;
		dmad->clidat    = 0;

		if (acq200mu_queue_dma_desc(dmad) == 0){
			err("mu_queue full");
			break;
		}
		++dmad_count;
	}
	dbg(1, "Q %p", dmad);
	return dmad_count;
}



static ssize_t acq200_mu_remote_write_chain(
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
{
	struct mu_rma *rma = getRma();
	struct iop321_dma_desc* dmad = 0;
	int rc = 0;
	int rlen = len;
	int dmad_count = 0;
#define BUF_PAYLOAD MU_RMA_PAYLOAD((struct mu_rma *)(buf))

	dbg(1, "01");

	while(rlen >= MU_RMA_SZ){
		COPY_FROM_USER(rma, buf, sizeof(u32));
		switch(MU_RMA_MAGIC(rma)){
		case MU_MAGIC_BB: {
			dbg(1, "MU_MAGIC_BB, copy %d", MU_RMA_RESIDUE);

			COPY_FROM_USER(MU_RMA_PAYLOAD(rma), 
				       BUF_PAYLOAD, MU_RMA_RESIDUE);

			dmad_count = queue_data_dmad_remote_write(rma); 

			buf += MU_RMA_SZ;	
			rlen -= MU_RMA_SZ;
			break;
		}
		case MU_MAGIC_OB: {
			/* pull down a MF, fill the message, store the MFA */
			MFA mfa = copy_user_to_mfa(file, BUF_PAYLOAD, rlen-4);

			dbg(1, "MU_MAGIC_OB: %p M:%08x", dmad, mfa);
			assert(mfa != 0); 

			if (dmad){
				dmad->clidat = (void*)mfa;
				if (dma1_IE){
					dmad->DC |= IOP321_DCR_IE;
				}
			}else{
				acq200mu_post_ob(mfa);
			}
			rlen = 0;
			break;
		}
		default:
			BUG();
		}
	}			    

	dbg(1, "dmad_count %d %s", dmad_count,
		    dmad_count? "WAKEUP": "done");

	if (dmad_count){
		spin_lock(&eot_lock);
		new_chain_ready = dmad_count;
		spin_unlock(&eot_lock);
		wake_up_interruptible(&DMA_ISYNC.waitq);
	}

	putRma(rma);

	if (rc == 0){
		*offset += len;
		return len;
	}else{
		return rc;
	}
#undef BUF_PAYLOAD	
}


static void acq200_mu_EOT_start(int start)
{
	static struct task_struct *the_worker;

	if (start){
		if (the_worker == 0){
			the_worker = kthread_run(
					acq200mu_EOT_task, NULL, "mu_EOT");
		}
	}else{
		if (the_worker != 0){
			kthread_stop(the_worker);
			the_worker = 0;
		}
	}
}
static ssize_t acq200_mu_remote_write(
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
/*
 * infuture, if non block set, could Q the RMA's, then read then back
 * when done. That will be cool!
 */
{
	struct mu_rma *rma;
	int rc = 0;

	dbg(2, "len %u", len);
	if (len < MU_RMA_SZ) return -E_MU_BAD_STRUCT;

	if (len > MU_RMA_SZ) {
		return acq200_mu_remote_write_chain(file, buf, len, offset);
	}

	rma = getRma();
	if (copy_from_user(rma, buf, MU_RMA_SZ)){
		return -EFAULT;
	}

	dbg(1, "rma %08x %08x %08x", 
	      rma->magic, rma->buffer_offset, rma->length);

	switch(MU_RMA_MAGIC(rma)){
	case MU_MAGIC: {
		int ibuf = LOCAL_BUF_INDEX(rma);

		dbg(1, "MU_MAGIC ibuf %d", ibuf );

		if ( ibuf < 0 || ibuf > NDATABUFS ){
			rc = -EFAULT;
		}else{
			struct PCI_DMA_BUFFER *dma_buf = &mug.databufs[ibuf];

			if (MU_RMA_IS_ACQBOUND(rma)){
				rc = post_dmac_incoming_request(
					dma_buf,
					LOCAL_BUF_OFFSET(rma),
					REMOTE_ADDR(rma),
					rma->length );			
			}else{
				rc = post_dmac_outgoing_request(
					dma_buf,
					LOCAL_BUF_OFFSET(rma),
					REMOTE_ADDR(rma),
					rma->length );
			}

			len = MU_RMA_SZ;
		}
	}
	case MU_MAGIC_BB: {
		struct PCI_DMA_BUFFER dma_buf;
		unsigned pci_addr = rma->bb_remote_pci_offset;

		if (MU_RMA_IS_PCI_REL(rma)){			
			pci_addr += mug.rma_base; /* rma relative offset */
		}
		
		dma_buf.va = bbva(rma->buffer_offset);
		dma_buf.direction = MU_RMA_IS_HOSTBOUND(rma)?
			DMA_TO_DEVICE: DMA_FROM_DEVICE;
		dma_buf.laddr = dma_map_single(
			mug.dev, dma_buf.va, rma->length, 
			dma_buf.direction);
		dma_buf.mapped = 1;


		dbg(1, "MU_MAGIC_BB:%s src 0x%08x dst 0x%08x len %d %s",
			    MU_RMA_IS_ACQBOUND(rma)? "IN": "OUT",
			    dma_buf.laddr, pci_addr,
			    rma->length,
			    MU_RMA_IS_PCI_REL(rma)? "REL": "ABS");

		if (MU_RMA_IS_ACQBOUND(rma)){
			rc = post_dmac_incoming_request(
				&dma_buf,
			        0,
				pci_addr,
				rma->length );			
		}else{
			rc = post_dmac_outgoing_request(
				&dma_buf,
				0,
				pci_addr,
				rma->length );
		}	
		len = MU_RMA_SZ;

		dma_unmap_single(mug.dev, dma_buf.laddr,
				 rma->length, dma_buf.direction);
		break;
	}	
	default:
		rc = -E_MU_BAD_MAGIC;
		break;
	}

	putRma(rma);

	if (rc == 0){
		*offset += len;
		return len;
	}else{
		return rc;
	}
}

static int acq200_mu_null_open (struct inode *inode, struct file *file)
{
	dbg(1,  "null" );
	return 0;
}

static int acq200_mu_rma_open (struct inode *inode, struct file *file)
{
	dbg(1,  "null" );
	acq200_mu_EOT_start(1);
	return 0;
}
	
static int mu_inbound_active;
static int acq200_mu_inbound_open(struct inode *inode, struct file *file)
{
	dbg(1,"");
#if (USE_INTERRUPTS)
	*IOP321_IISR |= IOP321_IIxR_IPQ;  /* clear any pended interrupt */
	if (downstream_is_set){	
		*IOP321_IIMR &= ~IOP321_IIxR_IPQ;
	}
#endif
	mu_inbound_active = 1;
	return 0;
}


static int acq200_mu_inbound_release(
	struct inode *inode, struct file *file)
{
	dbg(1,"");
	*IOP321_IIMR |= IOP321_IIxR_IPQ;
	mu_inbound_active = 0;
	return acq200_mu_release(inode, file);
}
static int acq200_mu_outbound_release(
	struct inode *inode, struct file *file)
{
	return acq200_mu_release(inode, file);
}
static int acq200_mu_rma_release(
	struct inode *inode, struct file *file)
{
	dbg(1, "" );
	acq200_mu_EOT_start(0);
	return acq200_mu_release(inode, file);
}


int getDirectionFromVma(struct vm_area_struct *vma)
{
	int direction;
/*
	switch(vma->vm_file->f_mode&(FMODE_READ|FMODE_WRITE)){
	case FMODE_READ|FMODE_WRITE:
		direction = DMA_BIDIRECTIONAL;
		break;
	case FMODE_READ:
		direction = DMA_FROM_DEVICE;
		break;
	case FMODE_WRITE:
		direction = DMA_TO_DEVICE;
		break;
	default:
		direction = DMA_NONE;
		break;
	}
*/
/* 
 * VM_READ: client wants to READ the MAPPING, DMA fills it FROM_DEVICE
 * VM_WRITE, client wants to WRITE the MAPPING, DMA write TO_DEVICE
 */
	switch(vma->vm_flags&(VM_READ|VM_WRITE)){
	case VM_READ|VM_WRITE:
		direction = DMA_BIDIRECTIONAL;
		break;
	case VM_READ:
		direction = DMA_FROM_DEVICE;
		break;
	case VM_WRITE:
		direction = DMA_TO_DEVICE;
		break;
	default:
		direction = DMA_NONE;
		break;		
	}
	return direction;
}


static inline void dbg_vm(struct vm_area_struct *vma)
{
	dbg( 1, "file %p mode %s %s", 
	      vma->vm_file, 
	      (vma->vm_flags&VM_WRITE)? "VM_WRITE": "",
	      (vma->vm_flags&VM_READ)?  "VM_READ": ""     );
}


static inline void dbg_vmap(char *label, int ibuf)
{
	dbg(1,"%10s %2d 0x%08x %p dir %d", 
	    label, ibuf,
	    mug.databufs[ibuf].laddr, 
	    mug.databufs[ibuf].va,
	    mug.databufs[ibuf].direction);
}
static void databuf_vma_open(struct vm_area_struct *vma)
/*
 * always assume mapping all the databufs
 */
{
	int direction = getDirectionFromVma(vma);
	int ibuf;

	dbg_vm(vma);
	
/*
dma_map_single(struct device *dev, void *cpu_addr, size_t size,
	       enum dma_data_direction dir);
*/	      
	for (ibuf = 0; ibuf != NDATABUFS; ++ibuf){
		mug.databufs[ibuf].laddr = dma_map_single(mug.dev, 
			       mug.databufs[ibuf].va,
			       DATABUFSZ,
			       mug.databufs[ibuf].direction = direction );
		dbg_vmap("dma_map_single()", ibuf);
	}
}
static void databuf_vma_close(struct vm_area_struct *vma)
{
	int ibuf;

	/* no need to ANYTHING any more: Thanks Linux 2.6 */
	dbg_vm(vma);

	for (ibuf = 0; ibuf != NDATABUFS; ++ibuf){
		dbg(1,"unmap %2d 0x%08x %p dir %d", 
		    ibuf,
		    mug.databufs[ibuf].laddr, 
		    mug.databufs[ibuf].va,
		    mug.databufs[ibuf].direction);
		dbg_vmap("dma_unmap_single()", ibuf);
	}	      

	/* this works for acq32 - maybe work here too ? */
	for( ibuf = 0; ibuf != NDATABUFS; ++ibuf ){
            struct page* page = virt_to_page(mug.databufs[ibuf].va);
            int count;

            if ( (count = page_count( page )) != 1 ){
		    init_page_count(page);
/** WORKTODO: pgmwashere for 2.6.18 port replaces:
		    set_page_count( page, 1 );
*/
		    dbg(1," BUFFER:%p %4x count %d fixed to %d\n", 
			    mug.databufs[ibuf].va,
			    page-mem_map,
			    count, 
			    page_count( page ) 
		    );
            }
            dbg( 2, "page %p", page );
	}
}

static struct page* databuf_vma_nopage(
	struct vm_area_struct* vma, unsigned long address, int *type)
{
	unsigned long offset = address - vma->vm_start;
	int ibuf = offset/DATABUFSZ;


	if (ibuf >= 0 && ibuf < NDATABUFS){
		unsigned boffset = offset - ibuf*DATABUFSZ;
		void* va = (char*)mug.databufs[ibuf].va + boffset;
		struct page *page = virt_to_page(va);

#ifndef PGMCOMOUT		
		if (boffset<PAGE_SIZE){
			get_page(page); /* only inc first page in buf count */
		}
#else
		get_page(page);
#endif
		DBDBG(3, "addr %08lx offset %08lx va %p return page %p", 
		      address, offset, va, page );
	        return page;
	}else{
		return 0;
	}
}

struct vm_operations_struct databuf_vm_ops = {
	.open = databuf_vma_open,
	.close = databuf_vma_close,
	.nopage = databuf_vma_nopage,
};


static int acq200_mu_mmap(
	struct file *file, struct vm_area_struct *vma )
/*
 * map the data buffers into contiguous user space
 */
{
	dbg(1,"hello");
	vma->vm_flags |= VM_RESERVED;
	vma->vm_ops = &databuf_vm_ops;
	vma->vm_file = file;
	vma->vm_ops->open(vma);

	return 0;
}

static int acq200_mu_host_window_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int acq200_mu_host_window_mmap(
	struct file *file, struct vm_area_struct *vma)
{
	unsigned paddr;
	unsigned len;

	if (machine_has_bridge()){
		/* assumes direct mapping - NB this may change */
		paddr = ACQ200_PCIMEM + mug.host_window_offset;;   
		len = ACQ200_PCIMEM_SIZE - mug.host_window_offset;

		info("acq200: paddr 0x%08x", paddr);
	}else{
		paddr = ACQ100_PCIMEM_P + mug.host_window_offset;
		len = 0x04000000 - mug.host_window_offset;

		info("acq100: paddr 0x%08x", paddr);
	}	

	if ( vma->vm_end - vma->vm_start < len ){
		len = vma->vm_end - vma->vm_start;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_RESERVED | VM_IO;

	if (io_remap_pfn_range(vma, vma->vm_start, __phys_to_pfn(paddr), 
		       				len, vma->vm_page_prot)){
		err( "remap_pfn_range() failed" );
		return -EAGAIN;
	}else{
		return 0;
	}
}

int acq200_mu_host_window_release (struct inode *inode, struct file *file)
{
	return 0;
}

static int acq200_mu_open (struct inode *inode, struct file *file)
{
	static struct file_operations mailbox_fops = {
		.open = acq200_mailbox_inbound_open,
		.release = acq200_mailbox_inbound_release,
		.read = acq200_mailbox_inbound_read,
		.poll = acq200_mailbox_inbound_poll,
	};

	static struct file_operations inbound_fops = {
		.open = acq200_mu_inbound_open,
		.release = acq200_mu_inbound_release,
		.read = acq200_mu_inbound_read,
		.poll = acq200_mu_inbound_poll,
	};

	static struct file_operations outbound_fops = {
		.open = acq200_mu_null_open,
		.write = acq200_mu_outbound_write,
		.mmap = acq200_mu_mmap,
		.release = acq200_mu_outbound_release
	};

	static struct file_operations rma_fops = {
		.open = acq200_mu_rma_open,
		.mmap = acq200_mu_mmap,
		.write = acq200_mu_remote_write,
		.read = acq200_mu_remote_read,
		.release = acq200_mu_rma_release
	};
	static struct file_operations host_window_fops = {
		.open = acq200_mu_host_window_open,
		.mmap = acq200_mu_host_window_mmap,
		.release = acq200_mu_host_window_release
	};
        int iminor = MINOR(file->f_dentry->d_inode->i_rdev);

	switch( iminor ){
	case ACQ200_MU_INBOUND:
		file->f_op = &inbound_fops;
		break;
	case ACQ200_MU_OUTBOUND:
		file->f_op = &outbound_fops;
		break;
	case ACQ200_MU_RMA:
		file->f_op = &rma_fops;
		break;
	case ACQ200_MU_MAILBOX:
		file->f_op = &mailbox_fops;
		break;
	case ACQ200_MU_PCIM:
		file->f_op = &host_window_fops;
		break;
	default:
		return -ENODEV;
	}
	file->private_data = 
		kmalloc(sizeof(struct MU_PATH_DESCRIPTOR), GFP_KERNEL);

	return file->f_op->open( inode, file );
}

static int __devinit
run_mu_mknod_helper( int major )
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};
	static char args[][5] = {
		{},
		{},
		{ '0'+ACQ200_MU_INBOUND, },
		{ '0'+ACQ200_MU_OUTBOUND, },
		{ '0'+ACQ200_MU_RMA, },
		{ '0'+ACQ200_MU_MAILBOX, },
		{}
	};
#define ARGLEN (sizeof(args)/sizeof(args[0]))	
	char *argv[ARGLEN];
	int rc;

        argv[0] = "/sbin/acq200_mu_helper";
	sprintf( args[1], "%d", major );
	
	for (rc = 1; rc < ARGLEN; ++rc){
		argv[rc] = args[rc];
	}

	info("call_usermodehelper %s %d args\n", argv[0], ARGLEN);

	rc = call_usermodehelper(argv [0], argv, envp, 0);

	if ( rc != 0 ) err( "call done returned %d", rc );

	return 0;
}

static void free_databufs(void)
{
	int ibuf;

	kfree(mug.databufs);
	
	for (ibuf = 0; ibuf != NDATABUFS; ++ibuf){
		u32* abuf = mug.databufs[ibuf].va;

		if (abuf == 0){
			break;
		}else{
			mug.databufs[ibuf].va = 0;
			free_pages((unsigned)abuf, PO(HBBLOCK));
		}
	}
}


static void debug_fill_databuf(int ibuf)
{
	int iw;

	for (iw = 0; iw != DATABUFLN; ++iw){
		mug.databufs[ibuf].va[iw] = (ibuf<<24)+iw;
	}
}


static int alloc_databufs(void)
{
	int ibuf;

#if 0
	if (HBLEN <= _HBLEN26){
		if (HBBLOCK > _HBBLOCK26){
			HBBLOCK = _HBBLOCK26;
		}
	}
#endif
	mug.databufs = kzalloc(DATABUFS_SZ, GFP_KERNEL);

	for (ibuf = 0; ibuf != NDATABUFS; ++ibuf){
		u32* abuf = (u32*)__get_free_pages(GFP_KERNEL, PO(HBBLOCK));

		if (abuf==0){
			err( " allocation failed at %d", ibuf);
			free_databufs();
			return -ENOMEM;
		}else{
			DBDBG(1," mug.databufs[%2d] = %p", ibuf, abuf );
			mug.databufs[ibuf].va = abuf;

			if (acq200_databuf_debug > 1){
				debug_fill_databuf(ibuf);
			}
		}
	}
	info("allocated %d bufs each 0x%08x", ibuf, HBBLOCK);

	return 0;
}

static ssize_t show_globs (
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
#define DEFPR(sym) \
	len += sprintf(buf+len, "%-40s 0x%08lx\n", #sym, (long)sym)
	DEFPR(high_memory);
	DEFPR(VMALLOC_START);
	DEFPR(VMALLOC_END);
	DEFPR(mug.host_base_pa);
	DEFPR(mug.host_base_va);
	DEFPR(mug.host_window_offset);
	DEFPR(mug.host_base_len);
	DEFPR(mug.rma_base);
	return len;
#undef DEFPR
}

static DEVICE_ATTR(globs, S_IRUGO, show_globs, 0);

static ssize_t show_downstream_window(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "0x%08x\n", *IOP321_OMWTVR0);
}

static ssize_t set_downstream_window(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
        unsigned offset;
	unsigned dw_mask = 0;

        if (downstream_is_set == 0 && 
	    sscanf(buf, "0x%x 0x%08x %x", &offset, &HBLEN, &dw_mask) >= 1){
		HBPHYS = offset;
		if (machine_has_bridge()){
			/* mug.rma_base is already set */
			mug.host_window_offset = offset&dw_mask;
		}else{
			*IOP321_OMWTVR0 = offset;
		/*
		 * Outbound translate window sits on 64MB boundary
                 * External memory def doesn't have this restriction,
		 * so we need to record the offset
		 * DMAC uses DIRECT addressing to rma_base _is_ offset
                 */
			mug.host_window_offset = offset - *IOP321_OMWTVR0;
			mug.rma_base = offset;
		}
#if (MU_LAZY_ALLOC != 0)
		alloc_databufs();
		init_mu_queues();
#endif
#if (USE_INTERRUPTS)
		/* clear any pended interrupt */
		*IOP321_IISR |= IOP321_IIxR_IPQ; 
		if (request_irq(IRQ_IOP321_MU_IPQ, 
	                        acq200_mu_ipq_isr, 0, 
				"acq200-mu-ipq", &mug) != 0){
			err( "request_irq() failed" );
			return -ENODEV;
		}
		if (mu_inbound_active){
			*IOP321_IIMR &= ~IOP321_IIxR_IPQ;
		}
#endif
		dbg(2, "downstream_is_set = 1");
		downstream_is_set = 1;
        }       

	return strlen(buf);
}


static DEVICE_ATTR(downstream_window,
		   S_IRUGO|S_IWUGO,
		   show_downstream_window,
		   set_downstream_window);

static void mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_dma_chain);
	DEVICE_CREATE_FILE(dev, &dev_attr_globs);
	DEVICE_CREATE_FILE(dev, &dev_attr_downstream_window);
}


static int acq200_mu_probe(struct device * dev)
{
	static struct file_operations mu_fops = {
		open: acq200_mu_open,
		release: acq200_mu_release
	};
	int rc;

	info( "" );

	mug.dev = dev;
	init_waitqueue_head(&mug.ip_waitq);
	init_waitqueue_head(&mug.mb_waitq);

	acq200_get_mumem_resource(&mug.mumem);
	mug.mumem_dmad = dma_map_single(dev, mumem_va(), mumem_len(), 
					DMA_BIDIRECTIONAL);

	dbg(1, "dmad 0x%08x", mug.mumem_dmad );

	acq200_get_bigbuf_resource(&mug.bigbuf);
#if (MU_LAZY_ALLOC == 0)
		alloc_databufs();
		init_mu_queues();
#endif
	mk_dev_sysfs(dev);

/*
	rc = request_mem_region(mumem_resource.start, len, "acq200mu_queues");
	dbg(1,  "request_mem_region returned %d\n", rc );
*/
	mk_sysfs(dev);

	*IOP321_IIMR = IOP321_IIxR_MASK;
	if (request_irq(IRQ_IOP321_MESSAGING, 
                         acq200_mb_isr, 0, "acq200-mu mb", &mug) != 0){
		err( "request_irq() failed" );
		return -ENODEV;
	}

	rc = register_chrdev( acq200_mu_major = 0, 
			      "acq200-mu", 
			      &mu_fops );

/* 
 * WORKTODO IMPROVE THIS PLEASE! - ioremap not needed??
 */
	if (machine_has_bridge()){
		mug.host_base_va = 
			ioremap(mug.host_base_pa, mug.host_base_len);
                /**
		 * base is address of bridge outbound window.
                 * it should NOT be rocket science to hook this up
		 */
		mug.rma_base = ACQ216_BRIDGE_WINDOW_BASE;
	}else{
		mug.host_base_pa = ACQ100_PCIMEM_P;
		mug.host_base_va = (void*)ACQ100_PCIMEM_START;
		mug.host_base_len = ACQ100_PCIMEM_END - ACQ100_PCIMEM_START;
		mug.rma_base = 0xdeadbeef;  /** set_downstream_window() */
	}

	u32rb_init(&DMADQ, 16);

	info(": va:%p pa:0x%08x len:%08x", 
	     mug.host_base_va, mug.host_base_pa, mug.host_base_len);

	if ( rc > 0 ){
		run_mu_mknod_helper(acq200_mu_major = rc);
	}else{
		err( "register_chrdev failed %d\n", rc );
		return rc;
	}
	return 0;
}



static int acq200_mu_remove(struct device * dev)
{
	free_databufs();
	dma_unmap_single(dev, mug.mumem_dmad, mumem_len(), DMA_BIDIRECTIONAL);
	unregister_chrdev(acq200_mu_major, "acq200-mu");
	free_irq(IRQ_IOP321_MESSAGING, &mug);
	free_irq(IRQ_IOP321_MU_IPQ, &mug);
	rm_sysfs(dev);
	return 0;
}





static struct device_driver mu_device_driver = {
	.name		= "acq200mu",
	.bus		= &platform_bus_type,
	.probe          = acq200_mu_probe,
	.remove         = acq200_mu_remove,
};


static void mu_release(struct device *dev)
{
	printk("mu_release\n");
}

static struct platform_device mu_device = {
	.name		= "acq200mu",
	.id		= 0,
	.dev = {
		.release   = mu_release,
		.kobj.name	= "acq200mu",
	},
};


static int __init acq200_mu_init( void )
{
	int rc = driver_register(&mu_device_driver);
	
	if (rc){
		return rc;
	}

	return platform_device_register(&mu_device);
}


static void __exit
acq200_mu_exit_module(void)
// Remove DRIVER resources on module exit
{
	platform_device_unregister(&mu_device);	
	driver_unregister(&mu_device_driver);
}

module_init(acq200_mu_init);
module_exit(acq200_mu_exit_module);

EXPORT_SYMBOL_GPL(mfa2va);
EXPORT_SYMBOL_GPL(mfa2pa);
EXPORT_SYMBOL_GPL(rma2va);
EXPORT_SYMBOL_GPL(mfa2pci);
EXPORT_SYMBOL_GPL(rma2pci);
EXPORT_SYMBOL_GPL(acq200mu_get_free_ob);
EXPORT_SYMBOL_GPL(acq200mu_post_ob);
EXPORT_SYMBOL_GPL(acq200mu_get_ib);
EXPORT_SYMBOL_GPL(acq200mu_return_free_ib);
EXPORT_SYMBOL_GPL(acq200mu_get_hb_len);
EXPORT_SYMBOL_GPL(acq200mu_get_hb_phys);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 Message Unit");

