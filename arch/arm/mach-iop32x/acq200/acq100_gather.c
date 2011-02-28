/* ------------------------------------------------------------------------- */
/* acq100_gather.c gathers data from self and slave devices                  */
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


/** @file acq100_gather.c
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

#include <linux/proc_fs.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include "acq200.h"
#include "acq200-common.h"

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/iop321-dma.h>
#include "acq200-dmac.h"
#include "acq200_debug.h"

#include "acq200_hostdrv.h"

/* from acq196-fifo-t.c */
extern struct pci_mapping acq196t_t2;
#define TSP(tx, block) ((tx).pa + (block)*sizeof(u32))

#define MAXDEV	16		/* including self */

#ifdef __ACQ200_INLINE_DMA_H__
#error FILE ALREADY INCLUDED
#endif
#define MAXCHAIN	(MAXDEV+4)
#include "acq200-inline-dma.h"

#define EXCLUDE_PBC_INLINES
#include "prebuiltChainUtils.h"

#include "acq100_rtm_t.h"

int caplen = (3*96*sizeof(short));	/* actual capture length to copy */
module_param(caplen, int, 0644);
MODULE_PARM_DESC(caplen, "actual capture length bytes to copy");

int dma_triggers;
module_param(dma_triggers, int, 0644);
MODULE_PARM_DESC(dma_triggers, "number of dma so far");

int dma_yield;
module_param(dma_yield, int, 0644);
MODULE_PARM_DESC(dma_yield, "dma busy count");

int dma_maxticks;
module_param(dma_maxticks, int, 0644);
MODULE_PARM_DESC(dma_maxticks, "max DMA wait (20nsec tick)");

int target_rtm_t = 1;
module_param(target_rtm_t, int, 0444);
MODULE_PARM_DESC(target_rtm_t, "1: send data to RTM-T (else lbuf)");

int init_ramp = 0;
module_param(init_ramp, int, 0644);
MODULE_PARM_DESC(init_ramp, "!=0 - initial data ramp (shorts) for id");

/** Globals .. keep to a minimum! */
char acq100_gather_driver_name[] = "acq100_gather";
char acq100_gather_driver_string[] = "D-TACQ gather driver";
char acq100_gather_driver_version[] = "B1006";
char acq100_gather_copyright[] = "Copyright (c) 2011 D-TACQ Solutions Ltd";

#define PBI_MAX	0x400		/* max in-order transfer on PBI */

extern int control_block;
extern int control_numblocks;

extern struct proc_dir_entry* proc_acq200;

struct Region {
	int len;
	unsigned pa;
	const char* name;
	unsigned dc;
};

struct Globals {
	struct Region sources[MAXDEV];
	struct Region destLocal;
	struct Region destRTM;
	struct Region* dest;
	struct pci_mapping padding;
	struct pci_mapping init_ramp;
	int is;

	struct DmaChannel* chains;	/* control_numblocks chains */
}
	GL;

static void addSource(unsigned pa, int len, const char* name, unsigned dc)
{
	GL.sources[GL.is].pa = pa;
	GL.sources[GL.is].len = len;
	GL.sources[GL.is].name = name;
	GL.sources[GL.is].dc = dc;
	++GL.is;
}
static void addDestLocal(unsigned pa, int len, const char* name, unsigned dc)
{
	GL.destLocal.pa = pa;
	GL.destLocal.len = len;
	GL.destLocal.name = name;
	GL.destLocal.dc = dc;
	GL.dest = &GL.destLocal;
}
static void addDestRTM(unsigned pa, int len, const char* name, unsigned dc)
{
	GL.destRTM.pa = pa;
	GL.destRTM.len = len;
	GL.destRTM.name = name;
	GL.destRTM.dc = dc;
	GL.dest = &GL.destRTM;
}

void get_regions(void)
{
	int idev;

	info("self: %08x block:%d numblocks:%d",
	     *IOP321_IATVR2, control_block, control_numblocks);

	addSource(*IOP321_IATVR2, control_block * control_numblocks, 
		  "local", DMA_DCR_MEM2MEM);

	
	for (idev = 0; ; ++idev){
		struct Acq200Device *device = acq200_devices[idev];

		if (device == 0){
			break;
		}
		if (device->ram.pa == 0){
			continue;	   /* non participating card */
		}
		addSource(device->ram.pa, device->ram.len, 
			  device->ram.name, DMA_DCR_PCI_MR);
		info("device %s pa: 0x%08lx len: %d",
		     device->ram.name, device->ram.pa, device->ram.len);

	}		

	if (target_rtm_t > 0){
		addDestRTM(RTMT_IOPFIFO_PA, 0x400, "rtm-t", DMA_DCR_MEM2MEM);
	}
	if (target_rtm_t == 0 || target_rtm_t == 2){
		addDestLocal(*IOP321_IATVR2 + control_block * control_numblocks,
			GL.is * control_block * control_numblocks, 
			"local", DMA_DCR_MEM2MEM);
	}	
}

u32 dinc(u32 *dest_pa, int nbytes)
/* target_rtm_t? => FIFO, dont incr, else debug, memory, want addr incr */
{
	u32 pa = *dest_pa;
	if (target_rtm_t == 0 || target_rtm_t == 2){
		*dest_pa += nbytes;	/* *dest_pa NOT a point, OK to add */
	}
	return pa;	
}
void append_padding_limit(
	struct DmaChannel* dmac, int cblock, int nbytes, u32 *dest_pa)
{
/** target_rtm_t==0 case isn't quite right, but it's not important */
	struct iop321_dma_desc* dmad = acq200_dmad_alloc();

	dmad->MM_SRC = GL.padding.pa;
	dmad->PUAD = 0;
	dmad->MM_DST =	dinc(dest_pa, nbytes);
	dmad->BC = nbytes;
	dmad->DC = DMA_DCR_MEM2MEM;
	dma_append_chain(dmac, dmad, "pad");
}

void make_ramp(struct pci_mapping* map, int len, u16 key)
{
	int iramp;
	short *the_ramp;

	map->len = len;
	map->va = kmalloc(map->len, GFP_KERNEL);
	map->pa = dma_map_single(0, map->va, map->len, DMA_TO_DEVICE);

	the_ramp = (short*)map->va;

	for (iramp = 0; iramp != len/2; ++iramp){
		the_ramp[iramp] = key|iramp;		
	}

	dma_sync_single_for_device(0, map->pa, map->len, DMA_TO_DEVICE);

}
int insert_ramp(struct DmaChannel* dmac, u32* dest_pa)
{
	struct iop321_dma_desc* dmad = acq200_dmad_alloc();

	make_ramp(&GL.init_ramp, init_ramp*sizeof(short), 0x1000);

	dmad->MM_SRC = GL.init_ramp.pa;
	dmad->PUAD = 0;
	dmad->MM_DST = dinc(dest_pa, GL.init_ramp.len);
	dmad->BC = GL.init_ramp.len;
	dmad->DC = DMA_DCR_MEM2MEM;
	dma_append_chain(dmac, dmad, "ramp");	

	return GL.init_ramp.len;
}


void append_padding(
	struct DmaChannel* dmac, int cblock, int nbytes, u32 *dest_pa)
{
	int nb = 0;

	make_ramp(&GL.padding, min(nbytes, PBI_MAX), 0x9000);

	while (nb < nbytes){
		int maxdma = min(nbytes-nb, PBI_MAX);
		append_padding_limit(dmac, cblock, maxdma, dest_pa);
		nb += maxdma;
	} 
}


void append_timestamper(struct DmaChannel* dmac, int cblock)
{
	struct iop321_dma_desc* dmad = acq200_dmad_alloc();
	
	dmad->MM_SRC = IOP321_REG_PA(IOP321_GTSR);
	dmad->PUAD = 0;
	dmad->MM_DST = TSP(acq196t_t2, cblock);
	dmad->BC = sizeof(u32);
	dmad->DC = DMA_DCR_MEM2MEM;
	dma_append_chain(dmac, dmad, "t2");
}

int next_power2(int nbytes)
{
	int cursor = 0x1;

	for ( ; cursor < nbytes; cursor <<= 1){
		;
	}
	return cursor;
}

void append_mem2rtm_limit(
	struct DmaChannel* dmac, int cblock, int offset, int nbytes)
{
	struct iop321_dma_desc* dmad = acq200_dmad_alloc();
	dmad->MM_SRC = GL.destLocal.pa + offset;
	dmad->PUAD = 0;
	dmad->MM_DST = GL.destRTM.pa;
	dmad->BC = nbytes;
	dmad->DC = DMA_DCR_MEM2MEM;
	dma_append_chain(dmac, dmad, "mem2rtm");	
}
void append_mem2rtm(struct DmaChannel* dmac, int cblock, int len){
	int offset = 0;
		
	while(offset < len){
		int maxdma = min(len - offset, PBI_MAX);

		append_mem2rtm_limit(dmac, cblock, offset, maxdma);
		offset += maxdma;	
	}
}


void make_chain(struct DmaChannel* dmac, int cblock, u32 *dest_pa)
{
	int isrc;
	int nbytes = 0;

	dmac->regs = IOP321_DMA1_CCR;
	dmac->id = 1;

	if (init_ramp > 0){
		nbytes += insert_ramp(dmac, dest_pa);
	}
	for (isrc = 0; isrc < GL.is; ++isrc){
		struct iop321_dma_desc* dmad = acq200_dmad_alloc();
		struct Region *region = &GL.sources[isrc];
		dmad->NDA = 0;
		dmad->PDA = region->pa + cblock*control_block;
		dmad->PUAD = 0;
		dmad->LAD = dinc(dest_pa, caplen);
		dmad->BC = caplen;
		dmad->DC = region->dc;
		dma_append_chain(dmac, dmad, region->name);
		nbytes += caplen;
	}

	append_padding(dmac, cblock, next_power2(nbytes) - nbytes, dest_pa);

	if (target_rtm_t == 2){
		append_mem2rtm(dmac, cblock, next_power2(nbytes));
	}
	append_timestamper(dmac, cblock);
}

void make_chains(void)
{
	int chn;
	u32 dest_pa = GL.dest->pa;

	GL.chains = kzalloc(
		control_numblocks*sizeof(struct DmaChannel), GFP_KERNEL);

	for (chn = 0; chn < control_numblocks; ++chn){
		make_chain(&GL.chains[chn], chn, &dest_pa);
	}
	info("sizeof of block: %d, total memory size: %d",
	     (dest_pa - GL.dest->pa)/control_numblocks, dest_pa - GL.dest->pa);
}

void free_chains(void)
{
	int chn;

	for (chn = 0; chn < control_numblocks; ++chn){
		struct DmaChannel* dmac = &GL.chains[chn];
		dma_cleanup(dmac);
	}

	kfree(GL.chains);
	GL.chains = 0;	
}

static int print_chain(struct DmaChannel* channel, char* buf)
{
	int ic;
	int len = 0;
	struct iop321_dma_desc* desc = channel->dmad[0];
	
	len += sprintf(buf+len, "[  ] %8s %8s %8s %8s %8s %8s\n",
		       "NDA", "PDA", "PUAD", "LAD", "BC", "DC");

	for (ic = 0; ic < channel->nchain; ++ic, ++desc){
		len += sprintf(buf+len, 
			       "[%2d] %08x %08x %08x %08x %08x %08x %s\n",
			       ic, 
			       desc->NDA, desc->PDA, desc->PUAD,
			       desc->LAD, desc->BC, desc->DC,
			       channel->description[ic]);
	}	
	return len;
}

int proc_dump_chain(char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	if (eof){
		*eof = 1;
	}
	return print_chain(&GL.chains[(int)data], buf);
}

void create_proc_entries(void)
{
	struct proc_dir_entry *gather = proc_mkdir("gather", proc_acq200);
	int ic;

	for (ic = 0; ic < control_numblocks; ++ic){
		char name[32];
		sprintf(name, "%02d", ic);
		create_proc_read_entry(
			name, S_IRUGO, gather, proc_dump_chain,	(void*)ic);
	}
}

#define IN_RANGE(xx, ll, rr) ((xx)>=(ll)&&(xx)<=(rr))

void fire_chain(int iblock)
{
	u32 stat;
	u32 gtsr1 = *IOP321_GTSR;
	u32 gtsr2;

	while (!DMA_DONE( GL.chains[iblock], stat)){
		++dma_yield;
		yield();
	}
	gtsr2 = *IOP321_GTSR;
	DMA_ARM(GL.chains[iblock]);
	DMA_FIRE(GL.chains[iblock]);
	++dma_triggers;

	if (likely(gtsr2 > gtsr1)){
		u32 delta = gtsr2 - gtsr1;
		if (delta > dma_maxticks){
			dma_maxticks = delta;
		}
	}
}

void prebuiltChainHandler(struct PrebuiltChain* pbc)
{
	if (!IN_RANGE(pbc->iblock, 0, control_numblocks-1)){
		err("pbc not in range .. discard");
	}else{
		fire_chain(pbc->iblock);
	}
}

void hookup(void)
{
	acq200_registerPrebuiltClient(prebuiltChainHandler);
}

static ssize_t set_fire(
	struct device *dev,
	struct device_attribute *attr,
	const char* buf,
	size_t count)
{
	int ichain;
	if (sscanf(buf, "%d", &ichain) == 1 &&
	    ichain >= 0 && ichain < control_numblocks){
		fire_chain(ichain);
	}
	return count;
}

static DEVICE_ATTR(fire, S_IWUGO, 0, set_fire);

static int mk_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_fire);
	return 0;
}
int acq100_gather_probe(struct device *dev)
{
	info("%s\n%s\n%s\n%s", 
	     acq100_gather_driver_name, acq100_gather_driver_string,
	     acq100_gather_driver_version, acq100_gather_copyright);

	get_regions();
	make_chains();
	hookup();

	create_proc_entries();
	mk_sysfs(dev);
	return 0;
}

int acq100_gather_remove(struct device *dev)
{       
	if (GL.padding.va) kfree(GL.padding.va);	
	if (GL.init_ramp.va) kfree(GL.init_ramp.va);
	free_chains();
	return 0;
}

static void acq100_gather_dev_release(struct device * dev)
{
	info("not used");
}


static struct device_driver acq100_gather_driver = {
	.name     = "acq100_gather",
	.probe    = acq100_gather_probe,
	.remove   = acq100_gather_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq100_gather_device = {
	.name = "acq100_gather",
	.id   = 0,
	.dev = {
		.release    = acq100_gather_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq100_gather_init_module( void )
{
	int rc = driver_register(&acq100_gather_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acq100_gather_device);
}


static void __exit
acq100_gather_exit_module(void)
{
	info("");
	platform_device_unregister(&acq100_gather_device);
	driver_unregister(&acq100_gather_driver);
}


module_init(acq100_gather_init_module);
module_exit(acq100_gather_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("acq100_gather mechanism");
