/* ------------------------------------------------------------------------- */
/* acqfiber_push.c test app pushes data on acq-fiber-hba	             */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2012 Peter Milne, D-TACQ Solutions Ltd
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


/** @file acqfiber_push.c
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
#define MAXCHAIN	(MAXDEV+16)
#include "acq200-inline-dma.h"

#define EXCLUDE_PBC_INLINES
#include "prebuiltChainUtils.h"

#include "acq100_rtm_t.h"

int dma_triggers;
module_param(dma_triggers, int, 0644);
MODULE_PARM_DESC(dma_triggers, "number of dma so far");

int dma_yield;
module_param(dma_yield, int, 0644);
MODULE_PARM_DESC(dma_yield, "dma busy count");

int dma_maxticks;
module_param(dma_maxticks, int, 0644);
MODULE_PARM_DESC(dma_maxticks, "max DMA wait (20nsec tick)");

int target_rtm_t = 2;
module_param(target_rtm_t, int, 0444);
MODULE_PARM_DESC(target_rtm_t, 
	">= 1: send data to RTM-T 2: use DRAM (else lbuf)");


/** Globals .. keep to a minimum! */
char acqfiber_push_driver_name[] = "acqfiber_push";
char acqfiber_push_driver_string[] = "D-TACQ gather driver";
char acqfiber_push_driver_version[] = "B1009";
char acqfiber_push_copyright[] = "Copyright (c) 2011 D-TACQ Solutions Ltd";

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
	struct Region destLocal;
	struct Region destRTM;
	struct Region* dest;

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


int next_power2(int nbytes)
{
	int cursor = 0x1;

	for ( ; cursor < nbytes; cursor <<= 1){
		;
	}
	return cursor;
}

void append_mem2rtm_limit(
	struct DmaChannel* dmac, int offset, int nbytes)
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
	int block_off = dbg_use_same_buffer? 0: cblock*next_power2(len);
		
	while(offset < len){
		int maxdma = min(len - offset, PBI_MAX);

		append_mem2rtm_limit(dmac, block_off+offset, maxdma);
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

	GL.chains = kzalloc(NCHAINS*sizeof(struct DmaChannel), GFP_KERNEL);

	for (chn = 0; chn < NCHAINS; ++chn){
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
		       "NDA", "PDA/MMSRC", "PUAD", "LAD/MMDST", "BC", "DC");

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


DEFINE_DMA_CHANNEL(DMAC, 1);

unsigned build_chain(int ichain, unsigned pa, unsigned len)
/* build the chain as long it can, return actual len */
/* actually, this is a really trivial single element chain, but no worries */
{

}
void run_chained_dma(unsigned pa, unsigned len)
{
	struct iop321_dma_desc* dmads[2][MAXCHAIN];
	int ichain = 0;
	int cursor;
	unsigned stat;


	for (cursor = 0; cursor < len; ichain =! ichain){
		cursor += build_chain(ichain, pa+cur, len);
		while (!DMA_BUSY(DMAC, stat){
			yield();
		}
		DMA_ARM();
		DMA_FIRE(GL.chains[ichain]);
	}

	while(!DMA_BUSY(GL.chains[!ichain], stat)){
		yield();
	}
}

static ssize_t set_fire(
	struct device *dev,
	struct device_attribute *attr,
	const char* buf,
	size_t count)
{
	unsigned pa, len;

	if (sscanf(buf, "%x %d", &pa, &len) == 2){
		run_chained_dma(pa, len);
	}
	return count;
}

static DEVICE_ATTR(fire, S_IWUGO, 0, set_fire);

static int mk_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_fire);
	return 0;
}
int acqfiber_push_probe(struct device *dev)
{
	info("%s\n%s\n%s\n%s", 
	     acqfiber_push_driver_name, acqfiber_push_driver_string,
	     acqfiber_push_driver_version, acqfiber_push_copyright);

	get_regions();
	create_proc_entries();
	mk_sysfs(dev);
	return 0;
}

int acqfiber_push_remove(struct device *dev)
{       
	if (GL.padding.va) kfree(GL.padding.va);	
	if (GL.init_ramp.va) kfree(GL.init_ramp.va);
	free_chains();
	return 0;
}

static void acqfiber_push_dev_release(struct device * dev)
{
	info("not used");
}


static struct device_driver acqfiber_push_driver = {
	.name     = "acqfiber_push",
	.probe    = acqfiber_push_probe,
	.remove   = acqfiber_push_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acqfiber_push_device = {
	.name = "acqfiber_push",
	.id   = 0,
	.dev = {
		.release    = acqfiber_push_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acqfiber_push_init_module( void )
{
	int rc = driver_register(&acqfiber_push_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acqfiber_push_device);
}


static void __exit
acqfiber_push_exit_module(void)
{
	info("");
	platform_device_unregister(&acqfiber_push_device);
	driver_unregister(&acqfiber_push_driver);
}


module_init(acqfiber_push_init_module);
module_exit(acqfiber_push_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("acqfiber_push mechanism");
