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

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/iop321-dma.h>
#include "acq200-dmac.h"
#include "acq200_debug.h"

#include "acq200_hostdrv.h"

#define MAXDEV	16		/* including self */

#ifdef __ACQ200_INLINE_DMA_H__
#error FILE ALREADY INCLUDED
#endif
#define MAXCHAIN	(MAXDEV+4)
#include "acq200-inline-dma.h"

#define EXCLUDE_PBC_INLINES
#include "prebuiltChainUtils.h"


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

/** Globals .. keep to a minimum! */
char acq100_gather_driver_name[] = "acq100_gather";
char acq100_gather_driver_string[] = "D-TACQ gather driver";
char acq100_gather_driver_version[] = "B1002";
char acq100_gather_copyright[] = "Copyright (c) 2011 D-TACQ Solutions Ltd";


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
	struct Region dest;
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
static void addDest(unsigned pa, int len, const char* name, unsigned dc)
{
	GL.dest.pa = pa;
	GL.dest.len = len;
	GL.dest.name = name;
	GL.dest.dc = dc;
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
		addSource(device->ram.pa, device->ram.len, 
			  device->ram.name, DMA_DCR_PCI_MR);
		info("device %s pa: 0x%08lx len: %d",
		     device->ram.name, device->ram.pa, device->ram.len);

	}		

	/** @@todo dest is local .. want rtm-t */
	addDest(*IOP321_IATVR2 + control_block * control_numblocks,
		GL.is * control_block * control_numblocks, 
		"local", DMA_DCR_MEM2MEM);
}

void make_chain(struct DmaChannel* dmac, int cblock)
{
	int isrc;
	u32 lad = GL.dest.pa + cblock*GL.is*caplen;

	dmac->regs = IOP321_DMA1_CCR;
	dmac->id = 1;

	for (isrc = 0; isrc < GL.is; ++isrc, lad += caplen){
		struct iop321_dma_desc* dmad = acq200_dmad_alloc();
		struct Region *region = &GL.sources[isrc];
		dmad->NDA = 0;
		dmad->PDA = region->pa + cblock*control_block;
		dmad->PUAD = 0;
		dmad->LAD = lad;
		dmad->BC = caplen;
		dmad->DC = region->dc;
		dma_append_chain(dmac, dmad, region->name);
	}
}

void make_chains(void)
{
	int chn;
	GL.chains = kzalloc(
		control_numblocks*sizeof(struct DmaChannel), GFP_KERNEL);

	for (chn = 0; chn < control_numblocks; ++chn){
		make_chain(&GL.chains[chn], chn);
	}
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

void prebuiltChainHandler(struct PrebuiltChain* pbc)
{
	if (!IN_RANGE(pbc->iblock, 0, control_numblocks-1)){
		err("pbc not in range .. discard");
	}else{
		int iblock = pbc->iblock;
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
}

void hookup(void)
{
	acq200_registerPrebuiltClient(prebuiltChainHandler);
}
int acq100_gather_init_module(void)
{
	info("%s\n%s\n%s\n%s", 
	     acq100_gather_driver_name, acq100_gather_driver_string,
	     acq100_gather_driver_version, acq100_gather_copyright);

	get_regions();
	make_chains();
	hookup();

	create_proc_entries();
	return 0;
}

void acq100_gather_exit_module(void)
{
	free_chains();
}


module_init(acq100_gather_init_module);
module_exit(acq100_gather_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("acq100_gather mechanism");
