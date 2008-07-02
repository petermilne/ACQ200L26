/* ------------------------------------------------------------------------- */
/* btt.c bus timing tester				                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Peter Milne, D-TACQ Solutions Ltd
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

#define VERID "btt $Revision:$ B1003"

#define ACQ196

/** @file btt.c Bus Timing Tester 
 *
 * config (write):
 * to,from,nbytes[,pat1,patN]
 *
 * to, from:  
 *	R0x12345678
 *
 * R:
 *	P : PCI | M : MEM
 *
 * nbytes=hex or decimal count; 0 means no entry (timing check).
 * pat1,patN = 32 bit pattern to write
 *
 * run (write):
 *	kickoff a capture
 *
 * stat (read):
 *	gtsr1,gtsr2,status
 *
 * where gtsr1 : gtsr initial reading
 *       gtsr2 : gtsr final reading
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

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


/* keep debug local to this module */
#define acq200_debug btt_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"

#include "acq200-dmac.h"

#define DIO_REG_TYPE (volatile u32*)
#include "acqX00-rtm.h"


#include "asm/arch/iop321-irqs.h"
#include "acq200-inline-dma.h"

#define R_PCI	'P'
#define R_MEM   'M'


int btt_debug;
module_param(btt_debug, int, 0664);

int force_wait_write;
module_param(force_wait_write, int, 0664);

int LOOPN = 128;
module_param(LOOPN, int, 0664);

#define DMA_CHANNEL 0
DEFINE_DMA_CHANNEL(dma0, 0);

struct iop321_dma_desc* gtsr_pad;
static u32* gtsr1;
static u32* gtsr2;
static u32 status;

struct Stats {
	unsigned s_min;
	unsigned s_max;
	unsigned total;
	unsigned samples;
	int is_error;
};

static struct Stats S_stats;

void init_dma(u32 lad, u32 pda, u32 bc, u32 dc) 
{
	static int init_done;
	struct iop321_dma_desc *gtsr1_desc;
	struct iop321_dma_desc *gtsr2_desc;


	if (init_done == 0){
		gtsr_pad = acq200_dmad_alloc();
		gtsr1 = (u32*)gtsr_pad;
		gtsr2 = ((u32*)gtsr_pad) + 1;				
		init_done = 1;
	}

	dma_cleanup(&dma0);

	gtsr1_desc = acq200_dmad_alloc();
	gtsr1_desc->NDA = 0;
	gtsr1_desc->PDA = IOP321_REG_PA(IOP321_GTSR);
	gtsr1_desc->PUAD = 0;
	gtsr1_desc->LAD = gtsr_pad->pa;
	gtsr1_desc->BC = 4;
	gtsr1_desc->DC = DMA_DCR_MEM2MEM;

	dma_append_chain(&dma0, gtsr1_desc, "gtsr1");

	if (bc != 0){
		struct iop321_dma_desc *payload = acq200_dmad_alloc();
		payload->NDA = 0;
		payload->PDA = pda;
		payload->PUAD = 0;
		payload->LAD = lad;
		payload->BC = bc;
		payload->DC = dc;

		dma_append_chain(&dma0, payload, "payload");
	}

	if (force_wait_write && dc == DMA_DCR_PCI_MW){
		/* force a read after write */
		struct iop321_dma_desc *post = acq200_dmad_alloc();
		post->NDA = 0;
		post->PDA = pda;
		post->PUAD = 0;
		post->LAD = lad;
		post->BC = 4;
		post->DC = DMA_DCR_PCI_MR;

		dma_append_chain(&dma0, post, "post");
	}
	gtsr2_desc = acq200_dmad_alloc();
	gtsr2_desc->NDA = 0;
	gtsr2_desc->PDA = IOP321_REG_PA(IOP321_GTSR);
	gtsr2_desc->PUAD = 0;
	gtsr2_desc->LAD = gtsr_pad->pa+4;
	gtsr2_desc->BC = 4;
	gtsr2_desc->DC = DMA_DCR_MEM2MEM;

	dma_append_chain(&dma0, gtsr2_desc, "gtsr2");
}

static struct CONFIG {
	u32 to;
	u32 from;
	char to_region;
	char from_region;
	int nbytes;
	u32 pat1;
	u32 pat2;
}
	S_config;


static void config_dma(void)
{
	u32 lad;
	u32 pda;
	u32 dc;

	switch(S_config.to_region){
	case R_PCI:				
		dc = DMA_DCR_PCI_MW;	/* WRITE PCI */
		pda = S_config.to;
		lad = S_config.from;
		break;
	case R_MEM:
		switch(S_config.from_region){
		case R_PCI:
			dc = DMA_DCR_PCI_MR;
			pda = S_config.from;
			lad = S_config.to;
			break;
		case R_MEM:
			dc = DMA_DCR_MEM2MEM;	/* MEM2MEM */
			pda = S_config.from;
			lad = S_config.to;
			break;
		default:
			assert(0);
		}
		break;
	default:
		assert(0);
	}
	init_dma(lad, pda, S_config.nbytes, dc);
}

static void get_stats(struct Stats *stats)
{
	unsigned g1 = *gtsr1;
	unsigned g2 = *gtsr2;
	unsigned delta = g2 - g1? g2-g1: g2 + (0xFFFFFFFF-g1);

	stats->total += delta;
	if (stats->samples++ == 0){
		stats->s_min = stats->s_max = delta;
	}else if (delta < stats->s_min){
		stats->s_min = delta;
	}else if (delta > stats->s_max){
		stats->s_max = delta;
	}
}
static int capture(struct Stats *stats)
{
	int pollcat = 0;

	int err0 = acq200_dma_error_count(DMA_CHANNEL);

	config_dma();
	DMA_ARM(dma0);
	DMA_FIRE(dma0);

	while(!DMA_DONE(dma0, status)){
		if ((++pollcat & 0xff) == 0){
			dbg(1, "pollcat: %d", pollcat);
			yield();
		}
	}

	if (acq200_dma_error_count(DMA_CHANNEL) > err0){
		stats->is_error = 1;
		return -1;
	}else{
		get_stats(stats);
		return 0;
	}
}
static ssize_t set_config(
	struct device *dev,
	struct device_attribute *attr,
	const char* buf, size_t count)
{	
	struct CONFIG cf;
	int nscan;

	if ((nscan = sscanf(buf,"%c%x,%c%x,%d,%x,%x",
			   &cf.to_region, &cf.to,
			   &cf.from_region, &cf.from,
			   &cf.nbytes,
			   &cf.pat1, &cf.pat2)) >= 5){
		switch(cf.to_region){
		case R_PCI:
		case R_MEM:
			break;
		default:
			err("Bad to_region %c", cf.to_region);
			return count;

		}
		switch(cf.from_region){
		case R_PCI:
		case R_MEM:
			break;
		default:
			err("Bad from_region %c", cf.from_region);
			return count;
		}
		
		if (cf.to_region == R_PCI &&
		    cf.from_region == R_PCI  ){
			err("PCI to PCI not supported");
			return count;
		}

		memcpy(&S_config, &cf, sizeof(cf));
	}		       
	

	return count;
}
static ssize_t show_config(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
	)
{
	return sprintf(buf, "%c0x%08x,%c0x%08x,%d,%08x,%08x\n",
		S_config.to_region, S_config.to,
		S_config.from_region, S_config.from,
		S_config.nbytes,
		S_config.pat1, S_config.pat2);
}

static DEVICE_ATTR(config, S_IWUGO|S_IRUGO, show_config, set_config);

static ssize_t set_run(
	struct device *dev,
	struct device_attribute *attr,
	const char* buf, size_t count)
/* NB : blocks until completion */
{	
	int ii;
	struct Stats stats = {};

	for (ii = 0; ii != LOOPN; ++ii){
		if (capture(&stats)){
			break;
		}
	}		
	S_stats = stats;	
	return count;
}

static DEVICE_ATTR(run, S_IWUGO, 0, set_run);

static ssize_t show_stat(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
	)
{
	if (S_stats.samples && !S_stats.is_error){
		unsigned mean = S_stats.total/S_stats.samples;
		return sprintf(buf, "%d,%u,%u,%u\n",
				S_stats.samples, S_stats.s_min,
			       S_stats.s_max, mean);
	}else{
		return sprintf(buf, "%d %s\n", S_stats.samples, 
			       S_stats.is_error? "ERROR": "");
	}
}

static DEVICE_ATTR(stat, S_IRUGO, show_stat, 0);


static int btt_mk_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_config);
	DEVICE_CREATE_FILE(dev, &dev_attr_run);
	DEVICE_CREATE_FILE(dev, &dev_attr_stat);
	return 0;
}

static int btt_probe(struct device *dev)
{
	info("");
	btt_mk_sysfs(dev);
	return 0;
}

static int btt_remove(struct device *dev)
{
	info("");
//	btt_rm_sysfs(dev);
	return 0;
}


static void btt_dev_release(struct device *dev)
{
	info("");
}
static u64 dma_mask = 0x00000000ffffffff;


static struct platform_device btt_device = {
	.name = "btt",
	.id   = 0,
	.dev = {
		.release    = btt_dev_release,
		.dma_mask   = &dma_mask
	}

};

static struct device_driver btt_driver = {
	.name = "btt",
	.probe = btt_probe,
	.remove = btt_remove,
	.bus = &platform_bus_type
};

static int __init btt_init(void)
{
	int rc;
	acq200_debug = btt_debug;

	info(VERID);
	rc = driver_register(&btt_driver);
	if (rc){
		err("driver_register failed");
		return rc;
	}	
	return platform_device_register(&btt_device);
	
}
static void __exit
btt_exit_module(void)
{
	info("");
	dma_cleanup(&dma0);
	acq200_dmad_free(gtsr_pad);
	platform_device_unregister(&btt_device);
	driver_unregister(&btt_driver);
}

module_init(btt_init);
module_exit(btt_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Bus Timing Tester for ACQ2xx");

