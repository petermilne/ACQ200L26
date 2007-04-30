/* ------------------------------------------------------------------------- */
/* acq2xx-pwm.c "continuous dma" PWM driver for ACQ2xx                       */
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

/** @file acq2xx-pwm.c
 *
 * kernel module to operate DIO32 port as a PWM controller.
 *
 * DMAC chain loops back on itself - dma runs forever until stopped.
 */


#define ACQ196

#include <linux/string.h>
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
#define acq200_debug acq2xx_pwm_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq200-dmac.h"

#include "acq196.h"

#include "acq200-inline-dma.h"

int acq2xx_pwm_debug;
module_param(acq2xx_pwm_debug, int, 0664);


#define ENTRIES_PER_CYCLE 1024

#define MAXPOOL (ENTRIES_PER_CYCLE*3)

#define VERID "$Revision: 1.4 $ build B1000 "
char acq2xx_pwm_driver_name[] = "acq2xx_pwm";
char acq2xx_pwm_driver_string[] = "D-TACQ Scatter Control Device";
char acq2xx_pwm_driver_version[] = VERID __DATE__ " Features:\n";
char acq2xx_pwm_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


#define ALL_CHANNELS 0xffffffff

#define Descriptor struct iop321_dma_desc

struct DescriptorListElement {
	Descriptor* descriptor;
	struct list_head list;
};

struct SourceData {
	u32 lo16;           /* dio is low 16 bits in each case */
	u32 hi16;
};
#define POOLSZ (MAXPOOL*sizeof(struct DescriptorListElement))

#define SRCDATASZ (sizeof(struct SourceData))
#define SRCDATA_BUFFER_LEN (ENTRIES_PER_CYCLE*SRCDATASZ)


#define DIO32_DATA (ACQ200_EXTERNIO_P+0x0c)

struct Globs {
	int entries_per_cycle;
	struct DescriptorListElement *pool;
	int ipool;

	struct list_head the_chain;    /* array [epc] */
	int iload;

	struct DmaBlock {
		struct SourceData* data;
		dma_addr_t handle;
	}
		source, dummy;
};


static struct Globs dg = {
	.entries_per_cycle = ENTRIES_PER_CYCLE
};





static Descriptor* buildDescriptorMem2Pbi(u32 src, u32 len)
{
	Descriptor* desc = acq200_dmad_alloc();

	if (desc){
		desc->NDA = 0;
		desc->PDA = src;
		desc->PUAD = 0;
		desc->LAD = DIO32_DATA;
		desc->BC = len;             
		desc->DC = DMA_DCR_MEM2MEM;
		desc->NDA = 0;
	}else{
		BUG();
	}
	return desc;
}

static Descriptor* buildDescriptorMem2Mem(u32 src, u32 dst, u32 len)
{
	Descriptor* desc = acq200_dmad_alloc();

	if (desc){
		desc->NDA = 0;
		desc->PDA = src;
		desc->PUAD = 0;
		desc->LAD = dst;
		desc->BC = len;             
		desc->DC = DMA_DCR_MEM2MEM;
		desc->NDA = 0;
	}else{
		BUG();
	}
	return desc;
}



static void freeElementsTable(void)
{
	kfree(dg.source.data);
	kfree(dg.dummy.data);
}
static void cleanPool(void)
{
	memset(dg.pool, 0, POOLSZ);
	dg.ipool = 0;
}

static void emptyPool(void) 
{
	int ix;

	for (ix = 0; ix != MAXPOOL; ++ix){
		if (dg.pool[ix].descriptor){
			acq200_dmad_free(dg.pool[ix].descriptor);
		}
	}
	cleanPool();
}

static void allocPool(void)
{
	dbg(1, "attempt allocate %d", POOLSZ);
	dg.pool = vmalloc(POOLSZ);
	cleanPool();
}

static void freePool(void)
{
	emptyPool();
	vfree(dg.pool);
	dg.pool = 0;
	dg.ipool = 0;
}
static struct DescriptorListElement* poolAlloc(void)
{
	if (dg.ipool < MAXPOOL-1){
		return dg.pool + dg.ipool++;
	}else{
		return 0;
	}
}


static void make_chain_loop(struct list_head* head)
{
	struct DescriptorListElement* cursor;
	int ecount = 0;

	list_for_each_entry(cursor, head, list){
		ecount++;
		if (cursor->list.next != head){
			struct DescriptorListElement* next = 
				list_entry(cursor->list.next,
				   struct DescriptorListElement,list);

			cursor->descriptor->NDA = next->descriptor->pa;
		}else{
			struct DescriptorListElement* first = 
				list_entry(head->next,
				   struct DescriptorListElement,list);
			cursor->descriptor->NDA = first->descriptor->pa;
			cursor->descriptor->DC |= IOP321_DCR_IE;
		}
	}

	dbg(1, "ecount %d", ecount);
}

static void build_list(void)
/** build array of DMA chains. */
{
	int entry;

	INIT_LIST_HEAD(&dg.the_chain);

	dbg(1, "entries_per_cycle %d", dg.entries_per_cycle);

	for (entry = 0; entry != dg.entries_per_cycle; ++entry){
		u32 src_pa = dg.source.handle + entry * SRCDATASZ;
		struct DescriptorListElement* the_entry = poolAlloc();
		struct DescriptorListElement* dummy_entry = poolAlloc();

		the_entry->descriptor =
			buildDescriptorMem2Pbi(src_pa, SRCDATASZ);
		list_add_tail(&the_entry->list, &dg.the_chain);

		dummy_entry->descriptor =
			buildDescriptorMem2Mem(
				src_pa, 
				dg.dummy.handle + entry * SRCDATASZ, 
				SRCDATASZ);
		list_add_tail(&dummy_entry->list, &dg.the_chain);
	}

	make_chain_loop(&dg.the_chain);
	dg.iload = entry;
}


#define DMA_EOT (IOP321_CSR_EOCIF|IOP321_CSR_EOTIF)
/** returns Non Zero on Error */
#define NZE int 

static inline NZE dmaTee(struct DmaChannel* channel, struct list_head* head)
{
	struct DescriptorListElement* first = 
			list_entry(head->next, 
				   struct DescriptorListElement, list);
	dbg(1, "channel %d", channel->id);

	DMA_DISABLE(*channel);
	DMA_STA(*channel) = DMA_EOT;
	DMA_ARM_DIRECT((*channel), first->descriptor);
	DMA_FIRE(*channel);
	return 0;
}





static void stop(void)
/** stop the capture. */
{
	DEFINE_DMA_CHANNEL(c1, 1);
	DMA_DISABLE(c1);
}



static void run_shot(void)
/** runs the capture.
 */
{
	DEFINE_DMA_CHANNEL(c1, 1);

	if (dmaTee(&c1, &dg.the_chain)){
		err("dmaTee ERROR");
	}
}


u8 getPattern(int amp) {
	static struct Patterns {
		int perthousand;
		u8 pattern;
	} PartPatterns [] = {
	{    0, 0x00 },
	{  128, 0x80 },
	{  256, 0x88 },
	{  384, 0xa8 },
	{  512, 0xaa },
	{  640, 0xea },
	{  768, 0xee },
	{  896, 0xfe },
	{ 1024, 0xff }
	};
#define NPATTERNS (sizeof(PartPatterns)/sizeof(struct Patterns))

	int ipat;

	for (ipat = NPATTERNS; ipat--; ){
		if (amp >= PartPatterns[ipat].perthousand){
			break;
		}
	}
	return PartPatterns[ipat].pattern;
}


int _amp;

static void initPattern(unsigned channels, int ampl)
/** trivial pattern implementation */
{

	u8 pat = getPattern(ampl);
	u32 *pcursor = (u32*)dg.source.data;
	u32 *pend    = pcursor + (ENTRIES_PER_CYCLE*SRCDATASZ/sizeof(u32));
	u8 mask = 0x80;
	
	while (pcursor < pend){
		*pcursor++ = ((pat & mask) != 0) ? channels: 0;
//		*pcursor++ = ((pat & mask) != 0) ? channels: 0;
		mask >>= 1;
		if (!mask){
			mask = 0x80;
		}
	}
	
}

static void init_default(struct device* dev){
	allocPool();

	dg.source.data = kmalloc(SRCDATA_BUFFER_LEN, GFP_KERNEL);
	assert(dg.source.data);
	memset(dg.source.data, 0, SRCDATA_BUFFER_LEN);

	dg.source.handle = dma_map_single( 
		dev, dg.source.data, SRCDATA_BUFFER_LEN, DMA_TO_DEVICE);

	dg.dummy.data = kmalloc(SRCDATA_BUFFER_LEN, GFP_KERNEL);
	dg.dummy.handle = dma_map_single( 
		dev, dg.source.data, SRCDATA_BUFFER_LEN, DMA_TO_DEVICE);
	build_list();

	initPattern(ALL_CHANNELS, _amp = ENTRIES_PER_CYCLE/2);
	dma_sync_single(dev, dg.source.handle, SRCDATA_BUFFER_LEN,
			DMA_TO_DEVICE);
}


/** sysfs hooks
 */

static ssize_t show_entries_per_cycle(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->sample_read_start);
}

static ssize_t store_entries_per_cycle(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int nelems;

	if (sscanf(buf, "%u", &nelems) == 1){
		; /** @todo const elems per cycle */
	}
	return strlen(buf);
}
static DEVICE_ATTR(epc, S_IRUGO|S_IWUGO,
		   show_entries_per_cycle, store_entries_per_cycle);


static void remove_default(void){
	freePool();
	freeElementsTable();
}



static ssize_t show_run(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"empty\n");
}

static ssize_t store_run(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	run_shot();
	return strlen(buf);
}
static DEVICE_ATTR(run, S_IRUGO|S_IWUGO,  show_run, store_run);


static ssize_t show_stop(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return 0;
}

static ssize_t store_stop(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	stop();
	return strlen(buf);
}
static DEVICE_ATTR(stop, S_IRUGO|S_IWUGO,  show_stop, store_stop);




static ssize_t show_amp(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d 0x%02x\n", _amp, getPattern(_amp));
	return 0;
}

static ssize_t store_amp(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	_amp = simple_strtol(buf, 0, 0);
	initPattern(ALL_CHANNELS, _amp);
	dma_sync_single(dev, dg.source.handle, SRCDATA_BUFFER_LEN,
			DMA_TO_DEVICE);
	return strlen(buf);
}
static DEVICE_ATTR(amp, S_IRUGO|S_IWUGO,  show_amp, store_amp);




static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq2xx_pwm_driver_name,
		       acq2xx_pwm_driver_string,
		       acq2xx_pwm_driver_version,
		       acq2xx_pwm_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);



static int mk_scatter_sysfs(struct device *dev)
/** create device hooks: in /dev/pwm (link into sysfs). */
{
	DEVICE_CREATE_FILE(dev, &dev_attr_run);
	DEVICE_CREATE_FILE(dev, &dev_attr_stop);
	DEVICE_CREATE_FILE(dev, &dev_attr_epc);
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_amp);
	return 0;
}


static void acq2xx_pwm_dev_release(struct device * dev)
{
	info("");
}


static int acq2xx_pwm_probe(struct device *dev)
{
	info("");
	init_default(dev);
	mk_scatter_sysfs(dev);
	return 0;
}

static int acq2xx_pwm_remove(struct device *dev)
{
	remove_default();
	return 0;
}


static struct device_driver acq2xx_pwm_driver = {
	.name     = "acq2xx_pwm",
	.probe    = acq2xx_pwm_probe,
	.remove   = acq2xx_pwm_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq2xx_pwm_device = {
	.name = "acq2xx_pwm",
	.id   = 0,
	.dev = {
		.release    = acq2xx_pwm_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq2xx_pwm_init( void )
{
	acq200_debug = acq2xx_pwm_debug;

	driver_register(&acq2xx_pwm_driver);
	return platform_device_register(&acq2xx_pwm_device);
}


static void __exit
acq2xx_pwm_exit_module(void)
{
	info("");
	platform_device_unregister(&acq2xx_pwm_device);
	driver_unregister(&acq2xx_pwm_driver);
}


module_init(acq2xx_pwm_init);
module_exit(acq2xx_pwm_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ2xx PWM Control");



