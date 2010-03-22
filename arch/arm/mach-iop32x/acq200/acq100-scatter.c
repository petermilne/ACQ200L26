/* ------------------------------------------------------------------------- */
/* acq100-scatter.c driver for acq100 lowlatency controller multi destination*/
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

/** @file acq100-scatter.c
 *
 * kernel module to control lowlatency scatter application
 *
 * Core capture loop is in do_run().
 * During capture, the program will
 *
 - Poll FIFO Not EMPTY
 - Tee off next DMA chain.
 *
 * The DMA chains have been precooked, and there are a set number of entries
 * per cycle before the chains are restarted.
 *
 * A chain always starts with a SOURCE element to copy data from FIFO to
 * local DDRAM. Subsequent chain entries may scatter this data to up to
 * MAXDEST destinations.
 *
 * The destinations are predefined by defining a Destination by writing
 * to the appropriate device node:
 - store
 - dest1
 - dest2
 - ...
 - destN
 * 
 * The definition is as follows:
 * "base_pa interval block_len dest_inc"
 - base_pa : physical bus address of destination
 - interval: gap between samples (1= take every sample)
 - block_len: length of transfer in samples
 - dest_inc : if 1, increment destination address every transfer, else overwrite base

 * Once the destinations have been defined (only define as many as you want)
 * Build chains by writing to build_list
 * Then run the shot by writing to run.
 * The shot starts on activation of trigger, and ends on deactivation.

 * in ACCUMULATE mode:
 * store 
 * dest1 - accumulator
 * dest2 ... regular remote destinations
 */


#define ACQ196

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
#define acq200_debug acq100_scatter_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq200-dmac.h"

#include "acq196.h"

#include "acq200-inline-dma.h"

int acq100_scatter_debug;
module_param(acq100_scatter_debug, int, 0664);


#define MAXDEST 6             
#define SAMPLE_SIZE sample_size()
#define ENTRIES_PER_CYCLE 1000

#define MAXPOOL (MAXDEST*ENTRIES_PER_CYCLE*2)

#define VERID "$Revision: 1.6 $ build B1000 "
char acq100_scatter_driver_name[] = "acq100_scatter";
char acq100_scatter_driver_string[] = "D-TACQ Scatter Control Device";
char acq100_scatter_driver_version[] = VERID __DATE__ " Features:\n";
char acq100_scatter_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";

int disable_acq_debug = 0;         /* set 1 */
module_param(disable_acq_debug, int, 0664);


int ESTOP_DIx = 5;
module_param(ESTOP_DIx, int, 0644);

int ESTOP_DIx_HIGH = 0;
module_param(ESTOP_DIx_HIGH, int, 0644);

int ACCUMULATE = 0;
module_param(ACCUMULATE, int, 0644);

#define CHANNEL_MASK (CAPDEF->channel_mask)

#define Descriptor struct iop321_dma_desc

struct Destination {
	u32 base_pa;
	int interval;
	int block_len;
	int dest_inc;
	int seq;
	Descriptor* (*build)(struct Destination* this, int idx);
};


struct DescriptorListElement {
	Descriptor* descriptor;
	struct list_head list;
};

#define POOLSZ (MAXPOOL*sizeof(struct DescriptorListElement))

#define ID_SRC 0 

struct Globs {
	int entries_per_cycle;
	struct DescriptorListElement *pool;
	int ipool;

	struct list_head* elements;    /* array [epc] */
	int iload;
	struct Destination destinations[MAXDEST];  /* ID_SRC is source */
	struct device_attribute dest_attr[MAXDEST];


	int please_stop;
	u32 imask;
	u32 coldfifo_ne_mask;

	u32 FIFERR;
	u32 fifo_th;
	u32 trig_mask;
	u32 trig_quit;

	u32 estop_mask;
	u32 estop_quit;

	int yield;
	unsigned long max_cycles;      /** stop here if non zero */
	unsigned soft_trigger;         /** soft trigger if set */

	struct pci_mapping fifo_incoming;
	struct pci_mapping accumulator;
};

static struct Stats {
	unsigned long cycle;
	int entry;
	int state;
	int data_pollcat;
	int dma_arms;
	int dma_pollcat;
	const char* stop_reason;
} ds;


#define ST_IDLE 0
#define ST_ARM  1
#define ST_RUN  2

#define SET_STATE(s) (ds.state = (s))

static struct Globs dg;


#define GET_BBPA(idx) (pa_buf(DG) + (idx)*SAMPLE_SIZE)

#define FULL_LEN	(96*sizeof(int))

void accumulate(void)
/* here is the "math". This could be extended .. */
/* output data will be valid for 32 bit data only, but on 16 bit
 * it will show timing
 */
{
	int ic;
	int* acc = (int*)dg.accumulator.va;
	int* data = (int*)dg.fifo_incoming.va;

	dma_sync_single_for_cpu(
		DG->dev, dg.fifo_incoming.pa, FULL_LEN, DMA_FROM_DEVICE);

	dbg(1, "acc %p += data %p", acc, data);

	for (ic = 0; ic < 96; ++ic){
		acc[ic] += data[ic];
	}

	dma_sync_single_for_device(DG->dev, 
			dg.accumulator.pa, FULL_LEN, DMA_TO_DEVICE);
}
int llc_intsDisable(void)
{
	MASK_ITERATOR_INIT(it, dg.imask);

	while ( mit_hasNext( &it ) ){
		disable_irq_nosync( mit_getNext( &it ) );
	}

	return 0;
}
int llc_intsEnable(void)
{
	MASK_ITERATOR_INIT(it, dg.imask);

	while ( mit_hasNext( &it ) ){
		enable_irq( mit_getNext( &it ) );
	}

	return 0;
}

static void set_fifo_ne_mask(unsigned cmask)
{
	u32 coldfifo_ne_mask = 0;
	if (cmask&1){
		coldfifo_ne_mask |= ACQ196_FIFSTAT_ADC1_NE;
	}
	if (cmask&2){
		coldfifo_ne_mask |= ACQ196_FIFSTAT_ADC2_NE;
	}
	if (cmask&4){
		coldfifo_ne_mask |= ACQ196_FIFSTAT_ADC3_NE;
	}
	dg.coldfifo_ne_mask = coldfifo_ne_mask;
}


static char* descriptor_to_string(Descriptor* d, char buf[])
{
	sprintf(buf, "%08x %08x %08x %04x %04x %08x",
		d->NDA, d->PDA, d->LAD, d->BC, d->DC, d->pa);
	return buf;
}

static int shouldBuild(struct Destination* this, int idx)
{
	if (this->block_len == 1){
		if (this->interval == 1){
			return 1;
		}else{
			return idx % this->interval == this->seq;
		}
	}else if (this->interval){
		return idx % this->interval == this->block_len - 1;
	}else{
		return 0;
	}
}

static Descriptor* buildDescriptorMem2Pci(struct Destination* this, int idx)
{
	if (shouldBuild(this, idx)){
		Descriptor* desc = acq200_dmad_alloc();
		int blockoff = (this->block_len-1);
		int remote_offset = 0;

		if (this->dest_inc){
			remote_offset = 
			(idx*this->block_len/this->interval - blockoff)*
				SAMPLE_SIZE;
		}

		if (desc){
			desc->NDA = 0;
			desc->PDA = this->base_pa + remote_offset;
			desc->PUAD = 0;
			if (ACCUMULATE){
				dbg(1, "ACCUMULATE desc->LAD %08x",
				    dg.accumulator.pa);
				desc->LAD = dg.accumulator.pa;
			}else{
				desc->LAD = GET_BBPA(idx - blockoff);
			}
			desc->BC = this->block_len * SAMPLE_SIZE;
			desc->DC = DMA_DCR_PCI_MW;
			desc->NDA = 0;
		}else{
			BUG();
		}
		return desc;
	}else{
		return 0;
	}

}

static Descriptor* buildDescriptorPbi2Mem(struct Destination* this, int idx)
{
	if (shouldBuild(this, idx)){
		Descriptor* desc = acq200_dmad_alloc();

		if (desc){
			desc->NDA = 0;
			desc->PDA = this->base_pa;
			desc->PUAD = 0;
			if (ACCUMULATE){
				desc->LAD = dg.fifo_incoming.pa;
				dbg(1, "ACCUMULATE LAD %08x", 
						dg.fifo_incoming.pa);
			}else{
				desc->LAD = GET_BBPA(idx - (this->block_len-1));
			}
			desc->BC = this->block_len * SAMPLE_SIZE;
			desc->DC = DMA_DCR_MEM2MEM|IOP321_DCR_IE;
		}else{
			BUG();
		}
		return desc;
	}else{
		return 0;
	}
}


#define FNAME_BUILD(f)						\
        ((f)==buildDescriptorPbi2Mem? "Pbi2Mem":		\
         (f)==buildDescriptorMem2Pci? "Mem2Pci": "no-build")

static ssize_t showDestination(
	struct Destination* dest, struct device * dev, char * buf)
{
	return sprintf(buf, "0x%08x %d %d %d %d %s\n",
		       dest->base_pa,
		       dest->interval,
		       dest->block_len,
		       dest->dest_inc,
		       dest->seq,
		       FNAME_BUILD(dest->build));
}
static ssize_t storeDestination(
	struct Destination* dest,
	struct device * dev, const char * buf, size_t count)
{
	int rc = sscanf(buf, "0x%08x %d %d %d %d",
		       &dest->base_pa,
		       &dest->interval,
		       &dest->block_len,
		       &dest->dest_inc,
		       &dest->seq);

	if (rc == 4){
		dest->seq = 0;
	}
	if (rc < 4){
		err("parsing FAILED");
	}
	return strlen(buf);
}

/** sorely missing m4 or C++ to generate these funcs */

#define DEF_DEST_ACCESSORS(id)					        \
static ssize_t showDestination ## id(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return showDestination(&dg.destinations[id], dev, buf);		\
}									\
static ssize_t storeDestination ## id(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count)					\
{									\
	return storeDestination(&dg.destinations[id], dev, buf, count);	\
}

DEF_DEST_ACCESSORS(0);
DEF_DEST_ACCESSORS(1);
DEF_DEST_ACCESSORS(2);
DEF_DEST_ACCESSORS(3);
DEF_DEST_ACCESSORS(4);
DEF_DEST_ACCESSORS(5);
/** @todo MAXDEST > 5 ? add more DEFS here */

#define INIT_DEST_ATTR(id, _name)			\
	.dest_attr[id].show = showDestination ## id,	\
	.dest_attr[id].store = storeDestination ## id,	\
        .dest_attr[id].attr.name = _name,		\
        .dest_attr[id].attr.mode = 0666,		\
        .dest_attr[id].attr.owner = THIS_MODULE


static struct Globs dg = {
	.entries_per_cycle = ENTRIES_PER_CYCLE,
	.FIFERR = ACQ196_FIFSTAT_HOT_OVER,
/** @todo  ACQ196_FIFSTAT_HOT_UNDER _should_ work but doesn't
	.FIFERR = ACQ196_FIFSTAT_HOT_UNDER|ACQ196_FIFSTAT_HOT_OVER,
*/
	INIT_DEST_ATTR(0, "source"),
	INIT_DEST_ATTR(1, "dest1"),
	INIT_DEST_ATTR(2, "dest2"),
	INIT_DEST_ATTR(3, "dest3"),
	INIT_DEST_ATTR(4, "dest4"),
	INIT_DEST_ATTR(5, "dest5")
/** @todo MAXDEST > 5 ? add more INITs here */
};

static void allocElementsTable(int epc){
	if (epc != dg.entries_per_cycle && dg.elements){
		kfree(dg.elements);
		dg.elements = 0;
	}
	if (epc != dg.entries_per_cycle || !dg.elements){
		int esize = dg.entries_per_cycle*sizeof(struct list_head);
		dbg(1, "attempt kmalloc %d", esize);
		dg.elements = kmalloc(esize, GFP_KERNEL);
		if (!dg.elements){
			BUG();
		}
		dg.entries_per_cycle = epc;
	}
}

void init_accumulate_globals(void)
{
	dbg(1, "10");	
	if (dg.accumulator.va == 0){
		dg.fifo_incoming.va = VA_TBLOCK(GET_TBLOCK(0));
		dg.fifo_incoming.pa = dma_map_single(
			DG->dev, dg.fifo_incoming.va,FULL_LEN,DMA_FROM_DEVICE);
		dg.fifo_incoming.len = FULL_LEN;

		dg.accumulator.va = VA_TBLOCK(GET_TBLOCK(1));		
		dg.accumulator.pa = dma_map_single(
			DG->dev, dg.accumulator.va, FULL_LEN, DMA_TO_DEVICE);
		dg.accumulator.len = FULL_LEN;


		dbg(1, "acc %p 0x%08x  fifo_incoming %p %08x",
			dg.accumulator.va, dg.accumulator.pa,
			dg.fifo_incoming.va, dg.fifo_incoming.pa);
	}
}

static void freeElementsTable(void)
{
	kfree(dg.elements);
	dg.elements = 0;
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


static void make_chain(struct list_head* head)
{
	struct DescriptorListElement* cursor;

	list_for_each_entry(cursor, head, list){
		if (cursor->list.next != head){
			struct DescriptorListElement* next = 
				list_entry(cursor->list.next,
					   struct DescriptorListElement,list);

			cursor->descriptor->NDA = next->descriptor->pa;
		}
	}

	if (ACCUMULATE){
		/* break the chain. SW will start twice */
		struct DescriptorListElement *head2;
		Descriptor *desc = 
			list_entry(head->next, struct DescriptorListElement, 
							    list)->descriptor;
		desc->NDA = 0;
		desc->DC |= IOP321_DCR_IE;

		head2 = list_entry(head->next->next, 
				   struct DescriptorListElement, list);

		if (!head2){
			err("no second descriptor");
		}else{
			head2->descriptor->DC |= IOP321_DCR_IE;
		}
	}
}

static void build_lists(void)
/** build array of DMA chains a lists. */
{
	int entry;
	int idest;
	struct Destination* source = &dg.destinations[0];

	if (ACCUMULATE){
		init_accumulate_globals();
	}
	for (entry = 0; entry != dg.entries_per_cycle; ++entry){
		struct list_head* head = &dg.elements[entry];
		struct DescriptorListElement* first = poolAlloc();

		first->descriptor = source->build(source, entry);
		INIT_LIST_HEAD(head);
		list_add_tail(&first->list, head);

		for (idest = 1; idest != MAXDEST; ++idest){
			struct Destination* dest = &dg.destinations[idest];
			Descriptor* desc = dest->build(dest, entry);
		
			if (desc){
				struct DescriptorListElement* next = 
					poolAlloc();
				
				next->descriptor = desc;
				list_add_tail(&next->list, head);
				dbg(1, "descriptor %d %d", entry, idest);
			}
		}
		make_chain(head);
	}

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
	u32 stat;

	if (!DMA_DONE((*channel), stat)){
		dbg(1, "!DMA_DONE %d 0x%08x", channel->id, stat);
		return 1;
	}else{
		DMA_ARM_DIRECT((*channel), first->descriptor);
		DMA_STA(*channel) = DMA_EOT;
		return 0;
	}
}


#define AIFIFO_NOT_EMPTY(fifstat)     (((fifstat)&ACQ196_FIFSTAT_HOT_NE) != 0)
#define AIFIFO_OVERTH(fifstat)        (((fifstat)&0xf) >= dg.fifo_th)

static inline int yielder(void)
{
	if (dg.yield){
		yield();
	}
	return 1;
}
#define YIELDER yielder()

#define ALL_DMACSTA_INTS  0x0000000f   /** @@hack */

static NZE arm(void)
/** starts the capture. */
{
	disable_acq();

	deactivateSignal(CAPDEF->ev[0]);
	deactivateSignal(CAPDEF->ev[1]);

	*ACQ200_ICR = 0;              /* polling only */
	disable_fifo();
	reset_fifo();
	set_fifo_ne_mask(CHANNEL_MASK);
	enable_fifo(CHANNEL_MASK);

	dg.imask |= ALL_DMACSTA_INTS;
	llc_intsDisable();

	dg.trig_mask = 1<<CAPDEF->trig->DIx;
	dg.trig_quit = CAPDEF->trig->rising? 0: dg.trig_mask;

	if (ESTOP_DIx){
		dg.estop_mask = 1<<ESTOP_DIx;
		dg.estop_quit = ESTOP_DIx_HIGH? dg.estop_mask: 0;
	}else{
		dg.estop_mask = 0;
	}
	activateSignal(CAPDEF->trig);
	enable_acq();

	if (dg.soft_trigger){
		acq196_syscon_set_all(ACQ196_SYSCON_SOFTTRIG);
		acq196_syscon_clr_all(ACQ196_SYSCON_SOFTTRIG);
	}
	return 0;
}

static inline NZE waitForTrigger(void) 
/** wait for trigger. */
{
	while(YIELDER){
		u32 dix = *ACQ200_DIOCON;
		if ((dix&dg.estop_mask) == dg.estop_quit){
		        ds.stop_reason = "ESTOP before trigger";
			dbg(1, "estop");
			return 1;
		}		
		if ((*ACQ196_SYSCON_ADC&ACQ196_SYSCON_TRIGGERED) != 0){
			return 0;
		}else if (dg.please_stop){
			return 1;
		}
	}
	return -1;
}

static inline NZE shouldStop(void)
{
	u32 dix = *ACQ200_DIOCON;

	if ((dix&dg.estop_mask) == dg.estop_quit){
		ds.stop_reason = "ESTOP";
		dbg(1, "estop");
		return 1;
	}else if ((dix&dg.trig_mask) == dg.trig_quit){
		ds.stop_reason = "TRIG_QUIT";
		dbg(1, "trig_quit");
		return 1;
	}else if (dg.please_stop){
		ds.stop_reason = "please_stop";
		dbg(1, "please_stop");
		return 1;
	}else if (dg.max_cycles != 0 && ds.cycle >= dg.max_cycles){
		ds.stop_reason = "reached cycle count";
		dbg(1, "reached cycle count");
		return 1;
	}else{
		return 0;
	}
}

static inline NZE waitForData(void)
/** Poll for FIFO_NOT_EMPTY
 *  @@todo this is currently a little more conservative than really necessary
 *         however it does avoid underrun errors
 */
{
	while(YIELDER){
		u32 fifstat = *ACQ196_FIFSTAT;

		ds.data_pollcat++;

		if (fifstat&dg.FIFERR){
			dbg(1, "FIFERR 0x%08x & 0x%08x => 0x%08x",
			    fifstat, dg.FIFERR, fifstat & dg.FIFERR);
			return -1;
		}else if (AIFIFO_NOT_EMPTY(fifstat) && AIFIFO_OVERTH(fifstat)){
			return 0;
		}else if (shouldStop()){
			return 1;
		}		
	}
	return -1;
}



static inline NZE waitForEOT(struct DmaChannel* channel)
/** waits for DMA transfer done
 *  we actually wait for the first leg (FPGA->memory) only
 */
{
	u32 dma_stat;
	do {
		dma_stat = DMA_STA(*channel);
		++ds.dma_pollcat;
		if (dma_stat&IOP321_CSR_ERR){
			return -1;
		}else if (shouldStop()){
			return 1;
		}
	} while((dma_stat&DMA_EOT) == 0);

	DMA_STA(*channel) = dma_stat;
	return 0;       
}

static void stop(void)
/** stop the capture. */
{
	llc_intsEnable();
	reactivateSignal(CAPDEF->ev[0]);
	reactivateSignal(CAPDEF->ev[1]);
}



static void run_shot(void)
/** runs the capture.
 */
{
	int rc = 0;
#define EXIT_LOOP(n)  do {rc = n; goto loop_done; } while(0)
	DEFINE_DMA_CHANNEL(c0, 0);
	DEFINE_DMA_CHANNEL(c1, 1);

	struct DmaChannel* channels[] = { &c0, &c1 };
	int entry;
	int triggered = 0;
	struct DmaChannel* active_channel;

	memset(&ds, 0, sizeof(ds));
	memset(&DG->stats, 0, sizeof(DG->stats));

	for (ds.cycle = 0; ; ds.cycle++){
		for (entry = 0; entry < dg.entries_per_cycle; ++entry){
			ds.entry = entry;
			active_channel = channels[entry&1];
			if (dmaTee(active_channel, &dg.elements[entry])){
				EXIT_LOOP(1);
			}

			if (!triggered){
				arm();
				SET_STATE(ST_ARM);
				if (waitForTrigger()){
					EXIT_LOOP(2);
				}else{
					SET_STATE(ST_RUN);
					triggered = 1;
				}
			}else{
				if (shouldStop()){
					EXIT_LOOP(ST_IDLE);
				}
			}
			if (waitForData()){
				EXIT_LOOP(4);
			}

			DG->stats.hot_fifo_histo[*ACQ196_FIFSTAT&0x0f]++;

			if (DMA_STA(*active_channel)&DMA_EOT){
				err("DMA_STA not clear 0x%08x", 
				    DMA_STA(*active_channel));
			}
			++ds.dma_arms;
			DMA_FIRE(*active_channel);

			if (waitForEOT(active_channel)){
				EXIT_LOOP(5);
			}

			DG->stats.hot_fifo_histo2[*ACQ196_FIFSTAT&0x0f]++;
		}
	}
	
	loop_done:
	stop();
	SET_STATE(-rc);
}



static int init_accumulate(void)
{
	if (dg.accumulator.va != 0){
		memset(dg.accumulator.va, 0, FULL_LEN);
		return 0;
	}else{
		return 1;
	}
}

static void run_shot_accumulate(void)
/** runs the capture.
 */
{
	int rc = 0;
#define EXIT_LOOP(n)  do {rc = n; goto loop_done; } while(0)
	DEFINE_DMA_CHANNEL(c0, 0);
	DEFINE_DMA_CHANNEL(c1, 1);

	struct DmaChannel* channels[] = { &c0, &c1 };
	int entry;
	int triggered = 0;
	struct DmaChannel* active_channel;

	memset(&ds, 0, sizeof(ds));
	memset(&DG->stats, 0, sizeof(DG->stats));

	if (init_accumulate()){
		EXIT_LOOP(100);
	}

	for (ds.cycle = 0; ; ds.cycle++){
		for (entry = 0; entry < dg.entries_per_cycle; ++entry){
			ds.entry = entry;
			active_channel = channels[entry&1];
			if (dmaTee(active_channel, &dg.elements[entry])){
				EXIT_LOOP(1);
			}

			if (!triggered){
				arm();
				SET_STATE(ST_ARM);
				if (waitForTrigger()){
					EXIT_LOOP(2);
				}else{
					SET_STATE(ST_RUN);
					triggered = 1;
				}
			}else{
				if (shouldStop()){
					EXIT_LOOP(ST_IDLE);
				}
			}
			if (waitForData()){
				EXIT_LOOP(4);
			}

			DG->stats.hot_fifo_histo[*ACQ196_FIFSTAT&0x0f]++;

			if (DMA_STA(*active_channel)&DMA_EOT){
				err("DMA_STA not clear 0x%08x", 
				    DMA_STA(*active_channel));
			}
			++ds.dma_arms;
			DMA_FIRE(*active_channel);

			if (waitForEOT(active_channel)){
				EXIT_LOOP(5);
			}

			accumulate();

			if (dmaTee(active_channel, dg.elements[entry].next)){
				EXIT_LOOP(6);
			}

			++ds.dma_arms;
			DMA_FIRE(*active_channel);

			if (waitForEOT(active_channel)){
				EXIT_LOOP(7);
			}


			DG->stats.hot_fifo_histo2[*ACQ196_FIFSTAT&0x0f]++;
		}
	}
	
	loop_done:
	stop();
	SET_STATE(-rc);
}

static void init_default(struct device* dev){
	int id;

	for (id = 0; id != MAXDEST; ++id){
		struct Destination* dst = &dg.destinations[id];

		memset(dst, 0, sizeof(struct Destination));

		if (id == ID_SRC){
			dst->base_pa = DG->fpga.fifo.pa;
			dst->interval = 1;
			dst->block_len = 1;
			dst->dest_inc = 1;
			dst->build = buildDescriptorPbi2Mem;
		}else{
			dst->build = buildDescriptorMem2Pci;
		}
	}

	allocElementsTable(dg.entries_per_cycle);
	allocPool();
}


/** sysfs hooks
 */

static ssize_t show_entries_per_cycle(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", dg.entries_per_cycle);
}

static ssize_t store_entries_per_cycle(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int nelems;

	if (sscanf(buf, "%u", &nelems) == 1){
		allocElementsTable(nelems);
	}
	return strlen(buf);
}
static DEVICE_ATTR(epc, S_IRUGO|S_IWUGO,
		   show_entries_per_cycle, store_entries_per_cycle);

static ssize_t show_max_cycles(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%lu\n", dg.max_cycles);
}

static ssize_t store_max_cycles(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%lu", &dg.max_cycles);
	return strlen(buf);
}

static DEVICE_ATTR(max_cycles, S_IRUGO|S_IWUGO,
		   show_max_cycles, store_max_cycles);

static ssize_t show_soft_trig(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%u\n", dg.soft_trigger);
}

static ssize_t store_soft_trig(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int unsigned enable;
	if (sscanf(buf, "%u", &enable) == 1){
		dg.soft_trigger = enable != 0;
	}
	return strlen(buf);
}

static DEVICE_ATTR(soft_trig, S_IRUGO|S_IWUGO,
		   show_soft_trig, store_soft_trig);

static void remove_default(void){
	freePool();
	freeElementsTable();
}

static ssize_t store_build_list(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	build_lists();
	return strlen(buf);
}

static ssize_t show_list(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
/** @todo this is a crude hack to get the info out */
{
	char _buf[80];
	int entry;
	int head;
	struct DescriptorListElement* cursor;

	for (entry = 0; entry < dg.iload; ++entry){
		head = 0;
		list_for_each_entry(cursor, &dg.elements[entry], list){
			info("%03d %c %s", entry, 
			     head==0? 'H': '|',
			     descriptor_to_string(cursor->descriptor, _buf));
			head++;
		}
	}
        return sprintf(buf,"%d\n", DG->sample_read_start);
}


static DEVICE_ATTR(build_list, S_IWUGO|S_IRUGO, show_list, store_build_list);

static ssize_t show_globs(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
#define GLPRINT(field, fmt) \
        sprintf(buf + len, "%20s : " fmt "\n", #field, dg.field)

	len += GLPRINT(entries_per_cycle, "%d");
	len += GLPRINT(pool, "%p");
	len += GLPRINT(ipool, "%d");
	len += GLPRINT(elements, "%p");
	len += GLPRINT(iload, "%d");
	len += GLPRINT(please_stop, "%d");
#undef GLPRINT
        return len;
}

static DEVICE_ATTR(globs, S_IRUGO|S_IWUGO, show_globs, 0);

static ssize_t show_stats(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
#define GLPRINT(field, fmt) \
        sprintf(buf + len, "%20s : " fmt "\n", #field, ds.field)

	len += GLPRINT(state, "%d");
	len += GLPRINT(cycle, "%ld");
	len += GLPRINT(entry, "%d");
	len += GLPRINT(dma_arms, "%d");
	len += GLPRINT(data_pollcat, "%d");
	if (ds.dma_arms){
		len += sprintf(buf + len, "%20s : %d\n", "data_polls/sample", 
		       ds.data_pollcat/ds.dma_arms);
	}
	len += GLPRINT(dma_pollcat, "%d");
	if (ds.dma_arms){
		len += sprintf(buf + len, "%20s : %d\n", "dma_polls/arm", 
		       ds.dma_pollcat/ds.dma_arms);
	}
	if (ds.stop_reason){
		len += GLPRINT(stop_reason, "%s");
	}
	
        return len;
}

static DEVICE_ATTR(stats, S_IRUGO|S_IWUGO, show_stats, 0);




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
	if (ACCUMULATE){
		run_shot_accumulate();
	}else{
		run_shot();
	}
	return strlen(buf);
}
static DEVICE_ATTR(run, S_IRUGO|S_IWUGO,  show_run, store_run);


static ssize_t show_stop(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", dg.please_stop);
}

static ssize_t store_stop(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.please_stop);
	return strlen(buf);
}
static DEVICE_ATTR(stop, S_IRUGO|S_IWUGO,  show_stop, store_stop);


static ssize_t show_FIFERR(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"0x%08x\n", dg.FIFERR);
}

static ssize_t store_FIFERR(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf(buf, "0x%x", &dg.FIFERR) == 0){
		sscanf(buf, "%x", &dg.FIFERR);
	}
	return strlen(buf);
}
static DEVICE_ATTR(FIFERR, S_IRUGO|S_IWUGO,  show_FIFERR, store_FIFERR);


static ssize_t show_yield(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", dg.yield);
}

static ssize_t store_yield(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.yield);
	return strlen(buf);
}
static DEVICE_ATTR(yield, S_IRUGO|S_IWUGO,  show_yield, store_yield);


static ssize_t show_fifo_th(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", dg.fifo_th);
}

static ssize_t store_fifo_th(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.fifo_th);
	return strlen(buf);
}
static DEVICE_ATTR(fifo_th, S_IRUGO|S_IWUGO,  show_fifo_th, store_fifo_th);


static ssize_t set_scatter_mask(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf(buf, "0x%x", &dg.imask) == 0){
		sscanf(buf, "%x", &dg.imask);
	}

	return strlen(buf);
}
static ssize_t show_scatter_mask(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "0x%08x\n", dg.imask);
}

static DEVICE_ATTR(imask, S_IWUGO|S_IRUGO, 
		   show_scatter_mask, set_scatter_mask);


static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq100_scatter_driver_name,
		       acq100_scatter_driver_string,
		       acq100_scatter_driver_version,
		       acq100_scatter_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);



static int mk_scatter_sysfs(struct device *dev)
/** create device hooks: in /dev/scatter (link into sysfs)
 - epc: entries per cycle
 - build_list: W: build the list, R: list the list
 - globs: view global settings
 - stats: statistics for current shot
 - run: W: start a shot
 - stop: W: request stop shot
 - yield: W1 : insert yield() in poll loop for easier diagnostic. Remove for production use.
 - fifo_th : fifo threshold to delay dma start from.
 - imask: set interrupt mask in force during shot (0xffffffff masks all)
*/
{
	int id;

	for (id = 0; id != MAXDEST; ++id){
		DEVICE_CREATE_FILE(dev, &dg.dest_attr[id]);
	}

	DEVICE_CREATE_FILE(dev, &dev_attr_epc);
	DEVICE_CREATE_FILE(dev, &dev_attr_build_list);
	DEVICE_CREATE_FILE(dev, &dev_attr_globs);
	DEVICE_CREATE_FILE(dev, &dev_attr_stats);
	DEVICE_CREATE_FILE(dev, &dev_attr_run);
	DEVICE_CREATE_FILE(dev, &dev_attr_stop);
	DEVICE_CREATE_FILE(dev, &dev_attr_yield);
	DEVICE_CREATE_FILE(dev, &dev_attr_fifo_th);
	DEVICE_CREATE_FILE(dev, &dev_attr_imask);
	DEVICE_CREATE_FILE(dev, &dev_attr_FIFERR);
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_max_cycles);
	DEVICE_CREATE_FILE(dev, &dev_attr_soft_trig);
	return 0;
}


static void acq100_scatter_dev_release(struct device * dev)
{
	info("");
}


static int acq100_scatter_probe(struct device *dev)
{
	info("");
	init_default(dev);
	mk_scatter_sysfs(dev);
	return 0;
}

static int acq100_scatter_remove(struct device *dev)
{
	remove_default();
	return 0;
}


static struct device_driver acq100_scatter_driver = {
	.name     = "acq100_scatter",
	.probe    = acq100_scatter_probe,
	.remove   = acq100_scatter_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq100_scatter_device = {
	.name = "acq100_scatter",
	.id   = 0,
	.dev = {
		.release    = acq100_scatter_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq100_scatter_init( void )
{
	int rc;
	acq200_debug = acq100_scatter_debug;

	rc = driver_register(&acq100_scatter_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acq100_scatter_device);
}


static void __exit
acq100_scatter_exit_module(void)
{
	info("");
	platform_device_unregister(&acq100_scatter_device);
	driver_unregister(&acq100_scatter_driver);
}


module_init(acq100_scatter_init);
module_exit(acq100_scatter_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ100 Scatter Control");



