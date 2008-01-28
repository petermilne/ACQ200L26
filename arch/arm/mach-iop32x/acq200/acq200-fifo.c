/* ------------------------------------------------------------------------- */
/* acq200-fifo.c driver for acq200 fifo streaming                            */
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

#include "acq200_minors.h"


#include <linux/poll.h>

#include "acq200-rb.h"

/** @file acq200-fifo.c
 *
 * Global Data Structure Policy
 *
 * Module Load / Unload
 * => allocate minimum resources on module load
 * Device Probe / Remove.
 * => allocate all resources on probe
 *
 * File Open / Close
 * => init all data structures on open.
 *
 * Initial FIFO latency is CRITICAL. 
 * Plan ch0: has one DBLK teed up ready to go at all times
 *
 * FIFO TIDE => FIQ => reads FIFCON, fires CH0, schedules 
 * chain fill on CH1. 
 * Chain Fill process rearms interrupt and reloads CH0.
 *
 * FIFO streaming operates using a "bottling plant" paradigym
 *
 * We Q endstops for unique ident of last data transferred.
 */

#include <asm/arch/iop321-dma.h>
#include <asm/arch/iop321-irqs.h>
#include <asm/arch/acqX00-irq.h>

#include "acqX00-port.h"
#include "acq200-fifo-top.h"
#include "acq200-fifo-tblock.h"
#include "acq200-stream-api.h"

#include <linux/dma-mapping.h>
#include <linux/kdev_t.h>


#include <asm-arm/arch-iop32x/acq200.h>

#include <linux/moduleparam.h>
#include <linux/workqueue.h>


#define VERID \
"$Id: acq200-fifo.c,v 1.38 2006/10/04 09:07:43 pgm Exp $ Build 1201 " \
__DATE__ " "__TIME__ MODEL_VERID


#define XMARK dbg(1, "xm:%d", __LINE__)

char *acq200_fifo_verid(void) { return VERID; }

char* verid = VERID;
module_param(verid, charp, 0444);


#define CFG_LOW_JITTER_BURST_START 1

#define EMPTY_FILL_THRESHOLD  (DG->empty_fill_threshold)
#define PUT_MAX_EMPTIES       (DG->put_max_empties)
#define GET_MAX_ACTIVE        (DG->get_max_active)
#define ACTIVE_BATCH_THRESHOLD (DG->active_batch_threshold)
#define INIT_ENDSTOPS          (DG->init_endstops)
#define EOC_INT_MODULO_MASK    (DG->eoc_int_modulo_mask)

#ifdef WAV232
#define INIT_TEE_EMPTIES (RBLEN-1024)
#else
#define INIT_TEE_EMPTIES (EMPTY_FILL_THRESHOLD*2)
#endif

extern void acq200_set_user_led(int led4, int on);


int acq200_timeout = 10000;

int acq200_fifo_debug = 0;
module_param(acq200_fifo_debug, int, 0644);


int acq200_dmad_debug = 0;
module_param(acq200_dmad_debug, int, 0644);

#define ACQ200_DMAD_DEBUG (acq200_dmad_debug==0? 16: 0)

int acq200_clk_hz = 1000000;
module_param(acq200_clk_hz, int, 0644);

int fastforward = 0;
module_param(fastforward, int, 0644);

int disable_acq_debug = 0;         /* set 1 */
module_param(disable_acq_debug, int, 0644);

int bda_debug = 2;
module_param(bda_debug, int, 0644);

#ifdef ACQ216
/** repeated transient mode. */
int live_one_frame_per_dcb;
module_param(live_one_frame_per_dcb, int, 0644);
#endif

int transient_dma_blocklimit;
module_param(transient_dma_blocklimit, int, 0644);

int uses_ST_CAPDONE = 1;
module_param(uses_ST_CAPDONE, int, 0644);

#ifndef AICHAN_DEFAULT
#define AICHAN_DEFAULT 0
#endif
int acq200_aichan = AICHAN_DEFAULT;
module_param(acq200_aichan, int, 0644);

int acq200_data_word_size = 0;
module_param(acq200_data_word_size, int, 0644);

int blt_dma_using_interrupt = 0;
module_param(blt_dma_using_interrupt, int, 0644);


/** pgm 20050513 - desperate diags */
int init_dmac_count = 0;
module_param(init_dmac_count, int, 0644);

int es_debug = 0;
module_param(es_debug, int, 0600);

#define DMA_REG(base, boffset) *(volatile u32*)((char*)(base)+(boffset))
#define DMA_ERROR IOP321_CSR_ERR


static char errbuf[80];

static void preEnable(void);   /* call immediately BEFORE trigger en */
static void onEnable(void);    /* call immediately AFTER trigger     */
static int try_fiq(void);
static void initPhaseDiagBuf(void);
static void schedule_dmc0_timeout(unsigned long arg);
static void increment_scc(struct SampleClockCounter *scc);


static struct IPC* IPC;


/*
 * ko Global Data  - exported at module end
 */
struct DMC_WORK_ORDER *DMC_WO;
struct CAPDEF *CAPDEF = (struct CAPDEF*)0xdeadbeef; 
/* MUST BE INITIALIZED BY DEV */
struct DevGlobs* DG; 

int use_endstop = 0;


#include <linux/kthread.h>

static struct DMC0 {
	struct task_struct* task;
	unsigned long v;
	wait_queue_head_t waitq;
} dmc0;

#define DMC0_RUN_REQUEST 0
#define DMC_TO	(HZ)


static void wake_dmc0(void)
{
	dbg(2, "01");
	set_bit(DMC0_RUN_REQUEST, &dmc0.v);
	wake_up_interruptible(&dmc0.waitq);
	dbg(2, "99");
}


/*
 * acq200-fifoi.h - defs private to acq200-fifo.c
 */


#include "acq200-pulse.c"

static void onTrigger(void);


static void acq200_global_mask_op(u32 mask, int maskon)
{
	int irq;

	if (maskon){
		dbg(2, "0x%08x %s", mask, "ON" );
	}

	for (irq = 0; mask != 0; mask >>= 1, irq++ ){
		if ((mask&1) != 0){
			if (maskon){
				acq200_mask_irq(irq);
			}else{
				acq200_unmask_irq(irq);
			}
		}
	}

	if (!maskon){
		dbg(2, "0x%08x %s", mask, "OFF" );
	}
}




#define HITIDE (DG->hitide)
#define LOTIDE (DG->lotide)




static void set_fpga_isr_steering(int use_fiq)
{
	volatile u32 intstr;

	asm volatile( "mrc p6, 0, %0, c4, c0, 0" : "=r" (intstr) );
	if ( use_fiq ){
		intstr |= 1<<FPGA_INT;
	}else{
		intstr &= ~(1<<FPGA_INT);
	}
	asm volatile( "mcr p6, 0, %0, c4, c0, 0" : : "r" (intstr) );
}


#define FIQ_SPACE    (0x0200-0x001c)

extern int acq200_pipe_fiq_isr_adds_endstop(void);
extern int acq200_pipe_fiq_isr_version(void);

int set_fpga_isr_use_fiq(int use_fiq)
{
	if (use_fiq && PIPE_FIQ_LEN > FIQ_SPACE ){
		info("FIQ VERSION 0x%08x", acq200_pipe_fiq_isr_version());
		info("FIQ LENGTH  0x%x", PIPE_FIQ_LEN);
		info("SLOT SIZE   0x%x", FIQ_SPACE);
		err("UNABLE to USE FIQ - too big for slot");
		use_fiq = 0;
	}

	set_fpga_isr_steering(0);
	if (use_fiq){
		set_fiq_handler(acq200_pipe_fiq, PIPE_FIQ_LEN);
		info("FIQ VERSION 0x%08x", acq200_pipe_fiq_isr_version());
		info("PIPE_FIQ_LEN 0x%x LOADED", PIPE_FIQ_LEN);
		info("SLOT SIZE   0x%x", FIQ_SPACE);
		info("you have %d instructions spare!",
		     (FIQ_SPACE-PIPE_FIQ_LEN)/4);
		asm( "bkpt #1" );     /* alert BDI2000 */

		use_endstop = !acq200_pipe_fiq_isr_adds_endstop();
		
		if (use_endstop){
			info( "provide precooked endstop for FIQ" );
		}
	}else{
		use_endstop = 0;
	}
	return use_fiq;
}


#define FIFO_ERROR -1
#define IPC_STALL  -2

void acq200_error_handler( unsigned long arg )
{
	switch(arg){
	case FIFO_ERROR:
		sprintf(errbuf, "%s FIFO_ERROR", FN);
		break;
	case IPC_STALL:
		sprintf(errbuf, "%s IPC_STALL", FN);
		break;
	default:
		sprintf(errbuf, "%s UNKNOWN %ld", FN, arg);
		break;
	}

	finish_with_engines(-__LINE__);
	DMC_WO->error = errbuf;
	wake_up_interruptible(&IPC->finished_waitq);
}

DECLARE_TASKLET(acq200_fifo_error_tasklet,acq200_error_handler, FIFO_ERROR);
DECLARE_TASKLET(acq200_ipc_error_tasklet,acq200_error_handler, IPC_STALL);

__attribute__ ((format (printf, 1, 2)))
static char* _acq200_printk (const char *fmt, ...)
{
	static char buf[128];
	va_list args;

	va_start (args, fmt);
	vsnprintf (buf, sizeof (buf), fmt, args);
	va_end (args);

	return buf;
}

static char* dmad_diag(struct iop321_dma_desc *d)
{
	return _acq200_printk( 
		"%p %08x %08x %08x %08x %08x",
		d, d->NDA, d->PDA, d->LAD, d->BC, d->DC);
}


static void init_dmac(void)
{
	++init_dmac_count;

	*IOP321_DMA0_CCR = 0;
	*IOP321_DMA1_CCR = 0;
	*IOP321_DMA0_NDAR = 0;
	*IOP321_DMA1_NDAR = 0;

	*IOP321_DMA0_CSR = 0xffffffff;
	*IOP321_DMA1_CSR = 0xffffffff;
}



static inline struct iop321_dma_desc *tee_isr_head(void) 
{
	return DG->head = rb_get_buf(&IPC->empties);
}




static void enable_interrupt_hook(struct InterruptSync *is)
{
	if (!is->enabled){
		enable_irq(is->irq);
		acq200_unmask_irq(is->irq);
		is->enabled = 1;
	}
}


static void unmask_eoc(void)
{
	unsigned long flags;
	struct InterruptSync* is = &IPC->is_dma[0].eoc;
	spin_lock_irqsave(&is->bh_lock, flags);
	enable_interrupt_hook(is);
	spin_unlock_irqrestore(&is->bh_lock, flags);
}


static void call_end_of_shot_hooks(void)
{
	struct list_head *cursor;
	struct Hookup *hookup;

	list_for_each(cursor, &DG->end_of_shot_hooks){
		hookup = list_entry(cursor, struct Hookup, list);
		hookup->the_hook(hookup->clidata);
	}
}


void finish_with_engines(int ifinish)
{
	if (!DG->finished_with_engines){
		stop_capture();
		call_end_of_shot_hooks();
		schedule_dmc0_timeout(0);
		if (DG->shot == 0 || ifinish < 0){
			DMC_WO_setState(ST_STOP);
		}else{
/** ST_CAPDONE is an attempt to avoid end of shot status race. */
			DMC_WO_setState(uses_ST_CAPDONE? ST_CAPDONE: ST_STOP);
		}
		iop321_stop_ppmu();


		DG->stats.end_gtsr = *IOP321_GTSR;
		DG->stats.end_jiffies = jiffies;
		DG->finished_with_engines = ifinish;
		DG->stats.finish_time = CURRENT_TIME;

		if (DG->global_irq_mask){
			acq200_global_mask_op(DG->global_irq_mask, 0);
		}	

		/*
                 * ensure eoc available for next time
		 */
		if (DG->bh_unmasks_eoc){
			unmask_eoc();
		}
		wake_up_interruptible(&IPC->finished_waitq);
	}
}	

#ifndef CHECK_FIFSTAT
#define CHECK_FIFSTAT(wo, fifstat, offset) 0
#endif


static void addPitStore(struct DMC_WORK_ORDER *wo, u32 offset, u32 status)
{
	if (wo->pit_count < DG->pit_store.max_pits){
		struct PIT_DEF *def = 
			&DG->pit_store.the_pits[wo->pit_count++];

		def->offset = offset;
		def->status = status;
	}	
}


static inline struct Phase * onPIT(struct Phase *phase, u32 status, u32 offset)
{
	return phase;
}

void onEvent(struct DMC_WORK_ORDER *wo, u32 status, unsigned *offset)
/* handles trigger, possibly updating offset */
{
	addPitStore(wo, *offset, status);

	wo->now = wo->now->onPIT(wo->now, status, offset);

	if (wo->pit_count == DMC_WO->pit_stop){
		wo->oneshot = 1;
	}
}


static inline void sccInit(
	struct SampleClockCounter *scc,
	int scc_per_block,
	int leap_clock_period
	)
{
	memset(scc, 0, sizeof(struct SampleClockCounter));
	scc->scc_per_block = scc_per_block;
	scc->scc_leap_clock_period = leap_clock_period;
}


static int woOnRefill(
	struct DMC_WORK_ORDER *wo, u32* fifstat, unsigned* offset )
/* returns 1 on phase change or END_OF_PHASE */
{
#define END_OF_PHASE 1
	struct Phase* phase = wo->now;

	dbg(3, "wo %p phase %p act %d demand %d",
	    wo, phase, 
	    phase? phase->actual_len:0, 
	    phase? phase->demand_len:0);

	if (!phase){
		return END_OF_PHASE;
	}
	if (phase->actual_len == 0){
		phase->start_off = *offset;
		dbg(1, "%d set phase->start_off phase:%s start_off %d",
		    __LINE__, phase->name, phase->start_off);
	}
	if (phase->actual_len < phase->demand_len+DMA_BLOCK_LEN){
		phase->actual_len += DMA_BLOCK_LEN;
	}else{
		/** event enabled after first lap */
		if (phase->lap_count == 0){
			phase->lap_count = 1;
			phase->lap_position = 0;
			if (phase->ev){
				activateSignal(phase->ev);
			}
		}
		phase->lap_position += DMA_BLOCK_LEN;
		if (phase->lap_position >= phase->actual_len){
			phase->lap_count++;
			phase->lap_position = 0;
		}
	}
	phase->end_off = *offset + DMA_BLOCK_LEN;

	if (*fifstat){
		*fifstat = CHECK_FIFSTAT(wo, *fifstat, offset);
	}

	
	if (phase == wo->now){
		if (phase_end(phase)){
			struct Phase *np = NEXT_PHASE(phase);
			if (np){
				wo->now = NEXT_PHASE(phase);
			}else{
				return END_OF_PHASE;
			}
		}else{
			; /* no action */
		}
	}else{
		wo->now->event_count++;
	}
	return phase != wo->now;         /* event is in NEW Phase */
}

#ifndef WAV232

static void _dmc_handle_tb_clients(struct TBLOCK *tblock)
{
	struct list_head* clients = &DG->tbc.clients;
	struct list_head* pool = &DG->bigbuf.pool_tblocks;
	spin_lock(&DG->tbc.lock);

	if (!list_empty(clients)){
		struct TblockConsumer *tbc;

		list_for_each_entry(tbc, clients, list){
			TBLE* tle = TBLE_LIST_ENTRY(pool->next);

			atomic_inc(&tblock->in_phase);
			tle->tblock = tblock;

			list_move_tail(pool->next, &tbc->tle_q);
			wake_up_interruptible(&tbc->waitq);	
		}
	}
	spin_unlock(&DG->tbc.lock);
}


static struct TblockListElement* dmc_phase_add_tblock(
	struct Phase* phase, unsigned* start_off
	)
/* append new tblock to phase. If phase is big enough, recycle one from tail */
{
	struct BIGBUF *bb = &DG->bigbuf;
	struct list_head* empty_tblocks = &bb->empty_tblocks; 

	dbg(2, "");

	if (unlikely(list_empty(empty_tblocks))){
		finish_with_engines(-__LINE__);
		return 0;				
	}else{
		struct TblockListElement* tle =	
			TBLE_LIST_ENTRY(empty_tblocks->next);
		unsigned long flags;

		if (atomic_read(&tle->tblock->in_phase)){	
			err("in_phase != 0");
			finish_with_engines(-__LINE__);
			return 0;			
		}

		atomic_inc(&tle->tblock->in_phase);
		if (!list_empty(&phase->tblocks)){
			/* pass old [full] tblock to clients */
			_dmc_handle_tb_clients(
				TBLE_LIST_ENTRY(phase->tblocks.prev)->tblock);
		}
		spin_lock_irqsave(&bb->tb_list_lock, flags);
		list_move_tail(empty_tblocks->next, &phase->tblocks);
		spin_unlock_irqrestore(&bb->tb_list_lock, flags);

		*start_off = tle->tblock->offset;

		if (phase->tblock_count >= phase->tblock_max_count){
			acq200_phase_release_tblock_entry(
				TBLE_LIST_ENTRY(phase->tblocks.next));
		}else{
			++phase->tblock_count;
		}
		return tle;
	}
}



static void share_tblock(
	struct Phase* np,
	struct TblockListElement* src,
	struct TblockListElement* dst )
{
	struct BIGBUF *bb = &DG->bigbuf;
	struct list_head* pool_tblocks = &bb->pool_tblocks;
	unsigned long flags;

	dst->tblock = src->tblock;
	atomic_inc(&src->tblock->in_phase);

	spin_lock_irqsave(&bb->tb_list_lock, flags);
	list_move_tail(pool_tblocks->next, &np->tblocks);
	spin_unlock_irqrestore(&bb->tb_list_lock, flags);
}

static void share_last_tblock(struct Phase* np, struct Phase* op)
/*
 * take a TBLE wrapper from the pool, share last tblock from old Phase,
 * add to new phase, incrementing usage count
 */
{
	struct list_head* pool_tblocks = &DG->bigbuf.pool_tblocks;
	struct TblockListElement* src;
	struct TblockListElement* dst;

	dbg(1, "np %p %s op %p %s", np, np->name, op, op->name);
	if (unlikely(list_empty(&op->tblocks))){
		info("op %s list empty, drop out", op->name);
		return;
	}
	if (unlikely(list_empty(pool_tblocks))){
		finish_with_engines(-__LINE__);
		return;
	}

	src = list_entry(op->tblocks.prev, struct TblockListElement, list);
	dst = list_entry(pool_tblocks->next, struct TblockListElement, list);

	dbg(1, "src %p dst %p", src, dst);
	share_tblock(np, src, dst);

}
#else
static void share_tblock(
	struct Phase* np,
	struct TblockListElement* src,
		struct TblockListElement* dst ) {}
#define dmc_phase_add_tblock(phase, offp)   1
#define share_last_tblock(np, op) 
#endif

#ifndef WAV232

static void _dmc_handle_dcb_inst(
	struct DataConsumerBuffer *dcb,
	u32 fifstat,
	u32 offset)
{

	if (dcb->flow_control_active){
		if (u32rb_is_empty(&dcb->rb)){
			dcb->flow_control_active = 0;			
		}else{
			DG->stats.dcb_flow_control_throttled_count++;
			wake_up_interruptible(&dcb->waitq);
			return;
		}
	}
	if (dcb->rb.nput - dcb->rb.nget > DG->dcb.dcb_max_backlog){
		dcb->flow_control_active = 1;
		DG->stats.dcb_flow_control_throttled_count++;
	}else{
		if (fifstat){
			if (!u32rb_put(&dcb->rb, fifstat)){
				DG->stats.dcb_flow_control_event_discards++;
			}
		}
		if (u32rb_put(&dcb->rb, offset)){
			wake_up_interruptible(&dcb->waitq);
		}else{
			DG->stats.dcb_flow_control_discards++;
		}
	}	
}

static void _dmc_handle_dcb(
	struct iop321_dma_desc *pbuf, u32 offset
	)
{
	struct list_head* clients = &DG->dcb.clients;


	dbg(2, "clients %p", clients);

	spin_lock(&DG->dcb.lock);

	
	if (!list_empty(clients)){
		struct DataConsumerBuffer *dcb;
		int ic = 0;

		dbg(2, "OK, now for the clients");

		list_for_each_entry(dcb, clients, list) {
			dbg(3, "client %d dcb %p\n", ic++, dcb);

			_dmc_handle_dcb_inst(dcb, pbuf->DD_FIFSTAT, offset);
		}		
	}
	pbuf->DD_FIFSTAT = 0;

	spin_unlock(&DG->dcb.lock);
}



static void dmc_handle_prep(
	struct Phase *prep,
	struct Phase *phase,
	struct iop321_dma_desc *pbuf, 
	u32 offset)
{
	if (DMC_WO->scc.scc >= prep->start_after){
		unsigned end_prep = prep->start_after + prep->demand_samples;

		/* if not already there, add tblock to phase */

		if (list_empty(&prep->tblocks) ||
                    TBLE_LIST_ENTRY(phase->tblocks.prev)->tblock !=
		    TBLE_LIST_ENTRY(prep->tblocks.prev)->tblock     ){
			share_last_tblock(prep, phase);
			prep->ref_start_scc = DMC_WO->scc.scc;
			prep->ref_offset = offset;
			do_gettimeofday(&prep->prep_start_time);
		}

		/* check for end of prep */

		if (DMC_WO->scc.scc > end_prep){
			struct Phase *np = PHASE_LIST_ENTRY(prep->list.next);

			if ((void*)np != &DMC_WO->prep_phases){
				DMC_WO->prep_now = np;
			}else{
				DMC_WO->prep_now = 0;
			}

			if (prep->onPhaseComplete){
				prep->onPhaseComplete(prep);
			}
		}
	}
}
#endif

#ifndef WAV232	
#define BDA_NZO 0x1BDA0         /* never supply zero offset */
#define BDA_FIN 0x2BDA0         /* offset flags completion  */
static struct TblockListElement* bbb;  /* bit bucket block */
#endif


#ifdef ACQ216
static void poll_dma_done(void);
#endif

#define IS_MFA(p) (((unsigned)p & 0xff000000) == 0)

static void _dmc_handle_refills(struct DMC_WORK_ORDER *wo)
{
	struct iop321_dma_desc *pbuf;
	int nrefills = 0;
	u32 offset;
	int end_of_phase = 0;
	struct Phase* phase;
	int bda_fin = 0;

	assert(wo);/** @todo - try to trap a null pointer exception */
	dbg(2, "wo %p", wo);

	while(rb_get(&IPC->active, &pbuf)){
		/** @todo - try to trap a null pointer exception */
		assert(pbuf);

		if (pbuf->clidat){
			
			/** return pbc to endstops, passing fifo_to_local
                         *  to regular downstream processing.
			 */
			struct PrebuiltChain* pbc = 
				(struct PrebuiltChain*)pbuf->clidat;

			if (IS_MFA(pbc)){
				/* this is a gross error. log it */
				err("MFA in clidat 0x%08x", (unsigned)pbc);
				goto no_clidat;
			}


#ifdef ACQ216
			/** do not interfere with DMAD in flight! 
			 *  this theory is DODGY because this code tinks with 
                         *  pbc, NOT dmad
			 */
			if (RB_IS_EMPTY(IPC->active)){
				poll_dma_done();
			}
#endif
			pbuf = pbc->the_chain[pbc->fifo_to_local];
			pbc->the_chain[pbc->fifo_to_local] = 0;
			rb_put(&IPC->endstops, &pbc->desc);
		}
	no_clidat:
		phase = wo->now;
		offset = pbuf->LAD - wo->pa;

#ifndef WAV232
		increment_scc(&wo->scc);

		if (bbb && TBLOCK_INDEX(offset) == bbb->tblock->iblock){
			if (TBLOCK_OFFSET(offset) == BDA_FIN){
				end_of_phase = 1;
				bda_fin = 1;
			}
		}else
#endif			
		{
			if (TBLOCK_OFFSET(offset) == 0 && 
			    dmc_phase_add_tblock(phase, &offset) == 0){
				break;
			}
			end_of_phase = woOnRefill(wo, 
						  &pbuf->DD_FIFSTAT, &offset);

			dbg( 3, "free %s", acq200_debug>3?dmad_diag(pbuf): "");
			++DG->stats.refill_blocks;
			++nrefills;
#ifndef WAV232	
			if (wo->prep_now){
				dmc_handle_prep(
					wo->prep_now, phase, pbuf, offset);
			}
			_dmc_handle_dcb(pbuf, offset);
#endif
		}

		acq200_dmad_free(pbuf);
		if (end_of_phase){
			dbg(1, "phase_end d:%d l:%d", 
			    phase->demand_len, phase_len(phase));

			if (bda_fin){
				wo->finished_code = 1;
				dbg( 1,"BDA_FIN - shut down and wake caller");
				finish_with_engines(__LINE__);
				break;
			}else if (wo->now != 0 && !phase_end(wo->now)){
				share_last_tblock(wo->now, phase);
			}else{
				wo->finished_code = 1;
				dbg( 1,"all done - shut down and wake caller");
				finish_with_engines(__LINE__);
				break;
			}
		}

		if (nrefills >= GET_MAX_ACTIVE){
			break;
		}
	}
}

static void dmc_handle_refills(struct DMC_WORK_ORDER *wo)
{
#if 0
	spin_lock(&DG->dmc_handle_refills_lock);
	if (DG->dmc_handle_refills_busy == SPIN_LOCK_UNLOCKED){
		spin_lock(&DG->dmc_handle_refills_busy);
	}else{
		info("wa hey - dmc goes re-entrant!");
		spin_unlock(&DG->dmc_handle_refills_lock);
		return;
	}
#endif
	_dmc_handle_refills(wo);
#if 0
	spin_lock(&DG->dmc_handle_refills_lock);
	spin_unlock(&DG->dmc_handle_refills_busy);
	spin_unlock(&DG->dmc_handle_refills_lock);
#endif
}

#ifndef WAV232



unsigned default_getNextEmpty(struct DMC_WORK_ORDER* wo)
{
	unsigned this_empty = wo->next_empty;

	if (unlikely(this_empty == 0)){
		struct list_head* free_blocks = &DG->bigbuf.free_tblocks;
		struct list_head* empty_tblocks = &DG->bigbuf.empty_tblocks; 

		if (unlikely(list_empty(free_blocks))){
/** @@todo - this isn't necessarily the EOTW should drop out and try later */
//			finish_with_engines(-__LINE__);
			return ~1;
		}else{
			struct BIGBUF *bb = &DG->bigbuf;
			struct TblockListElement* tble = 
				TBLE_LIST_ENTRY(free_blocks->next);
			unsigned long flags;

			spin_lock_irqsave(&bb->tb_list_lock, flags);
			list_move_tail(free_blocks->next, empty_tblocks);
			spin_unlock_irqrestore(&bb->tb_list_lock, flags);

			this_empty = tble->tblock->offset;
			wo->next_empty = this_empty + DMA_BLOCK_LEN;
		}
	}else{
		unsigned next_empty = this_empty + DMA_BLOCK_LEN;

		if (TBLOCK_OFFSET(next_empty) == 0){
			next_empty = 0;       /* force reservation next time */
		}
		wo->next_empty = next_empty;
	}
	return this_empty;
}


#ifdef PEEK_IS_REQUIRED
unsigned peekFirstEmpty(void)
/* return offset for first tblock in capture. DO NO de-queue it! */
{
	struct list_head* free_blocks = &DG->bigbuf.free_tblocks;

	if (unlikely(list_empty(free_blocks))){
		err("NO FREE BLOCKS!");
		return 0;
	}else{
		struct TblockListElement* tble = 
			TBLE_LIST_ENTRY(free_blocks->next);
		return tble->tblock->offset;
	}
}
#endif


unsigned bda_getNextEmpty(struct DMC_WORK_ORDER* wo)
{
	if (!bbb){
		struct BIGBUF *bb = &DG->bigbuf;
		unsigned long flags;

		spin_lock_irqsave(&bb->tb_list_lock, flags);
		bbb = TBLE_LIST_ENTRY(bb->free_tblocks.next);
		list_del(&bbb->list);
		spin_unlock_irqrestore(&bb->tb_list_lock, flags);
	}
	
	switch(wo->bda_blocks.state){
	case BDA_IDLE:
		wo->bda_blocks.state = BDA_BEFORE;
		/* fall thru */
	case BDA_BEFORE:
		if (IPC->empties.nput < wo->bda_blocks.before){
			dbg(bda_debug, "96 BEFORE return 0x%08x", 
					bbb->tblock->offset+BDA_NZO);
			return bbb->tblock->offset + BDA_NZO;
		}
		DG->stats.bda_times.before = jiffies;	
		wo->bda_blocks.state = BDA_DURING;	
		/* fall thru */
	case BDA_DURING:
		if (IPC->empties.nput < wo->bda_blocks.during){
			dbg(bda_debug, "97 DURING");
			return default_getNextEmpty(wo);
		}
		DG->stats.bda_times.during = jiffies;
		wo->bda_blocks.state = BDA_AFTER;
		/* fall thru */
	case BDA_AFTER:
		if (IPC->empties.nput < wo->bda_blocks.after){
			dbg(bda_debug, "98 AFTER return 0x%08x", 
				bbb->tblock->offset+BDA_NZO); 
			return bbb->tblock->offset + BDA_NZO;
		}
		DG->stats.bda_times.after = jiffies;
		wo->bda_blocks.state = BDA_DONE;
		dbg(bda_debug, "99 finished");
		return bbb->tblock->offset + BDA_FIN;
	default:	
	case BDA_DONE:
		/* same as above but with no message */
		return bbb->tblock->offset + BDA_FIN;
	}
}


static void bda_release_tblock(void)
{
	if (bbb){
		struct BIGBUF *bb = &DG->bigbuf;
		unsigned long flags;

		spin_lock_irqsave(&bb->tb_list_lock, flags);
		list_add_tail(&bbb->list, &bb->free_tblocks);
		bbb = 0;
		spin_unlock_irqrestore(&bb->tb_list_lock, flags);
	}
}

#endif



#if FPGA_IS_PCI_DEVICE
#define DMA_DCR_TODEVICE (DMA_DCR_PCI_MW)
#define DMA_DCR_FROMDEVICE (DMA_DCR_PCI_MR)
#else
#define DMA_DCR_MEM2MEM 0x00000040
#define DMA_DCR_TODEVICE (DMA_DCR_MEM2MEM)
#define DMA_DCR_FROMDEVICE (DMA_DCR_MEM2MEM)
#endif

#ifndef WAV232

#ifdef ACQ216
static void dmc_handle_empties_prebuilt(struct DMC_WORK_ORDER *wo)
{
	int nput = 0;
	unsigned empty_offset;

	while( !RB_IS_FULL(IPC->empties) && 
               !RB_IS_EMPTY(IPC->endstops) &&
		(empty_offset = wo->getNextEmpty(wo)) != ~1){

		struct iop321_dma_desc *dmad = acq200_dmad_alloc();
		u32 local_pa = wo->pa + empty_offset;
		struct iop321_dma_desc *endstop;
		struct PrebuiltChain *pbc;


		if (!dmad){
			err("acq200_dmad_alloc() STARVE");
			return;
		}

		dmad->PDA = DG->fpga.fifo.pa;
		dmad->DD_FIFSTAT = 0;
		dmad->LAD  = local_pa;
		dmad->BC = DMA_BLOCK_LEN;
		dmad->DC = DMA_DCR_FROMDEVICE;

		if (!rb_get(&IPC->endstops, &endstop)){
			err("STARVED for endstops");
			finish_with_engines(- __LINE__);
			return;
		}

		pbc = getPBChain(endstop);
		pbc->insert(pbc, dmad);
		pbc->the_chain[0]->DD_FIFSTAT = 0;

		if (nput < 3 || acq200_debug > 5){
			dbg( 3, "rb_put %s", dmad_diag( dmad ) );
		}
		rb_put( &IPC->empties, pbc->the_chain[0]);

		if (++nput >= PUT_MAX_EMPTIES){
			break;
		}
	}

	if (nput){
		dbg( 3,"direction %s rb_put %d", 
		     wo->direction==PCI_DMA_TODEVICE?
		     "outgoing" : "incoming", nput);
	}

	if (RB_IS_EMPTY(IPC->empties)){
		finish_with_engines(-__LINE__);
	}
}
#endif /* ACQ216 */
#endif

static void dmc_handle_empties_default(struct DMC_WORK_ORDER *wo)
{
	/* attempt to map all or remainder of wo into empties */
	int dc = wo->direction == PCI_DMA_TODEVICE?
		DMA_DCR_TODEVICE : DMA_DCR_FROMDEVICE;
	int nput = 0;
	unsigned empty_offset;

	while( !RB_IS_FULL(IPC->empties) &&
		(empty_offset = wo->getNextEmpty(wo)) != ~1){

		struct iop321_dma_desc *dmad = acq200_dmad_alloc();
		u32 local_pa = wo->pa + empty_offset;
#if (ISR_ADDS_ENDSTOP == 0)
		struct iop321_dma_desc *endstop = 0;
#endif

		if (!dmad){
			err("acq200_dmad_alloc() STARVE");
			return;
		}

		dmad->PDA = DG->fpga.fifo.pa;
		dmad->DD_FIFSTAT = 0;
		dmad->LAD  = local_pa;
		dmad->BC = DMA_BLOCK_LEN;
		dmad->DC = dc;

#if (ISR_ADDS_ENDSTOP == 0)
		if (!rb_get(&IPC->endstops, &endstop)){
			err("STARVED for endstops");
			finish_with_engines(- __LINE__);
		}

		dmad->NDA = endstop->pa;

		rb_put(&IPC->endstops, endstop);
#else
		dmad->NDA = 0;
#endif

#ifndef WAV232
		if (nput < 3 || acq200_debug > 5){
			dbg( 3, "rb_put %s", dmad_diag( dmad ) );
		}
#endif
		rb_put( &IPC->empties, dmad );

		if (++nput >= PUT_MAX_EMPTIES){
			break;
		}
	}
#ifndef WAV232
	if (nput){
		dbg( 3,"direction %s rb_put %d", 
		     wo->direction==PCI_DMA_TODEVICE?
		     "outgoing" : "incoming", nput);
	}
#endif	
	if (RB_IS_EMPTY(IPC->empties)){
		finish_with_engines(-__LINE__);
	}
}

static void tee_empties(int max_empties)
{
	while(!RB_IS_FULL(IPC->empties) && 
	      RB_ELEMENT_COUNT(IPC->empties) < max_empties){
		DMC_WO->handleEmpties(DMC_WO);
	}
}


static irqreturn_t dma_irq_eot(int irq, void *dev_id)
{
	struct InterruptSync* is = (struct InterruptSync*)dev_id;
	u32 flags = DMA_REG(is->regs,DMA_CSR);
	DMA_REG(is->regs,DMA_CSR) = flags;

	return IRQ_HANDLED;
} 



static void acq200_dmc0(struct DMC_WORK_ORDER *wo);

void acq200_eoc_bh1( unsigned long arg )
{

}


DECLARE_TASKLET(acq200_eoc_tasklet1, acq200_eoc_bh1, 0);

#if defined(ACQ196) || defined(ACQ132)
void acq200_service_clock_counters(unsigned long unused) 
{
/** we read Lreg FIRST to ensure it's earlier than Ireg ... */
	u32 Lreg = *ACQ216_TCR_LAT & ACQ216_TCR_COUNTER;
	u32 Ireg = *ACQ216_TCR_IMM & ACQ216_TCR_COUNTER;


	u32 Icount = DMC_WO->clock_count_immediate;
	u32 Lcount = DMC_WO->clock_count_latched;
	
	if (Ireg < (Icount & ACQ216_TCR_COUNTER)){
		/* assume single overflow */
		Icount &= ~ACQ216_TCR_COUNTER;
		Icount += ACQ216_TCR_COUNTER + 1;
		Icount |= Ireg;
	}else{
		Icount &= ~ACQ216_TCR_COUNTER;
		Icount |= Ireg;
	}


	if (unlikely(Lreg != 0 && Lcount == 0)){
		Lcount = Icount &~ ACQ216_TCR_COUNTER;
		if (unlikely(Lreg > Ireg)){
			Lcount -= ACQ216_TCR_COUNTER + 1;
		}
		Lcount |= Lreg;

		DMC_WO->clock_count_latched = Lcount;
	}
	DMC_WO->clock_count_immediate = Icount;
}
#endif
#if defined(ACQ216)
void acq200_service_clock_counters(unsigned long unused) 
{
/** we read Lreg FIRST to ensure it's earlier than Ireg ... 
 *  32 bit counters make this easy 
 */
	DMC_WO->clock_count_latched = *ACQ216_TCR_LAT;
	DMC_WO->clock_count_immediate = *ACQ216_TCR_IMM;
}
#endif


#ifdef ACQ216
static void repeating_gate_action(u32 fifsta) 
/** acq216 repeating gate mode */

{
	u32 fifstat2;

/** @todo big hammer used here ... 
 *  disable_acq() should NOT be required because we have our repeat gate mode
 */
	has_triggered = 1;

	if (++transient_dma_block_count >= transient_dma_blocklimit){
		if (live_one_frame_per_dcb > 2){
			disable_acq();
		}
		if (live_one_frame_per_dcb > 1){
			blip_fifo_reset();
		}
		if (live_one_frame_per_dcb > 2){
			enable_acq();
		}
		transient_dma_block_count = 0;
	}
	fifstat2 = *FIFSTAT;

	if ((fifstat2&ACQ200_FIFCON_EMPTY) != ACQ200_FIFCON_EMPTY){
		dbg(1, "ERROR:live_one_frame_per_dcb set, reset fifo "
		    "fifo NOT EMPTY! 0x%08x => 0x%08x",
		    fifsta, fifstat2);
	}
}
#endif


u32 acq200_get_fifsta(void) {
	return *FIFSTAT;
}

#ifdef ACQ196
static void run_dmc_early_action(u32 fifstat)
/** executes at interrupt priority. We assume client list is short! - 
 * also, that the early_actions() can be called from interrupt state.
 */
{
	struct list_head* clients = &DG->dcb.clients;

	if (!list_empty(clients)){
		struct DataConsumerBuffer *dcb;

		list_for_each_entry(dcb, clients, list) {
			if (dcb->early_action){
				dcb->early_action(dcb, fifstat);
			}
		}
	}
}
#endif

static void fifo_dma_irq_eoc_callback(struct InterruptSync *self, u32 flags)
{
	u32 fifsta = *FIFSTAT;

#ifdef ACQ216
	if (live_one_frame_per_dcb){
		repeating_gate_action(fifsta);
	}
#endif
#ifdef ACQ196
	run_dmc_early_action(fifsta);
#endif

	DG->stats.cold_fifo_histo2[COLD_FIFO_FULL_ENTRIES(fifsta)]++;
	DG->stats.hot_fifo_histo2[HOT_FIFO_FULL_ENTRIES(fifsta)]++;

	self->flags = flags;

#if defined(ACQ216) || defined(ACQ196)
#if defined(ACQ196)
	if (!DG->slow_clock)
#endif
	/** this assumes a sufficiently fast sample rate to service ... */
	acq200_service_clock_counters(0);
#endif
	if (DG->finished_with_engines || RB_IS_EMPTY(IPC->active)){
		return;
	}

/* acq216 - nothing waits on this Q. But may be needed by WAV etc WORKTODO
                wake_up_interruptible(&is->waitq);
*/
	if (fifsta&DG->FIFERR || (flags&DMA_ERROR)){
		if (!DG->fiferr){
			if (fifsta&DG->FIFERR){
				DG->fiferr = fifsta;
			}else{
				// this should never happen, flags stubbed
				DG->fiferr = 0xdead; 
			}
		}
		tasklet_schedule(&acq200_fifo_error_tasklet);
	}else{
		if (RB_ELEMENT_COUNT(IPC->empties) < EMPTY_FILL_THRESHOLD ||
		    RB_ELEMENT_COUNT(IPC->active) > ACTIVE_BATCH_THRESHOLD  ){
			wake_dmc0();
			DG->stats.num_eoc_bh++;
		}
	}

	self->interrupted = 0;  /* @@todo - most likely bogus */
}

static void regular_dma_irq_eoc_callback(struct InterruptSync *self, u32 flags)
{
	wake_up_interruptible(&self->waitq);
}




static irqreturn_t dma_irq_eoc(int irq, void *dev_id)
{
	struct InterruptSync* is = (struct InterruptSync*)dev_id;
	u32 flags = DMA_REG(is->regs,DMA_CSR);

	DMA_REG(is->regs,DMA_CSR) = flags;
	is->interrupted = 1;
	    
	if (is->isr_cb) is->isr_cb( is, 0 ); 

	if ((DG->stats.num_eoc_ints++&0xfff) == 0){
		*IOP321_GPOD ^= ACQ200_LED4;
	}
	return IRQ_HANDLED;
}



static int pci_abort(void)
{
	unsigned mbox0 = *IOP321_IMR0;

	if (DG->mbox_abort_mask){
		mbox0 &= DG->mbox_abort_mask;
		if (mbox0 == DG->mbox_abort_value){
			return 1;
		}
	}
	return 0;
}




void acq200_dmc0(struct DMC_WORK_ORDER *wo)
/* Data Movement Controller */
{
	if (DG->fiferr){
		sprintf(errbuf, "FIFERR detected 0x%08x", DG->fiferr);
		wo->error = errbuf;
		finish_with_engines(-__LINE__);
	}else{
		if (DMC_WO->triggered == 0 && DMC_WO->trigger_detect()){
			onTrigger();
		}

		dbg( 4, "active %s empties %s", 
		     RB_IS_EMPTY(IPC->active)? "EMPTY": "active",
		     RB_IS_FULL(IPC->empties)? "FULL": "active");
#ifdef WAV232
/* @@todo THEORY: packets are in short supply in WAV. so recycle them first */
		/* handle active */
		if (!RB_IS_EMPTY(IPC->active)){
			dmc_handle_refills(wo);
		}
		/* fill empties */
		if (RB_ELEMENT_COUNT(IPC->empties) < EMPTY_FILL_THRESHOLD){
			wo->handleEmpties(wo);
		}
#else		
		/* fill empties (emergency) */
		if (RB_ELEMENT_COUNT(IPC->empties) < EMPTY_FILL_THRESHOLD/2){
			wo->handleEmpties(wo);
		}
		/* handle active */
		if (!RB_IS_EMPTY(IPC->active)){
			dmc_handle_refills(wo);
		}
		/* fill empties (regular) */
		if (RB_ELEMENT_COUNT(IPC->empties) < EMPTY_FILL_THRESHOLD){
			wo->handleEmpties(wo);
		}
#endif
	}
	if ((DG->stats.num_dmc_run++ & 0xfff) == 0){
		*IOP321_GPOD ^= ACQ200_LED3;
	}
}

static void dmc0_timeout(unsigned long arg);

static void schedule_dmc0_timeout(unsigned long arg)
{
	static struct timer_list timeout;

	if (timeout.function == 0){
		init_timer(&timeout);
		timeout.function = dmc0_timeout;
	}

	if (arg != 0){
		if (!timer_pending(&timeout)){
			timeout.data = arg;		
			timeout.expires = jiffies + 3;
			add_timer(&timeout);
		}
		/* else, somehow it was already running, so leave it */
	}else{
		del_timer_sync(&timeout);
	}
}



static void dmc0_timeout(unsigned long arg)
{
	if (!DG->finished_with_engines){
		if (dmc0.task){
			wake_dmc0();
		}
		schedule_dmc0_timeout(arg);
	}
}

/*
 || 
  */

static int acq200_dmc0_task(void *arg)
{
/* this is going to be the top RT process */
	struct sched_param param = { .sched_priority = 10 };

	sched_setscheduler(current, SCHED_FIFO, &param);
	current->flags |= PF_NOFREEZE;
	
	while(1){
		int run_request;
		int timeout = wait_event_interruptible_timeout(
			dmc0.waitq,
			(run_request = test_and_clear_bit(
				 DMC0_RUN_REQUEST, &dmc0.v)) ||
			kthread_should_stop(),
			DMC_TO) == 0;

		if (kthread_should_stop()){
			return 0;
		}
		dbg(2, "run: %s %s", 
		    timeout? "TIMEOUT": "",
		    run_request? "RUN_REQUEST": "");

		if (run_request){
			acq200_dmc0(DMC_WO);
		}
	}
	return 0;
}

static void dmc0_start(int start)
{
	if (start){
		if (dmc0.task != 0){
			dmc0_start(0);
		}
		init_waitqueue_head(&dmc0.waitq);
		dmc0.v = 0;
		dmc0.task = kthread_run(acq200_dmc0_task, DMC_WO, "dmc0");
	}else{
		if (dmc0.task != 0){
			kthread_stop(dmc0.task);
			dmc0.task = 0;
		}
	}
}

static struct Phase *onPIT_default(
	struct Phase *phase, u32 status, u32* offset)
{
	return phase;
}


#define MINIMUM_RECYLE_SAMPLES (get_tblock_max_sam()*sample_size())

static void init_phase(struct Phase *phase, int len, int oneshot)
{
	char name[32];

	dbg(1, "phase %s", phase->name);

	strcpy(name, phase->name);
	memset(phase, 0, sizeof(struct Phase));
	strcpy(phase->name, name);
	
	dbg(1, "phase %s", phase->name);

	INIT_LIST_HEAD(&phase->tblocks);

	phase->required_len = len;

	if (oneshot){
		phase->demand_len = len;
	}else{
		phase->demand_len = max(len, MINIMUM_RECYLE_SAMPLES);
	}
	phase->tblock_max_count = phase->demand_len/TBLOCK_LEN + 2;
	phase->is_oneshot = oneshot;
	phase->onPIT = onPIT_default;

	list_add_tail(&phase->list, &DMC_WO->phases);
}



static struct Phase* onPIT_repeater(
	struct Phase *phase, u32 status, u32* offset)
{
	/* WORKTODO - should locate trigger and export ... */
	return phase;
}

static struct Phase * onPIT_clear(
	struct Phase *phase, u32 status, u32* offset)
{
	struct Phase* next = NEXT_PHASE(phase);
	/* @@todo common def 196 216 needed */
	/* @@todo WHY NOT eve[1] ?? */
	deactivateSignal(CAPDEF->ev[0]);
	if (next){
		next->start_off = phase->end_off;
	}
	return next;
}



static void release_phases(void)
{
	struct Phase *phase;

	list_for_each_entry(phase, &DMC_WO->phases, list){
		acq200_phase_release_tblocks(phase);
	}
	if (DG->ext_phase && DG->ext_phase->release){
		DG->ext_phase->release();
	}
}

static void init_phases(void)
/* first pass at setting up phasing */
{
	static struct Phase phases[2] = {
		[0].name = "phase1",
		[1].name = "phase2"
	};
	static struct Phase null_phase = {
		.name = "null_phase"
	};

	switch(CAPDEF->mode){
	case M_SOFT_CONTINUOUS:
		init_phase(&phases[0], CAPDEF->demand_prelen, 0);
		init_phase(&null_phase, 0, 1);
		phases[0].onPIT = onPIT_repeater;
		DMC_WO->pre = &phases[0];
		DMC_WO->post = &null_phase;
		break;
	case M_TRIGGERED_CONTINUOUS:
		init_phase(&phases[0], CAPDEF->demand_prelen,  0);
		init_phase(&phases[1], CAPDEF->demand_postlen, 1);
		init_phase(&null_phase, 0, 1);

#ifdef PEEK_IS_REQUIRED
#ifndef WAV232
		phases[0].start_off = peekFirstEmpty();
		dbg(1, "phase[0].start_off set %d", phases[0].start_off);
#endif
#endif
		phases[0].ev = CAPDEF->ev[0];
		deactivateSignal(phases[0].ev);   /** enabled at corner */
		phases[0].onPIT = onPIT_clear;
		DMC_WO->pre = &phases[0];
		DMC_WO->post = &phases[1];
		break;
	default:
		init_phase(&null_phase, 0, 1);
		init_phase(&phases[0], CAPDEF->demand_len, 1);
		DMC_WO->pre = &null_phase;
		DMC_WO->post = &phases[0];
	}
	DMC_WO->now = &phases[0];


	if (DG->ext_phase && DG->ext_phase->init){
		DG->ext_phase->init();
	}
	dbg(3,"DMC_WO %p now0 %p", DMC_WO, DMC_WO->now);
}


#define USE_DG 0

static void build_dmad( void *buf, int len, int dir )
{
	INIT_LIST_HEAD(&DMC_WO->phases);
	DMC_WO->direction = dir;
	DMC_WO->buf = buf;
	DMC_WO->wo_len = len;
	DMC_WO->oneshot = DG->is_oneshot;
	DMC_WO->pa  = pa_buf( DG );
	DMC_WO->pit_stop = CAPDEF->pit_stop;

	sccInit(&DMC_WO->scc, DMA_BLOCK_LEN/sample_size(), NCHAN == 96? 3: 0);
	
	/** @@todo ... initphases() here then remove all other refs */

	tee_empties(INIT_TEE_EMPTIES);
}


static void dma_sync( void *va, int len, int dir )
/* WORKTODO: use region sync when available, for now do the lot */
{
	dma_sync_single( DG->dev, DG->dma_handle, len, dir );
}



static void check_int_status(void)
{
	u32 intctl = *IOP321_INTCTL;
	u32 intstr = *IOP321_INTSTR;
	u32 iintsrc = *IOP321_IINTSRC;
	u32 fintsrc = *IOP321_FINTSRC;

	if ((iintsrc&FPGA_INT_MASK) != 0 || (fintsrc&FPGA_INT_MASK) != 0){
		err( "INTERRUPT BEFORE TRIGGER!\n"
		     "%20s: 0x%08x\n"
		     "%20s: 0x%08x\n"
		     "%20s: 0x%08x\n"
		     "%20s: 0x%08x\n"
		     "%20s: 0x%08x\n"
		     "%20s: 0x%08x\n"
		     "%20s: 0x%08x\n",
		     "FPGA_INT_MASK", FPGA_INT_MASK,
		     "IOP321_INTCTL", intctl,
		     "IOP321_INTSTR", intstr,
		     "IOP321_IINTSRC", iintsrc,
		     "IOP321_FINTSRC", fintsrc,
		     "ACQ200_SYSCON", *SYSCON,
		     "ACQ200_FIFCON", *FIFSTAT);

	}
#ifdef ACQ216
	acq200_unmask_irq(28);     /* WORKTODO .. should NOT be needed! */
#endif
}
static void preEnable(void)
{
	dbg(1, "ICR=%x", 0);

	run_pre_arm_hook();

	DMC_WO_setState(ST_ARM);
	*ACQ200_ICR = 0;

	/* GO RTOS! */
	if (DG->global_irq_mask){
		acq200_global_mask_op(DG->global_irq_mask, 1);
	}

	if (tee_isr_head() == 0){
		err("head == 0");
		return;
	}
	try_fiq();

	dbg(1, "ICR=%x", ACQ_INTEN);
	*ACQ200_ICR = ACQ_INTEN;

	check_int_status();
	dmc0_start(1);
	schedule_dmc0_timeout((unsigned long)&DMC_WO);
}

static void call_post_arm_hooks(void)
{
	struct list_head *cursor;
	struct Hookup *hookup;

	list_for_each(cursor, &DG->start_of_shot_hooks){
		hookup = list_entry(cursor, struct Hookup, list);
		hookup->the_hook(hookup->clidata);
	}
}

static void onEnableAction(struct work_struct *not_used)
{
	call_post_arm_hooks();	
	run_post_arm_hook();
}

static struct work_struct onEnable_work;

static void onEnable(void)
{
	schedule_work(&onEnable_work);
}

static void onTrigger(void)
{
	unsigned long flags;
	int my_turn = 0;

	spin_lock_irqsave(&DMC_WO->onTrigger.lock, flags);
	if (DMC_WO->onTrigger.done == 0){
		DMC_WO->triggered = 1;
		my_turn = DMC_WO->onTrigger.done = 1;
	}
	spin_unlock_irqrestore(&DMC_WO->onTrigger.lock, flags);

	if (my_turn){
		iop321_start_ppmu();
		DG->stats.start_gtsr = *IOP321_GTSR;
		DG->stats.start_jiffies = jiffies;
		DMC_WO_setState(ST_RUN);
		DG->shot++;
	}
}
static int soft_trigger_retry;


static int fiq_init_action(void)
{
	struct pt_regs regs = {
		.ARM_fp  = (long)IOP321_DMA0_CCR,
	};

	regs.ARM_ip = (long)DG,
	regs.ARM_r8 = DG->head->pa;

#ifndef ACQ196
	regs.ARM_r9 = (DG->fpga.regs.pa&~0xf0000000)|ACQ200_PCI_VSTAT;
#else
	regs.ARM_r9 = (long)DG->fpga.regs.va;
#endif


	assert(regs.ARM_r8);
	assert(regs.ARM_r9);

	dbg(1, "head    set %p", DG->head);
	dbg(1, "head_pa set 0x%08lx", regs.ARM_r8);
	dbg(1, "fiq dg  set 0x%08lx", regs.ARM_ip);

	set_fiq_regs(&regs);	
	set_fpga_isr_steering(1);
	return 0;
}

static int try_fiq(void)
{
	int rc = 0;

	if (DG->use_fiq){
		fiq_init_action();
	}else{
		use_endstop = 0;
	}

	return rc;
}



static int fifo_open( int direction )
{
	dma_addr_t handle = dma_map_single( 
		DG->dev, va_buf( DG ), len_buf( DG ), direction );

	init_dmac();

	memset(&DG->stats, 0, sizeof(DG->stats));
	DG->bigbuf.tblocks.cursor = 0;
	acq200_init_tblock_list();

	if ( handle ){
		DG->dma_handle = handle;
		DG->direction = direction;
		dbg ( 1, "SUCCESS handle 0x%08x", handle );
		return 0;
	}else{
		err( "pci_map_single() failed" );
		return -ENOMEM;
	}
}


static int acq200_fpga_fifo_release (struct inode *inode, struct file *file)
{
	disable_acq();
	disable_fifo();

	if ( DG->dma_handle ){
		dbg( 1, "pci_unmap()" );
		/** pgm 20060331 ... force invalidate.
                 *  this should NOT be necessary (invalidated on fifo_open().
                 *  but, maybe the code screws up somewhere (reads data before
                 *  it arrived.... this can't harm, and may fix 1:20
                 *  duff data before transform...
		 */
		consistent_sync(va_buf(DG), len_buf(DG), DMA_FROM_DEVICE);
		dma_unmap_single( DG->dev, DG->dma_handle, 
				  len_buf(DG), DG->direction );
		DG->dma_handle = 0;
	}

	if (soft_trigger_retry){
		info( "WARNING: soft_trigger_retry %d",
		      soft_trigger_retry );
	}

	dbg( 1, "DG->open_count %d", DG->open_count );

	return 0;
}




static ssize_t acq200_fpga_fifo_write_buf_write ( 
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
{
/*
 * DUMMY write - assume buffers previously filled to LEN via wav232 device 
 */
	DG->ifill = LEN;
#ifdef WAV232
	dbg(3, "FIFCON:%08x setting DG->ifill to %d",
	    *ACQ200_FIFCON, DG->ifill );
#endif

	return len;
}

static ssize_t acq200_fpga_fifo_write_buf_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	len = min( len, (size_t)(DG->ifill - (int)*offset) );

	dbg( 2, "len %d *offset %d", len, (int)*offset );

	if (copy_to_user(buf, va_buf_offset(DG, *offset), len )){
		return -EFAULT;
	}
	*offset += len;

	return len;
}


static void acq200_free_PBChainDesc(struct PrebuiltChain *pbc){
	int ichain;

	for (ichain = 0; ichain != pbc->length; ++ichain){
		struct iop321_dma_desc *dmad = pbc->the_chain[ichain];

		if (dmad){
			dbg(ACQ200_DMAD_DEBUG, "dmad free %p", dmad);
			dmad->clidat = 0;
			acq200_dmad_free(dmad);
		}
	}
}

void acq200_rb_drain(struct acq200_dma_ring_buffer* rb)
{
	struct iop321_dma_desc *pbuf;

	dbg(ACQ200_DMAD_DEBUG, "rb 01 %s", rb->name);

	while(rb_get(rb, &pbuf)){
		if (isPBChainDesc(pbuf)){
			dbg(ACQ200_DMAD_DEBUG, "isPBChainDesc");
			acq200_free_PBChainDesc(getPBChain(pbuf));
		}else if (pbuf->clidat){
			dbg(ACQ200_DMAD_DEBUG, "clidat");
			acq200_free_PBChainDesc(
				(struct PrebuiltChain*)pbuf->clidat);
		}else{
			dbg(ACQ200_DMAD_DEBUG, "rb %s dmad_free %p", 
			    rb->name, pbuf);
			acq200_dmad_free(pbuf);
		}
	}

	dbg(ACQ200_DMAD_DEBUG, "rb 99 %s", rb->name);
	acq200_rb_clear_counters(rb);
}


void acq200_add_ext_phase_handler(struct ExtPhase *ext_phase)
{
	/* @@todo later: use a list to allow multiple hooks if required */
	DG->ext_phase = ext_phase;
}
void acq200_del_ext_phase_handler(struct ExtPhase *ext_phase)
{
	DG->ext_phase = 0;
}

void acq200_add_start_of_shot_hook(struct Hookup *hook)
{
	list_add_tail(&hook->list, &DG->start_of_shot_hooks);
}

void acq200_del_start_of_shot_hook(struct Hookup *hook)
{
	struct list_head *cursor;

	list_for_each(cursor, &DG->start_of_shot_hooks){
		if (list_entry(cursor, struct Hookup, list) == hook){
			list_del(cursor);
			return;
		}
	}
	err("Failed to delete %p", hook);
}


void acq200_add_end_of_shot_hook(struct Hookup *hook)
{
	list_add_tail(&hook->list, &DG->end_of_shot_hooks);
}

void acq200_del_end_of_shot_hook(struct Hookup *hook)
{
	struct list_head *cursor;

	list_for_each(cursor, &DG->end_of_shot_hooks){
		if (list_entry(cursor, struct Hookup, list) == hook){
			list_del(cursor);
			return;
		}
	}
	err("Failed to delete %p", hook);
}




static void init_bda(void) 
{
	int spb = DMA_BLOCK_LEN/sample_size();

	DMC_WO->bda_blocks.before = DG->bda_samples.before/spb;
	DMC_WO->bda_blocks.during = DG->bda_samples.during/spb;
	DMC_WO->bda_blocks.after  = DG->bda_samples.after/spb;
	DMC_WO->bda_blocks.state = BDA_IDLE;
}

static int null_trigger_detect(void) {
	return 0;
}

static void clear_buffers(void)
{
	void* getNextEmpty = DMC_WO->getNextEmpty;

	dmc0_start(0);
	DG->ifill = 0;
	DG->finished_with_engines = 0;
	DG->fiferr = 0;

	release_phases();
	DMC_CLEAN(DMC_WO);
	spin_lock_init(&DMC_WO->onTrigger.lock);
	DMC_WO->trigger_detect = null_trigger_detect;
	DMC_WO_setState(ST_STOP);
	DMC_WO->getNextEmpty = getNextEmpty;

	init_bda();

	initPhaseDiagBuf();
#ifndef WAV232
	bda_release_tblock();
#endif
	acq200_empties_release_tblocks();
	acq200_rb_drain(&IPC->empties);
	acq200_rb_drain(&IPC->active);
	acq200_rb_drain(&IPC->endstops);
	init_endstops(0);
	/* leave endstops */
	memset( &DG->stats, 0, sizeof(DG->stats));
	acq200_dmad_clear();	
}

static void enable_regular_operation(void)
{
/* WORKTODO : what if DMA1 ? */
	*ACQ200_ICR = 0;
	enable_interrupt_hook(&IPC->is_dma[0].eoc);
	disable_acq();
	SET_SIMULATION_MODE(DG->simulate);
	acq200_reset_fifo();
}

static int acq200_fpga_fifo_write_open (struct inode *inode, struct file *file)
{
	enable_regular_operation();

	init_phases();
	clear_buffers();
	init_endstops(INIT_ENDSTOPS);
#ifdef WAV232
	*ACQ200_SYSCON |= WAV232_SYSCON_NDACRESET;
#endif
	return fifo_open( PCI_DMA_TODEVICE );
}




static int _oneshot_wait_for_done(void)
{
	struct DMC_WORK_ORDER *wo = DMC_WO;
	int ntimeout;
	int itry;
	int iloop = 0;


#define MAXTRY 0x200000

	dbg(2, "01");
	acq200_cdog(CDOG_INIT);

	for ( itry = 0; itry < MAXTRY; itry++, ++iloop){

		dbg(3,"itry %d ints %d", itry, DG->stats.num_fifo_ints);

/* pgm 20040125 ... need to time out somehow - */

		if (DG->finished_with_engines){
			return 0;
		}else if (DG->stats.num_fifo_ints){
			if (acq200_cdog(CDOG_REFRESH)){
				return 0;
			}else{
				itry = 0;
			}
		}

		if (DG->busywait){
			for ( ntimeout = 0x100; ntimeout; --ntimeout ){
				if (acq200_cdog(CDOG_REFRESH)){
					dbg(2, "95");
					return 0;
				}else if (wo->error||DG->finished_with_engines){
					dbg(2,"96");
					return 0;
				}else{
					if (DG->busywait == 2){
						yield();
					}
					++DG->stats.busy_pollcat;
				}
			}
		}else{
			ntimeout = wait_event_interruptible_timeout( 
				IPC->finished_waitq,
				wo->error || DG->finished_with_engines,
				10 );
			++DG->stats.busy_pollcat;
		}
		

		if (pci_abort()){
			strcpy(errbuf, "REMOTE ABORT");
			wo->error = errbuf;
			finish_with_engines(-__LINE__);
			info( "REMOTE ABORT itry %d", itry);
			dbg(2, "97");
			return 0;
		}
			
		if ( DG->fiferr ){
			if (!wo->error){
				sprintf( errbuf, "fiferr 0x%08x", DG->fiferr);
				wo->error = errbuf;
			}
			finish_with_engines(-__LINE__);
		}

		if (DMC_WO->triggered && (iloop&(MAXTRY/16 - 1))==0){
			dbg( 1, "%6d CLOSE DOWN %08x %s %s", 
			     itry, *FIFSTAT, 
			     ntimeout==1?"TIMEOUT":"",
			     wo->error? "WO_ERROR":"" );
		}
		if ( ntimeout || DG->finished_with_engines || wo->error ){
			dbg(2, "98");
			return 0;
		}
	}

	sprintf( errbuf, "ERROR: TIMEDOUT" );
	wo->error = errbuf;
	finish_with_engines(-__LINE__);

	dbg(2, "99");

	return 0;
}

static int oneshot_wait_for_done(void)
{
	int rc = _oneshot_wait_for_done();

	if (rc == 0 && DG->finished_with_engines > 0){
		run_post_shot_hook();
	}
	return rc;
}


static int acq200_fpga_release (struct inode *inode, struct file *file)
{
	dbg( 1, "WORKTODO" );
	return 0;
}


#ifdef WAV232
static int acq200_fpga_fifo_write_release (
	struct inode *inode, struct file *file)
{
	oneshot_wait_for_done();

	return acq200_fpga_fifo_release(inode, file);
}


static int acq200_fpga_fifo_write_buf_release(
	struct inode *inode, struct file *file)
{
	int len = DG->ifill;
	int rc;

	dma_sync( va_buf( DG ), len, PCI_DMA_TODEVICE );
	build_dmad( va_buf( DG ), len, PCI_DMA_TODEVICE );

	init_phases();  /* @@todo hack by pgm 20040813 - position is critical*/

	dbg(1, "bigbuf load and map done" );

	rc = fifo_write_action( DG->ifill );

	if ( rc != 0 ){
		err( "fifo_write_action() %d", rc );
	}

	return acq200_fpga_fifo_write_release(inode, file);
}
#endif
static int acq200_fpga_fifo_write_buf_nullrelease(
	struct inode *inode, struct file *file)
{
	dbg( 1, "data len: %d", DG->ifill );
	return 0;
}






int acq200_mmap_bigbuf( struct file* filp, struct vm_area_struct* vma )
// mmap the large contig area of mem at DG->bigbuf to vma
{
	return io_remap_pfn_range( 
		vma, vma->vm_start, 
		pfn_buf(DG), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
		);
}



/*
 * LIVE OPS
 */


int acq200_early_action_count = 0;
module_param(acq200_early_action_count, int, 0664);

#if defined (ACQ196) && CFG_LOW_JITTER_BURST_START == 1
static void pulse_start_early_action(
	struct DataConsumerBuffer* dcb, u32 fifstat)
{
	if (fifstat&ACQ196_FIFSTAT_ADC_EV1){
		pulse_start(&dcb->pdt);
		++acq200_early_action_count;
	}
}
#endif
#define DCB(filp) ((struct DataConsumerBuffer*)filp->private_data)
#define SET_DCB(filp, dcb) (filp->private_data = dcb)

static ssize_t acq200_fpga_fifo_live_read_data(
        struct file *file, char *buf, size_t len, loff_t *offset);
#ifdef ACQ216
static ssize_t acq200_fpga_live_read_one_frame_per_dcb(
	struct file *file, char *buf, size_t len, loff_t *offset);
#endif
static int acq200_fpga_fifo_live_open(
        struct inode *inode, struct file *file)
{
	struct DataConsumerBuffer *dcb = acq200_createDCB();
	SET_DCB(file, dcb);

	memcpy(&dcb->burst, &DG->burst, sizeof(struct BurstDef));



#ifdef ACQ216
#ifdef PGMCOMOUT
	if (live_one_frame_per_dcb){
		file->f_op->read = acq200_fpga_live_read_one_frame_per_dcb;
	}else{
		file->f_op->read = acq200_fpga_fifo_live_read_data;
	}
#else
#warning ACQ216 acq200_fpga_live_read_ read not modfied WORKTODO
#endif
#endif

	if (dcb->burst.len){
		dcb->state = DCS_WAIT_EVENT_WORD;
#ifdef ACQ196
#if CFG_LOW_JITTER_BURST_START
		dcb->early_action = pulse_start_early_action;
#endif
		if ((*FIFSTAT&ACQ196_FIFSTAT_ADC_EVX)){
			dbg(1, "FORCE event enable");
			*FIFSTAT = ACQ196_FIFSTAT_ADC_EVX;
		}
#endif
	}else{
		dcb->state = DCS_STREAMING_CONTINUOUS_ESIG;
	}

	if (DG->pulse.pulse_count){
		dcb->pdt.pulse_def = DG->pulse;
		pulse_init(&dcb->pdt);
	}

	sccInit(&dcb->scc, DMA_BLOCK_LEN/sample_size(), NCHAN == 96? 3: 0);
	dbg(1," state {%d}", dcb->state);

	acq200_addDataConsumer(dcb);
	return 0;
}


static int acq200_fpga_fifo_live_release(
        struct inode *inode, struct file *file)
{

	struct DataConsumerBuffer *dcb = DCB(file);

	if (dcb->pdt.pulse_def.pulse_count){
		pulse_close(&dcb->pdt);
	}
	acq200_removeDataConsumer(dcb);
	dbg(1," state {%d}", dcb->state);
	acq200_deleteDCB(dcb);

	return acq200_fpga_release(inode, file);
}


int acq200_check_entire_es(unsigned *es)
{
	int invalid = 0;
	int ipair;
	int npairs = ES_LONGS;

#define VALID (!invalid)	
#define EVENT_MAGIC_FAIL(ip) \
	(!EVENT_MAGIC_EXEMPT(ip) && !IS_EVENT_MAGIC(es[ip]))

	for (ipair = 0; ipair != npairs; ++ipair){
		if (es_debug){
			info("%2d 0x%08x %s %s",
			     ipair, es[ipair], 
			     IS_EVENT_MAGIC(es[ipair])? "MAGIC": "nmagc",
			     EVENT_MAGIC_FAIL(ipair)? "FAIL": "OK");
		}
		if (VALID && EVENT_MAGIC_FAIL(ipair)){
			err("BAD MAGIC [%2d] %08x", ipair, es[ipair]);
			invalid = ipair+1;
		}
	}

	if (invalid){
		if (!IS_EVENT_MAGIC(es[ipair-1])){
			err("BAD MAGIC [%2d] %08x", ipair-1, es[ipair-1]);
		}
		err("BAD ES start %p", es);
	}else{
		if (es_debug){
			for (ipair = npairs; ipair != npairs*2; ++ipair){
			     info("%2d 0x%08x %s %s",
				     ipair, es[ipair], 
			       	     IS_EVENT_MAGIC(es[ipair])? 
							"MAGIC": "nmagc",
					IS_EVENT_MAGIC(es[ipair])?	
							 "ERR": "OK");
			}
		}       
	}

	if (es_debug > 1){
		short *data = (short *)es;
		int nshorts = npairs *2;
		int ic;

		info("inserting marker pattern");

		for (ic = 0; ic < nshorts; ++ic){
			data[ic] = ic | 0x200;
		}
		data += nshorts;
		for (ic = 0; ic < nshorts; ++ic){
			data[ic] = ic | 0x300;
		}			
	}
	dbg(1 && VALID, "ES %08x %08x %08x %s",
	     es[0], 
	     npairs > 16? es[16]: 0, 
	     npairs > 32? es[32]: 0, 
	     VALID? "ES_VALID": "ERROR: NOT VALID");

	return VALID;
#undef VALID
}

#define CHECK_ENTIRE_ES(searchp) acq200_check_entire_es(searchp)




static struct PhaseDiagBuf {
	short found;
	short valid;
	int matches;
	u32 last_before, first_es, last_es, first_after;
	u32 first, last;

	int outsize;
	
	int metric, searchlen;
} pdb;



static unsigned ES_DIAG_BUFFER[PAGE_SIZE/sizeof(unsigned)];

int acq200_getES_DIAG(void* buf, int max_buf)
{  
	if (pdb.found){
		int ndiag = 3*ES_LONGS*sizeof(long);
		assert(ndiag <= sizeof(ES_DIAG_BUFFER));

		max_buf = min(max_buf, ndiag);    
		memcpy(buf, ES_DIAG_BUFFER, max_buf);
		return max_buf;
	}else{
		return 0;
	} 
}

static const char* formatPhaseDiagBuf(char* buf, int maxbuf)
{
	int nc;

	switch(pdb.found){
	case 1:
		nc = snprintf(buf, maxbuf,
			 "FOUND %d "
			 "[%08x|%08x ..%08x|%08x] %08x => %08x %s", 
			 pdb.matches, 
			 pdb.last_before, pdb.first_es, 
			 pdb.last_es, pdb.first_after,
			 pdb.first, pdb.last,
			 pdb.valid? "VALID ES": "INVALID_ES");
		if (pdb.outsize){
			snprintf(buf+nc, maxbuf-nc, 
				 " WARNING:outsize %d sample\n", pdb.outsize);
		}else{
			strcat(buf, "\n");
		}
		break;
	case -1:
		snprintf(buf, maxbuf,
			 "event not found in %s %d bytes\n", 
			 pdb.metric>0? "+": "+/-", pdb.searchlen);
		break;
	default:
		snprintf(buf, maxbuf, "---\n");
		break;
	}
	return buf; 
}

static void initPhaseDiagBuf(void)
{
	pdb.found = 0;
}
static void initPhaseDiagBufFound(
	short valid, int matches, 
	u32 last_before, u32 first_es, u32 last_es, u32 first_after,
	u32 first, u32 last
	)
{
	pdb.found = 1; 
	pdb.valid = valid;
	pdb.matches = matches;
	pdb.last_before = last_before; 
	pdb.first_es = first_es;
	pdb.last_es = last_es; 
	pdb.first_after = first_after;
	pdb.first = first;
	pdb.last = last;
	pdb.outsize = 0;



	if (acq200_debug){
		static char buf[256];
		dbg(1, "%s", formatPhaseDiagBuf(buf, 256));
	}
}	

static void initPhaseDiagBufNotFound(int metric, int searchlen)
{
	pdb.found = -1;
	pdb.metric = metric;
	pdb.searchlen = searchlen;
}

static int findEvent(struct Phase *phase, unsigned *first, unsigned *ilast)
/* search next DMA_BLOCK_LEN of data for event word, 
 * return 1 and update iput if found 
 */
{
	unsigned* searchp = (unsigned*)va_buf(DG);
	unsigned istart = *first/USS;
	unsigned max_search = istart + DMA_BLOCK_LEN/USS;
	unsigned isL;         /* bracket ES Left */
	unsigned isR;         /* bracket ES Right */
	int matches = 0;
	int iter = 0;

#define INTERESTING \
       (++iter < 3 || IS_EVENT_MAGIC(searchp[isL]) || iter%32==0)
/*
 * identify event start, and fast forward to it
 */
	for (isL = istart; isL < max_search; isL += SAMPLE_LONGS){

		dbg((INTERESTING? 1: 2),
			"%4d:off %08x max %08x d:%08x %s",
			iter, isL*4, max_search*4, searchp[isL], 
			IS_EVENT_MAGIC(searchp[isL])? "MAGIC": "");

		if ((matches = IS_EVENT_MAGIC(searchp[isL]))){

			int valid;

			while(isL >= istart && 
			      IS_EVENT_MAGIC(searchp[isL])){
				--isL;
				++matches;
			}
			if (IS_EVENT_MAGIC(searchp[isL])){
				err("FAILED to find beginning of event");
			}
			++isL;            /* select first match */

			if (!(matches > 1)){
				if (!IS_EVENT_MAGIC(searchp[isL+1])){
					err("false pos");
					break;
				}
			}
			{
				unsigned ileft = max(isL-ES_LONGS, 0U);
				int ndiag = 3*ES_LONGS*sizeof(long);
				assert(ndiag <= sizeof(ES_DIAG_BUFFER));
			
				memcpy(ES_DIAG_BUFFER, searchp+ileft, ndiag);
			}
			valid = CHECK_ENTIRE_ES(&searchp[isL]);

			initPhaseDiagBufFound(valid, matches,
				searchp[isL-1], 
				searchp[isL],
				searchp[isL+ES_LONGS-1],
				searchp[isL+ES_LONGS],
				*first, isL * USS);
			*first = isL * USS;

			if (ilast){
				int outsize = 0;

				isR = isL + ES_LONGS;
				while (IS_EVENT_MAGIC(searchp[isR]) &&
				       IS_EVENT_MAGIC(searchp[isR+1]) ){
					isR++;
					outsize++;
				}
				
				if (outsize){
					err("WARNING: outsize sample");
					pdb.outsize = outsize;
				}
				*ilast = isR * USS;
			}
			DTACQ_MACH_EVENT_ADJUST(phase, isL, first, ilast);

			dbg(1, "return first %08x last %08x",
			    first? *first: 0, ilast? *ilast: 0);

			return 1;
		}
	}

	return 0;
}

void find_es_anywhere(void)
/** @@todo - this function is BAD BAD... locks system for ages */
{
	unsigned * first = va_buf(DG);
	unsigned * last  = first + len_buf(DG)/sizeof(unsigned);

	findEvent(0, first, last);
}

/*
 * Shuffle up: the end point has moved.
 * So now move the beginning to get the right #samples
 */

static void shuffle_up(struct Phase* phase, int isearch)
{
	struct TblockListElement* tle;
	int actual_len = 0;
#define PLUG \
        dbg(1, "%d [%2d] start:%d is:%d len:%d", \
		 __LINE__, tle->tblock->iblock, phase->start_off, \
                 isearch, actual_len)

	dbg(1, "phase:%s start start_off %d end_off %d to isearch %d, len %d",
	    phase->name,
	    phase->start_off,
	    phase->end_off, isearch, phase->actual_len);


	list_for_each_entry(tle, &phase->tblocks, list){
		if (IN_TBLOCK(tle->tblock, phase->start_off) &&
			phase->lap_count == 0){

			if (IN_TBLOCK(tle->tblock, isearch)){
				actual_len = isearch - phase->start_off;
				PLUG;
			}else{
				actual_len = TBLOCK_LEN - 
					TBLOCK_OFFSET(phase->start_off);
				PLUG;
			}
		}else if (IN_TBLOCK(tle->tblock, isearch)){
			actual_len += TBLOCK_OFFSET(isearch);
			PLUG;
		}else{
			actual_len += TBLOCK_LEN;
			PLUG;
		}
	}


	phase->actual_len = actual_len;
	phase->end_off = isearch;

	dbg(1, "finit end_off %d to isearch %d, len %d",
	    phase->end_off, isearch, phase->actual_len);
}


unsigned int get_tblock2_offset(struct Phase* phase, unsigned* offset)
{
	struct TblockListElement* tle;
	struct TblockListElement* tmp;
	int iblock = 0;

	if (phase->tblocks.next == 0){
		return -1;
	}

	list_for_each_entry_safe(tle, tmp, &phase->tblocks, list){ 
		if (++iblock == 2){
			*offset = tle->tblock->offset;
			return 0;
		}
	}
	return -1;
}

/*
 * Event POSition
 * Where is the Event? event is reported async by interrupt.
 * We KNOW which DMA_BLOCK it was in, because the FIQ embeds the status.
 * So the event is somewhere nearby after the front of the reporting phase.
 * We need to find the event, move start_off back
 * Then, prev->end_off needs to move back as well.
 */


int search_for_epos_in_tblock(
	struct Phase* phase, unsigned isearch, int max_dma_blocks)
/** return 0 if epos found, and fixup phases */
{
	int dma_block;
	unsigned ilast;

	for (dma_block = 0; dma_block != max_dma_blocks; ++dma_block){
		if (findEvent(phase, &isearch, &ilast)){

			dbg(1, "FOUND AT phase:%s 0x%08x ilast 0x%08x ",
			    phase->name, isearch, ilast);
			dbg(1, "FOUND AT phase:%s %10d ilast %10d", 
			    phase->name, isearch, ilast  );

			phase->start_off = ilast;
			phase->flags |= PH_FIXED_AT_START;
			if (PREV_PHASE(phase) != phase){
				shuffle_up(PREV_PHASE(phase), isearch);
				PREV_PHASE(phase)->flags |= PH_FIXED_AT_END;
			}
			DMC_WO->epos = isearch;
			DMC_WO->epos_found = 1;
			return 0;
		}
		isearch += DMA_BLOCK_LEN;
	}

	return -1;
}
#define MAX_SEARCH (8)
#define FIRST_TBLE(p) TBLE_LIST_ENTRY(&(p)->tblocks)

void search_epos(struct Phase* phase, int search_metric)
{
	unsigned isearch = phase->start_off;
	unsigned max_block = abs(search_metric)*MAX_SEARCH;
	unsigned tb0_end = ((isearch&TBLOCK_LEN)+ TBLOCK_LEN);

#define MAX_BLOCKS  min((tb0_end - isearch)/DMA_BLOCK_LEN, max_block)
	unsigned rem_blocks = max_block>MAX_BLOCKS? max_block - MAX_BLOCKS: 0;



	dbg(1, "01");
	info("");

	if (search_metric < 0){
		isearch += search_metric * DMA_BLOCK_LEN; /* decrement */
		isearch = max(0U, isearch);
	}

/** most likely: search forward in  TBLOCK 0 in phase */

	if (search_for_epos_in_tblock(phase, isearch, MAX_BLOCKS) == 0){
		info("SUCCESS in initial tblock");
		return;
	}else if (rem_blocks == 0){
		err("FAILED to find epos in tblock 0, over %d bytes consider "
		    "increasing epos search bounds", MAX_BLOCKS*DMA_BLOCK_LEN);
	}else if (isearch > 0){
	/** search in next tblock */
		if (get_tblock2_offset(phase, &isearch) != 0){
			err("ERROR: run out of TBLOCKS forward");
		}else{
			if (search_for_epos_in_tblock(
				    phase, isearch, rem_blocks) == 0){
				info("SUCCESS in tb[1]");
				return;
			}else{
				err("EPOS search Failed in tb[1]");
			}
		}
	}else if (PREV_PHASE(phase) != phase){
	/** steal previous tblock from previous phase and search there */
		struct Phase* prev = PREV_PHASE(phase);
		struct TblockListElement* tle;

		list_for_each_entry_reverse(tle, &prev->tblocks, list){
			if (tle->tblock != FIRST_TBLE(phase)->tblock){
				share_tblock(phase, tle, FIRST_TBLE(phase));
				if (search_for_epos_in_tblock(
					    phase,
					    tle->tblock->offset +
					    tle->tblock->length - rem_blocks,
					    rem_blocks) == 0){
					info("SUCCESS in tb[-1]");
					return;
				}else{
					err("EPOS search failed in tb[-1]");
				}
			}
		}
		err("previous phase has no more tblocks to share");
	}else{
		err("no previous phase available");
	}			
			    
	initPhaseDiagBufNotFound(search_metric, DMA_BLOCK_LEN*max_block); 
	err("event not found in %d bytes", DMA_BLOCK_LEN*MAX_SEARCH);
	
	dbg(1, "99");

#undef MAX_BLOCKS
}





int find_event_diag(char *buf, int maxbuf)
{
	formatPhaseDiagBuf(buf, maxbuf);
	return strlen(buf);
}



static inline unsigned bytesLeftInBurst(struct DataConsumerBuffer *dcb)
{
	int nleft;

	if ((nleft = (dcb->burst.len*sample_size() - dcb->nburst))){
		return nleft;
	}else{
		return 0;
	}
}



static void increment_scc(struct SampleClockCounter *scc)
/* called every 1K. increment scc accordingly */
{
	int lcp =       scc->scc_leap_clock_period;
	int delta_scc = scc->scc_per_block;

	if (lcp && ++scc->scc_leap_clock == lcp){
		delta_scc += 1;
		scc->scc_leap_clock = 0;
	}

	scc->scc += delta_scc;
	scc->scc_since_e2 += delta_scc;
}


static u32 esig[ESIG_LAST];

static void cook_es(struct DataConsumerBuffer *dcb, u32 my_esig)
{
	static int last_burst_too_fast;
	int blen = dcb->burst.len/NSUBFRAMES;
	int delta_burst_too_fast = 
		DG->stats.burst_events_too_fast - last_burst_too_fast;

	
	last_burst_too_fast = DG->stats.burst_events_too_fast;

	if (!IS_EVENT_MAGIC(my_esig)){
	    err("data at esig is NOT ESIG!");
	}

	esig[ESIG_ESIG]     = my_esig;
	esig[ESIG_ESIG+1]   = my_esig;
	esig[ESIG_JIFFIES] = (u32)jiffies;
	esig[ESIG_SCC]     = dcb->scc.scc;
	esig[ESIG_BURST_LEN] = max(1, blen);
	esig[ESIG_BURST_DLY] = dcb->burst.delay;
	esig[ESIG_E2_OFFSET] = dcb->scc.scc_since_e2;
	esig[ESIG_EDIO]      = 0xd10d10d0;
	esig[ESIG_FLAGS]     = delta_burst_too_fast;
	esig[ESIG_MFN]       = DG->pulse_number++;
	do_gettimeofday((struct timeval*)&esig[ESIG_TVS]);
	esig[ESIG_ESIG2]     = my_esig;
}


/* @@todo - eval the number */
#define IS_EVENT2(dcb, esig_esig) (((esig_esig)&EVENT_EVENT) == 1)

static int isE2(struct DataConsumerBuffer *dcb)
/* E2 is the marker event. Confusingly, it may actually start the WHOLE
 * process!. set ESOF and return true if found.
 */
{
	char *base = va_buf(DG);
	u32 *esig = (u32*)(base+dcb->last_start);
	u32 esig_esig = esig[ESIG_ESIG];

	if (IS_EVENT_MAGIC(esig_esig) && IS_EVENT2(dcb, esig_esig)){
		dcb->scc.scc_since_e2 = 0;
		return 1;
	}else{
		return 0;
	}
	
}


static int _streaming_tblock;

static ssize_t show_streaming_tblock(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", _streaming_tblock);
}

static DEVICE_ATTR(streaming_tblock, S_IRUGO, show_streaming_tblock, 0);


static void block_diag(struct DataConsumerBuffer *dcb) {
        if (AT_TBLOCK_START(dcb->last_start)){
		_streaming_tblock = (dcb->last_start>>20)/6;
        }
}


static void mindTheBlock(struct DataConsumerBuffer *dcb, int iput)
/**
 * mind the BLOCK!
 * (1) : most frequent, stay in the same block
 * (2) : iput is in the next block, so finish up this one
 * (3) : start new block
 */
{
#if 0
#define PDCB(p) \
	dbg(0, "(%d) tb1 %d tb2 %d st 0x%06x fin 0x%06x iput 0x%06x", \
        p, tb1, tb2, dcb->last_start, dcb->last_finish, iput)
#else
#define PDCB(p)
#endif

	int tb1 = TBLOCK_NUM(dcb->last_finish-1);     /* ensure round down */
	int tb2 = TBLOCK_NUM(iput);

	if (likely(tb1 == tb2)){
		dcb->last_finish = iput;                      /* (1) */
	}else if (dcb->last_finish < TBLOCK_LIMIT(tb1)){
		dcb->last_finish = TBLOCK_LIMIT(tb1);         /* (2) */
		PDCB(2);
	}else{
		dcb->last_start = TBLOCK_START(tb2);          /* (3) */
		dcb->last_finish = iput;
		PDCB(3);
	}
}


/* not sure formula for LCM !! */
#define LCM(a,b) (12288)

/*
 * bursted streaming;
 * assume event0 controls burst start, stop
 * assume event1 is an independent signal that we want to track.
 */

static ssize_t acq200_fpga_fifo_live_read_data(
        struct file *file, char *buf, size_t len, loff_t *offset)
{
	unsigned idst = 0;
	char *base = va_buf(DG);
#define MY_ESIG(dcb) (((u32*)(base+(dcb)->last_start))[0])
	struct DataConsumerBuffer *dcb = DCB(file);
	unsigned iput;
	unsigned ncp = 0;
	unsigned ssb = sample_size();
	int return_request = 0;

	dbg(1, "{%d} enter %d [%d,%d] %u", 
	    dcb->state, idst, dcb->rb.nput, dcb->rb.nget, *(u32*)offset);


	len &= ~3;

/* poss on the fly tweaks, DG->burst.delay == 1 => include trigger */
	dcb->burst.delay = DG->burst.delay + 1; 

	dbg(2, "off %ld len %d", (long)*offset, len);
	/*
         * first time through, catch up to data pointer
         */
	if (*offset == 0){
		wait_event_interruptible(
				dcb->waitq, !u32rb_is_empty(&dcb->rb));
		u32rb_get(&dcb->rb, &dcb->last_finish);
		dcb->last_start = TBLOCK_START(TBLOCK_NUM(dcb->last_finish));
		dma_sync_single(DG->dev,
				dcb->handle+dcb->last_start, 
				len, DMA_FROM_DEVICE);
	}

	block_diag(dcb);

	while(idst < len){
/*
 * pick up any residual first, if there is enough, return;
 */
		switch(dcb->state){
		case DCS_AT_EVENT:
			if (isE2(dcb)){
				dbg(1, "E2 detected pulse_init");
				ncp = 0;
				dcb->state = DCS_WAIT_EVENT_WORD;
#if CFG_LOW_JITTER_BURST_START == 0 
				pulse_start(&dcb->pdt);
#endif
				break;
			}else{
			/* 
                         * pass event as special one sample record
                         */
				cook_es(dcb, MY_ESIG(dcb));
				return_request = __LINE__;

				dcb->state = DCS_DISCARDING_DELAY;
				dcb->idelay = -1;

				dbg(2, "{%d} copy +%d %p %d", 
				    dcb->state, idst, esig, ESIG_LEN);

				if (copy_to_user(buf+idst, esig, ESIG_LEN)){
					return -EFAULT;
				}
				idst += ESIG_LEN;	
				goto finish_up;
			}
		case DCS_STREAMING_CONTINUOUS:
			if (*offset - dcb->continuous_frame_start_offset <
			    NSUBFRAMES * sample_size() ){
				ncp = min(len-idst, 
					  dcb->last_finish-dcb->last_start);
				ncp &= ~3;
				break;
			}else if (idst != 0){
				dcb->state = DCS_STREAMING_CONTINUOUS_ESIG;
				return_request = __LINE__;
				break;
			}
			/* else 
			 * fall thru to state immediately
			 * 
			 */
		case DCS_STREAMING_CONTINUOUS_ESIG:
			cook_es(dcb, EVENT_MAGIC);
			return_request = __LINE__;
			dcb->state = DCS_STREAMING_CONTINUOUS;
			if (*offset == 0){
				increment_scc(&dcb->scc);
			}else{
				dcb->continuous_frame_start_offset = *offset;
			}
			if (copy_to_user(buf, esig, ESIG_LEN)){
				return -EFAULT;
			}
			idst = ESIG_LEN;
			goto finish_up;
		default:
			ncp = 0;
			break;
		case DCS_DISCARDING_DELAY:
			while(dcb->idelay < dcb->burst.delay){
				dcb->idelay++;
				if (dcb->last_start + ssb <= dcb->last_finish){
					dcb->last_start += ssb;
				}else{
					goto wait_next_block;
				}
			}
			dcb->state = DCS_STREAMING_BURST;
			dcb->burst_number++;
			dcb->nburst = 0;
			// delay done ... fall thru
		case DCS_STREAMING_BURST:
			ncp = min(len-idst, dcb->last_finish-dcb->last_start);
			ncp = min(ncp, bytesLeftInBurst(dcb));
			ncp &= ~3;
			dcb->nburst += ncp; 
			
			if (dcb->nburst >= dcb->burst.len * ssb){
				/* cheating to change state now, but saves 
                                 * additional state eval 
				 */
				 
				dcb->state = DCS_WAIT_EVENT_WORD;
				dcb->nburst = 0;
				return_request = __LINE__;
			}
			break;
		}
		

		if (ncp){
			dbg(2, "{%d} copy +%d %p %d", 
			    dcb->state, idst, base+dcb->last_start, ncp);
/** stub here to test overhead of copy_to_user */
			if (!DG->stub_live_copy_to_user){
				if(copy_to_user(buf+idst, 
						base+dcb->last_start, ncp)){
					return -EFAULT;
				}
			}
			dcb->last_start += ncp;
			idst += ncp;
		}
		else{
			dbg(1, "{%d} ncp 0", dcb->state);
		}
	wait_next_block:
		if (idst >= len || return_request){
			dbg(1, "{%d} break %d %c %d",
			    dcb->state, idst, idst>=len? '<': '>',
			    return_request);

			return_request = 0;
			break;
		}
/*
 * else wait for more data 
 */
		do {
			dbg(3, "{%d} wait %s", dcb->state,
			    u32rb_is_empty(&dcb->rb)? "EMPTY": "NOT EMPTY");

			wait_event_interruptible(
				dcb->waitq, !u32rb_is_empty(&dcb->rb));
		
			if (!u32rb_get(&dcb->rb, &iput)){
				dbg(1,"interrupted");
				goto finish_up;
			}else{
				dbg(3, "{%d} get 0x%08x", dcb->state, iput);
			}

			if (iput&DMC_EVENT_MARKER){
				dbg(1,"{%d} trig %08x", dcb->state, iput);

				switch(dcb->state){
				default:
					DG->stats.burst_events_too_fast++;
					/* fall thru */
				case DCS_WAIT_EVENT_WORD:
					dcb->state = DCS_SEARCHING_FOR_EVENT;
					break;
				case DCS_STREAMING_CONTINUOUS:
/** @@todo This is a HACK: not sure how to handle events in CONTINUOUS */
					goto finish_up;
				case DCS_STREAMING_BURST:
#ifdef ACQ196
					if (iput&ACQ196_FIFSTAT_ADC_EV0){
						DG->stats.event0_too_early++;
					}
					if (iput&ACQ196_FIFSTAT_ADC_EV1){
						DG->stats.event1_too_early++;
					}
#endif
					dbg(1, "EVENT TOO FAST");
					dcb->state = DCS_SEARCHING_FOR_EVENT;
				        DG->stats.burst_events_too_fast++;
					goto finish_up;
				}
			}else{
				dbg(2, "{%d} sync", dcb->state);

				increment_scc(&dcb->scc);

				dma_sync_single(
					DG->dev, dcb->handle+iput, 
					DMA_BLOCK_LEN, DMA_FROM_DEVICE);
				
				switch(dcb->state){
				case DCS_SEARCHING_FOR_EVENT: {
					unsigned ifirst = iput;
					unsigned ilast;

/** @todo ... findEvent fast forwards in data. But we don't adjust SCC? 
    scc is surely valid for end of block because it's just been incremented
    above. maybe isE2 should set to zero, but to BLOCKEND - pos.
    .... but hti is only a minor effect...
*/
					if (findEvent(0, &ifirst, &ilast)){
						dcb->state = DCS_AT_EVENT;
						dcb->last_finish = 
						dcb->last_start  = ifirst;
					}
				}
				default:
					break;
				}
				if (fastforward){
					if ((iput & (sample_size()-1)) != 0){
						err("streaming channel swap "
						    "ALERT 0x%08x %d", 
						    iput, sample_size());
					}
					dcb->last_start = iput;
				}
				iput += DMA_BLOCK_LEN;
			}
		} while((dcb->state&_DCS_DATA_PHASE) == 0);

		mindTheBlock(dcb, iput);
	}

finish_up:
	*offset += idst;

	dbg(idst>0? 1: -1, 
            "{%d} return %d [%d,%d] %u iburst %d", 
	    dcb->state, idst, dcb->rb.nput, dcb->rb.nget, 
	    *(u32*)offset, dcb->nburst);

	return idst >0? idst: -EINTR;
}




static ssize_t acq200_fpga_fifo_live_read_offsets(
        struct file *file, char *buf, size_t len, loff_t *offset)
/* in this variation, we actually read all the data */
{
	u32 *ubuf = (u32*)buf;
	int n32 = len/sizeof(u32);
	int ibuf = 0;

	wait_event_interruptible(
		DCB(file)->waitq, !u32rb_is_empty(&DCB(file)->rb));
	
	while(ibuf < n32 && u32rb_get_user(&DCB(file)->rb, ubuf+ibuf)){
		++ibuf;
	}
	return ibuf * sizeof(u32);
}
static ssize_t acq200_fpga_fifo_live_write(
	struct file *file, const char *buf, size_t len, loff_t *offset)
{
	return -ENODEV;
}
static unsigned int acq200_fpga_fifo_live_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	poll_wait(file, &DCB(file)->waitq, poll_table );

	if (DMC_WO_getState() == ST_STOP || DMC_WO_getState() == ST_CAPDONE){
		return POLLHUP;
	}else if (!u32rb_is_empty(&DCB(file)->rb)){
	        return POLLIN | POLLRDNORM;
	}else{
		return 0;
	}
}



static int acq200_fpga_open (struct inode *inode, struct file *file)
{
#ifdef WAV232
	static struct file_operations fifo_write_buf_ops = {
		.open = acq200_fpga_fifo_write_open,
		.write = acq200_fpga_fifo_write_buf_write,
		.read = acq200_fpga_fifo_write_buf_read,        /* test hook */
		.release = acq200_fpga_fifo_write_buf_release
	};
#endif
#if defined(ACQ_IS_INPUT)
	static struct file_operations fifo_read_buf_ops = {
		.open = acq200_fpga_fifo_read_open,
/* testpattern is now a more sensible way of doing this ... */
		.write = acq200_fpga_fifo_write_buf_write,      /* test hook */
		.read = acq200_fpga_fifo_read_buf_read,
		.release = acq200_fpga_fifo_release
	};
#endif
	static struct file_operations fifo_write_buf_test_ops = {
		.open = acq200_fpga_fifo_write_open,
		.write = acq200_fpga_fifo_write_buf_write,
		.release = acq200_fpga_fifo_write_buf_nullrelease
	};
	static struct file_operations fifo_read_buf_test_ops = {
		.read = acq200_fpga_fifo_write_buf_read,
		.release = acq200_fpga_fifo_write_buf_nullrelease
	};
	static struct file_operations fifo_live_data_ops = {
		.open = acq200_fpga_fifo_live_open,
		.release = acq200_fpga_fifo_live_release,
		.read = acq200_fpga_fifo_live_read_data,
		.write = acq200_fpga_fifo_live_write,
		.mmap = acq200_mmap_bigbuf,
		.poll = acq200_fpga_fifo_live_poll
	};
	static struct file_operations fifo_live_offsets_ops = {
		.open = acq200_fpga_fifo_live_open,
		.release = acq200_fpga_fifo_live_release,
		.read = acq200_fpga_fifo_live_read_offsets,
		.write = acq200_fpga_fifo_live_write,
		.mmap = acq200_mmap_bigbuf,
		.poll = acq200_fpga_fifo_live_poll
	};

        int iminor = MINOR(file->f_dentry->d_inode->i_rdev);
	int rc = 0;

	dbg( 1, "iminor %d\n", iminor );

        file->f_op = acq200_fifo_get_bigbuf_datafops(iminor);

	if (file->f_op == 0){
		switch( iminor ) {
#ifdef WAV232
		case FIFO_WO_DEVBUF:
			file->f_op = &fifo_write_buf_ops;
			break;
#endif
#if defined(ACQ_IS_INPUT)
		case FIFO_RO_DEVBUF:
			file->f_op = &fifo_read_buf_ops;
			break;
#endif
		case FIFO_RO_DEVBUF_TEST:
			file->f_op = &fifo_read_buf_test_ops;
			break;
		case FIFO_WO_DEVBUF_TEST:
			file->f_op = &fifo_write_buf_test_ops;
			break;
		case FIFO_RW_LIVE_DATA:
			file->f_op = &fifo_live_data_ops;
			break;
		case FIFO_RW_LIVE_OFFSETS:
			file->f_op = &fifo_live_offsets_ops;
			break;
#ifdef ACQ132
		case ACQ132_SFPGA_LOAD:
			file->f_op = &acq132_sfpga_load_ops;
			break;
#endif
		default:
			return 0;
		}
	}

	if ( file->f_op->open ){
		rc = file->f_op->open( inode, file );
	}

	if ( rc == 0 ){
		DG->open_count++;
	}

	return rc;
}

static void init_endstops( int count )
/* call with count==0 to deallocate */
{
/*
 * this structure used to send a '1' to arm interrupt
 * need to have a 1 at offset [0], also need to keep pa for eventual free
 */
/*
 * reserve a buffer, need a u32 to write to fpga, need to know its pa =>
 * using struct iop321_dma_desc is just a convenience to get some 
 * consistent-mapped memory and to get its pa
 */
	static struct iop321_dma_desc *lbuf;

	struct iop321_dma_desc *dmad;
	int rc;
	int istop;

	if (count == 0){
		if (lbuf != 0){
			acq200_dmad_free(lbuf);
			lbuf = 0;
		}
		return;
	}

	if ( lbuf == 0 ){
		lbuf = acq200_dmad_alloc();
/*
 * separate enable for DAC ??? WORKTODO 
 */
		*(u32*)lbuf = ACQ_INTEN;
		/* assert(offsetof(lbuf->pa, lbuf) != 0) */
		dbg(1,"ICREN lbuf at %p pa 0x%08x value 0x%04x", 
		     lbuf, lbuf->pa, *(u32*)lbuf );
	}

	for (istop = 0; istop != count; ++istop){
		u32 inten = 0;
		dmad = acq200_dmad_alloc();

		if (!dmad){
			return;
		}

		if ((istop&EOC_INT_MODULO_MASK)==0 
#ifdef ACQ216
		    || live_one_frame_per_dcb
#endif
			){
			inten = IOP321_DCR_IE;
		}else{
			inten = 0;
		}
		dmad->DC = DMA_DCR_TODEVICE | inten;
		

		dmad->NDA = 0;
		dmad->PUAD = 0;
#ifdef ACQ196 /* mem to mem on PBI: source is PDA, dest is LAD */
		dmad->PDA = lbuf->pa;
		dmad->LAD =  DG->fpga.regs.pa+ACQ200_ICR_OFFSET;
#else		
		dmad->PDA = DG->fpga.regs.pa+ACQ200_ICR_OFFSET;
		dmad->LAD = lbuf->pa;
#endif
		dmad->BC = 4;

		rc = rb_put( &IPC->endstops, dmad );
	}
}




static void dig_pits(void)
{
#define PIT_DEF_SZ sizeof(struct PIT_DEF)
	if (DG->pit_store.max_pits == 0){
		DG->pit_store.max_pits = 1;
	}
	DG->pit_store.the_pits = 
		kmalloc(DG->pit_store.max_pits*PIT_DEF_SZ, GFP_KERNEL);
}

static void fill_pits(void)
{
	if (DG->pit_store.max_pits){
		kfree(DG->pit_store.the_pits);
		DG->pit_store.the_pits = 0;
		DG->pit_store.max_pits = 0;
	}
}



char acq200_fpga_driver_name[] = "acq200_fpga";
char acq200_fpga_driver_string[] = "D-TACQ fpga device";
char acq200_fpga_driver_version[] = "1.1";
char acq200_fpga_copyright[] = "Copyright (c) 2003 D-TACQ Solutions Ltd";



static int __devinit map_local_resource(void)
{
	int bigbuf_len = acq200_get_bigbuf_resource(&DG->bigbuf.resource);

	dbg( 1, "bigbuf_len 0x%08x", bigbuf_len );


	IPC->empties.name = "empties";
	IPC->active.name  = "active";
	IPC->endstops.name = "endstops";

	acq200_rb_init(&IPC->empties, RBLEN);
	acq200_rb_init(&IPC->active,  RBLEN);
	acq200_rb_init(&IPC->endstops, RBLEN);
       

	IPC->is_dma = acq200_dma_getDmaChannelSync();

	DMA_REQUEST_IRQ(IPC->is_dma, EOT, 0, eot);
	DMA_REQUEST_IRQ(IPC->is_dma, EOC, 0, eoc);
	DMA_REQUEST_IRQ(IPC->is_dma, EOT, 1, eot);
	DMA_REQUEST_IRQ(IPC->is_dma, EOC, 1, eoc);


	IPC->is_dma[0].eoc.isr_cb = fifo_dma_irq_eoc_callback;
	IPC->is_dma[1].eoc.isr_cb = regular_dma_irq_eoc_callback;



#ifdef PGMCOMOUT
	IPC->is_dma[0].eoc.tasklet = &acq200_eoc_tasklet0;
#endif
	IPC->is_dma[1].eoc.tasklet = &acq200_eoc_tasklet1;

#ifdef PGMCOMOUT
	acq200_eoc_tasklet0.data = (unsigned long)&IPC->is_dma[0].eoc;
#endif
	acq200_eoc_tasklet1.data = (unsigned long)&IPC->is_dma[1].eoc;


	init_waitqueue_head(&IPC->finished_waitq);

	acq200_tblock_init_top();
	dig_pits();
	return bigbuf_len? 0: -ENOMEM;
}



static void free_rb_pool(struct acq200_dma_ring_buffer *rb)
{
	acq200_rb_drain(rb);
	acq200_free_rb(rb);
}

static void unmap_local_resource(void)
{
	DMA_FREE_IRQ(IPC->is_dma, eot, 0);
	DMA_FREE_IRQ(IPC->is_dma, eoc, 0);
	DMA_FREE_IRQ(IPC->is_dma, eot, 1);
	DMA_FREE_IRQ(IPC->is_dma, eoc, 1);

	clear_buffers();
	init_endstops(0);

	free_rb_pool(&IPC->empties);
	free_rb_pool(&IPC->active);
	free_rb_pool(&IPC->endstops);

	fill_pits();
	acq200_tblock_remove();
	/* no resources created by acq200_get_bigbuf_resource() */
}



static int __devinit
run_fpga_mknod_helper( int major )
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};

	static char arg1[10];
	static char arg2[10];
	static char *argv[4];
	int i;

	sprintf(arg1, "%d", major);
	sprintf(arg2, "%d", NCHAN);

        i = 0;
        argv[i++] = "/sbin/acq200_fpga_helper";
        argv[i++] = arg1;
	argv[i++] = arg2;
        argv[i] = 0;

	dbg( 1, "call_usermodehelper %s\n", argv[0] );

	i = call_usermodehelper(argv [0], argv, envp, 0);

        dbg( 1, "call done returned %d", i );
	return 0;
}






/*
 * tblock device: purely a debug hook to the internal tblocks
 */
static int acq200_tblock_open (struct inode *inode, struct file *file)
{
	int iblock = MINOR(file->f_dentry->d_inode->i_rdev);
	if (iblock > DG->bigbuf.tblocks.nblocks ){
		return -ENODEV;
	}else{
		return 0;
	}
}

static ssize_t acq200_tblock_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	int iblock = MINOR(file->f_dentry->d_inode->i_rdev);
	struct TBLOCK *tblock = &DG->bigbuf.tblocks.the_tblocks[iblock];
	int boffset = (int)(*offset);
	int maxread = min(len, tblock->length - boffset);
	char *bs = va_buf(DG) + tblock->offset + *offset;

	if (copy_to_user(buf, bs, maxread)){
		return -EFAULT;
	}	
	*offset += maxread;
	return maxread;
}

static int acq200_tblock_release (
	struct inode *inode, struct file *file)
{
	return 0;
}




static int __devinit
acqX00_fpga_driver_init(struct device *dev, int irq)
{
	static struct file_operations fpga_fops = {
		.open = acq200_fpga_open,
		.release = acq200_fpga_release
	};
	static struct file_operations tblock_fops = {
		.open = acq200_tblock_open, 
		.read = acq200_tblock_read,
		.release =  acq200_tblock_release
	};
	int rc = map_local_resource();

	info(VERID);

	if (rc){
		err( "map_local_resource() failed" );
		return rc;
	}

	rc = register_chrdev( DG->major = 0, "acq200-fpga", &fpga_fops );

	if (rc < 0){
		err( "can't get major %d", DG->major );
		return rc;
	}else{
		DG->major = rc;
		info( "device major set %d\n", DG->major );
	}

	rc = register_chrdev(0, "acq200-tblocks", &tblock_fops);
	if (rc < 0){
		err( "acq200-tblocks can't get major %d", rc );
		return rc;
	}else{
		info( "tblock device major set %d\n", rc );
	}

	create_proc_entries();
	run_fpga_mknod_helper( DG->major );

	acq200_init_interrupt_hook( irq, &IPC->is_fpga, 0 );

	if (DG->use_fiq){
		DG->use_fiq = set_fpga_isr_use_fiq(1);
	}

	DTACQ_MACH_DRIVER_INIT(dev);	
	DG->dma_block_len = DMA_BLOCK_LEN;
	return 0;
} 

extern void acq200_fixup_irqs(struct pci_dev* dev);


int init_arbiter(void)
{
	*IOP321_ATULT = 0xf0;         /* Maximum Latency Timer */
	*IOP321_IACR  = 0x00000482;   /* CORE=MED DMA1=LO ATU=LO */
	*IOP321_MTTR1 = 0x08;         /* minimise bus grant to core */
#ifdef MTTR2
	*IOP321_MTTR2 = MTTR2;        /* Max int bus grant for DMAC */
#endif
	return 0;
}


static irqreturn_t fpga_fifo_isr(int irq, void *dev_id)
/*
 * ISR
*/
{
	return IRQ_HANDLED;
}

int __devinit
acqX00_fpga_probe(struct device *dev, int irq)
{
	DG->dev = dev;

/*
 * SA_INTERRUPT means "don't unmask ints during action" (was "fast interrupt")
 * sounds like a good idea to me
 */
	if (!IPC->is_fpga.requested){
		if ( request_irq( irq, fpga_fifo_isr, 0, 
				  "fpga_fifo", (void*)&IPC->is_fpga ) != 0 ){
			err( "request_irq() failed for %d\n", irq );
			return -ENODEV;
		}else{
			IPC->is_fpga.requested = 1;
		}
	}

	switch(acq200_data_word_size){
	case 0:
		break;
	case 2:
	case 3:
	case 4:
		CAPDEF_set_word_size((acq200_data_word_size));
		break;
	default:
		err("acq200_data_word_size %d != {2,3,4} are you sure?",
		    acq200_data_word_size);
		break;
	}

	mk_dev_sysfs(dev);
	DEVICE_CREATE_FILE(dev, &dev_attr_streaming_tblock);

	if (acq200_aichan){
		acq200_fifo_create_AIfs(dev, acq200_aichan);
	}
	return acqX00_fpga_driver_init(dev, irq);
}

static void acqX00_fpga_remove (struct device *dev, int irq)
{
	if (IPC->is_fpga.requested){
		free_irq( irq, (void*)&IPC->is_fpga );
		IPC->is_fpga.requested = 0;
	}

	dbg( 1, "going down, unregister %d", DG->major );
	unregister_chrdev( DG->major, "acq200-fpga" );


	delete_proc_entries();
	unmap_local_resource();

}


static void init_dg(void)
{
	DMC_WO = kzalloc(sizeof(struct DMC_WORK_ORDER), GFP_KERNEL);
	IPC = kzalloc(sizeof(struct IPC), GFP_KERNEL);
	DG = kzalloc(sizeof(struct DevGlobs), GFP_KERNEL);

	INIT_LIST_HEAD(&DMC_WO->stateListeners);
	INIT_LIST_HEAD(&DMC_WO->phases);
	memcpy(DG, MYDG, sizeof(struct DevGlobs));
	DG->ipc = IPC;
	DG->wo = DMC_WO;
	DG->dcb.lock = SPIN_LOCK_UNLOCKED;

	acq200_transform_init();
	DG->bigbuf.tblocks.transform = acq200_getTransformer(2)->transform;

	DG->cdog_max_jiffies = CDOG_MAX_JIFFIES;
	INIT_LIST_HEAD(&DG->dcb.clients);
	spin_lock_init(&DG->dcb.lock);

	DMC_WO->getNextEmpty = GET_NEXT_EMPTY;
	DMC_WO->handleEmpties = dmc_handle_empties_default;

	INIT_LIST_HEAD(&DG->start_of_shot_hooks);
	INIT_LIST_HEAD(&DG->end_of_shot_hooks);

	INIT_LIST_HEAD(&DG->tbc.clients);
	spin_lock_init(&DG->tbc.lock);

	INIT_WORK(&onEnable_work, onEnableAction);
}

static void delete_dg(void)
{
	dmc0_start(0);
	acq200_transform_destroy();
	kfree(DMC_WO);
	kfree(IPC);
	kfree(DG);
}

/** 
 *  @@null_signal - default signal class
 */
static int def_commit(struct Signal* signal)
{
	dbg(1, "");
	return 0;
}

struct Signal* createSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
)
{
	static struct Signal def_signal = {
		.is_active = 0,
		.commit = def_commit
	};
	struct Signal* signal = kzalloc(SIGNAL_SZ, GFP_KERNEL);
	memcpy(signal, &def_signal, SIGNAL_SZ);
	strncpy(signal->name, name, sizeof(signal->name)-1);
	signal->_minDIx = minDIx;
	signal->_maxDIx = maxDIx;
	signal->DIx = DIx;
	signal->rising = rising;
	signal->is_active = is_active;
	if (commit){
		signal->commit = commit;
	}
	return signal;
}

void destroySignal(struct Signal* signal)
{
	kfree(signal);
}







void CAPDEF_set_nchan(int nchan)
{
	CAPDEF->_nchan = nchan;
}
void CAPDEF_set_word_size(int ws)
{
	CAPDEF->_word_size = ws;
}

void acq200_setDO6_bit(int ibit, int value)
{
	unsigned control;
	unsigned bit;

	assert(ibit >= 0 && ibit < 6);

	bit = 1 << (ibit + ACQ200_DIOCON_OUTDAT_SHL);

	control = *ACQ200_DIOCON;

	if (value){
		control |= bit;
	}else{
		control &= ~bit;
	}

	*ACQ200_DIOCON = control;
}


static void alertStateListeners(enum STATE s) 
{
	struct StateListener* sl;
	struct timeval ts;
	u32 scode = (u32)jiffies;

	do_gettimeofday(&ts);
	scode = (ts.tv_sec % (3600 * 24)) * 100;
	scode += ts.tv_usec/10000;
	scode |= s << 28;

	/* now alert any listeners */
	list_for_each_entry(sl, &DMC_WO->stateListeners, list){
		u32rb_put(&sl->rb, scode);
		wake_up_interruptible(&sl->waitq);
	}
}

void DMC_WO_setState(enum STATE s) {

	DMC_WO->_state = s;
	alertStateListeners(s);
}

EXPORT_SYMBOL_GPL(acq200_check_entire_es);
EXPORT_SYMBOL_GPL(acq200_setDO6_bit);
EXPORT_SYMBOL_GPL(CAPDEF_set_nchan);
EXPORT_SYMBOL_GPL(CAPDEF_set_word_size);
EXPORT_SYMBOL_GPL(acq200_registerTransformer);
EXPORT_SYMBOL_GPL(acq200_unregisterTransformer);
EXPORT_SYMBOL_GPL(acq200_getTransformer);
EXPORT_SYMBOL_GPL(acq200_setTransformer);
EXPORT_SYMBOL_GPL(acq200_resetBigBufCursor);

EXPORT_SYMBOL_GPL(acq200_add_start_of_shot_hook);
EXPORT_SYMBOL_GPL(acq200_del_start_of_shot_hook);
EXPORT_SYMBOL_GPL(acq200_add_end_of_shot_hook);
EXPORT_SYMBOL_GPL(acq200_del_end_of_shot_hook);
EXPORT_SYMBOL_GPL(acq200_add_ext_phase_handler);
EXPORT_SYMBOL_GPL(acq200_del_ext_phase_handler);


EXPORT_SYMBOL_GPL(DMC_WO);
EXPORT_SYMBOL_GPL(CAPDEF);
EXPORT_SYMBOL_GPL(DG);
