/* ------------------------------------------------------------------------- */
/* acq200-fifo.h internal defs for acq200 fifo streaming                     */
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


#ifndef __ACQ200_FIFO_H__
#define __ACQ200_FIFO_H__


#define FIQDEBUG 0
#define ACQ200_FIFO_H_VERSION 5

#define ACQ200_FIFO_H_VERSION_STR "acq200-fifo.h $Revision: 1.23 $"

#include <linux/list.h>
#include "ringbuffer.h"


struct TblockListElement;

#define IN_RANGE(xx, ll, rr) ((xx)>=(ll)&&(xx)<=(rr))

#define USS (sizeof(unsigned))
/*
 * WORKTODO: use a header file please
 */
enum STATE { ST_STOP, ST_ARM, ST_RUN, ST_TRIGGER, ST_POSTPROCESS, ST_CAPDONE };

enum MODE { 			
	M_GATED_TRANSIENT, 
	M_GATED_CONTINUOUS,  
	M_SOFT_TRANSIENT,
	M_SOFT_CONTINUOUS,
	M_TRIGGERED_CONTINUOUS,
	M_LAST
};

/*
 * ppmu - functionality supplied by iop321-ppmu.c
 */
extern void iop321_start_ppmu(void);
extern void iop321_stop_ppmu(void);


struct BurstDef {
	unsigned len;
	short delay;     /* set to -1 to see EVENT */
	short es_channel;/* set non-zero to embed event in this channel */
};

struct PulseDef {
/* optional pulse generation */
	int pulse_count;        /* 0 => disabled */
	int ibit;               /* DIO bit to toggle */
	int active_high;         
	int delay_ms;           /* inter pulse delay in msecs */
	int start_delay;        /* delay before start */
};

struct PulseDefTimer {
	struct PulseDef pulse_def;
	struct timer_list timer_list;
	enum {
		PDT_WAITING_EVENT = 1,
		PDT_ACTIVE,
		PDT_DELAY
	} state;
	int pulses_left;
};


void pulse_init(struct PulseDefTimer* pdt);
void pulse_start(struct PulseDefTimer* pdt);
void pulse_close(struct PulseDefTimer* pdt);


struct SampleClockCounter {
	unsigned long long scc;
	unsigned scc_per_block;
	unsigned scc_leap_clock;
	unsigned scc_leap_clock_period;  /* 96 ch doesn't fit block size */
	unsigned scc_since_e2;
};


struct TblockConsumer {
	wait_queue_head_t waitq;
	struct list_head list;	       /* list of fellow consumers */
	struct list_head tle_q;	       /* queue of TblockListElements */
	struct CURRENT {
		struct TblockListElement *tle;
		unsigned cursor;
	} c;
	unsigned flags;
};

struct DataConsumerBuffer {
	wait_queue_head_t waitq;
	struct list_head list;
	struct u32_ringbuffer rb;
	void (*early_action)(struct DataConsumerBuffer* dcb, u32 fifstat);
	dma_addr_t handle;
	int flow_control_active;
	int overrun;
	unsigned last_start;
	unsigned last_finish;
	struct BurstDef burst;
	int nburst;                  /* bytes in burst */
	int idelay;                  /* sample number in delay */

	unsigned burst_number;

	struct SampleClockCounter scc;
	loff_t continuous_frame_start_offset;

	enum {
		DCS_WAIT_EVENT_WORD = 1,
		DCS_SEARCHING_FOR_EVENT,
		_DCS_DATA_PHASE = 0x10,
		DCS_AT_EVENT,
		DCS_DISCARDING_DELAY,
		DCS_STREAMING_BURST,
		DCS_STREAMING_CONTINUOUS,
		DCS_STREAMING_CONTINUOUS_ESIG
	} state;
	struct PulseDefTimer pdt;


};

struct StateListener {
	wait_queue_head_t waitq;	/* StateListener blocks here */
	struct u32_ringbuffer rb;	/* Q of state values */
	struct list_head list;		/* List of StateListners */
};

#define ACQ200_FPGA_REG_BAR  0
#define ACQ200_FPGA_FIFO_BAR 1


#define MAXPREBUILT 8

struct PrebuiltChain {
	int length;
	int fifo_to_local;          /* indexes the_chain */
	int local_to_host;          /* indexes the_chain */
	struct iop321_dma_desc* the_chain[MAXPREBUILT];
	struct iop321_dma_desc desc;            
	void (* insert)(
		struct PrebuiltChain *_this, 
		struct iop321_dma_desc* _new);
	char id[MAXPREBUILT];
};

#define getPBChain(dp) (container_of((dp), struct PrebuiltChain, desc))
#define isPBChainDesc(dp) ((void*)getPBChain(dp) == (dp)->clidat)
#define PBC_SZ (sizeof(struct PrebuiltChain))


struct IPC {
/* first 3 used by FIQ */
	struct acq200_dma_ring_buffer empties; /* empty descrs => FIQ */
	struct acq200_dma_ring_buffer active;  /* active descrs FIQ=>EOC */
	struct acq200_dma_ring_buffer endstops;/* clears interrupt */

	struct InterruptSync is_fpga;
	struct DmaChannelSync* is_dma; /* -> array[2] */

	wait_queue_head_t finished_waitq;
};

/*
 * methods of remote phase-handling object
 */
struct ExtPhase {
	void (*init)(void);
	void (*release)(void);
};

/* PIT: Post Init Trigger (ie EVENT) */
struct Phase {
	/* len, off: units of BYTES sizeof(unit) == 1 */

	unsigned long long start_after;/* start after this sample number */
	int required_len;            /* client asked for this */
	int demand_len;              /* but we have to gather this amount */
	int actual_len;
	int is_oneshot;

	u32 start_off;
	u32 end_off;

	/* PREP only:sample: units of SAMPLES sizeof(unit) == sample_size() */
	unsigned start_sample;
	unsigned demand_samples;
	unsigned actual_samples;

	short event_count;
	struct Signal* ev;          /* Event terminates Phase */
	unsigned long lap_count;
	unsigned lap_position;
	short is_orig;
	struct list_head tblocks;   /* list of  TblockListElement s*/
	int tblock_count;
	int tblock_max_count;
	unsigned flags;
	struct list_head list;      /* list of Phases */
	struct Phase* orig;         /* for backup purposes */
	struct Phase* (* onPIT)(struct Phase *phase, u32 status, u32* offset);
	void (* onPhaseComplete)(struct Phase* phase);
	char name[16];

	unsigned long long ref_start_scc;  /* prep start ref mapping a */
	unsigned ref_offset;               /* prep start ref mapping b */
	struct timeval prep_start_time;    /* wall clock prep start    */
	unsigned transformer_private;
};
#define PHASE_SZ sizeof(struct Phase)


#define PH_FIXED_AT_START 0x1
#define PH_FIXED_AT_END   0x2

#define PHASE_LIST_ENTRY(ptr) (list_entry(ptr, struct Phase, list))
#define VALID_PLE(ptr, head) ((ptr) != head? PHASE_LIST_ENTRY(ptr): 0)

#define NEXT_PHASE(phase)     VALID_PLE(phase->list.next, &DMC_WO->phases)
#define PREV_PHASE(phase)     VALID_PLE(phase->list.prev, &DMC_WO->phases)



static inline unsigned phase_end_sample(struct Phase *phase)
{
	return phase->start_sample + phase->actual_samples;
}

struct TBLOCK;

struct TblockListElement {
/*
 * structure owned by phase, and maps tblock data to phase
 */
	struct TBLOCK* tblock;
	unsigned phase_sample_start; /* sample offset of first sample in ph */
	unsigned tblock_sample_start;/* sample offset of first sample in tb */
	unsigned sample_count;       /* number of samples in tb             */
	struct list_head list;
};

#define TBLE struct TblockListElement
#define TBLE_LIST_ENTRY(ptr) (list_entry(ptr, struct TblockListElement, list))

static inline unsigned tble_phase_end_sample(struct TblockListElement* tble)
{
	return tble->phase_sample_start + tble->sample_count;
}

struct BDA {
	unsigned before;
	unsigned during;
	unsigned after;		
	enum { BDA_IDLE, BDA_BEFORE, BDA_DURING, BDA_AFTER, BDA_DONE } state;
};


struct DMC_WORK_ORDER {
/*** WARNING: cleanzone gets scrubbed every shot */
	int direction;
	void *buf;
	dma_addr_t pa;
	enum STATE _state;
	int wo_len;     /* WORKTODO - what is the meaning of this? */
	int finished_code;  /* 1: OK, negative => fail */
	int next_empty;
	int next_load;
	int triggered;
	int oneshot;
	int (*trigger_detect)(void);
	struct {
		spinlock_t lock;
		int done;
	} onTrigger;
	char* error;

	int pit_stop;     /* stop after this number of pits */
	int pit_count;
	int looking_for_pit;  /* look for pit */

	struct list_head phases;
	struct Phase* now;
	struct Phase* pre;
	struct Phase* post;

	int epos_found;     /* Event POSition found */
	unsigned long bb_lap_count;

	struct list_head prep_phases;   /* list of PREP's */
	struct Phase* prep_now;         /* current PREP   */

	/** count samples by block */
	struct SampleClockCounter scc;  /* sample clock count */
	struct SampleClockCounter ecc;  /* external clock count */

	/** count clocks with hardware counter */
	unsigned clock_count_immediate;
	unsigned clock_count_latched;
	unsigned epos;

	unsigned clean_to_here;
/*** end of cleanzone */
	/** Before During After phase control */
	unsigned (*getNextEmpty)(struct DMC_WORK_ORDER* _this);
	void (*handleEmpties)(struct DMC_WORK_ORDER* _this);

	struct BDA bda_blocks;

	struct CONTROL_TARGET {
		unsigned pa_data;
		unsigned data_blocks;
		unsigned pa_status;
		unsigned stride;
		unsigned subsample;
		int iodd;           
	} control_target;

	struct list_head stateListeners;
};


#define CLEAN_LEN offsetof(struct DMC_WORK_ORDER, clean_to_here)
#define DMC_CLEAN(wo) (memset(wo, 0, CLEAN_LEN))

unsigned default_getNextEmpty(struct DMC_WORK_ORDER* _this);
unsigned bda_getNextEmpty(struct DMC_WORK_ORDER* _this);
#ifdef WAV232
unsigned wav232_getNextEmpty(struct DMC_WORK_ORDER* wo);
#endif


void onEvent(struct DMC_WORK_ORDER *wo, u32 status, u32* offset);

#define DMC_EVENT_MARKER 0x1

/* action on trigger detect */

typedef int (*DataMover)(
	struct TBLOCK *this,
	short* ubuf, int maxbuf, 
	int channel, int offset, 
	int stride);

typedef void* (*Memcpy)(void* to, const void* from, __kernel_size_t len);
/*
 * class TBLOCK - a block of data in bb
 */
struct TBLOCK {
	int iblock;          /* index of this block */
	unsigned offset;     /* byte offset in bb */
	unsigned length;     /* length in bytes   */
	unsigned locked;     /* replace with sem  WORKTODO */
	unsigned touched;    /* touched by transform iterator */
	atomic_t in_phase;
	DataMover extract;
	DataMover fill;
	Memcpy memcpy;
};

#define NHISTO 16

#define BTYPE_AI 0
#define BTYPE_AO 1

#define BTYPE_ACQ216 BTYPE_AI
#define BTYPE_WAV232 BTYPE_AO


#define TB_IN_PHASE(tb) (atomic_read(&tb->in_phase))

/*
 * data within a TBLOCK may be munged using one a Transformer:
 */

#define TF_RESULT_IS_RAW	0x0001	/* use raw extract even after tform */
#define TF_INPLACE		0x0002  /* transform is inplace so no BLT   */

struct Transformer {
	char name[16];
	unsigned t_flags;
	void (*transform)(short *to, short *from, int nwords, int stride);
	/* stride in words */
};


/*
 * interface to hook user Transformers
 */
int acq200_registerTransformer(const struct Transformer* transformer);
void acq200_unregisterTransformer(const struct Transformer* transformer);
const struct Transformer* acq200_getTransformer(int it);
void acq200_setTransformer(int it);
void acq200_resetBigBufCursor(void);

void acq200_transform_init(void);
void acq200_transform_destroy(void);

#define MAX_DMA_BLOCKS 8

#define MAX_TBLOCK_POOL (1024/6)       /** maximal tblock sharing case .. */

#define CDOG_MAX_JIFFIES  500   /* 5secs 1k/64bytes = 16 => 4Hz min rate */

struct pci_mapping {
	int len;
	unsigned pa;
	void* va;
};


struct ArgBlock {
	void *base;
	char** argv;
	int argc;
};

struct DevGlobs {
	int btype;
	int major;
	int open_count;
	int ifill;
	dma_addr_t dma_handle;
	struct device* dev;
	int direction;
	int hitide;
	int lotide;
	int max_alloc;

	unsigned CAFEBABE;
	struct IPC *ipc;
	struct DMC_WORK_ORDER *wo;

	u32 FIFERR;   /* FIFO ERRor mask */
	u32 fiferr;   /* fifo error condition */
	struct iop321_dma_desc *head;

	int load_two_blocks_if_half;
	unsigned DEADBEEF;
	u32 istack[16]; /* private to ISR - may stash regs here */
	unsigned FEEDCODE;
	int enable_from_eoc_isr;
	int bh_unmasks_eoc;
	int use_ob_clock;          /* ACQ216 On Board Clock */

	unsigned global_irq_mask;

	struct fpga {
		struct pci_mapping regs;
		struct pci_mapping fifo;
		struct pci_mapping extra;  /** ACQ216 DSP extension */
	} fpga;

	struct stats {
/* position the fields used by FIFO ISR first */
/* BEWARE: this gets translated to asm. be very afraid */
		int num_fifo_ints;
		int dma_blocks[MAX_DMA_BLOCKS];		
		unsigned cold_fifo_histo[NHISTO];
		unsigned hot_fifo_histo[NHISTO];
		unsigned hot_fifo_histo2[NHISTO];
		unsigned cold_fifo_histo2[NHISTO];		
/* OK: this stuff used from C only */
		unsigned long long refill_blocks;
		unsigned long start_jiffies;
		unsigned long end_jiffies;
		
		unsigned long start_gtsr;
		unsigned long end_gtsr;

		int num_eoc_ints;
		int num_eoc_bh;
		int num_eoc_bh2;
		int num_eoc_nomatches;
		int num_dmc_run;
		int busy_pollcat;
		int cdog_trips;
		u32 dmac_flags[2];
		int starve_line;
		unsigned starve_fifcon;
		int burst_events_too_fast;
		int local_pulse_count;
		int event0_count;         /* events we wanted */
		int event1_count;
		int event0_too_early;
		int event1_too_early;
		int event0_count2;        /* and events we did not */ 
		int event1_count2;

		int dcb_flow_control_throttled_count;
		int dcb_flow_control_event_discards;
		int dcb_flow_control_discards;

		struct timespec finish_time;

		unsigned long sendfile_bytes;
		struct BDA bda_times;		
	} stats;
	int shot;
	
	int use_fiq;
	int finished_with_engines;
	int busywait;

	unsigned sample_read_start;
	unsigned sample_read_stride;
	unsigned sample_read_length;

	int show_event;

	struct BIGBUF {
		spinlock_t tb_list_lock;	/* protects these lists: */
		struct list_head free_tblocks;  /* unallocated */
		struct list_head empty_tblocks; /* DMA_BLOCKS in empties Q */
		struct list_head pool_tblocks;  /* pool of tblock wrappers */
		struct resource resource;
		const struct Transformer** transformers;
		struct TBLOCKLIST {
			struct resource tmp;
			int blocklen;
			int nblocks;
			struct TBLOCK* the_tblocks;
			
			int cursor;
			int cursor_complete;


			void (*blt)(short *to, short *from, int nshorts);
			void (*transform)(
				short *to, short *from, 
				int nwords, int stride);
			unsigned t_flags;
		} tblocks;
	} bigbuf;

	struct PIT_STORE {
		int max_pits;
		struct PIT_DEF {
			u32 status;
			u32 offset;
		} *the_pits;
	} pit_store;

	unsigned mbox_abort_mask;
	unsigned mbox_abort_value;

	int simulate;

	int is_oneshot;

	struct DCB {
		int dcb_max;
		int dcb_max_backlog;
		spinlock_t lock;
		struct list_head clients;
	} dcb;

	struct BurstDef burst;
	struct PulseDef pulse;
/*
 * pulse number exists for the life of the module.
 * when in pulse mode, it increments every pulse.
 * it may be queried and set to any initial value by app software
 */
	unsigned long pulse_number;

	unsigned empty_fill_threshold;
	unsigned put_max_empties;
	unsigned get_max_active;
	unsigned active_batch_threshold;
	unsigned init_endstops;
	unsigned eoc_int_modulo_mask;

	struct ExtPhase *ext_phase;
	struct list_head start_of_shot_hooks;
	struct list_head end_of_shot_hooks;

	unsigned cdog_max_jiffies;
	int activate_event_on_arm;
	int stub_live_copy_to_user;    /* for profile by stubbing */

	int slow_clock;     /* eoc too slow to service 1MHz count */
	int dma_block_len;   /* copy of DMA_BLOCK_LEN for diags    */
	/* NB: use the constant for efficiency (FIQ does)         */

	struct BDA bda_samples;


	struct ArgBlock pre_arm_hook;
	struct ArgBlock post_arm_hook;
	struct ArgBlock post_shot_hook;	

	struct TblockClients {
		spinlock_t lock;
		struct list_head clients;
	} tbc;

	struct RefillClient {
		spinlock_t lock;
		void (* client)(void *data);
	} refillClient;
};

#define INDEXOF_TBLOCK(tblock) ((tblock) - DG->bigbuf.tblocks.the_tblocks)
#define VA_TBLOCK(tblock) (va_buf(DG) + (tblock)->offset)
#define MAX_TBLOCK (DG->bigbuf.tblocks.nblocks)

struct CAPDEF_PRIVATE;  /* opaque store used by subclass */

#define ACQ196_AO_HISTO cold_fifo_histo

/*
 * class Signal - generic per device, per function line handling
 * external interface deals in logical values, internal is board, func
 * specific
 */
struct Signal {
	char name[16];
	int is_active;
	int was_active;
	int DIx;                
	int rising;

	int _minDIx, _maxDIx;
	int has_internal_option;
	int is_output;

	int (*commit)(struct Signal* signal);
};

#define DIX_INTERNAL -2
#define DIX_NONE -1

struct Signal* createSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
);
/**
 * createSignal
 * name, minDIx, maxDIx, DIx, rising, is_active, commit
 */


void destroySignal(struct Signal* signal);
#define SIGNAL_SZ (sizeof(struct Signal))


#define createNullSignal() createSignal("null",0,0,0,0,0,0)

static inline int setSignal(struct Signal* signal, int DIx, int rising)
{
	if (DIx == DIX_NONE){
		signal->DIx = DIX_NONE;
		return 0;
	}else if (IN_RANGE(DIx, signal->_minDIx, signal->_maxDIx)){
		signal->DIx = DIx;
		signal->rising = rising;
		return 0;
	}else{
		return -1;
	}
}

static inline int enableSignal(struct Signal* signal, int enable)
{
	if (signal->DIx != DIX_NONE){
		return signal->is_active = enable;
	}else{
		return 0;
	}
}
static inline int signalCommit(struct Signal* signal)
{
	if (signal->DIx != DIX_NONE){
		return signal->commit(signal);
	}else{
		return 0;
	}
}


static inline void activateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	enableSignal(signal, 1);
	signalCommit(signal);
}
static inline void deactivateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	signal->was_active = signal->is_active;
	enableSignal(signal, 0);
	signalCommit(signal);
}
static inline void reactivateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	enableSignal(signal, signal->was_active);
	signalCommit(signal);
}





struct CAPDEF {
	int demand_len;
	int demand_postlen;
	int demand_prelen;
	unsigned channel_mask;
	int _nchan;
	int _word_size;
	enum MODE mode;
	struct Signal* ev[2];
	struct Signal* trig;
	struct Signal* ext_clk;
	struct Signal* int_clk_src; /* EXT SOURCE for int_clk */
	struct Signal* mas_clk;     /* derived clock - OUTPUT */
	int pit_stop; 

	struct Signal* ao_trig;    /* RTM_AO16, if fitted */
	struct Signal* ao_clk;     /* RTM_AO16, if fitted */

	struct Signal* sync_trig_src;    /* ACQ196 FAWG */
	struct Signal* sync_trig_mas;    /* ACQ196 FAWG */
	struct CAPDEF_PRIVATE *private;
#ifdef ACQ216
	struct Signal* ob_clk_src;
	int pipeline_offset[4];       /* offset from trigger pos, samples */
#endif
#if defined(ACQ196) || defined(ACQ216) || defined(ACQ132)
	struct Signal* counter_src;
	int counter_update;
#endif
};


static inline void* va_buf( struct DevGlobs* dg )
{
	return (void*)dg->bigbuf.resource.start;
}
static inline short* va_buf_s( struct DevGlobs* dg )
{
	return (short*)dg->bigbuf.resource.start;
}

static inline unsigned pa_buf( struct DevGlobs* dg )
{
	return virt_to_phys( (void*)dg->bigbuf.resource.start );
}

static inline unsigned pfn_buf( struct DevGlobs* dg )
{
	return pa_buf(dg) >> PAGE_SHIFT;
}
static inline void* va_buf_offset( struct DevGlobs* dg, int offset )
{
	return (void*)(dg->bigbuf.resource.start+offset);
}
static inline unsigned len_buf( struct DevGlobs* dg )
{
	return dg->bigbuf.resource.end - dg->bigbuf.resource.start;
}

static inline void* va_tblock_tmp(struct DevGlobs* dg )
{
	return (void*)dg->bigbuf.tblocks.tmp.start;
}

#define BB_PTR(offset)  (va_buf(DG) + offset)

#define RBLEN          0x4000
#define RBMASK         (RBLEN-1)
#define RB_IS_EMPTY( rb ) ((rb).iput==(rb).iget)
#define RB_INCR( ii )  (((ii)+1)&RBMASK)
#define RB_IS_FULL( rb )  (RB_INCR((rb).iput)==(rb).iget)

#define RB_WILL_BE_EMPTY(rb) ((rb).iput==RB_INCR((rb).iget))

/* next is an approximation - nput, nget will overflow */
#define RB_ELEMENT_COUNT(rb) ((rb).nput - (rb).nget)

#define CHECK_TIDES(rb)					\
do {							\
	unsigned short tide = rb->nput - rb->nget;	\
	if (tide > rb->hitide){				\
		rb->hitide = tide;			\
	}else if (tide < rb->lotide){			\
		rb->lotide = tide;			\
	}						\
} while(0)

#define INIT_TIDES(rb)					\
do {							\
	rb->hitide = rb->lotide = rb->nput - rb->nget;	\
} while(0)

static inline int rb_put( 
	struct acq200_dma_ring_buffer *rb, 
	struct iop321_dma_desc *buf )
{
	if ( !RB_IS_FULL( *rb ) ){
		rb->buffers[rb->iput] = buf;
		rb->iput = RB_INCR(rb->iput);
		rb->nput++;
		CHECK_TIDES(rb);
		return 1;
	}else{
		rb->lotide = 0;
		return 0;
	}
}


static inline int rb_get( 
	struct acq200_dma_ring_buffer* rb, 
	struct iop321_dma_desc** pbuf )
{
	if ( !RB_IS_EMPTY( *rb ) ){
		CHECK_TIDES(rb);
		*pbuf = rb->buffers[rb->iget];
		rb->iget = RB_INCR( rb->iget );
		rb->nget++;
		return 1;
	}else{
		rb->lotide = 0;
		return 0;
	}
}


static inline struct iop321_dma_desc* rb_get_buf(
	struct acq200_dma_ring_buffer* rb
	)
{
	struct iop321_dma_desc* pbuf = 0;

	if ( !RB_IS_EMPTY( *rb ) ){
		CHECK_TIDES(rb);
		pbuf = rb->buffers[rb->iget];
		rb->iget = RB_INCR( rb->iget );
		rb->nget++;
	}else{
		rb->lotide = 0;
	}
	
	return pbuf;
}




/*
 * entry to acq200-fifo-procfs.c
 */

extern void mk_dev_sysfs(struct device* dev);
extern int mk_sysfs(struct device_driver *driver);
extern void rm_sysfs(struct device_driver *driver);

extern void create_proc_entries(void);
extern void delete_proc_entries(void);
/*
 * Global Hook
 */

extern struct DevGlobs *DG;
extern struct DMC_WORK_ORDER *DMC_WO;
extern struct CAPDEF *CAPDEF;

void DMC_WO_setState(enum STATE s);

static inline enum STATE DMC_WO_getState(void) {
	return DMC_WO->_state;
}

/*
 * WORDSZ - sample size - almost always 16 bit except when dsp involved
 */

#define WORDSZ     (sizeof(short))
//#define NCHAN      (CAPDEF->nchan) 
#define LEN        (CAPDEF->demand_len) 

static inline int _sample_size(void)
{
	assert(CAPDEF->_nchan);
	assert(CAPDEF->_word_size);

	return CAPDEF->_nchan * CAPDEF->_word_size;
}
#ifndef WAV232
#define sample_size() _sample_size()
#else
int sample_size(void);
#endif

void CAPDEF_set_nchan(int nchan);
void CAPDEF_set_word_size(int ws);

static inline int CAPDEF_get_nchan(void) {return CAPDEF->_nchan; }

#ifdef ACQ216
static inline int CAPDEF_get_pipeline_offset(void) {
	return CAPDEF->pipeline_offset[(CAPDEF_get_nchan()/4)-1];
}
#endif

static inline void capdef_set_nchan(struct CAPDEF* capdef, int nchan)
{
	capdef->_nchan = nchan;
}
static inline void capdef_set_word_size(struct CAPDEF* capdef, int ws)
{
	capdef->_word_size = ws;
}
static inline int capdef_get_word_size(void)
{
	return CAPDEF->_word_size;
}


static inline int get_nchan(void)
{
	return CAPDEF->_nchan;
}

#define NCHAN get_nchan()

static inline int samplesToBytes(int samples)
{
	return samples * sample_size();
}
extern int acq200_clk_hz;

extern void acq200_setIntClkHz( int hz );
extern void acq200_setChannelMask(unsigned mask);

extern void acq200_reset_fifo(void);
int set_fpga_isr_use_fiq(int use_fiq);

int acq200_fifo_bigbuf_transform(int blocknum);

void blt_memcpy(short *to, short *from, int nwords);
void blt_dma(short *to, short *from, int nwords);



void acq200_fifo_set_bigbuf_read_method_raw(int raw);
int acq200_fifo_get_bigbuf_read_method_raw(void);

void acq200_transform_mk_sysfs(struct device_driver *driver);
void acq200_transform_rm_sysfs(struct device_driver *driver);

#define ACQ200_FPGA_REGS (DG->fpga.regs.va)

#define MAXRETRY 3


static inline void nsleep(int nsecs)
{
#define NSECS_PER_GTSR (1000/50)
	u32 startgtsr = *IOP321_GTSR;
	u32 maxgtsr = nsecs/NSECS_PER_GTSR;
	u32 gtsr;

	for(nsecs /= 10; nsecs; nsecs--){   /* extra delay if GTSR stopped */
		gtsr = *IOP321_GTSR;
		
		if (gtsr - startgtsr > maxgtsr){  /* normal case */
			break;
		}else if (startgtsr > gtsr){      /* rollover case */
			if (gtsr > maxgtsr){
				break;
			}
		}
	}
}


static inline short *tblock_va(int blocknum)
{
	unsigned block_off = blocknum * DG->bigbuf.tblocks.blocklen;

	return (short *)(va_buf(DG)) + block_off/sizeof(short);
}


#define USEC_DELAY_ONEUSEC 400 /* 400MIPS - good guess! */

static inline void usec_delay(unsigned usecs)
/* busy wait delay : NB, needs calibration */
{
	volatile int iwait;

	while(usecs-- > 0){
		for (iwait = USEC_DELAY_ONEUSEC; iwait; --iwait){
			;
		}
	}
}


/*
 * TBLOCK_LEN: pick a number divisible by 24, 32, 4096 ...
 */

#define TBLOCK_LEN 0x600000   


#define TBLOCK_INDEX(offset) ((offset)/TBLOCK_LEN)
#define TBLOCK_OFFSET(offset)  ((offset)%TBLOCK_LEN)
#define IN_TBLOCK(tblock, offset) (TBLOCK_INDEX(offset) == (tblock)->iblock)



static inline int phase_len(struct Phase *phase)
{
//	return min(phase->actual_len, phase->demand_len);
	return phase->actual_len;
}

static inline int phase_num_samples(struct Phase *phase)
{
	return phase_len(phase)/sample_size();
}
static inline int phase_full(struct Phase *phase)
{
	return phase_len(phase) >= phase->demand_len;
}
static inline int phase_end(struct Phase *phase)
{
	return phase->is_oneshot && phase_full(phase);
}



static inline int get_tblock_max_sam(void) 
{
	return DG->bigbuf.tblocks.blocklen/sample_size();
}

#define tb_incr(ib) ((ib)+1 >= DG->bigbuf.tblocks.nblocks? 0: (ib)+1)

#define foreach_block(ib, sb, eb, nb) \
        for(ib = sb; ib != tb_incr(eb) || nb == 0; ib = tb_incr(ib), ++nb)



#define SAMPLES_PRE    (phase_len(DMC_WO->pre)/sample_size())
#define SAMPLES_POST   (phase_len(DMC_WO->post)/sample_size())
#define SAMPLES (SAMPLES_PRE+SAMPLES_POST)


extern void acq200_pipe_fiq(void);
extern void acq200_pipe_fiq_end(void);

#define PIPE_FIQ_LEN ((char*)acq200_pipe_fiq_end - (char*)acq200_pipe_fiq)


#define ABS(a) ((a)<0? -(a): (a))

static inline int getTblockMaxSam(void)
{
	return DG->bigbuf.tblocks.blocklen/sample_size();
}
static inline char *tle2string(struct TblockListElement* tle)
{
	static char buf[80];

	sprintf(buf, "pss:%8u tbss:%8u tbsc:%8u", 
		tle->phase_sample_start, 
		tle->tblock_sample_start,
		tle->sample_count);

	return buf;
}


struct Hookup {
	void (*the_hook)(void *clidata);
	void *clidata;
	struct list_head list;	
};

#define DEFHOOKUP(name, hook, data)			\
	struct Hookup name = {				\
		.the_hook = hook, .clidata = clidata,	\
	}
		

void acq200_add_start_of_shot_hook(struct Hookup *hook);
void acq200_del_start_of_shot_hook(struct Hookup *hook);
void acq200_add_end_of_shot_hook(struct Hookup *hook);
void acq200_del_end_of_shot_hook(struct Hookup *hook);
void acq200_add_ext_phase_handler(struct ExtPhase *ext_phase);
void acq200_del_ext_phase_handler(struct ExtPhase *ext_phase);


#define ENO_PHASE  3737
#define ENO_TBLOCK 3738

struct BigbufReadPrams {
	struct TBLOCK *tblock;
	int my_samples_reqlen;
	int block_off_sample;
	int samples_left_in_block;
	int status;
	DataMover extract;   /** extractor to use this time */
	int tblock_samples;  /* == this->length/sample_size()/stride; */
	unsigned offsam;
};

#define BBRP_COMPLETE 1

void acq200_initBBRP_using_phase(
	struct file* file, size_t len, 
	unsigned offset_samples,
	struct BigbufReadPrams* bbrp,
	struct Phase* phase,
	struct TblockListElement* tble);



/*
 * size of single channel sample, row sample
 */
#define CSIZE (CAPDEF->_word_size)
#define RSIZE sample_size()
#define SSZ CSIZE

/**
 * per path data structure definition
 */
struct DataChannelInfo {
	int lchan;                    /** logical channel {1..N} */
	int pchan;                    /** physical offset {0..N-1} */
	int ssize;                    /** sample size              */
	DataMover extract;            /** optional custom extractor */
	Memcpy memcpy;                /** memory copy method (def:memcpy() */
	union {
		void *clidat;
		struct Phase* phase;          /** phase (if known) */
		struct TblockConsumer* tbc;
	} _u;
	unsigned flags;
	struct TblockListElement *tle_current;
};

#define DCI_FLAGS_NORELEASE_ON_READ 0x00000001

#define DCI(file) ((struct DataChannelInfo *)file->private_data)
#define DCI_PHASE(file) (DCI(file)->_u.phase)
#define DCI_TBC(file)	(DCI(file)->_u.tbc)

#define DCI_SZ (sizeof(struct DataChannelInfo))

void acq200_initDCI(struct file *file, int lchannel);
void acq200_releaseDCI(struct file *file);

int acq200_fifo_bigbuf_xx_open (struct inode *inode, struct file *file);
int acq200_fifo_bigbuf_xx_release(struct inode *inode, struct file *file);

static inline unsigned get_fileOffsetSamples(
	struct file* file, 
	loff_t *offset,
	unsigned stride,
	int sample_size)
{
	if (offset == 0){
		return DG->sample_read_start;
	}else{

		unsigned offw = *(unsigned*)offset/sample_size;
/*
 * offset is the byte offset in the FILE - translate to sample offset in BB
 *
 * (1) sample_read_start (obviously)
 * (2) translate the file offset to samples
 */
		unsigned offset_samples = 
			DG->sample_read_start +                   /* (1) */
			offw*stride;                              /* (2) */

		return offset_samples;
	}
}

ssize_t acq200_fifo_bigbuf_xxX_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset,
	struct BigbufReadPrams* bbrp
	);

#define CDOG_INIT 0
#define CDOG_REFRESH 1

int acq200_cdog(int mode);
void finish_with_engines(int ifinish); /* call when end detected */


struct DataConsumerBuffer * acq200_createDCB(void);
void acq200_deleteDCB(struct DataConsumerBuffer *dcb);
int acq200_addDataConsumer(struct DataConsumerBuffer *dcb);
int acq200_removeDataConsumer(struct DataConsumerBuffer *dcb);

#ifdef ACQ216
int acq216_setTrig(int enable, int dix, int rising);
int acq216_setAntiPhase(int enable);  /* ACQ216HS, sets channelMask too */
#endif

/** this is hack, but may end up becoming permanent :-) */

#define TBLOCK_NUM(addr) (((addr)>>21)/3)
#define TBLOCK_START(tb) (((tb)*3) << 21)
#define TBLOCK_LIMIT(tb) (TBLOCK_START(tb)+ACQ200_TBLOCK_SIZE)


extern int acq200_fifo_part_transform(struct Phase* phase);
extern void acq200_short_transform(
	short *to, short *from, int nwords, int stride, int nsamples);

extern void run_pre_arm_hook(void);
extern void run_post_arm_hook(void);
extern void run_post_shot_hook(void);

extern int show_hook(struct ArgBlock *argBlock, char* buf, int maxbuf);
extern int store_hook(
	struct ArgBlock *argBlock, const char* buf, int count);

extern struct proc_dir_entry* proc_acq200;

#endif /* ACQ200_FIFO_H__ */
