/* ------------------------------------------------------------------------- */
/* acq100-llc.c driver for acq100 lowlatency controller                      */
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

#define ACQ196

#define WRITE_LAST_AO_SINGLE 0
#define CHECK_DAC_UPDATE     0
#define USE_GTSR             0
#define COUNT_CLEARS_ON_TRIG 1
#define AVOID_HOT_UNDERRUN   0
#define USE_DBG              0
#define CHECK_DAC_DMA_ERRORS 0
#define SYNC2V               1
#define DMA_POLL_HOLDOFF     1
#define FIFO_ONESAM          0
#define DI_IN_1              0
#define CYCLE_STEAL	     1
#define AO32CPCI	     1
#define WDT		     1

/** @file acq100-llc.c acq1xxx low latency control kernel module.
 *  DMA should fire on HOT_NE, and it should be IMPOSSIBLE to get a 
 *  HOT UNDERRUN. However, this does happen 1/1000 times
 *  AVOID_HOT_UNDERRUN plays it safe by waiting for all COLD FIFOS to empty 1st
 * Module Design
 *
 * This driver is the "low latency controller" 
 * It builds on services provided by the acq100-fifo.c device driver
 *

- ECM:

- TRIG : DI3 mezz

- CLK : DI0 mezz => source for Internal Clock Gen, output on DO1

 * sync2V : two chain sync mode - for maximum reprate, IO data is aggregated 
 * so that there are only two DMA transfers 
 * - One Input Vector:
 *  - AI
 *  - DI32
 *  - TLATCH, DI6, STATUS, TLATCH
 *
 * - One Output Vector:
 *  - AO
 *  - DO32
 *
 * - NB DI32, DO32 are overlaid, the direction is configured externally
 *

 * @todo - can we do the marshalling and tee the DMA inside 2 usecs?.
 * 
 *
 * - extensions: IODD may be applied.
 * - IO_GAP : inserts a programmable GAP between AI and AO to allow an external
 *	CPU to process Ai data and output to AO in the same cycle.
 *	IO_GAP is tunable: 0: disabled, 1..TBLOCK_LEN (6M) delay
 * - AI_DELAY : inserts a programmable DELAY before AI.
 *      In a multi-card system, forces one ACQ to delay to avoid congestion
 *      with two-card action.
 *      Ideally, SB will go first with VI[S], then MB will do VI[M], VO
 * - DO64_READBACK: insert lower 32 bits AO32#1 DO64 value into  VI
 *      aim is to provide a tuning aid that enables a host program 
 *	to confirm that the last VO made the cut on previous cycle
 *   Value == 1 : use DMA, Value == 2 : use PIO
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
#define acq200_debug acq100_llc_debug   

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
#define DMACINTMASK0 \
	(IRQ_IOP321_DMA0_EOT|IRQ_IOP321_DMA0_EOC|IRQ_IOP321_DMA0_ERR)
#define DMACINTMASK1 \
	(IRQ_IOP321_DMA1_EOT|IRQ_IOP321_DMA1_EOC|IRQ_IOP321_DMA1_ERR)



int acq100_llc_debug;
module_param(acq100_llc_debug, int, 0664);

int acq100_llc_dump_ao2bb;
module_param(acq100_llc_dump_ao2bb, int , 0664);

int disable_acq_debug = 0;         /* set 1 */
module_param(disable_acq_debug, int, 0664);

/** set to non-zero for 2V mode */
int acq100_llc_sync2V = 0;
module_param(acq100_llc_sync2V, int, 0664);

#define ACQ100_LLC_SYNC2V         1
#define ACQ100_LLC_SYNC2V_DI      2
#define ACQ100_LLC_SYNC2V_DIDO    3
#define ACQ100_LLC_SYNC2V_DIDOSTA 4

#define MAXSAMPLES_CYCLE	5
int sync2v_samples_per_cycle = 1;
module_param(sync2v_samples_per_cycle, int, 0644);

/** recommend: set to 1<<5 = 0x20 to toggle DO5
 * use RTM-CLK
 * set.sys /dev/rtmclk/LEMO_OUT_4 DO5
 * this wd is controlled by HOST action and is different to wdt_action
 */
int host_wd_mask = 0;
module_param(host_wd_mask, int, 0644);

int use_adc_clkdet = 1;
module_param(use_adc_clkdet, int, 0644);

u32 fifo_clocked;
int DI_offset =
#if DI_IN_1
		1;			/** workaround for old FPGA bug */
#else
		LLC_SYNC2V_IN_DI32;
#endif
module_param(DI_offset, int, 0644);

#define FIFO_NE       (ACQ196_FIFSTAT_HOT_NE)
#define FIFO_CLOCKED0  (ACQ196_FIFSTAT_HOT_NE|ACQ196_FIFSTAT_ADC_CLKDET)
#define FIFO_CLOCKED1	FIFO_NE
#define AIFIFO_NOT_EMPTY(fifstat)     (((fifstat)&FIFO_NE) != 0)
#define COLDFIFO_EMPTY(fifstat)       (((fifstat)&dg.coldfifo_ne_mask)==0)

/* SYNC2V SCRATCHPAD */

#define I_SCRATCH_HWTLAT ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_TLATCH];
#define I_SCRATCH_TLAT32 ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_TLAT32]
#define I_SCRATCH_TINST	 ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_TLAT32]
#define I_SCRATCH_ITER ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_ITER]
#define I_SCRATCH_SC   ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_SCOUNT]
#define I_SCRATCH_FSTA ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_FIFSTA]
#define I_SCRATCH_DI32 ACQ196_LL_AI_SCRATCH[DI_offset]
#define I_SCRATCH_LASTE ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_LASTE]
#define I_SCRATCH_DORB  ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_DO64]

/** FIFO write:
 *  0x00..0x20 hot copies to AO16
 *  0x20..0x24 is IOP-readable here:
 *  */
#define O_SCRATCH_DO32  ACQ196_LL_AO_SCRATCH[0]

#define PA_SCRATCH(off_words) \
	(DG->fpga.regs.pa + ACQ196_LL_AI_SCRATCH_OFF + off_words*sizeof(u32))



int acq100_llc_sync2V_AO_len = 0;
module_param(acq100_llc_sync2V_AO_len, int, 0664);

/** one cycle "see how fast it can be mode" */
int acq100_mem2mem = 0;
module_param(acq100_mem2mem, int, 0664);

int acq100_dropACQEN_on_exit = 1;
module_param(acq100_dropACQEN_on_exit, int, 0664);

/* writes initial values to AO, DO on LLC entry (Sync2V only) */
int init_Xo_on_entry = 0;
module_param(init_Xo_on_entry, int, 0664);

#if CYCLE_STEAL
int pbi_cycle_steal = 0;
module_param(pbi_cycle_steal, int, 0664);
#endif

#define USE_MAX_SAMPLES_IGNORE	0
#define USE_MAX_SAMPLES_INTEN	1 /* enable interrupts after MAX_SAMPLES. */
#define USE_MAX_SAMPLES_DISABLE 2 /* disable acquisition. 		  */
#define USE_MAX_SAMPLES_QUIT	3 /* quit LLC (may break client           */
int use_max_samples = 0;
module_param(use_max_samples, int, 0644);

int MAX_SAMPLES = 0;
module_param(MAX_SAMPLES, int, 0644);

/** increment BUILD and VERID each build */
#define BUILD 1109
#define VERID "BUILD 1109"

char acq100_llc_driver_name[] = "acq100-llc";
char acq100_llc_driver_string[] = "D-TACQ Low Latency Control Device";
char acq100_llc_driver_version[] = VERID " "__DATE__ " Features:\n"
#if WRITE_LAST_AO_SINGLE
"WRITE_LAST_AO_SINGLE "
#endif
#if USE_GTSR
"GTSR "
#else
"TCR "
#endif
#if CHECK_DAC_UPDATE
"CHECK_DAC_UPDATE "
#endif
#if (COUNT_CLEARS_ON_TRIG==0)
"SOFTWARE_TO "
#endif
#if AVOID_HOT_UNDERRUN
"AVOID_HOT_UNDERRUN "
#endif
#if COUNT_CLEARS_ON_TRIG
"COUNT_CLEARS_ON_TRIG "
#endif
#if USE_DBG
"USE DEBUG "
#endif
#if CHECK_DAC_DMA_ERRORS
"CHECK_DAC_DMA_ERRORS "
#endif
"\n"
#if SYNC2V
"** SYNC2V ** "
#endif
#if DMA_POLL_HOLDOFF
"DMA_POLL_HOLDOFF "
#endif
#if FIFO_ONESAM
"FIFO_ONESAM "
#endif
#if CYCLE_STEAL
"ACQ196-500 "
#endif
#if AO32CPCI
"AO32CPCI"
#endif
#if WDT
"WDT "
#endif
"32 bit HW TLATCH"
"\n";

#define DBG dbg


char acq100_llc_copyright[] = "Copyright (c) 2004-2011 D-TACQ Solutions Ltd";


/** size of fifo extension for sync2v */
#define SCRATCHPAD_SIZE (32*sizeof(short)) 
#define AO_SCRATCHPAD_SIZE (4)


#define REPORT_ERROR(fmt, arg...) \
    sprintf(dg.status.report, "%d " fmt, dg.status.errline = __LINE__, arg)

#define REPORT_ERROR_AND_QUIT(fmt, arg...) \
        REPORT_ERROR(fmt, arg); goto quit

#define REPORT_AND_QUIT(fmt, arg...) \
        sprintf(dg.status.report, "%d " fmt, __LINE__, arg); goto quit

/**
 *   @@todo WORKTODO
 */

#define FIFO_DATA          0
#define LLC_CSR_IS_ARMED   1 


#define AOCHAN 16

#define CHANNEL_MASK (CAPDEF->channel_mask)


#define AO32_TMP_MAXLEN \
	(LLC_SYNC2V_AO32*sizeof(u32) + AO32_VECLEN*LLCV2_INIT_AO32_MAX)
#define AO32_TMP_LEN(n_ao32) \
	(LLC_SYNC2V_AO32*sizeof(u32) + AO32_VECLEN*(n_ao32))

static int getNumChan(u32 cmask){
	int nchan = 0;

	if (cmask&1) ++nchan;
	if (cmask&2) ++nchan;
	if (cmask&4) ++nchan;

	return nchan * 32;
}
static struct LlcDevGlobs {
	int emergency_stop;
	struct LlcPrams {
		enum { ECM = 1, SCM = 2 } m_mode;
		int clkpos;
		int trpos;
		int intsoftclock;
		int divisor;
		int simulate;
	} prams;
	unsigned imask;
	unsigned coldfifo_ne_mask;
	int sample_size;
	int dac_lowlat;
	int sync_output;                  /* ao, do sync with ai */
	unsigned hb_mask;		  /* host buffer size limit */

/* @@todo - publish SETTINGS thru /sys/.../settings (ro) */
	struct SETTINGS {                /* user specified things */
		int decim_base;               
		unsigned PUAD;
		unsigned AI_target;      /* current pci target addr */
		unsigned auto_incr;

		unsigned DI_target;      /* pci addr DI0-5 */
		/* copy 4 mailbox status info to host if NZ */		
		unsigned STATUS_target;      /* pci addr target status [4] */

		unsigned AO_src;         /* pci addr AO (for fixed scenario) */
		unsigned DO_src;         /* pci addr DO32 */
		/* Interrupt on Dma Done */
		int iodd; /* doorbell interrupt to host if set*/
		int soft_trigger;
		u32 dma_poll_holdoff; /* hold off dma poll 1 unit = 5nsec */

		struct AO32SETTINGS {
			u32 *tmp;
			u32 tmp_pa;
			int count;
			u32 pa[LLCV2_INIT_AO32_MAX];
		} ao32;

		unsigned alt_VI_target;
		unsigned io_gap;
		unsigned ai_delay;
		unsigned do64_readback;

		struct WDT_CONTROL {
			unsigned preset;
			unsigned toggle;
			unsigned cleanup;
		}
			wdt;

	} settings;

	u32 *llcv2_init_buf;
	int ai_dma_index;
	
	/* STATUS gets cleared on entry */
	struct STATUS {
		int is_triggered;
		unsigned iter;
		u32 t0;
		u32 tinst;
		u32 tlatch;
		u32 sample_count;
		char report[128];
		int errline;
		int waiting_for_permission_to_quit;
		unsigned fifo_poll_count;
		unsigned dmac_poll_count[2];
		unsigned dma_holdoff_poll_count;
		struct COMMANDS {
			unsigned SETADDR, ARM, SOFTCLOCK, READCTR;
		} cmd_counts;
		unsigned hot_starts;
		int wdt_preset_done;
	} status;

	int shot;
} dg = {
	.settings.decim_base = 1,
	.dac_lowlat = 1
};

#define DEADBAND ((volatile u32*)0xdeadbeef)

static volatile u32 * MBOX[] = {
	IOP321_IMR0,
	IOP321_IMR1,
	IOP321_OMR0,
	IOP321_OMR1,
	DEADBAND,
	DEADBAND,
	DEADBAND,
	DEADBAND
};


#define AO_BC ((AOCHAN-2*WRITE_LAST_AO_SINGLE) * sizeof(short))

static unsigned get_sample_size(unsigned mask)
/* @@todo - surely there is a generic func for this? */
{
	unsigned nblocks = 0;

	for(; mask != 0; mask >>= 1){
		if ((mask &1) != 0){
			++nblocks;
		}
	}
	return nblocks * 32 * 2;
}

static int __ints_disabled;

int llc_intsDisable(void)
{
	MASK_ITERATOR_INIT(it, dg.imask);

	if (__ints_disabled == 1) return 0;

	while ( mit_hasNext( &it ) ){
		disable_irq_nosync( mit_getNext( &it ) );
	}
	__ints_disabled = 1;
	return 0;
}

int llc_intsEnable(void)
{
	MASK_ITERATOR_INIT(it, dg.imask);

	if (__ints_disabled == 0) return 0;

	while ( mit_hasNext( &it ) ){
		enable_irq( mit_getNext( &it ) );
	}

	__ints_disabled = 0;
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

static int llc_onEntryEcm(int entry_code)
/** customisation for ECM mode. */
{
	u32 clkcon = *ACQ196_CLKCON;
	clkcon |= ACQ196_CLKCON_LLSYNC;
	*ACQ196_CLKCON = clkcon;
	*ACQ200_CLKDAT = dg.prams.divisor;

	activateSignal(CAPDEF->int_clk_src);
	activateSignal(CAPDEF->mas_clk);

	*ACQ196_TCR_IMMEDIATE = CAPDEF->int_clk_src->DIx; /** index frm 0 */

	acq196_syscon_clr_all(	
		ACQ196_SYSCON_SIM_MODE  |
		ACQ196_SYSCON_EC_RISING |
		ACQ196_SYSCON_EC_MASK
	);

	activateSignal(CAPDEF->ext_clk);
	acq196_syscon_set_all(
		(dg.prams.simulate? ACQ196_SYSCON_SIM_MODE: 0) |
		ACQ196_SYSCON_LOWLAT
	);
	if (dg.dac_lowlat){
		acq196_syscon_dac_set(ACQ196_SYSCON_LOWLAT);
	}else{
		acq196_syscon_dac_clr(ACQ196_SYSCON_LOWLAT);
	}

	return 0;
}

static int llc_onEntryScm(int entry_code)
/** customisation for SCM mode */
{
	return 0;
}

static void createAO_tmp(void)
{
	/* allocate ao32 (VO) temp area regardless of whether there are
	 * AO32 or not (used in host_wd_action() and do64 loopback
	 */
	dg.settings.ao32.tmp = kmalloc(AO32_TMP_MAXLEN, GFP_KERNEL);
	dg.settings.ao32.tmp_pa = dma_map_single(
			NULL, dg.settings.ao32.tmp, 
			AO32_TMP_MAXLEN, PCI_DMA_BIDIRECTIONAL);
	dbg(1, "tmp %p tmp_pa 0x%08x ao32_count %d",
			dg.settings.ao32.tmp, 
			dg.settings.ao32.tmp_pa, dg.settings.ao32.count);
}

static void pullAO32_targets(u32 *buf)
{
	int ii = 0;

	for (; ii < LLCV2_INIT_AO32_MAX &&
		buf[LLCV2_INIT_AO32PA0+ii] != 0; ++ii){
		dg.settings.ao32.pa[ii] = buf[LLCV2_INIT_AO32PA0+ii];
		dbg(2, "ao32.%d at 0x%08x", ii, dg.settings.ao32.pa[ii]);
	}
	dg.settings.ao32.count = ii;
	dbg(1, "tmp %p tmp_pa 0x%08x ao32_count %d",
			dg.settings.ao32.tmp,
			dg.settings.ao32.tmp_pa, dg.settings.ao32.count);
}
#define BLEN (max(LLCV2_INIT_LAST,LLCV2_INIT_AO32_LAST)*sizeof(u32))


static int checkLLCV2_INIT(void)
/** if A4 is non zero, then it could be an init block. 
 * reel it in and have a look.
 */
{
	u32 remaddr = *MBOX[BP_MB_A4];
	int rc = 0;
	if (remaddr){
		/** need to copy in via DMA. use a consistent map - not 
		    speed critical
		*/		
		u32 *buf = dg.llcv2_init_buf;
		dma_addr_t dmabuf = dma_map_single(NULL, buf, BLEN,
						   PCI_DMA_FROMDEVICE);

		acq200_post_dmac_request(1|DMA_CHANNEL_POLL, 
					 dmabuf, 0, remaddr, BLEN, 1);

		dma_unmap_single(NULL, dmabuf, BLEN, PCI_DMA_FROMDEVICE);

		dg.settings.ao32.count = 0;
		createAO_tmp();

		switch(buf[LLCV2_INIT_MARKER]){
		case LLCV2_INIT_MAGIC_AO32:
			pullAO32_targets(buf);
			/* fall thru */
		case LLCV2_INIT_MAGIC_MARKER:
			dg.settings.AI_target = buf[LLCV2_INIT_AI_HSBT];
			dg.settings.AO_src    = buf[LLCV2_INIT_AO_HSBS];
			dg.settings.DO_src    = buf[LLCV2_INIT_DO_HSBS];
			dg.settings.STATUS_target = 
				buf[LLCV2_INIT_STATUS_HSBT];
			break;
		default:
			REPORT_ERROR("LLCV2_INIT not MAGIC 0x%08x\n",
				     buf[LLCV2_INIT_MARKER]);
			rc = -1;
		}
	}

	return rc;
}
static int llc_onEntry(int entry_code)
/**
 * @returns 0 => OK to run
 */
{
	int rc;

	++dg.shot;
	memset(&dg.status, 0, sizeof(dg.status));
	sprintf(dg.status.report, "llc_onEntry()");

	rc = checkLLCV2_INIT();
	if (rc){
		return rc;
	}

	llc_intsDisable();

	dac_reset();
	dac_arm();

	disable_acq();

	deactivateSignal(CAPDEF->ev[0]);
	deactivateSignal(CAPDEF->ev[1]);

	*ACQ200_ICR = 0;              /* polling only */
	disable_fifo();
	reset_fifo();
	enable_fifo(CHANNEL_MASK);
	set_fifo_ne_mask(CHANNEL_MASK);
	dg.sample_size = get_sample_size(CHANNEL_MASK);
	dg.hb_mask = acq200mu_get_hb_mask();
	

	if (use_adc_clkdet){
		fifo_clocked = FIFO_CLOCKED0;
	}else{
		fifo_clocked = FIFO_CLOCKED1;
	}
	info("fifo_clocked mask: 0x%08x %s",
			fifo_clocked, use_adc_clkdet? "CLKDET": "NE");

	/**
	 * store FPGA state
	 */

	switch(dg.prams.m_mode){
	case ECM:
		llc_onEntryEcm(entry_code);
		break;
	case SCM:
		llc_onEntryScm(entry_code);
		break;
	default:
		return -1;
	}

	{
		u32 fs = *ACQ196_FIFSTAT;
		if (AIFIFO_NOT_EMPTY(fs)){
			err("FIFO_NOT_EMPTY before trig: %08x", fs);
			REPORT_ERROR("FIFO_NOT_EMPTY before trig: %08x", fs);
			return -1;
		}
	}
	activateSignal(CAPDEF->trig);

	return 0;
}

static void llc_onExit(void)
{
	/**
	 * restore FPGA state
	 */

	llc_intsEnable();
	reactivateSignal(CAPDEF->ev[0]);
	reactivateSignal(CAPDEF->ev[1]);

	if (dg.status.errline == 0){
		sprintf(dg.status.report, "llc_onExit OK");
	}
}

#define MAXCHAIN	16
#include "acq200-inline-dma.h"


DEFINE_DMA_CHANNEL(ai_dma, 0);
DEFINE_DMA_CHANNEL(ao_dma, 1);

#define TRADITIONAL 1

#if CYCLE_STEAL
static void dma_append_cycle_stealer(
	struct DmaChannel* dma)
{
	/* handy to use a dma_desc as scratchpad -> is dma mapped */
	static struct iop321_dma_desc* scratchpad;
#define SLEN 16
	struct iop321_dma_desc* stealer = acq200_dmad_alloc();
	unsigned bytes = max(4, pbi_cycle_steal);

	if (scratchpad == 0){
		scratchpad = acq200_dmad_alloc();	
	}

	bytes = min(bytes, sizeof(struct iop321_dma_desc));
	stealer->NDA	= 0;
	stealer->PDA	= DG->fpga.regs.pa;
	stealer->PUAD	= 0;
	stealer->LAD	= scratchpad->pa;
	stealer->BC	= bytes;
	stealer->DC	= DMA_DCR_MEM2MEM;		/* TODO */
	dma_append_chain(dma, stealer, "cycle stealer");
}
#endif


static void initAIdma_SyncAO(void)
{
	if (dg.settings.AO_src){
		struct iop321_dma_desc *ao_dmad = acq200_dmad_alloc();

		ao_dmad->NDA = 0;
		ao_dmad->PDA = dg.settings.AO_src;
		ao_dmad->PUAD = dg.settings.PUAD;
		ao_dmad->LAD = DG->fpga.fifo.pa;
		ao_dmad->BC = AO_BC;
		ao_dmad->DC = DMA_DCR_PCI_MR;
		dma_append_chain(&ai_dma, ao_dmad, "AO");
	}
	if (dg.settings.DO_src){
		struct iop321_dma_desc *do_dmad = acq200_dmad_alloc();
		do_dmad->NDA = 0;
		do_dmad->PDA = dg.settings.DO_src;
		do_dmad->PUAD = dg.settings.PUAD;
		do_dmad->LAD = 0xac00000c;
		do_dmad->BC = 8;
		do_dmad->DC = DMA_DCR_PCI_MR;
		dma_append_chain(&ai_dma, do_dmad, "DO");
	}
#if CYCLE_STEAL
	if (pbi_cycle_steal){
		dma_append_cycle_stealer(&ai_dma);
	}
#endif
}


static void initAIdma_AO16(void)
{
	struct iop321_dma_desc *ao_dmad = acq200_dmad_alloc();

	ao_dmad->NDA = 0;
	ao_dmad->PDA = dg.settings.AO_src;
	ao_dmad->PUAD = dg.settings.PUAD;
	ao_dmad->LAD = DG->fpga.fifo.pa;
	ao_dmad->BC = acq100_llc_sync2V_AO_len == 0?
		AO_BC + AO_SCRATCHPAD_SIZE: acq100_llc_sync2V_AO_len;
	ao_dmad->DC = DMA_DCR_PCI_MR;
	dma_append_chain(&ai_dma, ao_dmad, "AO16/DO32");

	if (acq100_mem2mem){
		ao_dmad->PDA = pa_buf(DG) + 0x100000;
		ao_dmad->LAD = DG->fpga.fifo.pa;
		ao_dmad->DC = DMA_DCR_MEM2MEM;			
	}
}

static void initAIdma_AO32(void)
/* AO32 : pull entire AO vector to local memory (to be allocated) *
 * THEN do local AO32
 * THEN do the slaves ... this is going to be a LONG CHAIN
 */
{
	struct iop321_dma_desc *tmp_dma = acq200_dmad_alloc();

	dbg(1, "01");	
	tmp_dma->NDA = 0;
	tmp_dma->PDA = dg.settings.AO_src;
	tmp_dma->PUAD = dg.settings.PUAD;
	tmp_dma->LAD = dg.settings.ao32.tmp_pa;
	tmp_dma->BC = AO32_TMP_LEN(dg.settings.ao32.count);
	tmp_dma->DC = DMA_DCR_PCI_MR;
	dma_append_chain(&ai_dma, tmp_dma, "aotmp");

	{
		int ii;
		int ao32_offset;	
		struct iop321_dma_desc *ao_dmad;	
		ao_dmad = acq200_dmad_alloc();

		ao_dmad->NDA = 0;
		ao_dmad->MM_SRC = dg.settings.ao32.tmp_pa;
		ao_dmad->PUAD = dg.settings.PUAD;
		ao_dmad->MM_DST = DG->fpga.fifo.pa;
		ao_dmad->BC = acq100_llc_sync2V_AO_len == 0?
			AO_BC + AO_SCRATCHPAD_SIZE: acq100_llc_sync2V_AO_len;
		ao_dmad->DC = DMA_DCR_MEM2MEM;
		dma_append_chain(&ai_dma, ao_dmad, "AO16/DO32");

		for (ii = 0, ao32_offset = LLC_SYNC2V_AO32*sizeof(u32); 
		     ii < dg.settings.ao32.count; ++ii, 
					ao32_offset += AO32_VECLEN){

			ao_dmad = acq200_dmad_alloc();
			ao_dmad->NDA = 0;
			ao_dmad->PDA = dg.settings.ao32.pa[ii];
			ao_dmad->PUAD = dg.settings.PUAD;
			ao_dmad->LAD = dg.settings.ao32.tmp_pa + ao32_offset;
			ao_dmad->BC = AO32_VECLEN;
			ao_dmad->DC = DMA_DCR_PCI_MW;
			dma_append_chain(&ai_dma, ao_dmad, "AO32");
		}
		dbg(1, "75 slaves %d", ii);
	}
	if (dg.settings.do64_readback == 1){
		struct iop321_dma_desc *do64_dmad = acq200_dmad_alloc();
		do64_dmad->NDA = 0;
		do64_dmad->MM_SRC = 
			dg.settings.ao32.tmp_pa + AO32_VECLEN-sizeof(u32);
		do64_dmad->PUAD = dg.settings.PUAD;
		do64_dmad->MM_DST = PA_SCRATCH(LLC_SYNC2V_IN_DO64);
		do64_dmad->BC = sizeof(u32);
		do64_dmad->DC = DMA_DCR_MEM2MEM;
		dma_append_chain(&ai_dma, do64_dmad, "DORB");
	}

	dbg(1, "99");
}

static void initAIdma_AltVI2(void)
{
	struct iop321_dma_desc *alt_dmad = acq200_dmad_alloc();
	alt_dmad->NDA = 0;
	alt_dmad->PDA = dg.settings.alt_VI_target;
	alt_dmad->PUAD = dg.settings.PUAD;
	alt_dmad->LAD = pa_buf(DG);
	alt_dmad->BC = dg.sample_size + SCRATCHPAD_SIZE;
	alt_dmad->DC = DMA_DCR_PCI_MW;
	/* Q at the end */
	dma_append_chain(&ai_dma, alt_dmad, "VI ALT");
}

static void initAIdma_AI(void)
{
	/** messaging regs can be in flight during conversion .. */
	if (dg.settings.STATUS_target){
		struct iop321_dma_desc* status_dmad = acq200_dmad_alloc();

		status_dmad->NDA = 0;
		status_dmad->PDA = dg.settings.STATUS_target;
		status_dmad->PUAD = dg.settings.PUAD;
		status_dmad->LAD = IOP321_REG_PA(IOP321_IMR0);
		status_dmad->BC = 16;
		status_dmad->DC = DMA_DCR_PCI_MW;
		dma_append_chain(&ai_dma, status_dmad, "mbox");
	}

	if (dg.settings.AI_target || TRADITIONAL){
		struct iop321_dma_desc* ai_dmad = acq200_dmad_alloc();
		ai_dmad->NDA = 0;
        /** may also be set dynamically using traditional LLC_CSR_M_SETADDR */
		ai_dmad->PDA = dg.settings.AI_target;  
		ai_dmad->PUAD = dg.settings.PUAD;
		ai_dmad->LAD = DG->fpga.fifo.pa;
		ai_dmad->BC = dg.sample_size;
		ai_dmad->DC = DMA_DCR_PCI_MW;

		dg.ai_dma_index = ai_dma.nchain;
		dma_append_chain(&ai_dma, ai_dmad, "AI");
	}

	/** signals end of transfer. */
	if (dg.settings.DI_target || dg.settings.STATUS_target){
		struct iop321_dma_desc* di_dmad = acq200_dmad_alloc();
		u32 target = dg.settings.STATUS_target?
			dg.settings.STATUS_target + (LLCV2_STATUS_DIO*4) :
			dg.settings.DI_target;
		di_dmad->NDA = 0;
		di_dmad->PDA = target;
		di_dmad->PUAD = dg.settings.PUAD;
		di_dmad->LAD = DG->fpga.regs.pa + 0x40; /* DIOCON */
		di_dmad->BC = 16;
		di_dmad->DC = DMA_DCR_PCI_MW;
		dma_append_chain(&ai_dma, di_dmad, "DI/STATUS");
	}
}
static void initAIdma(void)
/** initialise AI dma channel */
{
	dma_cleanup(&ai_dma);

	initAIdma_AI();

	dbg(1, "dg.sync_output %d dg.settings.ao32.count %d",
	    dg.sync_output, dg.settings.ao32.count);

	if (dg.sync_output){
		initAIdma_SyncAO();
	}

	DMA_DISABLE(ai_dma);
}




static void initAOdma(void)
/** initialise AO dma channel */
{
	struct iop321_dma_desc* ao_dmad = acq200_dmad_alloc();

	dma_cleanup(&ao_dma);

	if (dg.sync_output){
		return;
	}
	ao_dmad->NDA = 0;
	ao_dmad->PDA = 0;
	ao_dmad->PUAD = 0;
	ao_dmad->LAD = DG->fpga.fifo.pa;

	ao_dmad->BC = AO_BC;
	ao_dmad->DC = DMA_DCR_PCI_MR;

	dma_append_chain(&ao_dma, ao_dmad, "AO");


	if (dg.settings.DO_src){
		struct iop321_dma_desc* do_dmad = acq200_dmad_alloc();

		do_dmad->NDA = 0;
		do_dmad->PDA = dg.settings.DO_src;
		do_dmad->PUAD = 0;
		do_dmad->LAD = 0xac00000c;

		do_dmad->BC = 8;
		do_dmad->DC = DMA_DCR_PCI_MR;
		dma_append_chain(&ao_dma, do_dmad, "DO");
	}

	if (acq100_llc_dump_ao2bb){
		ao_dmad->LAD = pa_buf(DG);

		/** 
		 * @todo ... WBN to chain a second descr to do both
		 * but setting the PDA is a lemon, so leave it
		 */
	}
#if CYCLE_STEAL
	if (pbi_cycle_steal){
		dma_append_cycle_stealer(&ao_dma);
	}
#endif
	DMA_DISABLE(ao_dma);	
}


#if WRITE_LAST_AO_SINGLE
#define DMA_PRECHARGE_AO(dmac, pci_addr)				      \
        do {								      \
	       dmac.dmad[0]->PDA = (pci_addr);				      \
	       dmac.dmad[1]->PDA = (pci_addr)+AOCHAN*sizeof(u16)-sizeof(u32); \
        } while(0)
#else
#define DMA_PRECHARGE_AO(dmac, pci_addr)	\
        do {					\
	       dmac.dmad[0]->PDA = (pci_addr);	\
        } while(0)
#endif





static int dmacPollCompletion_pmmr(struct DmaChannel* dmac)
{
	u32 dmacsta;

	while(!DMA_DONE(*dmac, dmacsta) && !dg.emergency_stop){
		++dg.status.dmac_poll_count[dmac->id];	   /* @@todo idle?? */
	}
	if ((dmacsta & IOP321_CSR_ERR) != 0){
		err("dma error 0x%08x", dmacsta);
		return -1;
	}else{
		return 0;
	}
}


#define COUNT_DOWN(x0, xx, limit) \
	(((xx) < (x0))? (x0) - (xx): (xx) + ((limit) - (x0)))

static int dmacPollCompletion_cp(struct DmaChannel* dmac)
/* CP polling - maybe doesn't interfere with the bus?. */
/* We poll TIMER0 until dma_poll_holdoff has expired */
{
	u32 preload;
	u32 count0;
	u32 count;
	u32 total_ticks = 0;
	int pollcat = 0;
	u32 holdoff = dg.settings.dma_poll_holdoff;

	asm volatile("mrc p6, 0, %0, c4, c1, 0" : "=r" (preload));

	asm volatile("mrc p6, 0, %0, c2, c1, 0" : "=r" (count0));

	do {
		++pollcat;

		asm volatile("mrc p6, 0, %0, c2, c1, 0" : "=r" (count));
		total_ticks += COUNT_DOWN(count0, count, preload);
		count0 = count;
	} while (total_ticks < holdoff && !dg.emergency_stop);

	dg.status.dma_holdoff_poll_count += pollcat;

	return dmacPollCompletion_pmmr(dmac);
}

static int (* dmacPollCompletion)(struct DmaChannel* dmac) = 
	dmacPollCompletion_pmmr;








static void llPreamble(void)
{
	initAIdma();
	initAOdma();

	iop321_start_ppmu();  /* @@todo start timing */
}


static void llPreamble2V(void)
{
	int ireg;
	

	dma_cleanup(&ai_dma);	/* LEAVE ao_dma alone! - it's a copy */

	for (ireg = 0; ireg != 16; ++ireg){
		u32 marker;

		switch(ireg){
		case LLC_SYNC2V_IN_VERID:
			marker = BUILD;
			break;
		case LLC_SYNC2V_IN_LASTE:
			marker = 0;
			break;
		case LLC_SYNC2V_IN_BDR:
			marker = 0xdeadbeef;    /* emulate FPGA BDR reg */
			break;
		default:
			marker = LLC_SYNC2V_IDLE_PAT | (ireg<<8);
			break;
		}
		ACQ196_LL_AI_SCRATCH[ireg] = marker;

		dbg(2, "init scratch LLC_SYNC2V_xx [%02d] %p = 0x%08x",
		    ireg, &ACQ196_LL_AI_SCRATCH[ireg], marker);
	}

	if (dg.settings.ai_delay){
		struct iop321_dma_desc *delay_dmad = acq200_dmad_alloc();
		delay_dmad->NDA = 0;
		delay_dmad->PDA = PA_TBLOCK(&DG->bigbuf.tblocks.the_tblocks[5]);
		delay_dmad->PUAD = dg.settings.PUAD;
		delay_dmad->LAD = PA_TBLOCK(&DG->bigbuf.tblocks.the_tblocks[6]);
		delay_dmad->BC = dg.settings.ai_delay;
		delay_dmad->DC = DMA_DCR_MEM2MEM;
		dma_append_chain(&ai_dma, delay_dmad, "ai-delay");
	}
	if (dg.settings.alt_VI_target){
		struct iop321_dma_desc *ai_dmad = acq200_dmad_alloc();
		struct iop321_dma_desc *vi_dmad = acq200_dmad_alloc();

		ai_dmad->NDA = 0;
		ai_dmad->PUAD = 0;
		ai_dmad->PDA = DG->fpga.fifo.pa;
		ai_dmad->LAD = pa_buf(DG);
		ai_dmad->BC = dg.sample_size + SCRATCHPAD_SIZE;
		ai_dmad->DC = DMA_DCR_MEM2MEM;

		dma_append_chain(&ai_dma, ai_dmad, "AI FIFO");

		vi_dmad->NDA = 0;
		vi_dmad->PDA = dg.settings.AI_target;
		vi_dmad->PUAD = dg.settings.PUAD;
		vi_dmad->LAD = pa_buf(DG);
		vi_dmad->BC = dg.sample_size + SCRATCHPAD_SIZE;
		vi_dmad->DC = DMA_DCR_PCI_MW;

		dg.ai_dma_index = ai_dma.nchain;
		dma_append_chain(&ai_dma, vi_dmad, "VI HOST");
		
	}else if (dg.settings.AI_target || TRADITIONAL){
		struct iop321_dma_desc *ai_dmad = acq200_dmad_alloc();

		if (sync2v_samples_per_cycle > MAXSAMPLES_CYCLE){
			sync2v_samples_per_cycle = MAXSAMPLES_CYCLE;
		}
		ai_dmad->NDA = 0;
        /** may also be set dynamically using traditional LLC_CSR_M_SETADDR */
		ai_dmad->PDA = dg.settings.AI_target;  
		ai_dmad->PUAD = dg.settings.PUAD;
		ai_dmad->LAD = DG->fpga.fifo.pa;
		ai_dmad->BC = sync2v_samples_per_cycle * 
			(dg.sample_size + SCRATCHPAD_SIZE);
		ai_dmad->DC = DMA_DCR_PCI_MW;

		dg.ai_dma_index = ai_dma.nchain;
		dma_append_chain(&ai_dma, ai_dmad, "AI");

		if (acq100_mem2mem){
			info("mem2mem - one shot test config. host will hang");
			ai_dmad->PDA = DG->fpga.fifo.pa;
			ai_dmad->LAD = pa_buf(DG);
			ai_dmad->DC = DMA_DCR_MEM2MEM;
		}

	}

	if (dg.settings.io_gap){
		/* kill time by copying local memory. pick two random tblocks */
		struct iop321_dma_desc *gap_dmad = acq200_dmad_alloc();
		gap_dmad->NDA = 0;
		gap_dmad->PDA = PA_TBLOCK(&DG->bigbuf.tblocks.the_tblocks[5]);
		gap_dmad->PUAD = dg.settings.PUAD;
		gap_dmad->LAD = PA_TBLOCK(&DG->bigbuf.tblocks.the_tblocks[6]);
		gap_dmad->BC = dg.settings.io_gap;
		gap_dmad->DC = DMA_DCR_MEM2MEM;
		dma_append_chain(&ai_dma, gap_dmad, "gap");
	}

	if (dg.settings.AO_src){
		int ao_head = ai_dma.nchain;

		if (dg.settings.ao32.count || host_wd_mask != 0){
			initAIdma_AO32();	/* does AO16 also */
		}else{
			initAIdma_AO16();
		}

		if (init_Xo_on_entry){
			unsigned status;

			dma_copy_chain(&ao_dma, &ai_dma, ao_head);
			DMA_ARM(ao_dma);
			DMA_FIRE(ao_dma);
			while(!DMA_DONE(ao_dma, status)){
				yield();
			}
		}
	}

	if (dg.settings.alt_VI_target){
		initAIdma_AltVI2();
	}
	if (dg.settings.dma_poll_holdoff > 0){
		dbg(1, "dmacPollCompletion_cp() selected");
		dmacPollCompletion = dmacPollCompletion_cp;
	}else{
		dmacPollCompletion = dmacPollCompletion_pmmr;
	}

	acq196_syscon_set_all(ACQ196_SYSCON_SP_ENABLE);
	acq196_syscon_dac_set(ACQ196_SYSCON_SP_ENABLE);
	DMA_DISABLE(ai_dma);
	iop321_start_ppmu();  /* @@todo start timing */
}

static void llPostamble(void)
{
	iop321_stop_ppmu();
	if (acq100_llc_sync2V){
		acq196_syscon_clr_all(ACQ196_SYSCON_SP_ENABLE);
		acq196_syscon_dac_clr(ACQ196_SYSCON_SP_ENABLE);
	}
	if (acq100_dropACQEN_on_exit){
		acq196_syscon_clr_all(ACQ196_SYSCON_ACQEN);
	}
}


static void wdt_short_action(volatile u32 *reg, unsigned short wdt_bit)
{
	unsigned short xx = *reg;
	xx ^= wdt_bit;
	*reg = xx;
}


static void wdt_action(void)
{
	unsigned short xa = dg.settings.wdt.toggle;
	unsigned short xb = dg.settings.wdt.toggle >> 16;

	if (dg.status.wdt_preset_done == 0){
		unsigned short pa = *RTM_DIO_DATA_A;
		unsigned short pb = *RTM_DIO_DATA_B;
		u32 do32;

		pa |= dg.settings.wdt.preset;
		pb |= dg.settings.wdt.preset >> 16; 
		
		*RTM_DIO_DATA_A = pa;
		*RTM_DIO_DATA_B = pb;

		/* ensure same values occur on transition to DMA */
		do32 = pb; do32 <<= 16; do32 |= pa;
		O_SCRATCH_DO32 = do32;
		dg.status.wdt_preset_done = 1;	
	}

	if (xa){
		wdt_short_action(RTM_DIO_DATA_A, xa);
	}
	if (xb){
		wdt_short_action(RTM_DIO_DATA_B, xb);
	}
}

static void wdt_cleanup(void)
{
	unsigned short ca = ~(dg.settings.wdt.cleanup);
	unsigned short cb = (~(dg.settings.wdt.cleanup)) >> 16;

	*RTM_DIO_DATA_A &= ca;
	*RTM_DIO_DATA_B &= cb;	
}

#define IS_TRIGGERED(trig) \
     ((trig) || ((trig) = (*ACQ196_SYSCON_ADC&ACQ196_SYSCON_TRIGGERED) != 0))


#define CTR_RUN(run) \
     ((run) || ((run) = (*ACQ196_TCR_IMMEDIATE&ACQ196_TCR_RUN) != 0))

#if USE_GTSR
/**
 * timer doesn't work ... fake it for now.
 */





#define SERVICE_TIMER(t)			\
do {						\
	t = *IOP321_GTSR;			\
} while (0)

#define GTSRMHZ 50

static u32 service_tlatch(u32 tl)
{
	u32 tn = *IOP321_GTSR;

	if (tn > tl){
		if (tn > tl + (GTSRMHZ * dg.prams.divisor)){   
			return tn;
		}else{
			return tl;
		}
	}else{
		return tn;            /* rollover */
	}
}

#define SERVICE_TLATCH(tl) (tl = service_tlatch(tl))

#else

/*
#define SERVICE_ROLLOVER(tim, reg, mask, temp)	\
        while (CTR_RUN(ctr_run)) {	\
                temp = *(reg) & (mask);		\
                if (((tim) & (mask)) > temp){	\
                        (tim) += (mask) + 1;	\
                }				\
                (tim) = ((tim) & ~(mask)) | temp; \
                break;				\
       }
*/

/*
 * duplicate assignment statement adds a conditional bitop, but loses
 * an expensive str/ldr. Way to go!
 */
#define SERVICE_ROLLOVER(tim, reg, mask, temp)				   \
        if (CTR_RUN(ctr_run)) {						   \
                temp = *(reg) & (mask);					   \
                if (((tim) & (mask)) > (temp)){				   \
                        (tim) = (((tim) & ~(mask)) | (temp)) + ((mask)+1); \
                }else{							   \
			(tim) = (((tim) & ~(mask)) | (temp));		   \
	        }							   \
       }
  
#define SERVICE_TIMER(t) \
        SERVICE_ROLLOVER(t, ACQ196_TCR_IMMEDIATE, ACQ196_TCR_MASK, temp)

#define SERVICE_TLATCH(t) \
        SERVICE_ROLLOVER(t, ACQ196_TCR_LATCH, ACQ196_TCR_MASK, temp)
#endif
  
#define UPDATE_TINST(tinst) SERVICE_TIMER(tinst)
#define UPDATE_TLATCH(tlatch) SERVICE_TLATCH(tlatch)


static int aisample_clocked(u32 fifstat) {
	u32 sps = sync2v_samples_per_cycle;
	if (sps == 1){
		return (fifstat & fifo_clocked) != 0;
	}else{
/* tide mark is in 64 byte units. Convert sps to same units
 * if 1 block of 3 is present, start the transfer (fill faster than empty)
 */
		unsigned hitide = fifstat&ACQ196_FIFSTAT_HOTPOINT;

		if (sps*3 < hitide+2){
			return 1;
		}else{
			return 0;
		}
	}
}
#define AISAMPLE_CLOCKED(fifstat)     aisample_clocked(fifstat)

#define AO_DATA_WAITING(mfa)          (((mfa) = acq200mu_get_ib()) != 0)

/*
 * 2NOV: ACQ196_FIFSTAT_HOTPOINT is HOT d2..d5
 * HOTPOINT measures longwords.
 */

#define HOTSAM(fifstat)  (((fifstat)&ACQ196_FIFSTAT_HOTPOINT)*(1<<2)*2)

#define KICKOFF(fifstat) (HOTSAM(fifstat) >= 1)
#define OVERRUN(fifstat) (HOTSAM(fifstat) > dg.sample_size)
#define ESTOP            (dg.emergency_stop)

#if COUNT_CLEARS_ON_TRIG
/**
 * twere supposed to be thus :
 */
#define REPORT_TLATCH (*MBOX[BP_MB_LLC_TADC] = dg.status.tlatch)
#define REPORT_TINST  (*MBOX[BP_MB_LLC_TINST] = dg.status.tinst)
#else
#define REPORT_TLATCH (*MBOX[BP_MB_LLC_TADC] = dg.status.tlatch - dg.status.t0)
#define REPORT_TINST  (*MBOX[BP_MB_LLC_TINST] = dg.status.tinst - dg.status.t0)
#endif


#define SCHEDULE 1    /* mask to determine scheduled tasks */


#define SACK  (*MBOX[BP_MB_LLC_CSR] = csr |= LLC_CSR_SACK)
#define SNACK (*MBOX[BP_MB_LLC_CSR] = csr |= LLC_CSR_SNACK)
#define XACK  (LLC_CSR_SACK|LLC_CSR_SNACK)
#define COMMAND (((csr = *MBOX[BP_MB_LLC_CSR]) & XACK) == 0)

#if USE_DBG
#define DMAC_GO(dmac, fifstat)					\
	do {							\
		DMA_FIRE(dmac);				        \
		DBG(2,"DMAC_GO fs:0x%08x %08x=>%08x %d", 	\
		    fifstat, DG->fpga.fifo.pa, 			\
		    dg.settings.AI_target, dg.sample_size);	\
	} while(0)

#else
#define DMAC_GO(dmac, fifstat) 	DMA_FIRE(dmac)
#endif


static void llc_loop(int entry_code)
{
	u32 csr = 0;
	int decim_count = 1;
	u32 fifstat;
	MFA ao_data;
	u32 tcycle = 0;
	u32 fifstat2;
	u32 temp;
	int ctr_run = 0;
	int snack_on_quit = 1;

	DBG(1, "ENTER %d", entry_code);

	llPreamble();
	csr |= LLC_CSR_READY;
	SACK;

	for (dg.status.iter = 0; !dg.emergency_stop; ){
		/** @critical STARTS */
		fifstat = *ACQ196_FIFSTAT;

		if (AISAMPLE_CLOCKED(fifstat)){
#if AVOID_HOT_UNDERRUN
			if (!KICKOFF(fifstat)){
				do {
					fifstat2 = *ACQ196_FIFSTAT;
					++dg.status.fifo_poll_count;
				} while(!KICKOFF(fifstat2) && !ESTOP);
			}
#endif
			DMAC_GO(ai_dma, fifstat);
			/* @critical ENDS */

			if (dg.status.sample_count == 0){
				dg.status.t0 = dg.status.tlatch;
			}
			if ( csr&LLC_CSR_M_SOFTCLOCK ){
				// @todo acq32_softClock( 0 );      
			}
			if (dmacPollCompletion(&ai_dma) != 0){
				REPORT_ERROR_AND_QUIT(
					"ERROR:DMAC status bad 0x%08x\n",
					DMA_STA(ai_dma));
			}
			UPDATE_TLATCH(dg.status.tlatch);
			REPORT_TLATCH;

			/** Interrupt on DMA DONE */
			if (dg.settings.iodd){
				*IOP321_ODR |= BP_INT_LLC_DMA_DONE;
			}

			UPDATE_TINST(dg.status.tinst);
			REPORT_TINST;

			tcycle = dg.status.tinst - dg.status.tlatch;

			fifstat2 = *ACQ196_FIFSTAT;

			if (OVERRUN(fifstat)) goto onOverrun;
			if (AIFIFO_NOT_EMPTY(fifstat2)) goto onFifoNotEmpty;
#if USE_DBG
			DBG(2, "status 0x%08x, sent data to 0x%08x %d %d %d\n",
			    csr, dg.settings.AI_target, 
			    dg.status.tlatch, 
			    dg.status.tinst, tcycle);
#endif
			DMA_ARM(ai_dma);
			dg.status.sample_count++;
		}
		if (AO_DATA_WAITING(ao_data)){
#if CHECK_DAC_DMA_ERRORS
			u32 dmacsta;
#endif
			/**
			 * @critical - kick off DMAC with AO data
			 */
#if CHECK_DAC_UPDATE
			u32 syscon_dac = *ACQ196_SYSCON_DAC;

			if ((syscon_dac & ACQ196_SYSCON_DAC_UPD) != 0){
				err("WARNING: DAC update bit already set");
			}
#endif
#if CHECK_DAC_DMA_ERRORS
/** ERROR if still busy from last time */
			if (!(DMA_DONE(ao_dma, dmacsta))){
				REPORT_ERROR_AND_QUIT(
					"ERROR aodma NOT idle %08x\n",
					dmacsta);
			}else if ((dmacsta & IOP321_CSR_ERR) != 0){
				REPORT_ERROR_AND_QUIT(
					"ERROR aodma ERROR %08x\n",
					dmacsta);
			}
#endif
#if USE_DBG
			DBG(1, "AO DATA 0x%08x", ao_data);
#endif
			DMA_PRECHARGE_AO(ao_dma, 
			        (dg.settings.AI_target&dg.hb_mask)|ao_data);
			DMA_ARM(ao_dma);
			DMA_FIRE(ao_dma);
				      
			/* DMA from mfa2pa(mfa); */
			acq200mu_return_free_ib(ao_data);

#if CHECK_DAC_UPDATE
			if (dmacPollCompletion(&ao_dma) != 0){
				REPORT_ERROR_AND_QUIT(
					"ERROR:DMAC status bad 0x%08x\n",
					DMA_STA(ao_dma));
			}else{
				syscon_dac = *ACQ196_SYSCON_DAC;

				if ((syscon_dac & ACQ196_SYSCON_DAC_UPD) == 0){
					REPORT_ERROR_AND_QUIT(
						"ERROR DAC FAILED to update "
						" 0x%08x\n", syscon_dac);
				}
				*ACQ196_SYSCON_DAC = syscon_dac;
			}
#endif
		}
		
/** schedule the rest to reduce loading */

		switch((++dg.status.iter)&SCHEDULE){
		case 1:
		if (COMMAND){
#if USE_DBG
			DBG(3, "%8s 0x%08x", "COMMAND", csr);
#endif
			if ((csr & LLC_CSR_M_ESC) != 0){
				SACK;
				snack_on_quit = 0;
				REPORT_AND_QUIT("QUIT on remote 0x%08x\n",csr);
			}else{
				if ((csr & LLC_CSR_M_SETADDR) != 0){
					csr &= ~LLC_CSR_M_SETADDR;
					dg.settings.AI_target = 
						*MBOX[BP_MB_LLC_DATA_ADDR];
					DMA_PRECHARGEN(
						ai_dma, 
						dg.ai_dma_index,
						dg.settings.AI_target);
				}
				if ((csr & LLC_CSR_M_ARM) != 0){
					dg.settings.auto_incr = 
						(csr&LLC_CSR_M_AUTOINCR) != 0;

					csr &= ~LLC_CSR_M_ARM;
					dg.settings.AI_target = 
						*MBOX[BP_MB_LLC_DATA_ADDR];
					decim_count = dg.settings.decim_base =
						LLC_GET_DECIM(csr);
					DMA_PRECHARGEN(
						ai_dma, 
						dg.ai_dma_index,
						dg.settings.AI_target);
					DMA_ARM(ai_dma);
					
					enable_acq();
					if (dg.settings.soft_trigger){
						soft_trig_all();
					}
					csr |= LLC_CSR_IS_ARMED;
				}

				if ((csr&LLC_CSR_M_SOFTCLOCK) != 0){
					/* @@todo SOFTCLOCK */
					csr &= ~LLC_CSR_M_SOFTCLOCK;
				}

				if ((csr&LLC_CSR_M_READCTR) != 0){
					csr &= ~LLC_CSR_M_READCTR;
					UPDATE_TINST(dg.status.tinst);
					if (ctr_run){
						csr |= LLC_CSR_S_CTR_RUN;
					}
					REPORT_TINST;
				}

				csr &= ~LLC_CSR_S_TCYCLE;
				csr |= LLC_MAKE_TCYCLE(tcycle);

				SACK;
#if USE_DBG
 				DBG(3, "%8s 0x%08x", "SACK", csr);
#endif
			}
			break;
		}

		case 0:
			UPDATE_TINST(dg.status.tinst);
			break;
		default:
			;
		}
	}

 quit:
	if (snack_on_quit){
		long j1 = jiffies;

		if (dg.settings.iodd){
			*IOP321_ODR |= BP_INT_LLC_ERROR;
		}

		SNACK;
		while (!COMMAND){
			++dg.status.waiting_for_permission_to_quit;
			if (jiffies - j1 > 10*HZ){
				err("TIMEOUT on host handshake");
				break;
			}
		}
		SACK;
	}

	DBG(1, "QUITTING %s", dg.emergency_stop? "ESTOP": "");
	dg.emergency_stop = 0;

	llPostamble();
	return;


onOverrun: 
onFifoNotEmpty: {
		u32 tl2 = *ACQ196_TCR_LATCH;
		u32 ti2 = *ACQ196_TCR_IMMEDIATE;

		err("status 0x%08x, sent data to 0x%08x "
		    "%d %d %d TI 0x%08x TL 0x%08x\n",
		    csr, dg.settings.AI_target, dg.status.tlatch, 
		    dg.status.tinst, tcycle, ti2, tl2);

		if (OVERRUN(fifstat)){
			REPORT_ERROR(
			"ERROR:FIFO SAMPLE OVERRUN 0x%08x 0x%08x tc %d",
			fifstat, fifstat2, tcycle);
		}
		if (AIFIFO_NOT_EMPTY(fifstat2)){
			REPORT_ERROR(
			"ERROR:FIFO NOT EMPTY 0x%08x 0x%08x tc %d",
			fifstat, fifstat2, tcycle);
		}
		goto quit;
	}
}


/* project MBOXES into SCRATCHPAD ... */
/** @todo maybe drop writes to phys mbox altogether? */

#define SACK2V  do {							\
		SACK;							\
		ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_MBOX0] = csr;	\
	} while(0)

#define REPORT_TLATCH2V do {						\
		REPORT_TLATCH;						\
		ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_MBOX2] = dg.status.tlatch; \
	} while(0)							\

#define REPORT_TINST2V do {						\
		REPORT_TINST;						\
		ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_MBOX3] = dg.status.tinst; \
	} while(0)


/***********************************************************************************************
 *
 *
 *
 *  S Y N C 2 V
 *
 *
 **********************************************************************************************/

/* !!! KLUDGE alert: why does host_wd come from ao32+LLC_SYNC2V_WD */

static void host_wd_action(void)
{
	u32 *aox = (u32*)dg.settings.ao32.tmp;
	u32 host_wd;

	dma_sync_single_for_cpu(
			0, dg.settings.ao32.tmp_pa+LLC_SYNC2V_WD*sizeof(u32),
			sizeof(u32), DMA_FROM_DEVICE);

	host_wd = aox[LLC_SYNC2V_WD];

	dbg(1, "src:%p host_wd:%d mask:%02x DIOCON:=%02x",
			&aox[LLC_SYNC2V_WD], host_wd, host_wd_mask,
			(host_wd&LLC_SYNC2V_WD_BIT) != 0? host_wd_mask: 0);

	if ((host_wd&LLC_SYNC2V_WD_BIT) != 0){
		*ACQ200_DIOCON |= host_wd_mask << ACQ200_DIOCON_OUTDAT_SHL;
	}else{
		*ACQ200_DIOCON &= ~(host_wd_mask << ACQ200_DIOCON_OUTDAT_SHL);
	}
}


void llc_loop_sync2V(int entry_code)
/**< llc_loop_sync2V 2Vectors synchronized loop, optimized for max reprate.
 *
 * Sequence: 
 * - Detect incoming AI Clock
 * - local housekeeping:
 *  - read and store DI32, 
 *  - (fpga does DI6), 
 *  - STATUS (iter, !overrun), 
 *  - 32 bit TLATCH
 * - trigger DMA on AI data ready (maybe trimmed in)
 *  - 2 element DMA chain runs, VIN, VOUT.
 * -  On end of chain, update DO
 * - Check mailboxes
 * -Check for overrun
 *
 */
{
	u32 csr = 0;
	int decim_count = 1;
	u32 fifstat;
	u32 tcycle = 0;
	u32 fifstat2;
	u32 temp;
	int ctr_run = 0;
	int snack_on_quit = 1;
	unsigned iter1 = 0;
	u32 dmacsta;
	u32 command = 0;
#if FIFO_ONESAM
	unsigned fifo_onesam = 
		getNumChan(CHANNEL_MASK) == 32? 2:
		getNumChan(CHANNEL_MASK) == 64? 3: 4;
#endif		

	DBG(1, "ENTER %d", entry_code);
	dg.status.sample_count = 0;
	llPreamble2V();
	csr |= LLC_CSR_READY;
	SACK2V;

	for (iter1 = dg.status.iter = 0; !dg.emergency_stop;  ++dg.status.iter){
		/** @critical STARTS */
		fifstat = *ACQ196_FIFSTAT;

		if (AISAMPLE_CLOCKED(fifstat) != 0){
			/* @todo HOUSKEEPING - were we fast enough? */
		hot_start:
			DMAC_GO(ai_dma, fifstat);
			/* @critical ENDS */
			if (++dg.status.sample_count == 1){
				dg.status.iter = 1;
			}
#if 0
			if ( csr&LLC_CSR_M_SOFTCLOCK ){
				// @todo acq32_softClock( 0 );      
			}
#endif
			while(!DMA_DONE(ai_dma, dmacsta) && !dg.emergency_stop){
				if (!command) {
					command = COMMAND;                 /* maybe a mailbox cmd? */
				}
				++dg.status.dmac_poll_count[ai_dma.id];	   /* @@todo idle?? */
			}
			if ((dmacsta & IOP321_CSR_ERR) != 0){
				REPORT_ERROR_AND_QUIT(
					"ERROR:DMAC status bad 0x%08x\n",
					DMA_STA(ai_dma));
			}

			if (acq100_llc_sync2V >= ACQ100_LLC_SYNC2V_DI){
				I_SCRATCH_DI32 = getDI32();
				if (acq100_llc_sync2V >= ACQ100_LLC_SYNC2V_DIDO){
					setDO32(O_SCRATCH_DO32);
					if (host_wd_mask){
						host_wd_action();
					}
					if (acq100_llc_sync2V >=
					    ACQ100_LLC_SYNC2V_DIDOSTA ){
						I_SCRATCH_ITER = dg.status.iter;
						I_SCRATCH_TINST = *IOP321_GTSR;
						I_SCRATCH_SC = dg.status.sample_count;
		    				I_SCRATCH_FSTA = fifstat;
					}
				}
				/** housekeeping - error if zero polls */
				if (dg.status.iter - iter1 < 1){
					I_SCRATCH_LASTE	= dg.status.sample_count;
				}
			}

			if (dg.settings.do64_readback == 2){
				u32* va = (u32*)((void*)dg.settings.ao32.tmp+AO32_VECLEN);
				unsigned pa = dg.settings.ao32.tmp_pa+AO32_VECLEN-sizeof(u32);
				dma_sync_single_for_cpu(NULL, pa, sizeof(u32), DMA_FROM_DEVICE); 
				I_SCRATCH_DORB = va[-1];
			}

			/** Interrupt on DMA DONE */
			if (dg.settings.iodd){
				*IOP321_ODR |= BP_INT_LLC_DMA_DONE;
			}


			iter1 = dg.status.iter;


			if (use_max_samples != USE_MAX_SAMPLES_IGNORE &&
					dg.status.sample_count > MAX_SAMPLES){
				switch(use_max_samples){
				case USE_MAX_SAMPLES_QUIT:
					dg.emergency_stop = 1; /* fall thru */
				case USE_MAX_SAMPLES_DISABLE:
					disable_acq();		/* fall thru */
				case USE_MAX_SAMPLES_INTEN:
					llc_intsEnable();
				}
				err("use_max_samples:%d dg.status.sample_count %d",
					use_max_samples, dg.status.sample_count);
				use_max_samples = 0;
				continue;
			}
			DMA_ARM(ai_dma);

			/** @todo check overruns */
			fifstat2 = *ACQ196_FIFSTAT;

			if (OVERRUN(fifstat)) goto onOverrun;
#if FIFO_ONESAM
			if ((fifstat2&ACQ196_FIFSTAT_HOTPOINT) > fifo_onesam){
				goto onFifoNotEmpty;
			}
#else
			if (AIFIFO_NOT_EMPTY(fifstat2)) goto onFifoNotEmpty;
#endif
			if (!command && AISAMPLE_CLOCKED(fifstat2)){
				++dg.status.hot_starts;
				fifstat = fifstat2;
				goto hot_start;       /** skip fifstat poll */
			}
		}

		if (command || COMMAND){
			command = 0;
#if USE_DBG
			DBG(3, "%8s 0x%08x", "COMMAND", csr);
#endif
			if ((csr & LLC_CSR_M_ESC) != 0){
				SACK;
				snack_on_quit = 0;
				REPORT_AND_QUIT("QUIT on remote 0x%08x\n",csr);
			}else{
				if ((csr & LLC_CSR_M_SETADDR) != 0){
					csr &= ~LLC_CSR_M_SETADDR;
					dg.settings.AI_target = 
						*MBOX[BP_MB_LLC_DATA_ADDR];
					DMA_PRECHARGEN(
						ai_dma, 
						dg.ai_dma_index,
						dg.settings.AI_target);
					dg.status.cmd_counts.SETADDR++;
				}
				if ((csr & LLC_CSR_M_ARM) != 0){
					dg.settings.auto_incr = 
						(csr&LLC_CSR_M_AUTOINCR) != 0;

					csr &= ~LLC_CSR_M_ARM;
					dg.settings.AI_target = 
						*MBOX[BP_MB_LLC_DATA_ADDR];
					decim_count = dg.settings.decim_base =
						LLC_GET_DECIM(csr);
					DMA_PRECHARGEN(
						ai_dma, 
						dg.ai_dma_index,
						dg.settings.AI_target);
					DMA_ARM(ai_dma);
					
					enable_acq();
					if (dg.settings.soft_trigger){
						soft_trig_all();
					}
					csr |= LLC_CSR_IS_ARMED;
					dg.status.cmd_counts.ARM++;
				}
				if ((csr&LLC_CSR_M_SOFTCLOCK) != 0){
					/* @@todo SOFTCLOCK */
					csr &= ~LLC_CSR_M_SOFTCLOCK;
					dg.status.cmd_counts.SETADDR++;
				}

				if ((csr&LLC_CSR_M_READCTR) != 0){
					csr &= ~LLC_CSR_M_READCTR;
					UPDATE_TINST(dg.status.tinst);
					if (ctr_run){
						csr |= LLC_CSR_S_CTR_RUN;
					}
					REPORT_TINST2V;
					dg.status.cmd_counts.READCTR++;
					wdt_action();
				}

				csr &= ~LLC_CSR_S_TCYCLE;
				csr |= LLC_MAKE_TCYCLE(tcycle);
				SACK2V;
#if USE_DBG
 				DBG(3, "%8s 0x%08x", "SACK", csr);
#endif
			}
		}
	}

quit:
	wdt_cleanup();

	if (snack_on_quit){
		long j1 = jiffies;

		if (dg.settings.iodd){
			*IOP321_ODR |= BP_INT_LLC_ERROR;
		}

		SNACK;
		while (!COMMAND){
			++dg.status.waiting_for_permission_to_quit;
			if (jiffies - j1 > 10*HZ){
				err("TIMEOUT on host handshake");
				break;
			}
		}
		SACK;
	}

	DBG(1, "QUITTING %s", dg.emergency_stop? "ESTOP": "");
	dg.emergency_stop = 0;

	llPostamble();
	return;


onOverrun: 
onFifoNotEmpty: {
		u32 tl2 = *ACQ196_TCR_LATCH;
		u32 ti2 = *ACQ196_TCR_IMMEDIATE;

		err("status 0x%08x, sent data to 0x%08x "
		    "%d %d %d TI 0x%08x TL 0x%08x\n",
		    csr, dg.settings.AI_target, dg.status.tlatch, 
		    dg.status.tinst, tcycle, ti2, tl2);

		if (OVERRUN(fifstat)){
			REPORT_ERROR(
				"ERROR:FIFO SAMPLE OVERRUN 0x%08x 0x%08x tc %d",
				fifstat, fifstat2, tcycle);
		}
		if (AIFIFO_NOT_EMPTY(fifstat2)){
			REPORT_ERROR(
				"ERROR:FIFO NOT EMPTY 0x%08x 0x%08x tc %d",
				fifstat, fifstat2, tcycle);
		}
		goto quit;
	}
}



static void acq100_llc_loop(int entry_code)
/**
 * acq100_llc_loop
 * @entry_code - possible enhancements
 *
 */
{
	if (llc_onEntry(entry_code) == 0){
		if (acq100_llc_sync2V){
			llc_loop_sync2V(entry_code);
		}else{
			llc_loop(entry_code);
		}
		llc_onExit();
	}
}







static ssize_t set_mode(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
/** mode {SCM|ECM} clkpos trpos intsoftclock [divisor] */
{
	struct LlcPrams prams = {
		.divisor = 40
	};
	char modestr[80];
	int nscan = sscanf(buf, "%s %d %d %d %d",
			   modestr, 
			   &prams.clkpos,
			   &prams.trpos,
			   &prams.intsoftclock,
			   &prams.divisor);

	switch(nscan){
	case 5:	case 4:
		if (strcmp(modestr, "ECM") == 0){
			prams.m_mode = ECM;
		}else if (strcmp(modestr, "SCM") == 0){
			prams.m_mode = SCM;
		}else{
			err("BAD mode %s", modestr);
			break;
		}
		memcpy(&dg.prams, &prams, sizeof(prams));
	default:
		break;
	}
	return strlen(buf);
}
static ssize_t show_mode(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s %d %d %d %d\n",
		       dg.prams.m_mode==ECM? "ECM":
		       dg.prams.m_mode==SCM? "SCM": "no mode",
		       dg.prams.clkpos,
		       dg.prams.trpos,
		       dg.prams.intsoftclock,
		       dg.prams.divisor
		);
}

static DEVICE_ATTR(mode, S_IWUGO|S_IRUGO, show_mode, set_mode);


static ssize_t show_settings(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "decim:%d target:0x%08x auto_inc:%d \n",
		       dg.settings.decim_base,
		       dg.settings.AI_target,
		       dg.settings.auto_incr );
}

static DEVICE_ATTR(settings, S_IRUGO, show_settings, 0);


static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq100_llc_driver_name,
		       acq100_llc_driver_string,
		       acq100_llc_driver_version,
		       acq100_llc_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);


static ssize_t set_llc_mask(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf(buf, "0x%x", &dg.imask) == 0){
		sscanf(buf, "%x", &dg.imask);
	}

	return strlen(buf);
}
static ssize_t show_llc_mask(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "0x%08X\n", dg.imask);
}

static DEVICE_ATTR(imask, S_IWUGO|S_IRUGO, show_llc_mask, set_llc_mask);

#define DECLARE_HOST_BUF(id)					\
static ssize_t set_##id (					\
	struct device *dev,					\
	struct device_attribute *attr,				\
	const char * buf, size_t count)				\
{								\
	if (sscanf(buf, "0x%x", &dg.settings.id) == 0){		\
		sscanf(buf, "%x", &dg.settings.id);		\
	}							\
								\
	return strlen(buf);					\
}								\
static ssize_t show_##id (					\
	struct device *dev,					\
	struct device_attribute *attr,				\
	char * buf)						\
{								\
        return sprintf(buf, "0x%08x\n", dg.settings.id);	\
}								\
static DEVICE_ATTR(id, S_IWUGO|S_IRUGO, show_##id, set_##id)

DECLARE_HOST_BUF(DO_src);
DECLARE_HOST_BUF(AO_src);
DECLARE_HOST_BUF(AI_target);
DECLARE_HOST_BUF(DI_target);
DECLARE_HOST_BUF(STATUS_target);
DECLARE_HOST_BUF(PUAD);

#define DEVICE_CREATE_HOST_BUF(dev, id)		\
DEVICE_CREATE_FILE(dev, &dev_attr_##id)

static ssize_t set_iodd(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.settings.iodd);
	return strlen(buf);
}
static ssize_t show_iodd(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.settings.iodd);
}

static DEVICE_ATTR(IODD, S_IWUGO|S_IRUGO, show_iodd, set_iodd);

static ssize_t set_io_gap(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.settings.io_gap);
	return strlen(buf);
}
static ssize_t show_io_gap(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.settings.io_gap);
}

static DEVICE_ATTR(IO_GAP, S_IWUGO|S_IRUGO, show_io_gap, set_io_gap);

static ssize_t set_ai_delay(
	struct device *dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.settings.ai_delay);
	return strlen(buf);
}
static ssize_t show_ai_delay(
	struct device *dev,
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.settings.ai_delay);
}

static DEVICE_ATTR(AI_DELAY, S_IWUGO|S_IRUGO, show_ai_delay, set_ai_delay);

static ssize_t set_do64_readback(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.settings.do64_readback);
	return strlen(buf);
}
static ssize_t show_do64_readback(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.settings.do64_readback);
}

static DEVICE_ATTR(DO64_READBACK, S_IWUGO|S_IRUGO, 
	show_do64_readback, set_do64_readback);



static ssize_t set_soft_trigger(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.settings.soft_trigger);
	return strlen(buf);
}
static ssize_t show_soft_trigger(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.settings.soft_trigger);
}

static DEVICE_ATTR(soft_trigger, S_IWUGO|S_IRUGO, show_soft_trigger, set_soft_trigger);




static ssize_t set_sync_output(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.sync_output);
	return strlen(buf);
}
static ssize_t show_sync_output(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.sync_output);
}

static DEVICE_ATTR(sync_output, S_IWUGO|S_IRUGO, show_sync_output, set_sync_output);

static ssize_t show_hb_mask(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "0x%08x\n", dg.hb_mask);
}
static DEVICE_ATTR(hb_mask, S_IRUGO, show_hb_mask, 0);

static ssize_t set_llc_debug(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &acq100_llc_debug);
	return strlen(buf);
}
static ssize_t show_llc_debug(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", acq100_llc_debug);
}
static DEVICE_ATTR(debug, S_IWUGO|S_IRUGO, show_llc_debug, set_llc_debug);

static ssize_t show_status(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
#define STAPRINT(fmt, field) \
        len += sprintf(buf+len, "%20s: " fmt "\n", #field, dg.status.field)

	len += sprintf(buf+len, "%20s: %d %10s %d\n", 
		       "channels", getNumChan(CHANNEL_MASK),
		       "shot", dg.shot);

	STAPRINT("%d", is_triggered);
	STAPRINT("%u", sample_count);
	STAPRINT("%u", t0);
	STAPRINT("%u", tlatch);
	STAPRINT("%u", tinst);
	len += sprintf(buf+len, "%20s: %d\n", "tprocess", 
		       dg.status.tinst - dg.status.tlatch);
	STAPRINT("\"%s\"", report);
	STAPRINT("%d", errline);
	STAPRINT("%u", iter);
	STAPRINT("%u", fifo_poll_count);
	STAPRINT("%u", dmac_poll_count[0]);
	STAPRINT("%u", dmac_poll_count[1]);
	STAPRINT("%u", dma_holdoff_poll_count);
	STAPRINT("%u", hot_starts);
	STAPRINT("%d", waiting_for_permission_to_quit);
	STAPRINT("%d", cmd_counts.SETADDR);
	STAPRINT("%d", cmd_counts.ARM);
	STAPRINT("%d", cmd_counts.SOFTCLOCK);
	STAPRINT("%d", cmd_counts.READCTR);

	return len;
#undef STAPRINT
}

static DEVICE_ATTR(status, S_IRUGO, show_status, 0);



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
static ssize_t show_show_dma(
	struct device* dev, 
	struct device_attribute *attr,
	char* buf)
{
	int len = 0;

	len += sprintf(buf+len, "ai_dma\n");
	len += print_chain(&ai_dma, buf+len);
	len += sprintf(buf+len, "ao_dma\n");
	len += print_chain(&ao_dma, buf+len);

	return len;
}

static DEVICE_ATTR(show_dma, S_IRUGO, show_show_dma, 0);

static ssize_t set_llc(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int entry_code = 0;

	if (sscanf(buf, "%d", &entry_code) == 1 && entry_code > 0){
		acq100_llc_loop(entry_code);
	}
	return strlen(buf);
}
static DEVICE_ATTR(run, S_IWUGO, 0, set_llc);

static ssize_t set_emergency_stop(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.emergency_stop);
	return strlen(buf);
}
static ssize_t show_emergency_stop(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.emergency_stop);
}

static DEVICE_ATTR(emergency_stop, S_IWUGO|S_IRUGO, 
		   show_emergency_stop, set_emergency_stop);



static ssize_t show_llc(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n", "WORKTODO");
}

static DEVICE_ATTR(stats, S_IRUGO, show_llc, 0);

static ssize_t show_channel_mask(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", CHANNEL_MASK);
}
static DEVICE_ATTR(channel_mask, S_IRUGO, show_channel_mask, 0);

static ssize_t set_dac_lowlat(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &dg.dac_lowlat);
	return strlen(buf);
}
static ssize_t show_dac_lowlat(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", dg.dac_lowlat != 0);
}

static DEVICE_ATTR(dac_lowlat, S_IWUGO|S_IRUGO, 
		   show_dac_lowlat, set_dac_lowlat);

static ssize_t set_dma_poll_holdoff(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int nsecs;

	if (sscanf(buf, "%d", &nsecs)){
		dg.settings.dma_poll_holdoff = nsecs/5;
	}
	return strlen(buf);
}

static ssize_t show_dma_poll_holdoff(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d nsecs\n",dg.settings.dma_poll_holdoff*5);
}

static DEVICE_ATTR(dma_poll_holdoff, S_IWUGO|S_IRUGO, 
		   show_dma_poll_holdoff, set_dma_poll_holdoff);


static ssize_t set_alt_VI_target(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned addr;
	if (sscanf(buf, "0x%x", &addr) == 1){
		dg.settings.alt_VI_target = addr;
	}
	return strlen(buf);
}

static ssize_t get_alt_VI_target(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "0x%08x\n", dg.settings.alt_VI_target);
}


static DEVICE_ATTR(alt_VI_target, S_IWUGO|S_IRUGO,
		   get_alt_VI_target, set_alt_VI_target);

static ssize_t set_wdt_bit(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned parms[3];
	int nscan = sscanf(buf, "0x%x,0x%08x,0x%08x", 
			   &parms[0], &parms[1], &parms[2]);
	switch(nscan){
	case 1:
		dg.settings.wdt.toggle = parms[0];
		break;
	case 3:
		dg.settings.wdt.preset = parms[0];
		dg.settings.wdt.toggle = parms[1];
		dg.settings.wdt.cleanup = parms[2];
		break;
	default:
		return -1;
	}
	return strlen(buf);
}

static ssize_t get_wdt_bit(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "0x%08x,0x%08x,0x%08x\n", 
		       dg.settings.wdt.preset,
		       dg.settings.wdt.toggle,
		       dg.settings.wdt.cleanup);
}

static DEVICE_ATTR(wdt_bit, S_IWUGO|S_IRUGO, get_wdt_bit, set_wdt_bit);

/** test my ARM ASM! */
static ssize_t show_timer0_count(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 preload;
	u32 count0;
	u32 count;

	u32 pcount0;
	u32 pcount;

	asm volatile("mrc p6, 0, %0, c4, c1, 0" : "=r" (preload));
	asm volatile("mrc p6, 0, %0, c2, c1, 0" : "=r" (count0));
	asm volatile("mrc p6, 0, %0, c2, c1, 0" : "=r" (count));

	pcount0 = *IOP321_TU_TCR0;
	pcount  = *IOP321_TU_TCR0;
	       
        return sprintf(buf, 
		       "%8s: %08x\n"
		       "%8s:%08x %08x delta %u\n"
		       "%8s:%08x %08x delta %u\n", 
		       "preload", preload,
		       "mrc", count0, count, COUNT_DOWN(count0,count,preload),
		       "pmmr", pcount0, pcount, 
		       COUNT_DOWN(pcount0, pcount,preload));
}

static DEVICE_ATTR(timer0_count, S_IRUGO, 
		   show_timer0_count, 0);






static int mk_llc_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_run);
	DEVICE_CREATE_FILE(dev, &dev_attr_stats);
	DEVICE_CREATE_FILE(dev, &dev_attr_mode);
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_imask);
	DEVICE_CREATE_FILE(dev, &dev_attr_hb_mask);
	DEVICE_CREATE_FILE(dev, &dev_attr_debug);
	DEVICE_CREATE_FILE(dev, &dev_attr_settings);
	DEVICE_CREATE_FILE(dev, &dev_attr_emergency_stop);
	DEVICE_CREATE_FILE(dev, &dev_attr_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mask);
	DEVICE_CREATE_FILE(dev, &dev_attr_dac_lowlat);
	DEVICE_CREATE_FILE(dev, &dev_attr_dma_poll_holdoff);
	DEVICE_CREATE_FILE(dev, &dev_attr_show_dma);
	DEVICE_CREATE_FILE(dev, &dev_attr_timer0_count);

	DEVICE_CREATE_FILE(dev, &dev_attr_soft_trigger);
	DEVICE_CREATE_FILE(dev, &dev_attr_IODD);
	DEVICE_CREATE_FILE(dev, &dev_attr_IO_GAP);
	DEVICE_CREATE_FILE(dev, &dev_attr_AI_DELAY);
	DEVICE_CREATE_FILE(dev, &dev_attr_DO64_READBACK);
	DEVICE_CREATE_FILE(dev, &dev_attr_sync_output);
	DEVICE_CREATE_FILE(dev, &dev_attr_alt_VI_target);
	DEVICE_CREATE_FILE(dev, &dev_attr_wdt_bit);

	DEVICE_CREATE_HOST_BUF(dev, DO_src);
	DEVICE_CREATE_HOST_BUF(dev, AO_src);

	DEVICE_CREATE_HOST_BUF(dev, AI_target);
	DEVICE_CREATE_HOST_BUF(dev, DI_target);
	DEVICE_CREATE_HOST_BUF(dev, STATUS_target);
	DEVICE_CREATE_HOST_BUF(dev, PUAD);
	return 0;
}


extern struct proc_dir_entry *proc_acq200;

static void mk_llc_procfs(struct device *dev)
{
#define CPRE( name, func ) \
        create_proc_read_entry( name, 0, proc_acq200, func, 0 )

	
#undef CPRE
}

static void rm_llc_procfs(struct device *dev)
{
#define RMP( name ) remove_proc_entry( name, proc_acq200 )

#undef RMP
}


static void acq100_llc_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq100_llc_driver;

static int acq100_llc_probe(struct device *dev)
{
	info("");
	dg.llcv2_init_buf = kmalloc(BLEN, GFP_KERNEL); 
	mk_llc_sysfs(dev);
	mk_llc_procfs(dev);
	return 0;
}

static int acq100_llc_remove(struct device *dev)
{
	kfree(dg.llcv2_init_buf);
	if (dg.settings.ao32.tmp){
		kfree(dg.settings.ao32.tmp);
	}
	rm_llc_procfs(dev);
	return 0;
}


static struct device_driver acq100_llc_driver = {
	.name     = "acq100_llc",
	.probe    = acq100_llc_probe,
	.remove   = acq100_llc_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq100_llc_device = {
	.name = "acq100_llc",
	.id   = 0,
	.dev = {
		.release    = acq100_llc_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq100_llc_init( void )
{
	int rc;
	acq200_debug = acq100_llc_debug;

	rc = driver_register(&acq100_llc_driver);
	if (rc){
		return rc;
	}else{
		return platform_device_register(&acq100_llc_device);
	}
}


static void __exit
acq100_llc_exit_module(void)
{
	info("");
	dma_cleanup(&ai_dma);
	dma_cleanup(&ao_dma);

	platform_device_unregister(&acq100_llc_device);
	driver_unregister(&acq100_llc_driver);
}

module_init(acq100_llc_init);
module_exit(acq100_llc_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ100 Low Latency Control");


