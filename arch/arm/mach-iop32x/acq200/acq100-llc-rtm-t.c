/* ------------------------------------------------------------------------- */
/* acq100-llc-rtm-t.c driver for acq100 lowlatency controller                      */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
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

/* RTM-T LLC : the IOP's main task is to keep OUT THE WAY!
 */

#define ACQ196

#define FIFO_ONESAM          1
#define DI_IN_1              1
#define USES_CLKDLY          1
#define DBG dbg

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
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

#define acq200_debug acq100_llc_rtm_t_debug

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"

#include "acq100_rtm_t.h"

#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"

int acq100_llc_rtm_t_debug;
module_param(acq100_llc_rtm_t_debug, int, 0664);

int acq100_dropACQEN_on_exit;
module_param(acq100_dropACQEN_on_exit, int, 0664);

int sync2v_samples_per_cycle = 1;
module_param(sync2v_samples_per_cycle, int, 0644);

int disable_acq_debug = 0;         /* set 1 */
module_param(disable_acq_debug, int, 0644);

/** set to non-zero for 2V mode */
int acq100_llc_sync2V = 0;

#define ACQ100_LLC_SYNC2V         1
#define ACQ100_LLC_SYNC2V_DI      2
#define ACQ100_LLC_SYNC2V_DIDO    3
#define ACQ100_LLC_SYNC2V_DIDOSTA 4


/* SYNC2V SCRATCHPAD */
#define I_SCRATCH_ITER ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_ITER]
#define I_SCRATCH_SC   ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_VERID+1]
#define I_SCRATCH_FSTA ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_VERID+2]
#if DI_IN_1
#define I_SCRATCH_DI32 ACQ196_LL_AI_SCRATCH[1]
#else
#define I_SCRATCH_DI32 ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_DI32]
#endif
#define I_SCRATCH_LASTE ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_LASTE]
#define I_SCRATCH_DORB  ACQ196_LL_AI_SCRATCH[LLC_SYNC2V_IN_DO64]
#define O_SCRATCH_DO32  ACQ196_LL_AO_SCRATCH[0]

/** size of fifo extension for sync2v */
#define SCRATCHPAD_SIZE (32*sizeof(short))
#define AO_SCRATCHPAD_SIZE (4)


#define REPORT_ERROR(fmt, arg...) \
    sprintf(dg.status.report, "%d " fmt, dg.status.errline = __LINE__, arg)

#define REPORT_ERROR_AND_QUIT(fmt, arg...) \
        REPORT_ERROR(fmt, arg); goto quit

#define REPORT_AND_QUIT(fmt, arg...) \
        sprintf(dg.status.report, "%d " fmt, __LINE__, arg); goto quit

/** increment BUILD and VERID each build */
#define BUILD 1000

#define _VERID(build) "build " #build
#define VERID _VERID(BUILD)

char acq100_llc_rtm_t_driver_name[] = "acq100-llc";
char acq100_llc_rtm_t_driver_string[] = "D-TACQ Low Latency Control Device";
char acq100_llc_rtm_t_driver_version[] = VERID " "__DATE__ " Features:\n";

char acq100_llc_copyright[] = "Copyright (c) 2004-2011 D-TACQ Solutions Ltd";


#define CHANNEL_MASK (CAPDEF->channel_mask)

/** size of fifo extension for sync2v */
#define SCRATCHPAD_SIZE (32*sizeof(short))

#define REPORT_ERROR(fmt, arg...) \
    sprintf(dg.status.report, "%d " fmt, dg.status.errline = __LINE__, arg)

#define REPORT_ERROR_AND_QUIT(fmt, arg...) \
        REPORT_ERROR(fmt, arg); goto quit

#define REPORT_AND_QUIT(fmt, arg...) \
        sprintf(dg.status.report, "%d " fmt, __LINE__, arg); goto quit

#if USES_CLKDLY
#define FIFO_NE       (ACQ196_FIFSTAT_HOT_NE)
#define FIFO_CLOCKED  (ACQ196_FIFSTAT_HOT_NE|ACQ196_FIFSTAT_ADC_CLKDLY)
/* because it will be ready by the time we come to service it */
#else
#define FIFO_CLOCKED  (ACQ196_FIFSTAT_HOT_NE)
#define FIFO_NE       (ACQ196_FIFSTAT_HOT_NE)
#endif

#define HOTSAM(fifstat)  (((fifstat)&ACQ196_FIFSTAT_HOTPOINT)*(1<<2)*2)

#define KICKOFF(fifstat) (HOTSAM(fifstat) >= 1)
#define OVERRUN(fifstat) (HOTSAM(fifstat) > dg.sample_size)
#define ESTOP            (dg.emergency_stop)

static int askedToQuit(void);

static int aisample_clocked(u32 fifstat) {
	u32 sps = sync2v_samples_per_cycle;
	if (sps == 1){
		return (fifstat & FIFO_CLOCKED) != 0;
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
#define AIFIFO_NOT_EMPTY(fifstat)     (((fifstat)&FIFO_NE) != 0)
#define COLDFIFO_EMPTY(fifstat)       (((fifstat)&dg.coldfifo_ne_mask)==0)

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
} dg;


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

static void llPreamble2V(void)
{
	acq196_syscon_set_all(ACQ196_SYSCON_SP_ENABLE);
}

static void llPostamble(void)
{
	if (acq100_llc_sync2V){
		acq196_syscon_clr_all(ACQ196_SYSCON_SP_ENABLE);
	}
	if (acq100_dropACQEN_on_exit){
		acq196_syscon_clr_all(ACQ196_SYSCON_ACQEN);
	}
}

static void llc_loop_sync2V(void)
/**< llc_loop_sync2V 2Vectors synchronised loop, optimized for max reprate.
 *
 */
{
	u32 fifstat;
	u32 tcycle = 0;
	u32 fifstat2;
	unsigned iter1 = 0;
	u32 command = 0;
#if FIFO_ONESAM
	unsigned fifo_onesam =
		getNumChan(CHANNEL_MASK) == 32? 2:
		getNumChan(CHANNEL_MASK) == 64? 3: 4;
#endif
	dg.status.sample_count = 0;
	llPreamble2V();

	for (iter1 = dg.status.iter = 0; !dg.emergency_stop;  ++dg.status.iter){
		/** @critical STARTS */
		fifstat = *ACQ196_FIFSTAT;

		if (AISAMPLE_CLOCKED(fifstat) != 0){
			/* @todo HOUSKEEPING - were we fast enough? */

			/* we assume it doesn't matter if this stuff makes this transfer or not..
			 * We set CLKDLY short to give time for something useful ..
			 */
			hot_start:
			/* @critical ENDS */

			if (acq100_llc_sync2V >= ACQ100_LLC_SYNC2V_DI){
				if (acq100_llc_sync2V >=
						ACQ100_LLC_SYNC2V_DIDOSTA ){
					I_SCRATCH_ITER = dg.status.iter;
					I_SCRATCH_SC = dg.status.sample_count;
					I_SCRATCH_FSTA = fifstat;
				}
			}


			if (++dg.status.sample_count == 1){
				dg.status.iter = 1;
			}

			yield(); yield(); yield();

			/*
			if (dg.settings.do64_readback == 2){
				u32* va = (u32*)((void*)dg.settings.ao32.tmp+AO32_VECLEN);
				unsigned pa = dg.settings.ao32.tmp_pa+AO32_VECLEN-sizeof(u32);
				dma_sync_single_for_cpu(NULL, pa, sizeof(u32), DMA_FROM_DEVICE);
				I_SCRATCH_DORB = va[-1];
			}
			 */
			/** housekeeping - error if zero polls */
			if (dg.status.iter - iter1 < 1){
				I_SCRATCH_LASTE	= dg.status.sample_count;
			}
			iter1 = dg.status.iter;

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

		if(askedToQuit()){
			REPORT_AND_QUIT("QUIT on remote %d", 0);
		}
	}

	quit:

	DBG(1, "QUITTING %s", dg.emergency_stop? "ESTOP": "");
	dg.emergency_stop = 0;

	llPostamble();
	return;


onOverrun:
onFifoNotEmpty: {
		u32 tl2 = *ACQ196_TCR_LATCH;
		u32 ti2 = *ACQ196_TCR_IMMEDIATE;

		err("status TI 0x%08x TL 0x%08x\n", ti2, tl2);

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

static int llc_onEntryEcm(void)
/** customisation for ECM mode. */
{
	*ACQ200_CLKDAT = dg.prams.divisor;

	*ACQ196_CLKCON = ACQ196_CLKCON_LLSYNC;

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

static int llc_onEntry(void)
/**
 * @returns 0 => OK to run
 */
{
	++dg.shot;
	memset(&dg.status, 0, sizeof(dg.status));
	sprintf(dg.status.report, "llc_onEntry()");

	llc_intsDisable();
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

	llc_onEntryEcm();
	activateSignal(CAPDEF->trig);

	return 0;
}

static void onRun(u32 mbox)
{
	unsigned clkdiv, clk_pos, trg_pos;

	DECODE_LLC_CMD(mbox, clkdiv, clk_pos, trg_pos);

	dbg(1, "clkdiv %d clk_pos:%d trg_pos:%d", clkdiv, clk_pos, trg_pos);
	llc_onEntry();
}

static void runAction(void)
{
	llc_loop_sync2V();
}

static void onStop(void)
{
	dbg(1, "01");
}

static void ack(u32 command)
{
	u32 q_mbox1 = *RTMT_REG(RTMT_Q_MBOX1);
	q_mbox1 &= ~H_MBOX1_LLC_CMD;
	q_mbox1 |= command&H_MBOX1_LLC_CMD;
	*RTMT_REG(RTMT_Q_MBOX1) = q_mbox1;
}

static int askedToQuit(void)
{
	u32 h_mbox1 = *RTMT_REG(RTMT_H_MBOX1);
	return IS_LLC_STOP(h_mbox1);
}


static int llc_task(void *nothing)
{
	enum LLC_STATE { IDLE, RUN } state = IDLE;

	wait_queue_head_t queue;
	u32 h_mbox1 = *RTMT_REG(RTMT_H_MBOX1);

	init_waitqueue_head(&queue);

	while(!kthread_should_stop()){
		u32 new_h_mbox1 = *RTMT_REG(RTMT_H_MBOX1);
		int input_change;
		if ((input_change = new_h_mbox1 != h_mbox1) != 0){
			info("input change %08x => %08x", h_mbox1, new_h_mbox1);
			h_mbox1 = new_h_mbox1;
		}

		switch(state){
		case IDLE:
			if (input_change && IS_LLC_RUN(h_mbox1)){
				onRun(h_mbox1);
				ack(h_mbox1);
				state = RUN;	/* fall thru */

			}else{
				break;
			}
		case RUN:
			if (input_change && IS_LLC_STOP(h_mbox1)){
				onStop();
				state = IDLE;
				ack(h_mbox1);
			}else{
				runAction();
				state = IDLE;
				/* continue; */ /* comment in when runAction blocks */
			}
		}
		interruptible_sleep_on_timeout(&queue, HZ);
	}

	return 0;
}
static void llc_task_start(int start)
{
	static struct task_struct *the_worker;

	if (start){
		if (the_worker == 0){
			the_worker = kthread_run(llc_task, NULL, "llc_rtm_t");
		}
	}else{
		if (the_worker != 0){
			kthread_stop(the_worker);
			the_worker = 0;
		}
	}
}
static int acq100_llc_rtm_t_probe(struct device *dev)
{
	info("");
	llc_task_start(1);
	return 0;
}

static int acq100_llc_rtm_t_remove(struct device *dev)
{
	llc_task_start(0);
	return 0;
}

static void acq100_llc_rtm_t_dev_release(struct device * dev)
{
	info("");
}

static struct device_driver acq100_llc_rtm_t_driver = {
	.name     = "acq100_llc_rtm_t",
	.probe    = acq100_llc_rtm_t_probe,
	.remove   = acq100_llc_rtm_t_remove,
	.bus	  = &platform_bus_type,
};


/*
static u64 dma_mask = 0x00000000ffffffff;
*/
static struct platform_device acq100_llc_rtm_t_device = {
	.name = "acq100_llc_rtm_t",
	.id   = 0,
	.dev = {
		.release    = acq100_llc_rtm_t_dev_release,
/*
		.dma_mask   = &dma_mask
*/
	}

};


static int __init acq100_llc_rtm_t_init( void )
{
	int rc;
	acq200_debug = acq100_llc_rtm_t_debug;

	rc = driver_register(&acq100_llc_rtm_t_driver);
	if (rc){
		return rc;
	}else{
		return platform_device_register(&acq100_llc_rtm_t_device);
	}
}


static void __exit
acq100_llc_rtm_t_exit_module(void)
{
	info("");
	platform_device_unregister(&acq100_llc_rtm_t_device);
	driver_unregister(&acq100_llc_rtm_t_driver);
}

module_init(acq100_llc_rtm_t_init);
module_exit(acq100_llc_rtm_t_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ100 RTM_T Low Latency Control");

