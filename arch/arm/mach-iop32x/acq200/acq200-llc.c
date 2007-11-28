/* ------------------------------------------------------------------------- */
/* acq200-llc.c driver for acq100 lowlatency controller                      */
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


#define VERID "$Revision: 1.20 $ build B1018 "

#define ACQ216
#define USE_GTSR             1

/** @file acq200-llc.c
 * Module Design
 *
 * This driver is the "low latency controller" 
 * It builds on services provided by the acq100-fifo.c device driver
 *

- ECM:

- TRIG : DI3 mezz

- CLK : DI0 mezz => source for Internal Clock Gen, output on DO1

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
#define acq200_debug acq200_llc_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acqX00-rtm-dds.h"


#include "../../../../drivers/i2c/chips/pcf8575.h"
#include "../../../../drivers/i2c/chips/ad539x.h"

int acq200_llc_debug;
module_param(acq200_llc_debug, int, 0664);

int acq200_llc_dump_ao2bb;
module_param(acq200_llc_dump_ao2bb, int , 0664);

char* verid = VERID;
module_param(verid, charp, 0444);

/* device defs - [0] devid [1] driver type */

#define I2C_DEVID  0
#define I2C_DRVTYP 1
#define VR_DEVS    3

static char* vr1[2];
module_param_array(vr1, charp, NULL, S_IRUGO);

static char* vr2[2];
module_param_array(vr2, charp, NULL, S_IRUGO);

static char* vr3[2];
module_param_array(vr3, charp, NULL, S_IRUGO);

static char* voff[2];
module_param_array(voff, charp, NULL, S_IRUGO);

#define VR1 0
#define VR2 1
#define VR3 2
#define VOFF 4


char acq200_llc_driver_name[] = "acq200-llc";
char acq200_llc_driver_string[] = "D-TACQ Low Latency Control Device";
char acq200_llc_driver_version[] = VERID __DATE__ " Features:\n"
#if USE_GTSR
"GTSR\n"
#else
"TCR\n"
#endif
;

char acq200_llc_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";



#define REPORT_ERROR(fmt, arg...) \
    sprintf(dg.status.report, "%d " fmt, dg.status.errline = __LINE__, arg)

#define REPORT_ERROR_AND_QUIT(fmt, arg...) \
        REPORT_ERROR(fmt, arg); goto quit_the_game

#define REPORT_AND_QUIT(fmt, arg...) \
        sprintf(dg.status.report, "%d " fmt, __LINE__, arg); goto quit_the_game

#define CHANNEL_MASK (CAPDEF->channel_mask)


/**
 * lookup table maps block length to FIFO thresholds
 * @todo block_len > 16K not supported at this time, needs multiple transfers
 */
static const struct BLUT {
	u32 block_len;
	u8  hot_th;
	u8  cold_th;
} GBLUT[] = { 
	{ .block_len = 0x0400, .hot_th = 0x1, .cold_th = 0x0 },
	{ .block_len = 0x0800, .hot_th = 0x2, .cold_th = 0x0 },
	{ .block_len = 0x1000, .hot_th = 0x4, .cold_th = 0x0 },
	{ .block_len = 0x2000, .hot_th = 0x8, .cold_th = 0x0 },	
	{ .block_len = 0x4000, .hot_th = 0xf, .cold_th = 0x2 },	
	{ .block_len = 0x8000, .hot_th = 0xf, .cold_th = 0x2 },	
/* these last two rely on fill keeping pace with draw */
	{ .block_len =0x10000, .hot_th = 0xf, .cold_th = 0x2 },	
	{ .block_len =0x20000, .hot_th = 0xf, .cold_th = 0x4 },	
	{},
};

/**
 *  @todo - replace by size variables
 */
#define HOT_TH  (dg.settings.block_def.hot_th)
#define COLD_TH (dg.settings.block_def.cold_th)
#define BLOCK_LEN (dg.settings.block_def.block_len)

#define DEFAULT_BL 0x8000

#define PCI2BUS(pa) (acq216_pci2bus(pa) & ~(BLOCK_LEN-1))

enum TRIGGER_STATES {
	TS_IDLE, TS_REQUESTED, TS_TRIGGERED, TS_RESCINDED 
};
static const char *TRIGGER_STATES_STRINGS[] = {
	"TS_IDLE", "TS_REQUESTED", "TS_TRIGGERED", "TS_RESCINDED"
};

static int getNumChan(u32 cmask){
	int nchan = 0;
	u32 imask = 1;

	for (imask = 1; imask; imask <<= 1){
		if (imask&cmask){
			++nchan;
		}
	}

	return nchan;
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
	int sample_size;

/* @@todo - publish SETTINGS thru /sys/.../settings (ro) */
	struct SETTINGS {                /* user specified things */
		int decim_base;               
		unsigned target;         /* current pci target addr */
		unsigned auto_incr;
		struct BLUT block_def;
		int iodd;                      /* interrupt on DMA done */
	} settings;

	struct LLC200_INIT* llc200_init_buf;

	struct STATUS {
		int trigger_state;
		int iter;
		u32 t0;
		u32 tinst;
		u32 tlatch;
		u32 sample_count;
		char report[128];
		int errline;
		int waiting_for_permission_to_quit;
		unsigned fifo_poll_count;
		unsigned dmac_poll_count[2];
	} status;

	int shot;

	int vr_dev_count;
	struct device *vr_dev[3];
	struct device *voff;       
	unsigned short voff_cache[16];
	int voff_cache_valid;
} dg = {
	.settings.decim_base = 1
};


#define LLC200_INIT_SZ sizeof(struct LLC200_INIT)
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


static int hard_trigger_selected(void)
{
	return CAPDEF->trig->is_active;
}
static inline void enable_soft(void)
{
	deactivateSignal(CAPDEF->trig);
	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN|ACQ200_SYSCON_SOFTTRG;
}

static inline void soft_trigger(void) 
{
	dbg(1, "");
	*ACQ200_SYSCON &= ~ACQ200_SYSCON_SOFTTRG;
	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN|ACQ200_SYSCON_SOFTTRG;
}

static inline void enable_hard(void)
{
	*ACQ200_SYSCON &= ~ACQ200_SYSCON_DAQEN;
	activateSignal(CAPDEF->trig);
	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN;
}

static unsigned get_sample_size(unsigned mask)
/* @@todo - surely there is a generic func for this? */
{
	unsigned nchan = 0;

	for(; mask != 0; mask >>= 1){
		if ((mask &1) != 0){
			++nchan;
		}
	}
	return nchan * 2;
}

static void select_block_def(int bl)
{
	int ii;

	for (ii = 0;; ++ii){
		if (GBLUT[ii].block_len >= bl ||
		    GBLUT[ii+1].block_len == 0    ){
			break;
		}
	}
	/** ii always points to a valid record, default last one */
	memcpy(&dg.settings.block_def, &GBLUT[ii], sizeof(struct BLUT));
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

static int llc_onEntryEcm(int entry_code)
/** customisation for ECM mode. */
{
	return 0;
}

static int llc_onEntryScm(int entry_code)
/** customisation for SCM mode */
{
	return 0;
}



static int llc_onEntry(int entry_code)
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

	dg.sample_size = get_sample_size(CHANNEL_MASK);

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

	if (hard_trigger_selected()){
		enable_hard();
	}else{
		enable_soft();
	}
	return 0;
}

static void llc_onExit(void)
{
	llc_intsEnable();
	/**
	 * restore FPGA state
	 */
	disable_fifo();
	disable_acq();

	reactivateSignal(CAPDEF->ev[0]);
	reactivateSignal(CAPDEF->ev[1]);

	if (dg.status.errline == 0){
		sprintf(dg.status.report, "llc_onExit OK");
	}
}

#include "acq200-inline-dma.h"


DEFINE_DMA_CHANNEL(ai_dma, 0);



#define CHAIN(i) ai_dma.dmad[i] 

static void initAIdma(void)
/** initialise AI dma channel */
{
	CHAIN(0) = acq200_dmad_alloc();
	CHAIN(1) = acq200_dmad_alloc();

	CHAIN(0)->NDA = CHAIN(1)->pa;
	CHAIN(0)->PDA = DG->fpga.fifo.pa;
	CHAIN(0)->PUAD = 0;
	CHAIN(0)->LAD = pa_buf(DG);
	CHAIN(0)->BC = BLOCK_LEN;    
	CHAIN(0)->DC = DMA_DCR_PCI_MR;         
	
	CHAIN(1)->NDA = 0;
	CHAIN(1)->PDA = 0xdeadbeef;      /* update me! */
	CHAIN(1)->PUAD = 0;
	CHAIN(1)->LAD = pa_buf(DG);
	CHAIN(1)->BC = BLOCK_LEN;
	CHAIN(1)->DC = DMA_DCR_PCI_MW;

	ai_dma.nchain = 2;
	DMA_DISABLE(ai_dma);
}


static inline void setTarget(u32 pci_addr)
/** sets target addr, masked to 16MB window size */
{
	CHAIN(1)->PDA = dg.settings.target = PCI2BUS(pci_addr);
}

static inline void incrTarget(u32 incr)
{
	CHAIN(1)->PDA = dg.settings.target += incr;
}

static void llPreamble(void)
{
	initAIdma();

	iop321_start_ppmu();  /* @@todo start timing */
}



static int dmacPollCompletion(struct DmaChannel* dmac, int upoll)
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
static void llPostamble(void)
{
	iop321_stop_ppmu();
	dma_cleanup(&ai_dma);
}

static void ll200_init_ftw(unsigned char ftw[])
{
	dbg(1, "ftw: %02x%02x%02x%02x%02x%02x", 
	    ftw[0], ftw[1], ftw[2], ftw[3], ftw[4], ftw[5]);

	acq200_dds_set_ftw1_bin(ftw);
}

static void ll200_init_dds_qdac(short qdac)
{
	dbg(1, "qdac: %d\n", qdac);
	acq200_dds_set_qdac(qdac);
}
static void ll200_init_ranges(unsigned short range[])
{
	int ir;

	dbg(1, "range: %04x,%04x,%04x,%04x", 
	    range[0], range[1], range[2], range[3]);

	for (ir = 0; ir != dg.vr_dev_count; ++ir){
		pcf8575_write(dg.vr_dev[ir], range[ir]);
	}
}

static void ll200_init_offsets(unsigned short offsets[])
{
	int ch;

	dbg(1, "offsets");
	if (!dg.voff){
		err("WARNING: voffset device not configured");
		return;
	}

	for (ch = 0; ch != 16; ++ch){
		unsigned short my_offset = offsets[ch];

		if (!dg.voff_cache_valid || 
		    my_offset != dg.voff_cache[ch]){
			dbg(1,"ad539x_set_channel %d %u", ch, my_offset);
			ad539x_set_channel(dg.voff, ch, my_offset);
			dg.voff_cache[ch] = my_offset;
		}
	}

	dg.voff_cache_valid = 1;
}

static void ll200_init_trig(unsigned trig)
{
	dbg(1, "trig: %08x", trig);
	acq216_setTrig(
		(trig&LLC200_INIT_TRIG_EXTRIG) != 0,
		(trig&LLC200_INIT_TRIG_EXTLINE),
		(trig&LLC200_INIT_TRIG_RISING) != 0);
}

static void ll200_init_channel(unsigned mask)
/** Modify channel mask. DAQEN must be disabled while we do this. */
{
	dbg(1, "channel_mask: 0x%08x", mask);


	*ACQ200_SYSCON &= ~ACQ200_SYSCON_DAQEN;

	if ((mask & LLC200_INIT_CHANNEL_MASK_ANTIPHASE) != 0){
		dbg(1, "acq216_setAntiPhase(1)");
		acq216_setAntiPhase(1);
	        mask ^= LLC200_INIT_CHANNEL_MASK_ANTIPHASE;
	}else{
		dbg(1, "acq216_setAntiPhase(0)");
		acq216_setAntiPhase(0);
	}
	if (mask != 0x000f){    
		/* mask == 0x000f case handled by acq216_setAntiPhase() */
		acq200_setChannelMask(mask);
	}
	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN;
}

static void ll200_init_int_clk(unsigned hz)
{
	dbg(1, "hz: %d", hz);
	acq200_setIntClkHz(hz);
}
static void acton_LL200_init(struct LLC200_INIT* buf)
{
	u32 mask = buf->mask;
	u32 cursor;

	dbg(1, "mask 0x%08x", mask);
	dbg(2, "sizeof LLC200_INIT %d", sizeof(struct LLC200_INIT));

	for (cursor = 1; mask != 0; cursor <<= 1){
		if (mask&cursor){
			switch(cursor){
			case LLC200_INIT_MASK_DDS_FTW:
				ll200_init_ftw(buf->dds_ftw);
				break;
			case LLC200_INIT_MASK_RANGE:
				ll200_init_ranges(buf->vranges.w);
				break;
			case LLC200_INIT_MASK_OFFSETS:
				ll200_init_offsets(buf->offsets);
				break;
			case LLC200_INIT_MASK_TRIG:
				ll200_init_trig(buf->trig);
				break;
			case LLC200_INIT_MASK_CHANNEL:
				ll200_init_channel(buf->channel_mask);
				break;
			case LLC200_INIT_MASK_INTCLK:
				ll200_init_int_clk(buf->int_clk);
				break;
			case LLC200_INIT_MASK_DDS_QDAC:
				ll200_init_dds_qdac(*(short*)&buf->dds_ftw[6]);
				break;
			default:
				err("MASK 0x%08x not understood\n", cursor);
			}
			mask &= ~cursor;
		}
	}
}

static int do_LL200_init(void)
{
	u32 remaddr = *MBOX[BP_MB_A4];
	int rc = 0;

	dbg(1, "remaddr 0x%08x", remaddr);

	if (remaddr){
		struct LLC200_INIT* buf = dg.llc200_init_buf;
		dma_addr_t dmabuf = dma_map_single(
			NULL, buf, LLC200_INIT_SZ, PCI_DMA_FROMDEVICE);

		acq200_post_dmac_request(
			1|DMA_CHANNEL_POLL, dmabuf, 0, PCI2BUS(remaddr),
			LLC200_INIT_SZ, 1);

		dma_unmap_single(
			NULL, dmabuf, LLC200_INIT_SZ, PCI_DMA_FROMDEVICE); 

		if (buf->marker == LLC200_INIT_MAGIC_MARKER){
			if (buf->mask){
				acton_LL200_init(buf);
			}
		}else{
			err("Expected LLC200_INIT_MAGIC");
			rc = -1;
		}
	}

	return rc;
}


#define IS_TRIGGERED(trig) \
     ((trig) || ((trig) = (*ACQ200_SYSCON & ACQ200_SYSCON_TR) != 0))


#define CTR_RUN(run) \
     ((run) || ((run) = (*ACQ216_TCR_IMM & ACQ216_TCR_RUNNING) != 0))

#if USE_GTSR
/**
 * timer doesn't work ... fake it for now.
 * adjust to MHz (partly to avoid tprocess overflow)
 */
#define GTSRMHZ 50

#if 0
#define SERVICE_TIMER(t)			\
do {						\
	t = *IOP321_GTSR/GTSRMHZ;		\
} while (0)
#else
/* approximate /50 as  *5/256 or /8 * 5 / 32 or >>3, * 5, >> 5 */
#define SERVICE_TIMER(t)			\
do {						\
	t = *IOP321_GTSR >> 3;			\
        t = (t * 5) >> 5; 			\
} while (0)
#endif

#define SERVICE_TLATCH(tl) SERVICE_TIMER(tl)

#else
#define SERVICE_ROLLOVER(tim, reg, mask, temp)	\
        while (CTR_RUN(ctr_run)) {	\
                temp = *(reg) & (mask);		\
                if (((tim) & (mask)) <= temp){	\
                        (tim) &= ~(mask);	\
                }else{				\
			(tim) &= ~(mask);	\
                        (tim) += (mask) + 1;	\
                }				\
                (tim) |= temp;			\
                break;				\
       }
  
#define SERVICE_TIMER(t) \
        SERVICE_ROLLOVER(t, ACQ216_TCR_IMM, ACQ216_TCR_COUNTER, temp)

#define SERVICE_TLATCH(t) \
        SERVICE_ROLLOVER(t, ACQ216_TCR_LAT, ACQ216_TCR_COUNTER, temp)
#endif
  
#define UPDATE_TINST(tinst) SERVICE_TIMER(tinst)


#define AIFIFO_NOT_EMPTY(fifstat)     (((fifstat)&ACQ200_FIFCON_HOTEMPTY) == 0)

#define AIFIFO_HOTPOINT(fifstat)  \
        (((fifstat) & ACQ200_FIFCON_HOTPOINT) >> ACQ200_FIFCON_HOTP_SHIFT)

#define AIFIFO_COLDPOINT(fifstat) \
        (((fifstat) & ACQ200_FIFCON_COLDPOINT) >> ACQ200_FIFCON_COLDP_SHIFT)

#define AIFIFO_HOT_OVER(fifstat) \
         (AIFIFO_HOTPOINT(fifstat) >= HOT_TH && \
	 AIFIFO_COLDPOINT(fifstat) >= COLD_TH ) 

#define AO_DATA_WAITING(mfa)          (((mfa) = acq200mu_get_ib()) != 0)

/*
 * 2NOV: ACQ196_FIFSTAT_HOTPOINT is HOT d2..d5
 * HOTPOINT measures longwords.
 */

#define HOTSAM(fifstat)  (((fifstat)&ACQ196_FIFSTAT_HOTPOINT)*(1<<2)*2)

#define KICKOFF(fifstat) (HOTSAM(fifstat) >= 1)
#define OVERRUN(fifstat) (((fifstat) & ACQ200_FIFCON_COLDOVER) != 0)
#define ESTOP            (dg.emergency_stop)

#define REPORT_TLATCH (*MBOX[BP_MB_LLC_TADC] = dg.status.tlatch - dg.status.t0)
#define REPORT_TINST  (*MBOX[BP_MB_LLC_TINST] = dg.status.tinst - dg.status.t0)



static void llc_loop(int entry_code)
{
#define SACK  (*MBOX[BP_MB_LLC_CSR] = csr |= LLC_CSR_SACK)
#define SNACK (*MBOX[BP_MB_LLC_CSR] = csr |= LLC_CSR_SNACK)
#define XACK  (LLC_CSR_SACK|LLC_CSR_SNACK)
#define COMMAND (((csr = *MBOX[BP_MB_LLC_CSR]) & XACK) == 0)

#define DMAC_GO(dmac, fifstat)					\
	do {							\
		DMA_FIRE(dmac);				        \
		dbg(2,"DMAC_GO fs:0x%08x %08x=>%08x %d", 	\
		    fifstat, DG->fpga.fifo.pa, 			\
		    dg.settings.target, BLOCK_LEN);	\
	} while(0)

#define UPDATE_TLATCH(tlatch) \
        do { SERVICE_TLATCH(tlatch); latch_updated = 1; } while(0)

	u32 csr = 0;
	int decim_count = 1;
	u32 fifstat;
	u32 tcycle = 0;
	u32 fifstat2;
	int ctr_run = 1;             /** @todo REMOVE ME */
	int snack_on_quit = 1;
	int latch_updated = 0;
	int triggered;
#if !USE_GTSR
	u32 temp;
#endif
	dbg(1, "ENTER %d", entry_code);

	llPreamble();
	csr |= LLC_CSR_READY;
	SACK;

	for (dg.status.iter = 0; !dg.emergency_stop; ++dg.status.iter){
		/** @critical STARTS */
		fifstat = *ACQ200_FIFCON;

		if (!latch_updated && AIFIFO_NOT_EMPTY(fifstat)){
			UPDATE_TLATCH(dg.status.tlatch);
		}
		if (AIFIFO_HOT_OVER(fifstat)){
			DMAC_GO(ai_dma, fifstat);
			/* @critical ENDS */

			if (dg.status.sample_count == 0){
				dg.status.t0 = dg.status.tlatch;
			}
			if (dmacPollCompletion(&ai_dma, 1) != 0){
				REPORT_ERROR_AND_QUIT(
					"ERROR:DMAC status bad 0x%08x\n",
					DMA_STA(ai_dma));
			}

			UPDATE_TINST(dg.status.tinst);
			REPORT_TLATCH;

			/** Interrupt on DMA DONE */
			if (dg.settings.iodd){
				*IOP321_ODR |= BP_INT_LLC_DMA_DONE;
			}

			REPORT_TINST;
			tcycle = dg.status.tinst - dg.status.tlatch;
			if (tcycle > 255){
				tcycle = 255;  /** must fit 8 bit field :-( */
			}

			fifstat2 = *ACQ200_FIFCON;

			dg.status.sample_count++;

			dbg(2, "status 0x%08x, sent data to 0x%08x %d %d %d\n",
			    csr, dg.settings.target, 
			    dg.status.tlatch, 
			    dg.status.tinst, tcycle);

			if (dg.settings.auto_incr){
				incrTarget(BLOCK_LEN);
			}
			latch_updated = 0;
			DMA_ARM(ai_dma);
			
#if 0
			disable_fifo();
			reset_fifo();
#else
/** this works for streaming frame-per-capture so follow the lead .. */
			disable_acq();
			blip_fifo_reset();
			enable_acq();
#endif
			dg.status.trigger_state = TS_IDLE;
		}

		if (COMMAND){
			dbg(3, "%8s 0x%08x", "COMMAND", csr);

			if ((csr & LLC_CSR_M_ESC) != 0){
				SACK;
				snack_on_quit = 0;
				REPORT_AND_QUIT("QUIT on remote 0x%08x\n",csr);
			}else{
				if ((csr & LLC_CSR_M_SETADDR) != 0){
					csr &= ~LLC_CSR_M_SETADDR;
					setTarget(*MBOX[BP_MB_LLC_DATA_ADDR]);
				}
				if ((csr & LLC_CSR_M_ARM) != 0){
					csr &= ~LLC_CSR_M_ARM;
					decim_count = dg.settings.decim_base =
						LLC_GET_DECIM(csr);
					setTarget(*MBOX[BP_MB_LLC_DATA_ADDR]);
					DMA_ARM(ai_dma);
					
					enable_acq();
					csr |= LLC_CSR_S_IS_ARMED;
				}

				if ((csr&LLC_CSR_M_LLC200_INIT) != 0){
					do_LL200_init();
					csr &= ~LLC_CSR_M_LLC200_INIT;
				}
				if ((csr&LLC_CSR_M_SOFTCLOCK) != 0){
					dbg(2, "LLC_CSR_M_SOFTCLOCK");



					if (AIFIFO_NOT_EMPTY(fifstat)){
						dbg(1, 
						    "FIFO_NOT_EMPTY on SCLOCK"
						    "s: 0x%08x f: 0x%08x ", 
						    *ACQ200_SYSCON,
						    fifstat);
					}
					enable_fifo();
					if (!hard_trigger_selected()){
						UPDATE_TLATCH(
							dg.status.tlatch);
						soft_trigger();
					}
					dg.status.trigger_state = TS_REQUESTED;
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

				dg.settings.auto_incr = 
					(csr&LLC_CSR_M_AUTOINCR) != 0;

				csr &= ~LLC_CSR_S_TCYCLE;
				csr |= LLC_MAKE_TCYCLE( tcycle );

				SACK;
 				dbg(3, "%8s 0x%08x", "SACK", csr);
			}
		}
		/** end of loop tidy up */
		triggered = (*ACQ200_SYSCON & ACQ200_SYSCON_TR) != 0;
		switch (dg.status.trigger_state){
		case TS_IDLE:
		case TS_REQUESTED:
			if (triggered){
				dg.status.trigger_state = TS_TRIGGERED;
			}
			break;
		case TS_TRIGGERED:
			if (!triggered){
				dg.status.trigger_state = TS_RESCINDED;
				REPORT_ERROR_AND_QUIT(
					"ERROR: Trigger RESCINDED %d",
					dg.status.iter);
			}
			break;
		default:
			REPORT_ERROR_AND_QUIT(
				"ERROR Internal state inconsistent %d",
				dg.status.iter);
		}
	}

quit_the_game:
	if (snack_on_quit){
		long j1 = jiffies;
		while (!COMMAND){
			++dg.status.waiting_for_permission_to_quit;
			if (jiffies - j1 > 10*HZ){
				err("TIMEOUT on host handshake");
				break;
			}
		}
		SNACK;
	}
	dbg(1, "QUITTING %s", dg.emergency_stop? "ESTOP": "");
	dg.emergency_stop = 0;

	llPostamble();
	return;
}


static void acq200_llc_loop(int entry_code)
/**
 * acq200_llc_loop
 * @entry_code - possible enhancements
 *
 */
{
	if (llc_onEntry(entry_code) == 0){
		llc_loop(entry_code);
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
		       dg.settings.target,
		       dg.settings.auto_incr );
}

static DEVICE_ATTR(settings, S_IRUGO, show_settings, 0);


static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq200_llc_driver_name,
		       acq200_llc_driver_string,
		       acq200_llc_driver_version,
		       acq200_llc_copyright
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
        return sprintf(buf, "0x%08x\n", dg.imask);
}

static DEVICE_ATTR(imask, S_IWUGO|S_IRUGO, show_llc_mask, set_llc_mask);


static ssize_t set_fifo_th(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned hot_th, cold_th;

	if (sscanf(buf, "%d %d", &hot_th, &cold_th) == 2 ||
	    sscanf(buf, "0x%x 0x%x", &hot_th, &cold_th) == 2){
		HOT_TH = hot_th;
		COLD_TH = cold_th;
	}

	return strlen(buf);
}
static ssize_t show_fifo_th(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d %d\n", HOT_TH, COLD_TH);
}

static DEVICE_ATTR(fifo_th, S_IWUGO|S_IRUGO, show_fifo_th, set_fifo_th);



static ssize_t set_llc_debug(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &acq200_llc_debug);
	return strlen(buf);
}
static ssize_t show_llc_debug(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", acq200_llc_debug);
}

static DEVICE_ATTR(debug, S_IWUGO|S_IRUGO, show_llc_debug, set_llc_debug);

static ssize_t show_status(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
#define FFMT "%20s: "
#define STAPRINT(fmt, field) \
        len += sprintf(buf+len,  FFMT fmt "\n", #field, dg.status.field)

	len += sprintf(buf+len, "%20s: %d %10s %d\n", 
		       "channels", getNumChan(CHANNEL_MASK),
		       "shot", dg.shot);

	len += sprintf(buf+len, "block_len 0x%04x %d  HT:0x%x CT:0x%x\n",
		       BLOCK_LEN, BLOCK_LEN, HOT_TH, COLD_TH);
	len += sprintf(buf+len, "samples per block %d\n",
		       BLOCK_LEN/get_sample_size(CHANNEL_MASK));
	len += sprintf(buf+len, FFMT "%s", "TRIGGER_STATE",
		       TRIGGER_STATES_STRINGS[dg.status.trigger_state]);
	STAPRINT("%d", trigger_state);
	STAPRINT("%u", sample_count);
	STAPRINT("%u", t0);
	STAPRINT("%u", tlatch);
	STAPRINT("%u", tinst);
	len += sprintf(buf+len, "%20s: %d\n", "tprocess", 
		       dg.status.tinst - dg.status.tlatch);
	STAPRINT("\"%s\"", report);
	STAPRINT("%d", errline);
	STAPRINT("%d", iter);
	STAPRINT("%u", fifo_poll_count);
	STAPRINT("%u", dmac_poll_count[0]);
	STAPRINT("%u", dmac_poll_count[1]);
	STAPRINT("%d", waiting_for_permission_to_quit);

	return len;
#undef STAPRINT
}

static DEVICE_ATTR(status, S_IRUGO, show_status, 0);



static ssize_t set_llc(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int entry_code = 0;

	if (sscanf(buf, "%d", &entry_code) == 1 && entry_code > 0){
		acq200_llc_loop(entry_code);
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



static ssize_t set_block_len(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int bl;
	if (sscanf(buf, "%d", &bl) == 1){
		select_block_def(bl);
	}
	return strlen(buf);
}
static ssize_t show_block_len(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", BLOCK_LEN);
}

static DEVICE_ATTR(block_len, S_IWUGO|S_IRUGO, 
		   show_block_len, set_block_len);

static ssize_t show_LLC200_INIT_size(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%d\n", sizeof(struct LLC200_INIT));
}

static DEVICE_ATTR(LLC200_INIT_size, S_IRUGO, show_LLC200_INIT_size, 0);




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
	return sprintf(buf, "%04x\n", CHANNEL_MASK);
}

static DEVICE_ATTR(channel_mask, S_IRUGO, show_channel_mask, 0);


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



static int mk_llc_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_run);
	DEVICE_CREATE_FILE(dev, &dev_attr_stats);
	DEVICE_CREATE_FILE(dev, &dev_attr_mode);
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_imask);
	DEVICE_CREATE_FILE(dev, &dev_attr_debug);
	DEVICE_CREATE_FILE(dev, &dev_attr_settings);
	DEVICE_CREATE_FILE(dev, &dev_attr_emergency_stop);
	DEVICE_CREATE_FILE(dev, &dev_attr_block_len);
	DEVICE_CREATE_FILE(dev, &dev_attr_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mask);
	DEVICE_CREATE_FILE(dev, &dev_attr_LLC200_INIT_size);
	DEVICE_CREATE_FILE(dev, &dev_attr_IODD);
	DEVICE_CREATE_FILE(dev, &dev_attr_fifo_th);
	return 0;
}


static void acq200_llc_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq200_llc_driver;



static int device_matches(struct device* device, char* match[2])
{
	if (match[I2C_DRVTYP] == '\0' || match[I2C_DEVID] == '\0'){
		return 0;
	}else{
		return strcmp(device->driver->name, match[I2C_DRVTYP]) == 0 &&
			strcmp(device->bus_id, match[I2C_DEVID]) == 0;
	}
}

static int collect_vr_device(
	struct device* device, void* calldata)
{
	if (device_matches(device, voff)){
		dg.voff = device;
	}else if (device_matches(device, vr1) ||
	          device_matches(device, vr2) ||
		  device_matches(device, vr3)){
		dg.vr_dev[dg.vr_dev_count++] = device;
	}else{
		;
	}
	return 0;
}

/** deprecated in 2.6.18, but I like it ... */
/**
 *	find_bus - locate bus by name.
 *	@name:	name of bus.
 *
 *	Call kset_find_obj() to iterate over list of buses to
 *	find a bus by name. Return bus if found.
 *
 *	Note that kset_find_obj increments bus' reference count.
 */
struct bus_type * find_bus(char * name)
{
#if 0
	struct kobject * k = kset_find_obj(&bus_subsys.kset, name);
	return k ? to_bus(k) : NULL;
#else
#warning WORKTODO - je ne comprend pas
	return 0;
#endif
}

static void locate_i2c_devices(void)
{
	struct bus_type* bt = find_bus("i2c");
	int data;

	info("bt found %p", bt);

	if (bt){
		bus_for_each_dev(bt, NULL, &data, collect_vr_device);
	}
}
static int acq200_llc_probe(struct device *dev)
{
	info("");
	dg.llc200_init_buf = kmalloc(sizeof(struct LLC200_INIT), GFP_KERNEL);
	select_block_def(DEFAULT_BL);
	mk_llc_sysfs(dev);
	locate_i2c_devices();
	return 0;
}

static int acq200_llc_remove(struct device *dev)
{
	kfree(dg.llc200_init_buf);
	return 0;
}


static struct device_driver acq200_llc_driver = {
	.name     = "acq200_llc",
	.probe    = acq200_llc_probe,
	.remove   = acq200_llc_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq200_llc_device = {
	.name = "acq200_llc",
	.id   = 0,
	.dev = {
		.release    = acq200_llc_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq200_llc_init( void )
{
	int rc;
	acq200_debug = acq200_llc_debug;

	rc = driver_register(&acq200_llc_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acq200_llc_device);
}


static void __exit
acq200_llc_exit_module(void)
{
	info("");
	platform_device_unregister(&acq200_llc_device);
	driver_unregister(&acq200_llc_driver);
}

module_init(acq200_llc_init);
module_exit(acq200_llc_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 Low Latency Control");


