/* acq196-fifo.c customisation for acq196-fifo driver                        */
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

#define DTACQ_MACH 2
#define ACQ196
#define ACQ_IS_INPUT 1

#define MODEL_VERID \
"$Id: acq196-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $\n"

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#define ACQ196_VERID "$Revision: 1.13 $ " __DATE__ " " __TIME__

#define MTTR2	0x80

#include <linux/platform_device.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"

#include "acq200-fifo.h"
#include "acq196.h"


#include "acq196-AO.h"
#include "acq196-offset.h"

#define INT_CLK_CALC_ROUNDING 0x80


int int_clk_calcmode;
module_param(int_clk_calcmode, int, 0644);

/** may need a longer reset if heavy DSP involved */
int fifo_reset_nsec = 1000;
module_param(fifo_reset_nsec, int, 0600);

/* force re-enable icr on overflow detect */
int force_icr_hot_en = 1;
module_param(force_icr_hot_en, int, 0600);

#define AICHAN_DEFAULT 96


#define HOT_FIFO_FULL_ENTRIES(fifstat) (((fifstat)&ACQ196_FIFSTAT_HOTPOINT)>>1)
#define COLD_FIFO_FULL_ENTRIES(fifcon) ((fifcon)&1)
#define HOT_FIFO_FREE_ENTRIES(fifstat) (0x10-HOT_FIFO_FULL_ENTRIES(fifstat))
#define COLD_FIFO_FREE_ENTRIES(fifcon) ((fifcon)&1)


#define FIFCON  ACQ196_FIFCON
#define FIFSTAT ACQ196_FIFSTAT
#define SYSCON  ACQ196_SYSCON_ADC

#define FIFCON_COLDUNDER ACQ196_FIFSTAT_UNDER

#define ACQ_INTEN ACQ100_ICR_HOTEN
#define DAC_INTEN ACQ100_ICR_DACEN


#define DTACQ_MACH_DRIVER_INIT(dev)
#define DTACQ_MACH_DRIVER_REMOVE(dev)
#define DTACQ_MACH_ON_SET_MODE(mode)

static void init_endstops( int count );   /* @@todo SHOULD BE IN HEADER */


static int enable_soft_trigger(void);
static int wait_hard_trigger(void);
/* return 1 on success, -1 on error */

#define DBGSF(s) \
        dbg(3, "S: 0x%08x FC: 0x%08x FX: 0x%08x %s", \
        *SYSCON, *ACQ196_FIFCON, *FIFSTAT, s)


static int acq196_trigger_detect(void)
{
	return (*SYSCON & ACQ196_SYSCON_TRIGGERED) != 0;
}

void disable_acq(void) 
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq196_syscon_clr_all(ACQ196_SYSCON_ACQEN);
}
void enable_acq(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq196_syscon_set_all(ACQ196_SYSCON_ACQEN);
}




static unsigned check_fifstat(
	struct DMC_WORK_ORDER *wo, u32 fifstat, u32* offset)
/* returns 1 if trigger handled */
/** @todo this needs refactoring with the ACQ216 version ! */
{
	int adc_ev = fifstat&ACQ196_FIFSTAT_ADC_EVX;

	if (wo->looking_for_pit == NOLOOK_FOR_PIT){
		return 0;
	}

	dbg(3, "F%08x %s %s", fifstat,
	    adc_ev? "TR":"tr", wo->looking_for_pit? "LOOKING":"-");

	if (adc_ev){
		/* avoid race condition by allowing onEvent to possibly
                 * clear down the event enable (1) BEFORE ACK event (2)
                 */

		if (wo->looking_for_pit){
			onEvent(wo, fifstat, offset);	/* (1) */
		}

		*FIFSTAT = ACQ196_FIFSTAT_ADC_EVX;	/* (2) */

		if (wo->looking_for_pit){
			dbg(2, "EVENT %08x at 0x%08x", fifstat, *offset);
			wo->looking_for_pit = 0;
			if (fifstat&ACQ196_FIFSTAT_ADC_EV0){
				DG->stats.event0_count++;
			}
			if (fifstat&ACQ196_FIFSTAT_ADC_EV1){
				DG->stats.event1_count++;
			}
			return fifstat|DMC_EVENT_MARKER;
		}
	}else{
		wo->looking_for_pit = 1;
	}

	return 0;
}


#define CHECK_FIFSTAT(wo, fifstat, offset) check_fifstat(wo, fifstat, offset)


#define SET_SIMULATION_MODE(enable)				\
        do {							\
	        if (enable){					\
		        *SYSCON |= ACQ196_SYSCON_SIM_MODE;	\
	        }else{						\
		        *SYSCON &=~ ACQ196_SYSCON_SIM_MODE;	\
	        }						\
        } while(0)	

void acq200_reset_fifo(void)
{
	acq196_fifcon_clr_all(ACQ196_FIFCON_RESET_ALL|ACQ196_FIFCON_ENABLE_ALL);
	acq196_fifcon_set_all(ACQ196_FIFCON_RESET_ALL);
	nsleep(fifo_reset_nsec);
	acq196_fifcon_clr_all(ACQ196_FIFCON_RESET_ALL);
	*FIFSTAT = *FIFSTAT;
}

#define GMC_ROOLS_OK     /* error in hw spec - clkdiv out by one */
#ifdef GMC_ROOLS_OK
#define CLKDIV_OFFSET 0  /* observed by GMC */
#else
#define CLKDIV_OFFSET 1  /* fpga spec */
#endif

static void _setIntClkHz(int hz, long masterclk, u32 clksel)
{
#define MAXDIV    0x0000fffe
	u32 clkdiv;


	if ( hz > masterclk/2 ) hz = masterclk/2;

	clkdiv = (masterclk / hz) + CLKDIV_OFFSET;

	if ( clkdiv > MAXDIV ) clkdiv = MAXDIV;
	if ( clkdiv < 2 )      clkdiv = 2;

	acq196_syscon_clr_all(ACQ196_SYSCON_EXTCLK);
	*ACQ196_CLKCON &= ~ACQ196_CLKCON_CS_MASK;
	*ACQ196_CLKCON |= clksel;
	*ACQ200_CLKDAT = clkdiv;

	acq200_clk_hz = masterclk / (clkdiv - CLKDIV_OFFSET);

	if ((int_clk_calcmode&INT_CLK_CALC_ROUNDING) != 0){
		acq200_clk_hz = acq200_rounding(
			acq200_clk_hz, PRECISION(clkdiv));
	}

	dbg( 1, "set:%7d Hz clkdiv 0x%08x act 0x%08x %d Hz MCLK: %ld Hz\n",
	     hz, clkdiv, *ACQ200_CLKDAT, acq200_clk_hz, masterclk);
}

#define MASTERCLK_66 66666666  

#define MASTERCLK MASTERCLK_66


	static struct IntClkConsts {
		long masterclk;
		u32  clksel;
	} intclk[2] = {
		{
			.masterclk = MASTERCLK_66,
			.clksel    = ACQ196_CLKCON_CS_66M
		}
	};
#define MAXSEL 1


void acq200_setIntClkHz( int hz )
{
	int actual[2];
	int isel;
	u32 clkdiv;
	int deltamin = MASTERCLK;
	int imin = 0;
	int delta;
	int maxsel = MAXSEL;

	if ( hz == 0 ){
		signalCommit(CAPDEF->ext_clk);
	}else{
		if ( hz < 1 ){
			hz = 1;
		}else if ( hz > MASTERCLK/2 ){
			hz = MASTERCLK/2;
		}

		for (isel = 0; isel != maxsel; ++isel){
			clkdiv = intclk[isel].masterclk/hz + 1;
			actual[isel] = intclk[isel].masterclk/(clkdiv-1);
/*
 * bias to nearest clk BELOW
 */
			if (actual[isel] > hz){
				clkdiv += 1;
				actual[isel] = 
					intclk[isel].masterclk/(clkdiv-1);
			}
			delta = abs(hz - actual[isel]);
			if (delta < deltamin ){
				deltamin = delta;
				imin = isel;
			}
		}
	
		_setIntClkHz(hz, intclk[imin].masterclk, intclk[imin].clksel);
	}
}

static int fifo_read_init_action(void);


/*
 * WORKTODO

 */

#if DMA_BLOCK_LEN == 4096
#define ACQ196_FIFO_NBLOCKS(fifcon) (1)
#else
/*
 * fifcon 0..31 => 8K
 * we right shift one st 0..15 => 8K (only 4 bits needed).
 * => need 2 entries per K
 * roundup to be sure
 */

#define HOTPOINT(fifstat) ((fifstat)&ACQ196_FIFSTAT_HOTPOINT)
#define NEWPOINT(point) ((point)>>1)
#define ACQ196_FIFO_NBLOCKS(fifstat)  ((NEWPOINT(HOTPOINT(fifstat))-1)>>1)
#endif

/*
 * limit to 3 blocks because that is all the isr s/w can handle
 * NB: 2 blocks for FIQ
 */

#define GET_FIFO_NBLOCKS(nblocks, fifstat) \
do { \
        nblocks = ACQ196_FIFO_NBLOCKS(fifstat); \
        nblocks = min(3, nblocks); \
} while(0)


static int acq200_fpga_fifo_read_open (struct inode *inode, struct file *file);
static ssize_t acq200_fpga_fifo_read_buf_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	);



static void _lost_enable_action(int sfc0, int sfc1)
{
	u32 fifstat = *ACQ196_FIFSTAT;
	u32 icr = *ACQ200_ICR;
	int delta;

	if (likely(sfc1 > sfc0)){
		delta = sfc1 - sfc0;
	}else{
		delta = 0x10000 + sfc1 - sfc0;
	}

	
	if ((fifstat&ACQ196_FIFSTAT_HOT_HT) != 0){
		if (force_icr_hot_en){
			*ACQ200_ICR = icr|ACQ100_ICR_HOTEN;
		}
		err("HT fifsta:%08x icr:%08x,ICR:%08x sfc:%d FIQ:%d "
		    "EOC:%d TBF:%d TBE:%d",
		    fifstat, icr, *ACQ200_ICR, delta,
		    DG->stats.num_fifo_ints,
		    DG->stats.num_eoc_ints,
		    free_block_count(),
		    empty_block_count());
	}
}


static void acq196_cdog_check(int cdog_code)
{
/* history: [0] = n-1, [1] = n-2 */
	static int soft_fifo_count[2];
	static int nz_count;

	if (cdog_code == CDOG_REFRESH){
		int sfc = acq196_get_soft_fifo_count();

		if (sfc != soft_fifo_count[0]){
			/* act every second non-zero */
			if ((++nz_count & 0x1) == 0){
				_lost_enable_action(soft_fifo_count[1], sfc);
			}

			soft_fifo_count[1] = soft_fifo_count[0];
			soft_fifo_count[0] = sfc;
		}else{
			nz_count = 0;
		}
	}
}



static struct DevGlobs acq196_dg = {
	.btype = BTYPE_ACQ216,
	.hitide = 7,
	.max_alloc = 10240,
	.busywait = 0,
	.sample_read_start = 0,
	.sample_read_stride = 1,
	.bigbuf.tblocks.blocklen = TBLOCK_LEN,
	.bigbuf.tblocks.blt = blt_memcpy,
	.enable_from_eoc_isr = 1,
	.bh_unmasks_eoc = 0,
	.is_oneshot = 1,
	.use_fiq = 1,
	.pit_store.max_pits = 2000,
#ifdef FIQDEBUG
	.CAFEBABE = 0xcafebabe,
	.FEEDCODE = 0xfeedc0de,
	.DEADBEEF = 0xdeadbeef,
#endif
#ifdef ALL_FIFO_FLAGS_HAPPY
	.FIFERR = ACQ196_FIFSTAT_ADC_ERR,
#else
	.FIFERR = 0x00022260,       /* OVER|OVER|OVER|OVER+UNDER */
#endif
	.pulse.pulse_count = 0, 
	.pulse.ibit = 1,
	.pulse.active_high = 0,
	.pulse.start_delay = 20,   /* msec */
	
	.empty_fill_threshold = 4096,
	.put_max_empties = 1024,
	.get_max_active = 1024,
	.active_batch_threshold = 64,
	.init_endstops = 32,
	.eoc_int_modulo_mask = 3,
	.activate_event_on_arm = 1,

	.dcb.dcb_max = 2048,
	.dcb.dcb_max_backlog = 2048
};


#define MYDG &acq196_dg

#define ARCH_CDOG(cdog_code)	acq196_cdog_check(cdog_code)

#include "acq200-fifo.c"


static void enable_acq196_start(void)
{
/** previously in enable_regular_operation()
	disable_acq();
	acq200_reset_fifo();
*/
	int rc;

	dbg(3, "OK: let's trigger FIFCON: 0x%08x SYSCON: 0x%08x", 
	    *FIFSTAT, *SYSCON);

	DBGSF("TEMPORARY: blip enable");
	disable_acq();				/** was disabled already? */


	DBGSF("set events");
	if (CAPDEF->trig->is_active){
		signalCommit(CAPDEF->trig);
	}
	DMC_WO->trigger_detect = acq196_trigger_detect;

	signalCommit(CAPDEF->ev[0]);
	signalCommit(CAPDEF->ev[1]);

	DBGSF("FINAL:next enable FIFCON");
	enable_fifo(CAPDEF->channel_mask);

	preEnable();

#if 0
/* enabling LOWLAT zaps the interrupt altogether */
#if defined ACQ196c
	*ACQ196_SYSCON_ADC |= ACQ196_SYSCON_LOWLAT;
#endif
#endif
	if (acq196_trigger_detect()){
		err("WOAAH - triggered already, and not yet enabled not good\n"
		      "FIFCON: 0x%08x\n"
		      "FIFSTAT:0x%08x\n"
		      "SYSCON: 0x%08x",
		      *FIFCON, *FIFSTAT, *SYSCON);
		finish_with_engines(-__LINE__);
	}
	DBGSF("set ACQEN");
	enable_acq();

	if (DMC_WO->trigger_detect()){
		err("WOAAH - triggered already, not good\n"
		      "FIFCON: 0x%08x\n"
		      "FIFSTAT:0x%08x\n"
		      "SYSCON: 0x%08x",
		      *FIFCON, *FIFSTAT, *SYSCON);
		finish_with_engines(-__LINE__);
	}

	onEnable();

	DBGSF("now trigger");
	dbg(3, "use hard trigger if enabled %s", 
	      CAPDEF->trig->is_active? "HARD": "soft");

	if (CAPDEF->trig->is_active){
		rc = wait_hard_trigger();
	}else{
		rc = enable_soft_trigger();
	}

	if (DMC_WO->trigger_detect()){
		onTrigger();
	}
}



#define IN_RANGE(xx, ll, rr) ((xx)>=(ll)&&(xx)<=(rr))



static int enable_soft_trigger(void)
{
#define FIFO_EMPTY (ACQ200_FIFCON_COLDEMPTY|ACQ200_FIFCON_HOTEMPTY)
#define MAXWAIT 1000

	int nwait;

	for(soft_trigger_retry = 0; 
	    soft_trigger_retry < MAXRETRY;
	    ++soft_trigger_retry) {
		DBGSF("set SOFTTRIG");
		acq196_syscon_set_all(ACQ196_SYSCON_SOFTTRIG);
		DBGSF("clr SOFTTRIG");
		acq196_syscon_clr_all(ACQ196_SYSCON_SOFTTRIG);
		DBGSF("should be triggered");
		nwait = 0;

		do {
			if (acq196_trigger_detect()){
				return 1;
			}

			/**
			 *  some FPGA's don't support TRIGGERED - fallback:
			 */
			if ((*ACQ196_FIFSTAT&ACQ196_FIFSTAT_HOT_NE) != 0){
				return 1;				
			}
			if (nwait > 10){
				yield();
			}
		} while(++nwait < MAXWAIT);
	}

	DBGSF("failed to trigger");
	sprintf(errbuf, "ERROR failed to trigger after %d retries",
		MAXRETRY);
	DMC_WO->error = errbuf;
	return -1;
}

static int wait_hard_trigger(void)
{
	int nloop = 0;

	DBGSF("ACQEN...");

	while(!acq196_trigger_detect()){
		if (DG->finished_with_engines){
			return -1;
		}
		if ((++nloop&0xfff) == 0){
			DBGSF("POLL TRIG");
		}
		yield();
	}

	DBGSF("..");
	return 1;
}


#ifdef FIQDEBUG
static void debug_fill(void)
{
	int ifill;

	for (ifill = 0; ifill != 16; ++ifill){
		DG->istack[ifill] = ifill;
	}
}
#endif



static int fifo_read_init_action(void)
{
#ifdef FIQDEBUG
	debug_fill();
#endif
	
	DG->fiferr = 0;

	dbg(1, "%10s reset: F:0x%08x S:0x%08x",
	    "BEFORE", *FIFSTAT, acq196_getSyscon());

	*ACQ200_ICR = 0;

	acq196_fifcon_clr_all(ACQ196_FIFCON_HOT_HITIDE);
	acq196_fifcon_set_all(HITIDE);

	dbg(1, "%10s reset: F:0x%08x S:0x%08x", 
	    "AFTER", *FIFSTAT, acq196_getSyscon());


/*
 * pgm 20040125: keep soft trigger high (then it cannot be construed as
 * a falling edge on DAQEN?
 */
	dbg(DISABLE_ACQ_DEBUG, "ACQ196_SYSCON_SOFTTRIG|ACQ196_SYSCON_ACQEN");
	acq196_syscon_set_all(ACQ196_SYSCON_SOFTTRIG|ACQ196_SYSCON_ACQEN);

	dbg(1, "%10s reset: F:0x%08x S:0x%08x", 
	    "ENABLED", *ACQ196_FIFCON, acq196_getSyscon());

	if (DG->activate_event_on_arm){
		struct Phase* phase = DMC_WO->now;

		if (phase->ev){
			activateSignal(phase->ev);
		}
	}
	enable_acq196_start();
	return 0;
}

static int acq200_fpga_fifo_read_open (struct inode *inode, struct file *file)
{
	int rc;
	int len = LEN;    /* temp hack */

	dbg(1,"01");
	enable_regular_operation();

	clear_buffers();
	init_endstops(INIT_ENDSTOPS);

	fifo_open( PCI_DMA_FROMDEVICE );

/* and start the capture */

	dma_sync( va_buf( DG ), len, PCI_DMA_FROMDEVICE );
#ifdef WORKTODO_PGM
	build_dmad( va_buf( DG ), USE_DG, PCI_DMA_FROMDEVICE );
#else
	build_dmad( va_buf( DG ), len, PCI_DMA_FROMDEVICE );
#endif

	init_phases();
	if ((*ACQ196_RGATE & ACQ196_RGATE_MODE) != 0){
		DMC_WO->looking_for_pit = NOLOOK_FOR_PIT;
	}

	rc = fifo_read_init_action();
	if (rc==0){
		rc = oneshot_wait_for_done();
	}

	return rc;
}

static ssize_t acq200_fpga_fifo_read_buf_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	len = min(len, (size_t)4);

	dbg( 2, "len %d *offset %d", len, (int)*offset );

	if (copy_to_user( buf, va_buf_offset( DG, *offset ), len )){
		return -EFAULT;
	}
	*offset += len;

	return len;
}


static void init_pbi(struct device *dev)
{
	DG->fpga.regs.pa = ACQ200_FPGA_P;
	DG->fpga.regs.len = 0x400;
	DG->fpga.regs.va = (void*)ACQ200_FPGA;
	DG->fpga.fifo.pa = ACQ200_FPGA_P+ACQ196_FIFO_OFFSET;
	DG->fpga.fifo.len = 0x400;
	DG->fpga.fifo.va = (void*)(ACQ200_FPGA+ACQ196_FIFO_OFFSET);
}


static int _acq196_commitEvX(
	struct Signal* signal, 
	volatile u32* reg,
	int shift)
{
	u32 syscon = *reg;
	
	syscon &= ~(ACQ196_SYSCON_EV_MASK << shift);
	
	if (signal->is_active){
		u32 rising = signal->rising? 
			ACQ196_SYSCON_EV_RISING: ACQ196_SYSCON_EV_FALLING;

		syscon |= ((rising | acq196_lineCode(signal->DIx)) << shift);
	}	

	*reg = syscon;	
	return 0;
}


static int acq196_commitEv0(struct Signal* signal)
{
	return _acq196_commitEvX(
		signal, ACQ196_SYSCON_ADC, ACQ196_SYSCON_EV0_SHIFT);
}
static int acq196_commitEv1(struct Signal* signal)
{
	return _acq196_commitEvX(
		signal, ACQ196_SYSCON_ADC, ACQ196_SYSCON_EV1_SHIFT);
}
static int acq196_commitTrg(struct Signal* signal)
{
	return _acq196_commitEvX(
		signal, ACQ196_SYSCON_ADC, ACQ196_SYSCON_TRG_SHIFT);
}

static int acq196_commitAOTrg(struct Signal* signal)
{
	return _acq196_commitEvX(
		signal, ACQ196_SYSCON_DAC, ACQ196_SYSCON_TRG_SHIFT);
}

static int acq196_commitAXClk(struct Signal* signal, volatile u32* reg)
{
	u32 syscon = *reg;
	
	syscon &= ~ACQ196_SYSCON_EC_MASK; /* NO SHIFT */
	
	if (signal->is_active){

		if (signal->DIx == DIX_INTERNAL){
			syscon &= ~ACQ196_SYSCON_EXTCLK;
		}else{
			syscon |= signal->rising? ACQ196_SYSCON_EC_RISING: 0;
	                /* field value 0 => DI0 */
			syscon |= signal->DIx <<ACQ196_SYSCON_EC_SHIFT;
			syscon |= ACQ196_SYSCON_EXTCLK;
		}
	}	

	*reg = syscon;	
	return 0;
}


static int acq196_commitAIClk(struct Signal* signal)
{
	return acq196_commitAXClk(signal, ACQ196_SYSCON_ADC);
}

static int acq196_commitAOClk(struct Signal* signal)
{
	return acq196_commitAXClk(signal, ACQ196_SYSCON_DAC);
}

static int acq196_commitMasClk(struct Signal* signal)
{
	u32 clkcon = *ACQ196_CLKCON;
	
	clkcon &= ~(ACQ196_CLKCON_OCS_MASK|ACQ196_CLKCON_CLKMAS);

	if (signal->is_active){
                /* @@todo WARNING starts D0 == 0 */
		clkcon |= 
			ACQ196_CLKCON_OCS_DOx(signal->DIx)|
			ACQ196_CLKCON_CLKMAS;
	}
	*ACQ196_CLKCON = clkcon;
	return 0;
}



static int acq196_commitIntClkSrc(struct Signal* signal)
{
	u32 clkcon = *ACQ196_CLKCON;
	
	clkcon &= ~(ACQ196_CLKCON_CS_MASK);

	if (signal->is_active){
		clkcon |= ACQ196_CLKCON_CS_DIx(signal->DIx);
	}
	*ACQ196_CLKCON = clkcon;
	return 0;
}

static int acq196_commitSyncTrigSrc(struct Signal* signal)
{
	u32 clkcon = *ACQ196_CLKCON;
	
	clkcon &= ~(ACQ196_CLKCON_TR_MASK);

	if (signal->is_active){
		clkcon |= ACQ196_CLKCON_TR_DIx(signal->DIx);
	}
	*ACQ196_CLKCON = clkcon;
	return 0;	
}

static int acq196_commitMasSyncTrig(struct Signal *signal)
{
	u32 clkcon = *ACQ196_CLKCON;
	
	clkcon &= ~(ACQ196_CLKCON_OTR_MASK|ACQ196_CLKCON_TRMAS);

	if (signal->is_active){
                /* @@todo WARNING starts D0 == 0 */
		clkcon |= 
			ACQ196_CLKCON_OTR_DOx(signal->DIx)|
			ACQ196_CLKCON_TRMAS;
	}
	*ACQ196_CLKCON = clkcon;
	return 0;	
}

static int acq216_commitTcrSrc(struct Signal* signal)
{
	u32 clkcon = *ACQ216_TCR_IMM;
	
	clkcon &= ~ACQ216_TCR_TCS_MASK;	
	clkcon |= signal->DIx<<ACQ216_TCR_TCS_SHL;

	*ACQ216_TCR_IMM = clkcon;	
	return 0;
}

static struct CAPDEF* acq196_createCapdef(void)
{
	static struct CAPDEF _capdef = {
		.demand_len = 0x100000,
		.channel_mask = 0x00007,         /* 3 blocks of 32 */
		.mode = M_SOFT_TRANSIENT,
		.pit_stop = 1
	};
	struct CAPDEF* capdef = kmalloc(sizeof(struct CAPDEF), GFP_KERNEL);

	memcpy(capdef, &_capdef, sizeof(struct CAPDEF));

	capdef_set_nchan(capdef, 96);
	capdef_set_word_size(capdef, 2);

	capdef->ev[0] = 
		createSignal("event0", 0, 5, 3, 0, 0, acq196_commitEv0);
	capdef->ev[1] = 
		createSignal("event1", 0, 5, 3, 1, 0, acq196_commitEv1);
	capdef->trig  = 
		createSignal("trig", 0, 5, 3, 0, 0, acq196_commitTrg);
	
	capdef->ext_clk = 
		createSignal("ext_clk", 0, 5, 0, 0, 0, acq196_commitAIClk);
	capdef->ext_clk->has_internal_option = 1;
	
	capdef->ao_trig = 
		createSignal("ao_trig", 0, 5, 3, 0, 0, acq196_commitAOTrg);
	capdef->ao_clk = createSignal(
		"ao_clk", 0, 2, 0, 0, 0, acq196_commitAOClk);
	capdef->ao_clk->has_internal_option = 1;

	capdef->int_clk_src = createSignal(
		"int_clk_src", 0, 5, 0, 0, 0, acq196_commitIntClkSrc);
	capdef->mas_clk = createSignal(
		"mas_clk", 0, 5, 1, 0, 0,acq196_commitMasClk);
	capdef->mas_clk->is_output = 1;

	capdef->sync_trig_src = createSignal(
		"sync_trig_src", 3, 5, 3, 0, 0, acq196_commitSyncTrigSrc);

	capdef->sync_trig_mas = createSignal(
		"sync_trig_mas", 3, 5, 3, 0, 0, acq196_commitMasSyncTrig);
	capdef->sync_trig_mas->is_output = 1;

	capdef->counter_src = createSignal("counter_src", 0, 6, 6,0, 1, 
				       acq216_commitTcrSrc);
	/** @todo how to action? */
	capdef->counter_src->has_internal_option = 1;
	return capdef;
}

static void acq196_destroyCapdef(struct CAPDEF *capdef)
{
	destroySignal(capdef->ev[0]);
	destroySignal(capdef->ev[1]);
	destroySignal(capdef->trig);
	destroySignal(capdef->ext_clk);
	destroySignal(capdef->ao_trig);
	destroySignal(capdef->ao_clk);
	destroySignal(capdef->int_clk_src);
	destroySignal(capdef->mas_clk);
	destroySignal(capdef->sync_trig_src);
	destroySignal(capdef->sync_trig_mas);
	kfree(capdef);
}
static struct device_driver acq196_fpga_driver;

extern int init_arbiter(void);

static int acq196_fpga_probe(struct device *dev)
{
	init_pbi(dev);
	init_arbiter();

	if (*ACQ196_BDR == ACQ196_BDR_DEFAULT){
		int rc = acqX00_fpga_probe(dev, IRQ_ACQ100_FPGA);
		if (rc != 0){
			err("fpga_probe() failed");
		}else{
			mk_sysfs(&acq196_fpga_driver);
		}
		return rc;
	}else{
		err("DEVICE NOT FOUND 0x%p=0x%08x", ACQ196_BDR, *ACQ196_BDR);
		return -ENODEV;
	}
}

static int acq196_fpga_remove(struct device *dev)
{
	acqX00_fpga_remove(dev, IRQ_ACQ100_FPGA);
	return 0;
}


static void acq196_fpga_dev_release(struct device * dev)
{
	info("");
}



static struct device_driver acq196_fpga_driver = {
	.name     = "acq196fpga",
	.probe    = acq196_fpga_probe,
	.remove   = acq196_fpga_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq196_fpga_device = {
	.name = "acq196fpga",
	.id   = 0,
	.dev = {
		.release    = acq196_fpga_dev_release,
		.dma_mask   = &dma_mask
	}

};


static int __init acq196_fifo_init( void )
{
	int rc;

	IPC = kmalloc(sizeof(struct IPC), GFP_KERNEL);
	memset(IPC, 0, sizeof(struct IPC));

	CAPDEF = acq196_createCapdef();
	init_dg();
	DG->bigbuf.tblocks.transform = acq200_getTransformer(2)->transform;
	init_phases();

	acq200_eoc_tasklet1.data = (unsigned long)&IPC->is_dma[1].eoc;

	acq200_debug = acq200_fifo_debug;

	info(ACQ196_VERID);

	dbg(1, "acq200_debug set %d\n", acq200_debug );


/* extra kobject init needed to hook driver sysfs WHY? */

	kobject_init(&acq196_fpga_driver.kobj);
	kobject_set_name(&acq196_fpga_driver.kobj, acq196_fpga_driver.name);

	if ((rc = kobject_register(&acq196_fpga_driver.kobj)) != 0){
		err("kobject_register() failed");
	}else if ((rc = driver_register(&acq196_fpga_driver)) != 0){
		err("driver_register() failed");
	}else if ((rc = platform_device_register(&acq196_fpga_device)) !=0){
		err("platform_device_register() failed");
	}else if ((rc = acq196_offset_fs_create(&acq196_fpga_device.dev)) !=0){
		err("failed to register offset_fs");
	}else if ((rc = acq196_AO_fs_create(&acq196_fpga_device.dev)) != 0){
		err("failed to register AO fs");
	}else{
		info("all done");
	}

	
	return rc;
}




static void __exit
acq196_fifo_exit_module(void)
// Remove DRIVER resources on module exit
{
//	unregister_reboot_notifier(&e1000_notifier_reboot);


	acq196_AO_fs_remove();
	acq196_offset_fs_remove();
	dbg(1, "rm_sysfs()" );
	rm_sysfs(&acq196_fpga_driver);
	dbg(1, "platform_device_unregister()");
	platform_device_unregister(&acq196_fpga_device);
	dbg(1, "pci_unregister_driver()" );
	driver_unregister(&acq196_fpga_driver);
	dbg(1, "kobject-unregister()");
	kobject_unregister(&acq196_fpga_driver.kobj);


	delete_dg();
	acq196_destroyCapdef(CAPDEF);
	kfree(IPC);
}



module_init(acq196_fifo_init);
module_exit(acq196_fifo_exit_module);

EXPORT_SYMBOL_GPL(acq200_setIntClkHz);
EXPORT_SYMBOL_GPL(acq200_setChannelMask);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ100 FIFO");

