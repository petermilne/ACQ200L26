/* acq132-fifo.c customisation for acq132-fifo driver                        */
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
#define ACQ132
#define ACQ_IS_INPUT 1

#define MODEL_VERID \
"$Id: acq132-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $ B1011\n"

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#define ACQ132_VERID "$Revision: 1.13 $ " __DATE__ " " __TIME__

#define MTTR2	0x80

#include <linux/platform_device.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"

#include "acq200-fifo.h"
#include "acq132.h"


#include "acq196-AO.h"
#include "acq196-offset.h"

#define INT_CLK_CALC_ROUNDING 0x80


int int_clk_calcmode;
module_param(int_clk_calcmode, int, 0664);

int stub_transform;
module_param(stub_transform, int, 0664);

int acq132_transform_debug = 0;
module_param(acq132_transform_debug, int, 0664);

int acq132_late_inten;
module_param(acq132_late_inten, int, 0664);

int acq132_trigger_debug = 0;
module_param(acq132_trigger_debug, int, 0664);

#define TBG if (acq132_transform_debug) dbg

#define AICHAN_DEFAULT 32


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

static void init_endstops( int count );   /* @@todo SHOULD BE IN HEADER */


static int enable_soft_trigger(void);
static int enable_hard_trigger(void);
/* return 1 on success, -1 on error */

#define DBGSF(s)						\
        if (acq132_trigger_debug)				\
	dbg(1, "%4d S: 0x%08x FC: 0x%08x FX: 0x%08x %s",	\
	__LINE__,						\
        *SYSCON, *ACQ196_FIFCON, *FIFSTAT, s)


static int acq196_trigger_detect(void)
{
	return (*SYSCON & ACQ196_SYSCON_TRIGGERED) != 0;
}

static unsigned check_fifstat(
	struct DMC_WORK_ORDER *wo, u32 fifstat, u32* offset)
/* returns 1 if trigger handled */
/** @todo this needs refactoring with the ACQ216 version ! */
{
	int adc_ev = fifstat&ACQ196_FIFSTAT_ADC_EVX;

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
	acq196_fifcon_init_all(0);
	acq196_fifcon_set_all(ACQ196_FIFCON_RESET_ALL);
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







static struct DevGlobs acq132_dg = {
	.btype = BTYPE_AI,
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
	.FIFERR = 0x00000060,       /* HOT OVER+UNDER */
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
/* 	.eoc_int_modulo_mask = 3,	*/
	.eoc_int_modulo_mask = 0,
	.activate_event_on_arm = 1,

	.dcb.dcb_max = 2048,
	.dcb.dcb_max_backlog = 2048
};


#define MYDG &acq132_dg

/* Slave FPGA Load:
The complete programming sequence is 

#1. set the Programming Mode to 1 wait for INIT to return high 
#2 . then set UPDATE and  
#2 start loading the data 16 bits at a time, and 
	checking the BUSY bit before sending the next data 
	checking that INIT does not go low during load. 
#3 After load is complete check that DONE has gone high
#4 clear Programming Mode.

*/

#define TIMEOUT_RET(sta)						\
	for (timeout = 1000; sta; --timeout){				\
		if (timeout == 0){					\
			dbg(1, "TIMEOUT %s %d\n", #sta, __LINE__ );	\
			return -ETIMEDOUT;				\
		}else{							\
			schedule();					\
		}							\
}


static int acq132_sfpga_load_open (struct inode *inode, struct file *file)
{
	int timeout;

	/* #1 */
	sfpga_conf_clr_all();
	sfpga_conf_set_prog();
	schedule();
	TIMEOUT_RET(!sfpga_conf_init_is_hi());
	return 0;
}


static ssize_t acq132_sfpga_load_write ( 
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
{
	int timeout;
	u16 data;
	int isend = 0;
	unsigned long startoff = *offset;
/* #2 */       
	for (isend = 0; isend < len; isend += sizeof(short)){
		if (copy_from_user(&data, buf+isend, sizeof(short))){
			return -EFAULT;
		}

		dbg(1, "offset %ld isend %d cursor %ld data 0x%04x",
		    startoff, isend, startoff+isend, data);

		TIMEOUT_RET(sfpga_conf_get_busy());

		if (!sfpga_conf_init_is_hi()){
			err("INIT down before send %lu", startoff + isend);
			return -EFAULT;
		}
		
		sfpga_conf_send_data(data);

		if (!sfpga_conf_init_is_hi()){
			err("INIT down after send %lu", startoff + isend);
			return -EFAULT;
		}
	}

	*offset += isend;
	return isend;
}

static int acq132_sfpga_load_release (struct inode *inode, struct file *file)
{
	int timeout;
	TIMEOUT_RET(sfpga_conf_get_busy());

	/* #3, #4 */
	if (!sfpga_conf_done()){
		err("Quitting bad done status");
		return -EFAULT;
	}
	sfpga_conf_clr_prog();
	info("ADC_FPGA load complete");
	return 0;
}

static struct file_operations acq132_sfpga_load_ops = {
	.open = acq132_sfpga_load_open,
	.release = acq132_sfpga_load_release,
	.write = acq132_sfpga_load_write
};
#include "acq200-fifo.c"


static void enable_acq132_start(void)
{
	int rc;

	dbg(3, "OK: let's trigger FIFCON: 0x%08x SYSCON: 0x%08x", 
	    *FIFSTAT, *SYSCON);

	DBGSF("TEMPORARY: blip enable");
	disable_acq();

	if (acq196_trigger_detect()){
		err("WOAAH - triggered already, an not yet enabled not good\n"
		      "FIFCON: 0x%08x\n"
		      "FIFSTAT:0x%08x\n"
		      "SYSCON: 0x%08x",
		      *FIFCON, *FIFSTAT, *SYSCON);
		finish_with_engines(-__LINE__);
	}
	DBGSF("set ACQEN");
	enable_acq();

	if (acq196_trigger_detect()){
		err("WOAAH - triggered already, not good\n"
		      "FIFCON: 0x%08x\n"
		      "FIFSTAT:0x%08x\n"
		      "SYSCON: 0x%08x",
		      *FIFCON, *FIFSTAT, *SYSCON);
		finish_with_engines(-__LINE__);
	}

	DMC_WO->trigger_detect = acq196_trigger_detect;

	DBGSF("set events");
	signalCommit(CAPDEF->ev[0]);
	signalCommit(CAPDEF->ev[1]);

	DBGSF("FINAL:next enable FIFCON");
	enable_fifo(CAPDEF->channel_mask);

	preEnable();

	DBGSF("now trigger");
	dbg(3, "use hard trigger if enabled %s", 
	      CAPDEF->trig->is_active? "HARD": "soft");

	if (CAPDEF->trig->is_active){
		rc = enable_hard_trigger();
	}else{
		rc = enable_soft_trigger();
	}
	onEnable();

	if (DMC_WO->trigger_detect()){
		onTrigger();
	}

	/** @todo pgm: try late interrupt enable (stub in preEnable()) */
	if (acq132_late_inten){
		*ACQ200_ICR = 0;
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

static int enable_hard_trigger(void)
{
	int nloop = 0;

	DBGSF("ACQEN...");

	signalCommit(CAPDEF->trig);

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
	enable_acq132_start();
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

	dbg(1, "calling fifo_read_init_action()");
	rc = fifo_read_init_action();
	dbg(1, "return from fifo_read_init_action() %d", rc);
	if (rc==0){
		rc = oneshot_wait_for_done();
	}

	dbg(1, "returning %d", rc);
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

static struct CAPDEF* acq132_createCapdef(void)
{
	static struct CAPDEF _capdef = {
		.demand_len = 0x100000,
		.channel_mask = 0xffff,       
		.mode = M_SOFT_TRANSIENT,
		.pit_stop = 1
	};
	struct CAPDEF* capdef = kmalloc(sizeof(struct CAPDEF), GFP_KERNEL);

	memcpy(capdef, &_capdef, sizeof(struct CAPDEF));

	capdef_set_nchan(capdef, AICHAN_DEFAULT);
	capdef_set_word_size(capdef, 2);

	capdef->ev[0] = 
		createSignal("event0", 0, 5, 3, 0, 0, acq196_commitEv0);
	capdef->ev[1] = 
		createSignal("event1", 0, 5, 3, 1, 0, acq196_commitEv1);
	capdef->trig  = 
		createSignal("trig", 0, 5, 3, 0, 0, acq196_commitTrg);
	
	capdef->ext_clk = 
		createSignal("ext_clk", 0, 2, 0, 0, 0, acq196_commitAIClk);
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

static void acq132_destroyCapdef(struct CAPDEF *capdef)
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
static struct device_driver acq132_fpga_driver;

extern int init_arbiter(void);





/*
 * acq132 data is presented as 8-sample rows, in ROW_SAM blocks
 *
 * for block in BLOCKS
 *	for row in { A B C D }
 *		for ROW_SAM
 *			acq132_transform_row(to+ROW_OFF(row), from)
 *			update from
 *		update to
 *
 */
#define ROW_SAM		128
#define ROW_CHAN	8
#define ROW_WORDS	(ROW_SAM*ROW_CHAN)
#define ROW_BLOCK_OFF   (ROW_SAM*BLOCKSAM)	 // words in row-block

#define DEBUGGING


#ifdef DEBUGGING
short* to1;
short* to2;
short* from1;
short* from2;
#endif

void acq132_transform_row(
	short *to, short *from, int nsamples, int channel_sam)
/* read a block of data from to and farm 8 channels to from */
{
	union {
		unsigned long long ull[2];
		short ch[8];
	} buf;
	unsigned long long *full = (unsigned long long *)from;
	int sam;
	int chx;

	TBG(3, "to:%p from:%p nsamples:%d channel_sam:%d",
	    to, from, nsamples, channel_sam);

	if (stub_transform){
		TBG(3, "stub");
		return;

	}
	for (sam = 0; sam < nsamples; ++sam){
		buf.ull[0] = *full++;
		buf.ull[1] = *full++;

#ifdef DEBUGGING
		if ((short*)full < from1 || (short*)full > from2){
			err("from outrun at  %p %p %p sam:%d",
			    from1, full, from2, sam);
			return;
		}
#endif

		for (chx = 0; chx < ROW_CHAN; ++chx){

#ifdef DEBUGGING
			short *pto = to + chx*channel_sam + sam;
			if (pto < to1 || pto > to2){
				err("buffer outrun at %p %p %p chx:%d sam:%d",
				    to1, pto, to2, chx, sam);
				return;
			}
#endif
			to[chx*channel_sam + sam] = buf.ch[chx];
		}			
	}

	TBG(3, "99");	
}
static void acq132_transform(short *to, short *from, int nwords, int stride)
{
	const int nsamples = nwords/stride;	
	const int rows = stride/ROW_CHAN;
	const int block_words = rows * ROW_CHAN * ROW_SAM;
	int blocks = nwords/block_words;
	int nw = nwords;
	int block;
#define ROW_OFF(r)	((r)*ROW_CHAN*nsamples) 

	if (blocks * block_words < nwords){
		++blocks;
	}

	TBG(1, "blocks:%d", blocks);
	TBG(1, "nsamples:%d", nsamples);

#ifdef DEBUGGING
	to1 = to;
	to2 = to+nwords;
	TBG(1, "to   %p to %p", to1, to2);
	from1 = from;
	from2 = from+nwords;
	TBG(1, "from %p to %p", from1, from2);
#endif

	for (block = 0; block < blocks; ++block, nw -= block_words){
		const int bsamples = min(nw, block_words)/rows/ROW_CHAN;
		int row;

		TBG(2, "block:%d to:%p from:%p bsamples %d", 
		    block, to, from, bsamples);

		for (row = 0; row < rows; ++row){
			acq132_transform_row(
				to + ROW_OFF(row), 
				from, 
				bsamples,
				nsamples
			);
			from += bsamples*ROW_CHAN;
		}
		to += bsamples;
		TBG(2, "block:%d to:%p from:%p", block, to, from);
	}

	TBG(1, "99");
}



static 	struct Transformer transformer = {
	.name = "acq132",
	.transform = acq132_transform
};

static void register_custom_transformer(void)
{
	int it;
	it = acq200_registerTransformer(&transformer);
	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer %s NOT registered", transformer.name);
	}
}
static int acq132_fpga_probe(struct device *dev)
{
	init_pbi(dev);
	init_arbiter();

	register_custom_transformer();
	/* two reads allows for wedged PBI bus on jtag load */
	if (*ACQ196_BDR == ACQ196_BDR_DEFAULT || 
	    *ACQ196_BDR == ACQ196_BDR_DEFAULT    ){
		int rc = acqX00_fpga_probe(dev, IRQ_ACQ100_FPGA);
		if (rc != 0){
			err("fpga_probe() failed");
		}else{
			mk_sysfs(&acq132_fpga_driver);
		}
		return rc;
	}else{
		err("DEVICE NOT FOUND 0x%p=0x%08x", ACQ196_BDR, *ACQ196_BDR);
		return -ENODEV;
	}
}

static int acq132_fpga_remove(struct device *dev)
{
	acqX00_fpga_remove(dev, IRQ_ACQ100_FPGA);
	return 0;
}


static void acq132_fpga_dev_release(struct device * dev)
{
	info("");
}



static struct device_driver acq132_fpga_driver = {
	.name     = "acq132fpga",
	.probe    = acq132_fpga_probe,
	.remove   = acq132_fpga_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq132_fpga_device = {
	.name = "acq132fpga",
	.id   = 0,
	.dev = {
		.release    = acq132_fpga_dev_release,
		.dma_mask   = &dma_mask
	}

};


static int __init acq132_fifo_init( void )
{
	int rc;

	IPC = kmalloc(sizeof(struct IPC), GFP_KERNEL);
	memset(IPC, 0, sizeof(struct IPC));

	CAPDEF = acq132_createCapdef();
	init_dg();
	DG->bigbuf.tblocks.transform = acq200_getTransformer(2)->transform;
	init_phases();

	acq200_eoc_tasklet1.data = (unsigned long)&IPC->is_dma[1].eoc;

	acq200_debug = acq200_fifo_debug;

	info(ACQ132_VERID);

	dbg(1, "acq200_debug set %d\n", acq200_debug );


/* extra kobject init needed to hook driver sysfs WHY? */

	kobject_init(&acq132_fpga_driver.kobj);
	kobject_set_name(&acq132_fpga_driver.kobj, acq132_fpga_driver.name);

	if ((rc = kobject_register(&acq132_fpga_driver.kobj)) != 0){
		err("kobject_register() failed");
	}else if ((rc = driver_register(&acq132_fpga_driver)) != 0){
		err("driver_register() failed");
	}else if ((rc = platform_device_register(&acq132_fpga_device)) !=0){
		err("platform_device_register() failed");
	}else if ((rc = acq196_offset_fs_create(&acq132_fpga_device.dev)) !=0){
		err("failed to register offset_fs");
	}else if ((rc = acq196_AO_fs_create(&acq132_fpga_device.dev)) != 0){
		err("failed to register AO fs");
	}else{
		info("all done");
	}

	
	return rc;
}




static void __exit
acq132_fifo_exit_module(void)
// Remove DRIVER resources on module exit
{
//	unregister_reboot_notifier(&e1000_notifier_reboot);


	acq196_AO_fs_remove();
	acq196_offset_fs_remove();
	dbg(1, "rm_sysfs()" );
	rm_sysfs(&acq132_fpga_driver);
	dbg(1, "platform_device_unregister()");
	platform_device_unregister(&acq132_fpga_device);
	dbg(1, "pci_unregister_driver()" );
	driver_unregister(&acq132_fpga_driver);
	dbg(1, "kobject-unregister()");
	kobject_unregister(&acq132_fpga_driver.kobj);


	delete_dg();
	acq132_destroyCapdef(CAPDEF);
	kfree(IPC);
}



module_init(acq132_fifo_init);
module_exit(acq132_fifo_exit_module);

EXPORT_SYMBOL_GPL(acq200_setIntClkHz);
EXPORT_SYMBOL_GPL(acq200_setChannelMask);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ132 FIFO");

