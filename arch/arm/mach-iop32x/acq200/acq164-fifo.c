/* acq164-fifo.c customisation for acq164-fifo driver                        */
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
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                   */
/* ------------------------------------------------------------------------- */

#define DTACQ_MACH 2
#define ACQ164
#define ACQ_IS_INPUT 1

#define MODEL_VERID							\
	"$Id: acq164-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $ B1016\n"

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#define ACQ164_VERID "$Revision: 1.13 $ " __DATE__ " " __TIME__

#define MTTR2	0x80

#include <linux/delay.h>
#include <linux/platform_device.h>

#include "acq200-signal2.h"
#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"

#include "acq200-fifo.h"
#include "acq164.h"


#include "acq196-AO.h"
#include "acq100-offset.h"

#define INT_CLK_CALC_ROUNDING 0x80
//#define BEST_ICSINPUT_HZ 20000000
#define BEST_ICSINPUT_HZ 1000000

int int_clk_calcmode;
module_param(int_clk_calcmode, int, 0664);

int stub_event_adjust = 0;
module_param(stub_event_adjust, int, 0600);

int ACQ200_CLK_HZ = 32768000;
module_param(ACQ200_CLK_HZ, int, 0644);

int obclock_input_rate = BEST_ICSINPUT_HZ;
module_param(obclock_input_rate, int, 0600);

int intclock_actual_rate;
module_param(intclock_actual_rate, int, 0444);


int rgm_with_es = 1;
module_param(rgm_with_es, int, 0600);

int acq164_trigger_debug = 0;
module_param(acq164_trigger_debug, int, 0600);

int ads1278_group_delay = 38;
module_param(ads1278_group_delay, int, 0600);

int check_clock_lock = 0;
module_param(check_clock_lock, int, 0600);

int ics527_reset_usec = 2;
module_param(ics527_reset_usec, int, 0600);

#ifdef PGMCOMOUT
/* acq132-gated.c */
extern struct file_operations acq132_gate_pulse_ops;
#endif

void acq164_set_obclock(int FDW, int RDW, int R, int Sx);


#define AICHAN_DEFAULT 64


#define HOT_FIFO_FULL_ENTRIES(fifstat) (((fifstat)&ACQ164_FIFSTAT_HOTPOINT)>>1)
#define COLD_FIFO_FULL_ENTRIES(fifcon) ((fifcon)&1)
#define HOT_FIFO_FREE_ENTRIES(fifstat) (0x10-HOT_FIFO_FULL_ENTRIES(fifstat))
#define COLD_FIFO_FREE_ENTRIES(fifcon) ((fifcon)&1)


#define FIFCON  ACQ164_FIFCON
#define FIFSTAT ACQ164_FIFSTAT
#define SYSCON  ACQ164_SYSCON

#define FIFCON_COLDUNDER ACQ164_FIFSTAT_UNDER

#define ACQ_INTEN ACQ100_ICR_HOTEN
#define DAC_INTEN ACQ100_ICR_DACEN

static void acq164_mach_on_set_mode(struct CAPDEF* capdef);

#define DTACQ_MACH_ON_SET_MODE(capdef) acq164_mach_on_set_mode(capdef)

static void init_endstops( int count );   /* @@todo SHOULD BE IN HEADER */

static void acq164_event_adjust(
	struct Phase *phase, unsigned isearch, 
	unsigned* first, unsigned* last);



#undef DTACQ_MACH_EVENT_ADJUST
#define DTACQ_MACH_EVENT_ADJUST(phase, isearch, first, last) \
	if (!stub_event_adjust){ \
		dbg(1, "DTACQ_MACH_EVENT_ADJUST: isearch 0x%08x", isearch); \
	        acq164_event_adjust(phase, isearch, first, last); \
	}



static int enable_soft_trigger(void);
static int enable_hard_trigger(void);
/* return 1 on success, -1 on error */




void disable_acq(void) 
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq164_syscon_clr(ACQ164_SYSCON_ACQEN);
}
void enable_acq(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq164_syscon_set(ACQ164_SYSCON_ACQEN);
}


#define DBGSF(s)							\
        if (acq164_trigger_debug)					\
		dbg(1, "%4d S: 0x%08x FC: 0x%08x FX: 0x%08x %s",	\
		    __LINE__,						\
		    *SYSCON, *ACQ164_FIFCON, *FIFSTAT, s)


static int acq164_trigger_detect(void)
{
	return (*ACQ164_SYSCON & ACQ164_SYSCON_TRIGGERED) != 0;
}

static unsigned check_fifstat(
	struct DMC_WORK_ORDER *wo, u32 fifstat, u32* offset)
/* returns 1 if trigger handled */
/** @todo this needs refactoring with the ACQ216 version ! */
{
	int adc_ev = fifstat&ACQ164_FIFSTAT_ADC_EVX;

	dbg(3, "F%08x %s %s", fifstat,
	    adc_ev? "TR":"tr", wo->looking_for_pit? "LOOKING":"-");

	if (wo->looking_for_pit == NOLOOK_FOR_PIT){
		return 0;
	}else if (adc_ev){
		/* avoid race condition by allowing onEvent to possibly
                 * clear down the event enable (1) BEFORE ACK event (2)
                 */

		if (wo->looking_for_pit){
			onEvent(wo, fifstat, offset);	/* (1) */
		}

		*ACQ164_FIFSTAT = ACQ164_FIFSTAT_ADC_EVX;	/* (2) */

		if (wo->looking_for_pit){
			dbg(2, "EVENT %08x at 0x%08x", fifstat, *offset);
			wo->looking_for_pit = 0;
			DG->stats.event0_count++;
			return fifstat|DMC_EVENT_MARKER;
		}
	}else{
		wo->looking_for_pit = 1;
	}

	return 0;
}


#define CHECK_FIFSTAT(wo, fifstat, offset) check_fifstat(wo, fifstat, offset)



/* no simulation mode in ACQ132, bit is used for VRANGE @todo */
#define SET_SIMULATION_MODE(enable)				


void acq200_reset_fifo(void)
{
	*ACQ164_FIFCON = 0;
	acq164_fifcon_set(ACQ164_FIFCON_ADCX_RESET);
	acq164_fifcon_clr(ACQ164_FIFCON_ADCX_RESET);
	*ACQ164_FIFSTAT = *ACQ164_FIFSTAT;
}





// ./ob_calc_527 --fin 20000  32000
static int _set_ob_clock(int khz)
/* set ICS527 clock */
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};

	static char fout_def[20];
	static char fin_def[20];
	static char *argv[6];
	int ii;
	int decim = khz < 4000? 4: 1;	/* ics min f = 4MHz */

	sprintf(fin_def, "%d", ACQ200_CLK_HZ/1000);

	khz *= decim;

	sprintf(fout_def, "%d", khz);

        ii = 0;
        argv[ii++] = "/usr/local/bin/ob_calc_527";
	argv[ii++] = "--fin";
	argv[ii++] = fin_def;	
        argv[ii++] = fout_def;
        argv[ii] = 0;

	dbg( 1, "call_usermodehelper %s %s %s %s\n", 
	     argv[0], argv[1], argv[2], argv[3] );

	ii = call_usermodehelper(argv [0], argv, envp, 0);

	if (ii != 0){
	        err("call done returned %d", ii );
		acq200_clk_hz = -1;
	}
	return ii;
}


static int set_ob_clock(int hz)
/* returns 0 = OK */
{	
	if (hz < 1000000){
		return -1;
	}
	reg_set_field(ACQ164_CLKCON, ACQ164_CLKCON_CS_SHIFT, 
		      ACQ164_CLKCON_XX, ACQ164_CLKCON_32768);
	return _set_ob_clock(hz/1000);
}
void acq200_setIntClkHz( int hz )
{
	if ( hz == 0 ){
		signalCommit(CAPDEF->ext_clk);
	}else if (DG->use_ob_clock && set_ob_clock(hz) == 0){
		;
	}else{
		err("Failed to set OB clock");
	}
}
static int fifo_read_init_action(void);


/*
 * WORKTODO

 */

#if DMA_BLOCK_LEN == 4096
#define ACQ164_FIFO_NBLOCKS(fifcon) (1)
#else
/*
 * fifcon 0..31 => 8K
 * we right shift one st 0..15 => 8K (only 4 bits needed).
 * => need 2 entries per K
 * roundup to be sure
 */

#define HOTPOINT(fifstat) ((fifstat)&ACQ164_FIFSTAT_HOTPOINT)
#define NEWPOINT(point) ((point)>>1)
#define ACQ164_FIFO_NBLOCKS(fifstat)  ((NEWPOINT(HOTPOINT(fifstat))-1)>>1)
#endif

/*
 * limit to 3 blocks because that is all the isr s/w can handle
 * NB: 2 blocks for FIQ
 */

#define GET_FIFO_NBLOCKS(nblocks, fifstat)		\
	do {						\
		nblocks = ACQ164_FIFO_NBLOCKS(fifstat); \
		nblocks = min(3, nblocks);		\
	} while(0)


static int acq200_fpga_fifo_read_open (struct inode *inode, struct file *file);
static ssize_t acq200_fpga_fifo_read_buf_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	);


static int __rgm;

int acq132_getRGM(void)
{
	return __rgm;
}

void acq132_setRGM(int enable)
{
	__rgm = enable != 0;
}


static void acq164_mach_on_set_mode(struct CAPDEF* capdef)
{
	if (acq164_isRGM()){
		DMC_WO->looking_for_pit = NOLOOK_FOR_PIT;
	}
}

static struct DevGlobs acq164_dg = {
	.btype = BTYPE_ACQ164,
	.hitide = 2,
	.max_alloc = 10240,
	.busywait = 0,
	.sample_read_start = 0,
	.sample_read_stride = 1,
	.bigbuf.tblocks.blt = blt_memcpy,
	.enable_from_eoc_isr = 1,
	.bh_unmasks_eoc = 0,		   /* TODO: appears to have no effect */
	.use_ob_clock = 1,
	.is_oneshot = 1,
	.use_fiq = 1,
	.pit_store.max_pits = 2000,
#ifdef FIQDEBUG
	.CAFEBABE = 0xcafebabe,
	.FEEDCODE = 0xfeedc0de,
	.DEADBEEF = 0xdeadbeef,
#endif
	.FIFERR = 0x00000060,       /* HOT OVER+UNDER */

	.pulse.pulse_count = 0, 
	.pulse.ibit = 1,
	.pulse.active_high = 0,
	.pulse.start_delay = 20,   /* msec */
	
	.empty_fill_threshold = 4096,
	.put_max_empties = 1024,
	.get_max_active = 1024,
	.active_batch_threshold = 64,
	.init_endstops = 32,
 	.eoc_int_modulo_mask = 0xf,      
	.activate_event_on_arm = 1,

	.dcb.dcb_max = 2048,
	.dcb.dcb_max_backlog = 2048
};


#define MYDG &acq164_dg



#define ACQ200_CUSTOM_FPGA_OPEN
#include "acq200-fifo.c"


#if (ISR_ADDS_ENDSTOP == 0)
#error ACQ164 needs ISR_ADDS_ENDSTOP TRUE
#endif

static void dmc_handle_empties_acq164(struct DMC_WORK_ORDER *wo)
/* acq164 needs to dblock data on the fly
 * @todo: first assume 4 blocks. Then check the scanlist, may be less ..
 */
{
	/* attempt to map all or remainder of wo into empties */
	int dc = DMA_DCR_FROMDEVICE;
	int nput = 0;
	unsigned empty_offset;

	while( !RB_IS_FULL(IPC->empties) &&
		(empty_offset = wo->getNextEmpty(wo)) != ~1){

		struct iop321_dma_desc *dmad = acq200_dmad_alloc();
		u32 local_pa = wo->pa + empty_offset;

		if (!dmad){
			err("acq200_dmad_alloc() STARVE");
			return;
		}

		dmad->PDA = DG->fpga.fifo.pa;
		dmad->DD_FIFSTAT = 0;
		dmad->LAD  = local_pa;
		dmad->BC = DMA_BLOCK_LEN;
		dmad->DC = dc;
		dmad->NDA = 0;

		if (nput < 3 || acq200_debug > 5){
			dbg( 3, "rb_put %s", dmad_diag( dmad ) );
		}

		if (RB_IS_EMPTY(IPC->empties)){
			dbg(1, "first DMAD: %s", dmad_diag(dmad));
		}

		rb_put( &IPC->empties, dmad );

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

static int clock_check(void)
{
	u32 clkcon = *ACQ164_CLKCON;
	u32 clkcon2;

	if ((clkcon&ACQ164_CLKCON_SCLK_LOCK) == 0){
		*ACQ164_CLKCON = clkcon | ACQ164_CLKCON_SCLK_RESET;
		msleep(2);
		*ACQ164_CLKCON = clkcon & ~ACQ164_CLKCON_SCLK_RESET;
		clkcon2 = *ACQ164_CLKCON;
		
		err("NO CLOCK LOCK, we gave it a reset was:%08x now:%08x %s",
		    clkcon, clkcon2,
		    (clkcon2&ACQ164_CLKCON_SCLK_LOCK) != 0? 
					"OK": "still no lock");
		return 1;
	}

	return 0;
}


int acq200_custom_fpga_open (struct inode *inode, struct file *file)
{
	int iminor = MINOR(file->f_dentry->d_inode->i_rdev);
	int rc = 0;

	switch(iminor){
#ifdef PGMCOMOUT
	case ACQ132_GATE_PULSE_LOAD:
		file->f_op = &acq132_gate_pulse_ops;
		break;
#endif
	default:
		return acq200_fpga_open(inode, file);
	}

	if ( file->f_op->open ){
		rc = file->f_op->open( inode, file );
	}

	if ( rc == 0 ){
		DG->open_count++;
	}

	return rc;
}
static void enable_acq164_start(void)
{
	int rc;

	dbg(3, "OK: let's trigger FIFCON: 0x%08x SYSCON: 0x%08x", 
	    *FIFSTAT, *SYSCON);

	DBGSF("TEMPORARY: blip enable");
	disable_acq();

	clock_check();
	
	if (acq164_trigger_detect()){
		err("WOAAH - triggered already, and not yet enabled not good\n"
		    "FIFCON: 0x%08x\n"
		    "FIFSTAT:0x%08x\n"
		    "SYSCON: 0x%08x",
		    *FIFCON, *FIFSTAT, *SYSCON);
		finish_with_engines(-__LINE__);
	}
	DBGSF("set ACQEN");
	enable_acq();

	if (acq164_trigger_detect()){
		err("WOAAH - triggered already, not good\n"
		    "FIFCON: 0x%08x\n"
		    "FIFSTAT:0x%08x\n"
		    "SYSCON: 0x%08x",
		    *FIFCON, *FIFSTAT, *SYSCON);
		finish_with_engines(-__LINE__);
	}

	DMC_WO->trigger_detect = acq164_trigger_detect;

	DBGSF("set events");
	signalCommit(CAPDEF->ev[0]);


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
		acq164_syscon_set(ACQ164_SYSCON_SOFTTRIG);
		DBGSF("clr SOFTTRIG");
		acq164_syscon_clr(ACQ164_SYSCON_SOFTTRIG);
		DBGSF("should be triggered");
		nwait = 0;

		do {
			if (acq164_trigger_detect()){
				return 1;
			}

			/**
			 *  some FPGA's don't support TRIGGERED - fallback:
			 */
			if ((*ACQ164_FIFSTAT&ACQ164_FIFSTAT_HOT_NE) != 0){
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

	while(!acq164_trigger_detect()){
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
	    "BEFORE", *FIFSTAT, acq164_getSyscon());

	*ACQ200_ICR = 0;

	reg_set_field(ACQ164_FIFCON, 0, ACQ164_FIFCON_HOT_HITIDE, HITIDE);

	dbg(1, "%10s reset: F:0x%08x S:0x%08x", 
	    "AFTER", *FIFSTAT, acq164_getSyscon());


/*
 * pgm 20040125: keep soft trigger high (then it cannot be construed as
 * a falling edge on DAQEN?
 */
	dbg(DISABLE_ACQ_DEBUG, "ACQ164_SYSCON_SOFTTRIG|ACQ164_SYSCON_ACQEN");
	acq164_syscon_set(ACQ164_SYSCON_SOFTTRIG|ACQ164_SYSCON_ACQEN);

	dbg(1, "%10s reset: F:0x%08x S:0x%08x", 
	    "ENABLED", *ACQ164_FIFCON, acq164_getSyscon());

	if (DG->activate_event_on_arm){
		struct Phase* phase = DMC_WO->now;

		if (phase->ev){
			activateSignal(phase->ev);
		}
	}
	enable_acq164_start();
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
	DG->fpga.fifo.pa = ACQ200_FPGA_P+ACQ164_FIFO_OFFSET;
	DG->fpga.fifo.len = 0x400;
	DG->fpga.fifo.va = (void*)(ACQ200_FPGA+ACQ164_FIFO_OFFSET);
}


static const char* ACQ164_CLKDI3_MENU[] = {
	"internal", "DI0", "DI1", "DI2", 0
};
static const char* ACQ164_CLKDO3_MENU[] = {
	"none", "DO0", "DO1", "DO2", 0
};
static const char *ACQ164_DO345_MENU[] = {
	"none", "DO3", "DO4", "DO5", 0
};
static const char *ACQ164_DI345_MENU[] = {
	"none", "DI3", "DI4", "DI5", 0
};

static int acq164_commitClkXX(
		struct Signal* signal, int shl)
{
	u32 clkcon = *ACQ164_CLKCON;
	
	clkcon &= ~(ACQ164_CLKCON_XX << shl);

	if (signal->is_active){
		u32 linecode = signal->px[SIG_LINE].ix;
		clkcon |= linecode << shl;
		if (signal->px[SIG_EDGE].ix){
			clkcon |= ACQ164_CLKCON_RISING << shl;
		}
	}
	*ACQ164_CLKCON = clkcon;
	return 0;
}
static int acq164_commitIntClkSrc(struct Signal* signal)
{
	return acq164_commitClkXX(signal, ACQ164_CLKCON_CS_SHIFT);
}

static int acq164_commitMasClk(struct Signal* signal)
{
	return acq164_commitClkXX(signal, ACQ164_CLKCON_OCS_SHIFT);
}
static int acq164_commitIndexSrc(struct Signal* signal)
{
	return acq164_commitClkXX(signal, ACQ164_CLKCON_IND_SHIFT);
}
static int acq164_commitIndexMas(struct Signal* signal)
{
	return acq164_commitClkXX(signal, ACQ164_CLKCON_OIND_SHIFT);
}


static int _acq164_commitEvX(
	struct Signal* signal, 
	volatile u32* reg,
	int shift)
{
	u32 syscon = *reg;
	
	syscon &= ~(ACQ100_SYSCON_EV_MASK << shift);
	
	if (signal->is_active){
		u32 rising = signal->px[SIG_EDGE].ix? 
			ACQ100_SYSCON_EV_RISING: ACQ100_SYSCON_EV_FALLING;
		u32 linecode = signal->px[SIG_LINE].ix;

		syscon |= ((rising | linecode) << shift);
	}	

	*reg = syscon;	
	return 0;
}



static inline int acq164_commitEv0(struct Signal* signal)
{
	return _acq164_commitEvX(
		signal, ACQ164_SYSCON, ACQ164_SYSCON_EV0_SHIFT);
}
static inline int acq164_commitEv1(struct Signal* signal)
{
	return _acq164_commitEvX(
		signal, ACQ164_SYSCON, ACQ164_SYSCON_EV1_SHIFT);
}
static inline int acq164_commitTrg(struct Signal* signal)
{
	return _acq164_commitEvX(
		signal, ACQ164_SYSCON, ACQ164_SYSCON_TRG_SHIFT);
}

static int _acq164_commitTrigXXX(
	struct Signal* signal,
	volatile u32* reg,
	int shift)
{
	u32 syscon = *reg;

	syscon &= ~(0x3 << shift);

	if (signal->is_active){
		u32 linecode = signal->px[SIG_LINE].ix;
		syscon |= linecode << shift;
	}

	*reg = syscon;
	return 0;
}
static inline int acq164_commitSyncTrigMas(struct Signal *signal)
{
	return _acq164_commitTrigXXX(
		signal, ACQ164_SYSCON, ACQ164_SYSCON_OTR_SHIFT);
}

static inline int acq164_commitSyncTrigSrc(struct Signal *signal)
{
	return _acq164_commitTrigXXX(
		signal, ACQ164_SYSCON, ACQ164_SYSCON_ITR_SHIFT);
}
static int acq164_commitAOTrg(struct Signal* signal)
{
	return _acq164_commitEvX(
		signal, ACQ164_SYSCONDAC, ACQ164_SYSCON_TRG_SHIFT);
}

static int acq164_commitClkCounterSrc(struct Signal* signal)
{
	u32 clk_counter = *ACQ164_CLK_COUNTER;
	u32 linecode = signal->px[SIG_LINE].ix;

	clk_counter &= ~ACQ100_CLK_COUNTER_SRCMASK;

	if (linecode >=0 && linecode <= 7){
	       clk_counter |= linecode << ACQ100_CLK_COUNTER_SRCSHL;
	       clk_counter |= ACQ100_CLK_COUNTER_SRC_DIO;
	       acq200_start_clkCounterMonitor();		
	}

	*ACQ164_CLK_COUNTER = clk_counter;

	if (linecode == 9){
		acq200_stop_clkCounterMonitor();		
	}else if (acq200_clkCounterMonitor_requestedToStop()){
		acq200_start_clkCounterMonitor();
	}
	return 0;
}


static int acq164_commitAdcModeChoice(struct Signal* signal)
{
	if (signal->px[0].masks == 0){
		err("NO MASKS");
		return -1;
	}else{
		u32 syscon = *ACQ164_SYSCON;
		u32 mask = signal->px[0].masks[signal->px[0].ix];

		syscon &= ~ACQ164_SYSCON_CDM;
		syscon |=  mask << ACQ164_SYSCON_CDM_SHIFT;	
		*ACQ164_SYSCON = syscon;
		return 0;
	}
}

int acq164_getDecimation(void)
{
	switch(CAPDEF->adc_mode->px[0].ix >> ACQ164_SYSCON_CDM_SHIFT){
	case ACQ164_SYSCON_CDM_HISPEED_256:
	case ACQ164_SYSCON_CDM_LP_256:
		return 256;
	default:
		return 512;
	}
}

static struct Signal *createSignalAdcMode(void)
{
	static const char *ACQ164_ADC_MODE_CHOICES[] = {
		"HISPEED_256", "HIRES_512", "LP_256", "LP_512", "LS_512", 0 
	};
	static const unsigned ACQ164_ADC_MODE_MASKS[] = {
		ACQ164_SYSCON_CDM_HISPEED_256,	ACQ164_SYSCON_CDM_HIRES_512,
		ACQ164_SYSCON_CDM_LP_256,	ACQ164_SYSCON_CDM_LP_512,
		ACQ164_SYSCON_CDM_LS_512 
		/* terminator not required */
	};

	struct Signal *sig = createSignal(
		"adc_mode", 
		ACQ164_ADC_MODE_CHOICES, 0, 0, 0, 
		acq164_commitAdcModeChoice);

	sig->px[0].masks = ACQ164_ADC_MODE_MASKS;
	return sig;
}

static struct CAPDEF* acq164_createCapdef(void)
{
	static struct CAPDEF _capdef = {
		.demand_len = 0x100000,
		.channel_mask = 0xffff,       
		.mode = M_SOFT_TRANSIENT,
		.pit_stop = 1
	};
	struct CAPDEF* capdef = 
			acq2xx_createCapdef(&_capdef, ACQ164_SAMPLE_WORD_SIZE);
	
	capdef->ev[0] = createSignal(
		"event0", SIG(EVS), 3, SIG(EDG), 0, acq164_commitEv0);
	capdef->ev[1] =  createSignal(
		"event1", SIG(EVS), 3, SIG(EDG), 1, acq164_commitEv1);
	capdef->trig  = createSignal(
		"trig",   SIG(EVS), 3, SIG(EDG), 0, acq164_commitTrg);

	capdef->ao_trig = createSignal(
		"ao_trig", SIG(EVS), 3, SIG(EDG), 0, acq164_commitAOTrg);

	capdef->sync_trig_src = createSignal(
		"sync_trig_src", ACQ164_DI345_MENU, 0, 0, 0,
		acq164_commitSyncTrigSrc);
	capdef->sync_trig_mas = createSignal(
		"sync_trig_mas", ACQ164_DO345_MENU, 0, 0, 0,
		acq164_commitSyncTrigMas);
	/* @todo - ext clk is int_clk_src ? */
	capdef->ext_clk = createSignal(
		"ext_clk", ACQ164_CLKDI3_MENU, 0, SIG(EDG), 0,
		acq164_commitIntClkSrc);
	capdef->int_clk_src = createSignal(
		"int_clk_src", ACQ164_CLKDI3_MENU, 0, SIG(EDG), 0,
		acq164_commitIntClkSrc);

	capdef->mas_clk = createSignal(
		"mas_clk", ACQ164_CLKDO3_MENU, 0, SIG(EDG), 0, 
		acq164_commitMasClk);

	capdef->clk_counter_src = createSignal(
		"clk_counter_src", SIG(CKS07), 8, SIG(EDG), 0, 
		acq164_commitClkCounterSrc);

	capdef->index_src = createSignal(
		"index_src", ACQ164_CLKDI3_MENU, 0, SIG(EDG), 0,
		acq164_commitIndexSrc);
	capdef->index_mas = createSignal(
		"index_mas", ACQ164_CLKDO3_MENU, 0, SIG(EDG), 0,
		acq164_commitIndexMas);

	capdef->adc_mode = createSignalAdcMode();
	return capdef;
}

static void acq164_destroyCapdef(struct CAPDEF *capdef)
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
	destroySignal(capdef->gate_src);
	kfree(capdef);
}
static struct device_driver acq164_fpga_driver;

extern int init_arbiter(void);

void acq132_set_obclock(int FDW, int RDW, int R, int Sx)
{
	u32 ics527 = 0;
	ics527 |= to_mask(ACQ100_ICS527_FDW, FDW);
	ics527 |= to_mask(ACQ100_ICS527_RDW, RDW);
#ifdef BADPORT
	ics527 |= to_mask(ACQ100_ICS527_CLKDIV, R);
#endif
	ics527 |= to_mask(ACQ100_ICS527_S1S0, Sx);

	*ACQ164_ICS527 = ics527;

	acq200_clk_hz = ob_clock_def.actual * 1000;
}





static void acq164_set_defaults(void)
{
	info("setIntClkHz %d", ACQ164_BEST_CLK);
	acq200_setIntClkHz(ACQ164_BEST_CLK);
	*ACQ164_SYSCON |= ACQ164_SYSCON_CLKDIV;
}


static unsigned acq164_getClkCounter(void)
{
	u32 clk_counter = *ACQ164_CLK_COUNTER;

	if (check_clock_lock){
		u32 clk_con = *ACQ164_CLKCON;
		if ((clk_con&ACQ164_CLKCON_SCLK_LOCK) == 0){
			*ACQ164_CLKCON = clk_con ^= ACQ164_CLKCON_SCLK_RESET;
			dbg(2, "clk_con %08x clk_counter %08x", clk_con, clk_counter);
			/* pull reset down next tick */
		}
	}
	return (clk_counter) & ACQ100_CLK_COUNTER_COUNT;
}

static struct CLKCOUNTER_DESCR clk_probe = {
	.getCount = acq164_getClkCounter,
	.prescale = ACQ100_CLK_COUNTER_PRESCALE,
	.rollover = ACQ100_CLK_COUNTER_COUNT+1
};


static int acq164_fpga_probe(struct device *dev)
{
	init_pbi(dev);
	init_arbiter();

	/* two reads allows for wedged PBI bus on jtag load */
	if (*ACQ164_BDR == ACQ100_BDR_MAGIC || 
	    *ACQ164_BDR == ACQ100_BDR_MAGIC    ){
		int rc = acqX00_fpga_probe(dev, IRQ_ACQ100_FPGA);
		if (rc != 0){
			err("fpga_probe() failed");
		}else{
			mk_sysfs(&acq164_fpga_driver);
			acq164_set_defaults();
			
			acq200_init_clkCounterMonitor(&clk_probe);
			acq200_start_clkCounterMonitor();
		}
		return rc;
	}else{
		err("DEVICE NOT FOUND 0x%p=0x%08x", ACQ164_BDR, *ACQ164_BDR);
		return -ENODEV;
	}
}

static int acq164_fpga_remove(struct device *dev)
{
	acqX00_fpga_remove(dev, IRQ_ACQ100_FPGA);
	return 0;
}


static void acq164_fpga_dev_release(struct device * dev)
{
	info("");
}



static struct device_driver acq164_fpga_driver = {
	.name     = "acq164fpga",
	.probe    = acq164_fpga_probe,
	.remove   = acq164_fpga_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq164_fpga_device = {
	.name = "acq164fpga",
	.id   = 0,
	.dev = {
		.release    = acq164_fpga_dev_release,
		.dma_mask   = &dma_mask
	}

};

#define LINDEB dbg(1, "%d", __LINE__)

static int __init acq164_fifo_init( void )
{
	int rc;
	acq200_debug = acq200_fifo_debug;

	dbg(1, "acq200_debug set %d\n", acq200_debug );
	info(ACQ164_VERID);

	IPC = kmalloc(sizeof(struct IPC), GFP_KERNEL);
	memset(IPC, 0, sizeof(struct IPC));

	LINDEB;
	CAPDEF = acq164_createCapdef();
	init_dg();
	DG->bigbuf.tblocks.getChannelData = getChannelData32;
	acq200_setChannelMask(0x3);
	LINDEB;
	DG->bigbuf.tblocks.transform = acq200_getTransformer(2)->transform;
	DMC_WO->handleEmpties = dmc_handle_empties_acq164;
	LINDEB;
	init_phases();
	LINDEB;
/* extra kobject init needed to hook driver sysfs WHY? */

	kobject_init(&acq164_fpga_driver.kobj);
	kobject_set_name(&acq164_fpga_driver.kobj, acq164_fpga_driver.name);

	if ((rc = kobject_register(&acq164_fpga_driver.kobj)) != 0){
		err("kobject_register() failed");
	}else if ((rc = driver_register(&acq164_fpga_driver)) != 0){
		err("driver_register() failed");
	}else if ((rc = platform_device_register(&acq164_fpga_device)) !=0){
		err("platform_device_register() failed");
	}else if ((rc = acq196_AO_fs_create(&acq164_fpga_device.dev)) != 0){
		err("failed to register AO fs");
	}else{
		info("all done");
	}

	
	return rc;
}




static void __exit
acq164_fifo_exit_module(void)
// Remove DRIVER resources on module exit
{
//	unregister_reboot_notifier(&e1000_notifier_reboot);


	acq196_AO_fs_remove();
	dbg(1, "rm_sysfs()" );
	rm_sysfs(&acq164_fpga_driver);
	dbg(1, "platform_device_unregister()");
	platform_device_unregister(&acq164_fpga_device);
	dbg(1, "pci_unregister_driver()" );
	driver_unregister(&acq164_fpga_driver);
	dbg(1, "kobject-unregister()");
	kobject_unregister(&acq164_fpga_driver.kobj);


	delete_dg();
	acq164_destroyCapdef(CAPDEF);
	kfree(IPC);
}


void acq164_set_obclock(int FDW, int RDW, int R, int Sx)
{
	u32 clk_con = *ACQ164_CLKCON;
	u32 ics527 = 0;
	*ACQ164_CLKCON = clk_con |= ACQ164_CLKCON_SCLK_RESET;
	ics527 |= to_mask(ACQ100_ICS527_FDW, FDW);
	ics527 |= to_mask(ACQ100_ICS527_RDW, RDW);
//	ics527 |= to_mask(ACQ132_ICS527_CLKDIV, R);
	ics527 |= to_mask(ACQ100_ICS527_S1S0, Sx);

	*ACQ164_ICS527 = ics527;

	acq200_clk_hz = ob_clock_def.actual * 1000;

	udelay(ics527_reset_usec);
	*ACQ164_CLKCON = clk_con &= ~ACQ164_CLKCON_SCLK_RESET;
}


static void acq164_event_adjust(
	struct Phase *phase, unsigned isearch, 
	unsigned* first, unsigned* last)
{
	int doff = ads1278_group_delay * sample_size();
	int tboff = TBLOCK_OFFSET(*first);
	int tbresidue = TBLOCK_LEN(DG) - TBLOCK_OFFSET(*last);
	int tbix = TBLOCK_INDEX(*first);

	if (likely(tboff+doff < TBLOCK_LEN(DG))){
		/* remove the ES and compensate */
		memcpy(va_buf(DG)+*first, va_buf(DG)+*last, tbresidue);

		*first += doff;
		*last = *first;
		/* @@todo and adjust tblock length */
		DG->bigbuf.tblocks.the_tblocks[tbix].tb_length -= 
								sample_size();
	}else{
		info("Warning shift forwards out of tblock");
	}
}




module_init(acq164_fifo_init);
module_exit(acq164_fifo_exit_module);

EXPORT_SYMBOL_GPL(acq200_setIntClkHz);
EXPORT_SYMBOL_GPL(acq200_setChannelMask);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ164 FIFO");

