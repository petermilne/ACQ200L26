/* ------------------------------------------------------------------------- */
/* acq216-fifo.c customisation for acq216-fifo driver                        */
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

#define DTACQ_MACH 1
#define ACQ216 
#define ACQ_IS_INPUT 1

#include <linux/delay.h>

#define MODEL_VERID \
	"$Id: acq216-fifo.c,v 1.26 2006/07/28 14:46:14 pgm Exp $\n"

#define ACQ_INTEN 1
#define PCI_FPGA 1

#define FPGA_INT IRQ_ACQ200_FP
#define FPGA_INT_MASK (1<<FPGA_INT)

#define MTTR2	0xfe		/* maximse DMA block len on bus */
#include "acq200-fifo-local.h"

#define AICHAN_DEFAULT 16


#define DBGSF(s) \
        dbg(3, "S: 0x%08x FC: 0x%08x FX: 0x%08x %s", \
        *SYSCON, *ACQ200_FIFCON, *FIFSTAT, s)


/* set bits to mask chain elements: use with caution! */

#define MK_CDM(cdm) int cdm;module_param(cdm, int, 0600)

MK_CDM(cdm_capcom_to_local);
MK_CDM(cdm_local_to_host);
MK_CDM(cdm_capcom_test);
MK_CDM(cdm_capcom_host);


int cd_short_tlatch;
module_param(cd_short_tlatch, int, 0600);

int abort_acq_on_dma_error;
module_param(abort_acq_on_dma_error, int, 0600);

int stub_event_adjust;
module_param(stub_event_adjust, int, 0600);

int flood_es;
module_param(flood_es, int, 0600);

int nonaligned_es;
module_param(nonaligned_es, int, 0600);

/* the shift we had to apply to compensate non-aligned ESW */
int esw_shift;
module_param(esw_shift, int, 0400);

/* the expected ESW alignment */
int esw_expected_modulus;
module_param(esw_expected_modulus, int, 0400);

/* the actual ESW alignment */
int esw_actual_residue;
module_param(esw_actual_residue, int, 0400);

/* DCM [ext clk] monitoring valid HS only! */
int dcm_monitor;
module_param(dcm_monitor, int, 0664);

static void init_endstops( int count );   /* @@todo SHOULD BE IN HEADER */


static struct CAPDEF* acq216_createCapdef(void);
static void acq216_destroyCapdef(struct CAPDEF *capdef);

static void acq216_driverInit(struct device* dev);
static void acq216_driverRemove(struct device* dev);
static void acq216_event_adjust(
	struct Phase *phase, unsigned isearch, 
	unsigned* first, unsigned* last);



#define DTACQ_MACH_CREATE_CAPDEF acq216_createCapdef
#define DTACQ_MACH_DESTROY_CAPDEF acq216_destroyCapdef
#define DTACQ_MACH_DRIVER_INIT(dev) acq216_driverInit(dev)
#define DTACQ_MACH_DRIVER_REMOVE(dev) acq216_driverRemove(dev)

#undef DTACQ_MACH_EVENT_ADJUST
#define DTACQ_MACH_EVENT_ADJUST(phase, isearch, first, last) \
	if (!stub_event_adjust){ \
		dbg(1, "DTACQ_MACH_EVENT_ADJUST: isearch 0x%08x", isearch); \
	        acq216_event_adjust(phase, isearch, first, last); \
	}


static int enable_soft_trigger(void);
static int enable_hard_trigger(void);

/** @todo dodgy global indicates we have triggered -
 *   race condition for short transients */
static int has_triggered = 0;
static int transient_dma_block_count;   

#include "acq200-bridge.h"

u32 acq216_pci2bus(u32 pci_addr) {
/* convert backplane pci addr to local PCI addr */
	struct B_WINDOW downstream = {
		.w_len = 0x100000
	};
	acq200_bridge_get_windows(0, &downstream);
	
	
	pci_addr &= (downstream.w_len - 1);
	return pci_addr | ACQ200_PCIMEM;
}

EXPORT_SYMBOL_GPL(acq216_pci2bus);

static int acq216_trigger_detect(void)
{
	return (*ACQ200_SYSCON & ACQ200_SYSCON_TR) != 0;
}

#define CAPCOM_LENGTH (ACQ216_BLOCKID_OFFSET+8-ACQ216_TCR_IMM_OFFSET)

#define CAPCOM_PDA (DG->fpga.regs.pa + ACQ216_TCR_IMM_OFFSET)    /** @todo. */

/** Prebuilt Chains - for Combined Diag and Control. */
struct PrebuiltChainStore {
	int count;
	struct PrebuiltChain* chains;
	struct pci_mapping capcom_scratchpad;
} S_pbstore;


static unsigned check_fifstat(
	struct DMC_WORK_ORDER *wo, u32 fifstat, u32* offset)
/* returns 1 if trigger handled */
/** @todo this needs refactoring with the ACQ196 version ! */
{
	int adc_ev = fifstat&ACQ200_FIFCON_EVX;

	dbg(3, "F%08x %s %s", fifstat,
	    adc_ev? "TR":"tr", wo->looking_for_pit? "LOOKING":"-");

	if (adc_ev){
		/* avoid race condition by allowing onEvent to possibly
                 * clear down the event enable (1) BEFORE ACK event (2)
                 */
		if (wo->looking_for_pit){
			onEvent(wo, fifstat, offset);         /* (1) */
		}

		*FIFCON |= ACQ200_FIFCON_EVX;                 /* (2) */

		if (wo->looking_for_pit){
			dbg(2, "EVENT %08x at 0x%08x", fifstat, *offset);
			wo->looking_for_pit = 0;
			if (fifstat&ACQ200_FIFCON_EV0){
				DG->stats.event0_count++;
			}
			if (fifstat&ACQ200_FIFCON_EV1){
				DG->stats.event1_count++;
			}
			return fifstat|DMC_EVENT_MARKER;
		}else{
			if (fifstat&ACQ200_FIFCON_EV0){
				DG->stats.event0_count2++;
			}
			if (fifstat&ACQ200_FIFCON_EV1){
				DG->stats.event1_count2++;
			}			
		}
	}else{
		/** this isn't an event, trigger, anything, just NZ FIFSTAT */
		wo->looking_for_pit = 1;
	}

	return 0;
}

#define CHECK_FIFSTAT(wo, fifstat, offset) check_fifstat(wo, fifstat, offset)

extern int iop32x_pci_bus_speed(void);

static int set_ob_clock(int hz)
/* set ICS307 clock */
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};

	static char fout_def[20];
	static char fin_def[20];
	static char *argv[6];
	int i;
	int fin = iop32x_pci_bus_speed()/4/1000; /* input clk in kHz */

	sprintf(fin_def, "%d", fin);

/** source OB clock from INT 16M, source int clock divider from OBC, 
 *  see store_ob_clock_word() 
 */
	CAPDEF->ob_clk_src->DIx = ACQ200_CLKCON_DLL_CS_16M;

	sprintf(fout_def, "%d", hz/1000);

        i = 0;
        argv[i++] = "/usr/local/bin/ob_calc";
	argv[i++] = "--fin";
	argv[i++] = fin_def;	
        argv[i++] = fout_def;
	argv[i++] = "/dev/dtacq/ob_clock_word";
        argv[i] = 0;

	dbg( 1, "call_usermodehelper %s\n", argv[0] );

	i = call_usermodehelper(argv [0], argv, envp, 0);

        dbg( 1, "call done returned %d", i );
	return i;
}


static void _setExtClk(void)
{
	*ACQ200_CLKCON |= ACQ200_CLKCON_EXTCLK;
	signalCommit(CAPDEF->ext_clk);
}

static void _setIntClkHz(int hz, long masterclk, u32 clksel)
{
#define MAXDIV    0x0000fffe
	u32 clkdiv;


	if ( hz > masterclk/2 ) hz = masterclk/2;

	clkdiv = (masterclk / hz);

	if ( clkdiv > MAXDIV ) clkdiv = MAXDIV;
	if ( clkdiv < 2 )      clkdiv = 2;

	*ACQ200_CLKCON &= ~(ACQ200_CLKCON_CS_MASK|ACQ200_CLKCON_EXTCLK);
	*ACQ200_CLKCON |= clksel;
	*ACQ200_CLKDAT = clkdiv;

	acq200_clk_hz = masterclk / clkdiv;
	acq200_clk_hz = acq200_rounding(acq200_clk_hz, PRECISION(clkdiv));

	dbg( 1, "hz:%7d clkdiv 0x%08x *ACQ200_CLKDAT 0x%08x hz act %d\n",
	     hz, clkdiv, *ACQ200_CLKDAT, acq200_clk_hz );
}

#define MASTERCLK_66 66666666  
#define MASTERCLK_80 80000000


#define MASTERCLK MASTERCLK_80


static struct IntClkConsts {
	long masterclk;
	u32  clksel;
} intclk[2] = {
	{
		.masterclk = MASTERCLK_66,
		.clksel    = ACQ200_CLKCON_CS_66M
	},
};
#define MAXSEL 1



void ___setIntClkHz( int hz )
{

	int actual[2];
	int isel;
	u32 clkdiv;
	int deltamin = MASTERCLK, imin = 0;
	int delta;
	int maxsel = MAXSEL;

	if ( hz == 0 ){
		_setExtClk();
	}else{
		if ( hz < 1 ){
			hz = 1;
		}else if ( hz > MASTERCLK/2 ){
			hz = MASTERCLK/2;
		}
/*
 * WORKTDO - actually should bias to nearest clk BELOW
 * see acq32 firmware for code
 */
		for (isel = 0; isel != maxsel; ++isel){
			clkdiv = intclk[isel].masterclk/hz;
			actual[isel] = intclk[isel].masterclk/clkdiv;
			delta = abs(hz - actual[isel]);
			if (delta < deltamin ){
				deltamin = delta;
				imin = isel;
			}
		}
	
		_setIntClkHz(hz, intclk[imin].masterclk, intclk[imin].clksel);
	}
}




void acq200_setIntClkHz(int hz)
{
	if ( hz == 0 ){
		_setExtClk();
	}else if (DG->use_ob_clock && set_ob_clock(hz) == 0){
		;
	}else{
		___setIntClkHz(hz);
	}
}

/*
 * ACQ216 ONLY
 */
static int fifo_read_init_action(void);
static int acq200_fpga_fifo_read_open (struct inode *inode, struct file *file);

#ifdef PGMCOMOUT
static ssize_t acq200_fpga_fifo_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
);
#endif
static ssize_t acq200_fpga_fifo_read_buf_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
);

#define ACQ216_FIFO_NBLOCKS(fifcon) \
        ((HOT_FIFO_HALF(fifcon) != 0) + \
         (HOT_FIFO_FULL_ENTRIES(fifcon)==15) + \
         (COLD_FIFO_HALF(fifcon) != 0))


#define GET_FIFO_NBLOCKS(nblocks, fifcon) \
do { \
        nblocks = ACQ216_FIFO_NBLOCKS(fifcon); \
} while(0)


#define SET_SIMULATION_MODE(enable)					\
        do {								\
	        if (enable){						\
		        *ACQ200_SYSCON |= ACQ200_SYSCON_SIM_MODE;	\
	        }else{							\
		        *ACQ200_SYSCON &=~ ACQ200_SYSCON_SIM_MODE;	\
	        }							\
        } while(0)	


int acq200_lookup_pchan(int lchannel);






static struct DevGlobs acq216_dg = {
	.btype = BTYPE_ACQ216,
	.hitide = 1,
	.max_alloc = 10240,
	.busywait = 0,
	.sample_read_start = 0,
	.sample_read_stride = 1,
	.bigbuf.tblocks.blocklen = TBLOCK_LEN,
	.bigbuf.tblocks.blt = blt_memcpy,

	.enable_from_eoc_isr = 1,
	.bh_unmasks_eoc = 1,
	.use_ob_clock = 1,
	.is_oneshot = 1,
	.use_fiq = 1,
	.pit_store.max_pits = 10,
	.FIFERR = ACQ200_FIFCON_COLDOVER|ACQ200_FIFCON_COLDUNDER|
	          ACQ200_FIFCON_HOTOVER|ACQ200_FIFCON_HOTUNDER,
#ifdef FIQDEBUG
	.CAFEBABE = 0xcafebabe,
	.FEEDCODE = 0xfeedc0de,
	.DEADBEEF = 0xdeadbeef,
#endif
	.empty_fill_threshold = 4096,
	.put_max_empties = 1024,
	.get_max_active = 1024,
	.active_batch_threshold = 64,
	.init_endstops = 32,
	.eoc_int_modulo_mask = 3,
	.activate_event_on_arm = 1,

	.dcb.dcb_max = 1024,
	.dcb.dcb_max_backlog = 1024,
};

#define MYDG &acq216_dg

#include "acq200-fifo.c"


static void enable_acq216_start(void)
{
	int rc;

	DMC_WO->trigger_detect = acq216_trigger_detect;
	dbg(3, "OK: let's trigger FIFCON: 0x%08x SYSCON: 0x%08x", 
	    *ACQ200_FIFCON, *ACQ200_SYSCON);

	dbg(3, "FINAL:next enable FIFCON" );
	*ACQ200_FIFCON |= ACQ200_FIFCON_HC_ENABLE;

	preEnable();

	dbg(1,"use hard trigger if enabled %d", CAPDEF->trig->is_active);

	has_triggered = 0;
	transient_dma_block_count = 0;

	onEnable();	

	if (CAPDEF->trig->is_active){
		rc = enable_hard_trigger();
	}else{
		rc = enable_soft_trigger();
	}

	if (DMC_WO->trigger_detect()){
		onTrigger();
	}
}



void acq216_stop_capture(void)
{
	disable_fifo();

	if (dcm_monitor){
		unsigned lock = NCHAN==12?
 			ACQ200_CLKCON_DCMx3_LOCK: ACQ200_CLKCON_DCMx4_LOCK;
		unsigned clkfx = NCHAN==12?
			ACQ200_CLKCON_DCMx3_CLKFX: ACQ200_CLKCON_DCMx4_CLKFX;
		unsigned clkin = NCHAN==12?			
			ACQ200_CLKCON_DCMx3_CLKIN: ACQ200_CLKCON_DCMx4_CLKIN;

		unsigned clkcon = *ACQ200_CLKCON;

		if ((clkcon&lock) == 0 || (clkcon&clkfx) != 0){
			char emsg[80];
			sprintf(emsg, 
				"DCM LOCK ERROR 0x%08x LOCK:%s IN:%s OUT:%s",
				clkcon, 
				(clkcon&lock) ==1? "OK": "FAIL",
				(clkcon&clkin)==1? "OK": "FAIL",
				(clkcon&clkfx)==1? "OK": "FAIL");
			err("%s", emsg);
			strcat(errbuf, emsg);
			DMC_WO->error = errbuf;
		}else{
			dbg(1, "LOCK: OK");
		}
	}
}



int acq216_setAntiPhase(int enable)  
/** ACQ216HS, sets channelMask too. */
{
	if (enable){
		*ACQ200_SYSCON |= ACQ200_SYSCON_ANTIPHASE;
		acq200_setChannelMask(0x000f);		
	}else{
/** restores best guess channel mask - at least it's valid ... */
		*ACQ200_SYSCON &= ~ACQ200_SYSCON_ANTIPHASE;
		acq200_setChannelMask(0x000f);
	}
	return 0;
}


static void acq216_event_adjust(
	struct Phase *phase,
	unsigned isearch, 
	unsigned* first,       /* byte size index pts to first byte in ES */
	unsigned* last)        /* byte size index pts to last  byte in ES */
/**
 *  extract cold count. Then attempt to wind isearch forward
 *  approx 1:3000 times, this won't fit the current TBLOCK. That's tough.
 */
{
	unsigned* base = (unsigned*)va_buf(DG); 
	unsigned es = base[isearch];
	int ssb = sample_size();
	int ssu = ssb/sizeof(u32);
/** @@todo .. what if trigger point is in PREVIOUS TBLOCK? */
	unsigned tboffuss = TBLOCK_OFFSET(isearch*USS);
	unsigned tblock_l = (TBLOCK_INDEX(isearch*USS) * TBLOCK_LEN)/USS;
	unsigned tblock_r = tblock_l + (TBLOCK_LEN - 1)/USS;
	unsigned tailroom = tboffuss/USS;
	int cold_bytes = ACQ216_ES_COLD_QUADS(es) * 8;
	int cold_samples = cold_bytes/ssb;
	int cold_pairs = 0;
	unsigned es_len = *last - *first;
	int align_pairs = 0;	/* compensate nonaligned ES */

	static char ES_BUF[ES_SIZE_MAX];

	if (nonaligned_es){
		align_pairs = isearch % ES_LONGS;
	        dbg(1, "nonaligned_es test shift longs: %d", align_pairs);
		esw_shift = align_pairs * sizeof(int);
		esw_expected_modulus = NCHAN==12? 
			10 * sample_size(): 4 * sample_size();
		esw_actual_residue = esw_shift % esw_expected_modulus;
	}
	if (cold_samples*ssb < cold_bytes){
		cold_samples++;
	}
	
/* cold_samples is now a negative adjust: */
	cold_samples = -cold_samples;

	dbg(1, "es 0x%08x COLD_QUADS %d "
	    "cold_samples %d pipeline_offset %d adj samples %d",
	    es, ACQ216_ES_COLD_QUADS(es), 
	    cold_samples, CAPDEF_get_pipeline_offset(),
	    cold_samples + CAPDEF_get_pipeline_offset());

	cold_samples += CAPDEF_get_pipeline_offset();
	cold_pairs = cold_samples*ssu - align_pairs;

	dbg(1, "cold_samples %d cold_pairs %d bytes %d",
	    cold_samples, cold_pairs, cold_pairs*4);

	if (cold_samples < 0){
		dbg(1, "tailroom %d required %d %s es_len %d bytes",
		    tailroom*USS, cold_pairs*USS,
		    tailroom >= -cold_pairs? "OK": "DEEP GRIEF",
		    es_len);
	}

	if (DG->show_event > 1){
		dbg(1, "show_event > 1 drop out without applying comp");
		return;
	}else if (cold_pairs != 0){
		unsigned iinsert;

		dbg(1, "first %d last %d", *first, *last);

		assert(es_len <= sizeof(ES_BUF));
		memcpy(ES_BUF, &base[isearch], es_len);

		if (likely(cold_pairs < 0)){
			/*    iinsert ..... isearch 
                         *    shuffle range right to make room for ES
                         */
			if (likely(tailroom >= -cold_pairs)){
				iinsert = isearch + cold_pairs;
			}else{
				iinsert = tblock_l;
				err("WARNING: failed to adjust <-ES by "
				    "%d bytes settled for %d "
				    "searching from 0x%08x",
				    cold_pairs*4, 
				    (tailroom + cold_pairs)*4,
				    iinsert);
			}
			memmove(&base[iinsert + es_len/USS],
				&base[iinsert],
				(isearch - iinsert)*USS);
		}else{
			/* isearch .... iinsert 
                         * shuffle range left to make room for ES
                         */
			if (likely(isearch+cold_pairs < tblock_r)){
				iinsert = isearch + cold_pairs;
			}else{
				iinsert = tblock_r;

				err("WARNING: failed to adjust ES-> by"
				    "%d bytes settled for %d "
				    "searching from 0x%08x",
				    cold_pairs*4, 
				    (isearch+cold_pairs-tblock_r)*4,
				    iinsert);
			}
			memmove(&base[isearch],
				&base[isearch + es_len/USS],
				(iinsert - isearch)*USS);
		}

		memcpy(&base[iinsert], ES_BUF, es_len);

		iinsert *= USS;
		*first = iinsert; 
		*last =  iinsert + es_len;

		dbg(1, "first %d last %d", 
			*first, *last);
	}
}


static int _acq200_lookup_pchan(int lchannel)
{
#define RET_DEMANGLE(pcbase) return pcbase + (lchannel-1)%4
#define BRK_ERROR            break
	switch(NCHAN){
	case 16:
		if (IN_RANGE(lchannel, 1, 4)){
			RET_DEMANGLE(12);
		}else if (IN_RANGE(lchannel, 5, 8)){
			RET_DEMANGLE(0);
		}else if (IN_RANGE(lchannel, 9, 12)){
			RET_DEMANGLE(8);
		}else if (IN_RANGE(lchannel, 13, 16)){
			RET_DEMANGLE(4);
		}else{
			BRK_ERROR;
		}
	case 12:
		if (IN_RANGE(lchannel, 5, 8)){
			RET_DEMANGLE(0);
		}else if (IN_RANGE(lchannel, 1, 4)){
			RET_DEMANGLE(4);
		}else if (IN_RANGE(lchannel, 9, 12)){
			RET_DEMANGLE(8);
		}else{
			BRK_ERROR;
		}
	case 8:
		if (IN_RANGE(lchannel, 5, 8)){
			RET_DEMANGLE(0);
		}else if (IN_RANGE(lchannel, 1, 4)){
		        RET_DEMANGLE(4);
		}else{
			BRK_ERROR;
		}
	case 4:
		if (IN_RANGE(lchannel, 1, 4)){
			RET_DEMANGLE(0);
		}else{
			BRK_ERROR;
		}
	default:
		BRK_ERROR;
	}

	dbg(1, "this combination NFG nchan %d lchannel %d", NCHAN, lchannel);
	return -1;
#undef RET_DEMANGLE
#undef BRK_ERROR
}

static int _acq200_lookup_pchan_antiphase(int lchannel)
{
#define RET_DEMANGLE(pcbase) return pcbase + (lchannel-1)%4
#define BRK_ERROR            break
	switch(NCHAN){

	case 12:
		if (IN_RANGE(lchannel, 5, 8)){
			RET_DEMANGLE(0);
		}else if (IN_RANGE(lchannel, 9, 12)){
			RET_DEMANGLE(4);
		}else if (IN_RANGE(lchannel, 13, 16)){
			RET_DEMANGLE(8);
		}else{
			BRK_ERROR;
		}
	case 8:
		if (IN_RANGE(lchannel, 5, 8)){
			RET_DEMANGLE(0);
		}else if (IN_RANGE(lchannel, 9, 12)){
		        RET_DEMANGLE(4);
		}else{
			BRK_ERROR;
		}
/** the two channel case also has NCHAN 4 - channel pairing */
	case 4:
		switch(CAPDEF->channel_mask){
		case 0x000f:                    /* channel paired case */
			if (IN_RANGE(lchannel, 1, 4)){
				RET_DEMANGLE(0);
			}else{
				BRK_ERROR;
			}			
		case 0x00f0:
			if (IN_RANGE(lchannel, 5, 8)){
				RET_DEMANGLE(0);
			}else{
				BRK_ERROR;
			}
		default:
			BRK_ERROR;
		}        
	default:
		BRK_ERROR;
	}

	dbg(1, "this combination NFG nchan %d lchannel %d", NCHAN, lchannel);
	return -1;
#undef RET_DEMANGLE
#undef BRK_ERROR
}

int acq200_lookup_pchan(int lchannel)
{
	if ((*ACQ200_SYSCON & ACQ200_SYSCON_ANTIPHASE)){
		return _acq200_lookup_pchan_antiphase(lchannel);
	}else{
		return _acq200_lookup_pchan(lchannel);
	}
}

static int enable_soft_trigger(void)
{
#define FIFO_EMPTY (ACQ200_FIFCON_COLDEMPTY|ACQ200_FIFCON_HOTEMPTY)

	for(soft_trigger_retry = 0; 
	    soft_trigger_retry < MAXRETRY;
	    ++soft_trigger_retry) {
		*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN|ACQ200_SYSCON_SOFTTRG;

		msleep(50);

		dbg( 1, "set DAQEN and SOFTTRG %08x next, drop %08x", 
		     *ACQ200_SYSCON, ACQ200_SYSCON_SOFTTRG );	

		*ACQ200_SYSCON &= ~ACQ200_SYSCON_SOFTTRG;

		nsleep( 2000 );

		if (DMC_WO->trigger_detect() ||
		     (*ACQ200_FIFCON&FIFO_EMPTY) == 0			){
			return 1;
		}
	}

	sprintf(errbuf, "ERROR failed to trigger after %d retries",
		MAXRETRY);
	DMC_WO->error = errbuf;
	return -1;
}

static int enable_hard_trigger(void)
{
	int nloop = 0;

	activateSignal(CAPDEF->trig);

	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN;

	while(!DMC_WO->trigger_detect()){
		if (has_triggered){
			return 1;
		}
		if (DG->finished_with_engines){
			return -1;
		}
		if ((++nloop&0xfff) == 0){
			DBGSF("POLL TRIG");
		}
		yield();
	}

	DBGSF("..");

	dbg(1, "EV0 = DI3 falling, DAQEN SYSCON %08x", *ACQ200_SYSCON);
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
	    "BEFORE", *ACQ200_FIFCON, *ACQ200_SYSCON);

	if (HITIDE < 0){

		int hot_hitide = 16+HITIDE;
		*ACQ200_FIFCON = 
		(hot_hitide<<ACQ200_FIFCON_LOT_SHIFT)|
		(0xf<<ACQ200_FIFCON_HIT_SHIFT)| 
		ACQ200_FIFCON_HOTHITIE;
	}else{
		*ACQ200_FIFCON = 
			(0x0<<ACQ200_FIFCON_LOT_SHIFT)|
			(HITIDE<<ACQ200_FIFCON_HIT_SHIFT)| 
			ACQ200_FIFCON_HITIE;
	}

	dbg(1, "%10s reset: F:0x%08x S:0x%08x", 
	    "AFTER", *ACQ200_FIFCON, *ACQ200_SYSCON);

/*
 * pgm 20040125: keep soft trigger high (then it cannot be construed as
 * a falling edge on DAQEN?
 */
	*ACQ200_SYSCON |= ACQ200_SYSCON_SOFTTRG|ACQ200_SYSCON_DAQEN;

	dbg(1, "%10s reset: F:0x%08x S:0x%08x", 
	    "ENABLED", *ACQ200_FIFCON, *ACQ200_SYSCON);

	if (DG->activate_event_on_arm){
		struct Phase* phase = DMC_WO->now;

		if (phase->ev){
			activateSignal(phase->ev);
		}
	}
	enable_acq216_start();
	return 0;
}

/** @todo DODGY SHARED CONSTANT - do not change */
#define DATA_LEN   0x100000
#define TIMING_LEN 0x010000

static int nsample;


static u32 getDataPA(void) {
	u32 target = DMC_WO->control_target.pa_data;

	if ((nsample&1) != 0){
		target += DATA_LEN/2;
	}
	return target;
}

static u32 getStatusPA(void) {
	u32 target = DMC_WO->control_target.pa_status;

	if ((nsample&1) != 0){
		target += TIMING_LEN/2;
	}
	return target;
}

static inline void mk_link(
	struct PrebuiltChain *pbc, int ichain,
	struct iop321_dma_desc *dmad
	)
{
	if (ichain && pbc->the_chain[ichain-1]){
		pbc->the_chain[ichain-1]->NDA = dmad->pa;
	}
	dmad->clidat = pbc;
}

static inline void mk_tlatch_to_local(
	struct PrebuiltChain *pbc, int ichain)
{
	struct iop321_dma_desc *dmad = acq200_dmad_alloc();

	mk_link(pbc, ichain, dmad);
	pbc->the_chain[ichain] = dmad;
	dmad->PDA = CAPCOM_PDA + 0x8;    /** TLATCH only */
	dmad->PUAD = 0;
	dmad->LAD = S_pbstore.capcom_scratchpad.pa;
	dmad->BC  = 8;
	dmad->DC = DMA_DCR_FROMDEVICE;

	pbc->id[ichain] = 'T';
}


static inline void mk_capcom_to_local(
	struct PrebuiltChain *pbc, int ichain)
{
	struct iop321_dma_desc *dmad = acq200_dmad_alloc();

	mk_link(pbc, ichain, dmad);
	pbc->the_chain[ichain] = dmad;	
	switch(cd_short_tlatch){
	case 3:
		/** read the host by way of a test */
		dmad->PDA = acq216_pci2bus(DMC_WO->control_target.pa_data);
		pbc->id[ichain] = '3';
		break;
	case 2:
		/** just testing - read the fifo instead.
                 *  data-wise, this is garbage, but is good for testing
                 *  dma chains (master-abort problem).
		 */
		dmad->PDA = DG->fpga.fifo.pa;
		pbc->id[ichain] = '2';
		break;
	default:
		dmad->PDA = CAPCOM_PDA;
		pbc->id[ichain] = 'C';
	}
	dmad->PUAD = 0;
	dmad->LAD = S_pbstore.capcom_scratchpad.pa;
	dmad->BC  = CAPCOM_LENGTH;
	dmad->DC = DMA_DCR_FROMDEVICE;


}


static inline void mk_fifo_to_local(
	struct PrebuiltChain *pbc, int ichain)
{
	pbc->fifo_to_local = ichain;   /* dmad provided later */
	pbc->the_chain[ichain] = 0;

	pbc->id[ichain] = 'F';
}

static inline void mk_local_to_host(
	struct PrebuiltChain *pbc, int ichain, int iblock)
{
	struct iop321_dma_desc *dmad = acq200_dmad_alloc();
	mk_link(pbc, ichain, dmad);
	pbc->local_to_host = ichain;
	
	dmad->PDA = acq216_pci2bus(getDataPA() + iblock*DMA_BLOCK_LEN);
	dmad->PUAD = 0;
	dmad->LAD = 0x11adf00d;
	dmad->BC  = DMA_BLOCK_LEN;
//	dmad->DC  = DMA_DCR_TODEVICE+IOP321_DCR_IE; /** count EOT ints. */
	dmad->DC  = DMA_DCR_TODEVICE;

	pbc->the_chain[ichain] = dmad;

	pbc->id[ichain] = 'H';
}

static inline void mk_capcom_test_block(struct PrebuiltChain *pbc, int ichain)
{
	struct iop321_dma_desc *dmad = acq200_dmad_alloc();
	mk_link(pbc, ichain, dmad);
	
	dmad->PDA = IOP321_REG_PA(IOP321_GTSR);
	dmad->PUAD = 0;
	dmad->LAD = S_pbstore.capcom_scratchpad.pa+CAPCOM_LENGTH;
	dmad->BC = 4;
	dmad->DC = DMA_DCR_MEM2MEM;

	pbc->the_chain[ichain] = dmad;	

	pbc->id[ichain] = 'G';
}
static inline void mk_capcom_to_host(
	struct PrebuiltChain *pbc, int ichain)
{
	struct iop321_dma_desc *dmad = acq200_dmad_alloc();
	mk_link(pbc, ichain, dmad);
	
	dmad->PDA = acq216_pci2bus(getStatusPA());
	dmad->PUAD = 0;
	dmad->LAD = S_pbstore.capcom_scratchpad.pa;
	dmad->BC = CAPCOM_LENGTH+8;
	dmad->DC = DMA_DCR_TODEVICE;

	pbc->the_chain[ichain] = dmad;

	pbc->id[ichain] = 'h';
}


static inline void mk_iodd(struct PrebuiltChain *pbc, int ichain)
{
//	*IOP321_ODR |= BP_INT_LLC_DMA_DONE
//      dmad->LAD = IOP321_REG_PA(IOP321_ODR)
}


static inline void mk_endstop(
	struct PrebuiltChain *pbc, int ichain, 
	struct iop321_dma_desc* endstop)
{
	mk_link(pbc, ichain, endstop);
	pbc->the_chain[ichain] = endstop;

	pbc->id[ichain] = 'E';	
}

#define MK_CAPCOM_TO_LOCAL(pbc, ichain)				\
	if (!cdm_capcom_to_local){				\
		if (cd_short_tlatch == 1){			\
			mk_tlatch_to_local(pbc, ichain++);	\
		}else{						\
			mk_capcom_to_local(pbc, ichain++);	\
		}						\
	}

#define MK_FIFO_TO_LOCAL(pbc, ichain) mk_fifo_to_local(pbc, ichain++)

#define MK_LOCAL_TO_HOST(pbc, ichain, iblock) \
	if (!cdm_local_to_host){ mk_local_to_host(pbc, ichain++, iblock); }

#define MK_CAPCOM_TEST(pbc, ichain) \
	if (!cdm_capcom_test) { mk_capcom_test_block(pbc, ichain++); }
		

#define MK_CAPCOM_TO_HOST(pbc, ichain) \
	if (!cdm_capcom_host) { mk_capcom_to_host(pbc, ichain++); }

#define MK_ENDSTOP(pbc, ichain, es)     mk_endstop(pbc, ichain++, es)
#define MK_IODD(pbc, ichain)            info("WORKTODO");

#define CAPCOM_TO_HOST_ENABLED (DMC_WO->control_target.pa_status != 0)
#define LOCAL_TO_HOST_ENABLED \
	(DMC_WO->control_target.pa_data != 0  && !cdm_local_to_host)


static void prebuilt_insert_local (
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new)
{
	int ilocal = _this->fifo_to_local;
	mk_link(_this, ilocal, _new);
	_this->the_chain[ilocal] = _new;
	_new->NDA = _this->the_chain[ilocal+1]->pa;	
	_new->clidat = _this;                 /* id we are part of a pbc now */
}

static void prebuilt_insert_local_host(
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new)
{
	int ilocal = _this->fifo_to_local;
	mk_link(_this, ilocal, _new);
	_this->the_chain[ilocal] = _new;
	_new->NDA = _this->the_chain[ilocal+1]->pa;
	_this->the_chain[ _this->local_to_host]->LAD = _new->LAD;
	_new->clidat = _this;                 /* id we are part of a pbc now */
}

static void prebuilt_insert_local_host_nodata(
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new)
{
	int ilocal = _this->fifo_to_local;
	mk_link(_this, ilocal, _new);
	_this->the_chain[ilocal] = _new;
	_new->NDA = _this->the_chain[ilocal+1]->pa;
	_new->clidat = _this;                 /* id we are part of a pbc now */
}



const char* acq216_identifyInsert(void* fun)
{
#define FUNID(fun) { fun, #fun }
	struct LUT {
		void* fun;
		const char* fname; 
	} lut[] = {
		FUNID(prebuilt_insert_local),
		FUNID(prebuilt_insert_local_host),
		FUNID(prebuilt_insert_local_host_nodata)
	};
#define NLUT (sizeof(lut)/sizeof(struct LUT))

	int ilut;

	for (ilut = 0; ilut != NLUT; ++ilut){
		if (fun == lut[ilut].fun){
			return lut[ilut].fname + strlen("prebuilt_insert_");
		}
	}

	return "function not identified";
}


static void init_endstops_control_target(void)
/** init endstops, including control_target chains */
{
	struct CONTROL_TARGET* ct = &DMC_WO->control_target;

	int modulo = max(ct->stride, ct->data_blocks);
	int numstops = 1024/2;    /** 50msec buffer */
	int istop = 0;
	int iblock = 0;

	while((numstops % modulo) != 0){
		++numstops;
	}

	numstops *= 2;           /** guarantee 2 buffer op returns to start */

	dbg(1, "stride %d blocks %d modulo %d numstops %d",
	    ct->stride, ct->data_blocks, modulo, numstops);

	init_endstops(numstops);

	/** now build numstops chains
         *  - 0..data_blocks with increasing target offset
         *  - data_blocks..stride : unchanged.
	 */

	if (numstops > S_pbstore.count){
		if (S_pbstore.chains){
			kfree(S_pbstore.chains);
		}
		S_pbstore.chains = kmalloc(numstops*PBC_SZ, GFP_KERNEL);
		if (!S_pbstore.chains){
			err("FAILED to allocate PBC %d", numstops*PBC_SZ);
			return;
		}
		S_pbstore.count = numstops;
	}

	nsample = 0;

	for (nsample = 0; istop < numstops; nsample++){
		for (iblock = 0; iblock < modulo; ++iblock, ++istop){
			struct PrebuiltChain *pbc = S_pbstore.chains+istop;
			struct iop321_dma_desc* endstop;
			int ichain = 0;

			memset(pbc, 0, sizeof(struct PrebuiltChain));

			dbg(3, "istop:%3d iblock:%d", istop, iblock);

			if (!rb_get(&IPC->endstops, &endstop)){
				err("ENDSTOP STARVED");
				finish_with_engines(- __LINE__);
				return;
			}
			if (isPBChainDesc(endstop)){
				err("Already prebuilt");
				finish_with_engines(- __LINE__);
				return;
			}

			/** this idents container of desc */
			pbc->desc.clidat = pbc;
			
			if (iblock < ct->data_blocks){
				if (iblock == ct->data_blocks - 1){
					MK_CAPCOM_TO_LOCAL(pbc, ichain);
					MK_CAPCOM_TEST(pbc, ichain);
				}
				MK_FIFO_TO_LOCAL(pbc, ichain);
				if (LOCAL_TO_HOST_ENABLED){
					MK_LOCAL_TO_HOST(pbc, ichain, iblock);
					pbc->insert = 
						prebuilt_insert_local_host;
				}else{
					pbc->insert=
					 prebuilt_insert_local_host_nodata;
				}
				if (CAPCOM_TO_HOST_ENABLED &&
				    iblock == ct->data_blocks - 1){
					MK_CAPCOM_TO_HOST(pbc, ichain);
				}				
				if (ct->iodd){
					MK_IODD(pbx, ichain);
				}

			}else{
				MK_FIFO_TO_LOCAL(pbc, ichain);
				pbc->insert = prebuilt_insert_local;
			}
			MK_ENDSTOP(pbc, ichain, endstop);
			pbc->length = ichain;

			rb_put(&IPC->endstops, &pbc->desc);
		}
	}

	DG->put_max_empties = numstops/2;
	DG->empty_fill_threshold = numstops/2;
}


#ifdef ACQ216
#warning stop_dead_dma_err_cb enabled.
static void (*original_dma_err_cb)(struct InterruptSync *self, u32 flags);

static void stop_dead_dma_err_cb(struct InterruptSync *self, u32 flags)
{
	*ACQ200_SYSCON &= ~ACQ200_SYSCON_DAQEN;
	original_dma_err_cb(self, flags);
}
#endif



static int acq200_fpga_fifo_read_open (struct inode *inode, struct file *file)
{
	int rc;
	int len = LEN;    /* temp hack */

	dbg(1,"01");


#ifdef ACQ216
	if (abort_acq_on_dma_error){
		if (original_dma_err_cb == 0){
			original_dma_err_cb = IPC->is_dma[0].err.isr_cb;
		}
		IPC->is_dma[0].err.isr_cb = stop_dead_dma_err_cb;
	}
#endif
	enable_regular_operation();
	clear_buffers();
	
	if (DMC_WO->control_target.pa_data || 
	    DMC_WO->control_target.pa_status ){
#if ISR_ADDS_ENDSTOP 
		err("ISR_ADDS_ENDSTOP set - can't do this, revert to regular");
		init_endstops(INIT_ENDSTOPS);
#else
		init_endstops_control_target();
#endif
		DMC_WO->handleEmpties = dmc_handle_empties_prebuilt;
	}else{
		init_endstops(INIT_ENDSTOPS);
		DMC_WO->handleEmpties = dmc_handle_empties_default;
	}
	fifo_open( PCI_DMA_FROMDEVICE );



/* and start the capture */

	dma_sync( va_buf( DG ), len, PCI_DMA_FROMDEVICE );
	build_dmad( va_buf( DG ), len, PCI_DMA_FROMDEVICE );
	
	init_phases();

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
	len = min( len, (size_t)4 );

	dbg( 2, "len %d *offset %d", len, (int)*offset );

	COPY_TO_USER( buf, va_buf_offset( DG, *offset ), len );
	*offset += len;

	return len;
}




/** for use with gated modes. ASSUME DCB has enough data for frame.
 *
 * acq216: DMA_BLOCK_LEN 16384
 * 16 channels : 512 samples
 * @todo handle the single frame case first
 */
static ssize_t 
acq200_fpga_live_read_one_frame_per_dcb(
        struct file *file, 
	char *buf, 
	size_t len, 
	loff_t *offset)
{
	char *base = va_buf(DG);
	struct DataConsumerBuffer *dcb = DCB(file);

	dbg(1, "{%d} enter [%d,%d] %u", 
	    dcb->state, dcb->rb.nput, dcb->rb.nget, *(u32*)offset);


	len = min((int)len, DMA_BLOCK_LEN);
	len &= ~3;

/* poss on the fly tweaks, DG->burst.delay == 1 => include trigger */
	dcb->burst.delay = DG->burst.delay + 1; 

	dbg(2, "off %ld len %d", (long)*offset, len);
	/*
         * first time through, catch up to data pointer
         */
	wait_event_interruptible(dcb->waitq, !u32rb_is_empty(&dcb->rb));

	if (u32rb_is_empty(&dcb->rb)){
		return -EINTR;
	}

	u32rb_get(&dcb->rb, &dcb->last_start);
	dma_sync_single(DG->dev, dcb->handle+dcb->last_start, 
			len, DMA_FROM_DEVICE);

        COPY_TO_USER(buf, base+dcb->last_start, len);
	*offset += len;
	
	return len;
}












/******************************************************************************
 * 
 * SIGNALS
 *
 *****************************************************************************/




static int _acq216_commitEvX(
	struct Signal* signal, 
	volatile u32* reg,
	int shift)
{
	u32 syscon = *reg;
	
	syscon &= ~(ACQ200_SYSCON_EV_MASK << shift);
	
	if (signal->is_active){
		u32 rising = signal->rising? 
			ACQ200_SYSCON_EV_RISING: ACQ200_SYSCON_EV_FALLING;

		syscon |= (rising | acq216_lineCode(signal->DIx)) << shift;
	}	

	dbg(1, "%s %p = 0x%08x", signal->name, reg, syscon);

	*reg = syscon;	
	return 0;
}


static int acq216_commitEv0(struct Signal* signal)
{
	return _acq216_commitEvX(
		signal, ACQ200_SYSCON, ACQ200_SYSCON_EV0_SHIFT);
}

static int acq216_commitEv1(struct Signal* signal)
{
	return _acq216_commitEvX(
		signal, ACQ200_SYSCON, ACQ200_SYSCON_EV1_SHIFT);
}

static int acq216_commitTrg(struct Signal* signal)
{
	return _acq216_commitEvX(
		signal, ACQ200_SYSCON, ACQ200_SYSCON_TRG_SHIFT);
}




static int acq216_commitMasClk(struct Signal* signal)
{
	u32 clkcon = *ACQ200_CLKCON;
	
	clkcon &= ~(ACQ200_CLKCON_OCS_MASK|ACQ200_CLKCON_CLKMAS);

	if (signal->is_active){
                /* @@todo WARNING starts D0 == 0 */
		clkcon |= signal->DIx << ACQ200_CLKCON_OCS_SHIFT; 
		clkcon |= ACQ200_CLKCON_CLKMAS;
	}
	*ACQ200_CLKCON = clkcon;
	return 0;
}


static int acq216_commitAIClk(struct Signal* signal)
{
	u32 clkcon = *ACQ200_CLKCON;
	
	clkcon &= ~(ACQ200_CLKCON_EC_MASK|ACQ200_CLKCON_EXTCLK);
	
	if (signal->is_active){
		clkcon |= signal->rising? ACQ200_CLKCON_EC_RISING: 0;
		/** @@todo WARNING assumes 0:0 relationship */
		clkcon |= signal->DIx<<ACQ200_CLKCON_EC_SHIFT;
		clkcon |= ACQ200_CLKCON_EXTCLK;
	}

	*ACQ200_CLKCON = clkcon;	
	return 0;
}

static int acq216_commitIntClkSrc(struct Signal* signal)
{
	u32 clkcon = *ACQ200_CLKCON;
	
	clkcon &= ~ACQ200_CLKCON_CS_MASK;
	
	if (signal->is_active){
		clkcon |= ACQ200_CLKCON_CS_DI0;
		clkcon |= signal->DIx<<ACQ200_CLKCON_CS_SHIFT;
	}

	*ACQ200_CLKCON = clkcon;	
	return 0;
}


static int acq216_commitTcrSrc(struct Signal* signal)
{
	u32 clkcon = *ACQ216_CCT_CON;
	
	clkcon &= ~ACQ216_CCT_TCS_MASK;	
	clkcon |= signal->DIx<<ACQ216_CCT_TCS_SHL;;

	*ACQ216_CCT_CON = clkcon;	
	return 0;
}


static int acq216_commitObClkSrc(struct Signal* signal)
{
	u32 clkcon = *ACQ200_CLKCON;
	u32 dix;

	switch(signal->DIx){
	case 0: 
		dix = ACQ200_CLKCON_DLL_CS_DI0; break;
	case 1:
		dix = ACQ200_CLKCON_DLL_CS_DI1; break;
	case 2:
		dix = ACQ200_CLKCON_DLL_CS_DI2; break;
	case 3:
		dix = ACQ200_CLKCON_DLL_CS_DI3; break;
	case 4:
		dix = ACQ200_CLKCON_DLL_CS_DI4; break;
	case 5:
		dix = ACQ200_CLKCON_DLL_CS_DI5; break;
	default:
		dix = ACQ200_CLKCON_DLL_CS_16M; break;
	}


	if (signal->is_active){
		clkcon &= ~ACQ200_CLKCON_DLL_CS_MASK;
		clkcon |= dix;
	}
	*ACQ200_CLKCON = clkcon;
	return 0;
}

static struct CAPDEF* acq216_createCapdef(void)
{
	static struct CAPDEF _capdef = {
		.demand_len = 0x100000,
		.channel_mask = 0x0000ffff,
		.mode = M_SOFT_TRANSIENT,
		.pit_stop = 1,
		.pipeline_offset = { 0, 3, 3, 3 }
	};
	struct CAPDEF* capdef = kmalloc(sizeof(struct CAPDEF), GFP_KERNEL);

	memcpy(capdef, &_capdef, sizeof(struct CAPDEF));

	capdef_set_nchan(capdef, 16);
	capdef_set_word_size(capdef, 2);

/* name, minDIx, maxDIx, DIx, rising, is_active, commit */
	capdef->ev[0] = 
		createSignal("event0", 0, 5, 3, 0, 0, acq216_commitEv0);
	capdef->ev[1] = 
		createSignal("event1", 0, 5, 3, 1, 0, acq216_commitEv1);
	capdef->trig  = 
		createSignal("trig", 0, 5, 3, 0, 0, acq216_commitTrg);
	
	capdef->ext_clk = 
		createSignal("ext_clk", 0, 5, 0, 0, 0, acq216_commitAIClk);
	
	capdef->ao_trig = createNullSignal();
	capdef->ao_clk = createNullSignal();

	capdef->int_clk_src = createSignal(
		"int_clk_src", 0, 5, 0, 0, 0, acq216_commitIntClkSrc);
	capdef->mas_clk = createSignal(
		"mas_clk", 0, 5, 1, 0, 0,acq216_commitMasClk);
	capdef->mas_clk->is_output = 1;

	capdef->ob_clk_src = createSignal(
		"ob_clk_src", 0, 6, 6, 0, 1, acq216_commitObClkSrc);
	capdef->counter_src = createSignal("counter_src", 0, 6, 6,0, 1, 
				       acq216_commitTcrSrc);
	dbg(1, "returns capdef %p", capdef);

	return capdef;
}


#include "acq216-offset.h"







static void initPrebuiltChainStore(struct device* dev)
{
	S_pbstore.capcom_scratchpad.va = kmalloc(CAPCOM_LENGTH, GFP_KERNEL);
	S_pbstore.capcom_scratchpad.pa = dma_map_single( 
		dev, S_pbstore.capcom_scratchpad.va, 
		CAPCOM_LENGTH, DMA_BIDIRECTIONAL);
}

static void removePrebuiltChainStore(struct device* dev)
{
	if (S_pbstore.count){
		kfree(S_pbstore.chains);
	}
	dma_unmap_single(dev, 
			 S_pbstore.capcom_scratchpad.pa,
			 S_pbstore.capcom_scratchpad.len,
			 DMA_BIDIRECTIONAL);
	kfree(S_pbstore.capcom_scratchpad.va);
}


static void acq216_driverInit(struct device* dev)
{
	acq216_offset_fs_create(dev);
	initPrebuiltChainStore(dev);
}



static void acq216_driverRemove(struct device* dev)
{
	removePrebuiltChainStore(dev);
	acq216_offset_fs_remove();
}



static void acq216_destroyCapdef(struct CAPDEF *capdef)
{
	destroySignal(capdef->ev[0]);
	destroySignal(capdef->ev[1]);
	destroySignal(capdef->trig);
	destroySignal(capdef->ext_clk);
	destroySignal(capdef->ao_trig);
	destroySignal(capdef->ao_clk);
	destroySignal(capdef->mas_clk);
	destroySignal(capdef->ob_clk_src);
	destroySignal(capdef->counter_src);
	kfree(capdef);
}


int acq216_setTrig(int enable, int dix, int rising)
{
	CAPDEF->trig->is_active = enable;
	CAPDEF->trig->DIx = dix;
	CAPDEF->trig->rising = rising;
	acq216_commitTrg(CAPDEF->trig);
	return 0;
}

#include "acq200-fifo-pcidev.c"

EXPORT_SYMBOL_GPL(acq200_lookup_pchan);
EXPORT_SYMBOL_GPL(acq200_setIntClkHz);
EXPORT_SYMBOL_GPL(acq200_setChannelMask);
EXPORT_SYMBOL_GPL(acq216_setTrig);
EXPORT_SYMBOL_GPL(acq216_setAntiPhase);

#undef DMA_REG

/** hack in refills handling - force wait for DMA done on last entry.
 *  I don't believe this is strictly necessary, so can be disabled by
 *  setting poll_dma_done_calls to zero
 */
#include "acq200-inline-dma.h"

int poll_dma_done_calls = 0;
int poll_dma_done_maxpolls = 0;

module_param(poll_dma_done_calls, int, 0664);
module_param(poll_dma_done_maxpolls, int, 0664);



static void poll_dma_done(void)
{
	DEFINE_DMA_CHANNEL(ai_dma, 0);
	u32 stat;
	int npolls = 0;

	if (poll_dma_done_calls == -1 ){
		return;
	}
	++poll_dma_done_calls;

	while(!DMA_DONE(ai_dma, stat)){
		++npolls;
	}
	
	if (npolls > poll_dma_done_maxpolls){
		poll_dma_done_maxpolls++;
	}
}
