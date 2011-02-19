#define DTACQ_MACH 0
#define WAV232

#define MODEL_VERID "$Id: wav232-fifo.c,v 1.5 2006/02/08 22:16:09 pgm Exp $\n"

#include "acq200-fifo-top.h"

#define ACQ_INTEN 1
#define PCI_FPGA  1

#define FPGA_INT IRQ_ACQ200_FP
#define FPGA_INT_MASK (1<<FPGA_INT)



#include "wav232.h"
#include "acq200-fifo-local.h"
#include "wav232-fifo.h"

#include <linux/delay.h>

static void init_endstops( int count );   /* @@todo SHOULD BE IN HEADER */


struct pci_dev;

static void acq200_wavegen_driver_remove (struct device *dev);
static int acq200_wavegen_driver_init(struct device *dev);


static struct CAPDEF* wav232_createCapdef(void);
static void wav232_destroyCapdef(struct CAPDEF *capdef);

#define DTACQ_MACH_CREATE_CAPDEF wav232_createCapdef
#define DTACQ_MACH_DESTROY_CAPDEF wav232_destroyCapdef
#define DTACQ_MACH_DRIVER_INIT(dev)   acq200_wavegen_driver_init(dev)
#define DTACQ_MACH_DRIVER_REMOVE(dev) acq200_wavegen_driver_remove(dev)

static void load_dmac_chain( int entries );

/*
 * HACK: this because I don't know how else to force an N-way conditional
 * compile on a C compilation unit
 */

int acq200_lookup_pchan(int lchannel) {
	return lchannel;
}

static inline void stop_capture(void)
{
	*WAV232_SYSCON &= ~WAV232_SYSCON_DAQEN;
	*ACQ200_FIFCON &= ~ACQ200_FIFCON_HC_ENABLE;
}

static int enable_soft_trigger(void);
static int enable_hard_trigger(void); 
/* returns 1 on success */

static void _setExtClk(void)
{
	signalCommit(CAPDEF->ext_clk);
}


static inline unsigned phase_increment(unsigned offset)
{
	offset += DMA_BLOCK_LEN;

	if (offset > WAVDEF->buffer_maxlen){
		offset = 0;
	}
	return offset;
}



static void _setIntClkHz(int hz, long masterclk, u32 clksel)
{
#define MAXDIV    0x0000fffe
	u32 clkdiv;


	if ( hz > masterclk/2 ) hz = masterclk/2;

	clkdiv = (masterclk / hz);

	if ( clkdiv > MAXDIV ) clkdiv = MAXDIV;
	if ( clkdiv < 2 )      clkdiv = 2;

	*WAV232_CLKCON &= ~WAV232_CLKCON_CS_MASK;
	*WAV232_CLKCON |= clksel;
	*ACQ200_CLKDAT = clkdiv;
	*WAV232_SYSCON &= ~WAV232_SYSCON_EXTCLK;

	acq200_clk_hz = masterclk / clkdiv;
	acq200_clk_hz = acq200_rounding(acq200_clk_hz, PRECISION(clkdiv));

	dbg( 1, "hz:%7d clkdiv 0x%08x *ACQ200_CLKDAT 0x%08x hz act %d\n",
	     hz, clkdiv, *ACQ200_CLKDAT, acq200_clk_hz );
}

#define MASTERCLK_66 66666666  


#define MASTERCLK MASTERCLK_66


	static struct IntClkConsts {
		long masterclk;
		u32  clksel;
	} intclk[2] = {
		{
			.masterclk = MASTERCLK_66,
			.clksel    = WAV232_CLKCON_CS_66M
		}
	};
#define MAXSEL 1



void acq200_setIntClkHz( int hz )
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
	
		_setIntClkHz( hz, intclk[imin].masterclk, intclk[imin].clksel );
	}
}


#define ACQ200_SYSCON          WAV232_SYSCON /* a mistake? WORKTODO */
#define ACQ200_SYSCON_LOOPBACK WAV232_SYSCON_LOOPBACK
#define ACQ200_SYSCON_DAQEN    WAV232_SYSCON_DAQEN
#define ACQ200_SYSCON_SOFTTRG  WAV232_SYSCON_SOFTTRG

#define ACQ200_FIFCON_LOTIE    WAV232_FIFCON_LOTIE


#define WAV232_FIFO_NBLOCKS(fifcon)   ((COLD_FIFO_HALF(fifcon)==0) + 1)


#define GET_FIFO_NBLOCKS(nblocks, fifcon) \
do{\
        nblocks = WAV232_FIFO_NBLOCKS(fifcon); \
}while(0)

static int fifo_write_action( size_t len );
static void prime_outbound_fifo(void);

#define SET_SIMULATION_MODE(enable)  /* no simulation for WAV */


unsigned wav232_getNextEmpty(struct DMC_WORK_ORDER* wo)
{
	unsigned this_empty = wo->next_empty;
	unsigned next_empty = wo->next_empty + DMA_BLOCK_LEN;

	if (next_empty + DMA_BLOCK_LEN > WAVDEF->buffer_maxlen){
		next_empty = 0;
		wo->bb_lap_count++;
	}

	wo->next_empty = next_empty;
	return this_empty;
}



static struct DevGlobs wav232_dg = {
	.btype = BTYPE_WAV232,
	.hitide = 0,
	.lotide = 10,
	.max_alloc = 10240,
	.busywait = 0,           
	.sample_read_start = 0,
	.sample_read_stride = 1,
	.bigbuf.tblocks.blt = blt_memcpy,

	.enable_from_eoc_isr = 1,
	.bh_unmasks_eoc = 0,
	.is_oneshot = 1,
	.use_fiq = 1,
	.FIFERR = ACQ200_FIFCON_COLDOVER|ACQ200_FIFCON_COLDUNDER|
	ACQ200_FIFCON_HOTOVER|ACQ200_FIFCON_HOTUNDER,
#ifdef FIQDEBUG
	.CAFEBABE = 0xcafebabe,
	.FEEDCODE = 0xfeedc0de,
	.DEADBEEF = 0xdeadbeef,
#endif
	.empty_fill_threshold = 8000,
	.put_max_empties = 2048,
	.get_max_active = 2048,
	.active_batch_threshold = 1024,
	.init_endstops = 32,
	.eoc_int_modulo_mask = 0  
};
#define MYDG &wav232_dg

#include "acq200-fifo.c"


static void load_dmac_chain( int entries )
{
	struct iop321_dma_desc *pbuf = 0;
	struct iop321_dma_desc *head = 0;
	struct iop321_dma_desc *pbuf_m1 = 0;

	dbg( 2, "entries %d", entries );

	if (!entries){
		return;
	}

	while(entries-- && DMC_WO->next_load < DMC_WO->wo_len){
		if (rb_get(&IPC->empties, &pbuf)){
			if ( !head ){
				head = pbuf;
			}
			if ( pbuf_m1 ){
				pbuf_m1->NDA = pbuf->pa;
				dbg( 3, "m1 %s", dmad_diag(pbuf_m1) );
			}

			pbuf_m1 = pbuf;

			rb_put( &IPC->active, pbuf );
		}else{
			err( "rb_get %s", head==0? "none": "done" );
			break;
		}
		DMC_WO->next_load += DMA_BLOCK_LEN;
	}

	if ( head != 0 ){
		if( rb_get( &IPC->endstops, &pbuf ) ){
			pbuf_m1->NDA = pbuf->pa;
			dbg( 3, "m1 %s", dmad_diag(pbuf_m1) );
		}else{
			err("Run out of endstops next_load %d NDA %08x", 
			    DMC_WO->next_load, 
			    pbuf_m1->NDA);
			tasklet_schedule(&acq200_ipc_error_tasklet);
		}
		*IOP321_DMA0_NDAR = head->pa;
#ifdef OLD_STYLE_ACTIVE_AND_REFILLS
		rb_put( &IPC->active, pbuf );
#else
		rb_put(&IPC->endstops, pbuf); /* short circuit back */
#endif
		dbg( 3, "es %s", dmad_diag(pbuf) );
	}else{
		dbg( 2, "nothing to prime" );
	}

	dbg( 2, "99" );
}


static void enable_wav232_start(void)
{
	int rc;

	dbg(3, "OK: let's trigger FIFCON: 0x%08x SYSCON: 0x%08x", 
	    *ACQ200_FIFCON, *ACQ200_SYSCON);

	dbg(3, "FINAL:next enable FIFCON" );
	*ACQ200_FIFCON |= ACQ200_FIFCON_HC_ENABLE;

	preEnable();

	if (CAPDEF->trig->is_active){
		rc = enable_hard_trigger();
	}else{
		rc = enable_soft_trigger();
	}
	if (rc == 1){
		onEnable();
	}
}


static int fifo_write_action( size_t len )
{
	int ntimeout;

	*ACQ200_ICR = 0;
	*ACQ200_FIFCON = 
		(LOTIDE<<ACQ200_FIFCON_LOT_SHIFT)|
		(0<<ACQ200_FIFCON_HIT_SHIFT) | 
		ACQ200_FIFCON_LOTIE|ACQ200_FIFCON_HC_ENABLE;

	dbg(3, "before prime FIFCON 0x%08x", *ACQ200_FIFCON);

/* prime fifo */

	IPC->is_dma[0].eoc.interrupted = 0;
	IPC->is_dma[0].eoc.isr_cb = regular_dma_irq_eoc_callback;

/* launch */


	*IOP321_DMA0_CCR &= ~IOP321_CCR_CE;

	dbg(3, "IOP321_DMA0_CCR %p disabled 0x%08x", 
	    IOP321_DMA0_CCR, *IOP321_DMA0_CCR );

	prime_outbound_fifo();
	*IOP321_DMA0_CCR = IOP321_CCR_CE;

	dbg(3, "IOP321_DMA0_CCR %p  enabled 0x%08x", 
	    IOP321_DMA0_CCR, *IOP321_DMA0_CCR );

	dbg(3, "after  prime FIFCON 0x%08x", *ACQ200_FIFCON);
	ntimeout = wait_event_interruptible_timeout( 
		IPC->is_dma[0].eoc.waitq,
		IPC->is_dma[0].eoc.interrupted != 0,
		100 );

	dbg(3, "after  EOC   FIFCON 0x%08x", *ACQ200_FIFCON);


	if ( ntimeout == 0 ){
		err( "Timed out on DMAC prime" );
		return -ETIMEDOUT;
	}else{
		IPC->is_dma[0].eoc.isr_cb = fifo_dma_irq_eoc_callback;
		IPC->is_dma[0].eoc.interrupted = 0;
		enable_wav232_start();
		return 0;
	}
}


static void prime_outbound_fifo(void)
/* pre-fill the FIFO from the empties */
{
	int fifo_dblkfree = cold_fifo_dblkfree() + HOT_FIFO_DBLK;

	if ( !fifo_dblkfree ){
		
		err( "no free blocks in fifo" );
		return;
	}

	load_dmac_chain( (COLD_FIFO_SZ+HOT_FIFO_SZ/2)/DMA_BLOCK_LEN );
}




struct wavegen_sample_detail {
	int track;
	int channel;
	unsigned offset;
};

static struct wavegen_sample_detail *get_wavegen_sample_detail(
	struct wavegen_sample_detail* wsd,
	struct file *file,
	loff_t offset )
{
	wsd->track = (MINOR(file->f_dentry->d_inode->i_rdev)&0x00ff)>>5;
	wsd->channel = MINOR(file->f_dentry->d_inode->i_rdev)&0x001f;
	wsd->offset = offset/sizeof(short);
	return wsd;
}

#define WAV_NCHAN   WAVDEF->nchan
#define WAV_NTRACKS WAVDEF->ntracks
#define WAV_SAMPLE_SZ (WAV_NCHAN*WAV_NTRACKS*sizeof(short))


static int acq200_wavegen_open (struct inode *inode, struct file *file)
{
	struct wavegen_sample_detail wsd;

	get_wavegen_sample_detail(&wsd, file, file->f_pos);

/* WORKTODO : nchan should become "sample size" */
	CAPDEF_set_nchan(WAV_NCHAN * WAV_NTRACKS);
/* something like ---
        CAPDEF_set_nchan(WAV_NCHAN);
	CAPDEF_set_word_size(WAV_NTRACKS*sizeof(short));
*/
	WAVDEF->fill_cursor = 0;
	DG->bigbuf.tblocks.cursor = 0;
	WAVDEF->fill_state[wsd.track][wsd.channel] = FILL_STATE_WRITE;
	return 0;
}
static int acq200_wavegen_release (
	struct inode *inode, struct file *file)
{
/* store last pos to enable fill to end of track */
	struct wavegen_sample_detail wsd;

	get_wavegen_sample_detail(&wsd, file, file->f_pos);

	dbg(1, "closing pos %u", wsd.offset);

	WAVDEF->track_ends[wsd.track][wsd.channel] = wsd.offset;
	
	return 0;
}



#define TBLOCK_EXTRACT 1
#define TBLOCK_FILL    0

static ssize_t acq200_wavegen_bufop ( 
	struct file *file, char *buf, size_t len, loff_t *offset,
	int extract
	)
{
#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
	struct wavegen_sample_detail wsd_buf;
	struct wavegen_sample_detail *wsd =
		get_wavegen_sample_detail(&wsd_buf, file, *offset);
	struct BIGBUF *bb = &DG->bigbuf;
	int startoffb = (wsd->offset+DG->sample_read_start)*WAV_SAMPLE_SZ;
	int iblock = startoffb/bb->tblocks.blocklen;
	struct TBLOCK *tblock = &bb->tblocks.the_tblocks[iblock];
	int block_start = iblock * bb->tblocks.blocklen;
	int block_start_sample = block_start/WAV_SAMPLE_SZ;
	int block_off_sample = (startoffb - block_start)/WAV_SAMPLE_SZ;
	int last_sample = LEN/WAV_SAMPLE_SZ;
	int samples_in_block = DG->bigbuf.tblocks.blocklen/WAV_SAMPLE_SZ;
	int last_sample_in_block =
		min(last_sample, block_start_sample+samples_in_block);
	int samples_left_in_block = 
		last_sample_in_block - block_start_sample - block_off_sample;
	int my_samples_left = max(0, samples_left_in_block);
	int req_samples = (int)len/sizeof(short);
	int my_samples_reqlen = min(my_samples_left, req_samples);
	int cpwords;
	int cpbytes;
	
	dbg(2,"buf %p len %d offset %lu", buf, len, (unsigned long)*offset);
	LOCDEB(wsd->channel);
	LOCDEB(wsd->track);
	LOCDEB(extract);
	LOCDEB(startoffb);
	LOCDEB(iblock);
	LOCDEB(block_start);
	LOCDEB(block_start_sample);
	LOCDEB(block_off_sample);
	LOCDEB(samples_in_block);
	LOCDEB(last_sample);
	LOCDEB(last_sample_in_block);
	LOCDEB(samples_left_in_block);
	LOCDEB(my_samples_left);
	LOCDEB(req_samples);
	LOCDEB(my_samples_reqlen);


	if (block_start_sample+block_off_sample >= last_sample){
		dbg(2,"Length Limit reached");
		/* WE ARE DONE */
		return -2321;   /* WORKTODO: make errno */
	}
	if (iblock >= DG->bigbuf.tblocks.nblocks){
		err("ENOMEM");
		return -ENOMEM;
	}
	if (wsd->channel >= NCHAN ){
		err("ENODEV");
		return -ENODEV;
	}
	if ( my_samples_reqlen == 0 ){
		err("BUG");
		return -2320; /* WORKTODO: make errno */
	}


	cpwords = (extract? tblock->extract: tblock->fill)(
		tblock, 
		(short*)buf, 
		my_samples_reqlen,
		wsd->track*WAV_NCHAN + wsd->channel,
		block_off_sample, 
		DG->sample_read_stride );
		
	cpbytes = cpwords * sizeof(short);

	*offset += cpbytes;
	
	dbg(2, "offset %lu returns %d\n\n\n", (unsigned long)*offset, cpbytes);

	return cpbytes;
#undef LOCDEB
}



static ssize_t acq200_wavegen_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	return acq200_wavegen_bufop( 
		file, buf, len, offset, TBLOCK_EXTRACT );
}
static ssize_t acq200_wavegen_write ( 
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
{
	return acq200_wavegen_bufop( 
		file, (char *)buf, len, offset, TBLOCK_FILL );
}







static int w_major;


static int __devinit
acq200_wavegen_driver_init(struct device *dev)
{
	static struct file_operations wavegen_fops = {
		.open = acq200_wavegen_open,
		.write = acq200_wavegen_write,
		.read = acq200_wavegen_read,
		.release = acq200_wavegen_release
	};
	/* int major = */
	w_major = register_chrdev(0, "wav232", &wavegen_fops);

	SET_WAVDEF(kmalloc(sizeof(struct WAV232_PRIVATE), GFP_KERNEL));
	memset(WAVDEF, 0, sizeof(struct WAV232_PRIVATE));

	WAVDEF->buffer_maxlen = len_buf(DG);
	WAVDEF->ntracks = WAV232_MAXTRACKS;
	WAVDEF->nchan = WAV232_MAXCHAN;
	WAVDEF->fill_cursor = 0;

	return 0;
}

static void acq200_wavegen_driver_remove (struct device *dev)
{
	kfree(CAPDEF->private);
	unregister_chrdev(w_major, "wav232");
}


static int enable_soft_trigger(void)
/*
 * acq exit on FIFO not empty is bogus for wav.
 * WBN to have a "triggered" status flag to confirm it worked
 */
{
	*ACQ200_SYSCON  |= ACQ200_SYSCON_DAQEN;
	*ACQ200_SYSCON  |= ACQ200_SYSCON_SOFTTRG;

	udelay(10);

	dbg( 1, "set DAQEN and SOFTTRG %08x next, drop %08x", 
		     *ACQ200_SYSCON, ACQ200_SYSCON_SOFTTRG );	

	*ACQ200_SYSCON &= ~ACQ200_SYSCON_SOFTTRG;
	return 1;
}

static int enable_hard_trigger(void)
{
#define FIFSTA \
	(*ACQ200_FIFCON&(ACQ200_FIFCON_HOTPOINT|ACQ200_FIFCON_COLDPOINT))

	u32 initial_fifo = FIFSTA;
	u32 fifsta;
	int nloop1 = 0;
	int nloop = 0;
	unsigned long j0 = jiffies;
	unsigned long j1 = jiffies;

	/* loop until sure FIFO is stable ... shouldn't take long */


	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN;

	while (ABS(jiffies - j1) < 3){
		if ((fifsta = FIFSTA) != initial_fifo){
			initial_fifo = fifsta;
			j1 = jiffies;
		}
		yield();
		if (++nloop1 > 20){
			break;
		}
	}
	
	activateSignal(CAPDEF->trig);

	dbg(1, "EV0 = DI3 falling, DAQEN SYSCON %08x", *ACQ200_SYSCON);

	while((fifsta = FIFSTA) == initial_fifo){
		if (DG->finished_with_engines){
			return -1;
		}
		if ((++nloop&0xfff) == 0){
			dbg(2,"POLL TRIG");
		}
		yield();
	}

	
	dbg((nloop>100? 1: 0),
	    "POLL TRIG nloop1 %d jiff:%ld nloop %d init 0x%08x trig 0x%08x", 
	    nloop1, ABS(j1-j0), nloop, initial_fifo, fifsta);

	return 1;
#undef FIFSTA
}


static int wav232_commitTrg(struct Signal* signal)
{
	u32 diosfr = *WAV232_DIOSFR;
	
	diosfr &= ~(WAV232_DIOSFR_ET_EN|WAV232_DIOSFR_ET_MASK);
	
	if (signal->is_active){
		diosfr |= WAV232_DIOSFR_ET_EN;
		if (signal->rising){
			diosfr |= WAV232_DIOSFR_ET_RISING;
		}
		/** @@todo WARNING: assumes 0:0 mapping */
		diosfr |= signal->DIx << WAV232_DIOSFR_ET_MASK_SHL;
	}	

	*WAV232_DIOSFR = diosfr;
	return 0;
}



static int wav232_commitAIClk(struct Signal* signal)
{
	u32 diosfr = *WAV232_DIOSFR;
	
	diosfr &= ~(WAV232_DIOSFR_EC_MASK); /* @@todo - no enable?? */

	if (!signal->is_active){
		*WAV232_SYSCON &= ~WAV232_SYSCON_EXTCLK;
	}	
	if (signal->is_active){
		diosfr |= signal->rising? WAV232_DIOSFR_EC_RISING: 0;
		/** @@todo WARNING assumes 0:0 relationship */
		diosfr |= signal->DIx << WAV232_DIOSFR_EC_MASK_SHL;
	}	

	*WAV232_DIOSFR = diosfr;	

	if (signal->is_active){
		*WAV232_SYSCON |= WAV232_SYSCON_EXTCLK;
	}
	return 0;
}



static int wav232_commitMasClk(struct Signal* signal)
{
	u32 clkcon = *WAV232_CLKCON;
	
	clkcon &= ~(WAV232_CLKCON_OCS_MASK|WAV232_CLKCON_CLKMAS);

	if (signal->is_active){
                /* @@todo WARNING starts D0 == 0 */
		clkcon |= signal->DIx <<  WAV232_CLKCON_OCS_SHL; 
		clkcon |= WAV232_CLKCON_CLKMAS;
	}
	*WAV232_CLKCON = clkcon;
	return 0;
}


static struct CAPDEF* wav232_createCapdef(void)
{
	static struct CAPDEF _capdef = {
		.demand_len = 0x100000,
		.channel_mask = 0xffffffff,
		.mode = M_SOFT_TRANSIENT,
		.pit_stop = 1
	};
	struct CAPDEF* capdef = kmalloc(sizeof(struct CAPDEF), GFP_KERNEL);
	memcpy(capdef, &_capdef, sizeof(struct CAPDEF));

	capdef_set_nchan(capdef, 32);
	capdef_set_word_size(capdef, 2);

	capdef->ev[0] = createNullSignal();
	capdef->ev[1] = createNullSignal();

	capdef->trig  = createSignal("trig", 0, 5, 3, 0, 0, wav232_commitTrg);
	
	capdef->ext_clk = 
		createSignal("ext_clk", 0, 5, 0, 0, 0, wav232_commitAIClk);
	
	capdef->ao_trig = createNullSignal();
	capdef->ao_clk = createNullSignal();

	capdef->mas_clk = createSignal(
		"mas_clk", 0, 5, 1, 0, 0,wav232_commitMasClk);
	capdef->mas_clk->is_output = 1;

	return capdef;
}

static void wav232_destroyCapdef(struct CAPDEF *capdef)
{
	destroySignal(capdef->ev[0]);
	destroySignal(capdef->ev[1]);
	destroySignal(capdef->trig);
	destroySignal(capdef->ext_clk);
	destroySignal(capdef->ao_trig);
	destroySignal(capdef->ao_clk);
	destroySignal(capdef->mas_clk);
	kfree(capdef);	
}

#include "acq200-fifo-pcidev.c"


void disable_acq(void)
{

}

void enable_acq(void)
{

}


EXPORT_SYMBOL_GPL(acq200_setIntClkHz);
EXPORT_SYMBOL_GPL(acq200_setChannelMask);
