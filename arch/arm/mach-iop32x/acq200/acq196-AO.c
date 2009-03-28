/* ------------------------------------------------------------------------- */
/* acq196-AO.c AO control for acq196                                 */
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

/*
 * @todo LFAWG: Long FAWG : unlimited XX device, loaded from tblocks.
 * echo TBL,...0xbytes >XXL
 * TBL is a list of 3 digit tblock #'s, 0xbytes is a final length
 * NB: the tblocks MUST be contiguous
 * NB: number is decimal, lead 0 does NOT mean octal
 * reserve the tblocks.
 * Then, on COMMIT_LFAWG, feed tblocks to FAWG engine
 * on stop, release the tblocks.
 */
/*
 * VFS From example at http://lwn.net/Articles/57373/
 * Copyright 2002, 2003 Jonathan Corbet <corbet-AT-lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
 */

#define REVID "$Revision: 1.8 $ B110\n"

#define acq200_debug	fawg_debug

#define DTACQ_MACH 2
#define ACQ196
#define ACQ_IS_INPUT 1

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#include "acqX00-port.h"
#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq196.h"

#include "iop321-auxtimer.h"

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <linux/gfp.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>	/* copy_to_user */

#include "acq200-dmac.h"
#include "acq200-inline-dma.h"

#define LFS_MAGIC 0xa196a016

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((2+2*(numchan)+2+1)*TD_SZ)


#define DEF_HZ 10000
#define SAWG_MAXHZ 100000

#define MAX_AO_CHAN 16
#define MAX_AO_FCHAN 16      /* function capable outputs */

#define MAX_AO_LONGS (MAX_AO_CHAN/2)
/*
 * HAWG: Hardware AWG, 2 channels
 * SAWG: Software AWG, 16 channels;
 */

#define WAVE_MAX_SZ (ACQ196_WAVE_MAX*sizeof(u32))

/*
 * commit:
 * 0 => write out the values
 * 1 => commit DC values (reset + arm)
 * 2 => initiate awg 
 * 4 => initiate SAWG
 */
#define COMMIT_DC   0x01    /** set DC output				*/
#define COMMIT_HAWG 0x02    /** initiate HAWG output on trigger 	*/
#define COMMIT_FAWG 0x04    /** initiate FAWG output on trigger		*/
#define COMMIT_REF  0x08    /** enables REF output (MAC firmware only)  */
#define COMMIT_TRIG 0x10    /** software trigger on commit              */
#define COMMIT_ONESHOT 0x20 /** oneshot FAWG action when enabled        */
#define COMMIT_LFAWG 0x40    /** FAWG with large (tblock) buffer         */

#define COMMIT_XFAWG (COMMIT_FAWG|COMMIT_LFAWG)

#define COMMIT_XAWG (COMMIT_HAWG|COMMIT_FAWG|COMMIT_LFAWG)
#define COMMIT_OUTPUT (COMMIT_HAWG|COMMIT_FAWG|COMMIT_LFAWG|COMMIT_DC)
#define COMMIT_MUST_TRIG (COMMIT_TRIG|COMMIT_DC)

struct FunctionBuf {
	u16 *p_start;
	u16 *p_end;
};


#define IREF MAX_AO_FCHAN



struct Globs {
	int commit_word;
	short dacs[MAX_AO_CHAN]; /* direct write */
	struct FunctionBuf fb[MAX_AO_FCHAN+1];

	/**
	 * SAWG : we operate a double buffer scheme for smoothest changeover
	 */
	struct SawgBuffer {
		u32* block;
		u32* cursor;
		u32* last;
		dma_addr_t pa;
		int dmad_count;
		int dmad_cursor;
		struct iop321_dma_desc** dmad;		
	} sawg_buffers[2];
	
	struct Sawg {
		struct SawgBuffer* buffer;
		struct AuxTimerClient atc;
		void (*service_xawg)(struct Sawg* sawg);
		void (*onstop_xawg)(void);
		spinlock_t lock;    
		int update_in_progress; /** condition var prot by lock */
		int please_stop;
		int timer_running;
		void (*eoc_cb)(struct InterruptSync *self, u32 flags);
		int old_eoc_int_modulo_mask;

		u32* fifo_va;    /* copy for isr - static data not trusted */
		int hz;
		u32 sync;         /* optional DIO sync word */

		struct Stats {
			int updates;
			int writeups;
			int dma_busy;
			int zero_fifo_at;
			u32 mean_gtsr;
			u32 max_gtsr;
			u32 prev_gtsr;
			u32 max_delta_gtsr;
		} stats;
	} sawg;
};


struct LfawgStatic {
	struct list_head lfawg_tblocks;
	struct SawgBuffer sb;
	int cursor;
	int limit;
}
	LFAWG;

/* LFAWG uses cursors like this: */
#define LFAWG_CURSOR (LFAWG.cursor)
#define LFAWG_LEN    (LFAWG.limit)

#define _SAWG_MAX_SAMPLES (16384)	/* 8s at 2kHz */
#define SAWG_BUFFER_CURRENT_LEN(sb) ((void*)sb->last - (void*)sb->block)

#define SAWG_DMA_SAMPLES	16
#define SAWG_DMA_LEN		(MAX_AO_CHAN * sizeof(u16) * SAWG_DMA_SAMPLES)

#define AO_SAMPLES(len)	((len)/(MAX_AO_CHAN * sizeof(u16)))

DEFINE_DMA_CHANNEL(dmac1, 1);

int fawg_debug = 0;
module_param(fawg_debug, int, 0664);

int fawg_max_samples = _SAWG_MAX_SAMPLES;
module_param(fawg_max_samples, int, 0444);

int fawg_dma_len = SAWG_DMA_LEN;
module_param(fawg_dma_len, int, 0644);

int AO_channels = 16;
module_param(AO_channels, int, 0444);

int use_ref = 0;
module_param(use_ref, int, 0444);

#define FAWG_CHANNEL_SZ (fawg_max_samples*sizeof(u16))
#define FAWG_BLOCK_SZ (FAWG_CHANNEL_SZ * AO_channels)

int AO_shot = 0;
module_param(AO_shot, int, 0644);

/* linux 2.6.21 ... ino starts at 2 */
#define INO2DC_CHAN(ino) ((ino)-1)
#define INO2F_CHAN(ino) (INO2DC_CHAN(ino) - AO_channels)

static inline int FAWG_PAGE_ORDER(void)
{
	int order = 0;

	for (; (1 << order) * PAGE_SIZE < FAWG_BLOCK_SZ; ++order){
		;
	}
	return order;
}

static inline int CHANNEL_PAGE_ORDER(void)
{
	int order = 0;

	for (; (1 << order) * PAGE_SIZE < FAWG_CHANNEL_SZ; ++order){
		;
	}
	return order;
}
static struct Globs *AOG;


static inline int fbLen(struct FunctionBuf* fb)
{
	return fb->p_end - fb->p_start;
}
static inline int fbFree(struct FunctionBuf* fb)
{
	return fawg_max_samples - fbLen(fb);
}

static void *fbCursor(struct FunctionBuf* fb, int offset_bytes)
{
	return fb->p_start + offset_bytes/sizeof(u16);
}


#define FB(filp) ((struct FunctionBuf *)filp->private_data)
#define SET_FB(filp, fb) (filp->private_data = fb)

static inline unsigned key2offset(int ikey)
/* ikey 1..16 */
{
	return ikey -1;
}
static void set_AO(int ikey, short AO)
{
	AOG->dacs[key2offset(ikey)] = AO;
}


static short get_AO(int ikey)
{
	return AOG->dacs[key2offset(ikey)];
}


static void writeAll(u32* dst, u32* src)
{
	unsigned long long *d = (unsigned long long*)dst;
	unsigned long long *s = (unsigned long long*)src;

	*d++ = *s++;
	*d++ = *s++;
	*d++ = *s++;
	*d++ = *s++;
}

#define DODGY_DOUBLE_DC_RESET 1

#ifdef DODGY_DOUBLE_DC_RESET 
static int double_dc_reset;
#endif

static void write_all(void)
{
	u32* src = (u32*)AOG->dacs;
	u32* dst = DG->fpga.fifo.va;
	int ichan;
	int double_dc_reset_count = 0;


#ifdef DODGY_DOUBLE_DC_RESET
	while(double_dc_reset_count++ <= double_dc_reset){
#else
		{
#endif
#if 1
		__dac_disarm();
		__dac_reset();
		
		writeAll(dst, src);
		writeAll(dst, src);

		__dac_arm();
		__dac_softtrig();
#endif
	}

	if (acq200_debug>2){
		dbg(2, "------------------------");
		for (ichan = 0; ichan != MAX_AO_CHAN/2; ++ichan){
			dbg(2, "%d %p = %04x", 
			    ichan+1, &dst[ichan], src[ichan]);
		}
	}
}


#define TMPSIZE 32

static int access_open(struct inode *inode, struct file *filp)
{
	int chan = INO2DC_CHAN(inode->i_ino);

	if (!IN_RANGE(chan, 1, AO_channels)){
		return -ENODEV;
	}
	filp->private_data = (void*)chan;
	return 0;
}


static ssize_t access_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	char tmp[TMPSIZE];
	int len;
	int yy = get_AO((int)filp->private_data);

	len = snprintf(tmp, TMPSIZE, "0x%04x %d\n", yy&0x0ffff, yy);
	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, tmp + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
}


static ssize_t access_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	char tmp[TMPSIZE];
	int delta;
	long yy;

	if (*offset != 0){
		return -EINVAL;
	}
	if (count > TMPSIZE){
		return -EINVAL;
	}
	memset(tmp, 0, TMPSIZE);
	if (copy_from_user(tmp, buf, count)){
		return -EFAULT;
	}
	if (sscanf(tmp, "m%d", &delta) == 1){
		yy = get_AO((int)filp->private_data) - delta;
	}else if (sscanf(tmp, "p%d", &delta) == 1){
		yy = get_AO((int)filp->private_data) + delta;
	}else{
		yy = simple_strtol(tmp, NULL, 0);
	}
	if (yy < -32768 ) yy = -32768;
	if (yy >  32767 ) yy =  32767;

	set_AO((int)filp->private_data, yy);
	write_all();
	return count;
}

static int access_xx_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t access_xx_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	char tmp[TMPSIZE];
	long yy;
	int ichan;
	int delta;

	if (*offset != 0){
		return -EINVAL;
	}
	if (count > TMPSIZE){
		return -EINVAL;
	}
	memset(tmp, 0, TMPSIZE);
	if (copy_from_user(tmp, buf, count)){
		return -EFAULT;
	}
	if (sscanf(tmp, "m%d", &delta) == 1){
		yy = get_AO(1) - delta;
	}else if (sscanf(tmp, "p%d", &delta) == 1){
		yy = get_AO(1) + delta;
	}else{
		yy = simple_strtol(tmp, NULL, 0);
	}

	if (yy < -32768 ) yy = -32768;
	if (yy >  32767 ) yy =  32767;

	for (ichan = 1; ichan <= MAX_AO_CHAN; ++ichan){
		set_AO(ichan, yy);
	}
	write_all();
	return count;
}



static int commit_open(struct inode *inode, struct file *filp)
{
	return 0;
}



static void _write_wavemem(u32* src, int nsamples, unsigned offset)
{
	int ii;
	u32* dst = (u32*)((u8*)(DG->fpga.regs.va) + offset);

	dbg(1, "write %d to %p", nsamples, dst);

	for (ii = 0; ii != nsamples; ++ii){
		dst[ii] = src[ii];
	}
	setWAVLIMIT(nsamples);
}

static void clearStats(void)
{
	struct Sawg* sawg = &AOG->sawg;
	unsigned long flags;
	spin_lock_irqsave(&sawg->lock, flags);
	memset(&AOG->sawg.stats, 0, sizeof(struct Stats)); 
	memset(DG->stats.ACQ196_AO_HISTO,0,sizeof(DG->stats.ACQ196_AO_HISTO));
	spin_unlock_irqrestore(&sawg->lock, flags);
}
static void write_wavemem(u32* src, int nsamples)
{
	_write_wavemem(src, nsamples, ACQ196_WAVEFORM_OFFSET);
}

static void write_refmem(u32 *src, int nsamples)
{
	_write_wavemem(src, nsamples, ACQ196_MACREF_OFFSET);
}

static inline void setSyncOn(u32 sync)
{
	*ACQ200_DIOCON |= sync<<ACQ200_DIOCON_OUTDAT_SHL;
}

static inline void setSyncOff(u32 sync)
{
	*ACQ200_DIOCON &= ~(sync<<ACQ200_DIOCON_OUTDAT_SHL);
}

static int getAOheadroom (int update_histo) {
	u32 fifcon = *ACQ196_FIFSTAT;

	if (update_histo){
		DG->stats. ACQ196_AO_HISTO 
			[ACQ196_FIFSTAT_FAWG_COUNT(fifcon)>>2]++;
	}
	return 63 - ACQ196_FIFSTAT_FAWG_COUNT(fifcon);
}

static inline void update_gtsr(struct Sawg *sawg, u32 gtsr2, u32 gtsr1)
{
	struct Stats *stats = &sawg->stats;
	u32 dgtsr = gtsr2 - gtsr1;
	
	/* record max interrupt duration */
	if (likely(gtsr2 > gtsr1)){
		stats->mean_gtsr = (stats->mean_gtsr * 7 + dgtsr)>>3;
		if (dgtsr > stats->max_gtsr){
			stats->max_gtsr = dgtsr;
		}
	}
	
	/* and interval == f(latency) */
	if (likely(gtsr1 > stats->prev_gtsr)){
		if (stats->prev_gtsr != 0){
			dgtsr = gtsr1 - stats->prev_gtsr;
			if (dgtsr > stats->max_delta_gtsr){
				stats->max_delta_gtsr = dgtsr;
			}	
		}	
	}
	stats->prev_gtsr = gtsr1;   
}

#define IS_ONESHOT ((AOG->commit_word&COMMIT_ONESHOT) != 0)



static void runLFawgDma(struct Sawg *sawg)
{
	if (getAOheadroom(sawg->stats.updates != 0) > SAWG_DMA_SAMPLES){

//		struct SawgBuffer *sb = sawg->buffer;
		struct iop321_dma_desc *dmad = *LFAWG.sb.dmad;
		unsigned headroom = LFAWG_LEN - LFAWG_CURSOR;
		unsigned bc = min(headroom, SAWG_DMA_LEN);
		unsigned status;
	
		if (bc < SAWG_DMA_LEN){
			if (IS_ONESHOT){
				sawg->please_stop = 1;
				return;
			}else{
				LFAWG_CURSOR = 0;
				bc = SAWG_DMA_LEN;
			}
		}

		if (DMA_DONE(dmac1, status)){
			dmad->PDA = LFAWG.sb.pa + LFAWG_CURSOR;
			dmad->BC = bc;
			DMA_ARM_DIRECT(dmac1, dmad);
			DMA_FIRE(dmac1);
			/* house keeping in dead time */
			LFAWG_CURSOR += bc;
			++sawg->stats.writeups;
		}else{
			++sawg->stats.dma_busy;	
		}
	}	

	
	dbg(3, "99: cursor %d", LFAWG_CURSOR);		
}

static void service_lfawg(struct Sawg *sawg)
{
	u32 gtsr1 = *IOP321_GTSR;
		
	runLFawgDma(sawg);	

	++sawg->stats.updates;
	update_gtsr(sawg, *IOP321_GTSR, gtsr1);
}

static void service_fawg(struct Sawg *sawg)
{
	struct SawgBuffer *sb = sawg->buffer;
	u32 gtsr1 = *IOP321_GTSR;
	int nwrite = getAOheadroom(sawg->stats.updates != 0);
		
	if (nwrite > SAWG_DMA_SAMPLES){

		int ic = sb->dmad_cursor;
	       	unsigned stat;

		if (DMA_DONE(dmac1, stat)){
			if (ic >= sb->dmad_count){
				if (sawg->sync){
					setSyncOn((sawg->sync));
				}
				if (IS_ONESHOT){
					sawg->please_stop = 1;
					goto done_writing;
				}else{
					ic = 0;		
				}				
			}
			DMA_ARM_DIRECT(dmac1, sb->dmad[ic]);
			DMA_FIRE(dmac1);
			if (sawg->sync && ic == 1){
				setSyncOff(sawg->sync);		
			}
			sb->dmad_cursor = ic + 1;
			++sawg->stats.writeups;
				
			if (nwrite > 60){
				sawg->stats.zero_fifo_at = 
					sawg->stats.writeups;
			}
		}else{
			++sawg->stats.dma_busy;	
		}				
	}
done_writing:
	++sawg->stats.updates;
	update_gtsr(sawg, *IOP321_GTSR, gtsr1);
}
static void sawg_action(unsigned long clidata)
/**
 * runs at interrupt privilege. Service SAWG and rearm
 * uses lock and condition variable to allow possible multiple updaters.
 */
{
	struct Sawg *sawg = (struct Sawg*)clidata;
	unsigned long flags;
	unsigned service = 0;
	
	spin_lock_irqsave(&sawg->lock, flags);
	if (!sawg->update_in_progress){
		sawg->update_in_progress = service = 1;
	}
	spin_unlock_irqrestore(&sawg->lock, flags);
	
	if (!service){
		return;
	}
	if (!sawg->please_stop){
		sawg->service_xawg(sawg);
	}else{
		acq196_fifcon_clr_all(ACQ196_FIFCON_DAC_ENABLE);
		iop321_hookAuxTimer(&sawg->atc, 0);
		sawg->timer_running = 0;
		if (sawg->onstop_xawg){
			sawg->onstop_xawg();
		}
		AOG->commit_word = 0;
		dbg(1, "closedown");
	}
	
	spin_lock_irqsave(&sawg->lock, flags);
	sawg->update_in_progress = 0;
	spin_unlock_irqrestore(&sawg->lock, flags);	
}


static void AO_eoc_isr_cb(struct InterruptSync *self, u32 flags)
{
	struct Sawg* sawg = &AOG->sawg;
	sawg_action((unsigned long)sawg);
	sawg->eoc_cb(self, flags);		
}
static struct SawgBuffer* getInactiveSawgBuffer(void)
{
	return  AOG->sawg.buffer == &AOG->sawg_buffers[1] ?
		&AOG->sawg_buffers[0]:
		&AOG->sawg_buffers[1];
}

static void xawg_timer_arm(
	struct SawgBuffer *sb, 
	void (*service_fun)(struct Sawg *sawg),
	void (*onstop_xawg)(void)
	)
{
	struct Sawg* sawg = &AOG->sawg;
	unsigned long flags;
	/** 
         * timer actually does the work @@todo
         */
	spin_lock_irqsave(&sawg->lock, flags);
	sawg->buffer = sb;
	sawg->service_xawg = service_fun;
	sawg->onstop_xawg = onstop_xawg;

	if (!sawg->timer_running){
		sawg->please_stop = 0;
		sawg->fifo_va = DG->fpga.fifo.va;
		sawg->atc.func = sawg_action;
		sawg->atc.clidata = (unsigned long)sawg;
		sawg->timer_running = 1;
		iop321_hookAuxTimer(&sawg->atc, sawg->hz);
	}
	spin_unlock_irqrestore(&sawg->lock, flags);
}


static void primeFawgDma(struct SawgBuffer *sb)
{
	int ic = 0;
	long j1;
	unsigned status;
	
	/* we DONT want to chain this (would have to break chain on recycle) */
	while(getAOheadroom(0) > SAWG_DMA_SAMPLES && ic < sb->dmad_count){
		DMA_ARM_DIRECT(dmac1, sb->dmad[ic]);
		DMA_FIRE(dmac1);		
		j1 = jiffies;
	
		while(!DMA_DONE(dmac1, status)){
			if (jiffies - j1 > 10){
				break;	
			}		
		}
		++AOG->sawg.stats.writeups;
		++ic;
	}	
	sb->dmad_cursor = ic;	
	
	dbg(1, "99: cursor %d", sb->dmad_cursor);		
}



static void releaseDmad(struct SawgBuffer *sb)
{	
	if (sb->dmad_count){
		int ic;
		for (ic = 0; ic != sb->dmad_count; ++ic){
			acq200_dmad_free(sb->dmad[ic]);		
		}	
		kfree(sb->dmad);
		sb->dmad = 0;
		sb->dmad_count = sb->dmad_cursor = 0;
	}
}
static void prepareFawgDma(struct SawgBuffer *sb)
{
	void *bp, *cursor, *limit;
	int ic;
	int dmad_count;
	int dma_len = fawg_dma_len;
	
	dbg(1, "01");
	
	releaseDmad(sb);
	/* prepare the DMA buffer */
	

	dma_sync_single(
		DG->dev, sb->pa, 
		SAWG_BUFFER_CURRENT_LEN(sb), DMA_FROM_DEVICE);	

	dmad_count = (SAWG_BUFFER_CURRENT_LEN(sb) + dma_len - 1)/dma_len;
			
	sb->dmad = kmalloc(
		dmad_count*sizeof(struct iop321_dma_desc*), GFP_KERNEL);
	
	if (!sb->dmad){
		err("FAILED to allocate DMAD array");
		return;	
	}
	dbg(1, "kmalloc dmad_count %d sb->dmad %p len %d",
			dmad_count, sb->dmad, 
			dmad_count*sizeof(struct iop321_dma_desc*));
	
	
	
	/* parcel into DMAD's */
	for (cursor = bp = sb->block, limit = sb->last, ic = 0; cursor < limit;
		cursor += dma_len, ++ic){
			
		struct iop321_dma_desc *dmad = acq200_dmad_alloc();

		if (!dmad){
			err("acq200_dmad_alloc() STARVE");
			return;
		}
		
		/* MEM2MEM from PDA to LAD */
		dma_len = min(dma_len, limit-cursor);
		dmad->NDA = 0;
		dmad->MM_SRC = sb->pa + (cursor - bp);
		dmad->DD_FIFSTAT = 0;
		dmad->MM_DST  = DG->fpga.fifo.pa;
		dmad->BC = dma_len;
		dmad->DC = DMA_DCR_MEM2MEM;	
		
		sb->dmad[ic] = dmad;
		
		dbg(min(3, 2+(ic&0x3f)), 
			"%3d PDA:0x%08x LAD:0x%08x BC:%d",
			ic, dmad->PDA, dmad->LAD, dmad->BC);
	}
	
	sb->dmad_count = ic;
	
	dbg(1, "98 built %d dmads", ic);
	primeFawgDma(sb);
	
	dbg(1, "99");
}

static void prepareLFawgDma(struct SawgBuffer *sb)
{
	struct iop321_dma_desc *dmad = acq200_dmad_alloc();
	long j1;
	unsigned status;

	releaseDmad(sb);
	sb->dmad = kmalloc(2*sizeof(struct iop321_dma_desc*), GFP_KERNEL);
	if (!sb->dmad){
		err("FAILED to allocate DMAD array");
		return;	
	}

	dmad->NDA = 0;
	dmad->DD_FIFSTAT = 0;
	dmad->LAD  = DG->fpga.fifo.pa;
	dmad->DC = DMA_DCR_MEM2MEM;

	sb->dmad[0] = dmad;
	sb->dmad_count = 1;

	LFAWG_LEN = (sb->last - sb->cursor)*sizeof(u32);
	LFAWG_CURSOR = 0;

	runLFawgDma(&AOG->sawg);
	j1 = jiffies;
	while(!DMA_DONE(dmac1, status)){
		if (jiffies - j1 > 10){
			break;	
		}	
	}
}


static void lfawg_init(void)
{
	INIT_LIST_HEAD(&LFAWG.lfawg_tblocks);
}

static void lfawg_release(void)
{
	TBLE* tble;
	TBLE *n;
	int in_phase;
	int freed_todate = 0;
	
	dbg(1, "01");
iterate:
	list_for_each_entry_safe(tble, n, &LFAWG.lfawg_tblocks, list){
		dbg(2, "tblock %d", tble->tblock->iblock);
		list_del_init(&tble->list);		

		in_phase = atomic_read(&tble->tblock->in_phase);
		if (in_phase != 1){
			err("releasing %d, but in_phase (%d) is not 1 ",
			    tble->tblock->iblock, in_phase);
		}else{
			atomic_dec(&tble->tblock->in_phase);
		}
		acq200_replaceFreeTblock(tble);
		freed_todate++;
		if (freed_todate > 64){
			err("INFINITE loop detected: drop out");
			break;
		}
		goto iterate;
	}
	dbg(1, "99");
}

static inline void chomp(char *str)
{
	char *cursor;

	for (cursor = &str[strlen(str)-1]; cursor > str; --cursor){
		if (*cursor == '\n'){
			*cursor = '\0';
		}else{
			break;
		}
	}
}

static int lfawg_scan(char *def)
{
	int cursor = 0;
	int delta;
	int first_block = -1;
	int current_block = -1;
	char tok[80];

	chomp(def);
/* def is COMMIT[ TB[,TB...]] */
	for(cursor = 0; sscanf(def+cursor, "%s%n", tok, &delta) >= 1;
			cursor += delta					){
		if (cursor != 0){
			/* force base 10 from leading zero */
			int iblock = simple_strtol(tok, 0, 10);
			TBLE *tble = acq200_reserveSpecificTblock(iblock);

			dbg(2, "scan:\"%s\" tok \"%s\" delta %d", 
			    def+cursor, tok, delta);

			dbg(1, "iblock: %d TBLE:%p", iblock, tble);

			if (tble == 0){
				err("Failed to allocate tblock %d", iblock);
				return -1;
			}
			if (first_block == -1){
				first_block = current_block = iblock;
			}else{
				if (++current_block != iblock){
					err("Blocks must be consecutive");
					return -1;
				}
			}
			atomic_inc(&tble->tblock->in_phase);
			list_add_tail(&tble->list, &LFAWG.lfawg_tblocks);
		}
	}
	return (current_block-first_block+1)*TBLOCK_LEN;
}
static struct SawgBuffer *lfawg_load(int lfawg_len)
{
	static struct SawgBuffer *sb = &LFAWG.sb;
	TBLE *first = list_entry(LFAWG.lfawg_tblocks.next, TBLE, list);
	TBLE *last = list_entry(LFAWG.lfawg_tblocks.prev, TBLE, list);

	if (lfawg_len <= 0){
		err("length error"); return 0;
	}
	if (first == 0){
		err( "first is null"); return 0;
	}
	if (last == 0){
		err("last is null"); return 0;
	}
	dbg(1, "first: %p id %d", first, first->tblock->iblock);
	dbg(1, "last: %p id %d", last, last->tblock->iblock);
	memset(sb, 0, sizeof(sb));
	sb->block = sb->cursor = (u32*)VA_TBLOCK(first->tblock);
	sb->last = (u32*)(VA_TBLOCK(last->tblock)+TBLOCK_LEN);
	dbg(1, "dma_map_single %d", lfawg_len);
	/* mapping leaks ... */
	sb->pa = dma_map_single(DG->dev, sb->block,
				lfawg_len, PCI_DMA_TODEVICE);

	info("va:%p pa0x%08x length:%08x",
	     sb->block, sb->pa, (sb->last-sb->block)*sizeof(u32));
	/* now return thing */			

	return sb;
}
static struct SawgBuffer *sawg_load(void)
{
	int isample;
	int ichannel;
	u16 *cursors[MAX_AO_CHAN];
	u32 channel_busy_mask = 0;
	struct SawgBuffer *sb = getInactiveSawgBuffer();
	u32 *pdest = sb->block;
	u32 pair;


	for (ichannel = 0; ichannel < AO_channels; ++ichannel){
		struct FunctionBuf* fb = &AOG->fb[ichannel];

		/**
		 * if empty, user DC setting
		 */
		if (fbLen(fb) == 0){
			*fb->p_start = AOG->dacs[ichannel];
			*fb->p_end += 1;
		}

		cursors[ichannel] = fb->p_start;
		channel_busy_mask |= 1<<ichannel;
	}

	/** if HAWG enabled, ignore first two channels ...
         *  allows HAWG longer than FAWG
         */
	if (AOG->commit_word&COMMIT_HAWG){
		channel_busy_mask &= ~0x3;
	}

	for (isample = 0; 
	     channel_busy_mask && isample < fawg_max_samples; ++isample){

		for (ichannel = 0; ichannel < AO_channels; ichannel += 2){
			pair = *cursors[ichannel] | *cursors[ichannel+1] << 16;

			if (AOG->fb[ichannel].p_end > cursors[ichannel] + 1){
				++cursors[ichannel];
			}else{
				channel_busy_mask &= ~ (1<<ichannel);
			}
			if (AOG->fb[ichannel+1].p_end > cursors[ichannel+1]+1){
				++cursors[ichannel+1];
			}else{
				channel_busy_mask &= ~ (1<<(ichannel+1));
			}
			*pdest++ = pair;
		}


		dbg(3, "block isample:%d busy %08x", 
		    isample, channel_busy_mask);
	}
	sb->last = pdest;
	sb->cursor = sb->block;

	dbg(2, "block: %p cursor:%p max:%p", sb->block, sb->cursor, sb->last);
	return sb;
}
static void sawg_arm(struct SawgBuffer *sb)
/* load all wavedefs from channel buffers to single block buffer */
{
	if (sb != 0){
		AO_shot++;	
		prepareFawgDma(sb);
		xawg_timer_arm(sb, service_fawg, 0);
	}
}

static void lfawg_arm(struct SawgBuffer *sb)
/* lfawg - load from TBLOCK def. ... lazy start ... */
{
	if (sb != 0){
		AO_shot++;
       		prepareLFawgDma(sb);
		xawg_timer_arm(sb, service_lfawg, lfawg_release);
	}
}

static void stop_sawg(void)
{
	struct Sawg* sawg = &AOG->sawg;
	unsigned long flags;

	spin_lock_irqsave(&sawg->lock, flags);
	sawg->please_stop = 1;
	spin_unlock_irqrestore(&sawg->lock, flags);
}
static void hawg_arm(void)
{
	u32* src = kmalloc(WAVE_MAX_SZ, GFP_KERNEL);
	u16* s0 = AOG->fb[0].p_start;
	u16* s1 = AOG->fb[1].p_start;
	u16* e0 = AOG->fb[0].p_end - 1;
	u16* e1 = AOG->fb[1].p_end - 1;
	int iload;
	int finished = 0;

	for (iload = 0; iload < ACQ196_WAVE_MAX && !finished; ++iload){
		src[iload] = (*s1<<16) | *s0;
		finished = 1;                
		if (s0 < e0){
			++s0;
			finished = 0;
		}
		if (s1 < e1){
			++s1;
			finished = 0;
		}
	}

	write_wavemem(src, iload);
	kfree(src);
}

static void ref_arm(void)
{
	u32* src = kmalloc(WAVE_MAX_SZ, GFP_KERNEL);
	u16* s0 = AOG->fb[IREF].p_start;
	u16* e0 = AOG->fb[IREF].p_end - 1;
	int iload;
	int finished = 0;

	for (iload = 0; iload < ACQ196_WAVE_MAX && !finished; ++iload){
		src[iload] = (*s0<<16) | *s0;
		finished = 1;                
		if (s0 < e0){
			++s0;
			finished = 0;
		}
	}

	write_refmem(src, iload);
	kfree(src);
}

static char *commitWordDecode(void)
{
	static char str[80];

	str[0] = '\0';
#define DECODE(key) 					\
	if (AOG->commit_word&(key)){ 			\
		strcat(str, #key); strcat(str, " ");	\
	}
       
	DECODE(COMMIT_DC);
	DECODE(COMMIT_LFAWG);
	DECODE(COMMIT_HAWG);
	DECODE(COMMIT_FAWG);
	DECODE(COMMIT_REF);
	DECODE(COMMIT_TRIG);

	if (strlen(str) == 0){
		strcat(str, "DISABLED");
	}
	return str;
}


static ssize_t commit_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	if (*offset != 0){
		return -EINVAL;
	}else{
		char my_buf[128];
		unsigned commit_word;
		int ncopy = min(count, sizeof(my_buf)-1);

		COPY_FROM_USER(my_buf, buf, ncopy);
		my_buf[ncopy] = '\0';

		if (sscanf(my_buf, "0x%x", &commit_word) == 1 ||
                    sscanf(my_buf, "%d", &commit_word) == 1 ){
			;
		}else{
			return -EFAULT;	
		}

		dbg(1, "commit prev 0x%04x now 0x%04x", 
			    AOG->commit_word, commit_word);


		/* this to enable trigger to work on next try */
		acq196_fifcon_clr_all(ACQ196_FIFCON_DAC_ENABLE);

		if ((commit_word&COMMIT_XAWG) == 0){
			acq196_syscon_dac_clr(ACQ196_SYSCON_ACQEN);
			if ((AOG->commit_word&COMMIT_XFAWG) != 0){
				stop_sawg();
				if ((AOG->commit_word&COMMIT_LFAWG) != 0){
					lfawg_release(); /* free tblocks */
				}
			}
		}

		clearStats();

		if (commit_word&COMMIT_REF){
			ref_arm();
		}
		if (commit_word&COMMIT_HAWG){
			hawg_arm();
			*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_DAC_WAVE_MODE;
		}else{
			*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_DAC_WAVE_MODE;
		}
		if ((commit_word&COMMIT_OUTPUT) != 0){
			dac_reset();
		}
		if ((commit_word&COMMIT_XFAWG) != 0){
			*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_LOWLAT;
			if (AO_channels == 2){
				*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_DAC_2CHAN;
			}
			if ((commit_word&COMMIT_LFAWG) != 0){
				switch(DMC_WO_getState()){
				case ST_STOP:
				case ST_ARM:
					break;
				default:
					err("LFAWG commit state (%d) not "
					    "STOP||ARM", DMC_WO_getState());
					return -1;
				}
				lfawg_arm(lfawg_load(lfawg_scan(my_buf)));
			}else{
				sawg_arm(sawg_load());
			}
		}
		if ((commit_word&COMMIT_DC) != 0){
			*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_LOWLAT;
			write_all();
		}
		if ((commit_word&COMMIT_OUTPUT) != 0){
			dac_arm();
		}
		if ((commit_word&COMMIT_MUST_TRIG) != 0){
			dac_softtrig();
		}
		AOG->commit_word = commit_word;
	}

	return count;
}

static ssize_t commit_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	char tmp[80];
	int len;

	len = snprintf(tmp, 80, "%d 0x%02x %s\n", 
		       AOG->commit_word, AOG->commit_word, commitWordDecode());
	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, tmp + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
	return 0;
}


static int access_wave_open(struct inode *inode, struct file *filp)
{
	SET_FB(filp, &AOG->fb[INO2F_CHAN(inode->i_ino) - 1]);

	if ((filp->f_mode & FMODE_WRITE) != 0){
		/* truncate on write */
		FB(filp)->p_end = FB(filp)->p_start; 

		filp->f_dentry->d_inode->i_size = 0;
	}
	return 0;
}

static ssize_t access_wave_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	count = min(count, fbFree(FB(filp))*sizeof(u16));

	if (count == 0){
		return -EFBIG;
	}
	if (copy_from_user(fbCursor(FB(filp), *offset), buf, count)){
		return -EFAULT;
	}

	FB(filp)->p_end += count/sizeof(u16);
	*offset += count;
	filp->f_dentry->d_inode->i_size += count;

	return count;
}

static ssize_t access_wave_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	int fb_bytes = fbLen(FB(filp))*sizeof(u16);
	int fb_left;
	if (*offset >= fb_bytes){
		return 0;
	}

	fb_left = fb_bytes - *offset;

	count = min((int)count, fb_left);

	if (count > 0){
		COPY_TO_USER(buf, fbCursor(FB(filp), *offset), count);
		*offset += count;
	}
	return count;
}

static int access_wave_mmap(
	struct file *filp, struct vm_area_struct *vma)
{
	return io_remap_pfn_range( 
		vma, vma->vm_start, 
		__phys_to_pfn(virt_to_phys(FB(filp)->p_start)), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
	);
}
static int access_wave_release(
        struct inode *inode, struct file *file)
{
	return 0;
}


static struct tree_descr *my_files;



static int AOfs_fill_super (struct super_block *sb, void *data, int silent)
{
	static struct file_operations access_ops = {
		.open = access_open,
		.read = access_read,
		.write = access_write
	};
	static struct file_operations access_xx_ops = {
		.open = access_xx_open,
		.write = access_xx_write
	};
	static struct file_operations access_wave_ops = {
		.open = access_wave_open,
		.read = access_wave_read,
		.write = access_wave_write,
		.mmap = access_wave_mmap,
		.release = access_wave_release
	};
	static struct file_operations commit_ops = {
		.open = commit_open,
		.write = commit_write,
		.read = commit_read,
	};
	static struct tree_descr xx = {
		.name = "XX",
		.ops = &access_xx_ops,
		.mode = S_IWUSR
	};
	static struct tree_descr front = {
		NULL, NULL, 0
	};

	static struct tree_descr commit = {
		.name = "commit",
		.ops = &commit_ops,
		.mode = S_IWUSR|S_IRUGO
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};
	static char names[MAX_AO_CHAN+1][4];
	static char fnames[MAX_AO_FCHAN+1][8];

	int channel;
	int myfino = 0;
#define ICHAN(channel) ((channel)-1)  /* convert to array index */

	my_files = kmalloc(MY_FILES_SZ(NCHAN), GFP_KERNEL);


	memcpy(&my_files[myfino++], &front, TD_SZ);
	/* linux 2.6.21 ... ino starts at 2 */
	memcpy(&my_files[myfino++], &front, TD_SZ); 
	
	for (channel = 1; channel <= AO_channels; ++channel, ++myfino){
		sprintf(names[channel], "%02d", channel);
		my_files[myfino].name = names[channel];
		my_files[myfino].ops  = &access_ops;
		my_files[myfino].mode = S_IWUSR|S_IRUGO;
	}
	for (channel = 1; channel <= AO_channels; ++channel, ++myfino){
		sprintf(fnames[channel], "f.%02d", channel);
		my_files[myfino].name = fnames[channel];
		my_files[myfino].ops = &access_wave_ops;
		my_files[myfino].mode = S_IWUSR|S_IRUGO;

		AOG->fb[ICHAN(channel)].p_end = 
			AOG->fb[ICHAN(channel)].p_start = (u16*)
			__get_free_pages(GFP_KERNEL, CHANNEL_PAGE_ORDER());
	}
	if (use_ref){
		static char refname[8];

		sprintf(refname, "f.ref");
		my_files[myfino].name = refname;
		my_files[myfino].ops = &access_wave_ops;
		my_files[myfino].mode = S_IWUSR|S_IRUGO;

		AOG->fb[IREF].p_end = AOG->fb[IREF].p_start = (u16*)
			__get_free_pages(GFP_KERNEL, CHANNEL_PAGE_ORDER());
		myfino++;
	}
	memcpy(&my_files[myfino++], &xx, TD_SZ);
	memcpy(&my_files[myfino++], &commit, TD_SZ);
	memcpy(&my_files[myfino++], &backstop, TD_SZ);
	
	return simple_fill_super(sb, LFS_MAGIC, my_files);
#undef ICHAN
}



static int AOfs_get_super(
		struct file_system_type *fst,
		int flags, const char *devname, void *data,
		struct vfsmount* mnt)
{
	return get_sb_single(
		fst, flags, data, AOfs_fill_super, mnt);
}



static struct file_system_type AO_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq196_AOfs",
	.get_sb		= AOfs_get_super,
	.kill_sb	= kill_litter_super,
};

#define GET_PAGES(block)						  \
       do {								  \
       ((block) = (u32*)__get_free_pages(GFP_KERNEL, FAWG_PAGE_ORDER())); \
       info("GET_PAGES %d returned %p", FAWG_PAGE_ORDER(), block);	  \
       }while(0)

#define FREE_PAGES(block) \
        free_pages((unsigned long)(block), FAWG_PAGE_ORDER())

static void create_sb(struct SawgBuffer *sb)
{
	GET_PAGES(sb->block);		
	sb->pa = dma_map_single(
			DG->dev, sb->block, FAWG_BLOCK_SZ, PCI_DMA_TODEVICE);
			
	dbg(1, "sb->block %p pa 0x%08x len %d", 
		sb->block, sb->pa, FAWG_BLOCK_SZ);		
}

static void delete_sb(struct SawgBuffer *sb)
{
	releaseDmad(sb);
	FREE_PAGES(sb->block);	
}
static void create_sawg(void)
{
	create_sb(AOG->sawg_buffers+0);
	create_sb(AOG->sawg_buffers+1);

	AOG->sawg.lock = SPIN_LOCK_UNLOCKED;
	AOG->sawg.hz = DEF_HZ;
}

static void delete_sawg(void)
{
	stop_sawg();
	delete_sb(AOG->sawg_buffers+0);
	delete_sb(AOG->sawg_buffers+1);
}


static ssize_t set_sawg_sync(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	struct Sawg* sawg = &AOG->sawg;
        int ibit;

        if (sscanf(buf, "d%d", &ibit) == 1 ||
            sscanf(buf, "D%d", &ibit) == 1    ){
		if (ibit >= 0 && ibit <=7){
			sawg->sync = 1 << ibit;
		}
	}else if (sscanf(buf, "0x%x", &sawg->sync) == 1 ||
		  sscanf(buf, "%d", &sawg->sync)   == 1   ){
		sawg->sync &= 0x000000ff;
	}

	return strlen(buf);
}
static ssize_t show_sawg_sync(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "0x%02x\n", AOG->sawg.sync);
}

static DEVICE_ATTR(AO_sawg_sync, S_IRUGO|S_IWUGO, 
		   show_sawg_sync, set_sawg_sync);

static ssize_t set_hook_eoc(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int hookit;
	
	if (sscanf(buf, "%d", &hookit) == 1){
		struct Sawg* sawg = &AOG->sawg;
		
		if (hookit){
			if (sawg->eoc_cb == 0){
				sawg->eoc_cb = DG->ipc->is_dma[0].eoc.isr_cb;
				DG->ipc->is_dma[0].eoc.isr_cb = AO_eoc_isr_cb;
				sawg->old_eoc_int_modulo_mask =
					DG->eoc_int_modulo_mask;
				DG->eoc_int_modulo_mask = 0;					
				info("hook");	
			}	
		}else{
			if (sawg->eoc_cb != 0){
				DG->ipc->is_dma[0].eoc.isr_cb = sawg->eoc_cb;
				DG->eoc_int_modulo_mask =
					sawg->old_eoc_int_modulo_mask;	
				info("unhook");	
			}		
		}	
	}
	return count;		
}

static DEVICE_ATTR(hook_eoc, S_IWUGO, 0, set_hook_eoc);

static ssize_t set_sawg_rate(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	struct Sawg* sawg = &AOG->sawg;
	int hz;

	if (sscanf(buf, "%d", &hz) == 1){
		if (hz < 0){
			hz = 0;
		}else if (hz > SAWG_MAXHZ){
			hz = SAWG_MAXHZ;
		}
		sawg->hz = hz;
		if (sawg->timer_running){
			iop321_hookAuxTimer(&sawg->atc, sawg->hz);
		}
	}

	return strlen(buf);
}
static ssize_t show_sawg_rate(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d", AOG->sawg.hz);
}

static DEVICE_ATTR(AO_sawg_rate, S_IRUGO|S_IWUGO, 
		   show_sawg_rate, set_sawg_rate);

static DEVICE_ATTR(AO_fawg_rate, S_IRUGO|S_IWUGO, 
		   show_sawg_rate, set_sawg_rate);

#ifdef DODGY_DOUBLE_DC_RESET
static ssize_t set_double_dc_reset(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &double_dc_reset);
	return strlen(buf);
}
static ssize_t show_double_dc_reset(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d", double_dc_reset);
}

static DEVICE_ATTR(AO_double_dc_reset, S_IRUGO|S_IWUGO, 
		   show_double_dc_reset, set_double_dc_reset);
#endif

static ssize_t show_version(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%s", REVID);
}

static DEVICE_ATTR(AO_version, S_IRUGO, show_version, 0);

static ssize_t show_status(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct Sawg* sawg = &AOG->sawg;
	int len = 0;


#define SAWGPRINTF(fmt, field) \
	len += sprintf(buf+len, "%30s " fmt "\n", #field, sawg->field)

	SAWGPRINTF("%d", stats.updates);
	SAWGPRINTF("%d", stats.writeups);
	SAWGPRINTF("%d", stats.dma_busy);
	SAWGPRINTF("%d", stats.zero_fifo_at);
	SAWGPRINTF("%u usec", stats.max_gtsr/50);
	SAWGPRINTF("%u usec", stats.max_delta_gtsr/50);
	sawg->stats.max_gtsr = 0;
	SAWGPRINTF("%u", stats.mean_gtsr);
	SAWGPRINTF("%d", please_stop);
	SAWGPRINTF("%d", timer_running);
	if (sawg->buffer != 0){
		SAWGPRINTF("%p", buffer->block);
		SAWGPRINTF("%p", buffer->cursor);
		SAWGPRINTF("%p", buffer->last);
	}

	return len;
#undef SAWGPRINTF
}

static DEVICE_ATTR(AO_status, S_IRUGO, show_status, 0);

/** Entry Point */
int acq196_AO_fs_create(struct device* dev)
{
	info("about to register fs");
	AOG = kmalloc(sizeof(struct Globs), GFP_KERNEL);
	memset(AOG, 0, sizeof(struct Globs));

	create_sawg();
	lfawg_init();
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_sawg_rate);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_fawg_rate);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_sawg_sync);
#ifdef DODGY_DOUBLE_DC_RESET 
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_double_dc_reset);
#endif
	DEVICE_CREATE_FILE(dev, &dev_attr_hook_eoc);
	return register_filesystem(&AO_fs_type);
}
void acq196_AO_fs_remove(void)
{
	int ifchan;

	unregister_filesystem(&AO_fs_type);
	kfree(my_files);

	for (ifchan = 1; ifchan <= MAX_AO_FCHAN; ++ifchan){
		free_pages((unsigned)AOG->fb[ifchan-1].p_start, 
				CHANNEL_PAGE_ORDER());
	}

	delete_sawg();
	kfree(AOG);
}
