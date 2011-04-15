/* ------------------------------------------------------------------------- */
/* acq200-fifo-bigbuf-fops.c - bigbuf file interface to fifo driver          */
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

/**
 *   add mu_rma dma read type - read returns a dma descriptor, not data copy
 *
 *   add acq200dmafs - node for each channel
 *   mod DataChannelInfo to take a custom Extractor
 *   define customer Extractor
 *   hook up.
 */

#include "acqX00-port.h"
#include "acq200-fifo-local.h"

#include "acq200_minors.h"

#include <linux/poll.h>

#include "acq200-rb.h"

#include <asm/arch/iop321-dma.h>

#include "acq200-fifo.h"
#include "acq200-fifo-tblock.h"
#include "acq200-stream-api.h"

#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/ctype.h>

#include "acq200-mu-app.h"
#include "gtmr.h"

#if 1
#include <linux/blkdev.h>
#include <linux/pagemap.h>
#include <linux/swap.h>      /* mark_page_accessed() */
#endif

#define ID_CHANXX	0

static int XX_valid;

int debug_tbstat_ev = 0;
module_param(debug_tbstat_ev, int, 0644);
MODULE_PARM_DESC(debug_tbstat_ev, "turn on tbstat debugs");

int tblock_ev_noclear = 1;
module_param(tblock_ev_noclear, int, 0644);
MODULE_PARM_DESC(tblock_ev_noclear, "DEBUG ONLY");

extern int acq200_tblock_debug;		/* @@todo desperate debug measure */

struct FunctionBuf {
	char *p_start;
	char *p_end;
};
static ssize_t format__read(struct file *filp, char *buf,
			    size_t count, loff_t *offset);


#define FB(filp) ((struct FunctionBuf *)filp->private_data)
#define SET_FB(filp, fb) (filp->private_data = fb)

struct FunctionBuf fb_format;

#define FB_LIMIT 32768



void acq200_initDCI(struct file *file, int lchannel)
{
	file->private_data = kmalloc(DCI_SZ, GFP_KERNEL);
	memset(file->private_data, 0, DCI_SZ);

	DCI(file)->lchan = lchannel;
	if (lchannel != ID_CHANXX){
		DCI(file)->pchan = acq200_lookup_pchan(lchannel);
		DCI(file)->ssize = CSIZE;
	}else{
		int pchan;

		for (pchan = 0; pchan < NCHAN; ++pchan){
			if (acq200_pchanEnabled(pchan)){
				DCI(file)->pchan = pchan;
				break;
			}
		}
		DCI(file)->ssize = RSIZE;
	}
	INIT_LIST_HEAD(DCI_LIST(file));
}



void acq200_releaseDCI(struct file *file)
{
	kfree(file->private_data);
}


static struct TblockConsumer *_newTBC(struct LockedList * lockedList, unsigned backlog_limit)
{
	struct TblockConsumer *tbc = 
		kzalloc(sizeof(struct TblockConsumer), GFP_KERNEL);

	tbc->backlog_limit = backlog_limit;	
	init_waitqueue_head(&tbc->waitq);
        INIT_LIST_HEAD(&tbc->tle_q);

	spin_lock(&lockedList->lock);
	list_add_tail(&tbc->list, &lockedList->list);
	spin_unlock(&lockedList->lock);
	
	return tbc;
}

static void _deleteTBC(struct LockedList * lockedList, struct TblockConsumer *tbc)
{
	struct TblockListElement *cursor, *tmp;
	int tblock_backlog = 0;

	spin_lock(&lockedList->lock);
	list_del(&tbc->list);
	spin_unlock(&lockedList->lock);

	/* flush and free any waiting tblocks. 
	 * list disconnected, no need to lock */
	list_for_each_entry_safe(cursor, tmp, &tbc->tle_q, list){
		dbg(1, "release tblock %d", cursor->tblock->iblock);
		acq200_phase_release_tblock_entry(cursor);
		tblock_backlog++;
	}
	
	kfree(tbc);

	if (tblock_backlog > 10){
		err("TBLOCK BACKLOG %d", tblock_backlog);
	}
}

static struct TblockConsumer *newTBC(unsigned backlog_limit)
{
	return _newTBC(&DG->tbc_regular, backlog_limit);
}
static void deleteTBC(struct TblockConsumer *tbc)
{
	return _deleteTBC(&DG->tbc_regular, tbc);
}

#define EVBUF_LEN	256

static struct TblockConsumer *newEventTBC(unsigned backlog_limit)
{
	struct TblockConsumer *tbc = _newTBC(&DG->tbc_event, backlog_limit);
	tbc->clidat = kzalloc(EVBUF_LEN, GFP_KERNEL);
	return tbc;	
}

static void deleteEventTBC(struct TblockConsumer *tbc)
{
	kfree(tbc->clidat);
	return _deleteTBC(&DG->tbc_event, tbc);
}
#define NOSAMPLES 0xffffffff


static unsigned update_inode_stats(struct inode *inode)
{
	int ident = (int)inode->i_private;
	unsigned ssize = 0;
	unsigned samples = 0;

	switch(ident){
	case BIGBUF_DATA_DEVICE_XXP:
	case BIGBUF_DATA_DEVICE_XXL:
		if (XX_valid){
			ssize = sample_size();
			samples = SAMPLES;
		}
		break;
	case BIGBUF_DATA_DEVICE_FMT:
		return inode->i_size;
	default:
		if ((ident&BIGBUF_CHANNEL_DATA_DEVICE) != 0){
			int lchannel = ((unsigned)inode->i_private) & 0x7f;
			if (acq200_lchanEnabled(lchannel)){ 
				ssize = CSIZE;
				samples = DG->getChannelNumSamples(	
					acq200_lookup_pchan(lchannel));
			}
		}
	}

	if (ssize){
		unsigned totsam = (samples- DG->sample_read_start)/
							DG->sample_read_stride;
		if (DG->sample_read_length){
			totsam = min(totsam, DG->sample_read_length);
		}
		inode->i_size = ssize * totsam;	
	}else{
		inode->i_size = 0;
	}
	inode->i_mtime = DG->stats.finish_time;
	return inode->i_size;

}

int acq200_fifo_bigbuf_transform(int blocknum)
{
	struct TBLOCK* tblock = &DG->bigbuf.tblocks.the_tblocks[blocknum];
	short *tb_va = tblock_va(blocknum);
	short *tb_tmp = va_tblock_tmp(DG);
	
	
	unsigned nwords = DG->bigbuf.tblocks.blocklen/sizeof(short);
	unsigned t_flags = DG->bigbuf.tblocks.t_flags;

	XX_valid = false;

	if (!tb_tmp) return -1;

	dbg(1,"transform(%p %p %d %d)", tb_tmp, tb_va, nwords, NCHAN); 

	DG->bigbuf.tblocks.transform(tb_tmp, tb_va, nwords, NCHAN);
	tblock_lock(tblock);
	if ((t_flags&TF_INPLACE) == 0){
		dbg(1, "blt(%p, %p, %d)", tb_va, tb_tmp, nwords);
		DG->bigbuf.tblocks.blt(tb_va, tb_tmp, nwords);
	}

	if ((t_flags&TF_RESULT_IS_RAW) == 0){
		tblock->extract = capdef_get_word_size() == sizeof(u32)?
			tblock_cooked_extractor32: tblock_cooked_extractor;
	}
	tblock_unlock(tblock);

	return 0;
}





static struct TblockListElement* getTble(
	struct Phase *phase, unsigned offsetw )
{
	struct TblockListElement* tble;
	unsigned offset_in_phase = offsetw - phase->start_sample;


	dbg(1, "phase \"%s\" %p offsetw %d offset_in_phase %d", 
	    phase->name,
	    phase, offsetw, offset_in_phase);

	list_for_each_entry(tble, &phase->tblocks, list){
		if (offset_in_phase >= tble->phase_sample_start &&
                    offset_in_phase <  tble_phase_end_sample(tble) ){
			dbg(1, "\ttble %p tblock %p [%d] %s",
			    tble, tble->tblock, 
			    INDEXOF_TBLOCK(tble->tblock), "*");
			return tble;
		}
		dbg(2, "\ttble %p tblock %p [%d]",
		    tble, tble->tblock, INDEXOF_TBLOCK(tble->tblock));
	}

	dbg(1, "phase %p NO TBLE for offset", phase);
	return 0;
}

int acq200_fifo_part_transform(struct Phase* phase)
{
	struct TblockListElement* tble = getTble(phase, 0);
	struct TBLOCK* tblock = tble->tblock;
	short* tb_va = tblock_va(tblock->iblock) + tble->tblock_sample_start;
	short* tb_tmp = va_tblock_tmp(DG);
	unsigned nwords = DG->bigbuf.tblocks.blocklen/sizeof(short);

	if (!tb_tmp) return -1;

	acq200_short_transform(
		tb_tmp, tb_va, nwords, NCHAN, tble->sample_count);
	tblock_lock(tblock);
	/** unfortunately we have to blt the whole thing back */
	DG->bigbuf.tblocks.blt(tb_va, tb_tmp, nwords);
	tblock->extract = capdef_get_word_size() == sizeof(u32)?
			tblock_cooked_extractor32: tblock_cooked_extractor;
	tblock_unlock(tblock);
	return 0;
}




/* static */
struct Phase *getPhaseFromOffset(
	struct DMC_WORK_ORDER* wo, 
	unsigned offsam,
	struct TblockListElement** tble
	)
{
	/* WORKTODO */
	struct Phase *phase;
	int first = 1;

	list_for_each_entry(phase, &DMC_WO->phases, list){
		if (phase_len(phase)){

			dbg(1, "testing phase \"%s\" offsam %d "
				    "start: %d end %d %s",
				    phase->name,
				    offsam,
				    phase->start_sample,
				    phase_end_sample(phase),
				    (offsam >= phase->start_sample &&
				     offsam <  phase_end_sample(phase) )? 
								"GO": "no");

			if (offsam >= phase->start_sample &&
			    offsam <  phase_end_sample(phase) ){
				if (tble){
					*tble = getTble(phase, offsam);
				}
				return phase;
			}else{
				dbg(1, "skipping phase \"%s\" offsam %d "
				    "start: %d end %d",
				    phase->name,
				    offsam,
				    phase->start_sample,
				    phase_end_sample(phase));			
			}			
		}else{
			if (first && !phase->list.next){
				dbg(1,"deprecated single phase ops");
				return phase;
			}
		}
		first = 0;
	}

	return 0;
}



/**
 *  @@todo - implement GMC proposal for sample_read_len - limit length
 *  ... probably do this working on file->offset < sample_read_len ??
 */

void acq200_initBBRP_using_phase(
	struct file* file, size_t len, 
	unsigned offset_samples,
	struct BigbufReadPrams* bbrp,
	struct Phase* phase,
	struct TblockListElement* tble)
{
#define COULD_SHOW_EVENT \
        (DMC_WO->epos_found && phase == DMC_WO->post && stride == 1)

	int channel = DCI(file)->pchan;
	unsigned stride = DG->sample_read_stride;
	unsigned offstart = offset_samples - phase->start_sample;
	int phase_offset_samples =  
		max(tble->phase_sample_start, offstart) +
		(COULD_SHOW_EVENT? -DG->show_event: 0);
	int block_off_sample = 
		phase_offset_samples - tble->phase_sample_start;
	int samples_left_in_block = tble->sample_count - block_off_sample;

	int my_samples_left = max(0, samples_left_in_block)/stride;
	int req_samples = (int)len/DCI(file)->ssize;

	if (samples_left_in_block%stride != 0){
		my_samples_left += 1;   /* already -> first sample */
	}
	bbrp->tble = tble;
	bbrp->tblock = tble->tblock;
	bbrp->my_samples_reqlen = min(my_samples_left, req_samples);
	bbrp->block_off_sample = block_off_sample + tble->tblock_sample_start;
	bbrp->samples_left_in_block = samples_left_in_block;
	bbrp->tblock_samples = bbrp->tblock->tb_length/sample_size()/stride;

	bbrp->extract = DCI(file)->extract? 
		DCI(file)->extract:
		bbrp->tblock->extract;
		

#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define LOCDEP(lv) dbg(3,"%20s %p", #lv, lv)
	
	LOCDEB(bbrp->status);
	LOCDEB(COULD_SHOW_EVENT);
	LOCDEB(DG->show_event);
	LOCDEB(channel);
	LOCDEB(stride);
	LOCDEB(offset_samples);
	LOCDEP(bbrp->tblock);
	LOCDEB(bbrp->tblock->iblock);
	LOCDEB(block_off_sample);
	LOCDEB(samples_left_in_block);
	LOCDEB(my_samples_left);
	LOCDEB(req_samples);
	LOCDEB(tble->tblock_sample_start);
	LOCDEB(bbrp->my_samples_reqlen);
	LOCDEB(bbrp->block_off_sample);
#undef LOCDEB
#undef LOCDEP
}




static void initBBRP(
	struct file *file, size_t len, loff_t *offset,
	struct BigbufReadPrams* bbrp)
{
	struct TblockListElement* tble = 0;
	unsigned offsam = get_fileOffsetSamples(file, offset, 
				  DG->sample_read_stride, DCI(file)->ssize);

	struct Phase* phase = getPhaseFromOffset(DMC_WO, offsam, &tble);

	dbg(2,"len %d offset %lu phase %p tble %p", 
	    len, (unsigned long)*offset, phase, tble);

	if (!phase){
		dbg(1,"null PHASE");
		bbrp->status = BBRP_COMPLETE;
	}else if (phase_len(phase) == 0){
		bbrp->status = BBRP_COMPLETE;
	}else if (!tble){
		err("null TBLOCK LIST ENTRY");
		bbrp->status = -ENO_TBLOCK;
	}else if (!acq200_pchanEnabled(DCI(file)->pchan)){
		dbg(1, "channel Not enabled %d", DCI(file)->lchan);
		bbrp->status = BBRP_COMPLETE;
	}else{
		bbrp->offsam = offsam;
		if (DG->sample_read_length){
			unsigned max_len = 
				DG->sample_read_length*DCI(file)->ssize;
			unsigned len_now = (*(unsigned*)offset);

			if (max_len > len_now){
				max_len = max_len - len_now;
				len = min(len, max_len);
			}else{
				len = 0;
			}
		}
		acq200_initBBRP_using_phase(
			file, len, offsam, bbrp, phase, tble);
	}
}




static int acq200_fifo_bigbuf_open (
	struct inode *inode, struct file *file)
/** works with cdevs */
{
	int lchannel = MINOR(inode->i_rdev)&0x7f;

	assert(inode == file->f_dentry->d_inode);

	acq200_initDCI(file, lchannel);
	update_inode_stats(inode);
	return 0;
}



static int acq200_AIfs_fifo_bigbuf_open (
	struct inode *inode, struct file *file)
/** works with cdevs */
{
	int lchannel = ((unsigned)inode->i_private) & 0x7f;    
	dbg(1, "channel %d", lchannel);
	acq200_initDCI(file, lchannel);
	return 0;
}


ssize_t acq200_fifo_bigbuf_read_bbrp(
	struct file *file, char *buf, size_t len, loff_t *offset,
	struct BigbufReadPrams* bbrp
	)
{
	int channel = DCI(file)->pchan;
	int stride = DG->sample_read_stride;
	int cpwords = 0;
	int cpbytes;
	int rc;
#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define RETURN(val) do { rc = val; LOCDEB(__LINE__); return rc; } while(0)

	if (bbrp->status == BBRP_COMPLETE){
		RETURN(0);
	}else if (bbrp->status < 0){
		RETURN(bbrp->status);
	}else if (!acq200_pchanEnabled(channel)){
		RETURN(0);
	}else if ( bbrp->my_samples_reqlen == 0 ){
		RETURN(0);
	}else{
		int extract_words = bbrp->my_samples_reqlen;

		cpwords = bbrp->extract(
			        bbrp->tblock, 
				(short*)buf, 
				extract_words,
				channel,
				bbrp->block_off_sample, 
				stride );
		
		cpbytes = cpwords * sizeof(short);
		*offset += cpbytes;


		dbg(2, "offset %lu returns %d\n", 
				(unsigned long)*offset, cpbytes);

		RETURN(cpbytes);
	}
#undef RETURN
#undef LOCDEB
}
ssize_t acq200_fifo_bigbuf_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
/* read a linear buffer. len, offset in bytes */
/*
 * BEWARE: this is a nightmare of mixed units
 *
 * contract with caller: read bytes, return bytes
 * contract with extract: units are words
 * but we also have a concept of samples, NCHAN*sizeof(short)
 *
 */
{
	struct BigbufReadPrams bbrp = { 0, };

	dbg(1, "01: len:%u offset:%u", (unsigned)len, (unsigned)*offset);

	if (unlikely(len <= 0)){
		return 0;
	}

	initBBRP(file, len, offset, &bbrp);

/*
 * check for tblock rounddown trap - if trapped, force move into next tblock
 */
	if (bbrp.status != BBRP_COMPLETE && bbrp.my_samples_reqlen == 0 ){
		dbg(1, "block rounddown trap");
		*offset += CSIZE;
		memset(&bbrp, 0, sizeof(bbrp));
	        initBBRP(file, len, offset, &bbrp);
	}
	

	return acq200_fifo_bigbuf_read_bbrp(
		file, buf, len, offset, &bbrp);	

}





static ssize_t acq200_fifo_bigbuf_read_raw ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
/* read a linear buffer. len, offset in bytes. do not de-channlize
 * we do this to benchmark the effect of dechannelization
 */
{
	int smax_chan = DMC_WO->wo_len/sizeof(short);
	int channel = DCI(file)->pchan;
	int strides = DG->sample_read_stride*NCHAN;
	int offsets = *offset/sizeof(short)*strides;
	short* chanbase = 
		(short*)va_buf(DG) + 
		DG->sample_read_start*NCHAN + channel;
	int cplen = 0;

	if (channel >= NCHAN ){
		return 0;
	}

	dbg(1,"ch %02d smax_chan %d offsets %d", channel, smax_chan, offsets);

	len = min(len, (size_t)(smax_chan-offsets)/strides);
	if (copy_to_user(buf, chanbase+offsets, cplen = len)){
		return -EFAULT;
	}
	
	*offset += cplen;
	
	return cplen;
}


static int acq200_fifo_bigbuf_release (
	struct inode *inode, struct file *file)
{
	acq200_releaseDCI(file);
	return 0;
}


static struct file_operations fifo_bigbuf_data_fops = {
	.open = acq200_fifo_bigbuf_open,
	.read = acq200_fifo_bigbuf_read,
	.release = acq200_fifo_bigbuf_release
};


static void fifo_AIfs_vma_open(struct vm_area_struct *vma) {
	struct BigbufReadPrams* bbrp = 
		kzalloc(sizeof(struct BigbufReadPrams), GFP_KERNEL);
	vma->vm_private_data = bbrp;
	dbg(1, "");
}

static void fifo_AIfs_vma_close(struct vm_area_struct *vma) {
	dbg(1, "");
	if (vma->vm_private_data) kfree(vma->vm_private_data);
}

/** WORKTODO: when we run off the end of a Phase, re-init BBRP to 
 * hook next phase */
static struct page *fifo_AIfs_vma_nopage(
	struct vm_area_struct *vma,
	unsigned long address,
	int *type) 
{
#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define RETURN do { LOCDEB(__LINE__); return page; } while(0)


	struct page *page = NOPAGE_SIGBUS;

	loff_t offset = 
		(address - vma->vm_start); // + (vma->vm_pgoff << PAGE_SHIFT);
	struct BigbufReadPrams* bbrp = 
		(struct BigbufReadPrams*)vma->vm_private_data;

	if (offset >= bbrp->my_samples_reqlen){
		initBBRP(vma->vm_file, vma->vm_end-vma->vm_start,
			 &offset, bbrp);
	}
	if (bbrp->status == BBRP_COMPLETE){
		RETURN;
	}else if (bbrp->status < 0){
		RETURN;
	}else if ( bbrp->my_samples_reqlen == 0 ){
		RETURN;
	}else{
		const int ssize = DCI(vma->vm_file)->ssize;
		unsigned choffset = 
			DCI(vma->vm_file)->pchan*bbrp->tblock_samples*ssize;
		unsigned offblock = 
			offset - bbrp->tble->phase_sample_start*ssize;
		unsigned pa = PA_TBLOCK(bbrp->tblock) + offblock + choffset;
		       
		dbg(1, "addr:0x%08lx offset:%6lu pss:%u pat:%08x pa:%08x va:%p",
		    address,
		    (unsigned long)offset, bbrp->tble->phase_sample_start,
		    PA_TBLOCK(bbrp->tblock),
		    pa, VA_TBLOCK(bbrp->tblock) + choffset + offset);

		page = pfn_to_page(pa >> PAGE_SHIFT);
		get_page(page);
		if (type){
			*type = VM_FAULT_MINOR;
		}
		return page;
	}
#undef RETURN
#undef LOCDEB
}



int fifo_AIfs_mmap(struct file * file, struct vm_area_struct * vma)
{
	/** mmap: sets vm_ops */
	static struct vm_operations_struct fifo_AIfs_vm_ops = {
		.open           = fifo_AIfs_vma_open,
		.close          = fifo_AIfs_vma_close,
		.nopage         = fifo_AIfs_vma_nopage,
	};

	file_accessed(file);	     /* touch_atime() better in open() ? */
	vma->vm_ops = &fifo_AIfs_vm_ops;
	vma->vm_ops->open(vma);
	return 0;
}

static void dumpw(void *data, int len)
{
	char buf[180];
	char *pbuf;
	short* sdata = (short*)data;
	int iw;

	len /= 2;

	while(len){
		pbuf = buf;
		pbuf += sprintf(pbuf, "%p: ", sdata);
		for (iw = 0; iw < 16 && len-- != 0; ++iw){
			pbuf += sprintf(pbuf, "%d ", *sdata++);
		}
		info("%s", buf);
	}				
}

static ssize_t extractPages(
	struct TBLOCK *this,
	int maxsam, 
	int channel, 
	int ssize,
	int offset,
	read_descriptor_t * desc,
	read_actor_t actor
	)
/** pass each page in tblock to actor in turn returns #bytes */
{
/* is this valid? What if tblock not full. Doesn't matter, it's an offset! 
 * OK, but what about the maxbytes line?. surely not quite right?.
 */
	int chan_div = channel==ID_CHANXX? 1: NCHAN;
	int bblock_samples = this->tb_length/chan_div/ssize;
	short* bblock_base = (short*)(va_buf(DG) + this->offset + 
						channel*bblock_samples*ssize);
	int maxbytes = min(maxsam, bblock_samples-offset) * ssize;
/*
	unsigned maxbuf = (desc->count - desc->written)/sizeof(short);
*/
	char* bblock_basep = (char*)(bblock_base + offset);
	unsigned long cplen = 0;

	dbg(2, "kickoff maxbytes %d", maxbytes);

        while (cplen < maxbytes){
		struct page *page = virt_to_page(bblock_basep+cplen);
		unsigned poff = (unsigned)(bblock_basep+cplen)&(PAGE_SIZE-1);
		unsigned long len = min(PAGE_SIZE, maxbytes - cplen);

		actor(desc, page, poff, len);


		dbg(4, 
		    "1:calling actor: ka %p [%s] page %p offset:%x len %lu",
		    bblock_basep+cplen,
		    virt_addr_valid(bblock_basep+cplen)? "VALID": "NOT valid",
		    page, offset, len);
		if (acq200_debug > 4){
			dumpw(bblock_basep+cplen+poff, 32);
		}
		dbg(5, "2: page contains: fl:%ld ct %d idx %d",
			page->flags, atomic_read(&page->_count),	
		    (int)page->index);

		cplen += len;
	}
/** @todo ... unpage it here. */

	dbg(2, "returning %lu", cplen);

	return cplen;
}
static ssize_t acq200_fifo_bigbuf_mapping_read ( 
	struct file * file, loff_t *offset,
	read_descriptor_t * desc,
	read_actor_t actor
)
/** read a linear buffer. len, offset in bytes pass the data a page at a time
 *  to actor.
 */
/*
 * BEWARE: this is a nightmare of mixed units
 *
 * contract with caller: read bytes, return bytes
 * contract with extract: units are words
 * but we also have a concept of samples, NCHAN*sizeof(short)
 *
 */
{
	int channel = DCI(file)->pchan;
	int ssize = DCI(file)->ssize;
	int stride = DG->sample_read_stride;

	struct BigbufReadPrams bbrp = { 0, };
	int rc;
#define LOCDEB(lv) dbg(2,"%20s %d", #lv, lv)
#define RETURN(val) do { rc = val; LOCDEB(__LINE__); return rc; } while(0)

	if (unlikely(desc->count <= 0)){
		return 0;
	}

	dbg(2, "channel:%d ssize:%d stride:%d", channel, ssize, stride);
	initBBRP(file, desc->count, offset, &bbrp);
	dbg(2, "return initBBRP");
/*
 * check for tblock rounddown trap - if trapped, force move into next tblock
 */
	if (bbrp.status != BBRP_COMPLETE && bbrp.my_samples_reqlen == 0 ){
		*offset += CSIZE;
	        initBBRP(file, desc->count, offset, &bbrp);
	}
	
	dbg(2, "Through the trap, status %d", bbrp.status);

	if (bbrp.status == BBRP_COMPLETE){
		RETURN(0);
	}else if (bbrp.status < 0){
		RETURN(bbrp.status);
	}else if (channel >= NCHAN ){
		RETURN(0);
	}else if ( bbrp.my_samples_reqlen == 0 ){
		RETURN(0);
	}else if (stride != 1){
		RETURN(-1);
	}else{
/** for each page */             
		dbg(1, "RETURN extractPages %Lu", *offset);

		rc = extractPages(
			       bbrp.tblock, 
			       bbrp.my_samples_reqlen,
			       channel, ssize, bbrp.block_off_sample,
			       desc, actor);
		if (rc > 0){
			*offset += rc;
		}
		RETURN(rc);
	}
#undef RETURN
#undef LOCDEB
}


static ssize_t fifo_AIfs_file_sendfile(struct file *in_file, loff_t *ppos,
			 size_t count, read_actor_t actor, void __user *target)
{
	read_descriptor_t desc = {
		.written = 0,
		.count = count,
		.arg.data = target,
		.error = 0
	};
	ssize_t rc;

	if (!count)
		return 0;

	dbg(1, "STEP:A1 count %d", count);

	while (desc.written < count){
		rc = acq200_fifo_bigbuf_mapping_read(
			in_file, ppos, &desc, actor);

		if (rc == 0){
			break;
		}else if (rc < 0){
			dbg(1, "ERROR returning %d", rc);
			return rc;
		}

		dbg(1, "desc.written %d count %d rc %d desc.count %d", 
		    desc.written, count, rc, desc.count);
	}
		
	dbg(1, "returning %d", desc.written);

	if (desc.written > 0){
		DG->stats.sendfile_bytes += desc.written;
	}
	return desc.written;
}

static struct file_operations fifo_AIfs_bigbuf_data_fops = {
	.open = acq200_AIfs_fifo_bigbuf_open,
	.read = acq200_fifo_bigbuf_read,
	.release = acq200_fifo_bigbuf_release,
/** Experimental : memory mapping and sendfile */
	.mmap		= fifo_AIfs_mmap,
	.sendfile	= fifo_AIfs_file_sendfile
};



void acq200_fifo_set_bigbuf_read_method_raw(int raw)
{
	if (raw){
		fifo_bigbuf_data_fops.read = acq200_fifo_bigbuf_read_raw;
	}else{
		fifo_bigbuf_data_fops.read = acq200_fifo_bigbuf_read;
	}
}
int acq200_fifo_get_bigbuf_read_method_raw(void)
{
	return fifo_bigbuf_data_fops.read==acq200_fifo_bigbuf_read_raw;
}



short tbuf[96]; /* @@hack */
short LUT[96];


/* 
 * direct copy_to_user() is _dog_ slow - copy to tbuf is FASTER!!
 */

static int tblock_rawxx_extractor(
	struct TBLOCK *this,
	short* ubuf, 
	int maxbuf, 
	int channel, 
	int offsam, 
	int stride)
{
	int bblock_samples = this->tb_length/sample_size()/stride;
	short* bblock_base = (short*)(va_buf(DG) + this->offset);
	int cplen;
	int maxwords;
	
	int offsetw = offsam * NCHAN * stride;

	dbg(1, "channel %2d offsam %08x offsetw %08x maxbuf %x", 
	    channel, offsam, offsetw, maxbuf);

	dbg(2, "sample_size() %d stride %d NCHAN %d",
	    sample_size(), stride, NCHAN);

	maxwords = min(maxbuf, bblock_samples-offsam) * NCHAN;

	for(cplen = 0; cplen < maxwords; cplen += NCHAN){
		this->memcpy(tbuf, bblock_base + offsetw, sample_size());
		if (copy_to_user(ubuf+cplen, tbuf, sample_size())){
			return -EFAULT;
		}
		offsetw += stride*NCHAN;
	}		

	dbg(1, "returns maxwords %d", maxwords);
	return maxwords;
}

static int tblock_rawxx_extractor32(
	struct TBLOCK *this,
	short* ubuf, 
	int maxbuf, 
	int channel, 
	int offsam, 
	int stride)
{
	int bblock_samples = this->tb_length/sample_size()/stride;
	short* bblock_base = (short*)(va_buf(DG) + this->offset);
	int cplen;
	int maxwords;
	int stride_words = NCHAN * stride * 2; /* 2 shorts per long */
	
	int offsetw = offsam * stride_words;

	dbg(1, "channel %2d offsam %08x offsetw %08x maxbuf %x", 
	    channel, offsam, offsetw, maxbuf);

	dbg(2, "sample_size() %d stride %d NCHAN %d",
	    sample_size(), stride, NCHAN);

	maxwords = min(maxbuf, bblock_samples-offsam) * NCHAN*2;
	/** above looks really dodgy for overflow, but ... it works better 
		than this:
	maxwords = min(maxbuf, bblock_samples-offsam * NCHAN);
	*/

	for(cplen = 0; cplen < maxwords; cplen += NCHAN*2){
		this->memcpy(tbuf, bblock_base + offsetw, sample_size());
		if (copy_to_user(ubuf+cplen, tbuf, sample_size())){
			return -EFAULT;
		}
		offsetw += stride_words;
	}		

	dbg(1, "returns maxwords %d", maxwords);
	return maxwords;
}




static int dma_xx_extractor(
	struct TBLOCK *this,
	short* ubuf, 
	int maxbuf, 
	int channel, 
	int offsam, 
	int stride)
{
	int bblock_samples = this->tb_length/NCHAN/SSZ;
	struct mu_rma mu_rma = {		
		.magic = MU_MAGIC_BB|MU_HOSTBOUND,
		.status = MU_STATUS_OK
	};

	maxbuf = min(maxbuf, bblock_samples-offsam);

	mu_rma.buffer_offset = this->offset + offsam*NCHAN*SSZ;
	mu_rma.length = maxbuf*NCHAN*SSZ;

	dbg(1, "channel %2d offset %08x maxbuf %x", channel, offsam, maxbuf);

	if (copy_to_user(ubuf, &mu_rma, sizeof(struct mu_rma))){
		return -EFAULT;
	}

	dbg(1, "returns maxbuf %d", maxbuf);
	return maxbuf;
}



ssize_t acq200_fifo_bigbuf_xxX_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset,
	struct BigbufReadPrams* bbrp
	)
{
	int stride = DG->sample_read_stride;

	int cpwords = 0;
	int cpbytes;
	int rc;
#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define RETURN(val) do { rc = val; LOCDEB(__LINE__); return rc; } while(0)

	if (unlikely(len <= 0)){
		return 0;
	}
	
	if (bbrp->status == BBRP_COMPLETE){
		RETURN(0);
	}else if (bbrp->status < 0){
		RETURN(bbrp->status);
	}else if ( bbrp->my_samples_reqlen == 0 ){
		RETURN(0);
	}else{
		Memcpy mc = DCI(file)->memcpy==0? memcpy: DCI(file)->memcpy;
/* WARNING: left forever */
		bbrp->tblock->memcpy = mc;
		cpwords = bbrp->extract(
			        bbrp->tblock, 
				(short*)buf, 
				bbrp->my_samples_reqlen,
				DCI(file)->pchan,
				bbrp->block_off_sample, 
				stride );
		
		cpbytes = cpwords * sizeof(short);
		*offset += cpbytes;


		dbg(2, "offset %lu returns %d\n", 
		    (unsigned long)*offset, cpbytes);

		RETURN(cpbytes);
	}
#undef LOCDEB
}

static ssize_t fifo_bigbuf_xxX_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
{
	struct BigbufReadPrams bbrp = { 0, };

	if (unlikely(len <= 0)){
		return 0;
	}

	initBBRP(file, len, offset, &bbrp);
	
/*
 * check for tblock rounddown trap - if trapped, force move into next tblock
 */
	if (bbrp.status != BBRP_COMPLETE && bbrp.my_samples_reqlen == 0 ){
		*offset += sample_size();
	        initBBRP(file, len, offset, &bbrp);
	}
	
	return acq200_fifo_bigbuf_xxX_read(file, buf, len, offset, &bbrp);
}


int acq200_fifo_bigbuf_xx_open (
	struct inode *inode, struct file *file)
{
	if (!XX_valid){
		return -ENODEV;
	}
	acq200_initDCI(file, ID_CHANXX);
	if (capdef_get_word_size() == sizeof(u32)){
		DCI(file)->extract = tblock_rawxx_extractor32;
	}else{
		DCI(file)->extract = tblock_rawxx_extractor;
	}

	return 0;
}


int acq200_fifo_bigbuf_xx_release (
	struct inode *inode, struct file *file)
{
	acq200_releaseDCI(file);
	return 0;
}


static ssize_t acq200_fifo_bigbuf_xxp_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
{
	return fifo_bigbuf_xxX_read(file, buf, len, offset);
}



static void fillLUT(short LUT[96])
{
	int ichan;

	for (ichan = 0; ichan < NCHAN; ++ichan){
		LUT[ichan] = acq200_lookup_pchan(ichan+1);
	}
}

static void* lut_memcpy(void* to, const void* from, __kernel_size_t nbytes)
{
	short* dst = to;
	const short* src = from;
	int nchan = nbytes/sizeof(short);
	int channel;

	for (channel = 0; channel != nchan; ++channel){
		dst[LUT[channel]] = src[channel];
	}

	return to;
}


static int acq200_fifo_bigbuf_xxl_open (
	struct inode *inode, struct file *file)
{
	if (!XX_valid){
		return -ENODEV;
	}	
	fillLUT(LUT);	
	DCI(file)->memcpy = lut_memcpy;
	return acq200_fifo_bigbuf_xx_open(inode, file);
}



static ssize_t acq200_fifo_bigbuf_xxl_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
{
	return fifo_bigbuf_xxX_read(file, buf, len, offset);
}


/* ONLY valid for linear data starting at BIGBUF:
 * ie a TRANSIENT after
 * set.dtacq set.dtacq free_tblocks 1
 * setArm
 * NO TRANSFORM
 * now upload...
 * @todo - find a way to ensure these conditions are valid ...
 */

static ssize_t bigbuf_data_extractPages(
	int maxbytes,
	int offset,
	read_descriptor_t * desc,
	read_actor_t actor	
	)
{
	char* cursor = (char*)(va_buf(DG) + offset);
	unsigned long cplen = 0;

	while (cplen < maxbytes){
		struct page *page = virt_to_page(cursor);
		unsigned poff = (unsigned)(cursor)&(PAGE_SIZE-1);
		unsigned long len = min(PAGE_SIZE, maxbytes-cplen);

		actor(desc, page, poff, len);
		
		cplen += len;
		cursor += len;
	}

	return cplen;
}

static ssize_t bigbuf_data_mapping_read ( 
	struct file * file, loff_t *offset,
	read_descriptor_t * desc,
	read_actor_t actor
)
/** read a linear buffer. len, offset in bytes pass the data a page at a time
 *  to actor.
 */
/*
 * BEWARE: this is a nightmare of mixed units
 *
 * contract with caller: read bytes, return bytes
 * contract with extract: units are words
 *
 */
{
	int maxbytes = min(sample_size()*SAMPLES, (int)desc->count);

	int rc = bigbuf_data_extractPages(
		maxbytes,
		*offset,
		desc,
		actor);

	if (rc > 0){
		*offset += rc;
	}
	return rc;
}


static int bigbuf_linear_data_mmap(
	struct file *filp, struct vm_area_struct *vma)
{
	int rc = io_remap_pfn_range( 
		vma, vma->vm_start, 
		__phys_to_pfn(pa_buf(DG)), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
	);

	dbg(1, "mapping phys:%u len:%lu rc %d", 
	    pa_buf(DG), vma->vm_end - vma->vm_start, rc);

	return rc;
}

static ssize_t bigbuf_linear_data_sendfile(struct file *in_file, loff_t *ppos,
			 size_t count, read_actor_t actor, void __user *target)
{
	read_descriptor_t desc = {
		.written = 0,
		.count = count,
		.arg.data = target,
		.error = 0
	};
	ssize_t rc;

	if (!count)
		return 0;

	dbg(1, "STEP:A1 count %d", count);

	while (desc.written < count){
		rc = bigbuf_data_mapping_read(
			in_file, ppos, &desc, actor);

		if (rc == 0){
			break;
		}else if (rc < 0){
			dbg(1, "ERROR returning %d", rc);
			return rc;
		}

		dbg(1, "desc.written %d count %d rc %d desc.count %d", 
		    desc.written, count, rc, desc.count);
	}
		
	dbg(1, "returning %d", desc.written);

	if (desc.written > 0){
		DG->stats.sendfile_bytes += desc.written;
	}
	return desc.written;
}


/*
 * PUBLIC ENTRY POINTS
 */

static struct file_operations fifo_bigbuf_xxp_fops = {
	.open = acq200_fifo_bigbuf_xx_open,
	.read = acq200_fifo_bigbuf_xxp_read,
	.release = acq200_fifo_bigbuf_xx_release,
	.mmap		= bigbuf_linear_data_mmap,
//	.sendfile	= bigbuf_linear_data_sendfile
	.sendfile	= fifo_AIfs_file_sendfile
};
static struct file_operations fifo_bigbuf_xxl_fops = {
	.open = acq200_fifo_bigbuf_xxl_open,
	.read = acq200_fifo_bigbuf_xxl_read,
	.release = acq200_fifo_bigbuf_xx_release
};

struct file_operations *acq200_fifo_get_bigbuf_datafops(int iminor)
{
	if ( iminor&BIGBUF_CHANNEL_DATA_DEVICE ){
		return &fifo_bigbuf_data_fops;
	}else{
		switch( iminor ) {
		case BIGBUF_DATA_DEVICE_XXP:
			return &fifo_bigbuf_xxp_fops;
		case BIGBUF_DATA_DEVICE_XXL:
			return &fifo_bigbuf_xxl_fops;
		default:
			return 0;
		}
	}
}



/**
 * acq200dmafs
 */



static int dma_extractor(
	struct TBLOCK *this,
	short* ubuf, 
	int maxbuf, 
	int channel, 
	int offsam, 
	int stride)
{
	int bblock_samples = this->tb_length/NCHAN/sizeof(short);
	u32 tblock_offset =  this->offset + channel*bblock_samples*SSZ;
	struct mu_rma mu_rma = {		
		.magic = MU_MAGIC_BB|MU_HOSTBOUND,
		.status = MU_STATUS_OK
	};

	maxbuf = min(maxbuf, bblock_samples-offsam);

	mu_rma.buffer_offset = tblock_offset + offsam*sizeof(short);
	mu_rma.length = maxbuf*SSZ;

	dbg(1, "channel %2d offset %08x maxbuf %x", channel, offsam, maxbuf);

	if (copy_to_user(ubuf, &mu_rma, sizeof(struct mu_rma))){
		return -EFAULT;
	}

	dbg(1, "returns maxbuf %d", maxbuf);
	return maxbuf;
}


static int dma_ch_open (
	struct inode *inode, struct file *file)
{
/** @@todo - return -ENODEV on stride!=1 or !cooked */
	int lchan = inode->i_ino;    /** we engineered it thus */
	acq200_initDCI(file, lchan);
	DCI(file)->extract = dma_extractor;
	return 0;
}

static int dma_xx_open (
	struct inode *inode, struct file *file)
{
	acq200_initDCI(file, ID_CHANXX);
	DCI(file)->extract = dma_xx_extractor;
	return 0;
}

static int dma_ch_release (
	struct inode *inode, struct file *file)
{
	acq200_releaseDCI(file);
	return 0;
}

static int dma_xx_release (
	struct inode *inode, struct file *file)
{
	acq200_releaseDCI(file);
	return 0;
}


/**
 * whether you get P or L depends on TRANSFORM state - relies on app 
 */
static ssize_t dma_xxl_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
{
	return fifo_bigbuf_xxX_read(file, buf, len, offset);
}

static ssize_t dma_xxp_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
{
	return fifo_bigbuf_xxX_read(file, buf, len, offset);
}

static int dma_tb_open (
	struct inode *inode, struct file *file)
{
	acq200_initDCI(file, ID_CHANXX);
	DCI_TBC(file) = newTBC(DG->bigbuf.tblocks.nblocks - 10);
	DCI(file)->extract = dma_xx_extractor;
	return 0;
}

static int dma_tb_evopen (
	struct inode *inode, struct file *file)
{
	acq200_initDCI(file, ID_CHANXX);
	DCI_TBC(file) = newEventTBC(DG->bigbuf.tblocks.nblocks - 10);
	DCI(file)->extract = dma_xx_extractor;
	return 0;
}

#define STATUS_TB_ALL	999



static void status_tb_free_tblocks(struct list_head* tble_list, int tblock)
{
	TBLE* cursor;
	TBLE* tmp;
	int old_acq200_tblock_debug = acq200_tblock_debug;

	list_for_each_entry_safe(cursor, tmp, tble_list, list){
		if (tblock == STATUS_TB_ALL ||
		    tblock == cursor->tblock->iblock){
			spin_lock(&DG->tbc_regular.lock);
			spin_lock(&DG->tbc_event.lock);

			if (debug_tbstat_ev) acq200_tblock_debug = 1;
			acq200_phase_release_tblock_entry(cursor);
			if (debug_tbstat_ev) acq200_tblock_debug = 0;

			spin_unlock(&DG->tbc_event.lock);
			spin_unlock(&DG->tbc_regular.lock);
		}
	}

	acq200_tblock_debug = old_acq200_tblock_debug;
}


static int dma_tb_release (
	struct inode *inode, struct file *file)
{
	status_tb_free_tblocks(DCI_LIST(file), STATUS_TB_ALL);
	deleteTBC(DCI_TBC(file));
	acq200_releaseDCI(file);
	return 0;
}


static unsigned int tb_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct TblockConsumer *tbc = DCI_TBC(file);

	if (list_empty(&tbc->tle_q)){
		poll_wait(file, &tbc->waitq, poll_table);
	}
	if (!list_empty(&tbc->tle_q)){
		return POLLIN | POLLRDNORM;
	}else{
		return 0;
	}
}
static ssize_t dma_tb_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
/** read output a mu_rma descriptor. 
     NB: returns data size, not descriptor size 
     NB: tblock is mark as in_phase ie busy
     NB: responsibility client to return - by reading to next block ..
*/
{
	struct TblockConsumer *tbc = DCI_TBC(file);
	size_t headroom = 0;
	struct mu_rma mu_rma = {		
		.magic = MU_MAGIC_BB|MU_HOSTBOUND,
		.status = MU_STATUS_OK
	};
	int ncopy;

	if (tbc->c.tle){
		assert(tbc->c.tle->tblock->tb_length >= tbc->c.cursor);

		headroom = tbc->c.tle->tblock->tb_length - tbc->c.cursor;

		dbg(1, "file %p tbc->c.tle %p headroom %d %s", 
		    file, 
		    tbc->c.tle, headroom, headroom==0? "release": "hold");

		if (headroom == 0){
			spin_lock(&DG->tbc_regular.lock);
			spin_lock(&DG->tbc_event.lock);

			acq200_phase_release_tblock_entry(tbc->c.tle);
			spin_unlock(&DG->tbc_event.lock);
			spin_unlock(&DG->tbc_regular.lock);	
			tbc->c.cursor = 0;
		}
	}else{
		dbg(1, "file %p tbc->c.tle %d", file, 0);
	}

	if (headroom == 0){
		dbg(1, "file %p wait", file);
		wait_event_interruptible(tbc->waitq, !list_empty(&tbc->tle_q));

		if (list_empty(&tbc->tle_q)){
			return -EINTR;
		}
		if (tbc->backlog){
			--tbc->backlog;
		}
		tbc->c.tle = TBLE_LIST_ENTRY(tbc->tle_q.next);
		headroom = tbc->c.tle->tblock->tb_length;
		dbg(1, "file %p tbc->c.tle %p wait over headroom %d", 
		    file, tbc->c.tle, headroom);
	}

	ncopy = min(len, headroom);

	mu_rma.buffer_offset = tbc->c.tle->tblock->offset + tbc->c.cursor;
	mu_rma.length = ncopy;
	COPY_TO_USER(buf, &mu_rma, sizeof(struct mu_rma));
	tbc->c.cursor += ncopy;	
	if (offset){
		*offset += ncopy;
	}

	dbg(1, "file %p 99 return %d", file, ncopy);
	return ncopy;
}


static ssize_t status_tb_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
/** output last tblock as string data.
 * TB is NOT marked as in_phase (busy).
 */
{
	struct TblockConsumer *tbc = DCI_TBC(file);
	struct TblockListElement *tle;
	char lbuf[80];
	int rc;
		
	wait_event_interruptible(tbc->waitq, !list_empty(&tbc->tle_q));

	if (list_empty(&tbc->tle_q)){
		return -EINTR;
	}

	tle = TBLE_LIST_ENTRY(tbc->tle_q.next);
	list_del(&tle->list);

	if (len < 8){
		rc = snprintf(lbuf, len, "%3d\n", tle->tblock->iblock);
	}else{
		rc = snprintf(lbuf, min(sizeof(lbuf), len), 
			"tblock %3d off 0x%08x phys:0x%08x len %d scount %d\n",
			tle->tblock->iblock, 
			tle->tblock->offset,
			pa_buf(DG) + tle->tblock->offset,
			tle->tblock->tb_length,
			tle->sample_count
			);
	}
	tbc->c.tle = tle;

	if (tbc->backlog){
		--tbc->backlog;
	}


	COPY_TO_USER(buf, lbuf, rc);
	return rc;
}

/* threshold to include left,right TBLOCKS */
#define BORDERLINE		0x100000
#define TB_NOT_BORDERLINE	999


static int readClearEvCount(int tbix)
{
	if (tbix == TB_NOT_BORDERLINE){
		return 0;
	}else{
		struct TBLOCK_EVENT_INFO *tbinfo =
			&DG->bigbuf.tblock_event_table[tbix];
		unsigned stat = tbinfo->event;

		dbg(1, "[%d] = 0x%08x %d %06x",
		    tbix, stat, TBLOCK_EVENT_COUNT(stat), 
		    TBLOCK_EVENT_OFFSET(stat));

		if (!tblock_ev_noclear){
			memset(tbinfo, 0, sizeof(struct TBLOCK_EVENT_INFO));
		}

		return TBLOCK_EVENT_COUNT(stat);
	}
}

#define XX_DMA_BLOCK_LEN	4096	/** @@hack  ACQ132 ONLY  */
static ssize_t status_tb_evread (
	struct file *file, char *buf, size_t len, loff_t *offset)
/** output last tblock as string data.
 * TB is NOT marked as in_phase (busy).
 *
 * @@todo no way to recycle the buffers. Suggest maintain a list of tble
 *  - add a list element to DCI
 *  - then the write() function takes an int arg, scans the list and frees
 *    corresponding tble.
 */
{
	struct TblockConsumer *tbc = DCI_TBC(file);
	struct TblockListElement *tle;
	char* lbuf = tbc->clidat;
	int rc;

	if (tbc->clidat == 0){
		err("no local buffer");
		return -ENODEV;
	}
	if (DG->bigbuf.tblock_event_table == 0){
		err("no tblock_event_table");
		return -ENODEV;
	}

	wait_event_interruptible(tbc->waitq, !list_empty(&tbc->tle_q));

	if (list_empty(&tbc->tle_q)){
		return -EINTR;
	}

	tle = TBLE_LIST_ENTRY(tbc->tle_q.next);
	list_del(&tle->list);

	if (len < 8){
		rc = snprintf(lbuf, len, "%3d\n", tle->tblock->iblock);
	}else{
		int tb_prev = TB_NOT_BORDERLINE;
		int tb_next = TB_NOT_BORDERLINE;
		int ev_count[3] = {};
		int tbix = tle->tblock->iblock;

		TBLE *tble_prev = list_entry(tle->neighbours.prev, TBLE, list);
		TBLE *tble_next = list_entry(tle->neighbours.next, TBLE, list);

		unsigned last_offset = tle->event_offset;

		struct TBLOCK_EVENT_INFO *tbinfo = 
			DG->bigbuf.tblock_event_table+tbix;

		if (tle->event_offset > BORDERLINE){
			if (tble_prev->tblock){
				acq200_phase_release_tblock_entry(tble_prev);
			}
		}else{
			if (tble_prev->tblock){
				tb_prev = tble_prev->tblock->iblock;
				list_move_tail(&tble_prev->list, DCI_LIST(file));
			}
		}

		if (DMC_WO->early_event_checking && tbinfo->event){
			struct TBLOCK_EVENT_INFO *tbinfo =
				&DG->bigbuf.tblock_event_table[tbix];
			unsigned lo = tbinfo_get_offsetn(tbinfo->eventN);

			dbg(1, "change offset from %d to %d", last_offset, lo);

			if (lo){
				last_offset = lo;
			}
		}

		if (last_offset < TBLOCK_LEN(DG)-BORDERLINE){
			if (tble_next->tblock){
				acq200_phase_release_tblock_entry(tble_next);
			}
		}else{
			if (tble_next->tblock){
				tb_next = tble_next->tblock->iblock;
				list_move_tail(&tble_next->list,DCI_LIST(file));
			}
		}


		if (DMC_WO->early_event_checking && tbinfo->event){
			unsigned long long gtmr1 = 
				(DG->stats.early_start_gtmr!=0?
				 DG->stats.early_start_gtmr:
				 DG->stats.start_gtmr);
			unsigned long long gtmr = tbinfo->gtmr - gtmr1;
			unsigned tblockN = tbinfo_get_tbcount(tbinfo->eventN);
			unsigned offN = tbinfo_get_offsetn(tbinfo->eventN);

			dbg(1, "gtmr %llu start %llu diff %llu",
			    tbinfo->gtmr, DG->stats.start_gtmr, gtmr);

			do_div(gtmr, GTMR_TICK_PER_MSEC);

			dbg(1, "gtmr %llu start %llu diff %llu",
			    tbinfo->gtmr, DG->stats.start_gtmr, gtmr);

			ev_count[0] = readClearEvCount(tb_prev);
			ev_count[1] = readClearEvCount(tbix);
			ev_count[2] = readClearEvCount(tb_next);

			rc = snprintf(lbuf, min(EVBUF_LEN, (int)len),
				"tblock=%03d,%03d,%03d "
				"pss=%-8u esoff=0x%08x,0x%08x "
				"ecount=%d,%d,%d "
				"msec=%llu tblockN=%u\n",
				      tb_prev, tle->tblock->iblock, tb_next,
				      tle->phase_sample_start,
				      tle->event_offset, offN,
				      ev_count[0], ev_count[1], ev_count[2],
				      gtmr, tblockN);
		}else{
			rc = snprintf(lbuf, min(EVBUF_LEN, (int)len),
			"tblock=%03d,%03d,%03d pss=%-8u esoff=0x%08x "
				      "\n",
					tb_prev, tle->tblock->iblock, tb_next,
					tle->phase_sample_start,
					tle->event_offset);
		}


		


		DG->stats.event0_count += ev_count[0]+ev_count[1]+ev_count[2];
		list_add_tail(&tle->list, DCI_LIST(file));		
	}
	tbc->c.tle = tle;

	if (tbc->backlog){
		--tbc->backlog;
	}
	dbg(!debug_tbstat_ev, "read:\"%s\"", lbuf);
	COPY_TO_USER(buf, lbuf, rc);
	return rc;
}


static ssize_t status_tb_evwrite(
	struct file *file, const char *buf, size_t len, loff_t *offset)
{
	char lbuf[80];
	char *s1;
	char *s2;
	
	if (len >= 80) len = 80-1;

	/** lazy init - first write sets mode flag */
	DCI(file)->flags |= DCI_FLAGS_NORELEASE_ON_READ;

	COPY_FROM_USER(lbuf, buf, len);

	if ((s1 = strstr(lbuf, "tblock=")) != 0){
		s1 += strlen("tblock=");
	}else{
		s1 = lbuf;
	}
	for ( ; s1 - lbuf < len; s1 = s2){
		unsigned tbix;
		if ((s2 = strpbrk(s1, ", ")) != 0){
			*s2++ = '\0';
		}

		tbix = simple_strtoul(s1, 0, 10);

		dbg(!debug_tbstat_ev, "free:\"%03d\"", tbix);

		status_tb_free_tblocks(DCI_LIST(file), tbix);
	}
	return len;
}

#define status_tb_write status_tb_evwrite

static int dma_tb_evrelease (
	struct inode *inode, struct file *file)
{
	status_tb_free_tblocks(DCI_LIST(file), STATUS_TB_ALL);
	deleteEventTBC(DCI_TBC(file));
	acq200_releaseDCI(file);
	return 0;
}

static ssize_t status2_tb_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
/** output last tblock as string data. Release previous block .. 
 * TB is NOT marked as in_phase (busy).
 */
{
	struct TblockConsumer *tbc = DCI_TBC(file);
	struct TblockListElement *tle;
	char lbuf[80];
	int rc;

	if (tbc->c.tle){
		spin_lock(&DG->tbc_regular.lock);
		acq200_phase_release_tblock_entry(tbc->c.tle);
		spin_unlock(&DG->tbc_regular.lock);	
	}
		
	wait_event_interruptible(tbc->waitq, !list_empty(&tbc->tle_q));

	if (list_empty(&tbc->tle_q)){
		return -EINTR;
	}

	tle = TBLE_LIST_ENTRY(tbc->tle_q.next);

	/* phase_sample_start: allows clients to sync to samples,
         * in case of late joining, or missed tblocks 
         */
	rc = snprintf(lbuf, len, "%03d %d\n", 
		      tle->tblock->iblock, tle->phase_sample_start);

	if (tbc->backlog){
		--tbc->backlog;
	}
	spin_lock(&DG->tbc_regular.lock);

	if ((DCI(file)->flags&DCI_FLAGS_NORELEASE_ON_READ) == 0){
		acq200_phase_release_tblock_entry(tle);
	}else{
		list_move_tail(&tle->list, DCI_LIST(file));
	}
	spin_unlock(&DG->tbc_regular.lock);	

	COPY_TO_USER(buf, lbuf, rc);
	return rc;
}


/**
 *  create acq200dmafs
 */

#define DMAFS_MAGIC 0xd1acd10d
#define AIAFS_MAGIC 0xd1acd10a

#define TD_SZ  (sizeof(struct tree_descr))

/** files: head, channels * n, xxl, xxp, xx, tail */
/* then add a bit more for specials ... */
#define NODE_HEADROOM	12
#define MY_NODES(nc)    (1+(nc)+NODE_HEADROOM+1)
#define MY_FILES_SZ(nc) (MY_NODES(nc)*TD_SZ)
#define MY_IDENTS_SZ(nc) (MY_NODES(nc)*sizeof(short))

typedef char CHNAME[4];

static struct FS_DESCR {
	int nchannels;
	struct tree_descr *files;
	short *idents;
	CHNAME *chnames;
} S_AIFD, S_DFD;


#ifndef container_of
#error container_of NOT DEFINED see kernel.h
#endif


static inline int updateFiles(
	struct FS_DESCR* fsd,
	struct tree_descr *src,
	short ident,
	int ifile)
{
	struct tree_descr *_files = fsd->files;
	memcpy(&_files[ifile], src, TD_SZ);
	fsd->idents[ifile] = ident;
	return ifile + 1;
}



static int ai_getattr (struct vfsmount *mnt, struct dentry *d, struct kstat *k)
{
	struct inode *inode = d->d_inode;
	int was;

	generic_fillattr(inode, k);
	was = k->size;
	k->size = update_inode_stats(inode);
	k->mtime = inode->i_mtime;
	dbg(1, "size was %d set to %d", was, (int)k->size);
	return 0;
}



/* this is a straight crib from libfs.c.
 * not good, but it was proving _really_ difficult to get to the inodes!
 */


/*
 * Copyright (C) 2000 Linus Torvalds.
 *               2000 Transmeta Corp.
 * aops copied from ramfs.
 */
static int bigbuf_readpage(struct file *file, struct page * page)
{
	dbg(1, "Hello page %p %s", 
	    page, PageUptodate(page)? "UPTODATE": "not uptodate");
	dbg(1, "f:%08x count:%d mapc:%d mapping:%p index:%ld",
	    (unsigned)page->flags, 
	    atomic_read(&page->_count),
	    atomic_read(&page->_mapcount),
	    page->mapping, page->index);

	return simple_readpage(file, page);
	return 0;
}

static struct address_space_operations bigbuf_aops = {
	.readpage = bigbuf_readpage,
};

#include <linux/backing-dev.h>

static struct backing_dev_info ramfs_backing_dev_info = {
	.ra_pages	= 0,	/* No readahead */

};

int ai_simple_fill_super(
	struct super_block *s, int magic, struct FS_DESCR* fsd)
{
	static struct super_operations s_ops = {.statfs = simple_statfs};
	static struct inode_operations inops = {
		.getattr = ai_getattr
	};
	struct inode *inode;
	struct dentry *root;
	struct dentry *dentry;
	int i;
	struct tree_descr *files = fsd->files;

	s->s_blocksize = PAGE_CACHE_SIZE;
	s->s_blocksize_bits = PAGE_CACHE_SHIFT;
	s->s_magic = magic;
	s->s_op = &s_ops;

	inode = new_inode(s);
	if (!inode)
		return -ENOMEM;
	inode->i_mode = S_IFDIR | 0755;
	inode->i_uid = inode->i_gid = 0;
	inode->i_blkbits = blksize_bits(PAGE_CACHE_SIZE);
	inode->i_blocks = 0;
	inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;
	inode->i_op = &simple_dir_inode_operations;
	inode->i_fop = &simple_dir_operations;
	root = d_alloc_root(inode);
	if (!root) {
		iput(inode);
		return -ENOMEM;
	}
	for (i = 0; !files->name || files->name[0]; i++, files++) {
		struct qstr name;
		if (!files->name)
			continue;
		name.name = files->name;
		name.len = strlen(name.name);
		name.hash = full_name_hash(name.name, name.len);
		dentry = d_alloc(root, &name);
		if (!dentry)
			goto out;
		inode = new_inode(s);
		if (!inode)
			goto out;
		inode->i_mode = S_IFREG | files->mode;
		inode->i_uid = inode->i_gid = 0;
		inode->i_blkbits = blksize_bits(PAGE_CACHE_SIZE);
		inode->i_blocks = 0;
		inode->i_atime = inode->i_mtime = 
			inode->i_ctime = CURRENT_TIME;
		inode->i_op = &inops;
		inode->i_fop = files->ops;
		inode->i_ino = i;
		inode->i_private = (void*)(unsigned)(fsd->idents[i]);
		inode->i_mapping->a_ops = &bigbuf_aops;
		inode->i_mapping->backing_dev_info = &ramfs_backing_dev_info;
		d_add(dentry, inode);
	}
	s->s_root = root;
	return 0;
out:
	d_genocide(root);
	dput(root);
	return -ENOMEM;
}
/** crib ends */



static inline size_t fbLen(struct FunctionBuf* fb)
{
	return fb->p_end - fb->p_start;
}
static inline size_t fbFree(struct FunctionBuf* fb)
{
	return FB_LIMIT - fbLen(fb);
}

static void *fbCursor(struct FunctionBuf* fb, int offset_bytes)
{
	return fb->p_start + offset_bytes;
}

static int format__open(struct inode *inode, struct file *filp)
{
	SET_FB(filp, &fb_format);

	if (FB(filp)->p_start == 0){
		FB(filp)->p_start = kmalloc(FB_LIMIT, GFP_KERNEL);
		FB(filp)->p_end = FB(filp)->p_start;
	}

	if ((filp->f_mode & FMODE_WRITE) != 0){
		/* truncate on write */
		FB(filp)->p_end = FB(filp)->p_start; 

		filp->f_dentry->d_inode->i_size = 0;
	}
	return 0;
}

static ssize_t format__write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	count = min(count, fbFree(FB(filp)));

	if (count == 0){
		return -EFBIG;
	}
	if (copy_from_user(fbCursor(FB(filp), *offset), buf, count)){
		return -EFAULT;
	}

	FB(filp)->p_end += count;
	*offset += count;
	filp->f_dentry->d_inode->i_size += count;

	return count;
}

static ssize_t format__read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	size_t fb_left;
	if (*offset >= fbLen(FB(filp))){
		return 0;
	}

	fb_left = fbLen(FB(filp)) - *offset;

	count = min(count, fb_left);

	if (count > 0){
		COPY_TO_USER(buf, fbCursor(FB(filp), *offset), count);
		*offset += count;
	}
	return count;
}

static int format__mmap(
	struct file *filp, struct vm_area_struct *vma)
{
	return io_remap_pfn_range( 
		vma, vma->vm_start, 
		__phys_to_pfn(virt_to_phys(FB(filp)->p_start)), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
	);
}
static int format__release(
        struct inode *inode, struct file *file)
{
	inode->i_size = fbLen(FB(file));
	inode->i_mtime = CURRENT_TIME;
	return 0;
}




static int AI_fs_fill_super (
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations format_ops = {
		.open = format__open,
		.read = format__read,
		.write = format__write,
		.mmap = format__mmap,
		.release = format__release
	};


	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};	
	int tcount = 0;
	struct tree_descr src;
	int ch;

	tcount = updateFiles(&S_AIFD, &front, 0, tcount);

	src.mode = S_IRUGO;
	src.ops = &fifo_AIfs_bigbuf_data_fops;

	for (ch = 0; ch != S_AIFD.nchannels; ++ch){
		src.name = S_AIFD.chnames[ch];
		tcount = updateFiles(&S_AIFD, &src, 
				     ch + 1 + BIGBUF_CHANNEL_DATA_DEVICE,
				     tcount);
	}

	src.name = "XXL";
	src.ops = &fifo_bigbuf_xxl_fops;
	tcount = updateFiles(&S_AIFD, &src, 
			     BIGBUF_DATA_DEVICE_XXL,
			     tcount);

	src.name = "XXP";
	src.ops = &fifo_bigbuf_xxp_fops;
	tcount = updateFiles(&S_AIFD, &src, 
			     BIGBUF_DATA_DEVICE_XXP,
			     tcount);

	src.name = "XX";
	src.ops = &fifo_bigbuf_xxp_fops;
	tcount = updateFiles(&S_AIFD, &src, 
			     BIGBUF_DATA_DEVICE_XXP,
			     tcount);

	src.mode = S_IRUGO|S_IWUGO;

	src.name = "format";	/* User Data File */
	src.ops = &format_ops;
	tcount = updateFiles(&S_AIFD, &src, BIGBUF_DATA_DEVICE_FMT, tcount);

	tcount = updateFiles(&S_AIFD, &backstop, 0, tcount);
	return ai_simple_fill_super(sb, AIAFS_MAGIC, &S_AIFD);
}


static int dma_tblock_struct_open(
	struct inode *inode, struct file *file,	char *begin, int len)
{
	struct FunctionBuf* fb = 
		kmalloc(sizeof(struct FunctionBuf), GFP_KERNEL);
	
	fb->p_start = begin;
	fb->p_end   = begin + len;
	SET_FB(file, fb);
	return 0;
}

static int dma_tblock_offset_open (struct inode *inode, struct file *file)
{	
	if (DG->bigbuf.tblock_offset_lut){
		return dma_tblock_struct_open(
			inode, file, 
			(char*)DG->bigbuf.tblock_offset_lut, 
			DG->bigbuf.tblock_offset_lut_len*sizeof(short));
	}else{
		return -1;
	}			
}

static int dma_tblock_event_open (struct inode *inode, struct file *file)
{
	if (DG->bigbuf.tblock_event_table){
		return dma_tblock_struct_open(
			inode, file,
			(char*)DG->bigbuf.tblock_event_table,
			TBLOCK_EVENT_SZ);
	}else{
		return -1;
	}
}



static int dma_tblock_struct_release (struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}
 

static int dma_fs_fill_super (struct super_block *sb, void *data, int silent)
{
	static struct file_operations dma_ch_ops = {
		.open = dma_ch_open,
		.read = acq200_fifo_bigbuf_read,
		.release = dma_ch_release
	};
	static struct file_operations dma_xxp_ops = {
		.open = dma_xx_open,
		.read = dma_xxp_read,
		.release = dma_xx_release
	};
	static struct file_operations dma_xxl_ops = {
		.open = dma_xx_open,
		.read = dma_xxl_read,
		.release = dma_xx_release
	};
	static struct file_operations dma_tb_ops = {
		.open = dma_tb_open,
		.read = dma_tb_read,
		.release = dma_tb_release,
		.poll = tb_poll
	};
	static struct file_operations dma_tbstatus_ops = {
		.open = dma_tb_open,
		.read = status_tb_read,
		.write = status_tb_write,
		.release = dma_tb_release,
		.poll = tb_poll
	};
	static struct file_operations dma_tbstat_ev_ops = {
		.open = dma_tb_evopen,
		.read = status_tb_evread,
		.write = status_tb_evwrite,
		.release = dma_tb_evrelease,
		.poll = tb_poll
	};
	static struct file_operations dma_tbstatus2_ops = {
		.open = dma_tb_open,
		.read = status2_tb_read,
		.release = dma_tb_release,
		.poll = tb_poll
	};

	static struct file_operations dma_tblock_event_ops = {
		.open = dma_tblock_event_open,
		.read  = format__read,
		.release = dma_tblock_struct_release
	};
	static struct file_operations dma_tblock_offset_ops = {
		.open = dma_tblock_offset_open,
		.read = format__read,
		.release = dma_tblock_struct_release
	};	
	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};	
	int tcount = 0;
	struct tree_descr src;
	int ch;

	tcount = updateFiles(&S_DFD, &front, 0, tcount);

	src.mode = S_IRUGO;

	for (ch = 0, src.ops = &dma_ch_ops; ch != S_DFD.nchannels; ++ch){
		src.name = S_DFD.chnames[ch];
		tcount = updateFiles(&S_DFD, &src, 
				     ch + 1 + BIGBUF_CHANNEL_DATA_DEVICE,
				     tcount);
	}

	src.name = "XXL";
	src.ops = &dma_xxl_ops;
	tcount = updateFiles(&S_DFD, &src, 
			     BIGBUF_DATA_DEVICE_XXL,
			     tcount);

	src.name = "XXP";
	src.ops = &dma_xxp_ops;
	tcount = updateFiles(&S_DFD, &src, 
			     BIGBUF_DATA_DEVICE_XXP,
			     tcount);

	src.name = "tblock";
	src.ops = &dma_tb_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	/* the rest are all RW to allow buffer reserve/recycle */      
	/* NB: tbstat2 doesn't actually reserver, W is chucked */
	src.mode = S_IRUGO|S_IWUGO;

	src.name = "tbstat";
	src.ops = &dma_tbstatus_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	src.name = "tbstat2";
	src.ops = &dma_tbstatus2_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	src.name = "tbstat_ev";
	src.ops = &dma_tbstat_ev_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	src.name = "tblock_offset_lut";
	src.ops = &dma_tblock_offset_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	src.name = "tblock_event_table";
	src.ops = &dma_tblock_event_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	tcount = updateFiles(&S_DFD, &backstop, 0, tcount);
	return simple_fill_super(sb, DMAFS_MAGIC, S_DFD.files);
}


static int AI_fs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(fst, flags, data, AI_fs_fill_super, mnt);
}


static int dma_fs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(fst, flags, data, dma_fs_fill_super, mnt);
}


static struct file_system_type dma_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq200_dmafs",
	.get_sb		= dma_fs_get_super,
	.kill_sb	= kill_litter_super,
};


static struct file_system_type AI_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq200_AIfs",
	.get_sb		= AI_fs_get_super,
	.kill_sb	= kill_litter_super,
};


static void create_FSD(int nchannels, struct FS_DESCR* fsd)
{
	int ch;

	fsd->files = kmalloc(MY_FILES_SZ(nchannels), GFP_KERNEL);
	fsd->idents = kmalloc(MY_IDENTS_SZ(nchannels), GFP_KERNEL);
	fsd->chnames = kmalloc(nchannels*sizeof(CHNAME), GFP_KERNEL);
	fsd->nchannels = nchannels;

	for (ch = 0; ch != nchannels; ++ch){
		sprintf(fsd->chnames[ch], "%02d", ch+1);
	}
}

static void free_FSD(struct FS_DESCR* fsd)
{
	fsd->nchannels = 0;
	kfree(fsd->chnames);
	kfree(fsd->files);
	kfree(fsd->idents);
}

int acq200_fifo_create_AIfs(struct device* dev, int nchannels)
{
	int rc;

	create_FSD(nchannels, &S_AIFD);
	rc = register_filesystem(&AI_fs_type);

	if (rc == 0){
		create_FSD(nchannels, &S_DFD);
		rc = register_filesystem(&dma_fs_type);
	}
	return rc;
}

struct DataConsumerBuffer *acq200_createDCB(void) 
{
	struct DataConsumerBuffer *dcb =  
		kmalloc(sizeof(struct DataConsumerBuffer), GFP_KERNEL);
	assert(dcb);
	memset(dcb, 0, sizeof(struct DataConsumerBuffer));
	
	init_waitqueue_head(&dcb->waitq);
	u32rb_init(&dcb->rb, DG->dcb.dcb_max);
	dcb->handle = dma_map_single( 
		DG->dev, va_buf( DG ), len_buf( DG ), DMA_FROM_DEVICE);

	return dcb;
}

void acq200_deleteDCB(struct DataConsumerBuffer *dcb)
{
	dma_unmap_single(DG->dev, DG->dma_handle, len_buf(DG),DMA_FROM_DEVICE);
	u32rb_destroy(&dcb->rb);
	kfree(dcb);
}

int acq200_fifo_destroy_AIfs(void)
{
	unregister_filesystem(&AI_fs_type);
        unregister_filesystem(&dma_fs_type);
	free_FSD(&S_DFD);
	free_FSD(&S_AIFD);
	return 0;
}

int acq200_addDataConsumer(struct DataConsumerBuffer *dcb)
{
	struct DataConsumerBuffer *test;

	spin_lock(&DG->dcb.clients.lock);
	
	/* refuse to add twice .. */
	list_for_each_entry(test, &DG->dcb.clients.list, list){
		if (test == dcb){
			goto unlock;
		}
	}
	list_add_tail(&dcb->list, &DG->dcb.clients.list);

unlock:
	spin_unlock(&DG->dcb.clients.lock);
	return 0;
}
int acq200_removeDataConsumer(struct DataConsumerBuffer *dcb)
{
	spin_lock(&DG->dcb.clients.lock);
	list_del(&dcb->list);
	spin_unlock(&DG->dcb.clients.lock);
	return 0;
}

static void set_xx_valid(void *not_used) 
{
	XX_valid = 1;
}
void acq200_fifo_bigbuf_fops_init(void)
{
	static struct Hookup xx_valid_hook = {
		.the_hook = set_xx_valid
	};
	acq200_add_start_of_shot_hook(&xx_valid_hook);
}

EXPORT_SYMBOL_GPL(acq200_fifo_bigbuf_read_bbrp);
EXPORT_SYMBOL_GPL(acq200_fifo_bigbuf_xxX_read);
EXPORT_SYMBOL_GPL(acq200_initBBRP_using_phase);
EXPORT_SYMBOL_GPL(acq200_initDCI);
EXPORT_SYMBOL_GPL(acq200_releaseDCI);

EXPORT_SYMBOL_GPL(acq200_fifo_bigbuf_xx_open);
EXPORT_SYMBOL_GPL(acq200_fifo_bigbuf_xx_release);

EXPORT_SYMBOL_GPL(acq200_createDCB);
EXPORT_SYMBOL_GPL(acq200_deleteDCB);
EXPORT_SYMBOL_GPL(acq200_addDataConsumer);
EXPORT_SYMBOL_GPL(acq200_removeDataConsumer);
