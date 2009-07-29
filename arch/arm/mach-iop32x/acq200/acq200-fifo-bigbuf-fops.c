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

#include "acq200-mu-app.h"

#if 1
#include <linux/blkdev.h>
#include <linux/pagemap.h>
#include <linux/swap.h>      /* mark_page_accessed() */
#endif


void acq200_initDCI(struct file *file, int lchannel)
{
	file->private_data = kmalloc(DCI_SZ, GFP_KERNEL);
	memset(file->private_data, 0, DCI_SZ);

	DCI(file)->lchan = lchannel;
	if (lchannel != 0){
		DCI(file)->pchan = acq200_lookup_pchan(lchannel);
		DCI(file)->ssize = CSIZE;
	}else{
		DCI(file)->ssize = RSIZE;
	}
}



void acq200_releaseDCI(struct file *file)
{
	kfree(file->private_data);
}

static struct TblockConsumer *newTBC(unsigned backlog_limit)
{
	struct TblockConsumer *tbc = 
		kzalloc(sizeof(struct TblockConsumer), GFP_KERNEL);

	tbc->backlog_limit = backlog_limit;	
	init_waitqueue_head(&tbc->waitq);
        INIT_LIST_HEAD(&tbc->tle_q);

	spin_lock(&DG->tbc.lock);
	list_add_tail(&tbc->list, &DG->tbc.clients);
	spin_unlock(&DG->tbc.lock);
	
	return tbc;
}
static void deleteTBC(struct TblockConsumer *tbc)
{
	struct TblockListElement *cursor, *tmp;
	int tblock_backlog = 0;

	spin_lock(&DG->tbc.lock);
	list_del(&tbc->list);
	spin_unlock(&DG->tbc.lock);

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


#define NOSAMPLES 0xffffffff


static unsigned update_inode_stats(struct inode *inode)
{
	int ident = (int)inode->i_private;
	unsigned ssize;
	unsigned samples = 0;

	switch(ident){
	case BIGBUF_DATA_DEVICE_XXP:
	case BIGBUF_DATA_DEVICE_XXL:
		ssize = sample_size();
		samples = SAMPLES;
		break;
	default:
		if ((ident&BIGBUF_CHANNEL_DATA_DEVICE) != 0){
			int lchannel = ((unsigned)inode->i_private) & 0x7f;
			if (acq200_lchanEnabled(lchannel)){ 
				ssize = CSIZE;
				samples = DG->getChannelNumSamples(	
					acq200_lookup_pchan(lchannel));
			}else{
				ssize = 0;
			}
		}else{
			ssize = 0;
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

	if (!tb_tmp) return -1;

	dbg(1,"transform(%p %p %d %d)", tb_tmp, tb_va, nwords, NCHAN); 

	DG->bigbuf.tblocks.transform(tb_tmp, tb_va, nwords, NCHAN);
	tblock_lock(tblock);
	if ((t_flags&TF_INPLACE) == 0){
		DG->bigbuf.tblocks.blt(tb_va, tb_tmp, nwords);
	}

	if ((t_flags&TF_RESULT_IS_RAW) == 0){
		tblock->extract = tblock_cooked_extractor;
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
	tblock->extract = tblock_cooked_extractor;
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
	bbrp->tblock = tble->tblock;
	bbrp->my_samples_reqlen = min(my_samples_left, req_samples);
	bbrp->block_off_sample = block_off_sample + tble->tblock_sample_start;
	bbrp->samples_left_in_block = samples_left_in_block;
	bbrp->tblock_samples = bbrp->tblock->length/sample_size()/stride;

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
	}else if (channel >= NCHAN ){
		RETURN(0);
	}else if ( bbrp->my_samples_reqlen == 0 ){
		RETURN(0);
	}else{
		int extract_words = bbrp->my_samples_reqlen;

		
		if (DCI(file)->ssize > sizeof(short)){
			/* extractor works in shorts */
			extract_words *= DCI(file)->ssize;
			extract_words /= sizeof(short);
		}

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

	dbg(1, "address 0x%08lx", address);

	if (offset > bbrp->my_samples_reqlen){
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
		unsigned choffset = DCI(vma->vm_file)->lchan * 
			bbrp->tblock_samples * sizeof(short);
		unsigned pa = PA_TBLOCK(bbrp->tblock) + choffset + offset; 

		dbg(1, "offset:%6lu pa:%08x va:%p",(unsigned long)offset,
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
	int bblock_samples = this->length/NCHAN/ssize;
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

	dbg(2, "call initBBRP");
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
	int bblock_samples = this->length/sample_size()/stride;
	short* bblock_base = (short*)(va_buf(DG) + this->offset);
	int cplen;
	int maxwords;
	int offsetw = offsam * NCHAN * stride;

	dbg(1, "channel %2d offsam %08x offsetw %08x maxbuf %x", 
	    channel, offsam, offsetw, maxbuf);

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




static int dma_xx_extractor(
	struct TBLOCK *this,
	short* ubuf, 
	int maxbuf, 
	int channel, 
	int offsam, 
	int stride)
{
	int bblock_samples = this->length/NCHAN/SSZ;
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
	acq200_initDCI(file, 0);
	DCI(file)->extract = tblock_rawxx_extractor;
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
	fillLUT(LUT);	
	DCI(file)->memcpy = lut_memcpy;
	return acq200_fifo_bigbuf_xx_open(inode, file);
}



static ssize_t acq200_fifo_bigbuf_xxl_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset)
{
	return fifo_bigbuf_xxX_read(file, buf, len, offset);
}




/*
 * PUBLIC ENTRY POINTS
 */

static struct file_operations fifo_bigbuf_xxp_fops = {
	.open = acq200_fifo_bigbuf_xx_open,
	.read = acq200_fifo_bigbuf_xxp_read,
	.release = acq200_fifo_bigbuf_xx_release
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
	int bblock_samples = this->length/NCHAN/sizeof(short);
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
	acq200_initDCI(file, 0);
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
	acq200_initDCI(file, 0);
	DCI_TBC(file) = newTBC(DG->bigbuf.tblocks.nblocks - 10);
	DCI(file)->extract = dma_xx_extractor;
	return 0;
}

static int dma_tb_release (
	struct inode *inode, struct file *file)
{
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
		assert(tbc->c.tle->tblock->length >= tbc->c.cursor);

		headroom = tbc->c.tle->tblock->length - tbc->c.cursor;

		dbg(1, "file %p tbc->c.tle %p headroom %d %s", 
		    file, 
		    tbc->c.tle, headroom, headroom==0? "release": "hold");

		if (headroom == 0){
			spin_lock(&DG->tbc.lock);
			acq200_phase_release_tblock_entry(tbc->c.tle);
			spin_unlock(&DG->tbc.lock);	
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
		headroom = tbc->c.tle->tblock->length;
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

	if (len < 8){
		rc = snprintf(lbuf, len, "%3d\n", tle->tblock->iblock);
	}else{
		rc = snprintf(lbuf, min(sizeof(lbuf), len), 
			"tblock %3d off 0x%08x phys:0x%08x len %d scount %d\n",
			tle->tblock->iblock, 
			tle->tblock->offset,
			pa_buf(DG) + tle->tblock->offset,
			tle->tblock->length,
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


static ssize_t status_tb_write(
	struct file *file, const char *buf, size_t len, loff_t *offset)
{
//	struct TblockConsumer *tbc = DCI_TBC(file);
	struct TblockListElement *tle = DCI(file)->tle_current;
	
	/** @todo should check iblock on input; */

	/** lazy init - first write sets mode flag */
	DCI(file)->flags |= DCI_FLAGS_NORELEASE_ON_READ;

	if (tle != 0){
		spin_lock(&DG->tbc.lock);
		acq200_phase_release_tblock_entry(tle);
		DCI(file)->tle_current = 0;
		spin_unlock(&DG->tbc.lock);			
	}

	return len;
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
		spin_lock(&DG->tbc.lock);
		acq200_phase_release_tblock_entry(tbc->c.tle);
		spin_unlock(&DG->tbc.lock);	
	}
		
	wait_event_interruptible(tbc->waitq, !list_empty(&tbc->tle_q));

	if (list_empty(&tbc->tle_q)){
		return -EINTR;
	}

	tle = TBLE_LIST_ENTRY(tbc->tle_q.next);

	rc = snprintf(lbuf, len, "%03d\n", tle->tblock->iblock);

	if (tbc->backlog){
		--tbc->backlog;
	}
	spin_lock(&DG->tbc.lock);

	if ((DCI(file)->flags&DCI_FLAGS_NORELEASE_ON_READ) == 0){
		acq200_phase_release_tblock_entry(tle);
	}else{
		DCI(file)->tle_current = tle;
	}
	spin_unlock(&DG->tbc.lock);	

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
/* the add a bit more for specials ... */
#define NODE_HEADROOM	10	
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


static int AI_fs_fill_super (
	struct super_block *sb, void *data, int silent)
{
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

	tcount = updateFiles(&S_AIFD, &backstop, 0, tcount);
	return ai_simple_fill_super(sb, AIAFS_MAGIC, &S_AIFD);
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
	static struct file_operations dma_tbstatus2_ops = {
		.open = dma_tb_open,
		.read = status2_tb_read,
		.release = dma_tb_release,
		.poll = tb_poll
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

	src.name = "tbstat";
	src.ops = &dma_tbstatus_ops;
	tcount = updateFiles(&S_DFD, &src, 0, tcount);

	src.name = "tbstat2";
	src.ops = &dma_tbstatus2_ops;
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
	spin_lock(&DG->dcb.lock);
	list_add_tail(&dcb->list, &DG->dcb.clients);
	spin_unlock(&DG->dcb.lock);
	return 0;
}
int acq200_removeDataConsumer(struct DataConsumerBuffer *dcb)
{
	spin_lock(&DG->dcb.lock);
	list_del(&dcb->list);
	spin_unlock(&DG->dcb.lock);
	return 0;
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
