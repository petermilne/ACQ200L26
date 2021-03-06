#include <linux/kernel.h>
#include <linux/interrupt.h>

#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#ifdef PGMCOMOUT263
#include <asm-arm/proc-armv/cache.h>
#endif

#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#define DMA_BLOCK_LEN 0xdeadbeef  /* FIXME PLEASE - not used here */

#include "acqX00-port.h"
#include "acq200-fifo-local.h"
#include "acq200_debug.h"

#include "acq200-fifo-tblock.h"

#include <asm-arm/arch-iop32x/acq200.h>


#include <linux/moduleparam.h>


int acq200_tblock_debug = 0;
module_param(acq200_tblock_debug, int, 0664);

int fix_event_share_tblock;
module_param(fix_event_share_tblock, int, 0644);

#define DBG if (acq200_tblock_debug) dbg

void clear_tblock_event_table(void);

int tblock_raw_extractor(
	struct TBLOCK *this,
	short* ubuf, int maxbuf, 
	int channel, int offset, int stride)
{
	short* bblock_base = (short*)(
		va_buf(DG) + this->offset + channel*sizeof(short));
	int cplen;

	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);

	stride *= NCHAN;	
	offset *= NCHAN;

	for(cplen = 0; cplen < maxbuf && offset < this->tb_length; ++cplen){
		COPY_TO_USER(ubuf+cplen, bblock_base + offset, sizeof(short));
		offset += stride;
	}

	DBG(1, "returns cplen %d", cplen);
	return cplen;
}
int tblock_raw_extractor32(
	struct TBLOCK *this,
	short* ubuf, int maxbuf, 
	int channel, int offset, int stride)
{
	void* bblock_base = (void*)(
		va_buf(DG) + this->offset + channel*sizeof(u32));
	unsigned *usbuf = (unsigned *)ubuf;
	int cplen;


/* @@todo offset is in shorts - does this affect price of eggs ?? */
	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);

	stride *= NCHAN * 2;		/* two shorts per sample */
	offset *= NCHAN * 2;

	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);

	for(cplen = 0; cplen < maxbuf && offset < this->tb_length; ++cplen){
		COPY_TO_USER(usbuf+cplen, bblock_base + offset, sizeof(u32));
		offset += stride;
	}

	DBG(1, "returns cplen %d", cplen);
	return cplen;
}


int getChannelData(struct TBLOCK* tb, void **base, int channel, int offset)
{
/* DO NOT use tb_length : channels are spread over the whole block ! */
	/* @@todo experimenting */
	//int bblock_samples = TBLOCK_LEN(DG)/NCHAN/sizeof(short);
	int bblock_samples = tb->tb_length/NCHAN/sizeof(short);
	short* bblock_base = (short*)(va_buf(DG) + tb->offset + 
				channel*bblock_samples*sizeof(short));

	*base = bblock_base + offset;
	return bblock_samples;
}
int getChannelData32(struct TBLOCK* tb, void **base, int channel, int offset)
{
	int bblock_samples = TBLOCK_LEN(DG)/NCHAN/sizeof(u32);
	u32* bblock_base = (u32*)(va_buf(DG) + tb->offset + 
				channel*bblock_samples*sizeof(u32));

	DBG(1, "offset %d setting base %p", offset, bblock_base+offset);

	*base = bblock_base + offset;
	return bblock_samples;
}

int tblock_cooked_extractor(
	struct TBLOCK *this,
	short* ubuf, int maxbuf, 
	int channel, int offset, int stride)
{

	short* bblock_base;
	int bblock_samples = DG->bigbuf.tblocks.getChannelData(
		this, (void **)&bblock_base, channel, offset);

	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);

	maxbuf = min(maxbuf, bblock_samples-offset);
	if (stride == 1){
		COPY_TO_USER(ubuf, bblock_base, maxbuf*2);
	}else{
		int cplen;

		DBG(1, "c:%d stride:%d", channel, stride);

		for(cplen = 0; cplen < maxbuf && offset < this->tb_length; 
		    ++cplen, bblock_base += stride){
			COPY_TO_USER(ubuf+cplen, bblock_base, sizeof(short));
		}		
	}

	DBG(1, "returns maxbuf %d", maxbuf);
	return maxbuf;
}

int tblock_cooked_extractor32(
	struct TBLOCK *this,
	short* ubuf, int maxbuf, 
	int channel, int offset, int stride)
{
	unsigned *usbuf = (unsigned *)ubuf;
	short* bblock_base;
	int bblock_samples = DG->bigbuf.tblocks.getChannelData(
		this, (void**)&bblock_base, channel, offset);

	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);
/* @@todo offset is in shorts - does this affect price of eggs ?? */

	maxbuf = min(maxbuf, bblock_samples-offset);
	if (stride == 1){
		COPY_TO_USER(ubuf, bblock_base, maxbuf*sizeof(u32));
	}else{
		int cplen;

		DBG(1, "c:%d stride:%d", channel, stride);

		for(cplen = 0; cplen < maxbuf && offset < this->tb_length; 
		    ++cplen, bblock_base += stride){
			COPY_TO_USER(usbuf+cplen, bblock_base, sizeof(u32));
		}		
	}

	DBG(1, "returns maxbuf %d", maxbuf);
	return maxbuf * sizeof(u32)/sizeof(short);
}
int tblock_raw_filler(
	struct TBLOCK *this,
	/* const */  short* ubuf, int maxbuf, 
	int channel, int offset, int stride)
{
	short* bblock_base = (short*)(
		va_buf(DG) + this->offset + channel*sizeof(short));
	int cplen;

	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);

	stride *= NCHAN;	
	offset *= NCHAN;

	for(cplen = 0; cplen < maxbuf && offset < this->tb_length; ++cplen){
		COPY_FROM_USER(bblock_base+offset, ubuf+cplen, sizeof(short));
		offset += stride;
	}

	DBG(1, "returns cplen %d", cplen);
	return cplen;
}
int tblock_cooked_filler(
	struct TBLOCK *this,
	/* const */ short* ubuf, int maxbuf, 
	int channel, int offset, int stride)
{
	int bblock_samples = this->tb_length/NCHAN/sizeof(short);
	short* bblock_base = (short*)(
		va_buf(DG) + this->offset + 
		channel*bblock_samples*sizeof(short));

	DBG(1, "channel %2d offset %08x maxbuf %x", channel, offset, maxbuf);

	maxbuf = min(maxbuf, bblock_samples-offset);
	COPY_FROM_USER(bblock_base + offset, ubuf, maxbuf*2);

	DBG(1, "returns maxbuf %d", maxbuf);
	return maxbuf;
}

int tblock_lock(struct TBLOCK *this)
{
	return 0;
}
int tblock_unlock(struct TBLOCK *this)
{
	return 0;
}


void acq200_init_tblock_list(void)
{
	struct TBLOCK tb_temp = { };
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	int iblock;

	assert(tbl->the_tblocks);

/* @@todo here's where TBLOCK gets its length. 
 * A little meaningless, since no data yet.. */
	tb_temp.tb_length = tbl->blocklen;
	tb_temp.locked = 0;

	if (BTYPE_IS_AI(DG->btype)){
		if (capdef_get_word_size() == sizeof(u32)){
			tb_temp.extract = tblock_raw_extractor32;
		}else{
			tb_temp.extract = tblock_raw_extractor;
		}
		tb_temp.fill = tblock_raw_filler;
	}else{
		tb_temp.extract = tblock_cooked_extractor;
		tb_temp.fill = tblock_cooked_filler;
	}

	for (iblock = 0; iblock != tbl->nblocks; ++iblock){
		struct TBLOCK *tb_cursor = &tbl->the_tblocks[iblock];
		tb_temp.inode = tb_cursor->inode;
		tb_temp.iblock = iblock;
		tb_temp.offset = iblock * tbl->blocklen;
		memcpy(tb_cursor, &tb_temp, sizeof(tb_temp));
	}
}


int acq200_calc_max_tblocks(void)
{
	int max_tblock = len_buf(DG)/DG->bigbuf.tblocks.blocklen;
	if (max_tblock > ABS_MAX_TBLOCKS){
		max_tblock = ABS_MAX_TBLOCKS;
	}
	return max_tblock;
}
void build_tblock_list(void)
{
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	int kernel_blocklen;

	kernel_blocklen = acq200_get_tblock_resource(&DG->bigbuf.tblocks.tmp);

	if (kernel_blocklen <= tbl->blocklen){
		tbl->blocklen = kernel_blocklen;
	}

	if (DG->bigbuf.tblocks.blocklen == 0){
		err("BLOCKLEN == 0");
	}
	MAX_TBLOCK = acq200_calc_max_tblocks();

	DG->bigbuf.tblocks.the_tblocks = 
		kmalloc(sizeof(struct TBLOCK)*MAX_TBLOCK, GFP_KERNEL);

	DBG(1, "kmalloc(%d) => %p", sizeof(struct TBLOCK) * MAX_TBLOCK,
					    DG->bigbuf.tblocks.the_tblocks );
	acq200_init_tblock_list();
}


static void free_tblock_list(void)
{

	void *tmp = DG->bigbuf.tblocks.the_tblocks;

	DBG(1, "kfree(%p)", tmp );

	DG->bigbuf.tblocks.the_tblocks = 0;
	kfree(tmp);
}


static void tblock_clear(struct TBLOCK *tblock)
/** remove any state from tblock */
{
	tblock->locked = 0;
	tblock->touched = 0;
}

void acq200_empties_release_tblocks(void)
{
	struct BIGBUF *bb = &DG->bigbuf;
	unsigned long flags;
	
	spin_lock_irqsave(&bb->tb_list_lock, flags);
	list_splice_init(&bb->empty_tblocks, &bb->free_tblocks);
	spin_unlock_irqrestore(&bb->tb_list_lock, flags);	
}

void acq200_sort_free_tblocks(void)
{
	LIST_HEAD(free_tmp);
	struct list_head *free_tblocks = &DG->bigbuf.free_tblocks;
	struct TblockListElement* tle;
	struct TblockListElement* tmp;
	int ib_search = 0;

	if (acq200_tblock_debug > 3){
		return;				/* REMOVEME */
	}
	list_splice_init(free_tblocks, &free_tmp);

	while(!list_empty(&free_tmp)){
		int search_before = ib_search;

		if (ib_search >= MAX_TBLOCK){
			err("searching beyond the last tblock %d", MAX_TBLOCK);
			return;
		}

		list_for_each_entry_safe(tle, tmp, &free_tmp, list){
			if (tle->tblock->iblock == ib_search){
				list_move_tail(&tle->list, free_tblocks);
				++ib_search;
			}
		}
		
		if (ib_search == search_before && ib_search < MAX_TBLOCK){
			err("Missing block %d", ib_search++);
		}		
	}
}


void acq200_phase_release_tblock_entry(struct TblockListElement* tle)
{
	struct BIGBUF *bb = &DG->bigbuf;
	struct TBLOCK* tblock = tle->tblock;
	unsigned long flags;

	dbg(!acq200_tblock_debug, 
		"[%d] entry in_phase %d", tblock->iblock, TB_IN_PHASE(tblock));

	if (TB_IN_PHASE(tblock) <= 0){
		err("ERROR [%2d] in_phase %d", 
		    tblock->iblock, TB_IN_PHASE(tblock));
		return;
	}

	if (atomic_dec_and_test(&tblock->in_phase)){
		dbg(acq200_tblock_debug?1:5, "add_free %d old: next:%p prev:%p",
			 tblock->iblock, tle->list.next, tle->list.prev);

		tblock_clear(tblock);

		spin_lock_irqsave(&bb->tb_list_lock, flags);
		list_move_tail(&tle->list, &bb->free_tblocks);
		spin_unlock_irqrestore(&bb->tb_list_lock, flags);

		dbg(acq200_tblock_debug? 1: 5, 
			"[%d] ret  list_move_tail", tblock->iblock);
	}else{
		dbg(acq200_tblock_debug? 1: 5,
		 "[%d] shared tblock stash wrapper in pool", tblock->iblock);

		spin_lock_irqsave(&bb->tb_list_lock, flags);
		list_move_tail(&tle->list, &bb->pool_tblocks);
		spin_unlock_irqrestore(&bb->tb_list_lock, flags);
	}
}


void acq200_phase_release_tblocks(struct Phase* phase)
{
	struct TblockListElement* tle;
	struct TblockListElement* tmp;

	if (phase->tblocks.next == 0){
		return;
	}

	list_for_each_entry_safe(tle, tmp, &phase->tblocks, list){ 
		acq200_phase_release_tblock_entry(tle);
	}

	clear_tblock_event_table();
}

void build_tblock_offset_lut(void)
{
	int maxlut = len_buf(DG) >> TBLOCK_EVENT_ORDER;
	int il;
	short* tblock_offset_lut;

	if (TBLOCK_LEN(DG) < TBLOCK_EVENT_BLOCKSIZE){
		err("TBLOCK too small for offset lut");
		return;
	}
	info("lut length: %d", maxlut);

	tblock_offset_lut = kmalloc(sizeof(short)*maxlut, GFP_KERNEL);
	
	for (il = 0; il < maxlut; ++il){
		tblock_offset_lut[il] = TBLOCK_INDEX(il*TBLOCK_EVENT_BLOCKSIZE);
	}

	DG->bigbuf.tblock_offset_lut = tblock_offset_lut;
	DG->bigbuf.tblock_offset_lut_len = maxlut;
}

void build_tblock_event_table(void)
{
	DG->bigbuf.tblock_event_table = kzalloc(TBLOCK_EVENT_SZ, GFP_KERNEL);
}

void clear_tblock_event_table(void)
{
	if (DG->bigbuf.tblock_event_table){
		memset(DG->bigbuf.tblock_event_table, 0, TBLOCK_EVENT_SZ);
	}
}
void acq200_tblock_init_top(void)
{
	struct BIGBUF *bb = &DG->bigbuf;

	int iblock;

	spin_lock_init(&bb->tb_list_lock);
	INIT_LIST_HEAD(&bb->free_tblocks);
	INIT_LIST_HEAD(&bb->empty_tblocks);

	info("add %d tblocks",  DG->bigbuf.tblocks.nblocks);

	for (iblock = 0; iblock != DG->bigbuf.tblocks.nblocks; ++iblock){
		struct TblockListElement* tle = 
			kmalloc(sizeof(struct TblockListElement), GFP_KERNEL);
		memset(tle, 0, sizeof(struct TblockListElement));

		tle->tblock = &DG->bigbuf.tblocks.the_tblocks[iblock];
		list_add_tail(&tle->list, &bb->free_tblocks);
	}

	INIT_LIST_HEAD(&bb->pool_tblocks);

	for (iblock = 0; iblock != MAX_TBLOCK_POOL; ++iblock){
		struct TblockListElement* tle = 
			kmalloc(sizeof(struct TblockListElement), GFP_KERNEL);
		memset(tle, 0, sizeof(struct TblockListElement));

		list_add_tail(&tle->list, &bb->pool_tblocks);
	}
	build_tblock_offset_lut();
	build_tblock_event_table();	
}


void acq200_tblock_remove(void)
// @@todo - this wont work - tblocks not in free list
{
	struct TblockListElement* tle;

	list_for_each_entry(tle, &DG->bigbuf.free_tblocks, list){
//		atomic_clear_mask(-1, &tle->tblock->in_phase);
		DBG(1, "kfree %p", tle);
		kfree(tle);		
	}

	list_for_each_entry(tle, &DG->bigbuf.empty_tblocks, list){
//		atomic_clear_mask(-1, &tle->tblock->in_phase);
		DBG(1, "kfree %p", tle);
		kfree(tle);		
	}


	list_for_each_entry(tle, &DG->bigbuf.pool_tblocks, list){
//		atomic_clear_mask(-1, &tle->tblock->in_phase);
		DBG(1, "kfree %p", tle);
		kfree(tle);		
	}

	
	free_tblock_list();
}





static int gather_block(
	struct Phase* phase, 
	struct TblockListElement* tle, 
	int nblock)
{
	DBG(1, "01 Phase \"%s\" %p nblock %d", phase->name, phase, nblock);

	if (nblock == 0){
		tle->phase_sample_start = 0;

		if (TBLOCK_INDEX(phase->start_off) != tle->tblock->iblock){
			DBG(1, "98 TBLOCK_INDEX(phase->start_off) %d != %d",
			    TBLOCK_INDEX(phase->start_off), 
			    tle->tblock->iblock);
			return 0;
		}
		
		tle->tblock_sample_start = 
			TBLOCK_OFFSET(phase->start_off)/sample_size();
		DBG(1, "02 tbss set %d because phase->start_off is %d",
			tle->tblock_sample_start, phase->start_off);
	}else{
		TBLE* prev = TBLE_LIST_ENTRY(tle->list.prev);
		tle->phase_sample_start = 
			prev->phase_sample_start + prev->sample_count;

		tle->tblock_sample_start = 0;
	}


	tle->sample_count = 
		min(phase_num_samples(phase)- tle->phase_sample_start,
		    getTblockMaxSam(tle->tblock) - tle->tblock_sample_start);

	DBG(1, "97 sample_count %d = min( %d, %d )",
	    tle->sample_count,
	    phase_num_samples(phase)- tle->phase_sample_start,
	    getTblockMaxSam(tle->tblock) - tle->tblock_sample_start);

	DBG(1, "99 \"%s\" p:%p nb:%2d %s", phase->name,		
			phase, nblock, tle2string(tle));

	return tle->sample_count;
}


void acq200_phase_gather_tblocks(struct Phase* phase)
/*
 * collect and mark all the tblocks used by this phase
 */
{
	struct TblockListElement* tle;
	int nb = 0;
	int ngather;

	list_for_each_entry(tle, &phase->tblocks, list){
		DBG(1, "phase \"%s\" entry [%2d] %s", 
			phase->name,
			tle->tblock->iblock, tle2string(tle));

		ngather = gather_block(phase, tle, nb);
		if (ngather){
			phase->actual_samples += ngather;
			++nb;
		}
	}
}



int acq200_phase_tblocks_diag(struct Phase* phase, char *buf)
{
	struct TblockListElement* tle;
	int len = 0;
	unsigned tot_sample_count = 0;
	int nsprint;

	int ix = 0;
	

	list_for_each_entry(tle, &phase->tblocks, list){
		DBG(2, "entry %d %s", ix, tle2string(tle));
		DBG(2, "entry %d tle:%p tb:%p", ix, tle, tle->tblock);
		++ix;
	}

	DBG(2, "calling list_for_each_entry() %p", &phase->tblocks);
	list_for_each_entry(tle, &phase->tblocks, list){
		if (tle->tblock == 0){
			err("null tblock");
			break;
		}
		DBG(2, "entry %s", tle2string(tle));
		
		tot_sample_count += tle->sample_count;
		nsprint = sprintf(buf+len, "tblock:[%2d] %08x %s tsc:%8d %d\n",
			       tle->tblock->iblock,
				  tle->tblock->iblock*TBLOCK_LEN(DG),
			       tle2string(tle),
			       tot_sample_count,
			       atomic_read(&tle->tblock->in_phase));

		DBG(2, "%s", buf+len);
		len += nsprint;
		DBG(2, "len: %d next %p ", len, tle->list.next);
	}
	DBG(2, "all done now");

	return len;
}


static void phase_rollup_start_fixed(struct Phase* phase)
/** post-trigger: start at trigger point and work forwards, 
 *  discard unwanted blocks
 */

{
	enum phase_rollup_start_fixed { 
		PR_BEFORE, PR_DURING_FIRST, PR_DURING, PR_AFTER 
	} state = PR_BEFORE;

	int required_len = min(phase->required_len, phase->actual_len);
	int tblock_max_len = get_tblock_max_sam() * sample_size();
	unsigned tboff = TBLOCK_OFFSET(phase->start_off);
	unsigned tbix  = TBLOCK_INDEX(phase->start_off);

	struct TblockListElement* tle;
	struct TblockListElement* tmp;
	int len0 = 0;
	int len = 0;
	int nfull = 0;

	list_for_each_entry_safe(tle, tmp, &phase->tblocks, list){ 

		DBG(1, "stateB:%d iblock:%d len:%d",
		    state, tle->tblock->iblock, len);

		switch(state){
		case PR_BEFORE:
			if (tbix != tle->tblock->iblock){
				acq200_phase_release_tblock_entry(tle);
				break;
			}else{
				state = PR_DURING_FIRST; /** fall thru */
			}
		case PR_DURING_FIRST:
			len = len0 =  tblock_max_len - tboff;

			if (len0 >= required_len){
				int last_off = tboff + required_len;
					
				phase->end_off = 
					tle->tblock->offset + last_off;
				phase->actual_len = required_len;
				state = PR_AFTER;
			}else{
				state = PR_DURING;
			}
			break;
		case PR_DURING: {
			if ((len += tblock_max_len) >= required_len){
				int full_len = nfull * tblock_max_len;
				int last_off = required_len - len0 - full_len;

				phase->end_off = 
						tle->tblock->offset + last_off;
				phase->actual_len = required_len;
				state = PR_AFTER;
			}else{
				nfull++;
			}
			break;
		}
		case PR_AFTER:
			acq200_phase_release_tblock_entry(tle);
			break;			
		}


		DBG(1, "stateA:%d iblock:%d len:%d",
		    state, tle->tblock->iblock, len);

	}
}


static void phase_rollup_end_fixed(struct Phase* phase)

/** pre-trigger: start at trigger point and work back, discard unwanted blocks
 */

{
	enum phase_rollup_end_fixed { 
		PR_BEFORE, PR_DURING_FIRST, PR_DURING, PR_AFTER 
	} state = PR_BEFORE;

	int required_len = min(phase->required_len, phase->actual_len);
	unsigned tboff = TBLOCK_OFFSET(phase->end_off);
	unsigned tbix  = TBLOCK_INDEX(phase->end_off);

	struct TblockListElement* tle;

	int len0 = 0;
	int len = 0;
	int nfull = 0;

	list_for_each_entry_reverse(tle, &phase->tblocks, list){

		int tb_length = tle->tblock->tb_length;

		dbg(1, "stateB:%d iblock:%d len:%d",
		    state, tle->tblock->iblock, len);

		switch(state){
		case PR_BEFORE:
			if (tbix != tle->tblock->iblock){
				break;
			}else{
				state = PR_DURING_FIRST; /** fall thru */
			}
		case PR_DURING_FIRST:
			if ((len = len0 = tboff) >= required_len){
				int start_off = tboff - required_len;

				SET_PHASE_START_OFF(
					phase,				
					tle->tblock->offset + start_off);

				phase->actual_len = required_len;
				state = PR_AFTER;
				break;		
			}else{
				state = PR_DURING;
				break;			/** @:todo fall thru? */
			}
		case PR_DURING:
			if ((len += tb_length) >= required_len){
				int full_len = nfull * tb_length;
				int start_len = required_len - 
					(full_len + len0);
				int start_off = tb_length - start_len;

				SET_PHASE_START_OFF(phase, 
					tle->tblock->offset + start_off);
				phase->actual_len = required_len;
				state = PR_AFTER;
			}else{
				nfull++;
			}
			break;
		case PR_AFTER:
			break;
		}

		dbg(1, "stateA:%d iblock:%d len:%d",
		    state, tle->tblock->iblock, len);

	}

	phase_rollup_start_fixed(phase);
}

void acq200_phase_rollup_excess(struct Phase* phase)
/*
 * reduce phase size until actual_samples == required_len
 */
{
	dbg(1, "01:start 0x%08x len %d flags %d",  
	    phase->start_off, phase->actual_len, phase->flags);

	if ((phase->flags&PH_FIXED_AT_START) != 0){
		phase_rollup_start_fixed(phase);
	}
	if ((phase->flags&PH_FIXED_AT_END) != 0){
		phase_rollup_end_fixed(phase);
	}
	dbg(1, "99:start 0x%08x len %d",  phase->start_off, phase->actual_len);
}

TBLE* acq200_reserveFreeTblock(void)
/** reserve a TBLOCK
 *  return TBLE or 0 => no free tblocks available. 
 */
{
	struct BIGBUF *bb = &DG->bigbuf;
	unsigned long flags;
	TBLE* tble = 0;

	spin_lock_irqsave(&bb->tb_list_lock, flags);
	if (!list_empty(&bb->free_tblocks)){
		tble = TBLE_LIST_ENTRY(bb->free_tblocks.next);
		list_del(&tble->list);
	}
	spin_unlock_irqrestore(&bb->tb_list_lock, flags);

	return tble;
}

TBLE* acq200_reserveSpecificTblock(int iblock)
{
	struct BIGBUF *bb = &DG->bigbuf;
	unsigned long flags;
	TBLE* tble = 0;
	TBLE *n;
	TBLE *found = 0;

	spin_lock_irqsave(&bb->tb_list_lock, flags);
	
	list_for_each_entry_safe(tble, n, &bb->free_tblocks, list){
		if (tble->tblock->iblock == iblock){
			found = tble;
			list_del(&tble->list);
			break;
		}
	}
	spin_unlock_irqrestore(&bb->tb_list_lock, flags);

	return found;
}

void acq200_replaceFreeTblock(TBLE* tble)
{
	struct BIGBUF *bb = &DG->bigbuf;
	unsigned long flags;

	spin_lock_irqsave(&bb->tb_list_lock, flags);
	list_add_tail(&tble->list, &bb->free_tblocks);
	spin_unlock_irqrestore(&bb->tb_list_lock, flags);
}

struct TblockListElement *_locateTblockInPhase(struct Phase *phase, int tbix)
{
	struct TblockListElement* tble;

	list_for_each_entry(tble, &phase->tblocks, list){
		if (tble->tblock->iblock == tbix){
			dbg(1, "tblock %d found in phase %s", 
						tbix, phase->name);
			return tble;
		}
	}

	return 0;
}

struct TblockListElement *locateTblockInPhase(struct Phase *phase, int tbix)
{
	struct TblockListElement* tble;
	struct Phase *found_in_phase;


	tble = _locateTblockInPhase(found_in_phase = phase, tbix);

	if (tble == 0){
		struct Phase *nphase = NEXT_PHASE(phase);
		if ((found_in_phase = nphase) != 0){
			tble = _locateTblockInPhase(nphase, tbix);
		}
	}
	if (tble == 0){
		struct Phase *pphase = PREV_PHASE(phase);
		if ((found_in_phase = pphase) != 0){
			tble = _locateTblockInPhase(pphase, tbix);
		}
	}

	dbg(1, "tbix:%d %s %s", tbix, tble? "FOUND": "", found_in_phase->name);

	return tble;
}


struct TblockListElement *locatePrevTblockInPhase(struct Phase *phase, 
       struct TblockListElement *tble)
{
	struct TblockListElement* new_tble;

	if (tble->list.prev != &tble->list){
		new_tble = TBLE_LIST_ENTRY(tble->list.prev);

		dbg(1, "TBLOCK %d found in current phase %s",
		    new_tble->tblock->iblock, phase->name);
		return new_tble;

	}else{		 
		struct Phase *prev_phase = PREV_PHASE(phase);
		if (prev_phase){
			list_for_each_entry_reverse(
				new_tble, &prev_phase->tblocks, list){
				if (new_tble->tblock != tble->tblock){
					dbg(1, "TBlock %d found in %s",
					    new_tble->tblock->iblock, 
					    prev_phase->name);
					return new_tble;
				}
			}
		}
	}
	
	return 0;
}

struct TblockListElement *locateNextTblockInPhase(struct Phase *phase, 
       struct TblockListElement *tble)
{
	struct TblockListElement* new_tble;

	if (tble->list.next != &tble->list){
		new_tble = TBLE_LIST_ENTRY(tble->list.next);

		dbg(1, "TBLOCK %d found in current phase %s",
		    new_tble->tblock->iblock, phase->name);
		return new_tble;

	}else{
		struct Phase *next_phase = NEXT_PHASE(phase);

		if (next_phase){
			list_for_each_entry(
				new_tble, &next_phase->tblocks, list){
				if (new_tble->tblock != tble->tblock){
					dbg(1, "TBlock %d found in %s",
					    new_tble->tblock->iblock,
					    next_phase->name);
					return new_tble;
				}
			}
		}
	}
	
	return 0;
}

void shareTblockInPhase(struct Phase *phase, TBLE *tle_cur)
{
	struct TBLOCK* tb = tle_cur->tblock;
	TBLE *cursor;

	if (phase == 0){
		err("null phase"); return;
	}
	if (strcmp(phase->name, "phase1") != 0){
		err("expected phase1 got %s", phase->name);
		return;
	}

	list_for_each_entry(cursor, &phase->tblocks, list){
		if (cursor->tblock->iblock == tb->iblock){
			dbg(1, "tblock %d already in phase %s", 
					tb->iblock, phase->name);
			return;
		}
	}	

	share_tblock(phase, tle_cur, TBLE_SHARE_FROM_POOL);

	++fix_event_share_tblock;
	dbg(1,"shared phase %s tblock %d", phase->name, tb->iblock);
}

int getTblockFromOffset(unsigned offset)
{
	return TBLOCK_INDEX(offset);
}
EXPORT_SYMBOL_GPL(acq200_phase_release_tblocks);
EXPORT_SYMBOL_GPL(acq200_reserveFreeTblock);
EXPORT_SYMBOL_GPL(acq200_reserveSpecificTblock);
EXPORT_SYMBOL_GPL(acq200_replaceFreeTblock);
EXPORT_SYMBOL_GPL(getTblockFromOffset);
