/* ------------------------------------------------------------------------- */
/* acq200-prep.c preprogrammed trigger control                               */
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

/*

Module implements a filesystem prepfs:

spec       : specify pre definition { start, length [extra] } RW
sum        : summary of overall status all preps
stat       : current status (block on this for next prep in real time

data/START/      : data set for prep {START}
data/START/XXP   : data file in raw format, available real time
data/START/01 .. : channelized data, optional availablility with post process

To free a data set: rm -Rf data/START

Semantics of spec:

open-write: clears old spec.
write: accepts and buffers data
close: processes and validates data

open-read:
read: read data to check acceptance.
close: nothing

 * VFS From example at http://lwn.net/Articles/57373/
 * Copyright 2002, 2003 Jonathan Corbet <corbet-AT-lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
*/

#include <linux/blkdev.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/list.h>
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
#include <linux/pagemap.h>            /* PAGE_CACHE_SIZE */
#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/atomic.h>
#include <asm/uaccess.h>	/* copy_to_user */

/* keep debug local to this module */
#define acq200_debug acq200_prep_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-local.h"     /* DG */
#include "acq200-fifo-tblock.h"    /* acq200_phase_release_tblocks() */


#include "acq32busprot.h"          /* soft link to orig file */

int acq200_prep_debug;
module_param(acq200_prep_debug, int, 0664);

int channel_devices;
module_param(channel_devices, int, 0664);

#define VERID "$Revision: 1.3 $ build B1004 "

#define TD_SZ (sizeof(struct tree_descr))

#define PREPFS_MAGIC 0xfeed2005
#define TMPSIZE 20

#define MAXSPEC (512)               /* 512 preps limit */
#define MAXSPEC_TXT (MAXSPEC*20)
#define MAXERR  (4096)              /* errbuffer */

char acq200_prep_driver_name[] = "acq200-prep";
char acq200_prep_driver_string[] = "PRE Programmed Captures";
char acq200_prep_driver_version[] = VERID __DATE__;
char acq200_prep_copyright[] = "Copyright (c) 2005 D-TACQ Solutions Ltd";


/*
 * External to this module, this struct gets used as a simple Phase
 */
struct PrepPhase {
	struct Phase phase;
	struct Spec* spec;
};


struct PrepInode {
	int id;
	int ch;			/* 0=>XX */
	struct Phase *phase;
	char name[4];
};

#define PREPID 0xca11eeee
#define PREPI(inode) ((struct PrepInode*)inode->i_private)

#define XXP_ID	0

static struct PrepInode* createPrepi(struct Phase *phase, int chan_id)
{
	struct PrepInode *prepi = kmalloc(sizeof(struct PrepInode), GFP_KERNEL);
	prepi->id = PREPID;
	prepi->phase = phase;
	prepi->ch = chan_id;
	if (chan_id == XXP_ID){
		strcpy(prepi->name, "XXP");
	}else{
		snprintf(prepi->name, 4, "%02d", chan_id);
	}
	return prepi;
}

static void deletePrepi(struct PrepInode* prepi)
{
	prepi->id = 0xdeadbeef;
	kfree(prepi);
}

struct Spec {
	int id;
	int start;
	int length;
	u16 flags;
	u16 state;
	/* possible others here */
	struct list_head list;
	const char *def;
	const char *errmsg;
};

#define S_FLAGS_COMMENT 1
#define S_FLAGS_BLANK   2
#define S_FLAGS_SPEC    4
#define S_FLAGS_VALID   8

enum SPEC_STATES {
	SS_IDLE,       /* defined, before ARM */
	SS_PENDING,    /* hasn't happened yet */
        SS_BUSY,       /* in progress (unlikely to see this one) */
	SS_COMPLETE,   /* completed, and data is reserved in memory */
        SS_DELETED,    /* data has been returned to main capture buffer. */
	SS_ERROR
};


#define SPEC_ENTRY(ptr) list_entry(ptr, struct Spec, list)

#define IS_SPEC(spec)  (((spec)->flags & S_FLAGS_SPEC) != 0)
#define IS_VALID(spec) (((spec)->flags & S_FLAGS_VALID) != 0)

struct SpecFileBuffer {
	struct TextBuffer {
		char *buffer;
		char *end;
		char *cursor;
	} source_text, err_text;

	struct list_head specs;	

	struct Spec* spec_pool;
	int spec_pool_index;
};

#define SFB(file) ((struct SpecFileBuffer*)file->private_data)


struct StatusConsumer {
	int is_active;
	wait_queue_head_t waitq;
	struct u32_ringbuffer rb;	
};

/*
 * Module-Global Data
 */
static struct PrepFsGlobs {
	struct super_block *sb;
	struct dentry *data;

	struct SpecFileBuffer spec;
	struct StatusConsumer status;
} PFG;



/*
 * local methods
 */

static inline void txtClr(struct TextBuffer *tb) { tb->cursor = tb->buffer; }
static inline char* txtGetCursor(struct TextBuffer *tb) { return tb->cursor; };
static inline int txtAddData(struct TextBuffer *tb, int len) {
	int nadd = min(tb->end - tb->cursor, len);
	
	tb->cursor += nadd;
	return nadd;
}
static inline int txtGetLength(struct TextBuffer *tb)
{
	return tb->cursor - tb->buffer;
}

static void clearSpecs(struct SpecFileBuffer *spec);


static inline void sfbClearPool(struct SpecFileBuffer *spec) {
	spec->spec_pool_index=0;
}
static inline struct Spec *sfbAllocSpec(struct SpecFileBuffer *spec) {
	return &spec->spec_pool[spec->spec_pool_index++];
}
static inline struct Spec *sfbGetSpec(struct SpecFileBuffer *spec, int ix) {
	if (ix < spec->spec_pool_index){
		return &spec->spec_pool[ix];
	}else{
		return 0;
	}
}

static const char* specShowFlags(struct Spec *spec)
{
	static char buf[80];

	u16 flags = spec->flags;
	buf[0] = '\0';

	if (flags&S_FLAGS_SPEC){
		strcat(buf, "SPEC ");
	}
	if (flags&S_FLAGS_VALID){
		strcat(buf, "VALID ");
	}
	return buf;
}

static const char* specShowState(struct Spec *spec) 
{
	switch(spec->state){
	case SS_IDLE:     return "IDLE";
	case SS_PENDING:  return "PENDING";
        case SS_BUSY:     return "BUSY";
	case SS_COMPLETE: return "COMPLETE";
        case SS_DELETED:  return "DELETED";
	default:          return "ERROR";
	}
}

static struct Phase *getPhaseFromSpec(struct Spec *spec)
{
	struct PrepPhase *pp;

	list_for_each_entry(pp, &DMC_WO->prep_phases, phase.list){
		if (pp->spec == spec){
			return &pp->phase;
		}
	}
	return 0;
}
static const char* phaseShowTblocks(struct Phase *phase, const char *sep)
{
#define LBUF 40
	if (phase != 0){
		static char lbuf[LBUF];
		struct TblockListElement* tble;
		int ibuf = 0;

		lbuf [0] = '\0';
		list_for_each_entry(tble, &phase->tblocks, list){
			ibuf += snprintf(lbuf+ibuf, LBUF-ibuf, "%03d%s",
					 tble->tblock->iblock, sep);
		}
		return lbuf;
	}else{
		return "null phase";
	}
#undef LBUF
}

static struct inode *prepfs_make_inode(struct super_block *sb, int mode);

static struct dentry *prepfs_create_file (
	struct super_block *sb,	
	struct dentry *dir, 
	struct file_operations* fops,
	const char *name,
	void *clidata);

static struct dentry *prepfs_create_dir(
	struct super_block *sb,	
	struct dentry *parent, 
	const char *name);


/** WORKTODO: cut n paste from acq200-fifo-bigbuf-fops.c */
static struct TblockListElement* getTble(
	struct Phase *phase, unsigned offsetw )
{
	struct TblockListElement* tble;
	unsigned offset_in_phase = offsetw - phase->start_sample;


	dbg(2, "phase %p offsetw %d offset_in_phase %d", 
	    phase, offsetw, offset_in_phase);

	list_for_each_entry(tble, &phase->tblocks, list){
		if (offset_in_phase >= tble->phase_sample_start &&
                    offset_in_phase <  tble_phase_end_sample(tble) ){
			dbg(1, "tble %p tblock %p [%d] %s",
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
static void prep_initBBRP(
	struct file *file, char *buf, size_t len, loff_t *offset,
	struct BigbufReadPrams* bbrp)
{
	unsigned offsam = get_fileOffsetSamples(file, offset, 
				  DG->sample_read_stride, DCI(file)->ssize);
	struct Phase* phase = DCI_PHASE(file);
	struct TblockListElement* tble = getTble(phase, offsam);

	dbg(2,"buf %p len %d offset %lu phase %p", 
	    buf, len, (unsigned long)*offset, phase);

	if (!phase){
		err("null PHASE");
		bbrp->status = -ENO_PHASE;
	}else if (phase_len(phase) == 0 || offsam >= phase_end_sample(phase)){
		bbrp->status = BBRP_COMPLETE;
	}else if (!tble){
		err("null TBLOCK LIST ENTRY");
		bbrp->status = -ENO_TBLOCK;
	}else{
		acq200_initBBRP_using_phase(
			file, len, offsam, bbrp, phase, tble);
	}
}

/*
 * actions
 */
static inline int getPhaseTblockCount(struct Phase *phase)
{
	struct TblockListElement* tble;
	int tble_count = 0;

	list_for_each_entry(tble, &phase->tblocks, list){
		dbg(2, "phase %s tblock %d", 
		    phase->name, tble->tblock->iblock);
		++tble_count;
	}
	return tble_count;
}

static int xxp_open(struct inode *inode, struct file *file)
{
	struct PrepInode *prepi = PREPI(inode);
	dbg(1, "");
	acq200_fifo_bigbuf_xx_open (inode, file);

	DCI_PHASE(file) = prepi->phase;

	dbg(1, "phase %s tb %d ssize %d", 
	    DCI_PHASE(file)->name,
	    getPhaseTblockCount(DCI_PHASE(file)),
	    DCI(file)->ssize);

	return 0;
}

static ssize_t xxp_read(
	struct file *file, char *buf, size_t len, loff_t *offset)
{ 
	struct BigbufReadPrams bbrp = { 0, };

	dbg(1, "offset %d", *(unsigned*)offset);
	prep_initBBRP(file, buf, len, offset, &bbrp);

	return acq200_fifo_bigbuf_xxX_read(file, buf, len, offset, &bbrp);
}

static int xxp_release(struct inode *inode, struct file *file)
{
	return acq200_fifo_bigbuf_xx_release(inode, file);
}


static int prep_gather_block(
	struct Phase *phase,
	struct TblockListElement *tle,
	int nblock
	)
{
	if (nblock == 0){
		int ssz = sample_size();
		unsigned map_delta = 
			ssz * (phase->ref_start_scc - phase->start_after);
		unsigned ref_offset_in_tb = TBLOCK_OFFSET(phase->ref_offset);

		dbg(1, "ref_start_scc %Lu start_after %Lu delta %u",
		    phase->ref_start_scc, phase->start_after, map_delta);

		if (map_delta > ref_offset_in_tb){

			err( "map_delta %u > ref_offset_in_tb %u",
			     map_delta, ref_offset_in_tb);
			/* @@ we does our best */
			map_delta = ref_offset_in_tb; 
		}
	
		tle->phase_sample_start = 0;
		tle->tblock_sample_start = 
			TBLOCK_OFFSET(ref_offset_in_tb - map_delta)/ssz;
	}else{

		TBLE* prev = TBLE_LIST_ENTRY(tle->list.prev);
		tle->phase_sample_start = 
			prev->phase_sample_start + prev->sample_count;

		tle->tblock_sample_start = 0;		

		dbg(1, "prev start %u count %u this_start %u",
		    prev->phase_sample_start, prev->sample_count,
		    prev->phase_sample_start + prev->sample_count);

	}

	tle->sample_count = 
		min(phase_num_samples(phase) - tle->phase_sample_start,
		    getTblockMaxSam() - tle->tblock_sample_start);

	dbg(1, "entry [%2d] %s sample_count %d", 
	    tle->tblock->iblock, tle2string(tle), tle->sample_count);

	return tle->sample_count;
}


static void prep_gather_blocks(struct Phase *phase)
/* set exact sample begin end points in TLEaves 
Data:
phase->start_after  : desirec start point
phase->ref_start_scc/phase->ref_offset : ref points to find start.
*/
{
	struct TblockListElement *tle;
	int nb = 0;
	int ngather;

	/** this is valid because we don't get called until len exceeded */
	phase->actual_len = phase->required_len;

	list_for_each_entry(tle, &phase->tblocks, list){
		ngather = prep_gather_block(phase, tle, nb);
		if (ngather){
			phase->actual_samples += ngather;
			++nb;
		}
	}
	dbg(1, "actual_len:%d actual_samples:%d",
	    phase->actual_len, phase->actual_samples);	
}


static void prep_create_xxp(
	struct super_block *sb, 
	struct dentry *dir, 
	struct Phase* phase)
{
	static struct file_operations xxp_fops = {
		.open	= xxp_open,
		.read 	= xxp_read,
		.release= xxp_release
	};

	struct PrepInode *prepi = createPrepi(phase, XXP_ID);
	struct dentry *d = 
		prepfs_create_file(sb, dir, &xxp_fops, prepi->name, prepi);

        if (d == 0){
		err( "ERROR:failed to create file");
	}
}

static int chx_open(struct inode *inode, struct file *file)
{
	struct PrepInode *prepi = PREPI(inode);
	acq200_initDCI(file, prepi->ch);
	DCI_PHASE(file) = prepi->phase;
	return 0;	
}

static ssize_t chx_read(
	struct file *file, char *buf, size_t len, loff_t *offset)
{ 
	struct BigbufReadPrams bbrp = { 0, };
	prep_initBBRP(file, buf, len, offset, &bbrp);

	return acq200_fifo_bigbuf_read_bbrp(file, buf, len, offset, &bbrp);
}

static int chx_release(struct inode *inode, struct file *file)
{
	acq200_releaseDCI(file);
	return 0;
}

static void prep_create_chx(
	struct super_block *sb, 
	struct dentry *dir, 
	struct Phase* phase,
	int chx
	)
{
	/* @todo ... put in chx equivs */
	static struct file_operations chx_fops = {
		.open	= chx_open,
		.read 	= chx_read,
		.release= chx_release
	};

	struct PrepInode *prepi = createPrepi(phase, chx);
	struct dentry *d = 
		prepfs_create_file(sb, dir, &chx_fops, prepi->name, prepi);

        if (d == 0){
		err( "ERROR:failed to create file");
	}
}


static void prep_on_new_phase(struct Phase* phase)
{
	struct dentry *dir = prepfs_create_dir(PFG.sb, PFG.data, phase->name);
	int ch;

	dir->d_inode->i_private = phase;

	if (dir){
		int nchan = channel_devices != 0? NCHAN: 0;

		prep_gather_blocks(phase);
		prep_create_xxp(PFG.sb, dir, phase);
		
		for (ch = 1; ch <= nchan; ++ch){
			prep_create_chx(PFG.sb, dir, phase, ch);
			schedule();
		}
		((struct PrepPhase *)phase)->spec->state = SS_COMPLETE;
	}else{
		((struct PrepPhase *)phase)->spec->state = SS_ERROR;
	}
}

#if 0
static void prep_on_transform_phase(const char* phase)
{

}
#endif

static void prep_on_del_phase(struct Phase* phase)
{
	dbg(1, "delete %p", phase);
	((struct PrepPhase *)phase)->spec->state = SS_DELETED;
	acq200_phase_release_tblocks(phase);
	list_del(&phase->list);
	kfree(phase);
}

static void add_spec(struct SpecFileBuffer *sfb, const char* line)
{
	struct Spec *spec = sfbAllocSpec(sfb);

	memset(spec, 0, sizeof(struct Spec));
	spec->def = line;

	dbg(1, "line %s", line);

	if (strlen(line) == 0){
		spec->flags = S_FLAGS_BLANK|S_FLAGS_VALID;
	}else if (line[0] == '#'){
		spec->flags = S_FLAGS_COMMENT|S_FLAGS_VALID;
	}else if (sscanf(line, "%d %d", &spec->start, &spec->length) == 2){
		spec->flags = S_FLAGS_SPEC|S_FLAGS_VALID;
		spec->state = SS_IDLE;
		list_add_tail(&spec->list, &sfb->specs);
	}else{
		spec->flags = S_FLAGS_SPEC;
		spec->errmsg = "ERROR:spec needs <start> <length>:";
	}	
}


static void parse_spec(struct SpecFileBuffer *sfb)
/* run thru text buffer, build a chain of specs. */
{
	char *buf = sfb->source_text.buffer;
	char *end = sfb->source_text.cursor;

	char *left, *right;

	for (left = right = buf; left < end; left = right + 1){
		for (right = left; right < end && *right != '\n'; ++right){
			;
		}
		*right = '\0';

		add_spec(sfb, left);
	}
}

static void validate_spec(struct SpecFileBuffer *sfb)
{
	struct Spec *spec;
	int id = 1;

	list_for_each_entry(spec, &PFG.spec.specs, list){
		if (spec->list.prev != &PFG.spec.specs){
			struct Spec *prev = SPEC_ENTRY(spec->list.prev);
			unsigned prev_end = prev->start + prev->length;

			if (!spec->start > prev_end){
				spec->flags &= ~S_FLAGS_VALID;
				spec->errmsg = "ERROR: overlap range";
			}

		}
		spec->id = id++;
	}
}
/*
 * ops on static files 
 */
static int spec_open(struct inode *inode, struct file *file)
{
	file->private_data = &PFG.spec;

	if ((file->f_mode & FMODE_WRITE) != 0){
		struct SpecFileBuffer *sfb = SFB(file);

		sfbClearPool(sfb);
		clearSpecs(sfb);
		txtClr(&sfb->source_text);
		txtClr(&sfb->err_text);
		
	}

	dbg(1, "ready to go");
	return 0;
}

static ssize_t spec_write(struct file *file, const char *buf,
		size_t count, loff_t *offset)
{
	struct SpecFileBuffer *sfb = SFB(file);
	char* bp = txtGetCursor(&sfb->source_text);
	int ncopy = txtAddData(&sfb->source_text, count);

	dbg(1, "count %d ncopy %d", count, ncopy);
	if (copy_from_user(bp, buf, ncopy)){
		return -EFAULT;
	}

	*offset += ncopy;
	return ncopy;
}

static ssize_t spec_read(struct file *file, char *buf,
		size_t count, loff_t *offset)
{
	struct SpecFileBuffer *sfb = SFB(file);
	struct Spec *spec = sfbGetSpec(sfb, *offset);

	if (spec){
		int msg_len = (spec->errmsg? strlen(spec->errmsg): 0) + 
			      (spec->def? strlen(spec->def): 0) + 1;
		int ncopy;

		if (msg_len <= count){

			if (spec->errmsg){
				ncopy = strlen(spec->errmsg);
				COPY_TO_USER(buf, spec->errmsg, ncopy);
				buf += ncopy;
			}
			if (spec->def){
				dbg(1, "%12s %s", specShowFlags(spec),
				    spec->def);
				ncopy = strlen(spec->def);
				COPY_TO_USER(buf, spec->def, ncopy);
				buf += ncopy;
			}
			COPY_TO_USER(buf, "\n", 1);
			*offset += 1;
			return msg_len;
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}

static int spec_release(
        struct inode *inode, struct file *file)
{
	if ((file->f_mode & FMODE_WRITE) != 0){
		parse_spec(SFB(file));
		validate_spec(SFB(file));
	}
	return 0;
}

static int test_open(struct inode *inode, struct file *file)
{
	info("f_mode %d", file->f_mode);
	return 0;
}

#if 0
static char* chomp(char* name)
{
	char* cp = name;

	while(*cp > ' ' && *cp < 'z'){
		++cp;
	}
	*cp = '\0';

	return name;
}
#endif


static ssize_t test_write(struct file *file, const char *buf,
		size_t count, loff_t *offset)
{
	return count;
}

static ssize_t test_read(struct file *file, char *buf,
		size_t count, loff_t *offset)
{
	return 0;
}

static int test_release(
        struct inode *inode, struct file *file)
{
	return 0;
}


static int sum_open(struct inode *inode, struct file *file)
{
	return 0;
}



/*
 * this is _very_ crude - we should stash the cursor between calls ...
 */
static ssize_t sum_read(struct file *file, char *buf,
		size_t count, loff_t *offset)
{
#define LBL 128
	struct Spec *spec;
	int idx = *offset;
	int ii = 0;

	list_for_each_entry(spec, &PFG.spec.specs, list){
		if (ii == idx){
			char lb[LBL];
			struct Phase *phase = getPhaseFromSpec(spec);
			const char *ps = phase != 0?
				phaseShowTblocks(phase, ","):
				"null-phase";
			int len = snprintf(lb, LBL, 
					   "%03d %10d %8d %10s %s %s\n",
					  spec->id, 
					  spec->start, 
					  spec->length, 
					  specShowState(spec),
					  specShowFlags(spec),
					  ps);

			if (len <= count){
				if (copy_to_user(buf, lb, len)){
					return -EFAULT;
				}
				*offset += 1;
				return len;
			}else{
				return 0;
			}
		}
		++ii;
	}

	return 0;
#undef LBL
}


/*
 * this is _very_ crude - we should stash the cursor between calls ...
 */
static ssize_t tb_read(struct file *file, char *buf,
		size_t count, loff_t *offset)
{
#define LBL 128
	struct Spec *spec;
	int idx = *offset;
	int ii = 0;

	list_for_each_entry(spec, &PFG.spec.specs, list){
		if (ii == idx){
			struct Phase *phase = getPhaseFromSpec(spec);
			const char *ps = phaseShowTblocks(phase, "\n");
			int len = strlen(ps);
			if (len <= count){
				if (copy_to_user(buf, ps, len)){
					return -EFAULT;
				}
				*offset += 1;
				return len;
			}else{
				return 0;
			}
		}
		++ii;
	}

	return 0;
#undef LBL
}


static void initStatusConsumer(struct StatusConsumer *sc)
{
	u32rb_init(&sc->rb, MAXSPEC);
	init_waitqueue_head(&sc->waitq);
	sc->is_active = 1;
}

static void clearStatusConsumer(struct StatusConsumer *sc)
{
	sc->is_active = 0;
	u32rb_destroy(&sc->rb);
}
static int stat_open(struct inode *inode, struct file *file)
{
	initStatusConsumer(&PFG.status);
	file->private_data = &PFG.status;
	return 0;
}



static ssize_t stat_read(struct file *file, char *buf,
		size_t count, loff_t *offset)
{
#define LBL 64
	struct StatusConsumer *sc = 
		(struct StatusConsumer *)file->private_data;
	struct PrepPhase *pp;

	wait_event_interruptible(sc->waitq, !u32rb_is_empty(&sc->rb));

	if (!u32rb_is_empty(&sc->rb)){
		char lb[LBL];
		int len = (int)count;

		len = min(len, LBL);

		u32rb_get(&sc->rb, (u32*)&pp);

		{
			struct Spec *spec = pp->spec;
			struct Phase *phase = &pp->phase;

			len = snprintf(lb, len, 
				       "%03d.%-8d %8d %10s %Ld %ld %ld\n", 
				       spec->id, 
				       spec->start, spec->length, 
				       specShowState(spec),
				       phase->ref_start_scc,
				       phase->prep_start_time.tv_sec, 
				       phase->prep_start_time.tv_usec);

			if (copy_to_user(buf, lb, len)){
				return -EFAULT;
			}
			*offset += 1;
			return len;
		}
	}else{
		return 0;
	}
#undef LBL
}

static int stat_release(
        struct inode *inode, struct file *file)
{
	clearStatusConsumer(&PFG.status);
	return 0;
}

static void wakeStatusConsumer(struct StatusConsumer *sc, struct Phase *phase)
{
	if (sc->is_active){
		u32rb_put(&sc->rb, (u32)(struct PrepPhase *)phase);
		wake_up_interruptible(&sc->waitq);
	}
}



static void onPhaseComplete(struct Phase* phase)
{
	dbg(1, "phase %s", phase->name);
	prep_on_new_phase(phase);
	wakeStatusConsumer(&PFG.status, phase);
}

static struct Phase *onPIT_default(
	struct Phase *phase, u32 status, u32* offset)
/* @@todo - maybe respond to PIT later */
{
	return phase;
}

static int iprep;


static int run_deleteall_helper(void)
/* this is a hack until we understand how to delete dentry tree internally */
/* 'tis a hack because assumes mount point name is known */
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};

	static char *argv[4];
	int i;

        i = 0;
        argv[i++] = "/bin/rm";
        argv[i++] = "-Rf";
	argv[i++] = "/dev/prep/data/";
        argv[i] = 0;

	dbg( 1, "call_usermodehelper %s\n", argv[0] );

	i = call_usermodehelper(argv [0], argv, envp, 1);

        dbg( 1, "call done returned %d", i );
	return 0;
}


static void clearOldPrepPhases(void)
{
	if (DMC_WO->prep_phases.next == 0){
		dbg(1,"prep_phases not initialized, do it now");
		INIT_LIST_HEAD(&DMC_WO->prep_phases);
	}else{
#ifdef PGMCOMOUT
                struct PrepPhase *phase, *tmp;
		dbg(1, "list all");

		list_for_each_entry_safe(phase, tmp, 
					 &DMC_WO->prep_phases, phase.list){
			prep_on_del_phase(&phase->phase);
		}
#else
		run_deleteall_helper();
#endif
	}

	iprep = 0;
}


static void appendNewPrepPhase(struct Spec* spec)
{
	struct PrepPhase* prep_phase = 
		kmalloc(sizeof(struct PrepPhase), GFP_KERNEL);
	struct Phase *phase;

	if (!prep_phase){
		err("ERROR: failed to allocate phase");
		return;
	}

	 phase = &prep_phase->phase;

	memset(phase, 0, sizeof(struct Phase));
	sprintf(phase->name, "%03d.%u", ++iprep, spec->start);

	dbg(1,"appending %s", phase->name);

	INIT_LIST_HEAD(&phase->tblocks);
	phase->start_after = spec->start;
	phase->tblock_max_count = phase->demand_len/TBLOCK_LEN + 2;
	phase->is_oneshot = 1;
	phase->onPIT = onPIT_default;
	phase->onPhaseComplete = onPhaseComplete;
	phase->demand_samples = spec->length;
	phase->required_len = phase->demand_len = spec->length*sample_size();

	prep_phase->spec = spec;

	list_add_tail(&phase->list, &DMC_WO->prep_phases);
	dbg(1,"done");
}


static void onStartOfShot(void)
{
	struct Spec *spec;

	dbg(1,"");
	clearOldPrepPhases();

	list_for_each_entry(spec, &PFG.spec.specs, list){
		if (IS_SPEC(spec) && IS_VALID(spec)){
			appendNewPrepPhase(spec);
			spec->state = SS_PENDING;
		}
	}
	DMC_WO->prep_now = PHASE_LIST_ENTRY(DMC_WO->prep_phases.next);
}


static int prepfs_fill_super_statics(
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations spec_ops = {
		.open = spec_open,
		.read = spec_read,
		.write = spec_write,
		.release = spec_release
	};
	static struct file_operations sum_ops = {
		.open = sum_open,
		.read = sum_read,
	};
	static struct file_operations stat_ops = {
		.open = stat_open,
		.read = stat_read,
		.release = stat_release
	};
	static struct file_operations tb_ops = {
		.read = tb_read
	};
	static struct file_operations test_ops = {
		.open = test_open,
		.read = test_read,
		.write = test_write,
		.release = test_release
	};

	static struct tree_descr static_files[] = {
		{ NULL, NULL, 0	},
		{ .name = "spec",  .ops = &spec_ops, .mode = S_IWUSR|S_IRUGO },
		{ .name = "summary",  .ops = &sum_ops,  .mode = S_IRUGO },
		{ .name = "status", .ops = &stat_ops, .mode = S_IRUGO },
		{ .name = "tblocks", .ops = &tb_ops, .mode = S_IRUGO },
		{ .name = "test", .ops = &test_ops, .mode = S_IWUSR|S_IRUGO },
		{ "", NULL, 0 },
	};
	
	PFG.sb = sb;
	return simple_fill_super(sb, PREPFS_MAGIC, static_files);
}


int pgm_unlink(struct inode *dir, struct dentry *dentry)
{
	struct PrepInode *prepi = PREPI(dentry->d_inode);
	dbg(1, "prepi: %p id 0x%08x %s", 
	     prepi, prepi->id, prepi->id==PREPID? "PREPID": "ERR - no PREPID");
	if (prepi->id == PREPID){
		deletePrepi(prepi);
	}
	return simple_unlink(dir, dentry);
}

int pgm_rmdir(struct inode *dir, struct dentry *dentry)
/* recycle the Phase on data directory deletion */
{
	struct Phase* phase = (struct Phase*)dentry->d_inode->i_private;
	int rc = simple_rmdir(dir, dentry);

	if (rc == 0 && phase != 0){
		prep_on_del_phase(phase);
	}

	return rc;
}

static unsigned update_inode_stats(struct inode *inode)
{
	struct PrepInode *prepi = PREPI(inode);
	struct Phase *phase = prepi->phase;
       	unsigned ssize;
	unsigned totsam;

	dbg(1, "prepi %p id 0x%08x ch 0x%08x", prepi, prepi->id, prepi->ch);

	if (prepi->ch == XXP_ID){
		ssize = sample_size();
	}else{
		ssize = CSIZE;
	}
	totsam = phase->actual_samples;
	dbg(1, "ssize %d totsam %d total %d",
	    ssize, totsam, ssize*totsam);
		/* could be better ... */
	inode->i_mtime = DG->stats.finish_time;
	inode->i_size = ssize * totsam;	

	return inode->i_size;

}

/** copy from acq200-fifo-bigbuf-fops.c */
static int ai_getattr (struct vfsmount *mnt, struct dentry *d, struct kstat *k)
{
	struct inode *inode = d->d_inode;
	int was;

	generic_fillattr(inode, k);
	was = k->size;
	k->size = update_inode_stats(inode);
	dbg(1, "size was %d set to %d", was, (int)k->size);
	return 0;
}

static struct dentry *prepfs_create_file (
	struct super_block *sb,
	struct dentry *dir, 
	struct file_operations *fops,
	const char *name,
	void *clidata
	)
{
	static struct inode_operations inops = {
		.getattr = ai_getattr
	};
	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;
/*
 * Make a hashed version of the name to go with the dentry.
 */
	qname.name = name;
	qname.len = strlen (name);
	qname.hash = full_name_hash(name, qname.len);
/*
 * Now we can create our dentry and the inode to go with it.
 */
	dentry = d_alloc(dir, &qname);
	if (! dentry)
		goto out;
	inode = prepfs_make_inode(sb, S_IFREG | 0644);
	if (! inode)
		goto out_dput;
	inode->i_fop = fops;
	inode->i_private = clidata;
	inode->i_op = &inops;
/*
 * Put it all into the dentry cache and we're done.
 */
	d_add(dentry, inode);
	return dentry;
/*
 * Then again, maybe it didn't work.
 */
  out_dput:
	dput(dentry);
  out:
	return 0;
}

/*
 * Anytime we make a file or directory in our filesystem we need to
 * come up with an inode to represent it internally.  This is
 * the function that does that job.  All that's really interesting
 * is the "mode" parameter, which says whether this is a directory
 * or file, and gives the permissions.
 */
static struct inode *prepfs_make_inode(struct super_block *sb, int mode)
{
	struct inode *ret = new_inode(sb);

	if (ret) {
		ret->i_mode = mode;
		ret->i_uid = ret->i_gid = 0;
		ret->i_blkbits = blksize_bits(PAGE_CACHE_SIZE);
		ret->i_blocks = 0;
		ret->i_atime = ret->i_mtime = ret->i_ctime = CURRENT_TIME;
	}
	return ret;
}

static struct dentry* prepfs_create_dir(struct super_block *sb,
		struct dentry *parent, const char *name)
{
	static struct inode_operations deletable_dir_inode_operations = {
		.lookup		= simple_lookup,
		.rmdir          = pgm_rmdir,
		.unlink         = pgm_unlink
	};

	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;

	qname.name = name;
	qname.len = strlen (name);
	qname.hash = full_name_hash(name, qname.len);
	dentry = d_alloc(parent, &qname);
	if (!dentry)
		goto out;

	inode = prepfs_make_inode(sb, S_IFDIR | 0755);

	if (!inode)
		goto out_dput;
	inode->i_op = &deletable_dir_inode_operations;
	inode->i_fop = &simple_dir_operations;

	d_add(dentry, inode);
	return dentry;

  out_dput:
	dput(dentry);
  out:
	return 0;
}


static int prepfs_fill_super(struct super_block *sb, void *data, int silent)
{
	int rc;

	rc = prepfs_fill_super_statics(sb, data, silent);
	if (rc != 0){
		return rc;
	}
	
	PFG.data = prepfs_create_dir(sb, sb->s_root, "data");
	return rc;
}



static int prepfs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(fst, flags, data, prepfs_fill_super, mnt);
}

	

static struct file_system_type prepfs_type = {
	.owner = THIS_MODULE,
	.name  = "prepfs",
	.get_sb = prepfs_get_super,
	.kill_sb = kill_litter_super,
};


static int init_text(struct TextBuffer *tb, int len)
{
	tb->buffer = vmalloc(len);

	if (tb->buffer == 0){
		return -ENOMEM;
	}else{
		tb->cursor = tb->buffer;
		tb->end = tb->buffer + len;
		return 0;
	}
}
static int init_spec(struct SpecFileBuffer *spec)
{
#define POOLSZ (MAXSPEC*sizeof(struct Spec))
	int rc = 0;

	spec->spec_pool = vmalloc(POOLSZ);
	if (!spec->spec_pool){
		BUG();
	}

	if ((rc = init_text(&spec->source_text, MAXSPEC_TXT)) != 0){
		return rc;
	}
	if ((rc = init_text(&spec->err_text, MAXERR)) != 0){
		return rc;
	}

	INIT_LIST_HEAD(&spec->specs);
	return rc;
}

static void clearSpecs(struct SpecFileBuffer *spec)
{
	INIT_LIST_HEAD(&spec->specs);
}

static void clear_spec(struct SpecFileBuffer *spec)
{
	clearSpecs(spec);
	vfree(spec->spec_pool);
	vfree(spec->source_text.buffer);
	vfree(spec->err_text.buffer);
}
static int init_buffers(void)
{
	int rc = 0;

	rc = init_spec(&PFG.spec);
	INIT_LIST_HEAD(&DMC_WO->prep_phases);
	if (rc) return rc;

	return rc;
}

static void prep_clear_buffers(void)
{
	clear_spec(&PFG.spec);
}

static struct ExtPhase extPhase = {
	.init = onStartOfShot,
	.release = clearOldPrepPhases
};


static int __init acq200_prep_init( void )
{
	acq200_debug = acq200_prep_debug;

	info("%s %s %s",
	     acq200_prep_driver_name,
	     acq200_prep_driver_version, acq200_prep_copyright);
	init_buffers();
	acq200_add_ext_phase_handler(&extPhase);
	return register_filesystem(&prepfs_type);
}


static void __exit
acq200_prep_exit_module(void)
{
	prep_clear_buffers();
	acq200_del_ext_phase_handler(&extPhase);
	unregister_filesystem(&prepfs_type);
}

module_init(acq200_prep_init);
module_exit(acq200_prep_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Module for 2G PreProgrammed Triggers");

