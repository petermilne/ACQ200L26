/* ------------------------------------------------------------------------- */
/* eb_ppcustom.c event burst custom post processing example for acq196       */
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

/** @file eb_ppcustom.c
 *
 * ACQ196 captures a large data set.
 * But we're only interested in a fraction of the data.
 * Data is in pulses, typ 10usec long
 * Software examines a single marker channel for presence of PULSE
 * We use PULSE as an event input, so multiple ES are buried in the data set,
 * this reduces the marker channel search area. There may be multiple PULSEs
 * per EVENT (because event locks out for about 1msec at full speed),
 * so extend the search for additional PULSES past the ES.
 */

#define ACQ196

#define VERID "$Revision: 1.1 $ build B1025 "


char eb_ppcustom_driver_name[] = "eb_ppcustom";
char eb_ppcustom_driver_string[] = 
	"D-TACQ Low Latency Control Device";
char eb_ppcustom_driver_version[] = VERID __DATE__ "\n";
char eb_ppcustom_copyright[] = 
	"Copyright (c) 2006 D-TACQ Solutions Ltd\n";

#include <linux/blkdev.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/pagemap.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm-arm/mach/irq.h>

#include <asm-arm/fiq.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

/* keep debug local to this module */
#define acq200_debug eb_ppcustom_debug

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int eb_ppcustom_debug;
module_param(eb_ppcustom_debug, int, 0664);


int NSAMPLES = 32;
module_param(NSAMPLES, int, 0664);

/** NCHANNELS in output. NCHANNELS <= NCHANNELS_SAMPLE */
int NCHANNELS = 32;
module_param(NCHANNELS, int, 0664);

int PRETRIG = 8;
module_param(PRETRIG, int, 0664);

int TRIG_CHAN = 32;
module_param(TRIG_CHAN, int, 0664);

int TRIG_RISING = 1;
module_param(TRIG_RISING, int, 0664);

int TRIG_LO_BELOW = 100;		/* ~0 */
module_param(TRIG_LO_BELOW, int, 0664);

int TRIG_HI_ABOVE = 32768/100  * 25;		/* 2.5V */
module_param(TRIG_HI_ABOVE, int, 0664);

int S_pulse_count = 0;
module_param(S_pulse_count, int, 0444);

int S_search_limit = 1024;		/* samples */
module_param(S_search_limit, int, 0664);

int S_stubber = 0;
module_param(S_stubber, int, 0664);

int debug_debug = 0;
module_param(debug_debug, int, 0664);


int MAX_PULSES = 400*8;	      /* 400Hz, 8s .. avoid running out of memory */
module_param(MAX_PULSES, int, 0664);

int S_first_time = 1;

#define SAMPLE_CHECK_INITIAL -1
int S_sample_number = SAMPLE_CHECK_INITIAL;	/* tracks current sample */
short *S_last_es = 0;

/* NB: sample != offset, sample = f(offset, #ES) */

static struct super_block *S_sb;
static struct dentry *S_dir;
static char S_dname[32];

struct EventLocator {
	int evnum;
	int sample_in_phase;
	char fname[32];				/* "P.%06d.%08d" sample */
	unsigned offset_in_block;
	struct TblockListElement *tble;
	struct list_head list;
	int nsamples;
	int nchannels;
	int flags;
	short* data;
};

/* bits for S_stubber */
#define S_STUBBER_PROCESS_EVENT	1

#define ELDATASIZE (NSAMPLES*NCHANNELS*sizeof(short))
#define ELSIZE (sizeof(struct EventLocator))

/** actual size per node may differ (eg not enough pre-data at beginning */
#define ACTUAL_DATASIZE(elp) \
	((elp)->nchannels * (elp)->nsamples * sizeof(short))


/** data is an array channels*samples. Each channel stride is samples long */
#define DATA(elp, c, t) ((elp)->data[(c)*NSAMPLES+(t)])


/** sampling - not necessarily the same as the output */
#define ES_WORDS		(ES_SIZE/sizeof(short))

#define SAMPLE_WORDS		(sample_size()/sizeof(short))
#define NCHANNELS_SAMPLE	(CAPDEF->_nchan)

static int pulse_page_order(void)
{
	int eldatasize = ELDATASIZE;
	int po = 0;

	for (po = 0; (1 << po)*PAGE_SIZE < eldatasize; ++po){
		;
	}
	return po;
}
#define PULSE_PAGE_ORDER	pulse_page_order()


#define COMB_ES_FOUND		0x1
#define COMB_PULSE_FOUND	0x2
#define COMB_ES_ES2		0x4

static int comb(
	struct TblockListElement *tble, 
	int search_for_es,
	unsigned *offinblock,			/* bytes from TB start */
	unsigned *sample_in_phase
	);

static int createFile(struct EventLocator *elp);
static int createShotDir(void);

struct list_head S_elp_current;
struct list_head S_elp_fs;


static void dump(void *base, int size, int nelems)
{
	char buf[128];
	int cursor = 0;
	int item = 0;
	int LINELEN = 8;
	char *fmt;
#define PBS ((unsigned short *)base)
#define PBL ((unsigned long *)base)

	switch(size){
	case 2:
		fmt = "%04x ";
		LINELEN = 8;
		break;
	case 4:
		fmt = "%08x ";
		LINELEN = 4;
		break;
	default:
		assert(0);
	}

	for ( ; item < nelems; ++item){
		int tab = item%LINELEN;

		if (tab == 0){
			cursor += sprintf(buf, "%p:", base+item*size);
		}
			
		if (size == 2){
			cursor += sprintf(buf+cursor, fmt, PBS[item]);
		}else{
			cursor += sprintf(buf+cursor, fmt, PBL[item]);
		}
		assert(cursor < sizeof(buf));

		if (tab == LINELEN-1){
			dbg(1, "%s", buf);
			cursor = 0;
		}
	}

	if (cursor != 0){
		dbg(1,"%s", buf);
	}
}


static void freeEventLocator(struct EventLocator *elp)
{
	if (elp == 0){
		err("attempting to free a null ELP");
		return;
	}
	if (elp->data == 0){
		err("attempting to free null data");
		return;
	}

	dbg(2, "list delp");

	list_del(&elp->list);
	
	dbg(2, "kfree elp");
	free_pages((unsigned)elp->data, PULSE_PAGE_ORDER);
	kfree(elp);
}

static void freeDataList(struct list_head *mylist)
{
	struct EventLocator* eventLocator;

del_top:
	list_for_each_entry(eventLocator, mylist, list){
		freeEventLocator(eventLocator);
		goto del_top;
	}
	assert(list_empty(mylist));
}

static void freeData(void)
{
	freeDataList(&S_elp_current);
	freeDataList(&S_elp_fs);
}


#ifdef FAKEIT
static void test_data(struct EventLocator *elp)
{
	int ch, tt;

	for (ch = 0; ch != NCHANNELS; ++ch){
		for (tt = 0; tt != NSAMPLES; ++tt){
			short yy;

			if (tt < 5){
				yy = tt*10;
			}else if (tt >= NSAMPLES-5){
				yy = (NSAMPLES - tt)*10;
			}else{
				yy = 50;
			}	
			DATA(elp, ch, tt) = ch*10+yy;
		}
	}
}

#define INIT_DATA test_data
#else
#define INIT_DATA(elp) (memset((elp)->data, 0, ELDATASIZE))
#endif

static struct EventLocator* allocData(
	int evnum, 
	struct TblockListElement *tble,
	unsigned block_off,
	unsigned sample_in_phase,
	unsigned flags)
{
	dbg(2, "kmalloc(%d)", ELSIZE);

	{
		struct EventLocator *elp = kmalloc(ELSIZE, GFP_KERNEL);

		if (elp){
			elp->data = (short*)__get_free_pages(
					       GFP_KERNEL, PULSE_PAGE_ORDER);
			if (!elp->data){
				err("failed to alloc data");
				kfree(elp);
				return 0;
			}
			elp->evnum = evnum;
			elp->tble = tble;
			elp->nsamples = NSAMPLES;
			elp->nchannels = NCHANNELS;
			elp->offset_in_block = block_off;
			elp->sample_in_phase = sample_in_phase;
			elp->flags = flags;

			list_add_tail(&elp->list, &S_elp_current);
			INIT_DATA(elp);

			dbg(2, "kmalloc OK elp %p oib %d", elp, block_off);
		}else{
			err("kmalloc failed");
		}
		return elp;
	}
}
static void onFirstTime(void) 
{
	if (NCHANNELS > sample_size()/sizeof(short)){
		NCHANNELS = sample_size()/sizeof(short);
		info("reduced NCHANNELS to match sample_size %d", NCHANNELS);
	}
	if (TRIG_CHAN > NCHANNELS){
		TRIG_CHAN = NCHANNELS;
		info("reduced TRIG_CHAN to match sample_size %d", TRIG_CHAN);
	}

	dbg(1, "ELSIZE %d ELDATASIZE %d", ELSIZE, ELDATASIZE);

	list_splice_init(&S_elp_current, &S_elp_fs);

	S_pulse_count = 0;
	S_sample_number = SAMPLE_CHECK_INITIAL;	
	S_last_es = 0;
	createShotDir();		
}


static int __getEventDef(int iblock, int* event_cursor, unsigned* bb_offset)
{
	int cursor = *event_cursor;
	int nevents = DMC_WO->pit_count;
	struct PIT_DEF *pd = DG->pit_store.the_pits;

     
	while(++cursor < nevents){
		if (TBLOCK_INDEX(pd[cursor].offset) == iblock){
			*bb_offset = pd[cursor].offset;
			*event_cursor = cursor;

			dbg(2, "use this event %d at %d", cursor, *bb_offset);
			return 1;
		}
	}

	return 0;
}

static int getEventDef(
	int iblock, 
	int* event_cursor, unsigned* bb_offset,
	int *search_for_es
	)
{


/** fake an event def at offset 0, ES not always present at start 
 *  (typ if trig used!)
 * but set search_for_es 0 to ensure that a "real" ES in the data is handled
 * correctly.  (... fiery hoops R us).
 */
	if (S_sample_number == SAMPLE_CHECK_INITIAL){
		S_sample_number = 0; /* this only happens once per run */
		*search_for_es = 1;  /* ES not recorded in first block */
		return 1;
	}else{
		*search_for_es = 1;
		return __getEventDef(iblock, event_cursor, bb_offset);
	}
}

static struct TblockListElement *getTble(struct Phase *phase, int iblock)
/* locate TBLOCK container in phase, return NULL if not found */
{
	struct TblockListElement* tble;	

	list_for_each_entry(tble, &phase->tblocks, list){
		if (tble->tblock->iblock == iblock){
			return tble;
		}
	}

	return 0;
}

#ifdef FAKE_IT
static int findEvent(struct TblockListElement *tble, unsigned *block_off)
/** fake findEvent "finds" first event only */
{
	static unsigned _block_off;

	if (_block_off != *block_off){
		_block_off = *block_off;
		dbg(1, "STUB, return block_off %d", *block_off);
		return 1;
	}else{
		return 0;
	}
}
#else


#define MAXTH	4

struct TriggerHistory {
	short previous[MAXTH];
	int ip;				/* -1 if no previous */
};

static void __th_push(struct TriggerHistory *th, short value)
{
	int ip = th->ip;

	if (th->ip >= MAXTH){
		for (ip = 0; ip < MAXTH-1; ++ip){
			th->previous[ip] = th->previous[ip+1];
		}
	}
	th->previous[ip++] = value;
	th->ip = ip;
}

static int __th_any(
	struct TriggerHistory *th,
	int compare(short, short),
	short threshold
	)
{
	int ip;

	for (ip = th->ip; ip--; ){
		if (compare(th->previous[ip], threshold)){
			return 1;
		}
	}
	return 0;
}

static int __compare_gt(short left, short right)
{
	return left > right;
}

static int __compare_lt(short left, short right)
{
	return left < right;
}


static char* __th_history(struct TriggerHistory *th)
{
	static char buf[80];
	int cursor;
	int ic = sprintf(buf, "ip:%d: ", th->ip);

	for (cursor = 0; cursor < th->ip; ++cursor){
		ic += sprintf(buf+ic, "0x%04x ", 
			      (unsigned short)th->previous[cursor]);
	}
	return buf;
}

#define TH_CLEAR(th)	((th)->ip = 0)
#define TH_IS_CLEAR(th) ((th)->ip == 0)
#define TH_PUSH(th, value) __th_push((th), (value))
#define TH_ANY_PREVIOUS(th, cond, threshold) __th_any(th, cond, threshold)
#define TH_HIST(th)	__th_history(th)

#define TH_LT __compare_lt
#define TH_GT __compare_gt


static int isES(unsigned* ps){
	int rc = 0;

	if (IS_EVENT_MAGIC(*ps)){
		rc = acq200_check_entire_es(ps);
		
		if (!rc){
			void *bad_es = (void*)ps - sample_size();
			unsigned bboff = (void*)ps - va_buf(DG);
			err("TBLOCK:[%d] off %d", 
				TBLOCK_INDEX(bboff), TBLOCK_OFFSET(bboff));
			dump(bad_es, 4, 2*sample_size()+ES_SIZE);
		}
	}

	return rc;
}

static unsigned updateSampleInPhaseFromLastES(
	struct TblockListElement *tble, 
	short* ps,
	unsigned sample_in_phase
	)
{
	int doff = TBLOCK_OFFSET((void*)ps - va_buf(DG));

	if (!list_empty(&S_elp_current)){
		struct EventLocator *elp_last = 
			list_entry(S_elp_current.prev, struct EventLocator, list);

		if (tble != elp_last->tble){
			doff += elp_last->tble->tblock->length; 
		}

		doff -= elp_last->offset_in_block;
		if ((elp_last->flags&COMB_ES_FOUND) != 0){
			doff -= ES_WORDS*sizeof(short);
		}

		dbg(2, "previous oib: %d %s doff was %d doff now %d",
			elp_last->offset_in_block,
			(elp_last->flags&COMB_ES_FOUND)? "ES": "--",
			TBLOCK_OFFSET((void*)ps - va_buf(DG)),
			doff);

		if (acq200_debug && tble == elp_last->tble){
			dbg(1, "SAME TBLOCK compute from last Event dsam %d",
			    doff/sample_size());
		}else{
			dbg(1, "NEW TBLOCK, add remaindr prev TBLCK dsam %d",
			    doff/sample_size());
		}
	}

	if (doff < 0){
		info("WARNING: bogus offset %d", doff);
		doff = 0;
	}

	dbg(1, "sip %d return %d", sample_in_phase, 
			sample_in_phase + doff/sample_size());

	return sample_in_phase + doff/sample_size();
}
static int triggerFound(short *ps, struct TriggerHistory *th)
{
	int found = 0;
	short yy = ps[acq200_lookup_pchan(TRIG_CHAN)];

	dbg(3, "ps %p TRIG_CHAN %d phys %d value %04x\nhistory %s",
	    ps, 
	    TRIG_CHAN, acq200_lookup_pchan(TRIG_CHAN), 
	    (unsigned short)yy, TH_HIST(th));
	
	if (!TH_IS_CLEAR(th)){
		if (TRIG_RISING){
			if (yy > TRIG_HI_ABOVE &&
			    TH_ANY_PREVIOUS(th, TH_LT, TRIG_LO_BELOW)){
				found = 1;
				TH_CLEAR(th);
			}
		}else{
			if (yy < TRIG_LO_BELOW &&
			    TH_ANY_PREVIOUS(th, TH_GT, TRIG_HI_ABOVE)){
				found = 1;
				TH_CLEAR(th);
			}
		}
	}

	TH_PUSH(th, yy);

	dbg(3-found, "%p value 0x%04x return %d", 
			ps, (unsigned short)yy, found);

	return found;
}

#define TH_PRIME(thp) \
	TH_PUSH(&th, TRIG_RISING? TRIG_LO_BELOW-1: TRIG_HI_ABOVE+1)

static int comb(
	struct TblockListElement *tble, 
	int search_for_es,
	unsigned *offinblock,			/* bytes from TB start */
	unsigned *sample_in_phase
)
{
#define STRIDE		NCHANNELS   /* ES always on sample boundary */
#define MYTB (tble->tblock)
	short *ps = (short*)(VA_TBLOCK(MYTB) + *offinblock);
	short *ps1 = ps;
	short *pse = (short*)(VA_TBLOCK(MYTB) + MYTB->length - sample_size());

	int tb_off_sam = *sample_in_phase;
	int mss = S_search_limit * sample_size()/sizeof(short);
	unsigned rc = 0;
	struct TriggerHistory th = {};
	int dumps = 0;

	dbg(2, "offinblock 0x%08x sip %d ps %p. max_search %d",
	    *offinblock, *sample_in_phase, ps, S_search_limit);

	if (*sample_in_phase == 0){
		/* sometimes the edge has already been */
		TH_PRIME(&th);
	}

	while(ps - ps1 < mss && ps <= pse){
		unsigned *ups = (unsigned *)ps;

		/** 96 ch case : offinblock can be MISALIGNED!! 
		 *  so if there is potentially an ES, use this
                 *  to realign by backing up to beginning of the ES.
		 */

		if (IS_EVENT_MAGIC(*ups)){
			while(IS_EVENT_MAGIC(ups[-1])){
				--ups;
			}
			if (isES(ups)){
				ps = (short*)ups;

				if (ps == S_last_es){
					dbg(1, "had this before ES %p", ps);
				/** leave it for the correct event record */
					return COMB_ES_FOUND|COMB_ES_ES2;
				}
				S_last_es = ps;

				dbg(1, "COMB_ES_FOUND %p", ps);
				if (!search_for_es){
					dbg(1, "second ES found, (OK) %p", ps);
				}

				if (debug_debug){
					acq200_debug += 3;
				}
				if (acq200_debug >3){
					dump(ps, 2, 32);
					dumps = 0;
				}

				tb_off_sam = updateSampleInPhaseFromLastES(
					tble, ps, *sample_in_phase);
			
				TH_PRIME(&th);

				ps += ES_WORDS;
				rc |= COMB_ES_FOUND;
				continue;
			}
		}

		if (triggerFound(ps, &th)){
			rc |= COMB_PULSE_FOUND;
			
			dbg(1, "COMB_PULSE_FOUND %p", ps);

			break;
		}else{			
			ps += SAMPLE_WORDS;
		}
		tb_off_sam++;
	}

	if ((rc&COMB_ES_FOUND) != 0 && (rc&COMB_PULSE_FOUND) == 0){
		err("ES but no PULSE sample %d start %p end %p", 
		    *sample_in_phase, ps1, ps);
	}
	/** callers pointers updated ONLY if have something to say .. */
	if (rc){
		*offinblock = (void*)ps - VA_TBLOCK(MYTB);
		*sample_in_phase = tb_off_sam;
	}

	dbg(2, "offinblock 0x%08x %d", *offinblock, rc);
	return rc;
#undef MYTB
}

static int findEvent(
	struct TblockListElement *tble, 
	int search_for_es,
	unsigned *offinblock,
	unsigned *sample_in_phase)
/** identify first pulse from ES, on subsequent calls, look for 
 *  additional analog pulses that didn't fire event, to limit of S_search_limit
 */
{
	unsigned rc = comb(tble, search_for_es, offinblock, sample_in_phase);
	dbg(1, "%08x P.%05d comb returned %s %s %s", 
	    *offinblock, S_pulse_count+1,
	    (rc&COMB_ES_FOUND) != 0?"COMB_ES_FOUND":"",
	    (rc&COMB_PULSE_FOUND) !=0 ? "COMB_PULSE_FOUND":"",
	    (rc&COMB_ES_ES2) != 0? "COMB_ES_ES2": "");
	
	if ((rc&COMB_ES_FOUND) && !(rc&COMB_PULSE_FOUND) && 
				!(rc&COMB_ES_ES2)){
		err("we have ES but NO PULSE FOUND offinblock:%u", 
				*offinblock);
	}
	return rc;
		
}
#endif

static void __extractData(
	struct EventLocator *elp,
	short* ps,
	int startsam,
	int copy_samples)
{
	int ch, sample;
	/** copy in 16ch bocks to optimise cache use */
	int nblock;
	int nsamples = elp->nsamples;
	short *dst = elp->data + startsam - nsamples; /* index from 1 */



	dbg(3, "elp %p ps %p start:%d copy %d", 
			elp, ps, startsam, copy_samples);

	for ( ch = 1; ch <= NCHANNELS; ){
		for (nblock = 16; nblock--; ch++){
			int pch = acq200_lookup_pchan(ch);
			for (sample = 0; sample != copy_samples; ++sample){
				dst[ch*nsamples + sample] =
					ps[sample*NCHANNELS_SAMPLE + pch];
			}
		}
	}
}
static void extractData(struct EventLocator *elp)
/* copy out the data into a nice compact bundle */
{
	struct TBLOCK *tblock = elp->tble->tblock;
	short *ps = (short*)(VA_TBLOCK(tblock) + elp->offset_in_block);
	int copy_samples = elp->nsamples;
	int startsam = PRETRIG;
			
	if (startsam){
		unsigned nrewind = PRETRIG * NCHANNELS_SAMPLE;
		int expect_es = (elp->flags&COMB_ES_FOUND) != 0;
		unsigned es_rewind = expect_es? ES_WORDS: 0;
		short *psp = ps;
		int startsam1 = startsam;

		if (nrewind + es_rewind > elp->offset_in_block/sizeof(short)){
			info("WARNING:not enough pre-data, reduce pulse size");
			copy_samples = elp->nsamples -= PRETRIG;
			nrewind = 0;
			goto post_trig;
		}

		if (expect_es){
			/* rewind sample-by-sample, checking es */

			for ( ; startsam1; 
				nrewind -= NCHANNELS_SAMPLE, startsam--){
				if (isES((unsigned*)(psp-ES_WORDS))){
					psp -= ES_WORDS;
					dbg(2, "ES FOUND @ %p", psp);
					break;
					
				}
				psp -= NCHANNELS_SAMPLE;
				__extractData(elp, psp, startsam1, 1);	
			}
		}

		/* no ES in the way, simply rewind and spool the rest */
		psp -= nrewind;
		__extractData(elp, psp, 0, startsam1);
		copy_samples -= PRETRIG;
	}

post_trig:
	__extractData(elp, ps, startsam, copy_samples);
}




static int processEvent(struct EventLocator *elp)
/** first find the event, then copy data to complete elp */
{
	extractData(elp);
	return createFile(elp);
}
static void find_the_pulses(void *start)
{
	/* first, list the PITS in this tblock */
	unsigned bboffset = start - va_buf(DG);
	int iblock = TBLOCK_INDEX(bboffset);
		/* find sample # */
	struct TblockListElement *tble = getTble(DMC_WO->post, iblock);
		/* iterate events in tblock */
#define MYTB tble->tblock

	int event_cursor = -1;
	int search_for_es;

	if (!tble){
		return;
	}


	if (tble->sample_count == 0){
		info("WARNING: tblock[%d] with zero sample count ", iblock);
		return;
	}

	dbg(1, "start: %p bboffset 0x%08x ib %d tble %p MYTB->offset 0x%08x"
	    "MYTB->iblock %d",
	    start, bboffset, iblock, tble, MYTB->iblock, MYTB->offset);



	while(getEventDef(iblock, &event_cursor, &bboffset, &search_for_es)){
		unsigned offinblock1 = bboffset - MYTB->offset;
		unsigned offinblock = offinblock1;
		unsigned limit = S_search_limit*sample_size();

		limit = min(limit, MYTB->length);

		dbg(2, "eventDef bboffset 0x%08x offinblock1 0x%08x", 
		    bboffset, offinblock1);

		while(offinblock - offinblock1 < limit){
			int flags;
			struct EventLocator *elp;

			flags =	findEvent(
				tble, search_for_es, 
				&offinblock, &S_sample_number);

			if ((flags&COMB_ES_ES2) != 0){
				dbg(1, "COMB_ES_ES2, get another Event");
				break;				
			}
			if ((flags&COMB_ES_FOUND) != 0){
				search_for_es = 0;
			}
			if ((flags&COMB_PULSE_FOUND) == 0){
				break;				      
			}
			if (flags == 0){
				info("WARNING: no ES no PULSE offset %x",
				     offinblock);
				break;
			}

			elp = allocData(S_pulse_count+1, tble, offinblock,
					S_sample_number, flags);

			if (!elp){
				err("game over");
				return;
			}

			if (S_stubber&S_STUBBER_PROCESS_EVENT){
				++S_pulse_count;
				continue;
			}
			if (processEvent(elp) == 0){
				++S_pulse_count;
			}else{
				err("processEvent failed, drop out");
				return;
			}
		}
	}
#undef MYTB
}


struct CUSTOM_DATA;
#define GET_CUSTOM_DATA
#define DO_CUSTOM_MATH(a, b, c, d)

static void ppcustom_transform(short *to, short *from, int nwords, int stride)
/*
 * handle data in 2x2 squares
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 */
{
	if (S_first_time){
		onFirstTime();
	}
	dbg(1, "to %p from %p nwords %d stride %d",
	    to, from, nwords, stride);

	find_the_pulses(from);


	S_first_time = 0;
	dbg(1, "finished processing %d events", S_pulse_count);
}


static void bf_transform(short *to, short *from, int nwords, int stride)
{
	short *ps;
	struct TriggerHistory th = {};

	if (S_first_time){
		onFirstTime();
	}
#if 0
	for (ps = from; ps - from < nwords; ps += stride){
		int flags;

		if (triggerFound(ps, &th)){
			unsigned offinblock = ps - from;   /** @@todo */
			flags - COMB_PULSE_FOUND;
			elp = allocData(S_pulse_count+1, 
					tble, offinblock,
					S_sample_number, flags);

			if (!elp){
				err("game over");
				return;
			}
			if (processEvent(elp) == 0){
				++S_pulse_count;
			}else{
				err("processEvent failed, drop out");
				return;
			}
	
		}
	}
#endif
#warning STUB ONLY WORKTODO
}

static ssize_t store_clear(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	/** @@ todo */
	S_first_time = 1;
	return strlen(buf);
}

static DEVICE_ATTR(clear, S_IWUGO, 0, store_clear);



static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       eb_ppcustom_driver_name,
		       eb_ppcustom_driver_string,
		       eb_ppcustom_driver_version,
		       eb_ppcustom_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);


static int mk_ppcustom_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_clear);
	return 0;
}




static int eb_data_open(struct inode *inode, struct file *filp)
{
	struct EventLocator *elp = (struct EventLocator *)inode->i_private;

	filp->private_data = (void*)inode->i_private;

	{
		int index = -1;
		if (elp) index = elp->evnum;
		dbg(1, "i_ino %lu elp %p index %d", inode->i_ino, elp, index);
	}
	dbg(1, "return 0");

	return 0;
}


static ssize_t eb_data_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	struct EventLocator *elp = (struct EventLocator *)filp->private_data;
	int len = ELDATASIZE;

	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, (char*)elp->data + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
}



static int eb_data_mmap(
	struct file *filp, struct vm_area_struct *vma)
{
	struct EventLocator *elp = (struct EventLocator *)filp->private_data;
	return io_remap_pfn_range( 
		vma, vma->vm_start, 
		__phys_to_pfn(virt_to_phys(elp->data)), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
	);
}







#define EB_MAGIC 0xa21600eb

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+2+1)*TD_SZ)
static struct tree_descr *my_files;


static void ebfs_onUnlink(void *clidata)
{
	freeEventLocator((struct EventLocator *)clidata);
}
/*
 * Anytime we make a file or directory in our filesystem we need to
 * come up with an inode to represent it internally.  This is
 * the function that does that job.  All that's really interesting
 * is the "mode" parameter, which says whether this is a directory
 * or file, and gives the permissions.
 */
static struct inode *ebfs_make_inode(struct super_block *sb, int mode)
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
static int ebfs_unlink(struct inode *dir, struct dentry *dentry)
{
	struct inode *inode = dentry->d_inode;

	if (inode){
		dbg(1, "free my data %p", inode);
		ebfs_onUnlink(inode->i_private);
	}else{
		err("WOT? no inode?");
	}

	return simple_unlink(dir, dentry);
}

static struct dentry *ebfs_create_file (
	struct super_block *sb,
	struct dentry *dir, 
	struct file_operations *fops,
	const char *name,
	void *clidata
	)
{
	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;
/*
 * Make a hashed version of the name to go with the dentry.
 */
	qname.name = name;
	qname.len = strlen(name);
	qname.hash = full_name_hash(name, qname.len);
/*
 * Now we can create our dentry and the inode to go with it.
 */
	dentry = d_alloc(dir, &qname);
	if (! dentry)
		goto out;
	inode = ebfs_make_inode(sb, S_IFREG | 0644);
	if (! inode)
		goto out_dput;
	inode->i_fop = fops;
	inode->i_private = clidata;
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



static struct dentry* ebfs_create_dir(struct super_block *sb,
		struct dentry *parent, const char *name)
{
	static struct inode_operations deletable_dir_inode_operations = {
		.lookup		= simple_lookup,
		.rmdir          = simple_rmdir,
		.unlink         = ebfs_unlink
	};

	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;

	qname.name = name;
	qname.len = strlen(name);
	qname.hash = full_name_hash(name, qname.len);
	dentry = d_alloc(parent, &qname);
	if (!dentry)
		goto out;

	inode = ebfs_make_inode(sb, S_IFDIR | 0755);

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


static int createFile(struct EventLocator *elp)
{
	static struct file_operations access_ops = {
		.open = eb_data_open,
		.read = eb_data_read,
		.mmap = eb_data_mmap
	};
	struct dentry *d;

	if (!S_sb){
		err("superblock NOT defined - file system not mounted?");
		return -1;
	}
	if (!S_dir){
		err("top level dir NOT available");
		return -1;
	}
	snprintf(elp->fname, sizeof(elp->fname), 
				"P.%04d.%08d", 
				elp->evnum,
				elp->sample_in_phase);

	d = ebfs_create_file(S_sb, S_dir, &access_ops, elp->fname, elp);

	if (d == 0){
		err("failed to create file %s", elp->fname);
		return -1;
	}

	d->d_inode->i_size = ACTUAL_DATASIZE(elp);
	return 0;
}


static struct dentry *my_root;

static int createShotDir(void)
{
	if (!S_sb){
		err("superblock NOT defined - file system not mounted?");
		return -1;
	}
	if (!my_root){
		my_root = ebfs_create_dir(S_sb, S_sb->s_root, "EB");
	}

	snprintf(S_dname, 16, "SHOT.%06d", DG->shot);
	S_dir = ebfs_create_dir(S_sb, my_root, S_dname);
	return 0;
}


static int ver_open(struct inode *inode, struct file *file)
{
	dbg(1, "return 0");
	return 0;
}



static ssize_t ver_read(struct file *file, char *buf,
		size_t count, loff_t *offset)
{
	char *src = eb_ppcustom_driver_version;
	int len = strlen(src);
	ssize_t rc = 0;

	if (*offset < len){
		if (count > len - *offset){
			count = len - *offset;
		}
		if (copy_to_user(buf, src + *offset, count)){
			rc = -EFAULT;
		}else{
			*offset += count;
			rc = count;
		}
	}
	dbg(1, "return %d", rc);	
	return rc;
}

static int ver_release(
        struct inode *inode, struct file *file)
{
	dbg(1, "return 0");
	return 0;
}


static int eb_ppcustom_fill_super (
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations ver_ops = {
		.open = ver_open,
		.read = ver_read,
		.release = ver_release
	};
	static struct tree_descr static_files[] = {
		{ NULL, NULL, 0	},
		{ .name = "version", .ops = &ver_ops, .mode = S_IRUGO },
		{ "", NULL, 0	},		
	};
	S_sb = sb;
	return simple_fill_super(sb, EB_MAGIC, static_files);
}




static int eb_ppcustom_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, 
	void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(
			fst, flags, data, 
			eb_ppcustom_fill_super, mnt);
}

static struct file_system_type custom_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "ebfs",
	.get_sb		= eb_ppcustom_get_super,
	.kill_sb	= kill_litter_super,
};

static void mk_ppcustom_fs(void)
/* store results as nodes in a custom file system */
{
	register_filesystem(&custom_fs_type);
}

static void rm_ppcustom_fs(void)
{
	unregister_filesystem(&custom_fs_type);
	freeData();
	kfree(my_files);
}

static void eb_ppcustom_dev_release(struct device * dev)
{
	info("");
}


static void eb_init_statics(void)
{
	INIT_LIST_HEAD(&S_elp_current);
	INIT_LIST_HEAD(&S_elp_fs);
}

static struct device_driver eb_ppcustom_driver;

static struct Transformer bf_transformer = {
	.name = "bf_search",
	.transform = bf_transform,
	.t_flags = TF_RESULT_IS_RAW|TF_INPLACE
};

static 	struct Transformer transformer = {
	.name = "ppcustom",
	.transform = ppcustom_transform,
	.t_flags = TF_RESULT_IS_RAW|TF_INPLACE
};

static int eb_ppcustom_probe(struct device *dev)
{
	int it;
	info("");

	it = acq200_registerTransformer(&transformer);

	info("transformer registered %d", it);

	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer NOT registered");
	}

	it  = acq200_registerTransformer(&bf_transformer);
	if (it < 0 ){
		err("transformer NOT registered");
	}
	eb_init_statics();
	mk_ppcustom_fs();
	mk_ppcustom_sysfs(dev);

	dbg(1, "99");
	return 0;
}

static int eb_ppcustom_remove(struct device *dev)
{
	rm_ppcustom_fs();

	acq200_unregisterTransformer(&transformer);
	acq200_unregisterTransformer(&bf_transformer);
	return 0;
}


static struct device_driver eb_ppcustom_driver = {
	.name     = "eb_ppcustom",
	.probe    = eb_ppcustom_probe,
	.remove   = eb_ppcustom_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device eb_ppcustom_device = {
	.name = "eb_ppcustom",
	.id   = 0,
	.dev = {
		.release    = eb_ppcustom_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init eb_ppcustom_init( void )
{
	int rc;
	acq200_debug = eb_ppcustom_debug;

	rc = driver_register(&eb_ppcustom_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&eb_ppcustom_device);
}


static void __exit
eb_ppcustom_exit_module(void)
{
	info("");
	platform_device_unregister(&eb_ppcustom_device);
	driver_unregister(&eb_ppcustom_driver);
}

module_init(eb_ppcustom_init);
module_exit(eb_ppcustom_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ216 Custom Postprocessing");


