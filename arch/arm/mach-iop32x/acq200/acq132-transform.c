/* acq132-transform.c acq132 transforms and timebase                         */
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

/** @todo  handle multirate! */
#define DEBUG 1

#define DTACQ_MACH 2
#define ACQ132
#define ACQ_IS_INPUT 1

#define MODEL_VERID							\
	"$Id: acq132-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $ B1020\n"

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


int stub_transform;
module_param(stub_transform, int, 0664);

int acq132_transform_debug = 0;
module_param(acq132_transform_debug, int, 0664);

int es_tblock;
module_param(es_tblock, int, 0444);

int event_offset_samples = 0;
module_param(event_offset_samples, int, 0644);

int show_timebase_calcs = 0;
module_param(show_timebase_calcs, int, 0600);

int time_stamp_size = sizeof(u32);
module_param(time_stamp_size, int, 0444);

int time_stamp_adj = 2;
module_param(time_stamp_adj, int, 0644);

int stub_acq132_find_event = 0;
module_param(stub_acq132_find_event, int, 0644);

/*
int xxs_readsam[NCB];
module_param_array(xxs_readsam, int, NULL, 0644);
*/
int acq132_es_cold_samples[8];
module_param_array(acq132_es_cold_samples, int, NULL, 0444);

#define TIMEBASE_ENCODE_GATE_PULSE_INDEX_MOD256 0
#define TIMEBASE_ENCODE_GATE_PULSE		1



int timebase_encoding = 0;
module_param(timebase_encoding, int, 0644);

/* -1: advance, 0: ignore, 1: retard */
int control_event_adjust = -1;
module_param(control_event_adjust, int, 0644);

/* 2 sample pipeline, could be more: eg with FIR found empirically */
/* [decim==1][decim!=1] */
int es_cold_offset_samples[2] = { -4, 2 };
module_param_array(es_cold_offset_samples, int, NULL, 0644);


/* @TODO: make this variable 4byte, 8 byte 4byte = 4s worth of nsecs */

extern int stub_event_adjust;

#define TBG if (acq132_transform_debug) dbg

/* tbxo : encodes tblock index {31:24} offset {23:0} */

typedef u32 Tbxo;

struct ES_DESCRIPTOR {
	void *va;
	Tbxo tbxo;
	u32 ts;
	u32 spare;
};

static struct ES_META_DATA{
	TBLE* es_tble;
	TBLE* es_deblock;	/* to go when done at source */
	struct ES_DESCRIPTOR *es_base;
	struct ES_DESCRIPTOR *es_cursor;
}
	g_esm;

static Tbxo G_current_transform_tbxo;	/* hack avoids extra call prams */
static int G_rows;


#define DEBUGGING
#ifdef DEBUGGING
short* to1;
short* to2;
short* from1;
short* from2;
#endif

/* cursor: encode as TBLOCK{31:24}, OFFSET{23:0} */


#define MAXTBLOCKS (DG->bigbuf.tblocks.nblocks)
#define TBLOCKCHSZ (MAXTBLOCKS * sizeof(ChannelData))
#define NOTINMASK 0xffffffff

typedef unsigned ChannelData[MAXCHAN];

typedef unsigned * pUNS;

struct Bank {
	int nchan;	/* 2, 4, 8 */
	unsigned id;
	ChannelBankCursors bk_channelCursors;
};


/* encoding tbxo tblock index, offset */


#define TBOFF(tb)	((tb)&~0xff000000U)
#define TBIX(tb)	((tb) >> 24)

#define TO_TBIX(ix)	((ix) << 24)


static void * tbxo2va(Tbxo tb)
{
	unsigned tbix = TBIX(tb);
	unsigned tboff = TBOFF(tb);

	return va_buf(DG) + 
		DG->bigbuf.tblocks.the_tblocks[tbix].offset + tboff;
}


static Tbxo va2tbxo(void *va)
{
	unsigned offset = va - va_buf(DG);
	unsigned tbix = TBLOCK_INDEX(offset);
	unsigned tboff = TBLOCK_OFFSET(offset);
	Tbxo tbxo = tbix<<24 | tboff;

	if (acq132_transform_debug){
		void* va2 = tbxo2va(tbxo);
		dbg(1, "va:%p va2:%p tbxo:%08x %s",
		    va, va2, tbxo, va==va2? "EQUAL": "ERROR: NOT EQUAL");
	}

	return tbxo;
}

struct TBLOCK* getTblock(Tbxo tb)
{
	return &DG->bigbuf.tblocks.the_tblocks[TBIX(tb)];
}


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

		TBG(3, "block:%d to:%p from:%p bsamples %d", 
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
		TBG(3, "block:%d to:%p from:%p", block, to, from);
	}

	TBG(1, "99");
}

/* It makes sense to build the ES detect and remove into the transform 
 * because the DQAD is already in cache. ie the cost of search is small
 */



/* todo assumes TS is clocking at intclock rate */
extern int intclock_actual_rate;
#define NS		1000000000

static inline unsigned getTsclkNs(void)
{
	unsigned tshz = acq200_clk_hz/acq132_get_prescale();
	return NS/max(1U,tshz);
}


#define ES_MAGIC_WORD 0xaa55

struct ES_INFO {
	struct ES_DESCRIPTOR *esi_base;   /* base of ES vector */
	struct ES_DESCRIPTOR *esi_cursor; /* start-> base of ES vector */
	struct ES_DESCRIPTOR *esi_end;
	int nstamps;		/* length of vector */
	int tb_total;		/* cumulative samples so far */
};

#define ESI_BURST_START(esi)	((esi).esi_base[(esi).itb+ESC_OFFSET])

#define ES_LEN		(ROW_CHAN*SWS)
#define ESI(file)	((struct ES_INFO *)file->private_data)

/* @@todo NROWS = f(scan list) */
#define NROWS		MAX_ROWS	
#define ROW_SAMPLE_SIZE	(ROW_CHAN*SWS)
#define ROW_ES_SIZE	ROW_SAMPLE_SIZE
#define BLOCK_BYTES	(ROW_SIZE*NROWS)
/* there may be NROWS, but the #samples is the same .. */
#define BLOCK_SAM	ROW_SAM	


static int tb_get_blen_same_tblock(Tbxo start, Tbxo end)
{
	unsigned d_bytes = end - start;
	unsigned d_sam;

	d_bytes -= ROW_ES_SIZE;
	d_sam = d_bytes / ROW_SAMPLE_SIZE;	

/* @@todo : what about across tblocks? */

	dbg(2, "start:%10u end:%10u db:%8u return %d", 
			start, end, d_bytes, d_sam);

	return d_sam;
#undef PRTVAL
}

static int count_span_blocks_in_phase(int tb1, int tb2)
/* return # of intervening blocks for case of very long pulse */
{
	struct Phase *phase;
	struct TblockListElement* tle;
	int in_the_span = 0;
	int span = 0;

	list_for_each_entry(phase, &DMC_WO->phases, list){
		list_for_each_entry(tle, &phase->tblocks, list){
			if (!in_the_span){
				if (tle->tblock->iblock == tb1){
					in_the_span = 1;
				}
			}else{
				if (tle->tblock->iblock == tb2){
					in_the_span = 0;
					break;
				}else{
					++span;
				}
			}
		}
	}
	return span;
}
static int tb_get_blen_other_tblock(Tbxo tbix1, Tbxo tbix2)
/* we're ASSUMING maxlen < TBLOCK_LEN or 98304 samples */
{
	u32 tb1_bytes = TBLOCK_LEN(DG)/NROWS - TBOFF(tbix1);
	u32 tb2_bytes = TBOFF(tbix2);
	unsigned d_bytes = tb1_bytes + tb2_bytes;
	int span_blocks;
	unsigned d_sam;

	span_blocks = count_span_blocks_in_phase(TBIX(tbix1), TBIX(tbix2));
	if (span_blocks){
		d_bytes += span_blocks * TBLOCK_LEN(DG)/NROWS;
	}

	d_bytes -= ROW_ES_SIZE;
	d_sam = d_bytes / ROW_SAMPLE_SIZE;	

/* @@todo : what about across tblocks? */

	dbg(2, "start:%08x end:%08x 1:%10u 2:%10u db:%8u return %d", 
	    tbix1, tbix2, tb1_bytes, tb2_bytes, d_bytes, d_sam);

	return d_sam;
#undef PRTVAL
}



static int tb_get_blen(Tbxo start, Tbxo end)
{
	if (likely(TBIX(start) == TBIX(end))){
		return tb_get_blen_same_tblock(start, end);
	}else{
		return tb_get_blen_other_tblock(start, end);
	}
}




static int already_known(unsigned ts)
/* search back towards g_esm.base to see if ts already found 
	we're assuming that timestamps are always increasing ...
*/
{
	if (g_esm.es_cursor - g_esm.es_base > 1){
		unsigned tsm1 = g_esm.es_cursor[-1].ts;
		dbg(3, "ts:%10u tsm1:%10u %s", ts, tsm1,
		    ts <= tsm1? "YES": "NO - this is new");
		return ts <= tsm1;
	}

	return 0;
}


static void stash_es_cold_samples(unsigned short *ch)
{
	acq132_es_cold_samples[0] = ACQ132_ES_COLD_SAMPLES(ch[2]);
	acq132_es_cold_samples[1] = ACQ132_ES_COLD_SAMPLES(ch[3]);
			
	acq132_es_cold_samples[2] = ACQ132_ES_COLD_SAMPLES(ch[6]);
	acq132_es_cold_samples[3] = ACQ132_ES_COLD_SAMPLES(ch[7]);
}		

extern void copyDiags(unsigned* searchp, unsigned ileft);
extern void initPhaseDiagBufFound(
	short valid, int matches, 
	u32 last_before, u32 first_es, u32 last_es, u32 first_after,
	u32 first, u32 last
	);

int remove_es(int sam, unsigned short* ch, void *before)
/* NB: cursor points to start of pulse, AFTER ES */
{
	const char* id = "none";
	int rc = DG->show_event == 0; /* remove unless show_event */
	
	if (g_esm.es_cursor){	
		unsigned ts = ch[5] << 16 | ch[7];
		if (already_known(ts)){
			/* catch close bunched TS */
			id = "already_known";
		}else{
/* before is the exact position in the temporary buffer
 * but we need to record what the exact position was in the original buffer
 */      
			Tbxo pos_temp = va2tbxo(before);
			struct ES_DESCRIPTOR es_descr;

			es_descr.tbxo = TO_TBIX(TBIX(G_current_transform_tbxo))
						|TBOFF(pos_temp);

			es_descr.va = before;	/* dubious */
/* should be:
			es_descr.va =  tbxo2va(es_descr.va);
  ... why bother with the va field at all?
*/						
      		      

			es_descr.ts = ts;
			es_descr.spare = 0;
			*g_esm.es_cursor++ = es_descr;
			id = "remove";

			dbg(2, "TBIX: %d TBOFF :%d before:%p",
			    TBIX(es_descr.tbxo), TBOFF(es_descr.tbxo), before);

			copyDiags((unsigned *)ch, 0);
			initPhaseDiagBufFound(1, 1, 0, 0, 0, 0, 0, 0);
			stash_es_cold_samples(ch);

			if (rc == 1){	
				struct TBLOCK* tb = 
					getTblock(G_current_transform_tbxo);
				tb->tb_length -= sample_size();
			}
		}

		dbg(1, 
		    "sam:%6d ts:%8u "
		    "%04x %04x %04x %04x %04x %04x %04x %04x %s ret:%d",
		    sam, ts, 
		    ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
		    id,	rc);
	}else{
		err("es_cursor is null");
		rc = 0;
	}

	return rc;
}

static void timebase_debug(void)
{
	struct ES_DESCRIPTOR *cursor = g_esm.es_base;
	struct ES_DESCRIPTOR burst_start = *cursor;
	int iburst = 0;

	if (burst_start.ts != 0){
		info("first timestamp %08x use as offset",burst_start.ts);
	}
	
	while(++cursor < g_esm.es_cursor){
		iburst++;

	        info("burst [%4d] ts:0x%08x len:%d tb_get_blen:%d",
		     iburst, cursor->ts, 
		     (cursor->va - burst_start.va)/sample_size(),
		     tb_get_blen(burst_start.tbxo, cursor->tbxo));
		burst_start = *cursor;
	}
}



int acq132_transform_row_es(
	short *to, 
/*      short *to[ROW_CHAN], */
	short *from, int nsamples, int channel_sam)
/* read a1 block of data from to and farm 8 channels to from */
{
	union {
		unsigned long long ull[ROW_CHAN/4];
		unsigned short ch[ROW_CHAN];
	} buf;
	unsigned long long *full = (unsigned long long *)from;
	int sam;
	int tosam = 0;
	int chx;

	TBG(3, "to:%p from:%p nsamples:%d channel_sam:%d",
	    to, from, nsamples, channel_sam);

	if (stub_transform){
		TBG(3, "stub");
		return nsamples;

	}
	for (sam = nsamples; sam != 0; --sam){
		void *before = full;
		buf.ull[0] = *full++;
		buf.ull[1] = *full++;

		/* rapid fail ES_MAGIC test 
		 aa55 aa55 0000 0000 aa55 aa55 0000 0000
		 */
		if (buf.ch[0] == ES_MAGIC_WORD &&
		    buf.ch[1] == ES_MAGIC_WORD &&
		    buf.ch[4] == ES_MAGIC_WORD &&
		    buf.ch[5] == ES_MAGIC_WORD &&
		    /* remove pre-increment from cursor */
			remove_es(nsamples-sam, buf.ch, before)){
			
			continue;
		}
#ifdef DEBUGGING
		if ((short*)full < from1 || (short*)full > from2){
			err("from outrun at  %p %p %p sam:%d",
			    from1, full, from2, sam);
			return nsamples;
		}
#endif

		for (chx = 0; chx < ROW_CHAN; ++chx){

#ifdef DEBUGGING
			short *pto = to + chx*channel_sam + tosam;
			if (pto < to1 || pto > to2){
				err("buffer outrun at %p %p %p chx:%d tosam:%d",
				    to1, pto, to2, chx, tosam);
				return nsamples;
			}
#endif
			to[chx*channel_sam + tosam] = buf.ch[chx];
/* change to splitter function to handle masks:
			to[chx][tosam] = buf.ch[chx];
*/
			
		}	
		++tosam;		
	}

	TBG(3, "99");	
	return tosam;
}



#define DQ_BLOCK_OFF(blk) (blk*TBLOCK_LEN(DG)/4)

static void acq132_transform_unblocked(
	short *to, short *from, int nwords, int stride)
{
/* keep a stash of NSCAN to vectors */
	const int nsamples = nwords/stride;	
	const int rows = stride/ROW_CHAN;
#define ROW_OFF(r)	((r)*ROW_CHAN*nsamples) 
	int row_off[MAX_ROWS];
	int row;

	if (rows > MAX_ROWS){
		err("rows %d > MAX_ROWS", rows);
		return;
	}

	G_rows = rows;

	for (row = 0; row < rows; ++row){
		row_off[row] = DQ_BLOCK_OFF(row)/sizeof(short);
	}

	TBG(1, "nsamples:%d", nsamples);

#ifdef DEBUGGING
	to1 = to;
	to2 = to+nwords;
	TBG(1, "to   %p to %p", to1, to2);
	from1 = from;
	from2 = from+nwords;
	TBG(1, "from %p to %p", from1, from2);
#endif


	for (row = 0; row < rows; ++row){
		row_off[row] += acq132_transform_row_es(
			to + row_off[row], 
			from, 
			nsamples,
			nsamples
			);
		from += nsamples*ROW_CHAN;
	}

	TBG(1, "99");
}

static void* acq132_deblock(short * const from, int nwords, int stride)
{
	void* const to = BB_PTR(g_esm.es_deblock->tblock->offset);
	void *frm = from;
	int row_off[MAX_ROWS];
	int block;

	dbg(1, "00 block: %03d", TBLOCK_INDEX(frm-BB_PTR(0)));

	for (block = 0; block != 4; ++block){
		row_off[block] = DQ_BLOCK_OFF(block);	
		dbg(1+block, "01b:%d", row_off[block]);
	}
	
	while(nwords > 0){
		for (block = 0; block != 4; ++block){

			dbg(4, "b:%d memcpy(%p, %p, %d)",
			    block, to+row_off[block], frm, ROW_SIZE);

			memcpy(to+row_off[block], frm, ROW_SIZE);
			row_off[block] += ROW_SIZE;
			frm += ROW_SIZE;
			nwords -= ROW_SIZE/SWS;
		}
	}

	/* ES detection needs to know TBLOCK info ... this copy
	 * could be a DMA of course, or, deblock should be done at source
         * then this function can disappear
	 */
	for (block = 0; block != 4; ++block){
		dbg(1+block, "99b:%d", row_off[block]);
	}
	return to;

}
static void acq132_transform_es(short *to, short *from, int nwords, int stride)
{
	dbg(1, "to:tblock:%d from:tblock:%d", 
	    TBLOCK_INDEX((void*)to - va_buf(DG)),
	    TBLOCK_INDEX((void*)from - va_buf(DG)));

	G_current_transform_tbxo = va2tbxo(from);

	acq132_transform_unblocked(
		to, acq132_deblock(from, nwords, stride), nwords, stride);
}

static 	struct Transformer transformer = {
	.name = "acq132",
	.transform = acq132_transform
};

static struct Transformer transformer_es = {
	.name = "acq132es",
	.transform = acq132_transform_es
};

static int transformSelected(void *fun, const char* name)
{
	int selected = acq200_getSelectedTransformerFunc() == fun;
	dbg(1, "%s %s", name, selected? "YES": "NO");
	return selected;
}

#define TRANSFORM_SELECTED(fun) transformSelected(fun, #fun)

static TBLE *reserveFreeTblock(const char *id){
	TBLE *tble = acq200_reserveFreeTblock();
	if (tble == 0){
		err("%s:failed to reserve TBLOCK", id);
		return 0;
	}else{
		info("reserved %10s TBLOCK %d", id, tble->tblock->iblock);
	}
	atomic_inc(&tble->tblock->in_phase);
	return tble;
}
static void transformer_es_onStart(void *unused)
/* @TODO wants to be onPreArm (but not implemented) */
{
	if (!TRANSFORM_SELECTED(acq132_transform_es)){
		return;
	}else{

		if (g_esm.es_tble == 0 &&
			(g_esm.es_tble = reserveFreeTblock("ES")) == 0){
			return;
		}else{
			es_tblock = g_esm.es_tble->tblock->iblock;
		}
		if (g_esm.es_deblock == 0 &&
			(g_esm.es_deblock = reserveFreeTblock("DEBLOCK")) == 0){
			return;
		}

		g_esm.es_base = 
		g_esm.es_cursor = (struct ES_DESCRIPTOR*)
				BB_PTR(g_esm.es_tble->tblock->offset);
		dbg(1, "g_esm.es_cursor reset %p", g_esm.es_base);
		memset(g_esm.es_cursor, 0, TBLOCK_LEN(DG));
		memset(acq132_es_cold_samples, 0, 
		       sizeof(acq132_es_cold_samples));
	}
}

static struct Hookup transformer_es_hook = {
	.the_hook = transformer_es_onStart
};




void acq132_register_transformers(void)
{
	int it;
	it = acq200_registerTransformer(&transformer);
	if (it >= 0){
		acq200_add_start_of_shot_hook(&transformer_es_hook);
		acq200_registerTransformer(&transformer_es);
		acq200_setTransformer(it);
	}else{
		err("transformer %s NOT registered", transformer.name);
	}
}




extern int findEvent(struct Phase *phase, unsigned *first, unsigned *ilast);

#define ES_COLD_OFFSET_SAMPLES	\
	es_cold_offset_samples[get_acq132_decim() != 1]	

/* What if tblock is slightly shorter - because we removed an ES?
 * tblock->length appears to be constant, not a reflection of #samples
 */

static int acq132_get_event_adjust(void)
{
	int cold_adj_samples = 0;
	int event_offset_bytes;

	if (control_event_adjust){
		int cold_sam = acq132_es_cold_samples[0];
		int ii;
		int mismatch = 0;
		int total = cold_sam;

		for (ii = 1; ii <= 3; ++ii){
			if (cold_sam != acq132_es_cold_samples[ii]){
				err("cold samples mismatch");
				mismatch = 1;
			}
			total += acq132_es_cold_samples[ii];
		}
		cold_sam = total/4 + ES_COLD_OFFSET_SAMPLES;

		dbg(1-mismatch, "%2d %2d %2d %2d, mean %2d",
		    acq132_es_cold_samples[0],	
		    acq132_es_cold_samples[1],	
		    acq132_es_cold_samples[2],	
		    acq132_es_cold_samples[3],	
		    cold_sam);
			
		cold_adj_samples = cold_sam * control_event_adjust;
	}

	event_offset_bytes = 
		(event_offset_samples + cold_adj_samples) * sample_size();

	return event_offset_bytes;
}
static unsigned acq132_adjust_event(unsigned es_tboff)
{
	int event_offset_bytes = acq132_get_event_adjust();

	if (event_offset_bytes > 0){
		if ((es_tboff += event_offset_bytes) > TBLOCK_LEN(DG)){
			es_tboff = TBLOCK_LEN(DG);
			err("Best effort: TBLOCK_round down");
		}				       
	}else if (event_offset_bytes < 0){
		if (likely(es_tboff > ABS(event_offset_bytes))){
			es_tboff += event_offset_bytes;
		}else{
			es_tboff = 0;
			err("Best effort: TBLOCK truncate");
		}
	}

	return es_tboff;
}



int _acq132_fix_event_already_found(
	struct Phase *phase, unsigned *first, unsigned *ilast)
{
	unsigned search_tbix = TBLOCK_INDEX(*first);
	unsigned es_tbix = TBIX(g_esm.es_base->tbxo);

	if (search_tbix == es_tbix){
		unsigned es_tboff = TBOFF(g_esm.es_base->tbxo)*G_rows;
		unsigned tb_offset = 
			DG->bigbuf.tblocks.the_tblocks[es_tbix].offset;

		dbg(1, "TBIX match %d let's do it, first was %d", 
		    search_tbix, *first);

		es_tboff = acq132_adjust_event(es_tboff);

		*first = tb_offset + es_tboff;
		/* we already removed the ES .. */
		*ilast = tb_offset + es_tboff;

		dbg(1, "*first %d tblock: %d off-in-block 0%06x",
		    *first, TBLOCK_INDEX(*first), 
		    TBLOCK_OFFSET(*first));
		dbg(1, "*first %u *ilast %u", *first, *ilast);
		return 1;
	}else{
		dbg(1, "HAVE ES but not my tblock: search:%d es:%d",
		    search_tbix, es_tbix);
		return 0;
	}
}


int acq132_find_event(
	struct Phase *phase, unsigned *first, unsigned *ilast)
/* search next DMA_BLOCK_LEN of data for event word, 
 * return 1 and update iput if found 
 * This is now all a bit bogus, 
	since we already found the event during Transform
 */
{
	dbg(1, "*first %d tblock: %d off-in-block 0%06x",
	    *first, TBLOCK_INDEX(*first), TBLOCK_OFFSET(*first));

	/* ACQ132 - may work better to adjust event AFTER transform */
	if (stub_acq132_find_event == 0 && 
		g_esm.es_cursor != 0 && g_esm.es_cursor - g_esm.es_base >= 1){

		return _acq132_fix_event_already_found(phase, first, ilast);
	}else{
		return findEvent(phase, first, ilast);
	}
}


static void init_esi(struct file * file)
{
	struct ES_INFO *esi = ESI(file);
	esi->esi_cursor = 
	esi->esi_base = g_esm.es_base;
	esi->esi_end = g_esm.es_cursor;
	esi->nstamps = g_esm.es_cursor - g_esm.es_base;
	dbg(1, "init entries %d", esi->nstamps);
}

static int acq132_timebase_open(struct inode *inode, struct file *file)
{
	file->private_data = kzalloc(sizeof(struct ES_INFO), GFP_KERNEL);
	
	init_esi(file);

	if (show_timebase_calcs) timebase_debug();
	return 0;
}

//#define TSTYPE unsigned long long
#define TSTYPE u32

/*
 * timebase structure in memory
 *
unsigned[] :
[0] = 0
[1] = [0].OFFSET
[2] = [1].TS
[3] = [2].OFFSET
[4] = [3].TS

Read strategy:
	limit read to points in current burst.

First time:
	set itb = 1, set last_end = [0].OFFSET

Iterate:
	search to TB[+2] to calculate TBLEN
        decide if in this range or not.

TS encoding : 
{31:08} Time of gate in usec 
{7:0} : sample in gate%255

Read:

*/


#define TS_ENCODE_GATE_PULSE_INDEX_MOD256(gate_time, sample_in_gate) \
	(((gate_time) << 8) | ((sample_in_gate)&0x0ff))

#define TS_ENCODE(gate_time, sample_in_gate) \
	(timebase_encoding==TIMEBASE_ENCODE_GATE_PULSE ? gate_time: \
	 TS_ENCODE_GATE_PULSE_INDEX_MOD256(gate_time, sample_in_gate))


static ssize_t acq132_timebase_read_copy(
	char *buf, size_t len, u32 gate_time, u32 sample_in_gate)
{
	int ncopy = 0;
	int sample = 0;

	while(ncopy < len){
		TSTYPE ts = TS_ENCODE(gate_time, sample_in_gate);
		if (copy_to_user(buf+ncopy, &ts, sizeof(TSTYPE))){
			return -EFAULT;
		}else{
			++sample_in_gate;
			ncopy += sizeof(TSTYPE);
			++sample;
		}
	}	

	return sample;
}

static ssize_t acq132_timebase_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	struct ES_INFO esi = *ESI(file);
	int sample = *offset;
	int maxsamples = SAMPLES;
	int last = 0;
	int copysam;
	struct ES_DESCRIPTOR* searchp = esi.esi_cursor;

	dbg(1, "kickoff with offset (sample) %d", sample);

	if (sample >= maxsamples){
		return 0;
	}

	while ( searchp++ < esi.esi_end - 1 ){
		/* pick last_end from NEXT record */
		unsigned blen = tb_get_blen(esi.esi_cursor->tbxo,searchp->tbxo);

		if (sample < (last = esi.tb_total + blen)){
			goto in_burst;
		}else{
			++esi.esi_cursor;
			esi.tb_total += blen;
		}
	}

	last = maxsamples;

in_burst:
	dbg(1, "ok: here with sample:%d last:%d", sample, last);
	dbg(1, "ok: esi.itb: %d esi.tb_total: %d", 
			esi.esi_cursor-esi.esi_base, esi.tb_total);

	last = min(last, maxsamples);

	copysam = acq132_timebase_read_copy(buf, 
			min((last-sample)*sizeof(TSTYPE), len),
			esi.esi_cursor->ts - time_stamp_adj, 
			sample - esi.tb_total);

	*ESI(file) = esi;
	*offset = sample + copysam;

	dbg(1, "99 cursor: %d returning %d",esi.esi_cursor-esi.esi_base,
	    copysam*sizeof(TSTYPE));

	return copysam * sizeof(TSTYPE);
}
static ssize_t acq132_timebase_release(struct inode *inode, struct file *file)
{
	kfree(ESI(file));
	return 0;
}
struct file_operations acq132_timebase_ops = {
	.open = acq132_timebase_open,
	.read = acq132_timebase_read,
	.release = acq132_timebase_release
};

