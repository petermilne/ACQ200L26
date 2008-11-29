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

#define DTACQ_MACH 2
#define ACQ132
#define ACQ_IS_INPUT 1

#define MODEL_VERID							\
	"$Id: acq132-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $ B1012\n"

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
#include "acq196-offset.h"

int stub_transform;
module_param(stub_transform, int, 0664);

int acq132_transform_debug = 0;
module_param(acq132_transform_debug, int, 0664);

int es_stash_full;
module_param(es_stash_full, int, 0600);

int es_tblock;
module_param(es_tblock, int, 0444);

/* event_adjust_delta_blocks = 1; // the one true value that works */
int event_adjust_delta_blocks = 1;
module_param(event_adjust_delta_blocks, int, 0600);

int mark_event_data = 0;
module_param(mark_event_data, int, 0600);

extern int stub_event_adjust;

#define TBG if (acq132_transform_debug) dbg

static TBLE* es_tble;
static unsigned *es_cursor;

#define DEBUGGING
#ifdef DEBUGGING
short* to1;
short* to2;
short* from1;
short* from2;
#endif


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

		TBG(2, "block:%d to:%p from:%p bsamples %d", 
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
		TBG(2, "block:%d to:%p from:%p", block, to, from);
	}

	TBG(1, "99");
}

/* It makes sense to build the ES detect and remove into the transform 
 * because the DQAD is already in cache. ie the cost of search is small
 */

#define ES_MAGIC_WORD 0xaa55


int remove_es(int sam, int row, unsigned short* ch)
{
	dbg(1, "sam:%6d row:%d %04x %04x %04x %04x %04x %04x %04x %04x",
	    sam, row, ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7]);

	if (es_cursor){	
		if (es_stash_full){
			memcpy(es_cursor, ch, ROW_CHAN_SZ);
			es_cursor += ROW_CHAN_LONGS;
		}else{
			unsigned ts = ch[5] << 16 | ch[7];
			if (ts != *es_cursor){
				*++es_cursor = (unsigned)ch;
				*++es_cursor = ts;
			}
		}
	}
	return 1;
}

static void transformer_es_onStart(void *unused)
/* @TODO wants to be onPreArm (but not implemented) */
{
	if (!es_tble){
		es_tble = acq200_reserveFreeTblock();
		if (es_tble == 0){
			err("failed to reserve ES TBLOCK");
		}else{
			es_tblock = es_tble->tblock->iblock;
			info("reserved ES TBLOCK %d", es_tblock);
		}
	}

	if (es_tble){
		es_cursor = (unsigned*)BB_PTR(es_tble->tblock->offset);
		memset(es_cursor, 0, TBLOCK_LEN);
	}
}

int acq132_transform_row_es(
	int row,
	short *to, short *from, int nsamples, int channel_sam)
/* read a block of data from to and farm 8 channels to from */
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
		buf.ull[0] = *full++;
		buf.ull[1] = *full++;

		if (buf.ch[0] == ES_MAGIC_WORD &&
			    buf.ch[1] == ES_MAGIC_WORD &&
			    buf.ch[2] == ES_MAGIC_WORD &&
			    buf.ch[3] == ES_MAGIC_WORD &&
			    remove_es(nsamples-sam, row, buf.ch)){
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
				err("buffer outrun at %p %p %p chx:%d sam:%d",
				    to1, pto, to2, chx, tosam);
				return nsamples;
			}
#endif
			to[chx*channel_sam + tosam] = buf.ch[chx];
		}	
		++tosam;		
	}

	TBG(3, "99");	
	return tosam;
}
static void acq132_transform_es(short *to, short *from, int nwords, int stride)
{
	const int nsamples = nwords/stride;	
	const int rows = stride/ROW_CHAN;
	const int block_words = rows * ROW_CHAN * ROW_SAM;
	int blocks = nwords/block_words;
	int nw = nwords;
	int block;
#define ROW_OFF(r)	((r)*ROW_CHAN*nsamples) 
	int row_off[MAX_ROWS];
	int row;

	if (rows > MAX_ROWS){
		err("rows %d > MAX_ROWS", rows);
		return;
	}
	for (row = 0; row < rows; ++row){
		row_off[row] = ROW_OFF(row);
	}

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

		TBG(2, "block:%d to:%p from:%p bsamples %d", 
		    block, to, from, bsamples);

		for (row = 0; row < rows; ++row){
			row_off[row] += acq132_transform_row_es(
				row,
				to + row_off[row], 
				from, 
				bsamples,
				nsamples
				);
			from += bsamples*ROW_CHAN;
		}
		TBG(2, "block:%d to:%p from:%p", block, to, from);
	}

	TBG(1, "99");
}

static 	struct Transformer transformer = {
	.name = "acq132",
	.transform = acq132_transform
};

static struct Transformer transformer_es = {
	.name = "acq132es",
	.transform = acq132_transform_es
};

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


static void check_first_row(unsigned* first, unsigned* last)
{
	unsigned *searchp = (unsigned*)va_buf(DG);
	int ifirst = *first / USS;
	int nblocks = acq132_getScanlistLen();
	int block;

	if (TBLOCK_OFFSET(*first) > ROW_SIZE){
		if (IS_EVENT_MAGIC(searchp[ifirst - ROW_SIZE/USS])){
			err("previous MAGIC at 0x%08x", ifirst - ROW_LONGS);
		}else{
			dbg(1, "no previous MAGIC (good)");
		}
	}else{
		dbg(1, "too close to tblock start, can't look back");
	}

	for (block = 1; block < nblocks; ++block){
		if (TBLOCK_OFFSET(*first) + block*ROW_SIZE < TBLOCK_LEN){
			if (IS_EVENT_MAGIC(searchp[ifirst + block*ROW_LONGS])){
				dbg(1, "got a magic going forward %d [Good]", 
				    block);
			}else{
				err( "no MAGIC going forward %d", block);
			}
		}else{
				dbg(1, "too close to tblock end, can't look");
		}
	}	
}


void acq132_event_adjust(
	struct Phase *phase, unsigned isearch, 
	unsigned* first, unsigned* last)
/* acq132 data is in N blocks.
 * so the ES is N* bigger than detected... move out last to 
 * represent this
 */
{
	int nblocks = acq132_getScanlistLen();
	int eslen = *last - *first;
	int newlast = *last + eslen * (nblocks-1 + event_adjust_delta_blocks);

	
	dbg(1, "eslen %d nblocks %d event_adjust_delta_blocks %d",
	    eslen, nblocks, event_adjust_delta_blocks);

	check_first_row(first, last);

	if (mark_event_data){
		memset(BB_PTR(*first-sample_size()), 0xfe, 8*sizeof(short));
		memset(BB_PTR(*last), 0xde, 8*sizeof(short));
	}
	dbg(1, "nblocks:%d first,last: 0x%08x,0x%08x => 0x%08x,0x%08x",
	    nblocks, *first, *last, *first, newlast);

	if (stub_event_adjust < 0){
		dbg(1, "updating last");
		*last = newlast;
	}

	if (stub_event_adjust < -1){
/* here be empirical black magic. TODO: find theory that fits facts */
/* also, this is probably only valid in 32 ch case.. */
		int modulo = (isearch/sizeof(short))%ROW_WORDS;
		int deltasam = 3*modulo/8 - 3;

		*first += deltasam * sample_size();
		*last += deltasam * sample_size();;

		dbg(1, "deltasam %d => 0x%08x,0x%08x", deltasam, *first, *last);
	}	
}


static ssize_t acq132_timebase_read ( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	unsigned *es_base =  (unsigned*)BB_PTR(es_tble->tblock->offset);
	int nstamps = es_cursor - es_base;

	info("ES tblock: %d cursor %d", es_tble->tblock->iblock, nstamps);
	return -1;
}

struct file_operations acq132_timebase_ops = {
	.read = acq132_timebase_read
};


