/* ------------------------------------------------------------------------- */
/* acq200-transform.c internal defs for acq200 bigbuffer transform           */
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

#include "acqX00-port.h"
#include "acq200-fifo-local.h"
#include "acq200-fifo.h"


#include "acq200-fifo-tblock.h"

#include <asm-arm/arch-iop32x/acq200.h>

#include <linux/dma-mapping.h>

#define MAX_TRANSFORMS 16

void transform1(short *to, short *from, int nwords, int stride);
void transform2(short *to, short *from, int nwords, int stride);
void transform3(short *to, short *from, int nwords, int stride);

/*
 * transform: called per tblock, primary function to:
 *
 * transform data from xx[sample][channel] to yy[channel][sample]
 *
 * raw capture bigbuf is split into Temporary Blocks (tblocks), 6MB long to
 * allow a sensibly sized system temporary buffer.
 *
 * process is:
 * bigbuf.tblock[n] => transform => temp => blt => bigbuf.tblock[n]
 *
 * ie the transformed data ends up back where it started
 * currently the copy back via blt_memcpy() accounts for ~30% overhead,
 * and it is planned to use DMA to speed this up (and even pipeline it).
 * 
 * transform1() - simple but slow, 16 bits at a time
 * transform2() - faster, 2 x 32 bits at a time
 * transform3() - example with math data negation
 */
 
void blt_memcpy(short *to, short *from, int nwords)
{
      memcpy(to, from, nwords*sizeof(short));
}

/*
DMA_NONE
DMA_BIDIRECTIONAL
DMA_TODEVICE
DMA_FROMDEVICE


dma_addr_t 
dma_map_single(struct device *dev,
               void *addr,
	       size_t size,
	       enum dma_data_direction direction);	

void 
dma_unmap_single(struct device *dev,
                 dma_addr_t dma_address,
		 size_t size,
		 enum dma_data_direction direction);

*/

extern int blt_dma_using_interrupt;

void blt_dma(short *to, short *from, int nwords)
{
	u32 bc = nwords*sizeof(short);
	u32 pto = dma_map_single(DG->dev, to, bc, DMA_FROM_DEVICE);
	u32 pfrom = dma_map_single(DG->dev, from, bc, DMA_TO_DEVICE);
	u32 poll = blt_dma_using_interrupt == 0? DMA_CHANNEL_POLL: 0;
	
	acq200_post_mem2mem_dmac_request(1|poll, pto, pfrom, bc);
		
	dma_unmap_single(DG->dev, pto, bc, DMA_FROM_DEVICE);
	dma_unmap_single(DG->dev, pfrom, bc, DMA_TO_DEVICE);
}



void transform1(short *to, short *from, int nwords, int stride)
{
	int nsamples = nwords/stride;
	int isample, ichannel;

	for (isample = 0; isample != nsamples; ++isample){
		for (ichannel = 0; ichannel != stride; ++ichannel){
			to[ichannel*nsamples + isample] =
				from[isample*stride + ichannel];
		}
	}
}

static void __transform32(u32 *to, u32* from, int nwords, int stride)
{
	int nsamples = nwords/stride;
	int isample, ichannel;

	for (isample = 0; isample != nsamples; ++isample){
		for (ichannel = 0; ichannel != stride; ++ichannel){
			to[ichannel*nsamples + isample] =
				from[isample*stride + ichannel];
		}
	}
}


void transform32(short *to, short *from, int nwords, int stride)
{
	__transform32((u32*)to, (u32*)from, nwords/2, stride);

}




void transform2(short *to, short *from, int nwords, int stride)
/*
 * handle data in 2x2 squares
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 */
{
        unsigned *dstp = (unsigned *)to;
        unsigned *srcp = (unsigned *)from;

        int nsamples = nwords/stride;
	int choffset = nsamples/2;
        int isample, ipair;
        unsigned p0, p1, c0, c1;

        stride /= 2;  /* count pairs, not channels */

        for (isample = 0; isample != nsamples; isample += 2){
                for (ipair = 0; ipair != stride; ++ipair){

#define ISP0 (isample*stride+ipair)
#define ISP1 (ISP0+stride)
#define IDC0 (2*ipair*choffset+isample/2)
#define IDC1 (IDC0+choffset)

                        p0 = srcp[ISP0];
                        p1 = srcp[ISP1];

                        c0 = ((p0 << 16)>>16) | (p1 << 16);
                        c1 = ((p1 >> 16)<<16) | (p0 >> 16);

                        dstp[IDC0] = c0;
                        dstp[IDC1] = c1;
                }
        }
#undef ISP0
#undef ISP1
#undef IDC0
#undef IDC1
}

void acq200_short_transform(
	short *to, short *from, int nwords, int stride, int nsamples)
/*
 * handle data in 2x2 squares
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 */
{
        unsigned *dstp = (unsigned *)to;
        unsigned *srcp = (unsigned *)from;

	int choffset = nwords/stride/2;
        int isample, ipair;
        unsigned p0, p1, c0, c1;

        stride /= 2;  /* count pairs, not channels */

        for (isample = 0; isample != nsamples; isample += 2){
                for (ipair = 0; ipair != stride; ++ipair){

#define ISP0 (isample*stride+ipair)
#define ISP1 (ISP0+stride)
#define IDC0 (2*ipair*choffset+isample/2)
#define IDC1 (IDC0+choffset)

                        p0 = srcp[ISP0];
                        p1 = srcp[ISP1];

                        c0 = ((p0 << 16)>>16) | (p1 << 16);
                        c1 = ((p1 >> 16)<<16) | (p0 >> 16);

                        dstp[IDC0] = c0;
                        dstp[IDC1] = c1;
                }
        }
#undef ISP0
#undef ISP1
#undef IDC0
#undef IDC1
}


void transform3(short *to, short *from, int nwords, int stride)
/*
 * SAME as transform2() but negates the data
 * handle data in 2x2 squares
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 */
{
        unsigned *dstp = (unsigned *)to;
        unsigned *srcp = (unsigned *)from;

        int nsamples = nwords/stride;
	int choffset = nsamples/2;
        int isample, ipair;
        unsigned p0, p1, c0, c1;

        stride /= 2;  /* count pairs, not channels */

        for (isample = 0; isample != nsamples; isample += 2){
                for (ipair = 0; ipair != stride; ++ipair){

#define ISP0 (isample*stride+ipair)
#define ISP1 (ISP0+stride)
#define IDC0 (2*ipair*choffset+isample/2)
#define IDC1 (IDC0+choffset)

                        p0 = srcp[ISP0];
                        p1 = srcp[ISP1];

                        c0 = ((p0 << 16)>>16) | (p1 << 16);
                        c1 = ((p1 >> 16)<<16) | (p0 >> 16);

                        dstp[IDC0] = ~c0;
                        dstp[IDC1] = ~c1;
                }
        }

#undef ISP0
#undef ISP1
#undef IDC0
#undef IDC1
}

static int sectors = 1;



#define SECTORS sectors


void transform4(short *to, short *from, int nwords, int stride)
{
	short* tp = to;
	short* bp = from;
	int sector_words = nwords/SECTORS;
	int sector;

	for (sector = SECTORS; sector != 0; --sector){
		transform2(tp, bp, sector_words, stride);

		tp += sector_words;
		bp += sector_words;
	}	
}


void transform5(short *to, short *from, int nwords, int stride)
/*
 * SAME as transform2()
 * handle data in 2x2 squares
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 * BUT: now that we know the cache is 32 way, stride the SOURCE data
 * in groups of 16 channels.
 */
{
#define CPB 32
        unsigned *dstp = (unsigned *)to;
        unsigned *srcp = (unsigned *)from;

        int nsamples = nwords/stride;
	int choffset = nsamples/2;
        int isample, ipair;
        unsigned p0, p1, c0, c1;
	int nblocks = stride / CPB;
	int iblock;
	int pairs_per_block = CPB/2;

        stride /= 2;  /* count pairs, not channels */

	for (iblock = 0; iblock != nblocks; ++iblock){
		for (isample = 0; isample != nsamples; isample += 2){
			for (ipair = 0; ipair != pairs_per_block; ++ipair){

#define ISP0 (isample*stride + ipair)
#define ISP1 (ISP0+stride)
#define IDC0 (2*ipair*choffset + isample/2)
#define IDC1 (IDC0+choffset)

				p0 = srcp[ISP0];
				p1 = srcp[ISP1];

				c0 = ((p0 << 16)>>16) | (p1 << 16);
				c1 = ((p1 >> 16)<<16) | (p0 >> 16);

				dstp[IDC0] = c0;
				dstp[IDC1] = c1;
			}
                }
		/** pointers are in units of PAIRS */
		srcp += pairs_per_block;
		dstp += nsamples * pairs_per_block;
        }

#undef ISP0
#undef ISP1
#undef IDC0
#undef IDC1
}

/** acq216 speed up

* Q1Q2Q3Q4
* Q4Q3Q2Q1
*
* Logical:
* 12 13 14 15  00 01 02 03 08 09 10 11 04 05 06 07
* 04 05 06 07  08 09 10 11 00 01 02 03 12 13 14 15
*/

void transform12344321(short *to, short *from, int nwords, int stride)
{
	int ns = nwords/stride;
	int isample;
	int nq = stride/4;

	for (isample = 0; isample < ns-1; ++isample){
		int iq;
		int sq4;
		int dq4;
		int iss = isample * stride;

		/* Q1..QN */
		for (iq = 0; iq < nq; ++iq){
			sq4 = iq*4; 			      
			dq4 = iq * 4 * ns + isample; 

			to[dq4] = from[iss + sq4 + 0]; dq4 += ns;
			to[dq4] = from[iss + sq4 + 1]; dq4 += ns;
			to[dq4] = from[iss + sq4 + 2]; dq4 += ns;
			to[dq4] = from[iss + sq4 + 3];
		}
	
		iss = ++isample*stride;
		/* QN..Q1 */
		for (iq = 0; iq < nq; ++iq){
			sq4 = (nq - 1 - iq) * 4; 
			dq4 = iq * 4 * ns + isample;

			to[dq4] = from[iss + sq4 + 0]; dq4 += ns;
			to[dq4] = from[iss + sq4 + 1]; dq4 += ns;
			to[dq4] = from[iss + sq4 + 2]; dq4 += ns;
			to[dq4] = from[iss + sq4 + 3];
		}			
	}
}




static ssize_t show_transformer_transform(
	struct device_driver * driver, char * buf)
{
	const struct Transformer **pt = DG->bigbuf.transformers;
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	int it;
	const char *name = "error";

	for (it = 1; it != MAX_TRANSFORMS; ++it){
		if (pt[it] == 0){
			break;
		}else if (tbl->transform == pt[it]->transform){
			name = pt[it]->name;
			break;
		}else{
			continue;
		}
	}

	return sprintf(buf, "%d %s\n", it, name);
}

static ssize_t store_transformer_transform(
	struct device_driver * driver, const char * buf, size_t count)
{
	const struct Transformer **pt = DG->bigbuf.transformers;
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	int it;

	if (sscanf(buf, "%d", &it) == 1){
		if (IN_RANGE(it, 0, MAX_TRANSFORMS-1) &&
		    pt[it] != 0){
			tbl->transform = pt[it]->transform;
			return strlen(buf);
		}
	}else{
		char ukey[32];
		if (sscanf(buf, "%31s", ukey) == 1){
			for (it = 0; it < MAX_TRANSFORMS; ++it){
				const char *key = pt[it]->name;

				if (key != 0){
					dbg(1, "matching \"%s\" and \"%s\"",
					    ukey, key);
				}

				if (key != 0 && strcmp(key, ukey) == 0){
					tbl->transform = pt[it]->transform;
					return strlen(buf);
				}
			}
		}
		else err("sscanf failed");
	}

	return -1;
}

static DRIVER_ATTR(transformer_transform,S_IRUGO | S_IWUGO, 
		   show_transformer_transform, store_transformer_transform);


static ssize_t show_sectors(
	struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%d\n", sectors);
}

static ssize_t store_sectors(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%d", &sectors);
	sectors = max(1, sectors);
	sectors = min(sectors, 512);
	return strlen(buf);
}
static DRIVER_ATTR(transformer_sectors, S_IRUGO | S_IWUGO, 
		   show_sectors, store_sectors);



void acq200_transform_mk_sysfs(struct device_driver *driver)
{
	DRIVER_CREATE_FILE(driver, &driver_attr_transformer_transform);
	DRIVER_CREATE_FILE(driver, &driver_attr_transformer_sectors);
}

void acq200_transform_rm_sysfs(struct device_driver *driver)
{
	driver_remove_file(driver, &driver_attr_transformer_transform);
	driver_remove_file(driver, &driver_attr_transformer_sectors);
}


int acq200_registerTransformer(const struct Transformer* transformer)
{
	const struct Transformer **pt = DG->bigbuf.transformers;
	int it;

	for (it = 1; it != MAX_TRANSFORMS; ++it){
		if (pt[it] == transformer){
			return it;
		}else if (pt[it] == 0){
			pt[it] = transformer;
			return it;
		}else{
			continue;
		}
	}

	return -1;
}


void acq200_unregisterTransformer(const struct Transformer* transformer)
{
	const struct Transformer **pt = DG->bigbuf.transformers;
	int it;

	for (it = 1; it != MAX_TRANSFORMS; ++it){
		if (pt[it] == transformer){
			pt[it] = 0;
			return;
		}
	}
}

void acq200_setTransformer(int it)
{
	const struct Transformer **pt = DG->bigbuf.transformers;
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;

	if (IN_RANGE(it, 1, MAX_TRANSFORMS) &&
	    pt[it] != 0){
		tbl->transform = pt[it]->transform;
		tbl->t_flags = pt[it]->t_flags;
	}
}
const struct Transformer* acq200_getTransformer(int it)
{
	const struct Transformer **pt = DG->bigbuf.transformers;

	if (IN_RANGE(it, 1, MAX_TRANSFORMS) && pt[it] != 0){
		return pt[it];
	}
	    
	return 0;
}

const void* acq200_getSelectedTransformerFunc(void)
{
	return DG->bigbuf.tblocks.transform;
}


void acq200_resetBigBufCursor(void)
{
	DG->bigbuf.tblocks.cursor = 0;
}

static int acq200_proc_transformers(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	char *bp = buf;
	int it;
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	const struct Transformer **pt = DG->bigbuf.transformers;
#define PRINTF(fmt, args...) bp += sprintf(bp, fmt, ## args)
	
	for (it = 0; it < MAX_TRANSFORMS; ++it){
		const struct Transformer *cursor = pt[it];
		PRINTF("%02d %s \"%s\"\n", 
			it,  
		       cursor? cursor->transform == tbl->transform?"*":"-":" ",
		       cursor? cursor->name: "");
	}
#undef PRTREG
#undef PRINTF
	return bp-buf;
}

void acq200_transform_init(void)
{
	static struct Transformer defaults[] = {
		{ .name = "transform1", .transform = transform1 },
		{ .name = "transform2", .transform = transform2 },
		{ .name = "transform3", .transform = transform3 },
		{ .name = "transform4", .transform = transform4 },
		{ .name = "transform5", .transform = transform5 },
		{ .name = "t12344321",  .transform = transform12344321 },
		{ .name = "transform32", .transform = transform32 },
	};
#define NDEFAULTS (sizeof(defaults)/sizeof(struct Transformer))
	int ireg;

	DG->bigbuf.transformers = 
		kmalloc(sizeof(struct Transformer*)*MAX_TRANSFORMS,GFP_KERNEL);
	memset(DG->bigbuf.transformers, 0, 
	       sizeof(struct Transformer*)*MAX_TRANSFORMS);

	for (ireg = 0; ireg != NDEFAULTS; ++ireg){
		acq200_registerTransformer(&defaults[ireg]);
	}

	create_proc_read_entry(
		"transformers", 0, proc_acq200, acq200_proc_transformers, 0 );
}
void acq200_transform_destroy(void)
{
	remove_proc_entry("transformers", proc_acq200);
	kfree(DG->bigbuf.transformers);     
}


int acq200_rounding(int khz, int precision)
{
	int sigfigs;
	int incoming_digits;
	int decade = 1;

	if (precision > 100){
		sigfigs = 3;    /* more than it deserves */
	}else if (precision > 10){
		sigfigs = 2;
	}else{
		sigfigs = 1;
	}

	for (incoming_digits = 1; incoming_digits < 10; ++incoming_digits){
		if (khz < decade){
			break;
		}else{
			decade *= 10;
		}
	}

	while(sigfigs--){
		decade /= 10;
	}

	if (decade){
		khz /= decade;
		khz *= decade;
	}
	return khz;
}


static unsigned char *cmap;


extern void acq200_setChannelEnabled(int pchan, int enable)
{
	if (!cmap){
		assert(get_nchan() != 0);
		cmap = (unsigned char*)kzalloc(acq200_maxchan, GFP_KERNEL);
		assert(cmap != 0);
	}

	dbg(1, "cmap %p pchan %d enabled %d", cmap, pchan, enable);
	cmap[pchan] = enable;

}
extern int acq200_pchanEnabled(int pchan)
{
	dbg(1, "cmap %p pchan %d enabled %d", cmap, pchan, 
	    cmap==0? 0: cmap[pchan]);

	if (!cmap){
		return 0;
	}else{
		return cmap[pchan];
	}
}
extern int acq200_lchanEnabled(int lchan)
{
	return acq200_pchanEnabled(acq200_lookup_pchan(lchan));
}

EXPORT_SYMBOL_GPL(acq200_getTransformer);
EXPORT_SYMBOL_GPL(acq200_setTransformer);
EXPORT_SYMBOL_GPL(acq200_getSelectedTransformerFunc);
