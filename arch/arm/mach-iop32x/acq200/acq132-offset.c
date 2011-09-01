/* ------------------------------------------------------------------------- */
/* acq132-offset.c offset control for acq132                                 */
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

#define ACQ132
#include "acq100-offset-inc.h"

#include "acq132.h"

#include <linux/ctype.h>

const int acq132_default_plut[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] =  1, [ 2] = 17,
	[ 3] =  2, [ 4] = 18,
	[ 5] =  3, [ 6] = 19,
	[ 7] =  4, [ 8] = 20,
	[ 9] =  5, [10] = 21,
	[11] =  6, [12] = 22,
	[13] =  7, [14] = 23,
	[15] =  8, [16] = 24,
	[17] =  9, [18] = 25,
	[19] = 10, [20] = 26,
	[21] = 11, [22] = 27,
	[23] = 12, [24] = 28,
	[25] = 13, [26] = 29,
	[27] = 14, [28] = 30,
	[29] = 15, [30] = 31,
	[31] = 16, [32] = 32
};

static const int acq132_default_plut55[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] =  1, [ 2] = 17,
	[ 3] =  3, [ 4] = 19,
	[ 5] =  5, [ 6] = 21,
	[ 7] =  7, [ 8] = 23,
	[ 9] =  9, [10] = 25,
	[11] = 11, [12] = 27,
	[13] = 13, [14] = 29,
	[15] = 15, [16] = 31,
};

static const int acq132_default_plut11[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] =  1, [ 2] = 17,
	[ 3] =  5, [ 4] = 21,
	[ 5] =  9, [ 6] = 25,
	[ 7] = 13, [ 8] = 29,
};

#define FULL_LUT 0

static struct Acq132ChannelLUT default_luts[] = {
//	[ FULL_LUT ] = 
       	{	0xffffffff, 32, acq132_default_plut,	"DCBA"	},
	{	0x00ff00ff, 16, acq132_default_plut,    "DC"	},
	{	0x000f000f,  8, acq132_default_plut,	"D"	},
	{	0x00030003,  4, acq132_default_plut,	"D"	},
	{	0x00010001,  2, acq132_default_plut,	"D"	},
	{	0x55555555, 16, acq132_default_plut55,	"DCBA"	},
	{	0x11111111,  8, acq132_default_plut11,  "DCBA"	},
	{       0x00110011,  4, acq132_default_plut11,  "DC"	},
};

static struct Acq132ChannelLUT_Collection acq132_default_LUTs = {
	.model = "acq132",
	.nmasks = sizeof(default_luts)/sizeof(struct Acq132ChannelLUT),
	.luts = default_luts,
};
static const int *__plut = acq132_default_plut;

#define __PLUT_ELEMS (sizeof(acq132_default_plut)/sizeof(int))

static const int *plut = acq132_default_plut;
static int nlut = __PLUT_ELEMS;

static struct Acq132ChannelLUT_Collection* acq132_LUTs = &acq132_default_LUTs;


int acq200_lookup_pchan_from_lut(const int* alut, int alut_len, int lchannel)
{
	int index = 1;

	for (; index <= alut_len; ++index){
		if (alut[index] == lchannel){
			return index -1;
		}
	}
	return -1;
}
int acq200_lookup_pchan(int lchannel)
/** lchannel in = 1:32 nameplate order return 0:32 memory order */
{
	return acq200_lookup_pchan_from_lut(plut, nlut, lchannel);
}

int acq200_lookup_pchan_from_full(int lchannel)
{
	struct Acq132ChannelLUT* full_lut = &acq132_LUTs->luts[0];
	return acq200_lookup_pchan_from_lut(
			full_lut->plut, full_lut->nlut, lchannel);
}

int acq200_lookup_lchan(int pchan)
/* lookup pchan = 0:31 return 1:32 nameplate */
{
	return plut[pchan+1];
}

static u32 normalizeMask(u32 mask){
	int ch = 1;	
	u32 nmask = 0;
	for (ch = 1; ch <= 32; ++ch){
		if ((mask & (1 << (ch-1))) != 0){
			nmask |= 1 << (acq132_rewire(ch)-1);
		}
	}
	return nmask;
}




static int count_bits(unsigned mask) {
	int ibit;
	int count = 0;

	for (ibit = 0; ibit < 32; ++ibit){
		if (mask & (1<<ibit)){
			++count;
		}
	}
	return count;
}


int acq132_setChannelMask(unsigned mask)
{
	int ii;

	dbg(1, "01: mask %08x", mask);
	for (ii = 0; ii < acq132_LUTs->nmasks; ++ii){
		if (acq132_LUTs->luts[ii].mask == mask){
			int lchan;
			u32 mm;

			plut = acq132_LUTs->luts[ii].plut;
			nlut = acq132_LUTs->luts[ii].nlut;

			CAPDEF->channel_mask = mask;

			for (lchan = 1, mm = 1; mm; mm<<=1, lchan++){
				acq200_setChannelEnabled(
					acq200_lookup_pchan(lchan), 
					(mm&mask) != 0);
			}			

			dbg(1, "MATCH %08x len %d sl %s", 
				mask, acq132_LUTs->luts[ii].nlut,
				acq132_LUTs->luts[ii].scanlist);

			acq132_store_scanlist(acq132_LUTs->luts[ii].scanlist);
			acq132_set_channel_mask(normalizeMask(mask));
			CAPDEF_set_nchan(count_bits(mask));
			return ii;
		}
	}
	return -1;
}

int acq132_rewire(int ch)
/* normalizes ch to regular wiring */
{
#ifdef PGMCOMOUT
	if (plut == acq132_default_plut){
		return ch;
	}else{
		int pchan = acq200_lookup_pchan_from_full(ch);
		int ch2 = acq132_default_plut[pchan+1];

		dbg(1, "%02d => %02d [pchan=%d]", ch, ch2, pchan);
		return ch2;
	}
#else
	return ch;
#endif
}


int acq132_store_scanlist(const char* sl_def)
{
	int maxscan = strlen(sl_def);
	int iscan;
	int nscan = 0;
	u32 scan_def = 0;
	int ok = 1;	

	for (iscan = 0; iscan < maxscan; ++iscan){
		char dquad = sl_def[iscan];

		if (IS_SX(dquad)){
			scan_def |= TO_SX(dquad) << 2*nscan;
			nscan += 1;
		}else if (isspace(dquad)){
			continue;
		}else{
			ok = 0;
		}
	}
	if (ok){
		acq132_setScanList(nscan, scan_def);
	}
	return ok;
}

void acq132_setChannelLUTs(const struct Acq132ChannelLUT_Collection* luts)
{
	if (luts){
		acq132_LUTs = luts;
		plut = luts->luts[0].plut;
		nlut = luts->luts[0].nlut;
	}else{
		acq132_LUTs = &acq132_default_LUTs;
		plut = acq132_default_plut;
		nlut = __PLUT_ELEMS;	
	}
}

EXPORT_SYMBOL_GPL(acq132_setChannelLUTs);
EXPORT_SYMBOL_GPL(acq200_lookup_lchan);
EXPORT_SYMBOL_GPL(acq132_default_plut);
EXPORT_SYMBOL_GPL(acq132_rewire);

/* EOF */
