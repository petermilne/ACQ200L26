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

/*
 * map chip, dac to channel in block
 */
static const int MAP[NCHIPSBLOCK+1][NDACSCHIP+1] = {
/* MAP[chip][dac] ... unfortunately, these numbers are pchan order :-( */
	[ 1][DACA] = 17,	[ 2][DACA] = 25,  
	[ 1][DACB] = 18,	[ 2][DACB] = 26,
	[ 1][DACC] = 19,	[ 2][DACC] = 27,
	[ 1][DACD] = 20,	[ 2][DACD] = 28,
	[ 1][DACE] = 21,	[ 2][DACE] = 29, 
	[ 1][DACF] = 22,	[ 2][DACF] = 30,
	[ 1][DACG] = 23,	[ 2][DACG] = 31,
	[ 1][DACH] = 24,	[ 2][DACH] = 32,
	
	[ 3][DACA] =  1,	[ 4][DACA] =  9,   
	[ 3][DACB] =  2,	[ 4][DACB] = 10,
	[ 3][DACC] =  3,	[ 4][DACC] = 11,
	[ 3][DACD] =  4,	[ 4][DACD] = 12,
	[ 3][DACE] =  5,	[ 4][DACE] = 13,
	[ 3][DACF] =  6,	[ 4][DACF] = 14,
	[ 3][DACG] =  7,	[ 4][DACG] = 15,
	[ 3][DACH] =  8,	[ 4][DACH] = 16,	 
};


static	const int __plut[] = {
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
#define __PLUT_ELEMS (sizeof(__plut)/sizeof(int))

static const int *plut = __plut;
static int nlut = __PLUT_ELEMS;


static int hw_fudge(int lchan)
{
	switch(lchan){
	default:
		return lchan;
	}
}


int acq200_lookup_pchan(int lchannel)
/** lchannel in = 1:32 nameplate order return 0:32 memory order */
{
	int index = 1;

	for (; index <= nlut; ++index){
		if (plut[index] == lchannel){
			return index -1;
		}
	}
	return -1;
}

int acq200_lookup_lchan(int pchan)
/* lookup pchan = 0:31 return 1:32 nameplate */
{
	return plut[pchan+1];
}

static inline unsigned short *key2offset(int ikey)
/* ikey 1..32. in this case, NO pchan lookup is required */
{
	int block = (ikey-1)/NCHANNELSBLOCK + 1;
	int lchan = (ikey-1)%NCHANNELSBLOCK + 1;

	return &offsets[block][lchan];
}
static void set_offset(int ikey, unsigned short offset)
{
	*key2offset(ikey) = offset;
}


static unsigned short get_offset(int ikey)
{
	return *key2offset(ikey);
}

static unsigned short lookup_offset(int block, int chip, int dac)
{
	dbg(1, "block: %d chip:%d dac:%c pchan %02d lchan %02d value:%d",
	    block, chip, dac-1+'A', 
	    (block-1)*32+MAP[chip][dac], 
	    acq200_lookup_lchan((block-1)*32+MAP[chip][dac]),
	    offsets[block][MAP[chip][dac]] );

	if (MAP[chip][dac] == 0){
		err("BAD MAP[%d][%d]", chip, dac);
	}
	return offsets[block][MAP[chip][dac]];
}


#include "acq100-offset-inc.c"
/* EOF */
