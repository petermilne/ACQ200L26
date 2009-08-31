/* ------------------------------------------------------------------------- */
/* acq196-offset.c offset control for acq196                                 */
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

#define ACQ196
#include "acq100-offset-inc.h"

/*
 * map chip, dac to channel in block
 */
static const int MAP[NCHIPSBLOCK+1][NDACSCHIP+1] = {
/* MAP[chip][dac] ... unfortunately, these numbers are pchan order :-( */
	[ 1][DACA] = 1,  [ 1][DACB] = 2,
	[ 1][DACC] = 17, [ 1][DACD] = 18,
	[ 1][DACE] = 9,  [ 1][DACF] = 10,
	[ 1][DACG] = 25, [ 1][DACH] = 26,
	[ 2][DACA] = 3,  [ 2][DACB] = 4,
	[ 2][DACC] = 19, [ 2][DACD] = 20,
	[ 2][DACE] = 11, [ 2][DACF] = 12,
	[ 2][DACG] = 27, [ 2][DACH] = 28,
	[ 3][DACA] = 5,  [ 3][DACB] = 6,
	[ 3][DACC] = 21, [ 3][DACD] = 22,
	[ 3][DACE] = 13, [ 3][DACF] = 14,
	[ 3][DACG] = 29, [ 3][DACH] = 30,
	[ 4][DACA] = 7,  [ 4][DACB] = 8,
	[ 4][DACC] = 23, [ 4][DACD] = 24,
	[ 4][DACE] = 15, [ 4][DACF] = 16,
	[ 4][DACG] = 31, [ 4][DACH] = 32,
};


static const int __plut[] = {
/* index: nameplate order 1:32
 * value: memory order 1:32 
 */
		[ 1] =  1, 
		[ 2] = 17, 
		[ 3] =  2, 
		[ 4] = 18, 
		[ 5] =  9,	
		[ 6] = 25,
		[ 7] = 10,
		[ 8] = 26,

		[ 9] =  3,
		[10] = 19,
		[11] =  4,
		[12] = 20,
		[13] = 11,
		[14] = 27,
		[15] = 12,
		[16] = 28,

		[17] =  5,
		[18] = 21,
		[19] =  6,
		[20] = 22,
		[21] = 13,
		[22] = 29,
		[23] = 14,
		[24] = 30,

		[25] =  7,
		[26] = 23,
		[27] =  8,
		[28] = 24,
		[29] = 15,
		[30] = 31,
		[31] = 16,
		[32] = 32 
};
#define __PLUT_ELEMS (sizeof(__plut)/sizeof(int))

static const int *plut = __plut;
static int nlut = __PLUT_ELEMS;


static int hw_fudge(int lchan)
{
	switch(lchan){
	case 61:
		return 63;
	case 63:
		return 61;
	default:
		return lchan;
	}
}


int acq200_lookup_pchan(int lchannel)
/** lchannel in = 1:96 nameplate order return 0:95 memory order */
{
	int block = ((lchannel-1)/NCHANNELSBLOCK);
	int index = lchannel - block*NCHANNELSBLOCK;

	return block*NCHANNELSBLOCK + plut[index] - 1;
}

int acq200_lookup_lchan(int pchan)
{
       int block = ((pchan-1)/NCHANNELSBLOCK);
       int match = ((pchan-1)%NCHANNELSBLOCK) + 1;
       int index = 1;

       for (; index <= NCHANNELSBLOCK; ++index){
               if (plut[index] == match){
                       return block * NCHANNELSBLOCK + index;
               }
       }
       return -1;
}


static inline unsigned short *key2offset(int ikey)
/* ikey 1..96 */
{
	int block = (ikey-1)/NCHANNELSBLOCK + 1;
	int lchan = (ikey-1)%NCHANNELSBLOCK + 1;
	int pchan = acq200_lookup_pchan(lchan);

	dbg(1, "key2offset key %d block %d lchan %d pchan %d",
	    ikey, block, lchan, pchan);

	return &offsets[block][pchan+1];
}


#include "acq100-offset-inc.c"
/* EOF */
