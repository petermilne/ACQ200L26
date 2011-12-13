/* ------------------------------------------------------------------------- */
/* acq164-offset.c offset control for acq164                                 */
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

#define ACQ164

#define NCHANNELSBLOCK 32


static	const int __plut[] = {
/* index: nameplate order 1:32
 * value: memory order 1:32 
 */
	[ 1] = 1,
	[ 2] = 5,
	[ 3] = 9,
	[ 4] = 13,
	[ 5] = 2,
	[ 6] = 6,
	[ 7] = 10,
	[ 8] = 14,
	[ 9] = 3,
	[10] = 7,
	[11] = 11,
	[12] = 15,
	[13] = 4,
	[14] = 8,
	[15] = 12,
	[16] = 16,
/* goes wacky */
	[17] = 20,
	[18] = 24,
	[19] = 28,
	[20] = 32,
	[21] = 19,
	[22] = 23,
	[23] = 27,
	[24] = 31,
	[25] = 18,
	[26] = 22,
	[27] = 26,
	[28] = 30,
	[29] = 17,
	[30] = 21,
	[31] = 25,
	[32] = 29
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


#define BLOCK_SHR	5
#define BLOCK_MASK	((1<<BLOCK_SHR)-1)

int acq200_lookup_pchan(int lchannel)
/** lchannel in = 1:32 nameplate order return 0:32 memory order */
{
	int block = ((lchannel-1)>>BLOCK_SHR);  /* 1:32=>0, 33:64=>1 */
	int index = lchannel - block*NCHANNELSBLOCK;

	return (block<<BLOCK_SHR) + plut[index] - 1;
}

int acq200_lookup_lchan(int pchan)
/* this looks wrong? pchan starts at zero .. */
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

EXPORT_SYMBOL_GPL(acq200_lookup_pchan);
EXPORT_SYMBOL_GPL(acq200_lookup_lchan);
