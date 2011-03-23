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

static const int *__plut = acq132_default_plut;

#define __PLUT_ELEMS (sizeof(acq132_default_plut)/sizeof(int))

static const int *plut = acq132_default_plut;
static int nlut = __PLUT_ELEMS;



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


EXPORT_SYMBOL_GPL(acq200_lookup_lchan);
EXPORT_SYMBOL_GPL(acq132_default_plut);

/* EOF */
