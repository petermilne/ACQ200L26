/* ------------------------------------------------------------------------- */
/* acq132_lfp.c  - Lemo Front Panel Channel Mapping                          */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2007 Peter Milne, D-TACQ Solutions Ltd
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


#include <linux/kernel.h>

#define ACQ132


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"
#include "acq200-fifo.h"


static	int plut[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] = 15 /*  1 */, [ 2] = 31 /* 17 */,
	[ 3] = 16 /*  2 */, [ 4] = 32 /* 18 */,
	[ 5] = 13 /*  3 */, [ 6] = 29 /* 19 */,
	[ 7] = 14 /*  4 */, [ 8] = 30 /* 20 */,
	[ 9] = 11 /*  5 */, [10] = 27 /* 21 */,
	[11] = 12 /*  6 */, [12] = 28 /* 22 */,
	[13] =  9 /*  7 */, [14] = 25 /* 23 */,
	[15] = 10 /*  8 */, [16] = 26 /* 24 */,
	[17] =  7 /*  9 */, [18] = 23 /* 25 */,
	[19] =  8 /* 10 */, [20] = 24 /* 26 */,
	[21] =  5 /* 11 */, [22] = 21 /* 27 */,
	[23] =  6 /* 12 */, [24] = 22 /* 28 */,
	[25] =  3 /* 13 */, [26] = 19 /* 29 */,
	[27] =  4 /* 14 */, [28] = 20 /* 30 */,
	[29] =  1 /* 15 */, [30] = 17 /* 31 */,
	[31] =  2 /* 16 */, [32] = 18 /* 32 */
};
#define PLUT_ELEMS (sizeof(plut)/sizeof(int))

/* BANK B A */

static int plut16[] = {
	[ 1] =  7 /*  9 */, [ 2] = 23 /* 25 */,
	[ 3] =  8 /* 10 */, [ 4] = 24 /* 26 */,
	[ 5] =  5 /* 11 */, [ 6] = 21 /* 27 */,
	[ 7] =  6 /* 12 */, [ 8] = 22 /* 28 */,
	[ 9] =  3 /* 13 */, [10] = 19 /* 29 */,
	[11] =  4 /* 14 */, [12] = 20 /* 30 */,
	[13] =  1 /* 15 */, [14] = 17 /* 31 */,
	[15] =  2 /* 16 */, [16] = 18 /* 32 */
};

/* BANK A */
static int plut8[] = {
	[ 1] =  3 /* 13 */, [ 2] = 19 /* 29 */,
	[ 3] =  4 /* 14 */, [ 4] = 20 /* 30 */,
	[ 5] =  1 /* 15 */, [ 6] = 17 /* 31 */,
	[ 7] =  2 /* 16 */, [ 8] = 18 /* 32 */
};

static int lfp_lookup(int ch)
{
	int ii;

	for (ii = 1; ii < PLUT_ELEMS; ++ii){
		if (plut[ii] == ch){
			return ii;
		}
	}
	return 0;
}

extern const int acq132_default_plut[];

int acq132_lfp_rewire(int ch) {
/* index logical (user) channel order 1:32
 * value physical (rewired) channel order 1:32
 * lookup value in plut, get ix, get lchan from acq200_lookup_lchan
 */
	int ch2 = acq132_default_plut[lfp_lookup(ch)];
	dbg(1, "ch %02d -> %02d", ch, ch2);
	return ch2;
}

static int acq132_lfp_set_special_lut(unsigned mask)
{
	static const int lut11[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] =  15, [ 2] = 31,
	[ 3] =  11, [ 4] = 27,
	};

	switch(mask){
	case 0x00010001:
		acq200_setChannelLut(lut11, 2);
		break;
	case 0x00110011:
		acq200_setChannelLut(lut11, 4);
		break;
	case 0x000f000f:
		acq200_setChannelLut(plut8, 8);
		break;
	case 0x00ff00ff:
		acq200_setChannelLut(plut16, 16);
		break;
	default:
		acq200_setChannelLut(plut, PLUT_ELEMS);
		break;
	}
	return 0;
}


static int __init acq132_lfp_init(void)
{
	info("lfp setting custom channel LUT");	
	acq200_setChannelLut(plut, PLUT_ELEMS);
	acq132_set_special_lut = acq132_lfp_set_special_lut;
	acq132_rewire = acq132_lfp_rewire;
	return 0;
}




static void __exit
acq132_lfp_exit_module(void)
{
	info("lfp restoring default channel LUT");
	acq200_setChannelLut(0, 0);
}

module_init(acq132_lfp_init);
module_exit(acq132_lfp_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("ACQ132 Lemo Front Panel Mapping");


