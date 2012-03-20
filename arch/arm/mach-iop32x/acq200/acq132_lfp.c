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

#include "acq132.h"

/* BANK D C B A */
static const int acq132_lfp_plut[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] = 15 /*  1 */, [ 2] = 31 /* 17 */,	/* BANK_D MASK_1 */
	[ 3] = 16 /*  2 */, [ 4] = 32 /* 18 */, 
	[ 5] = 13 /*  3 */, [ 6] = 29 /* 19 */, /* BANK_D MASK_1B */
	[ 7] = 14 /*  4 */, [ 8] = 30 /* 20 */,
	[ 9] = 11 /*  5 */, [10] = 27 /* 21 */, /* BANK_C MASK_1 */
	[11] = 12 /*  6 */, [12] = 28 /* 22 */,
	[13] =  9 /*  7 */, [14] = 25 /* 23 */, /* BANK_C MASK_1B */
	[15] = 10 /*  8 */, [16] = 26 /* 24 */,
	[17] =  7 /*  9 */, [18] = 23 /* 25 */, /* BANK_B MASK_1 */
	[19] =  8 /* 10 */, [20] = 24 /* 26 */,
	[21] =  5 /* 11 */, [22] = 21 /* 27 */, /* BANK_B MASK_1B */
	[23] =  6 /* 12 */, [24] = 22 /* 28 */,
	[25] =  3 /* 13 */, [26] = 19 /* 29 */, /* BANK_A MASK_1 */
	[27] =  4 /* 14 */, [28] = 20 /* 30 */,
	[29] =  1 /* 15 */, [30] = 17 /* 31 */, /* BANK_A MASK_1B */
	[31] =  2 /* 16 */, [32] = 18 /* 32 */
};
#define PLUT_ELEMS (sizeof(plut)/sizeof(int))

/* BANK B A */

static const int acq132_lfp_plut16[] = {
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
static const int acq132_lfp_plut8[] = {
	[ 1] =  3 /* 13 */, [ 2] = 19 /* 29 */,
	[ 3] =  4 /* 14 */, [ 4] = 20 /* 30 */,
	[ 5] =  1 /* 15 */, [ 6] = 17 /* 31 */,
	[ 7] =  2 /* 16 */, [ 8] = 18 /* 32 */
};




/* BANK D C B A */
static const int acq132_lfp_plut55555555[] = {
/* index: memory order 1:32 
 * value: nameplate order 1:32 
 */
	[ 1] = 15 /*  1 */, [ 2] = 31 /* 17 */,	/* BANK_D MASK_1 */
	[ 3] = 13 /*  3 */, [ 4] = 29 /* 19 */, /* BANK_D MASK_1B */
	[ 5] = 11 /*  5 */, [ 6] = 27 /* 21 */, /* BANK_C MASK_1 */
	[ 7] =  9 /*  7 */, [ 8] = 25 /* 23 */, /* BANK_C MASK_1B */
	[ 9] =  7 /*  9 */, [10] = 23 /* 25 */, /* BANK_B MASK_1 */
	[11] =  5 /* 11 */, [12] = 21 /* 27 */, /* BANK_B MASK_1B */
	[13] =  3 /* 13 */, [14] = 19 /* 29 */, /* BANK_A MASK_1 */
	[15] =  1 /* 15 */, [16] = 17 /* 31 */, /* BANK_A MASK_1B */
};
/* Bank DCBA */
static const int acq132_lfp_plut44444444[] = {
/* index: memory order 1:32, value: nameplate order 1:32 */
	[ 1] = 15,  [ 2] = 31,			/* BANK_D MASK_1 */
	[ 3] = 11,  [ 4] = 27,			/* BANK_C MASK_1 */
	[ 5] =  7,  [ 6] = 23,			/* BANK_B MASK_1 */
	[ 7] =  3,  [ 8] = 19,			/* BANK_A MASK_1 */
};
static const int acq132_lfp_plut11[] = {
/* index: memory order 1:32
 * value: nameplate order 1:32
 */
	[ 1] =  1 /* 15 */, [ 2] = 17 /* 31 */, /* BANK_A MASK_1B */
	[ 3] =  5 /* 11 */, [ 4] = 21 /* 27 */, /* BANK_B MASK_1B */
	[ 5] =  9 /*  7 */, [ 6] = 25 /* 23 */, /* BANK_C MASK_1B */
	[ 7] = 13 /*  3 */, [ 8] = 29 /* 19 */, /* BANK_D MASK_1B */
};

static const int acq132_lfp_plut_8080[] = {
/* index: memory order 1:32
 * value: nameplate order 1:32
 */
	[ 1] =  1 /* 15 */, [ 2] = 17 /* 31 */, /* BANK_A MASK_1B */
	[ 3] =  3 /* 13 */, [ 4] = 19 /* 29 */, /* BANK_A MASK_1 */
};

static const struct Acq132ChannelLUT acq132_lfp_luts[] = {
	{	0xffffffff, 32, acq132_lfp_plut,		"DCBA"	},
	{ 	0x0fff0fff, 24, acq132_lfp_plut+8,		"CBA"	},
	{	0x00ff00ff, 16, acq132_lfp_plut16,		"BA"	},
	{	0x000f000f,  8, acq132_lfp_plut8,		"A"	},
	{	0x55555555, 16, acq132_lfp_plut55555555,	"DCBA"  },
//	{	0x44444444,  8, acq132_lfp_plut44444444,	"DCBA"  },
/* .. replace with below: makes 7, 23 come up in the right place .. */
	{	0x44444444,  8, acq132_lfp_plut44444444,	"DBCA"  },
	{       0x44004400,  4, acq132_lfp_plut44444444,	"DC"	},
	{	0x40004000,  4, acq132_lfp_plut44444444,	"D"	},

	{	0x55555555, 16, acq132_lfp_plut55555555,	"DCBA"	},

	{	0x11111111,  8, acq132_lfp_plut11,		"ABCD"	},
	{	0x00110011,  4, acq132_lfp_plut11,		"AB"	},
	{	0x00010001,  2, acq132_lfp_plut11,		"A"	},
	{       0x00050005,  4, acq132_lfp_plut_8080,           "A"     },
};


static const struct ADC_CHANNEL_LUT LFP_ADC_CHANNEL_LUT[] = {
/* index by faceplate channel */
	[ 0] = {},
	[15] = { BANK_D, 'R', ACQ132_ADC_RANGE_R1, MASK_1 },
	[16] = { BANK_D, 'R', ACQ132_ADC_RANGE_R2, MASK_4 },
	[13] = { BANK_D, 'R', ACQ132_ADC_RANGE_R3, MASK_1B },
	[14] = { BANK_D, 'R', ACQ132_ADC_RANGE_R4, MASK_4 },
	[11] = { BANK_C, 'R', ACQ132_ADC_RANGE_R1, MASK_1 },
	[12] = { BANK_C, 'R', ACQ132_ADC_RANGE_R2, MASK_4 },
	[ 9] = { BANK_C, 'R', ACQ132_ADC_RANGE_R3, MASK_1B },
	[10] = { BANK_C, 'R', ACQ132_ADC_RANGE_R4, MASK_4 },
	[ 7] = { BANK_B, 'R', ACQ132_ADC_RANGE_R1, MASK_1 },
	[ 8] = { BANK_B, 'R', ACQ132_ADC_RANGE_R2, MASK_4 },
	[ 5] = { BANK_B, 'R', ACQ132_ADC_RANGE_R3, MASK_1B },
	[ 6] = { BANK_B, 'R', ACQ132_ADC_RANGE_R4, MASK_4 },
	[ 3] = { BANK_A, 'R', ACQ132_ADC_RANGE_R1, MASK_1 },
	[ 4] = { BANK_A, 'R', ACQ132_ADC_RANGE_R2, MASK_4 },
	[ 1] = { BANK_A, 'R', ACQ132_ADC_RANGE_R3, MASK_1B },
	[ 2] = { BANK_A, 'R', ACQ132_ADC_RANGE_R4, MASK_4 },
	[31] = { BANK_D, 'L', ACQ132_ADC_RANGE_L1, MASK_1 },
	[32] = { BANK_D, 'L', ACQ132_ADC_RANGE_L2, MASK_4 },
	[29] = { BANK_D, 'L', ACQ132_ADC_RANGE_L3, MASK_1B },
	[30] = { BANK_D, 'L', ACQ132_ADC_RANGE_L4, MASK_4 },
	[27] = { BANK_C, 'L', ACQ132_ADC_RANGE_L1, MASK_1 },
	[28] = { BANK_C, 'L', ACQ132_ADC_RANGE_L2, MASK_4 },
	[25] = { BANK_C, 'L', ACQ132_ADC_RANGE_L3, MASK_1B },
	[26] = { BANK_C, 'L', ACQ132_ADC_RANGE_L4, MASK_4 },
	[23] = { BANK_B, 'L', ACQ132_ADC_RANGE_L1, MASK_1 },
	[24] = { BANK_B, 'L', ACQ132_ADC_RANGE_L2, MASK_4 },
	[21] = { BANK_B, 'L', ACQ132_ADC_RANGE_L3, MASK_1B },
	[22] = { BANK_B, 'L', ACQ132_ADC_RANGE_L4, MASK_4 },
	[19] = { BANK_A, 'L', ACQ132_ADC_RANGE_L1, MASK_1 },
	[20] = { BANK_A, 'L', ACQ132_ADC_RANGE_L2, MASK_4 },
	[17] = { BANK_A, 'L', ACQ132_ADC_RANGE_L3, MASK_1B },
	[18] = { BANK_A, 'L', ACQ132_ADC_RANGE_L4, MASK_4 }
};

static const struct Acq132ChannelLUT_Collection acq132_lfp_LUTs = {
	.model = "acq132-lfp",
	.nmasks = sizeof(acq132_lfp_luts)/sizeof(struct Acq132ChannelLUT),
	.luts = acq132_lfp_luts
};

static const struct ADC_CHANNEL_LUT* default_adc_channel_lut; 

static int __init acq132_lfp_init(void)
{
	info("lfp setting custom channel LUT");	
	acq132_setChannelLUTs(&acq132_lfp_LUTs);
	default_adc_channel_lut = ADC_CHANNEL_LUT;
	ADC_CHANNEL_LUT = LFP_ADC_CHANNEL_LUT;
	return 0;
}




static void __exit
acq132_lfp_exit_module(void)
{
	info("lfp restoring default channel LUT");
	acq132_setChannelLUTs(0);
	ADC_CHANNEL_LUT = default_adc_channel_lut;
}

module_init(acq132_lfp_init);
module_exit(acq132_lfp_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("ACQ132 Lemo Front Panel Mapping");


