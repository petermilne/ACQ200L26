/* ------------------------------------------------------------------------- */
/* acq216-fiq.S                                                              */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

#define DTACQ_MACH 1
#define ACQ216 

#include "acq200-fifo-top.h"
	
#include "acq216.h"
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/hardware.h>

	
#include <asm/arch/iop321-dma.h>
#include <asm/arch/memory.h>
#include "acq200-pipek.h"

#define ACQ200_FIFCON_HITIDE_01      0x00001000	

/*
 * CHECK_2B 3 => for 16K HOT FIFO WAY TO GO!!
 */
/* BLOCK 8192 experiment */

/* BS 16384 - NO CHECK_2B, app processing adds endstop */


#if (DMA_BLOCK_LEN == 16384)
#warning DMA_BLOCK_LEN 16384 ISR_ADDS_ENDSTOP 0
#define ACQ200_FIFCON_COLD_TH ACQ200_FIFCON_HITIDE_01	
#define CHECK_2B 0
#define TIDE_RAMP 0
#elif (DMA_BLOCK_LEN == 8192)
#define CHECK_2B 1
#define ACQ200_FIFCON_COLD_TH ACQ200_FIFCON_HITIDE_01
#define TIDE_RAMP 0	
#elif (CHECK_2B == 3)
#define ACQ200_FIFCON_COLD_TH ACQ200_FIFCON_HITIDE_01
#define TIDE_RAMP 0
#elif (CHECK_2B == 2)
#define ACQ200_FIFCON_COLD_TH ACQ200_FIFCON_COLD_HALF
#define TIDE_RAMP 1	
#else
#define ACQ200_FIFCON_COLD_TH 0x00200000
#define TIDE_RAMP 0
#endif

#if (TIDE_RAMP)

/*
 * this stat isn't used and saves 6 INSTRUCTIONS!!
 */	 	
#define STUB_WO_NEXT_LOAD_UPDATE 1
/*
 * MAJOR league hack - ramp hitide until it reachs optimum
 * WARNING:	 DO NOT tweak these values :	-)
 * HACK:	r9 assumed fpga
 */
#define ACQ200_FIFCON_HITIDE_STOP    0x00008000
#define ACQ200_FIFCON_HITIDE_LOWBITS 0x00007000


/*
 * DO_TIDE_RAMP:
 *
 * if (HITIDE != 0 && HITIDE < ACQ200_FIFCON_HITIDE_LOWBITS){
 *	++HITIDE
 * }
 *
 * if it's zero, chances are HOTHITIDE is set, and we don't want to mess
 * with this one.
 */
	.macro DO_TIDE_RAMP tmp, fifcon
	ands	\tmp, \fifcon, #ACQ200_FIFCON_HITIDE
	beq	99f
	cmp	\tmp, #ACQ200_FIFCON_HITIDE_LOWBITS
	addlt   \fifcon, \fifcon, #ACQ200_FIFCON_HITIDE_01
	strlt	\fifcon, [r9, #ACQ200_FIFCON]
99:	
	.endm

#warning TIDE RAMP enabled	
#endif

	
	.macro DO_CHECK_2B tmp, fifcon, not_2b
#if (TIDE_RAMP)
	DO_TIDE_RAMP \tmp, \fifcon
#endif	
	and	\tmp, \fifcon, #ACQ200_FIFCON_COLDPOINT
	cmp	\tmp, #ACQ200_FIFCON_COLD_TH
	blt	\not_2b
	.endm

		
#define UPDATE_COLD_HISTO 1
#define UPDATE_HOT_HISTO  0

#define MACH_VER (TIDE_RAMP << 4) | CHECK_2B
	
#include "acq200-pipe-fiq.S"

	
