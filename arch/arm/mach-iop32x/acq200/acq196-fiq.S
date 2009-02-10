/* ------------------------------------------------------------------------- */
/* acq196-fiq.S                                                              */
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

#define DTACQ_MACH 2
#define ACQ196

#include "acq200-fifo-top.h"
	
#include "acq196.h"
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/hardware.h>

	
#include <asm/arch/iop321-dma.h>
#include <asm/arch/memory.h>

	
#include "acq200-pipek.h"

#define ACQ200_FIFCON ACQ196_FIFSTAT
#define ACQ200_SYSCON ACQ196_SYSCON
#define ACQ200_FIFCON_MASK 0xf

#if defined ACQ196M

/* special short block - we want to Q N every time */
#define CHECK_2B 7
	
	.macro DO_CHECK_2B tmp, fifcon, not_2b
	.endm

#elif defined ACQ196C

#if (ISR_ADDS_ENDSTOP == 1)
/* if FIFO mark>=1, pull an extra block. 
 * 512 bytes available, 2 blocks@192bytes each means we are safe from underrun.
 */	
#define CHECK_2B 1
	.macro DO_CHECK_2B tmp, fifcon, not_2b
	and	\tmp, \fifcon, #ACQ196_FIFSTAT_HOTPOINT
	cmp	\tmp, #1
	blt	\not_2b		
	.endm
#else
#define CHECK_2B 0
#endif
				
#elif defined ACQ196F
#define CHECK_2B 0
	
	.macro DO_CHECK_2B tmp, fifcon, not_2b
	.endm	
#else
	/* regular ACQ196 */
/*
 * HOTPOINT: 16 levels, 8K.
 * 0x2 : 1K
 * 0x4 : 2K
 * 0x6 : 3K
 * 0x8 : 4K
 * But we also have min 2K in the colds
 */
	
#if DMA_BLOCK_LEN < 256
#define CHECK_2B 0	
#else	
#define HOT3B 0x9	     /* for 96 channels */		
#define CHECK_2B 3           /* Q Fourth block */		
#endif	

	.macro DO_CHECK_2B tmp, fifcon, not_2b
	and	\tmp, \fifcon, #ACQ196_FIFSTAT_HOTPOINT
	cmp	\tmp, #HOT3B
	blt	\not_2b
	.endm
#endif
	
#define UPDATE_COLD_HISTO 0
#define UPDATE_HOT_HISTO 1

#if defined (ACQ216)
#error BAD STATE, BROKEN BUILD, STOP.
#endif

	.macro GET_HOT tmp, fifcon
	and	\tmp, \fifcon, #ACQ200_FIFCON_MASK
	.endm

#define MACH_VER CHECK_2B
				
#include "acq200-pipe-fiq.S"

	
