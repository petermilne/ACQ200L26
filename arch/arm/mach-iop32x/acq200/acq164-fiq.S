/* ------------------------------------------------------------------------- */
/* acq132-fiq.S                                                              */
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

#warning WORKTODO ACQ164 crib of ACQ132

/* NB: fifcon in case ACQ132, ACQ196 is actually FIFO_STATUS !! */
	
#define DTACQ_MACH 2
#define ACQ164

#include "acq200-fifo-top.h"
	
#include "acq164.h"
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/hardware.h>

	
#include <asm/arch/iop321-dma.h>
#include <asm/arch/memory.h>

	
#include "acq200-pipek.h"


#if defined ACQ164

#define ACQ200_FIFCON		ACQ164_FIFSTAT
#define ACQ200_SYSCON		ACQ164_SYSCON
#define ACQ200_FIFCON_MASK	0x0000000f
#define ACQ132_FIFSTAT_HOTPOINT 0x0000000f
	
/*
 * HOTPOINT: 16 levels, 32K., 2 K per level
 * NB BLOCKSIZE 4K
 */
	
#define HOTxB 	 6
			/* remember, we already moved one block */
#define CHECK_2B 1	/* 1 additional blocks */


	.macro DO_CHECK_2B tmp, fifcon, not_2b
	and	\tmp, \fifcon, #ACQ164_FIFSTAT_HOTPOINT
	cmp	\tmp, #HOTxB
	blt	\not_2b
	.endm
#else
#error BAD STATE, we want ACQ164 ONLY HERE
#endif	
	
#define UPDATE_COLD_HISTO 0
#define UPDATE_HOT_HISTO 1
	
	.macro GET_HOT tmp, fifcon
	and	\tmp, \fifcon, #ACQ164_FIFSTAT_HOTPOINT
	.endm

#define MACH_VER CHECK_2B
				
#include "acq200-pipe-fiq.S"

	
