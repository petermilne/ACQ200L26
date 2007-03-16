/* ------------------------------------------------------------------------- */
/* wav232-fiq.S                                                              */
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

#define DTACQ_MACH 0
#define WAV232

#include "acq200-fifo-top.h"

	
#include "wav232.h"
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/hardware.h>

	
#include <asm/arch/iop321-dma.h>
#include <asm/arch/memory.h>
#include "acq200-pipek.h"

#define ACQ200_SYSCON WAV232_SYSCON
#define UPDATE_COLD_HISTO 1
#define UPDATE_HOT_HISTO  0

	.macro GET_HOT tmp, fifcon
	mov	\tmp, \fifcon, lsr #ACQ200_FIFCON_HOTP_SHIFT
	.endm

#define CHECK_2B 6

	/*
	 * if COLD < half, add another block
	 */
	.macro DO_CHECK_2B tmp, fifcon, not_2b
	and	\tmp, \fifcon, #ACQ200_FIFCON_COLDPOINT
	cmp	\tmp, #ACQ200_FIFCON_COLD_HLFF
	bgt	\not_2b      
	.endm

#define MACH_VER CHECK_2B
			
#include "acq200-pipe-fiq.S"

	
