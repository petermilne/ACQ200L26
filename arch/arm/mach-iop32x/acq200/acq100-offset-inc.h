/* ------------------------------------------------------------------------- */
/* acq100-offset-inc.h acq100 common offset internal defs		     */
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


#ifdef ACQ196
#define MAXCHAN		96
#elif defined(ACQ132)
#define MAXCHAN		32
#elif defined(ACQ164)
#define MAXCHAN		64
#else
#error "ACQ196 || ACQ132 || ACQ164 not defined"
#endif

#define MAXBLOCKS	(MAXCHAN/32)
#define ACQ_IS_INPUT 1
#define DTACQ_MACH 2

#include "acq200-fifo-top.h"

#include "acq200_debug.h"
#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq196.h"

#include "acq100-offset.h"


/* this stuff is internal to acq1xx-offset.c */
#define DACX 0
#define DACY 1

#define NCHIPSBLOCK 4
#define NDACSCHIP  8
#define NCHANNELSBLOCK 32

/* convert ino to channel - ch01 is at ino 2 */
#define INO2CH(ch) (int)((ch) - 1)  

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((2+(numchan)+1+1)*TD_SZ)

#define LFS_MAGIC 0xa100a196

static unsigned short offsets[MAXBLOCKS+1][NCHANNELSBLOCK+1];
