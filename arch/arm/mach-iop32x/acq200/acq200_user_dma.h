/* ------------------------------------------------------------------------- */
/* file acq200_user_dma.h                                                                 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
 *  Created on: Jun 2, 2013
 *      Author: pgm

    http://www.d-tacq.com

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

/** @file acq200_user_dma.h DESCR 
 *
 */

#ifndef ACQ200_USER_DMA_H_
#define ACQ200_USER_DMA_H_

/* write: u32[4], index values: */
#define ACQ200_USER_DMA_PDA	0
#define ACQ200_USER_DMA_LAD	1
#define ACQ200_USER_DMA_BC	2
#define ACQ200_USER_DMA_DC	3

#define ACQ200_USER_DMA_DC_CHAIN	0x80000000
#define ACQ200_USER_DMA_DC_CACHE_FLUSH	0x40000000 /* tell driver to flush this data */
#define ACQ200_USER_DMA_DC_CACHE_INVAL  0x20000000 /* tell driver to invalidate this region before returning it */
#define ACQ200_USER_DMA_DC_MASK		0x0000ffff

#define ACQ200_USER_DMA_WRITE_LEN	(4*sizeof(u32))

typedef unsigned DMADEF[4];

#endif /* ACQ200_USER_DMA_H_ */
