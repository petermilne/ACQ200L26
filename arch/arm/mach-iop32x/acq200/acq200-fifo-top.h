/* ------------------------------------------------------------------------- */
/* acq200-fifo-top.h customisation for fifo driver                           */
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

#ifndef __ACQ200_FIFO_TOP_H__
#define __ACQ200_FIFO_TOP_H__

#if defined ACQ196

#if defined ACQ196C
/* pulls one 96 channel sample per DMA block */


#warning ACQ196c - control version DMA_BLOCK_LEN set to 192
#define DMA_BLOCK_LEN 192
#define ISR_ADDS_ENDSTOP 1

/** experiment with big blocks 
#define DMA_BLOCK_LEN 8192
#define ISR_ADDS_ENDSTOP 0
*/

#elif defined ACQ196M
#define DMA_BLOCK_LEN 64
#warning DMA_BLOCK_LEN set to 64 for overlapped HAWG
#define ISR_ADDS_ENDSTOP 1
#elif defined(ACQ196F)
#warning DMA_BLOCK_LEN set to 1024 for FIR
#define DMA_BLOCK_LEN 1024
/* #define DMA_BLOCK_LEN 1024 */
#define ISR_ADDS_ENDSTOP 0
#else
#define DMA_BLOCK_LEN 1024
#define ISR_ADDS_ENDSTOP 1
#endif


#define FPGA_IS_PCI_DEVICE 0

#define ACQ_HAS_COUNTER	1


#elif defined ACQ216


#define DMA_BLOCK_LEN 16384
/*
#define DMA_BLOCK_LEN (16384+8192)
*/
#define ISR_ADDS_ENDSTOP 0
#define FPGA_IS_PCI_DEVICE 1
#define MAXCHAN 16

#define ACQ_HAS_COUNTER	1

#elif defined ACQ164

#define DMA_BLOCK_LEN	1024
#define ISR_ADDS_ENDSTOP 1
#define FPGA_IS_PCI_DEVICE 0
#define MAXCHAN		64
#define ACQ_HAS_COUNTER	1
#define ACQ_HAS_GATE	1

#elif defined WAV232

#define DMA_BLOCK_LEN 1024
#define ISR_ADDS_ENDSTOP 1
#define FPGA_IS_PCI_DEVICE 1

#define GET_NEXT_EMPTY wav232_getNextEmpty

#elif defined ACQ132

#define DMA_BLOCK_LEN 4096
/* @todo Big blocks 4096 work better but PBI will go out of order */
#define ISR_ADDS_ENDSTOP 1
#define FPGA_IS_PCI_DEVICE 0

#define MAXCHAN 32

#define ACQ_HAS_COUNTER	1
#define ACQ_HAS_GATE	1
#else

#error Please ensure ACQxxx is defined

#endif

#ifndef GET_NEXT_EMPTY
#define GET_NEXT_EMPTY default_getNextEmpty
#endif

#endif
