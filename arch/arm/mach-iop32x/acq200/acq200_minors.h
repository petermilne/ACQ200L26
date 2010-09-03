/* ------------------------------------------------------------------------- */
/* acq200-minors.h - driver node minor number coding                         */
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

#ifndef __ACQ200_MINORS_H__
#define __ACQ200_MINORS_H__ 


/*
 * acq200-core device 
 */

/*
 * MINOR NUMBER CODING
 */
	

#define ACQ200_CORE_FLASH 16
#define ACQ200_CORE_CPLD  17
#define ACQ200_CORE_UART  18
#define ACQ200_CORE_FPGA  19
#define ACQ200_CORE_LIO   20
#define ACQ200_CORE_EXIO  21
#define ACQ200_CORE_PCIM  22
#define ACQ200_CORE_PCIIO 23
#define ACQ200_CORE_PMMR  30
#define ACQ200_CORE_IRAM  31
#define ACQ200_CORE_MUMEM 32
#define ACQ200_CORE_BIGBUF 33

#define IS_DEBUG_MAP_DEVICE(minor) \
        ((minor) >= ACQ200_CORE_FLASH && (minor) <= ACQ200_CORE_BIGBUF)

#define IS_FPGA_LOAD_DEVICE(minor) ((minor)==0)


#define ACQ200_MU_INBOUND  0        /* Read Incoming Messages */
#define ACQ200_MU_OUTBOUND 1        /* Write Outgoing Messages */
#define ACQ200_MU_RMA      2        /* Read or Write remote memory */
#define ACQ200_MU_MAILBOX  3
#define ACQ200_MU_PCIM    ACQ200_CORE_PCIM   /* host memory window */

/* host memory window may be offset from PCIM
 * cf IOP321_OMWTVR0 on 64MB boundary 
 */

/*
 * acq200-fifo device
 */
#define FIFO_DIAGS_DEVICE 0
#define FIFO_RO_DEVICE 1
#define FIFO_WO_DEVICE 2
#define FIFO_RW_DEVICE 3
#define FIFO_RO_DEVBUF 5    /* Read only, fill bigbuf first */
#define FIFO_WO_DEVBUF 6    /* Write only, fill bigbuf first */
#define FIFO_RO_DEVBUF_TEST 15
#define FIFO_WO_DEVBUF_TEST 16
#define FIFO_RW_LIVE_DATA 20
#define FIFO_RW_LIVE_OFFSETS 21

#define ACQ132_SFPGA_LOAD	22	/* device loads SFPGA */
#define ACQ132_GATE_PULSE_LOAD  23	/* GATE PULSE definitions */
#define ACQ132_TIMEBASE		24

/*
 * access to stored data:
 * +1  : ch1 etc
 * +127: ch 127
 * +0  : ch 128 (if ever implemented).
 */
#define BIGBUF_CHANNEL_DATA_DEVICE 0x80

/*
 * Cross Section Device XX P- physical (fastest) L- logical channel order (goriest)
 */
#define BIGBUF_DATA_DEVICE_XXP 127
#define BIGBUF_DATA_DEVICE_XXL 126
#define BIGBUF_DATA_DEVICE_FMT 125
/*
 * main module should define ACQ200_FPGA_REGS, void* regs mapping
 */



#endif  /* __ACQ200_MINORS_H__ */
