/*------------------------------------------------------------------------- */
/* acqX00-irq.h irq special handling for ACQX00                             */
/*------------------------------------------------------------------------- */
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
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.               */
/*------------------------------------------------------------------------- */


#ifndef __ACQX00_IRQ_H__
#define __ACQX00_IRQ_H__

#include <asm/mach-types.h>

#define IRQ_ACQ100_FPGA IRQ_IOP32X_XINT0
#define IRQ_ACQ100_ETH  IRQ_IOP32X_XINT1
#define IRQ_ACQ100_UART IRQ_IOP32X_XINT2

#define IRQ_ACQ200_UART IRQ_IOP32X_HPI

#define IRQ_ACQX00_UART 3737

/*
 * catch machine-dependent variation in UART interrupt
 * the rest pass untransformed, except we make sure 0 => NONE
 */

#ifdef irq_canonicalize
#undef irq_canonicalize
#endif

#define machine_is_acq1xx() (machine_is_acq100()||machine_is_acq132())

#define irq_canonicalize(i) \
        ((i) == 0? IRQ_NONE: \
         (i) == IRQ_ACQX00_UART? ( \
		  machine_is_acq1xx()? IRQ_ACQ100_UART:		  \
	          machine_is_acq200()? IRQ_ACQ200_UART: IRQ_NONE):\
	 (i))

#endif /* __ACQX00_IRQ_H__ */
