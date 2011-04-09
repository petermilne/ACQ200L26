/* ------------------------------------------------------------------------- */
/* gtmr driver interface for acq200				                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
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

/*
 * GTMR/GTSR is an iop231 facility, a 32 bit counter with 20nsec tick
 * This driver handles overflows and presents a 64 bit timestamp,
 * Overflow in 2**64 * 2e-9 = .. never
 */

#ifndef __GTMR_H__
#define __GTMR_H__

#define GTMR_TICK_NSEC		20
#define GTMR_TICK_PER_MSEC	50000

unsigned long long gtmr_update_timestamp(void);
/* returns timestamp in GTMR ticks */

#endif /* __GTMR_H__ */
