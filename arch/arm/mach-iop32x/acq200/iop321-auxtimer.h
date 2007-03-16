/* ------------------------------------------------------------------------- */
/* iop321-auxtimer.h driver interface for timer1                             */
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

#ifndef __IOP321_AUXTIMER_H__
#define __IOP321_AUXTIMER_H__



struct AuxTimerClient {
	void (*func)(unsigned long clidata);
	unsigned long clidata;
};


int iop321_hookAuxTimer(struct AuxTimerClient* client, int hz);
/*
 * connect to timer, if hz > 0, start running,, == 0 unhook and stop
 * RETURNS 0 or -ERR.
 */


#endif /* __IOP321_AUXTIMER_H__ */
