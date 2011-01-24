/* ------------------------------------------------------------------------- */
/* acq100_rtm_t.h  - RTM-T adapter hw defs	                             */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __ACQ100_RTM_T_H__
#define __ACQ100_RTM_T_H__

#include "rtm-t.h"

#define DIO_REG_TYPE (volatile u32*)
#define RTMT_REG(offset) (DIO_REG_TYPE((unsigned)ACQ200_EXTERNIO+(offset)))



/* {R: read, H: host write, Q: acQ write }	*/

#define RTMT_REG_REVID_SIG	0xaa010000 /* {R} */





int __init	acq100_rtm_t_uart_init(void);
void __exit	acq100_rtm_t_uart_exit(void);

#endif /* __ACQ100_RTM_T_H__ */
