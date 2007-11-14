/* ------------------------------------------------------------------------- */
/* acq132.h                                                                  */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2007 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __ACQ132_H__
#define __ACQ132_H__

#include "acq196.h"

#define ACQ132_SFPGA_CONF	FPGA_REG(0x10)

#define ACQ132_SFPGA_CONF_PROG		31
#define ACQ132_SFPGA_CONF_INIT_LR	29	
#define ACQ132_SFPGA_CONF_DONE_8	20
#define ACQ132_SFPGA_CONF_BUSY		17
#define ACQ132_SFPGA_CONF_UPDATE	16
#define ACQ132_SFPGA_CONF_DATA		0x0000ffff

#define ACQ132_SFPGA_CONF_LRMASK	0x3
#define ACQ132_SFPGA_CONF_8MASK		0xff


static inline void sfpga_conf_clr_all(void) {
	dbg(1, "01");
	*ACQ132_SFPGA_CONF = 0;
}
static inline void sfpga_conf_set_prog(void) {
	dbg(1, "01");
	*ACQ132_SFPGA_CONF |= (1 << ACQ132_SFPGA_CONF_PROG);
}	

static inline void sfpga_conf_clr_prog(void) {
	dbg(1, "01");
	*ACQ132_SFPGA_CONF &= ~(1 << ACQ132_SFPGA_CONF_PROG);
}

static inline void sfpga_conf_set_update(void) {
	dbg(1, "01");
	*ACQ132_SFPGA_CONF |= (1 << ACQ132_SFPGA_CONF_UPDATE);
}

static inline int sfpga_conf_get_busy(void) {
	int rc = (*ACQ132_SFPGA_CONF & (1<<ACQ132_SFPGA_CONF_BUSY)) != 0;
	dbg(1, "busy %d", rc);
	return rc;
}

static inline int sfpga_conf_get_init(void) {
	int lr = *ACQ132_SFPGA_CONF & ACQ132_SFPGA_CONF_INIT_LR;
	int rc = lr == ACQ132_SFPGA_CONF_INIT_LR;

	dbg(1, "LR %d return %d", lr, rc);

	return lr;
}

static inline void sfpga_conf_send_data(u16 data)
{
	u32 conf = *ACQ132_SFPGA_CONF;
	conf &= ~ACQ132_SFPGA_CONF_DATA;
	conf |= data;


	*ACQ132_SFPGA_CONF = conf;		

	dbg(1, "conf set 0x%08x", conf);
}

static inline int sfpga_conf_done(void) {
	u32 conf = *ACQ132_SFPGA_CONF;
	int done = ((conf>>ACQ132_SFPGA_CONF_DONE_8) & ACQ132_SFPGA_CONF_8MASK)
		== ACQ132_SFPGA_CONF_8MASK;
	
	dbg(1, "conf 0x%08x done %d", conf, done);
	return done;
}
#endif	/*  __ACQ132_H__ */

