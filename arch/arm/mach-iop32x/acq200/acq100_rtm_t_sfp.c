/* ------------------------------------------------------------------------- */
/* acq100_rtm_t_sfp.c  - RTM-T aurora/SFP control and status	                             */
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

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#define acq200_debug rtm_t_debug
#include "acq200.h"
#include "acq200_debug.h"


#include "acq100_rtm_t.h"
#include "rtm-t.h"

char* getFlags(u32 stat, char buf[], int maxbuf)
{
	int cursor = 0;

	cursor += sprintf(buf+cursor, "%c%s ",
		(stat & RTMT_D_AURORA_STAT_SFP_PRESENTn)? '-': '+',
				"PRESENT");
	if ((stat & RTMT_D_AURORA_STAT_SFP_LOS) != 0){
		cursor += sprintf(buf+cursor, "LOS ");
	}
	if ((stat & RTMT_D_AURORA_STAT_SFP_TX_FAULT) != 0){
		cursor += sprintf(buf+cursor, "TX_FAULT ");
	}
	if ((stat & RTMT_D_AURORA_STAT_HARD_ERR) != 0){
		cursor += sprintf(buf+cursor, "HARD_ERR ");
	}
	if ((stat & RTMT_D_AURORA_STAT_SOFT_ERR) != 0){
		cursor += sprintf(buf+cursor, "SOFT_ERR ");
	}
	if ((stat & RTMT_D_AURORA_STAT_FRAME_ERR) != 0){
		cursor += sprintf(buf+cursor, "FRAME_ERR ");
	}
	if ((stat & RTMT_D_AURORA_STAT_CHANNEL_UP) != 0){
		cursor += sprintf(buf+cursor, "+CHANNEL_UP ");
	}
	if ((stat & RTMT_D_AURORA_STAT_LANE_UP) != 0){
		cursor += sprintf(buf+cursor, "+LANE_UP ");
	}
	strcat(buf, "\n");
	assert(cursor < maxbuf);
	return buf;
}

#define AURORA(SFPN) \
static ssize_t store_aurora##SFPN(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	u32 ctrl = simple_strtoul(buf, 0, 16);				\
	*RTMT_REG(RTMT_D_AURORA##SFPN##_CTRL) = ctrl;			\
	return count;							\
}									\
									\
								\
static ssize_t show_aurora##SFPN(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	char flags[80];							\
	u32 stat = *RTMT_REG(RTMT_D_AURORA##SFPN##_STAT);		\
									\
	return sprintf(buf, "0x%08x %s\n", stat, getFlags(stat, flags, 80)); \
}									\
									\
									\
static DEVICE_ATTR(aurora##SFPN, S_IRUGO|S_IWUGO,			\
		show_aurora##SFPN, store_aurora##SFPN);


AURORA(0);

void create_sfp_knobs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_aurora0);
}
