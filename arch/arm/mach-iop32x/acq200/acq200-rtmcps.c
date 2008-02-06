/* ------------------------------------------------------------------------- */
/* acq200-rtm.c driver for acq200 rtmcps                                     */
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

/*
 * Module: provides hook to rtmcps.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>

#include <asm/delay.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "dio_defs.h"

int rtmcps_debug;
module_param(rtmcps_debug, int, 0664);





char acq100_rtmcps_driver_string[] = "D-TACQ RTMCPS driver";
char acq100_rtmcps_driver_version[] = "$Revision:$ build B1002 " __DATE__;
char acq100_rtmcps_copyright[] = "Copyright (c) 2007 D-TACQ Solutions Ltd";

#define DIO_REG_TYPE (volatile u16*)

#include "acqX00-rtm.h"

#define CPS_CTRL RTM_REG(0x20)
#define CPS_STAT RTM_REG(0x24)

/* bit positions */
#define CPS_CTRL_SYSSLOT	0
#define CPS_CTRL_TXEN		1
#define CPS_CTRL_WATCHDOG	2



static ssize_t show_stat_raw(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u16 stat = *CPS_STAT;
	return sprintf(buf, "0x%04x", stat);
}

static ssize_t store_stat_raw(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	u32 ustat;

	if (sscanf(buf, "0x%x", &ustat) == 1){
		*CPS_STAT = ustat;
	}
	return strlen(buf);
}

static DEVICE_ATTR(stat, S_IRUGO|S_IWUGO, show_stat_raw, store_stat_raw);

static ssize_t show_ctrl_raw(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u16 uctrl = *CPS_CTRL;
	return sprintf(buf, "0x%04x\n", uctrl);
}

static ssize_t store_ctrl_raw(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	u32 uctrl;

	if (sscanf(buf, "0x%x", &uctrl) == 1){
		*CPS_CTRL = uctrl;
	}
	return strlen(buf);
}

static DEVICE_ATTR(ctrl, S_IRUGO|S_IWUGO, show_ctrl_raw, store_ctrl_raw);


static ssize_t show_ctrl_bit(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf,
	int bit)
{
	u16 uctrl = *CPS_CTRL;
	return sprintf(buf, "%d\n", (uctrl>>bit)&1);
}

static ssize_t store_ctrl_bit(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count,
	int bit)
{
	u32 uctrl = *CPS_CTRL;
	u32 ustat;

	if (sscanf(buf, "%d", &ustat) == 1){
		if (ustat){
			uctrl |= (1<<bit);
		}else{
			uctrl &= ~(1<<bit);
		}
		*CPS_CTRL = uctrl;
	}
	return strlen(buf);
}


static ssize_t show_sys_slot(	
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return show_ctrl_bit(dev, attr, buf, CPS_CTRL_SYSSLOT);
}

static ssize_t store_sys_slot(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	return store_ctrl_bit(dev, attr, buf, count, CPS_CTRL_SYSSLOT);
}


static DEVICE_ATTR(SYS_SLOT, S_IRUGO|S_IWUGO, show_sys_slot, store_sys_slot);


static ssize_t show_txen(	
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return show_ctrl_bit(dev, attr, buf, CPS_CTRL_TXEN);
}

static ssize_t store_txen(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	return store_ctrl_bit(dev, attr, buf, count, CPS_CTRL_TXEN);
}


static DEVICE_ATTR(TXEN, S_IRUGO|S_IWUGO, show_txen, store_txen);

static ssize_t show_watchdog(	
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return show_ctrl_bit(dev, attr, buf, CPS_CTRL_WATCHDOG);
}

static DEVICE_ATTR(WATCHDOG, S_IRUGO, show_watchdog, 0);





static void mk_rtmcps_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_stat);
	DEVICE_CREATE_FILE(dev, &dev_attr_ctrl);
	DEVICE_CREATE_FILE(dev, &dev_attr_SYS_SLOT);
	DEVICE_CREATE_FILE(dev, &dev_attr_TXEN);
	DEVICE_CREATE_FILE(dev, &dev_attr_WATCHDOG);
}

static void rtmcps_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver rtmcps_driver;

static void choose_mode(void)
{
	u32 ctrl = *CPS_CTRL;

	dbg(1, "CPLD+3 = %02x", ctrl);
	
	if ((*(volatile u8*)(ACQ200_CPLD+3) & 0x01) == 1){
		info("peripheral slot");
		ctrl &= ~CPS_CTRL_SYSSLOT;
	}else{
		info("system slot");
		ctrl |= CPS_CTRL_SYSSLOT;
	}
	*CPS_CTRL = ctrl;
}

static int rtmcps_probe(struct device *dev)
{

	info("%s %s", 
		acq100_rtmcps_driver_string, acq100_rtmcps_driver_version);
	mk_rtmcps_sysfs(dev);
	choose_mode();
	init_inputs();
	return 0;
}

static int rtmcps_remove(struct device *dev)
{
	return 0;
}


static struct device_driver rtmcps_driver = {
	.name     = "rtmcps",
	.probe    = rtmcps_probe,
	.remove   = rtmcps_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device rtmcps_device = {
	.name = "rtmcps",
	.id   = 0,
	.dev = {
		.release    = rtmcps_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init rtmcps_init( void )
{
	int rc;
	acq200_debug = rtmcps_debug;

	rc = driver_register(&rtmcps_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&rtmcps_device);
}


static void __exit
rtmcps_exit_module(void)
{
	info("");
	platform_device_unregister(&rtmcps_device);
	driver_unregister(&rtmcps_driver);
}

module_init(rtmcps_init);
module_exit(rtmcps_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ2xx RTMCPS");


