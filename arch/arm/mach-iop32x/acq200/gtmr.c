/* ------------------------------------------------------------------------- */
/* gtmr driver for acq200				                     */
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
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>

#include <linux/delay.h>
#include <asm/delay.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#include <linux/proc_fs.h>

#include "acqX00-port.h"

#define acq200_debug	debug
#include "acq200_debug.h"

char acq100_rtm_driver_name[] = "acq100-gtmr";

#define REVID	"acq100_gtmr B1000"



int debug = 0;
module_param(debug, int, 0644);


static void mk_gtmr_sysfs(struct device *dev)
{
//	DEVICE_CREATE_FILE(dev, &dev_attr_LEMO_IN);
}

static int gtmr_probe(struct device *dev)
{

}
static void gtmr_dev_release(struct device * dev)
{
	info("");
}

static int gtmr_remove(struct device *dev)
{

}


static u64 dma_mask = 0x00000000ffffffff;
static struct platform_device gtmr_device = {
	.name	= "acq200_gtmr",
	.id	= 0,
	.dev	= {
		.release	= gtmr_dev_release,
		.dma_mask	= &dma_mask
	}
};



static struct device_driver gtmr_driver = {
	.name	= "acq200_gtmr",
	.probe	= gtmr_probe,
	.remove = gtmr_remove,
	.bus	= &platform_bus_type
};




static int __init gtmr_init( void )
{
	int rc;

	info(REVID);

	rc = driver_register(&gtmr_driver);
	if (rc != 0){
		goto driver_fail;
	}
	rc = platform_device_register(&gtmr_device);
	if (rc != 0){
		goto device_fail;
	}
	
	return 0;

device_fail:
	driver_unregister(&gtmr_driver);
driver_fail:
	return rc;	
}


static void __exit
gtmr_exit_module(void)
{
	platform_device_unregister(&gtmr_device);
	driver_unregister(&gtmr_driver);
}

module_init(gtmr_init);
module_exit(gtmr_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for acq200 precision timestamp");



