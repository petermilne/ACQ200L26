/* ------------------------------------------------------------------------- */
/* acq100-skel.c driver for acq100 lowlatency controller multi destination*/
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Peter Milne, D-TACQ Solutions Ltd
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

#define ACQ196

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
/* #include <linux/pci.h> */
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

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

/* keep debug local to this module */
#define acq200_debug acq100_skel_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"

int acq100_skel_debug;
module_param(acq100_skel_debug, int, 0664);

#define VERID "$Revision: 1.4 $ build B1000 "

#define DIO_REG_TYPE (volatile u32*)
#include "acqX00-rtm.h"

static void speed_test(void) 
{
	int tests[10][4];

	int itest;
	unsigned command;
	

	for (itest = 9; itest >= 0; --itest){
		tests[itest][0] = *IOP321_GTSR;   /* timer: 50MHz */
		tests[itest][1] = *IOP321_GTSR;
		command = *IOP321_IMR0;           /* read mailbox */
		tests[itest][2] = *IOP321_GTSR;
		if (command&1){                   /* decode "command" */
			*ACQ196_BDR = 2;          /* write DIO */
		}else{
			*ACQ196_BDR = 3;
		}
		tests[itest][3] = *IOP321_GTSR;
	}

	for (itest = 9; itest >= 0; --itest){
		printk("[%d] %10u %10u %10u %10u ",
		       10 - itest,
		       tests[itest][0],
		       tests[itest][1],
		       tests[itest][2],
		       tests[itest][3]);
		if (itest != 9){
			printk(" looptime %u\n", 
			       tests[itest][0] - tests[itest+1][0]);
		}else{
			printk("\n");
		}
	}

	for (itest = 9; itest >= 0; --itest){
		unsigned t0 = tests[itest][0];        /* get start time */
		unsigned tmeas = tests[itest][1]-t0;  /* measurement overhead*/

		printk("[%d] %10u %10u %10u %10u\n",
		       10 - itest,
		       tests[itest][0]-t0,
		       tests[itest][1]-t0,
		       tests[itest][2]-t0 - tmeas,
		       tests[itest][3]-t0 - tmeas);
	}

}

static void dio_test(void)
{
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;
	*RTM_DIO_DATA_A = 0xffff;
	*RTM_DIO_DATA_A = 0x0000;

}
static int mk_skel_sysfs(struct device *dev)
{
	speed_test();
	dio_test();
	return 0;
}


static void acq100_skel_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq100_skel_driver;

static int acq100_skel_probe(struct device *dev)
{
	info("");
	mk_skel_sysfs(dev);
	return 0;
}

static int acq100_skel_remove(struct device *dev)
{
	return 0;
}


static struct device_driver acq100_skel_driver = {
	.name     = "acq100_skel",
	.probe    = acq100_skel_probe,
	.remove   = acq100_skel_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq100_skel_device = {
	.name = "acq100_skel",
	.id   = 0,
	.dev = {
		.release    = acq100_skel_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq100_skel_init( void )
{
	int rc;
	acq200_debug = acq100_skel_debug;

	rc = driver_register(&acq100_skel_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acq100_skel_device);
}


static void __exit
acq100_skel_exit_module(void)
{
	info("");
	platform_device_unregister(&acq100_skel_device);
	driver_unregister(&acq100_skel_driver);
}


module_init(acq100_skel_init);
module_exit(acq100_skel_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ100 Skel Control");


