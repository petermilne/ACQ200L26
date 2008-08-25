/* ------------------------------------------------------------------------- */
/* lockin.c driver for acq196 lockin  FPGA                                   */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2008 Peter Milne, D-TACQ Solutions Ltd
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
#define acq200_debug acq196_lockin_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"

int acq196_lockin_debug;
module_param(acq196_lockin_debug, int, 0664);

int acq196_lockin_word_size = sizeof(u32);
module_param(acq196_lockin_word_size, int , 0664);


#define VERID "$Revision: 1.3 $ build B1012 "

char acq196_lockin_driver_name[] = "acq196-lockin";
char acq196_lockin_driver_string[] = "D-TACQ Low Latency Control Device";
char acq196_lockin_driver_version[] = VERID __DATE__;
char acq196_lockin_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


/**
 * todo LOCKIN transform - 32 bit data ignore shorts and shift ints
 * piece of cake
 */


void do_transform32(u32* to, u32* from, int nitems, int stride)
{
	int nsamples = nitems/stride;
	int isample, ichannel;

	for (isample = 0; isample != nsamples; ++isample){
		for (ichannel = 0; ichannel != stride; ++ichannel){
			to[ichannel*nsamples + isample] =
				from[isample*stride + ichannel];
		}
	}
}

void transform32(short *to, short *from, int nwords, int stride)
{
	do_transform32((u32*)to, (u32*)from, nwords/2, stride);
}


static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq196_lockin_driver_name,
		       acq196_lockin_driver_string,
		       acq196_lockin_driver_version,
		       acq196_lockin_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);






static int mk_lockin_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	return 0;
}


static void acq196_lockin_dev_release(struct device * dev)
{
	info("");
}


static 	struct Transformer transformer = {
	.name = "lock-in",
	.transform = transform32
};

static void register_custom_transformer(void)
{
	int it;
	it = acq200_registerTransformer(&transformer);
	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer %s NOT registered", transformer.name);
	}
}
static struct device_driver acq196_lockin_driver;

static int acq196_lockin_probe(struct device *dev)
{
	info("");
	register_custom_transformer();
	mk_lockin_sysfs(dev);
	return 0;
}

static int acq196_lockin_remove(struct device *dev)
{
	return 0;
}


static struct device_driver acq196_lockin_driver = {
	.name     = "acq196_lockin",
	.probe    = acq196_lockin_probe,
	.remove   = acq196_lockin_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq196_lockin_device = {
	.name = "acq196_lockin",
	.id   = 0,
	.dev = {
		.release    = acq196_lockin_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq196_lockin_init( void )
{
	int rc;
	acq200_debug = acq196_lockin_debug;

	CAPDEF_set_word_size(acq196_lockin_word_size);
	rc = driver_register(&acq196_lockin_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acq196_lockin_device);
}


static void __exit
acq196_lockin_exit_module(void)
{
	info("");
	platform_device_unregister(&acq196_lockin_device);
	driver_unregister(&acq196_lockin_driver);
}

module_init(acq196_lockin_init);
module_exit(acq196_lockin_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ196 LOCKIN");


