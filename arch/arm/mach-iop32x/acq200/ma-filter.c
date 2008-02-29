/* ------------------------------------------------------------------------- */
/* ma-filter.c moving average filter: custom post processing example         */
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

/** NOT FUNCTIONAL! THERE ARE MANY ISSUES WITH THIS APPROACH!!!
 * end of tblock handling is the worst 
 * we need a tblock fixup on every tblock
 */
#define ACQ216


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

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
#define acq200_debug macustom_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int macustom_debug;
module_param(macustom_debug, int, 0664);


#define VERID "$Revision: 1.0 $ build B1004 "


char macustom_driver_name[] = "acq196-ppcustom";
char macustom_driver_string[] = "D-TACQ Low Latency Control Device";
char macustom_driver_version[] = VERID __DATE__;
char macustom_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";

int LOG2_NAVG = 2;		/* Div 4 by default */
module_param(LOG2_NAVG, int, 0644);


static void ma_transform(short *to, short *from, int nwords, int stride)
/**< moving average filter on data .
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 */
{
        short *dstp = (short *)to;
        unsigned *srcp = (unsigned *)from;

	int nchan = stride/sizeof(short);
        int nsamples = nwords/stride;
        int isample, ipair;
        unsigned p0, p1;
	int imean;
	int navg = 1<<LOG2_NAVG;
	int *sums;
	int ichan;

	sums = kmalloc(nchan*sizeof(int), GFP_KERNEL);

	stride /= 2;  /* count pairs, not channels */

#define ISP0 (isample*stride+ipair)
#define ISP1 (ISP0+stride)

#define GET0(x) (short)(((x) << 16) >> 16)
#define GET1(x)	(short)((x) >> 16)

#define SUM0(ip) sums[(ip)*2]
#define SUM1(ip) sums[(ip)*2+1]

#define IDC0 (ichan*nsamples+(isample>>LOG2_NAVG))

        for (isample = 0; isample < nsamples; ){
		memset(sums, 0, nchan*sizeof(int));

		for (imean = 0; imean < navg; imean += 2, isample += 2){
	                for (ipair = 0; ipair < stride; ++ipair){
		                p0 = srcp[ISP0];
			        p1 = srcp[ISP1];

				SUM0(ipair) += GET0(p1);
				SUM0(ipair) += GET0(p0);
				SUM1(ipair) += GET1(p1);
				SUM1(ipair) += GET1(p0);
			}
		}

		for (ichan = 0; ichan < nchan; ++ichan){
			dstp[IDC0] = sums[ichan] / navg;
		} 
        }
#undef ISP0
#undef ISP1
#undef GET0
#undef GET1
#undef IDC0
	/* now adjust maxsamples in TBLOCK */

	kfree(sums);
}



static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr, 
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       macustom_driver_name,
		       macustom_driver_string,
		       macustom_driver_version,
		       macustom_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);


static int mk_ppcustom_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	return 0;
}



static struct device_driver macustom_driver;

static 	struct Transformer transformer = {
	.name = "ma",
	.transform = ma_transform
};

static int macustom_probe(struct device *dev)
{
	int it;
	info("");

	it = acq200_registerTransformer(&transformer);

	info("transformer registered %d", it);

	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer NOT registered");
	}

	mk_ppcustom_sysfs(dev);
	dbg(1, "99");
	return 0;
}

static int macustom_remove(struct device *dev)
{
	acq200_unregisterTransformer(&transformer);
	return 0;
}


static struct device_driver macustom_driver = {
	.name     = "macustom",
	.probe    = macustom_probe,
	.remove   = macustom_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static void macustom_dev_release(struct device * dev)
{
	info("");
}


static struct platform_device macustom_device = {
	.name = "macustom",
	.id   = 0,
	.dev = {
		.release    = macustom_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init macustom_init( void )
{
	int rc = driver_register(&macustom_driver);
	if (rc){
		return rc;
	}

	acq200_debug = macustom_debug;
	return platform_device_register(&macustom_device);
}


static void __exit
macustom_exit_module(void)
{
	info("");
	platform_device_unregister(&macustom_device);
	driver_unregister(&macustom_driver);
}

module_init(macustom_init);
module_exit(macustom_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Moving Average filter postprocessing");


