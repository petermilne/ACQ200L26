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

int malength = 32;
module_param(malength, int, 0664);

#define VERID "$Revision: 1.3 $ build B1000 "


char macustom_driver_name[] = "acq196-ppcustom";
char macustom_driver_string[] = "D-TACQ Low Latency Control Device";
char macustom_driver_version[] = VERID __DATE__;
char macustom_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";




static void ma_transform(short *to, short *from, int nwords, int stride)
/**< moving average filter on data .
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 */
{
        unsigned *dstp = (unsigned *)to;
        unsigned *srcp = (unsigned *)from;

        int nsamples = nwords/stride;
	int choffset = nsamples/2;
        int isample, ipair;
        unsigned p0, p1, c0, c1;

	Histogram* hg = getHistogram(from);

        stride /= 2;  /* count pairs, not channels */

	dbg(1,"");

        for (isample = 0; isample != nsamples; isample += 2){
                for (ipair = 0; ipair != stride; ++ipair){

#define ISP0 (isample*stride+ipair)
#define ISP1 (ISP0+stride)
#define IDC0 (2*ipair*choffset+isample/2)
#define IDC1 (IDC0+choffset)

                        p0 = srcp[ISP0];
                        p1 = srcp[ISP1];

                        c0 = ((p0 << 16)>>16) | (p1 << 16);
                        c1 = ((p1 >> 16)<<16) | (p0 >> 16);

			DO_CUSTOM_MATH(hg, ipair/2, isample,   c0>>16);
			DO_CUSTOM_MATH(hg, ipair/2, isample+1, c0&0x0ffff);

			DO_CUSTOM_MATH(hg, ipair/2+1, isample, c1>>16);
			DO_CUSTOM_MATH(hg, ipair/2+1, isample+1, c1&0x0ffff);
			       
                        dstp[IDC0] = c0;
                        dstp[IDC1] = c1;
                }
        }
}


static void mk_test_pattern(int iblock, Histogram* hg)
{
	int ichan;
	int ibin;

	dbg(1,"01 %d", iblock);

	for (ichan = 0; ichan != H_NCHAN; ++ichan){
		for (ibin = 0; ibin != H_NBINS; ++ibin){
			switch(ibin){
			case 0:
				(*hg)[ichan][ibin] = iblock;
				break;
			case 1:
				(*hg)[ichan][ibin] = ichan;
				break;
			default:
				(*hg)[ichan][ibin] = (ichan<<16)|(ibin);
			}
		}
	}

	dbg(1,"99");
}





static ssize_t store_test_pattern(
	struct device * dev, const char * buf, size_t count)
{
	int iblock;

	for (iblock = 0; iblock != MAX_TBLOCK; ++iblock){
		mk_test_pattern(iblock, histograms[iblock]);
	}	
	return strlen(buf);
}

static DEVICE_ATTR(test_pattern, S_IWUGO, 0, store_test_pattern);

static ssize_t store_clear(
	struct device * dev, const char * buf, size_t count)
{
	int iblock;

	for (iblock = 0; iblock != MAX_TBLOCK; ++iblock){
		memset(histograms[iblock], 0, sizeof(Histogram));
	}	
	return strlen(buf);
}

static DEVICE_ATTR(clear, S_IWUGO, 0, store_clear);



static ssize_t show_version(struct device *dev, char * buf)
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
	device_create_file(dev, &dev_attr_version);
	device_create_file(dev, &dev_attr_test_pattern);
	device_create_file(dev, &dev_attr_clear);
	return 0;
}




static int meta_data_open(struct inode *inode, struct file *filp)
{
	if (inode->i_ino == 0 || inode->i_ino > MAX_TBLOCK){
		return -ENODEV;
	}
	filp->private_data = (void*)inode->i_ino;
	return 0;
}


static ssize_t meta_data_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	Histogram* hg = histograms[(int)filp->private_data];
	int len = sizeof(Histogram);
	char* tmp = (char*)*hg;

	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, tmp + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
}











static void macustom_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver macustom_driver;

static 	struct Transformer transformer = {
	.name = "ma",
	.transform = ma_transform
};

static int macustom_probe(struct device *dev)
{
	int it;
	int iblock;
	info("");

	it = acq200_registerTransformer(&transformer);

	info("transformer registered %d", it);

	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer NOT registered");
	}

	dbg(1, "99");
	return 0;
}

static int macustom_remove(struct device *dev)
{
	int iblock;

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
	acq200_debug = macustom_debug;

	driver_register(&macustom_driver);
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


