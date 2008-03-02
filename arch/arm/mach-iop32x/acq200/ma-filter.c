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


#define VERID "$Revision: 1.0 $ build B1014 "


char macustom_driver_name[] = "acq196-ppcustom";
char macustom_driver_string[] = "D-TACQ Low Latency Control Device";
char macustom_driver_version[] = VERID __DATE__;
char macustom_copyright[] = "Copyright (c) 2008 D-TACQ Solutions Ltd";

int LOG2_NAVG = 2;		/* Div 4 by default */
module_param(LOG2_NAVG, int, 0644);


static void adjust_this_tblock(
	struct Phase * phase, struct TblockListElement *tble)
{
	tble->phase_sample_start >>= LOG2_NAVG;
	tble->tblock_sample_start >>= LOG2_NAVG;
	tble->sample_count >>= LOG2_NAVG;


	if (phase->transformer_private == 0){
		phase->start_sample >>= LOG2_NAVG;
		phase->actual_len >>= LOG2_NAVG;
		phase->actual_samples >>= LOG2_NAVG;
				
		phase->start_off = 
			tble->tblock->offset + 
			tble->tblock_sample_start;

		phase->transformer_private = 1;
	}

	if (tble->phase_sample_start+tble->sample_count ==
	    phase->actual_samples - 1){
		struct Phase *next_phase;

		dbg(1, "correct rounding error");
		phase->actual_samples -= 1;
		phase->actual_len -= sample_size();

		next_phase = list_entry(phase->list.next, struct Phase, list);

		if (next_phase != 0){
			next_phase -> start_sample -= 1;
		}
	}
}
static void adjust_tblock_in_phase(struct Phase * phase, void *cursor)
/* the data in this tblock just got shortened, so we have to find all
 * the corresponding tble's and adjust them...
 * Also, make a once-only adjustment to phase global actual_samples ..
 * there is an implicit assumption that all the phase gets transformed,
 * not just this tblock, but that is reasonable, we don't do partials.
 */
{
	struct TblockListElement *tble;
	int this_index = TBLOCK_INDEX(cursor - va_buf(DG));

	if (phase == 0){
		return;
	}

	list_for_each_entry(tble, &phase->tblocks, list){
		if (TBLOCK_INDEX(tble->tblock->offset) == this_index){
			dbg(1, "looking for %d got tblock:%d %s", 
			    this_index,	tble->tblock->iblock, "Adjusting");
			adjust_this_tblock(phase, tble);
			break;
		}
	}	
}

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

	int nchan = stride;
	int npairs = stride/2;
        int nsamples = nwords/stride;
        int isample, ipair;
        unsigned p0, p1;
	int imean;
	int navg = 1<<LOG2_NAVG;
	int *sums;
	int ichan;

	dbg(1, "from: %p TBLOCK: %d nw:%d",
	    from, TBLOCK_INDEX((void*)from - va_buf(DG)),
	    nwords);

	sums = kmalloc(nchan*sizeof(int), GFP_KERNEL);

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
	                for (ipair = 0; ipair < npairs; ++ipair){
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

	adjust_tblock_in_phase(DMC_WO->pre, from);
	adjust_tblock_in_phase(DMC_WO->post, from);


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


static void rm_ppcustom_sysfs(struct device *dev) 
{
	device_remove_file(dev, &dev_attr_version);
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
	rm_ppcustom_sysfs(dev);
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


