/* ------------------------------------------------------------------------- */
/* acq216_ppcustom.c custom post processing example for ACQ216               */
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

#define ACQ216


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
#define acq200_debug acq216_ppcustom_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int acq216_ppcustom_debug;
module_param(acq216_ppcustom_debug, int, 0664);


#define VERID "$Revision: 1.3 $ build B1000 "


char acq216_ppcustom_driver_name[] = "acq196-ppcustom";
char acq216_ppcustom_driver_string[] = "D-TACQ Low Latency Control Device";
char acq216_ppcustom_driver_version[] = VERID __DATE__;
char acq216_ppcustom_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


/*
 * meta data - compact representation of the data set.
 * we use histograms (simple integer generation function)
 * one histogram per channel per TBLOCK (6MB)
 */
#define H_NCHAN 16
#define H_NBINS 64



typedef int Histogram[H_NCHAN][H_NBINS];

#define DO_CUSTOM_MATH(histogram, ch, sample, xx) \
    (*histogram)[ch][(((xx)^0x8000)>>10)]++


static Histogram** histograms;


static Histogram* getHistogram(short *from)
{
	char* bb = (char*)DG->bigbuf.resource.start;
	char* cursor = (char*)from;
	int tblen = DG->bigbuf.tblocks.blocklen;
	int tblock = (cursor - bb)/tblen;

	if (tblock < 0 || tblock >= DG->bigbuf.tblocks.nblocks){
		err("tblock out of range %d", tblock);
		tblock = 0;
	}
	
	return histograms[tblock];
}

static void ppcustom_transform(short *to, short *from, int nwords, int stride)
/*
 * handle data in 2x2 squares
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
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int iblock;

	for (iblock = 0; iblock != MAX_TBLOCK; ++iblock){
		mk_test_pattern(iblock, histograms[iblock]);
	}	
	return strlen(buf);
}

static DEVICE_ATTR(test_pattern, S_IWUGO, 0, store_test_pattern);

static ssize_t store_clear(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int iblock;

	for (iblock = 0; iblock != MAX_TBLOCK; ++iblock){
		memset(histograms[iblock], 0, sizeof(Histogram));
	}	
	return strlen(buf);
}

static DEVICE_ATTR(clear, S_IWUGO, 0, store_clear);



static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq216_ppcustom_driver_name,
		       acq216_ppcustom_driver_string,
		       acq216_ppcustom_driver_version,
		       acq216_ppcustom_copyright
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










#define LFS_MAGIC 0xa2160001

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+2+1)*TD_SZ)
static struct tree_descr *my_files;

static int acq216_ppcustom_fill_super (
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations access_ops = {
		.open = meta_data_open,
		.read = meta_data_read,
	};

	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};

	int iblock;
	static char names[256][4];

	my_files = kmalloc(MY_FILES_SZ(MAX_TBLOCK), GFP_KERNEL);

	memcpy(&my_files[0], &front, TD_SZ);
	
	for (iblock = 1; iblock <= MAX_TBLOCK; ++iblock){
		sprintf(names[iblock], "%02d", iblock);
		my_files[iblock].name = names[iblock];
		my_files[iblock].ops  = &access_ops;
		my_files[iblock].mode = S_IRUGO;
	}

	memcpy(&my_files[iblock++], &backstop, TD_SZ);

	info("call simple_fill_super");
	return simple_fill_super(sb, LFS_MAGIC, my_files);
}




static int acq216_ppcustom_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, 
	void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(
			fst, flags, data, acq216_ppcustom_fill_super, mnt);
}

static struct file_system_type custom_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq216_ppcustomfs",
	.get_sb		= acq216_ppcustom_get_super,
	.kill_sb	= kill_litter_super,
};

static void mk_ppcustom_fs(void)
/* store results as nodes in a custom file system */
{
	register_filesystem(&custom_fs_type);
}

static void rm_ppcustom_fs(void)
{
	unregister_filesystem(&custom_fs_type);
	kfree(my_files);
}

static void acq216_ppcustom_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq216_ppcustom_driver;

static 	struct Transformer transformer = {
	.name = "ppcustom",
	.transform = ppcustom_transform
};

static int acq216_ppcustom_probe(struct device *dev)
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
	mk_ppcustom_fs();
	mk_ppcustom_sysfs(dev);
	histograms = 
		kmalloc(DG->bigbuf.tblocks.nblocks*sizeof(void *), GFP_KERNEL);

	for (iblock = 0; iblock != MAX_TBLOCK; ++iblock){
		histograms[iblock] = kmalloc(sizeof(Histogram), GFP_KERNEL);
	}
	dbg(1, "99");
	return 0;
}

static int acq216_ppcustom_remove(struct device *dev)
{
	int iblock;

	for (iblock = 0; iblock != MAX_TBLOCK; ++iblock){
		kfree(histograms[iblock]);
	}
	rm_ppcustom_fs();
	kfree(histograms);

	acq200_unregisterTransformer(&transformer);
	return 0;
}


static struct device_driver acq216_ppcustom_driver = {
	.name     = "acq216_ppcustom",
	.probe    = acq216_ppcustom_probe,
	.remove   = acq216_ppcustom_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq216_ppcustom_device = {
	.name = "acq216_ppcustom",
	.id   = 0,
	.dev = {
		.release    = acq216_ppcustom_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq216_ppcustom_init( void )
{
	acq200_debug = acq216_ppcustom_debug;

	driver_register(&acq216_ppcustom_driver);
	return platform_device_register(&acq216_ppcustom_device);
}


static void __exit
acq216_ppcustom_exit_module(void)
{
	info("");
	platform_device_unregister(&acq216_ppcustom_device);
	driver_unregister(&acq216_ppcustom_driver);
}

module_init(acq216_ppcustom_init);
module_exit(acq216_ppcustom_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ216 Custom Postprocessing");


