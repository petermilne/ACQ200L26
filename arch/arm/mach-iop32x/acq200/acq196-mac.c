/* ------------------------------------------------------------------------- */
/* acq196-mac.c driver for acq196 MAC FPGA                                   */
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
#define acq200_debug acq196_mac_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"

int acq196_mac_debug;
module_param(acq196_mac_debug, int, 0664);

int acq196_mac_word_size = sizeof(u32);
module_param(acq196_mac_word_size, int , 0664);


#define VERID "$Revision: 1.3 $ build B1012 "

char acq196_mac_driver_name[] = "acq196-mac";
char acq196_mac_driver_string[] = "D-TACQ Low Latency Control Device";
char acq196_mac_driver_version[] = VERID __DATE__;
char acq196_mac_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


/**
 * todo MAC transform - 32 bit data ignore shorts and shift ints
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
		       acq196_mac_driver_name,
		       acq196_mac_driver_string,
		       acq196_mac_driver_version,
		       acq196_mac_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);



static ssize_t show_K(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf, int bank)
{
	unsigned ref = acq100_get_maccon_ref(bank);

	return sprintf(buf, "%s\n", 
		       ref==ACQ196_MACCON_REF_SEL_ONE? "ONE":
		       ref==ACQ196_MACCON_REF_SEL_DAC? "REF":
		       ref==ACQ196_MACCON_REF_SEL_ADC? "ADC": "ERROR");
}

static ssize_t store_K(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count, int bank)
{
	unsigned ref;

	if (strncmp(buf, "ADC", 3) == 0){
		ref = ACQ196_MACCON_REF_SEL_ADC;
	}else if (strncmp(buf, "DAC", 3) == 0 || strncmp(buf, "REF", 3) == 0){
		ref = ACQ196_MACCON_REF_SEL_DAC;
	}else if (strncmp(buf, "ONE", 3) == 0){
		ref = ACQ196_MACCON_REF_SEL_ONE;
	}else{
		return strlen(buf);
	}
	
	acq100_set_maccon_ref(bank, ref);
	return strlen(buf);
}


#define K_KNOB(BANK)					\
static ssize_t show_K ## BANK(				\
	struct device * dev,				\
	struct device_attribute *attr,			\
	char * buf)					\
{							\
	return show_K(dev, attr, buf, BANK);		\
}							\
							\
static ssize_t store_K ## BANK (			\
	struct device * dev,				\
	struct device_attribute *attr,			\
	const char * buf, size_t count)			\
{							\
	return store_K(dev, attr, buf, count, BANK);	\
}							\
							\
static DEVICE_ATTR(K ## BANK, S_IRUGO|S_IWUGO,		\
		   show_K ## BANK, store_K ## BANK);

K_KNOB(1);
K_KNOB(2);
K_KNOB(3);

static ssize_t show_status(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int status = (*ACQ196_MACCON & ACQ196_MACCON_OVFL);

	
        return sprintf(buf,"%4s 3:%-4s 2:%-4s 1:%-4s\n", 
		       status==0? "OK": "FAIL",
		       (status&ACQ196_MACCON_OVFL3) == 0? "OK": "OVER",
		       (status&ACQ196_MACCON_OVFL2) == 0? "OK": "OVER",
		       (status&ACQ196_MACCON_OVFL1) == 0? "OK": "OVER" );

}

static ssize_t clear_status(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	*ACQ196_MACCON |= ACQ196_MACCON_OVFL;
	return strlen(buf);
}

static DEVICE_ATTR(status, S_IRUGO|S_IWUGO, show_status, clear_status);

#define COUNTS_FROM_ZERO	1

static ssize_t show_depth(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int depth = *ACQ196_MACCON & ACQ196_MACCON_LENGTH;

        return sprintf(buf,"%d\n", depth + COUNTS_FROM_ZERO);
}

static ssize_t store_depth(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int depth = 0;

	if (sscanf(buf, "%d", &depth) == 1 &&
	    depth >= 1 && depth <= 0xfff){
		*ACQ196_MACCON &= ~ACQ196_MACCON_LENGTH;
		*ACQ196_MACCON |= depth - COUNTS_FROM_ZERO;
	}
	
	return strlen(buf);
}

static DEVICE_ATTR(depth, S_IRUGO|S_IWUGO, show_depth, store_depth);

static ssize_t show_subcon(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int subcon = *ACQ196_MACSUB & 0x0ffff;

        return sprintf(buf,"%d\n", subcon);
}

static ssize_t store_subcon(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int subcon = 0;

	if (sscanf(buf, "0x%x", &subcon) == 1 || 
	    sscanf(buf, "%d", &subcon) == 1       ){
		*ACQ196_MACSUB = subcon;
	}
	
	return strlen(buf);
}

static DEVICE_ATTR(subcon, S_IRUGO|S_IWUGO, show_subcon, store_subcon);





static int mk_mac_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_K1);
	DEVICE_CREATE_FILE(dev, &dev_attr_K2);
	DEVICE_CREATE_FILE(dev, &dev_attr_K3);
	DEVICE_CREATE_FILE(dev, &dev_attr_depth);
	DEVICE_CREATE_FILE(dev, &dev_attr_subcon);
	return 0;
}


static void acq196_mac_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq196_mac_driver;

static int acq196_mac_probe(struct device *dev)
{
	info("");
	DG->bigbuf.tblocks.transform = transform32;
	mk_mac_sysfs(dev);
	return 0;
}

static int acq196_mac_remove(struct device *dev)
{
	return 0;
}


static struct device_driver acq196_mac_driver = {
	.name     = "acq196_mac",
	.probe    = acq196_mac_probe,
	.remove   = acq196_mac_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq196_mac_device = {
	.name = "acq196_mac",
	.id   = 0,
	.dev = {
		.release    = acq196_mac_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq196_mac_init( void )
{
	int rc;
	acq200_debug = acq196_mac_debug;

	CAPDEF_set_word_size(acq196_mac_word_size);
	rc = driver_register(&acq196_mac_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&acq196_mac_device);
}


static void __exit
acq196_mac_exit_module(void)
{
	info("");
	platform_device_unregister(&acq196_mac_device);
	driver_unregister(&acq196_mac_driver);
}

module_init(acq196_mac_init);
module_exit(acq196_mac_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ196 MAC");


