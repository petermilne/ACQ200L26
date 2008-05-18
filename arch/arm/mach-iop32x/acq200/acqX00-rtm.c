/* ------------------------------------------------------------------------- */
/* acqX00-rtm.c driver for acq200 DIO32                                      */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
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
 * Module: provides hook to dio32.
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

int rtm_debug;
module_param(rtm_debug, int, 0664);




char acq100_rtm_driver_string[] = "D-TACQ RTM driver";
char acq100_rtm_driver_version[] = "$Revision: 1.6 $ build B1001 " __DATE__;
char acq100_rtm_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


#include "acqX00-rtm.h"

extern void acq200_setDO6_bit(int ibit, int value);

#define SYNC (3)		/* d3 */

#define SET_SYNC_ON  acq200_setDO6_bit(SYNC, 1)
#define SET_SYNC_OFF acq200_setDO6_bit(SYNC, 0)

static ssize_t store_dio(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ibit;

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		switch(buf[ibit]){
		case DIO_MASK_OUTPUT1:
			DIO_SET_OUTPUT1(ibit);
			break;
		case DIO_MASK_OUTPUT0:
			DIO_SET_OUTPUT0(ibit);
			break;
		case DIO_MASK_OUTPUT_PP:
			DIO_SET_OUTPUT0(ibit); set_outputs();
			DIO_SET_OUTPUT1(ibit); set_outputs();
			DIO_SET_OUTPUT0(ibit);
			break;
		case DIO_MASK_OUTPUT_NP:
			DIO_SET_OUTPUT1(ibit); set_outputs();
			DIO_SET_OUTPUT0(ibit); set_outputs();
			DIO_SET_OUTPUT1(ibit);
			break;
		case DIO_MASK_INPUT:
			DIO_SET_INPUT(ibit);
			break;
		default:
			; /* do nothing 'x' by convention */
		}
	}
	set_outputs();
        return strlen(buf);
}

static ssize_t show_dio(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int ibit;

	read_inputs();

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		if (DIO_IS_OUTPUT(ibit)){
			buf[ibit] = DIO_IS_OUTPUT1(ibit)?
				DIO_MASK_OUTPUT1: DIO_MASK_OUTPUT0;
		}else{
			buf[ibit] = DIO_IS_INPUTH(ibit)?
				DIO_MASK_INPUT1: DIO_MASK_INPUT0;
		} 
	}
	buf[ibit++] = '\n';
	buf[ibit] = '\0';
	return strlen(buf);
}

static DEVICE_ATTR(dio32, S_IRUGO|S_IWUGO, show_dio, store_dio);

#define MAXDIOBIT 32

static ssize_t store_dio_bit(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ibit;
	char value;

	if (sscanf(buf, "%d %c", &ibit, &value) == 2 ||
            sscanf(buf, "%d=%c", &ibit, &value) == 2    ){
		if (ibit >= 0 && ibit < MAXDIOBIT){
			switch(value){
			case DIO_MASK_OUTPUT1:
				DIO_SET_OUTPUT1(ibit);
				break;
			case DIO_MASK_OUTPUT0:
				DIO_SET_OUTPUT0(ibit);
				break;
			case DIO_MASK_OUTPUT_PP:
				DIO_SET_OUTPUT0(ibit);
				set_outputs();
				DIO_SET_OUTPUT1(ibit);
				set_outputs();
				DIO_SET_OUTPUT0(ibit);
				break;
			case DIO_MASK_OUTPUT_NP:
				DIO_SET_OUTPUT1(ibit);
				set_outputs();
				DIO_SET_OUTPUT0(ibit);
				set_outputs();
				DIO_SET_OUTPUT1(ibit);
				break;
			case DIO_MASK_INPUT:
			default:
				DIO_SET_INPUT(ibit);
			}
			set_outputs();
		}
	}

        return strlen(buf);
}

static DEVICE_ATTR(dio32_bit, S_IRUGO|S_IWUGO, 0, store_dio_bit);


static ssize_t show_dio_raw(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	read_inputs();
	memcpy(buf, &dio32.input_values, sizeof(u32));
	return sizeof(u32);
}

static ssize_t store_dio_raw(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	memcpy(&dio32.output_values, buf, sizeof(u32));
	set_outputs();
	return strlen(buf);
}

static DEVICE_ATTR(dio32_raw, S_IRUGO|S_IWUGO, show_dio_raw, store_dio_raw);



static ssize_t store_pulse_dio_sync(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int usecs1 = 10;
	int usecs0 = 0;
	int repeat = 1;

	if (sscanf(buf, "%d,%d,%d", &usecs1, &usecs0, &repeat) == 2){
		repeat = max(0, repeat);
		repeat = min(repeat, 10);
	}

	while(repeat--){
		dio32.output_values = ~1;
		SET_SYNC_ON;
		set_outputs();
		udelay(usecs1);

		dio32.output_values = 0;
		SET_SYNC_OFF;
		set_outputs();
		if (repeat){
			udelay(usecs0);
		}
	}

	return strlen(buf);
}

static DEVICE_ATTR(pulse_dio_sync, 
	S_IRUGO|S_IWUGO, 0, store_pulse_dio_sync);


static ssize_t show_dio_hex(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	read_inputs();
	return sprintf(buf, "%08x %08x",  
		       dio32.input_values, dio32.is_output);
}

static ssize_t store_dio_hex(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int nscan = sscanf(buf, "%x %x", 
			   &dio32.output_values,
			   &dio32.is_output);

	if (nscan > 0){
		set_outputs();
	}
	return strlen(buf);
}

static DEVICE_ATTR(dio32_hex, S_IRUGO|S_IWUGO, show_dio_hex, store_dio_hex);




static void mk_rtm_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32_bit);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32_raw);
	DEVICE_CREATE_FILE(dev, &dev_attr_pulse_dio_sync);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32_hex);
}

static void rtm_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver rtm_driver;

static int rtm_probe(struct device *dev)
{
	info("");
	mk_rtm_sysfs(dev);
	init_inputs();
	return 0;
}

static int rtm_remove(struct device *dev)
{
	return 0;
}


static struct device_driver rtm_driver = {
	.name     = "rtm",
	.probe    = rtm_probe,
	.remove   = rtm_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device rtm_device = {
	.name = "rtm",
	.id   = 0,
	.dev = {
		.release    = rtm_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init rtm_init( void )
{
	int rc;
	acq200_debug = rtm_debug;

	rc = driver_register(&rtm_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&rtm_device);
}


static void __exit
rtm_exit_module(void)
{
	info("");
	platform_device_unregister(&rtm_device);
	driver_unregister(&rtm_driver);
}


unsigned acq200_getDIO32(void) {
	return read_inputs();
}

EXPORT_SYMBOL_GPL(acq200_getDIO32);

module_init(rtm_init);
module_exit(rtm_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ2xx RTM DIO32");


