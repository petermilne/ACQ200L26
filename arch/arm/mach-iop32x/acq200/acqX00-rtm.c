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

int pulse_top_usec;
module_param(pulse_top_usec, int, 0600);

int poll_ms = 10;
module_param(poll_ms, int, 0644);

char acq100_rtm_driver_string[] = "D-TACQ RTM driver";
char acq100_rtm_driver_version[] = "$Revision: 1.6 $ build B1001 " __DATE__;
char acq100_rtm_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


#include "acqX00-rtm.h"

extern void acq200_setDO6_bit(int ibit, int value);

#define SYNC (3)		/* d3 */

#define SET_SYNC_ON  acq200_setDO6_bit(SYNC, 1)
#define SET_SYNC_OFF acq200_setDO6_bit(SYNC, 0)


#define INTERRUPTED	0	/* simulate interrupts for now */


int major;


unsigned acq200_getDIO32(void) {
	return read_inputs();
}

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
			udelay(pulse_top_usec);
			DIO_SET_OUTPUT0(ibit);
			break;
		case DIO_MASK_OUTPUT_NP:
			DIO_SET_OUTPUT1(ibit); set_outputs();
			DIO_SET_OUTPUT0(ibit); set_outputs();
			udelay(pulse_top_usec);
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

static ssize_t show_dev(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d:%d\n", major, 0);
}

static DEVICE_ATTR(dev, S_IRUGO, show_dev, 0);

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

static char last_store[32];

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
			strncpy(last_store, buf, sizeof(last_store));
			set_outputs();
			return strlen(buf);
		}else{
			snprintf(last_store, sizeof(last_store), 
						"ERR2:%s", buf);
		}
	}else{
		snprintf(last_store, sizeof(last_store), "ERR1:%s", buf);
	}
        return strlen(buf);
}

static ssize_t show_dio_bit(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	strcpy(buf, last_store);
	return strlen(last_store);
}


static DEVICE_ATTR(dio32_bit, S_IRUGO|S_IWUGO, show_dio_bit, store_dio_bit);


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
	DEVICE_CREATE_FILE(dev, &dev_attr_dev);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32_bit);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32_raw);
	DEVICE_CREATE_FILE(dev, &dev_attr_pulse_dio_sync);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio32_hex);
}

struct DeviceState {
	unsigned (*getDio)(void);
	unsigned state;	
};

extern unsigned acq200_getDIO6(void);


static int dio_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	struct DeviceState state;
	
	switch(minor){
	case 0:
		state.getDio = acq200_getDIO6;
		break;
	case 1:
		state.getDio = acq200_getDIO32;
		break;
	default:
		return -ENODEV;
	}	

	file->private_data = kzalloc(sizeof(struct DeviceState), GFP_KERNEL);
	if (file->private_data == 0){
		err("failed to allocate device state");
		return -ENOMEM;
	}
	memcpy(file->private_data, &state, sizeof(state));
	return 0;
}

static int wait_cos(struct DeviceState *ds)
{
	unsigned s2;
	int rc;

	s2 = ds->state;

	while((ds->state = ds->getDio()) == s2){
		rc = msleep_interruptible(poll_ms);
		if (rc){
			return rc;
		}
	}
	
	return 0;	
}

static ssize_t dio_read(
	struct file* file, char* buf, size_t count, loff_t* posp)
{
	struct DeviceState* ds = (struct DeviceState*)file->private_data;
	
	if (count < sizeof(unsigned)){
		return -EINVAL;
	}
	count = min(count, sizeof(unsigned));

	if ((file->f_flags & O_NONBLOCK) != 0 || *posp == 0){
		ds->state = ds->getDio();
	}else{
		if (wait_cos(ds)){
			return -ERESTARTSYS;
		}
	}
	if (copy_to_user(buf, &ds->state, count)){
		return -EFAULT;
	}
	(*posp)++;
	return count;
}
int dio_release (struct inode *inode, struct file *file)
{
	if (file->private_data){
		kfree(file->private_data);
	}
	return 0;
}


static void rtm_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver rtm_driver;

static int rtm_probe(struct device *dev)
{
	static struct file_operations dio_fops = {
		.open = dio_open,
		.read = dio_read,
		.release = dio_release
	};
	info("");
	major = register_chrdev(0, "dio", &dio_fops);
	if (major < 0){
		err("can't get major");
		return major;
	}
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



EXPORT_SYMBOL_GPL(acq200_getDIO32);

module_init(rtm_init);
module_exit(rtm_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ2xx RTM DIO32");


