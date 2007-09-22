/* ------------------------------------------------------------------------- */
/* iop321-auxtimer.c driver for timer1                                       */
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


#include <linux/platform_device.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm/arch-iop32x/iop321-irqs.h>
#include <asm/arch-iop32x/iop321.h>

#include "acq200.h"
#include "iop321-auxtimer.h"


#define AUXTIMER_STUBOUT 1


static struct AuxTimerClient* timer1_client;
static int timer1_hz;

extern unsigned acq200_setAuxClock(unsigned hz);
extern unsigned acq200_getAuxClock(void);

void iop321_auxtimer_func(void)
{
	struct AuxTimerClient *client = timer1_client;

	if (client){
		client->func(client->clidata);
	}
}



int iop321_hookAuxTimer(struct AuxTimerClient* client, int hz)
/*
 * connect to timer, if hz > 0, start running,, == 0 unhook and stop
 * RETURNS 0 or -ERR.
 */
{
	if (timer1_client != 0 && timer1_client != client){
		return -EBUSY;
	}else if (hz == 0){
		timer1_client = 0;
		return 0;
	}else{
		int oldhz = acq200_getAuxClock();

		timer1_client = client;
		if (hz != oldhz){
			acq200_setAuxClock(hz);
		}
		return 0;
	}
}


static int test_data;

static void test_func(unsigned long clidata)
{
	int* ptest_count = (int*)clidata;
	++(*ptest_count);
}


static struct AuxTimerClient test_client = {
	.func = test_func,
	.clidata = (unsigned long)&test_data
};

static ssize_t show_test(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct AuxTimerClient* client = timer1_client; /* atomic */
	if (client){
		if (client == &test_client){
			return sprintf(buf, "%d Hz count %d\n", 
				       timer1_hz, test_data);
		}else{
			return sprintf(buf, "%d Hz\n", timer1_hz);
		}
	}else{
		return sprintf(buf, "x\n");
	}
}

static ssize_t store_test(
	struct device * dev, 
	struct device_attribute *attr,	
	const char * buf, size_t count)
{
	int hz;

	if (sscanf(buf, "%d", &hz) == 1){
		iop321_hookAuxTimer(
			timer1_client==0? &test_client: timer1_client, hz);
	}

	return strlen(buf);
}

static DEVICE_ATTR(test, S_IRUGO|S_IWUGO, show_test, store_test);


static void mk_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_test);
}

static void rm_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_test);
}


static int iop321_auxtimer_probe(struct device * dev)
{
	mk_sysfs(dev);
	return 0;
}



static int iop321_auxtimer_remove(struct device * dev)
{
	timer1_client = 0;
	rm_sysfs(dev);
	return 0;
}




static struct device_driver auxtimer_device_driver = {
	.name		= "iop321auxtimer",
	.bus		= &platform_bus_type,
	.probe          = iop321_auxtimer_probe,
	.remove         = iop321_auxtimer_remove,
};

static void iop321_auxtimer_dev_release(struct device * dev)
{

}


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device auxtimer_device = {
	.name		= "iop321auxtimer",
	.id		= 0,
	.dev = {
		.kobj.name	= "iop321auxtimer",
		.release        = iop321_auxtimer_dev_release,
		.dma_mask       = &dma_mask
	}
};


static int __init iop321_auxtimer_init( void )
{
	int rc = driver_register(&auxtimer_device_driver);

	if (rc){
		return rc;
	}

	return platform_device_register(&auxtimer_device);
}


static void __exit
iop321_auxtimer_exit_module(void)
// Remove DRIVER resources on module exit
{
	platform_device_unregister(&auxtimer_device);	
	driver_unregister(&auxtimer_device_driver);
}

module_init(iop321_auxtimer_init);
module_exit(iop321_auxtimer_exit_module);

EXPORT_SYMBOL_GPL(iop321_hookAuxTimer);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for iop321 aux timer");

