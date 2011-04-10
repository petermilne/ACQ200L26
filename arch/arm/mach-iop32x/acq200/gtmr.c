/* ------------------------------------------------------------------------- */
/* gtmr driver for acq200				                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
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
 * GTMR/GTSR is an iop231 facility, a 32 bit counter with 20nsec tick
 * This driver handles overflows and presents a 64 bit timestamp,
 * Overflow in 2**64 * 2e-9 = .. never
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

#include <asm/arch/iop321.h>
#include <asm/arch/iop321-irqs.h>

#include "gtmr.h"

#include "acqX00-port.h"	

#define acq200_debug	debug
#include "acq200_debug.h"

#define REVID	"acq100_gtmr B1000"


int debug = 0;
module_param(debug, int, 0644);

int reads;
module_param(reads, int, 0644);

int reads_after_rollover_but_before_int;
module_param(reads_after_rollover_but_before_int, int, 0644);

int errors;
module_param(errors, int, 0644);

struct GTMR_DATA {
	unsigned long long timestamp;	
	spinlock_t lock;	
	unsigned last_gtsr;
	unsigned int_count;
};

struct GTMR_DATA gtmr_data;


static void update_timestamp(int is_interrupt)
{
	unsigned gtsr = *IOP321_GTSR;
	unsigned msw;
	unsigned long long new_ts;

	if (is_interrupt){
		msw = ++gtmr_data.int_count;
		gtmr_data.last_gtsr = gtsr;
	}else{
		if (unlikely(gtsr < gtmr_data.last_gtsr)){
			/* after rollover but before isr? */
			msw = gtmr_data.int_count+1;
			++reads_after_rollover_but_before_int;
		}else{
			msw = gtmr_data.int_count;
		}

		++reads;
	}
	new_ts = ((unsigned long long)msw << 32) | gtsr;

	if (new_ts < gtmr_data.timestamp){
		err("GTMR runs backwards! reads:%d", reads);
		++errors;
	}
	gtmr_data.timestamp = new_ts;

/*
	info("%3s: int:%d msw:%d last:%08x gtsr:%08x ts:%llx",
	     is_interrupt? "INT": "reg",
	     gtmr_data.int_count, msw, gtmr_data.last_gtsr,
	     gtsr, gtmr_data.timestamp);
*/
}

unsigned long long gtmr_update_timestamp(void)
{
	unsigned long flags;

	spin_lock_irqsave(&gtmr_data.lock, flags);
	update_timestamp(0);
	spin_unlock_irqrestore(&gtmr_data.lock, flags);
	return gtmr_data.timestamp;
}

static irqreturn_t gtmr_isr(int irq, void *dev_id)
{
	u32 flags = *IOP321_EMISR;
	update_timestamp(flags&0x01);	/* 0x01 : GTSR OVERFLOW */
	*IOP321_EMISR = flags;		/* INT ACK */
	return IRQ_HANDLED;
}

/* show msecs: awk '{ print $1/50000 }' */
static ssize_t show_msecs(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf
	)
{
	unsigned long long ts = gtmr_update_timestamp();
	do_div(ts, GTMR_TICK_PER_MSEC);

	return sprintf(buf, "%llu\n", ts);
}

static DEVICE_ATTR(msecs, S_IRUGO, show_msecs, 0);

static ssize_t show_gtmr(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf
	)
{
	return sprintf(buf, "%llu\n", gtmr_update_timestamp());
}

static DEVICE_ATTR(GTMR, S_IRUGO, show_gtmr, 0);

static void mk_gtmr_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_msecs);
	DEVICE_CREATE_FILE(dev, &dev_attr_GTMR);
}


static int gtmr_probe(struct device *dev)
{
	if (request_irq(IRQ_IOP321_PERFMON, gtmr_isr, 0, "gtmr", 0)){
		err("request_irq failed");
		return -ENODEV;
	}else{
		memset(&gtmr_data, 0, sizeof(gtmr_data));
		gtmr_data.lock = SPIN_LOCK_UNLOCKED;
		gtmr_update_timestamp();
		*IOP321_GTMR = IOP321_GTMR_INTEN;
		mk_gtmr_sysfs(dev);
		return 0;
	}
}
static void gtmr_dev_release(struct device * dev)
{

}

static int gtmr_remove(struct device *dev)
{
	info("");

	*IOP321_GTMR = 0;
	free_irq(IRQ_IOP321_PERFMON, 0);
	return 0;
}



static u64 dma_mask = 0x00000000ffffffff;
static struct platform_device gtmr_device = {
	.name	= "gtmr",
	.id	= 0,
	.dev	= {
		.release	= gtmr_dev_release,
		.dma_mask	= &dma_mask
	}
};



static struct device_driver gtmr_driver = {
	.name	= "gtmr",
	.probe	= gtmr_probe,
	.remove = gtmr_remove,
	.bus	= &platform_bus_type
};




static int __init gtmr_init( void )
{
	int rc;

	info(REVID);

	rc = driver_register(&gtmr_driver);
	if (rc != 0){
		goto driver_fail;
	}
	rc = platform_device_register(&gtmr_device);
	if (rc != 0){
		goto device_fail;
	}
	
	return 0;

device_fail:
	driver_unregister(&gtmr_driver);
driver_fail:
	return rc;	
}


static void __exit
gtmr_exit_module(void)
{
	platform_device_unregister(&gtmr_device);
	driver_unregister(&gtmr_driver);
}

module_init(gtmr_init);
module_exit(gtmr_exit_module);

EXPORT_SYMBOL_GPL(gtmr_update_timestamp);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for acq200 precision timestamp");



