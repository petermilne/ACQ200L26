/*
 * drivers/watchdog/iop3xx_wdt.c
 *
 * Watchdog driver for Intel IOP3xx IO Processors
 *
 * Author: Peter Milne <peter DOT milne AT d-tacq DOT com>
 *
 * Copyright 2006 (c) D-TACQ Solutions Ltd http://www.d-tacq.com
 *
 * Based on ixp4xx driver, Copyright (C) 2004 (c) MontaVista, Software, Inc.
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>

#include <asm/hardware.h>
#include <asm/uaccess.h>

static int nowayout = WATCHDOG_NOWAYOUT;
#ifdef CONFIG_ARCH_IOP321
static int heartbeat = 21;	/* (secs) 200MHz bus on IOP321 */
#else
static int heartbeat = 15;	/* (secs) 266MHz bus on IOP331 */
#endif
static unsigned long wdt_status;
static unsigned long boot_status;


#define	WDT_IN_USE		0
#define	WDT_OK_TO_CLOSE		1

#define IOP3XX_WDT_MAGIC1 0x1E1E1E1E
#define IOP3XX_WDT_MAGIC2 0xE1E1E1E1

static void
wdt_enable(void)
{
	asm volatile("mcr p6, 0, %0, c7, c1, 0" : : "r" (IOP3XX_WDT_MAGIC1));
	asm volatile("mcr p6, 0, %0, c7, c1, 0" : : "r" (IOP3XX_WDT_MAGIC2));
}

static int
iop3xx_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

	wdt_enable();

	return nonseekable_open(inode, file);
}

static ssize_t
iop3xx_wdt_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	if (len) {
		wdt_enable();
	}

	return len;
}

static struct watchdog_info ident = {
	.options	= WDIOF_CARDRESET | WDIOF_MAGICCLOSE | 
						WDIOF_KEEPALIVEPING,
	.identity	= "IOP3xx Watchdog",
};


static int
iop3xx_wdt_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	int time;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *)arg, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, (int *)arg);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(boot_status, (int *)arg);
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, (int *)arg);
		if (ret)
			break;

		if (time <= 0 || time > 60) {
			ret = -EINVAL;
			break;
		}

		wdt_enable();
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(heartbeat, (int *)arg);
		break;

	case WDIOC_KEEPALIVE:
		wdt_enable();
		ret = 0;
		break;
	}
	return ret;
}

static int
iop3xx_wdt_release(struct inode *inode, struct file *file)
{
	printk(KERN_CRIT "WATCHDOG: Device closed unexpectedly - "
					"timer will not stop\n");

	clear_bit(WDT_IN_USE, &wdt_status);
	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

	return 0;
}


static const struct file_operations iop3xx_wdt_fops =
{
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= iop3xx_wdt_write,
	.ioctl		= iop3xx_wdt_ioctl,
	.open		= iop3xx_wdt_open,
	.release	= iop3xx_wdt_release,
};

static struct miscdevice iop3xx_wdt_miscdev =
{
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &iop3xx_wdt_fops,
};

static int __init iop3xx_wdt_init(void)
{
	int ret;

	ret = misc_register(&iop3xx_wdt_miscdev);
	if (ret == 0)
		printk("iop3xx Watchdog Timer: heartbeat %d sec\n", heartbeat);

	boot_status = 0;
	return ret;
}

static void __exit iop3xx_wdt_exit(void)
{
	misc_deregister(&iop3xx_wdt_miscdev);
}


module_init(iop3xx_wdt_init);
module_exit(iop3xx_wdt_exit);

MODULE_AUTHOR("Peter Milne <peter DOT milne AT d-tacq DOT com>");
MODULE_DESCRIPTION("IOP3xx IO Processor Watchdog");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds (default 15s)");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

