/* ------------------------------------------------------------------------- */
/* acq100_rtmclk.c driver for acq100 rtm_clk		                     */
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

#include "acqX00-port.h"

#define acq200_debug	debug
#define DIO_REG_TYPE (volatile u32*)
#include "acq200_debug.h"

char acq100_rtm_driver_name[] = "acq100-rtmclk";
#include "acqX00-rtm.h"

#define REVID	"acq100_rtmclk B1000"

#define	RTMCLK_CONTROL_OPTOS_OFF	(1<<3)

#define	RTMCLK_LICR \
	(DIO_REG_TYPE((unsigned)ACQ200_EXTERNIO+0x14))	

#define RTMCLK_LICR_DI0	0x0
#define RTMCLK_LICR_DI1 0x1
#define RTMCLK_LICR_DI2 0x2
#define RTMCLK_LICR_NC  0x3	/* DISCONNECTED */

#define RTMCLK_LICR_MASK 0x3

#define	RTMCLK_LOCR \
	(DIO_REG_TYPE((unsigned)ACQ200_EXTERNIO+0x18))	

#define LOCR_LIN	0x0	/* Lemo IN */
#define LOCR_DO0	0x1
#define LOCR_DO1	0x2
#define LOCR_DO2	0x3

#define LOCR_MASK	0x3

#define LOCR1_SHL	0
#define LOCR2_SHL	2
#define LOCR3_SHL	4
#define LOCR4_SHL	6

#define LOCR1(lv)	((lv) << LOCR1_SHL)
#define LOCR2(lv)	((lv) << LOCR2_SHL)
#define LOCR3(lv)	((lv) << LOCR3_SHL)
#define LOCR4(lv)	((lv) << LOCR4_SHL)

#define LOCR_ALL(lv)	(LOCR1(lv)|LOCR2(lv)|LOCR3(lv)|LOCR4(lv))

static unsigned licr;
static unsigned locr;


int debug = 0;
module_param(debug, int, 0644);

#define KEYMAX	3		/* compare this many letters */

#define NKEYS	4		/* 4 options each function */

/** @@warning: LUT order MUST match RTMCLK_LICR_xxx */
const char* LICR_LUT[] = {
	"DI0", "DI1", "DI2", "none"
};

/** @@warning: LUT order MUST match LOCR_xxx */
const char* LOCR1_LUT[] = {
	"LIN", "DO0", "DO1", "DO2"
};

const char* LOCR2_LUT[] = {
	"LIN", "DO0", "DO1", "DO3"
};
const char* LOCR3_LUT[] = {
	"LIN", "DO0", "DO1", "DO4"
};
const char* LOCR4_LUT[] = {
	"LIN", "DO0", "DO1", "DO5"
};

const char** LOCRX [] = {
	/* index from 1 */
	0, LOCR1_LUT, LOCR2_LUT, LOCR3_LUT, LOCR4_LUT
};

int LOCRX_SHL [] = {
	/* index from 1 */
	0, LOCR1_SHL, LOCR2_SHL, LOCR3_SHL, LOCR4_SHL
};

static ssize_t store_licr(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	char key[KEYMAX+1];
	int ik;

	strncpy(key, buf, KEYMAX); key[KEYMAX] = '\0';

	for (ik = 0; ik < NKEYS; ++ik){
		if (strncmp(key, LICR_LUT[ik], KEYMAX) == 0){
			dbg(1, "match %s", key);

			licr &= ~RTMCLK_LICR_MASK;
			licr |= ik;
			SET_REG(RTMCLK_LICR, =, licr);
			return count;
		}		
	}
	return -EINVAL;	       	
}

static ssize_t show_licr(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
	)
{
	int ik = licr&RTMCLK_LICR_MASK;
	
	return sprintf(buf, "%s\n", LICR_LUT[ik]);
}	


static DEVICE_ATTR(LEMO_IN, S_IRUGO|S_IWUGO, show_licr, store_licr);

static ssize_t store_locr(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count,
	const char** LUT,
	int shl)	
{
	char key[4];
	int ik;

	strncpy(key, buf, 3); key[3] = '\0';

	for (ik = 0; ik < 4; ++ik){
		if (strcmp(key, LUT[ik]) == 0){
			dbg(1, "match %s  (%d<<%d)", key, ik, shl);

			locr &= ~(LOCR_MASK << shl);
			locr |= ik << shl;
			SET_REG(RTMCLK_LOCR, =, locr);
			return count;
		}		
	}
	return -EINVAL;	       
}

static ssize_t show_locr(
	struct device *dev,
	struct device_attribute *attr,
	char *buf,
	const char** LUT,
	int shl)
{
	int ik = (locr >> shl)&LOCR_MASK;
	
	return sprintf(buf, "%s\n", LUT[ik]);
}	

#define LOCR_KNOB(LOCRN)					\
static ssize_t store_locr_##LOCRN(				\
	struct device *dev,					\
	struct device_attribute *attr,				\
	const char *buf,					\
	size_t count)						\
{								\
	return store_locr(dev, attr, buf, count,		\
			LOCRX[LOCRN], LOCRX_SHL[LOCRN]);	\
}								\
								\
static ssize_t show_locr_##LOCRN(				\
	struct device *dev,					\
	struct device_attribute *attr,				\
	char *buf)						\
{								\
	return show_locr(dev, attr, buf,			\
			LOCRX[LOCRN], LOCRX_SHL[LOCRN]);	\
}								\
								\
static DEVICE_ATTR(LEMO_OUT_##LOCRN,  S_IRUGO|S_IWUGO,		\
		   show_locr_##LOCRN, store_locr_##LOCRN)

LOCR_KNOB(1);
LOCR_KNOB(2);
LOCR_KNOB(3);
LOCR_KNOB(4);
	
static void mk_rtmclk_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_LEMO_IN);
	DEVICE_CREATE_FILE(dev, &dev_attr_LEMO_OUT_1);
	DEVICE_CREATE_FILE(dev, &dev_attr_LEMO_OUT_2);
	DEVICE_CREATE_FILE(dev, &dev_attr_LEMO_OUT_3);
	DEVICE_CREATE_FILE(dev, &dev_attr_LEMO_OUT_4);
}

static int rtmclk_probe(struct device *dev)
{
	unsigned rev = RTM_REVID(*RTM_DIO_CONTROL);

	if (rev >= 8){
		info("RTM_CLK located, setting up");
		SET_REG(RTM_DIO_CONTROL, |=, RTMCLK_CONTROL_OPTOS_OFF);
		SET_REG(RTMCLK_LICR, =, licr = RTMCLK_LICR_NC);
		SET_REG(RTMCLK_LOCR, =, locr = LOCR_ALL(LOCR_DO1));

		if (rev >= 9){
			info("REV 9+ add LOCR opts");
			mk_rtmclk_sysfs(dev);
		}
		return 0;
	}else{
		err("RTM is not RTM_CLK, rev code %02x", rev);
		return -ENODEV;
	}
}
static void rtmclk_dev_release(struct device * dev)
{
	info("");
	info("restore LIN default");
	SET_REG(RTMCLK_LICR, =, licr = RTMCLK_LICR_DI0);
	SET_REG(RTMCLK_LOCR, =, locr = LOCR_ALL(LOCR_LIN));
}

static int rtmclk_remove(struct device *dev)
{
	return 0;
}


static u64 dma_mask = 0x00000000ffffffff;
static struct platform_device rtmclk_device = {
	.name	= "acq100_rtmclk",
	.id	= 0,
	.dev	= {
		.release	= rtmclk_dev_release,
		.dma_mask	= &dma_mask
	}
};



static struct device_driver rtmclk_driver = {
	.name	= "acq100_rtmclk",
	.probe	= rtmclk_probe,
	.remove = rtmclk_remove,
	.bus	= &platform_bus_type
};




static int __init rtmclk_init( void )
{
	int rc;

	info(REVID);

	rc = driver_register(&rtmclk_driver);
	if (rc != 0){
		goto driver_fail;
	}
	rc = platform_device_register(&rtmclk_device);
	if (rc != 0){
		goto device_fail;
	}
	
	return 0;

device_fail:
	driver_unregister(&rtmclk_driver);
driver_fail:
	return rc;	
}


static void __exit
rtmclk_exit_module(void)
{
	platform_device_unregister(&rtmclk_device);
	driver_unregister(&rtmclk_driver);
}

module_init(rtmclk_init);
module_exit(rtmclk_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ1xx RTM_CLK");



