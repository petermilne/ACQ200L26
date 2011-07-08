/* ------------------------------------------------------------------------- */
/* acq100_rtm_t_sfp_i2c.c RTM-T SFP monitoring driver hooks 		     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
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


/** @file acq100_rtm_t_sfp_i2c.c  RTM-T SFP monitoring driver hooks */
#define ACQ196

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/moduleparam.h>

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#define acq200_debug rtm_t_debug
#include "acq200.h"
#include "acq200_debug.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"

#include "rtm-t.h"
#include "acq100_rtm_t.h"

#define REVID	"acq100_rtm_t B1000"


#include <linux/i2c-gpio.h>

int rtm_t_debug;
module_param(rtm_t_debug, int , 0644);

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>




int gpio_get_value(unsigned gpio)
{
	u32 reg = *RTMT_REG(RTMT_Q_I2C);
	int bit = (reg& (1<<gpio)) != 0;
	dbg(1, "%d -> %d", gpio, bit);

	return bit;
}
void gpio_set_value(unsigned gpio, int value)
{
	u32 reg = *RTMT_REG(RTMT_Q_I2C);
	dbg(1, "%d := %d", gpio, value);

	if (value){
		reg |= 1 << (gpio+8);
	}else{
		reg &= ~(1 << (gpio+8));
	}
	*RTMT_REG(RTMT_Q_I2C) = reg;
}


static struct i2c_gpio_platform_data rtm_t_i2c_gpio_data = {
	.sda_pin	= RTMT_Q_I2C_SDA_R,
	.scl_pin	= RTMT_Q_I2C_SCL_R,
	.udelay = 1,				/* AVO min time=1usec */
	.sda_is_open_drain = 1,
	.scl_is_open_drain = 1
};

static struct platform_device rtm_t_i2c_gpio = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev	 = {
		.platform_data	= &rtm_t_i2c_gpio_data,
	},
};


static int __init acq100_rtm_t_sfp_i2c_init(void)
{
	int rc;

	rc = platform_device_register(&rtm_t_i2c_gpio);
	if (rc != 0){
		err("platform device register failed");
	}
	return rc;
}

static void __exit acq100_rtm_t_sfp_i2c_exit(void)
{
	platform_device_unregister(&rtm_t_i2c_gpio);
}

module_init(acq100_rtm_t_sfp_i2c_init);
module_exit(acq100_rtm_t_sfp_i2c_exit);

EXPORT_SYMBOL_GPL(gpio_get_value);
EXPORT_SYMBOL_GPL(gpio_set_value);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("ACQ100 RTM-T sfp i2c adapter");

