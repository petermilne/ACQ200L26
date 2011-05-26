
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

#define REVID	"acq100_rtm_t B1000"


#include <linux/i2c-gpio.h>

int rtm_t_debug;
module_param(rtm_t_debug, int , 0644);



static struct i2c_gpio_platform_data rtm_t_i2c_gpio_data = {
	.sda_pin	= RTMT_Q_I2C_SDA_R,
	.scl_pin	= RTMT_Q_I2C_SCL_R,
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("ACQ100 RTM-T sfp i2c adapter");

