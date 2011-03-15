/* ------------------------------------------------------------------------- */
/* acq100_rtmclk.c driver for acq100 rtm_clk		                     */
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

#define acq200_debug	debug
#define DIO_REG_TYPE (volatile u32*)
#include "acq200_debug.h"

char acq100_rtm_driver_name[] = "acq100-rtmclk";
#include "acqX00-rtm.h"

#define	RTMCLK_LICR \
	(DIO_REG_TYPE((unsigned)ACQ200_EXTERNIO+0x14))	
#define	RTMCLK_LOCR \
	(DIO_REG_TYPE((unsigned)ACQ200_EXTERNIO+0x18))	

#define	RTMCLK_CONTROL_OPTOS_OFF	(1<<3)

int debug = 1;

static int __init rtmclk_init( void )
{
	u32 rev = *RTM_DIO_CONTROL;
	if (rev >= 8){
		info("RTM_CLK located, setting up");
		SET_REG(RTM_DIO_CONTROL, =, RTM_REGCLR|RTMCLK_CONTROL_OPTOS_OFF);
		SET_REG(RTMCLK_LICR, =, 0x3);
//		SET_REG(RTMCLK_LOCR, =, 0x55);
		SET_REG(RTMCLK_LOCR, =, 0xaa);
	}else{
		err("RTM is not RTM_CLK, rev code %02x", rev);
	}
	return 0;
}


static void __exit
rtmclk_exit_module(void)
{
	info("");
}

module_init(rtmclk_init);
module_exit(rtmclk_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ1xx RTM_CLK");



