/* ------------------------------------------------------------------------- */
/* acq100_rtm_t_uart.c  - RTM-T uart shim                                    */
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


#include <linux/kernel.h>
#include <linux/serial_8250.h>
#include <asm/arch-iop32x/acqX00-irq.h>

#include "rtm-t.h"		/* @@todo - clash? .. */
#include "acq100_rtm_t.h"

#define ACQ196

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif
#include <linux/moduleparam.h>

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"

#define RTM_T_UART_2		(ACQ200_UART+0x100)
#define RTM_T_UART_2_IRQ	IRQ_ACQ100_ETH
#define RTM_T_UART_2_NEW_IRQ	IRQ_ACQ100_UART


int force_slip_newirq;
module_param(force_slip_newirq, int, 0444);



static struct plat_serial8250_port acqX00_serial_platform_port[] = {
	{
	.membase        = (void*)ACQ200_UART+0x100,
	.mapbase	= RTM_T_UART_2,
	.irq		= RTM_T_UART_2_IRQ,
	.uartclk	= RTM_T_UART_XTAL,
	.regshift	= 0,
	.iotype		= UPIO_MEM,
	.flags		= UPF_BOOT_AUTOCONF|UPF_SHARE_IRQ,
	},
	{}
};

/*
static struct resource uart_res = {
	.name = "acqX00 uart",
	.flags = IORESOURCE_MEM
};
*/
static struct resource acqX00_uart_resources[] = {
	[0] = {
		.start = RTM_T_UART_2,
		.end = RTM_T_UART_2+0x40,
		.flags = IORESOURCE_MEM,
		.parent = &iomem_resource
	}
};

static void acqX00_serial_platform_port_release(struct device * dev)
{
	info("");
}

static struct platform_device acq100_rtm_t_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= acqX00_serial_platform_port,
		.release = acqX00_serial_platform_port_release
	},

	.num_resources = 1,
	.resource = acqX00_uart_resources
};



int __init acq100_rtm_t_uart_init(int new_irq)
{
	if (force_slip_newirq || new_irq){
		acqX00_serial_platform_port[0].irq = RTM_T_UART_2_NEW_IRQ;
	}
	return platform_device_register(&acq100_rtm_t_serial_device);
}

void __exit acq100_rtm_t_uart_exit(void)
{
	platform_device_unregister(&acq100_rtm_t_serial_device);
}



