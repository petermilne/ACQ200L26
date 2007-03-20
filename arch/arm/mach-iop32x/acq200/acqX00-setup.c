/*
 * linux/arch/arm/mach-iop32x/acq200/acqX00-setup.c
 *
 * Author: Peter Milne www.d-tacq.com extract from iop321-setup.c
 * Author: Nicolas Pitre <nico@cam.org>
 * Copyright (C) 2001 MontaVista Software, Inc.
 * Copyright (C) 2004 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/system.h>
#include <asm/memory.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <linux/serial_8250.h>

/** @todo - not sure why ACQ100 rate is this way ... */
#define ACQ100_TICK_RATE	(33326000*6)
#define ACQ200_TICK_RATE	(33333000*6)

#define ACQ200_UART_XTAL 14181800

#include <asm/arch-iop32x/acqX00-irq.h>
#include <asm/arch-iop32x/iop321.h>

extern void __init acq200_init_time(unsigned long tick_rate);

/*
 * ACQX00 timer tick configuration.
 */
static void __init acqX00_timer_init(void)
{
	acq200_init_time(machine_is_acq100()? 
				ACQ100_TICK_RATE: ACQ200_TICK_RATE);
}

struct sys_timer acqX00_timer = {
	.init		= acqX00_timer_init,
	.offset		= iop_gettimeoffset,
};

static struct plat_serial8250_port acqX00_serial_platform_port[] = {
	{
	.membase        = (void*)ACQ200_UART,
	.mapbase	= ACQ200_UART,
	.irq		= IRQ_ACQX00_UART,
	.uartclk	= ACQ200_UART_XTAL,
	.regshift	= 0,
	.iotype		= UPIO_MEM,
	.flags		= UPF_BOOT_AUTOCONF|UPF_SHARE_IRQ,
	},
	{}
};

static struct resource uart_res = {
	.name = "acqX00 uart",
	.flags = IORESOURCE_MEM
};

static struct resource acqX00_uart_resources[] = {
	[0] = {
		.start = ACQ200_UART,
		.end = ACQ200_UART+0x40                          ,
		.flags = IORESOURCE_MEM,
		.parent = &uart_res
	}
};
static struct platform_device acqX00_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= acqX00_serial_platform_port,
	},
	.num_resources = 1,
	.resource = acqX00_uart_resources
};

void __init acqX00_init_machine(void)
{
	platform_device_register(&iop3xx_i2c0_device);
	platform_device_register(&iop3xx_i2c1_device);

#define ACQ200_UART_REGION_LEN 0x01000000
#define ACQ200_UART_REGION_START ACQ200_UART
#define ACQ200_UART_REGION_END \
        (ACQ200_UART_REGION_START+ACQ200_UART_REGION_LEN)


	allocate_resource(acq200_pbi_resource, &uart_res, 
		  ACQ200_UART_REGION_LEN,
		  ACQ200_UART_REGION_START,
		  ACQ200_UART_REGION_END,
		  ACQ200_UART_REGION_LEN,
		  NULL, NULL);

	acqX00_serial_platform_port[0].irq = 
			irq_canonicalize(acqX00_serial_platform_port[0].irq);

	platform_device_register(&acqX00_serial_device);
}

