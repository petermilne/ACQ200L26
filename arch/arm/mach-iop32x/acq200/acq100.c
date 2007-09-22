/*
 * linux/arch/arm/mach-iop3xx/acq200/acq100.c
 *
 * Author: Nicolas Pitre <nico@cam.org>
 * Copyright (C) 2001-2003 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Please do not add extern function declarations here. Instead 
 * Put them * in the appropriate board or chipset header file in 
 * include/asm-arm/arch-iop3xx.  This keeps the code much cleaner.
 *
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>

#include <asm/setup.h>
#include <asm/system.h>
#include <asm/memory.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <linux/ioport.h>

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/acq200.h>
#include <asm/arch/acqX00-irq.h>


static struct resource smc91x_resources[] = {
	[0] = {
		.start	= ACQ100_ETHERNET+0x300,
		.end	= ACQ100_ETHERNET+0x100000,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_ACQ100_ETH,
		.end	= IRQ_ACQ100_ETH,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};


static struct platform_device *devices[] __initdata = {
	&smc91x_device,
};


extern void __init acqX00_init_machine(void);

void __init acq100_init_machine(void)
{
	acqX00_init_machine();
	smc91x_resources[0].parent = acq200_pbi_resource;
	(void) platform_add_devices(devices, ARRAY_SIZE(devices));
}

extern struct sys_timer acqX00_timer;



MACHINE_START(ACQ100, "D-TACQ ACQ100")
	/* Maintainer: D-TACQ Solutions Ltd */
	.phys_io	= ACQ200_UART_P,
	.io_pg_offst	= ((ACQ200_UART) >> 18) & 0xfffc,
	.boot_params	= ACQ200_PARAMS,
	.fixup		= fixup_acq200,
        .timer		= &acqX00_timer,
	.map_io		= acq100_map_io,
	.init_irq	= acq100_init_irq,
        .init_machine	= acq100_init_machine,
MACHINE_END

MACHINE_START(ACQ132, "D-TACQ ACQ132")
	/* Maintainer: D-TACQ Solutions Ltd */
	.phys_io	= ACQ200_UART_P,
	.io_pg_offst	= ((ACQ200_UART) >> 18) & 0xfffc,
	.boot_params	= ACQ200_PARAMS,
	.fixup		= fixup_acq200,
        .timer		= &acqX00_timer,
	.map_io		= acq100_map_io,
	.init_irq	= acq132_init_irq,
        .init_machine	= acq100_init_machine,
MACHINE_END
