/*
 * linux/arch/arm/mach-iop3xx/acq200/acq200.c
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
#include <linux/config.h>
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

#include <asm/arch-iop32x/acqX00-irq.h>
#include <asm/arch-iop32x/iop321.h>

extern void __init acqX00_init_machine(void);
extern struct sys_timer acqX00_timer;

void __init acq200_init_machine(void)
{
	acqX00_init_machine();
}

MACHINE_START(ACQ200, "D-TACQ ACQ200")
	/* Maintainer: D-TACQ Solutions Ltd */
	.phys_io = ACQ200_UART_P,
	.io_pg_offst = ((ACQ200_UART) >> 18) & 0xfffc,
	.boot_params = ACQ200_PARAMS,
	.fixup = fixup_acq200,
        .timer		= &acqX00_timer,
	.map_io(acq200_map_io),
	.init_irq = acq200_init_irq,
        .init_machine = acq200_init_machine,
MACHINE_END
