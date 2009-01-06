/*------------------------------------------------------------------------- */
/* acq200-irq.c irq special handling for ACQ200                             */
/*------------------------------------------------------------------------- */
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
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.               */
/*------------------------------------------------------------------------- */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/hardware.h>

#include <asm/arch-iop32x/iop321.h>

#include <asm/mach-types.h>

#include "acq200.h"

extern void __init iop32x_init_irq(void);



static void cpld_mask(unsigned int irq)
{
	*(volatile u8*)ACQ200_CPLD = 0;
}

static u8 acq200_cpld_mask;

static void cpld_unmask_acq200(unsigned int irq)
{
	*(volatile u8*)ACQ200_CPLD = acq200_cpld_mask;
}

void acq200_set_cpld_mask_byte(u8 mask) {
	acq200_cpld_mask = mask;
	cpld_unmask_acq200(31);
}

unsigned acq200_get_cpld_mask_byte(void) {
	return acq200_cpld_mask;
}

static void cpld_unmask(unsigned int irq)
{
	BUG();
}



static struct irq_chip hpi_chip = {
	.ack    = cpld_mask,
	.mask   = cpld_mask,
	.unmask = cpld_unmask,
	.name  = "ACQ200"
};


void acq200_mask_irq(int irq)
{
	irq_desc[irq].chip->mask(irq);
}

void acq200_unmask_irq(int irq)
{
	irq_desc[irq].chip->unmask(irq);	
}


static void __init __acq2xx_init_irq(void) {
	iop32x_init_irq();
	*IOP321_PCIIRSR = 0x0f; // all interrupts are inputs to chip
}

static void __init init_hpi_chip(void (*unmask)(unsigned int irq)){
	hpi_chip.unmask = unmask;
	set_irq_chip(IRQ_IOP32X_HPI, &hpi_chip);
	set_irq_handler(IRQ_IOP32X_HPI, handle_level_irq);
	set_irq_flags(IRQ_IOP32X_HPI, IRQF_VALID | IRQF_PROBE);
}

void __init acq200_init_irq(void)
{
	__acq2xx_init_irq();
	acq200_set_cpld_mask_byte(0x10);
	init_hpi_chip(cpld_unmask_acq200);
}

void __init acq100_init_irq(void)
{
	__acq2xx_init_irq();
	init_hpi_chip(cpld_unmask_acq200);
}

void __init acq132_init_irq(void)
/** @@todo */
{
	__acq2xx_init_irq();
	init_hpi_chip(cpld_unmask_acq200);
}

/*
 * IRQ31 MUX pattern
Slot 1: 0x8
Slot 2: 0x4
Slot 3: 0x2
Slot 4: 0x1.
 */
void acq100_setCpldMaskBit(int slot) {
/** NB: we have plug, not swap. */
	u8 enable = 0;

	if (slot == -1){
		acq200_cpld_mask |= 0x10;	/* use XINT3 */
		*(volatile u8*)ACQ200_CPLD = acq200_cpld_mask;
	}


	switch(slot%4){
	case 0:
		enable = 1 << 0; break;
	case 1:
		enable = 1 << 3; break;
	case 2:
		enable = 1 << 2; break;
	case 3:
		enable = 1 << 1; break;
	}

	acq200_cpld_mask |= enable;

	printk("acq100_setCpldMaskBit IRQ 31 slot %d enable 0x%x mask 0x%x\n",
	       slot, enable, acq200_cpld_mask);
}


