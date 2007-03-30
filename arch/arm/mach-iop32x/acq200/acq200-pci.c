/*
 * arch/arm/mach-iop3xx/acq200-pci.c
 *
 * PCI support for the ACQ200 intelligent digitizer platform
 *
 * Author: Peter Milne <Peter.Milne@d-tacq.com>
 * Copyright (C) 2003 Peter Milne
 * Acknowledgements to 
 * Author: Rory Bolt <rorybolt@pacbell.net>
 * Copyright (C) 2002 Rory Bolt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/pci.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#include <asm/arch/iop321.h>
#include <asm/arch/acq200.h>

#include "acq200_debug.h"

extern void acq200_set_cpld_mask_byte(u8);

static int
acq200_map_irq(struct pci_dev *dev, u8 idsel, u8 pin)
{
	static int i3 = 0; // ETH0, 1 at same slot
	int rv = 0;
 	BUG_ON(pin < 1 || pin > 4);
 
	if (dev->bus->number > 1){
		/** offboard PCI devices - all map to IRQ31 */
		/** WORKTODO: enable ALL PCI interrupts + UART */
		acq200_set_cpld_mask_byte(0x1f);		
		rv = 31;
	}else{
		switch ( idsel ){
		case 1:
			rv = IRQ_ACQ200_HB;
			break;
		case 2:
			rv = IRQ_ACQ200_FP;
			break;
		case 3:
			rv = i3? IRQ_ACQ200_E1: IRQ_ACQ200_E0;
			i3 = !i3;
			break;
		default:
			rv = 31;    // PGM WORKTODO
		}
	}
 	printk( "PCI:PGM acq200_map_irq %p %d %d ret %d\n",
                dev, idsel, pin, rv );

	return rv;
}



/*
 * Setup the system data for controller 'nr'.   Return 0 if none found,
 * 1 if found, or negative error.
 */

static struct resource acq200_resources[] = {
	{
		name: "PCI IO Primary",
		flags: IORESOURCE_IO
	},{
		name: "PCI Memory Direct",
		flags: IORESOURCE_MEM
	}
};

int acq200_iop321_setup(int nr, struct pci_sys_data *sys)
{
	struct resource* res = acq200_resources;

	if ( nr != 0 ){
		return 0;
	}

	allocate_resource( &ioport_resource, &res[0], 
			   0x10000,
			   IOP321_PCI_LOWER_IO_PA,
			   IOP321_PCI_UPPER_IO_PA,
			   0x10000, NULL, NULL );

	allocate_resource( &iomem_resource, &res[1], 
			   ACQ200_PCIMEM_SIZE,
			   ACQ200_PCIMEM,
			   ACQ200_PCIMEM + ACQ200_PCIMEM_SIZE,
			   ACQ200_PCIMEM_SIZE, NULL, NULL );


			   
	sys->resource[0] = &res[0];
	sys->resource[1] = &res[1];
	sys->resource[2] = NULL;
	sys->io_offset = 0;
	return 1;
}




static int acq200_setup(int nr, struct pci_sys_data *sys)
{
	switch (nr) {
	case 0:
		sys->map_irq = acq200_map_irq;
		return acq200_iop321_setup(nr, sys);
	default:
		return 0;
	}
}
extern struct pci_bus *iop321_scan_bus(int nr, struct pci_sys_data *sys);
extern void iop321_init(void);

static struct hw_pci acq200_pci __initdata = {
	.swizzle	= pci_std_swizzle,
	.nr_controllers = 1,
	.setup		= acq200_setup,
	.scan		= iop321_scan_bus,
	.preinit	= iop321_init,
};

void mmr_setup(void)
{
	/* Translate outbound not used */
	*IOP321_OMWTVR0 = 0;
	*IOP321_OUMWTVR0 = 0;

        /* PGM: bus (pci) = phys (local) = virt */
	*IOP321_OIOWTVR = 0x90000000;

        /* Enable ATU with direct Addressing Enable */
	*IOP321_ATUCR = 0x102;        
	*IOP321_ATUCMD = 0x146; /* SERR|PARITY|MASTER|MEMORY */
	*IOP321_APMCSR = 3;     /* PM D3 HOT                 */

	/* IALRO - slave the MU region from PCI */
        /* use default physaddr - out of harm's way, this is never mapped */
	*IOP321_IALR0  =0xfffff000;
	*IOP321_IABAR0 =ACQ200_MU+0x0000000c;
	*IOP321_IATVR0 =0xff000000;

	/* set the inbound window. map incoming pci=0 to mem=0xc0000000 */
	*IOP321_IALR2 = PHYS_OFFSET; /* and map all the way to top of mem */

	*IOP321_IATVR2= PHYS_OFFSET;/* PHYS_ADDR?*/
	*IOP321_IABAR2= PHYS_OFFSET+0x0000000c;/* phys==bus */
}

static void __devinit pci_fixup_acq216(struct pci_dev *dev)
{
	int fixme = ((dev->class>>8)==0x680) && dev->bus->number > 0;
	info("HINT: cl:%04x %s"
	     "devfn 0x%08x v:%04x d:%04x "
	     "sv:%04x sd:%04x",
	     dev->class>>8, fixme? "FIX": "ok ",
		dev->devfn, dev->vendor, dev->device, 
	     dev->subsystem_vendor, dev->subsystem_device);


	if (fixme){
		/** limit downstream bridge resource to 1M */
		int i;

		for (i = 0; i < PCI_NUM_RESOURCES; i++) {
			if (dev->resource[i].end - dev->resource[i].start >
			    0x100000){
				dev->resource[i].end = dev->resource[i].start +
					0x100000;
			}
		}
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_HINT, 0x0029, pci_fixup_acq216);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_HINT, 0x0028, pci_fixup_acq216);

extern int G_iop321_pci_debug;

static void __init set_pcidebug(char **p)
{
	G_iop321_pci_debug = memparse((*p), p);
}
__early_param("pcidbg=", set_pcidebug);

static int __init acq200_pci_init(void)
{
	if (machine_is_acq200()){
		mmr_setup();
		pci_common_init(&acq200_pci);
	}
	return 0;
}

subsys_initcall(acq200_pci_init);




