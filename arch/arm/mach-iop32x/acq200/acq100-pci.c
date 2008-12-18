/*
 * arch/arm/mach-iop3xx/acq100-pci.c
 *
 * PCI support for the ACQ100 intelligent digitizer platform
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

#include <asm/arch/iop321.h>
#include <asm/arch/acq200.h>

#include <asm/setup.h>

#include "acq200_debug.h"
#include "acq196.h"



extern void acq100_setCpldMaskBit(int slot);

int acq100_is_system_slot_enabled(void);

#define IRQ_EXT_PCI_ORIG	31
#define IRQ_EXT_PCI_CUSTOM	29	/* special for acq196_022 */
#define IRQ_EXT_PCI_NEW		30	/* REV3 boards	          */

static int is_system_slot = 0;
static int irq_ext_pci = IRQ_EXT_PCI_ORIG;

static int
acq100_map_irq(struct pci_dev *dev, u8 idsel, u8 pin)
{
	int rv = 0;
	int slot = 17 - idsel;         /** PGM cooks the magic stew */
 	BUG_ON(pin < 1 || pin > 4);
 

	switch ( idsel ){
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		rv = irq_ext_pci;
		if (irq_ext_pci == 31){
			acq100_setCpldMaskBit(slot);
		}
		break;
	}

 	printk( "PCI:acq100_map_irq dev:%p idsel:%d slot:%d pin:%d ret %d\n",
                dev, idsel, slot, pin, rv );
 
	return rv;
}


static int
acq132_map_irq(struct pci_dev *dev, u8 idsel, u8 pin)
{
	int rv = 0;
	int slot = 17 - idsel;         /** PGM cooks the magic stew */
 	BUG_ON(pin < 1 || pin > 4);
 

/** @@todo ... what are the other pci interrupt sources ? */
	switch ( idsel ){
	case 1:
		rv = 30;	       /* gigE */
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		rv = irq_ext_pci;
		if (irq_ext_pci == 31){
			acq100_setCpldMaskBit(slot);
		}
		break;
	}

 	printk( "PCI:acq132_map_irq dev:%p idsel:%d slot:%d pin:%d ret %d\n",
                dev, idsel, slot, pin, rv );
 
	return rv;
}



/*
 * Setup the system data for controller 'nr'.   Return 0 if none found,
 * 1 if found, or negative error.
 */

static struct resource acq100_resources[] = {
	{
		name: "PCI IO Primary",
		flags: IORESOURCE_IO
	},{
		name: "PCI Memory Direct",
		flags: IORESOURCE_MEM
	}
};


static int acq100_allocate_resources(void)
{
 	struct resource* res = acq100_resources;
  
 	allocate_resource( &ioport_resource, &res[0], 
  			   0x10000,
  			   IOP321_PCI_LOWER_IO_PA + 0x6e000000,
  			   IOP321_PCI_UPPER_IO_PA + 0x6e000000,
  			   0x10000, NULL, NULL );
  
 	allocate_resource( &iomem_resource, &res[1],
  			   ACQ100_PCIMEM_END-ACQ100_PCIMEM_START,
  			   ACQ100_PCIMEM_START,
  			   ACQ100_PCIMEM_END,
  			   ACQ100_PCIMEM_END-ACQ100_PCIMEM_START, NULL, NULL );
 	return 0;
}

extern int acq200_iop321_setup(int nr, struct pci_sys_data *sys);

static int acq100_iop321_setup(int nr, struct pci_sys_data *sys)
{
	if ( nr != 0 ){
		return 0;
	}else{
		if (acq100_is_system_slot_enabled()){
			acq200_iop321_setup(nr, sys);
		}else{
			struct resource* res = acq100_resources;
			acq100_allocate_resources();

			sys->resource[0] = &res[0];
			sys->resource[1] = &res[1];
			sys->resource[2] = NULL;
			sys->io_offset   = 0x6e000000;
		}
		return 1;
	}
}

int acq132_is_system_slot(void);
int acq132_is_standalone(void);

static int acq132_iop321_setup(int nr, struct pci_sys_data *sys)
{
	if ( nr != 0 ){
		return 0;
	}else{
		if (acq132_is_system_slot() ||
		    acq132_is_standalone() ){
			acq200_iop321_setup(nr, sys);
		}else{
			struct resource* res = acq100_resources;
			acq100_allocate_resources();

			sys->resource[0] = &res[0];
			sys->resource[1] = &res[1];
			sys->resource[2] = NULL;
			sys->io_offset   = 0x6e000000;
		}
		return 1;
	}
}




static int acq100_setup(int nr, struct pci_sys_data *sys)
{
	switch (nr) {
	case 0:
		sys->map_irq = acq100_map_irq;
		return acq100_iop321_setup(nr, sys);
	default:
		return 0;
	}
}

static int acq132_setup(int nr, struct pci_sys_data *sys)
{
	switch (nr) {
	case 0:
		sys->map_irq = acq132_map_irq;
		return acq132_iop321_setup(nr, sys);
	default:
		return 0;
	}
}


extern struct pci_bus *iop321_scan_bus(int nr, struct pci_sys_data *sys);
extern void iop321_init(void);


static struct hw_pci acq100_pci __initdata = {
	.swizzle	= pci_std_swizzle,
	.nr_controllers = 1,
	.setup		= acq100_setup,
	.scan		= iop321_scan_bus,
	.preinit	= iop321_init,
};


extern struct pci_bus *acq132_scan_bus(int nr, struct pci_sys_data *sys);

static struct hw_pci acq132_pci __initdata = {
	.swizzle	= pci_std_swizzle,
	.nr_controllers = 1,
	.setup		= acq132_setup,
	.scan		= acq132_scan_bus,
	.preinit	= iop321_init
};


static void acq100_mmr_setup(void)
{
	printk( "PCI:acq100_mmr_setup()\n");
	/* Translate outbound not used */
	*IOP321_OMWTVR0 = 0;
	*IOP321_OUMWTVR0 = 0;

	*IOP321_ATUCR = 2;        /* Enable ATU Outbound, Translate */

	/* IALRO - slave the MU region from PCI */
        /* use default physaddr - out of harm's way, this is never mapped */
	*IOP321_IALR0  =0xfffff000;
	*IOP321_IATVR0 =0xfff00000;
}


extern int mmr_setup(void);  /* acq200-pci.c */

extern int G_iop321_pci_debug;




int acq100_is_system_slot(void)
{
	return acq100_get_pci_env() == ACQ100_PCIENV_SSM;
}

int acq100_is_system_slot_enabled(void) 
{
	if (acq100_is_system_slot()){
		return is_system_slot > 0;
	}else{
		return 0;
	}
}

static void acq100_check_cpld_capability(void)
{
	volatile u8* cpld = (volatile u8*)ACQ200_CPLD;
	u8 revid = cpld[3];

	printk("acq100 cpld rev %d\n", revid);

	if (revid >= 5){
		is_system_slot = 1;
		acq100_setCpldMaskBit(-1);	/* HACK */
		irq_ext_pci = IRQ_EXT_PCI_NEW;

		printk("rev5 CPLD, CPCI int %d selected\n", irq_ext_pci);
	}
}
int acq132_is_system_slot(void)
/** @@worktodo ... check CPLD for slot # */
{
	return is_system_slot;
}

int acq132_is_standalone(void)
{
	return is_system_slot;
}

/* R3 boards - enable PCI A,B,C,D + ROUTE XINT3 */
static unsigned acq100_cpld_pci_mask = 0x1f;	

static void __init set_system_slot(char **p)
{
	is_system_slot = memparse(*p, p);
	if (**p == ',') {
		irq_ext_pci = memparse((*p) + 1, p);
	}
	if (**p == ','){
		G_iop321_pci_debug = memparse((*p) + 1, p);
	}
	if (**p == ','){
		acq100_cpld_pci_mask = memparse((*p) + 1, p);
	}
}
__early_param("sysslot=", set_system_slot);

extern void iop32x_check_pci_bus_speed(void);

static int __init acq100_pci_init(void)
{
	if (machine_is_acq132()){
		if (acq132_is_system_slot() || acq132_is_standalone()){
			mmr_setup();
			printk("PCI:acq132 system slot device debug %d\n",
				G_iop321_pci_debug);
			pci_common_init(&acq132_pci);
			pci_enable_bridges(pci_find_bus(0, 0));
		}else{
			acq100_mmr_setup();
			acq100_allocate_resources();
			printk("PCI:acq100 peripheral slot device\n");
			iop32x_check_pci_bus_speed();
		}
	}else if (machine_is_acq100()){
		if (is_system_slot == 0){
			/* only do this for DEFAULT config */
			acq100_check_cpld_capability();
		}
		if (acq100_is_system_slot_enabled()){
			if (irq_ext_pci == IRQ_EXT_PCI_NEW){
				printk("PCI: ext int %d set cpld %x\n",
				       irq_ext_pci, acq100_cpld_pci_mask);
				*(volatile u8*)ACQ200_CPLD = 
						acq100_cpld_pci_mask;
			}
			mmr_setup();
			printk("PCI:acq100 system slot device debug %d\n",
				G_iop321_pci_debug);
			pci_common_init(&acq100_pci);
			pci_enable_bridges(pci_find_bus(0, 0));
		}else{
			acq100_mmr_setup();
			acq100_allocate_resources();
			switch(acq100_get_pci_env()){
			case ACQ100_PCIENV_PM:
				printk("PCI:acq100 peripheral slot device\n");
				iop32x_check_pci_bus_speed();
				break;
			case ACQ100_PCIENV_SAM:
				printk("PCI:acq100 standalone device\n");
				break;
			}
		}
	}
	return 0;
}

subsys_initcall(acq100_pci_init);




