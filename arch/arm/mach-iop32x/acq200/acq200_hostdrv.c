/* ------------------------------------------------------------------------- */
/* acq200_hostdrv.c skeleton pci driver for slave acq2xx                     */
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


/** @file acq200_hostdrv.c
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

#include <linux/proc_fs.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif


#include "acq200_debug.h"

#include "acq200_hostdrv.h"


int slots[MAXDEV] = { 0, };
module_param_array(slots,int,NULL,0444);
MODULE_PARM_DESC(slots, "maps logical device# to CPCI slot#");

/** Globals .. keep to a minimum! */
char acq200_driver_name[] = "acq200";
char acq200__driver_string[] = "D-TACQ intelligent data acquisition device";
char acq200__driver_version[] = "B1000";
char acq200__copyright[] = "Copyright (c) 2011 D-TACQ Solutions Ltd";
int idx;

struct Acq200Device* acq200_devices[MAXDEV];

struct class* acq200_device_class;

#define PCI_BA_CSR  0 /* surely there's an official def ?? WORKTODO */
#define CSR_SIZE    0x80
#define CSR_SIZE_MAX 0x1000

struct Acq200Device* acq200_lookupDevice(struct device *dev)
{
	int id;

	for (id = 0; id < idx; ++id){
		struct Acq200Device* acq200_device = acq200_devices[id];
		if (dev == &acq200_devices[id]->pci_dev->dev){
			return acq200_device;
		}
	}
	
	return 0;
}
int acq200_get_cpci_slot(struct pci_dev * pci_dev)
{
	u32 devfn =  pci_dev->devfn;
	int slot = -1;


	int pslot = PCI_SLOT(devfn);
	
	if (devfn >= 72 && devfn <= 120){
		slot = 9 - (devfn/8 -8);			
	}

	info("devfn %d slot %d  pslot:%d", devfn, slot, pslot);

	return slot;	
}

static int acq200_makeIoMapping(
	struct Acq200Device* device, int bar, const char* ident, int len)
// return 0 on success
{
#define BA_MASK PCI_BASE_ADDRESS_MEM_MASK
	struct IoMapping* pim = &device->csr;
	int rc = 0;

	snprintf(pim->name, sizeof(pim->name), "%s.%s", 
		 device->ldev.drv_name, ident);

	pim->pa = pci_resource_start(device->pci_dev, bar)&BA_MASK;
	pim->len = len;
    
	dbg(1, "request_mem_region 0x%08lx %d %s\n", 
		pim->pa, pim->len, pim->name );

	if ( request_mem_region( pim->pa, pim->len, pim->name ) != 0 ){
		pim->va = ioremap_nocache( pim->pa, pim->len );
		dbg(1, "ioremap_nocache pa: 0x%08lx len: 0x%02x va: 0x%p", 
		    pim->pa, pim->len, pim->va);
	}else{
		dbg( 1, "request_mem_region( 0x%08lx, %d, %s ) failed\n",
			    pim->pa, pim->len, pim->name );
		rc = -ENODEV;
	}

	return rc;
}

#define RAM_BAR	4

int acq200_map_pci_ram(struct Acq200Device* device)
{
	struct IoMapping* pim = &device->ram;
	struct pci_dev *pdev = device->pci_dev;
	int rc;

	snprintf(pim->name, sizeof(pim->name), "%s.%s", 
		 device->ldev.drv_name, "ram");

	if ((rc = pci_request_region(pdev, RAM_BAR, pim->name)) == 0){
		pim->pa = pci_resource_start(pdev, RAM_BAR);
		pim->len = pci_resource_len(pdev, RAM_BAR);
		pim->va = ioremap_nocache( pim->pa, pim->len );
		return 0;
	}

	return rc;
}



int acq200_map_pci_memory(struct Acq200Device* device)
/**
 *  map remote regs area on device load
 */
{
	int rc = 0;                /* assume success until fail */
    
	if ( (rc = acq200_makeIoMapping(
			device, PCI_BA_CSR, "csr", CSR_SIZE)) != 0 ){
		return rc;
	}
	return acq200_map_pci_ram(device);    
}

static void clearIoMapping(struct IoMapping* iomap)
{
    if (iomap->va){
        iounmap(iomap->va);
    }
    if (strlen( iomap->name ) ){
        release_mem_region(iomap->pa, iomap->len);
    }
    memset( iomap, 0, sizeof(struct IoMapping));     
}

void acq200_unmap_pci_memory( struct Acq200Device* device )
/**
 * unmap remote regs are on device unload
 */
{
    clearIoMapping(&device->csr);
}


static int acq200_device_init(struct Acq200Device* device)
{
	return 0;
}


static struct Acq200Device *acq200_device_create(struct pci_dev *pci_dev)
{
	struct Acq200Device *device =
		kmalloc(sizeof(struct Acq200Device), GFP_KERNEL);

	assert(device);
	memset(device, 0, sizeof(struct Acq200Device));
	assert(idx < MAXDEV);

	acq200_devices[idx] = device;
	device->pci_dev = pci_dev;
	device->idx = idx++;
	sprintf(device->ldev.drv_name, "acq200.%d", device->idx );


	acq200_map_pci_memory(device);

	slots[device->idx] = acq200_get_cpci_slot(device->pci_dev);

	return device;
}
static void acq200_device_free(struct Acq200Device *device)
{
	acq200_unmap_pci_memory(device);
	kfree(device);
}


static int __devinit
acq200_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct Acq200Device* device = acq200_device_create(dev);

	return acq200_device_init(device);
}


static void acq200_remove (struct pci_dev *dev)
{
	struct Acq200Device *device = acq200_lookupDevice(&dev->dev);
	if (device != 0){
		unregister_chrdev(device->ldev.major, device->ldev.drv_name);
		acq200_device_free(device);
	}
}


/*
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, String Index }
 */
static struct pci_device_id acq200_pci_tbl[] __devinitdata = {
	{PCI_VENDOR_ID_HINT, 0x0029, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{PCI_VENDOR_ID_INTEL, 0x0318, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{PCI_VENDOR_ID_INTEL, 0x0319, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{ }
};
static struct pci_driver acq200_driver = {
	.name     = acq200_driver_name,
	.id_table = acq200_pci_tbl,
	.probe    = acq200_probe,
	.remove   = __devexit_p(acq200_remove),
#ifdef PGMCOMOUT
	/* Power Managment Hooks */
#ifdef CONFIG_PM
	.suspend  = acq200_suspend,
	.resume   = acq200_resume
#endif
#endif
};




static int __init acq200_init( void )
{
	int rc;
	acq200_device_class = class_create(THIS_MODULE, "acqX00");

	rc = pci_register_driver(&acq200_driver);
	dbg(1, "pci_register_driver() returned %d", rc );
	return 0;
}

int acq200_init_module(void)
{
	info("%s\n%s\n%s\n%s", 
	     acq200_driver_name, acq200__driver_string,
	     acq200__driver_version, acq200__copyright);
	acq200_init();
	return 0;
}

void acq200_exit_module(void)
{
	if (!idx) return;
	pci_unregister_driver(&acq200_driver);
}


module_init(acq200_init_module);
module_exit(acq200_exit_module);

EXPORT_SYMBOL_GPL(acq200_devices);

MODULE_DEVICE_TABLE(pci, acq200_pci_tbl);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 BRIDGE");


