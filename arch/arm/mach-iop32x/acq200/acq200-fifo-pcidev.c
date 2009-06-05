/* ------------------------------------------------------------------------- */
/* acq200-fifo-pcidev.c pci device wrapper for fifo driver
 * WARNING: DO NOT COMPILE DIRECTLY
\* ------------------------------------------------------------------------- */
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


#if FPGA_IS_PCI_DEVICE

static inline void disable_parity_check(struct pci_dev *dev)
/*
 * disable parity check on XILINX WORKTODO
 */
{
 	u16 pci_command;

	pci_read_config_word(dev, PCI_COMMAND, &pci_command);
	pci_command &= ~PCI_COMMAND_PARITY;
	pci_write_config_word(dev, PCI_COMMAND, pci_command);
	
}



static int init_pci_resource(
	struct pci_mapping* res,
	struct pci_dev *dev,
	int bar )
/* returns non zero on error */
{
	res->pa = pci_resource_start(dev, bar);
	res->len = pci_resource_len(dev, bar);
	res->va = ioremap( res->pa, res->len );

	return res->va == 0;
}



static int init_pci(struct pci_dev *dev)
{
	int rc;

	acq200_fixup_irqs(dev);

	dbg( 1,"enabling device irq was %d", dev->irq);
	rc = pci_enable_device(dev);	/* XXX check return */
	dbg( 1,"now dev->irq = %d  rc %d", dev->irq, rc);

	if ( rc ) return rc;


	if (dev->irq != IRQ_ACQ200_FP){
		err("dev->irq wanted:%d got:%d", IRQ_ACQ200_FP, dev->irq);
		return -1;
	}

	disable_parity_check(dev);

	rc = pci_request_regions(dev, acq200_fpga_driver_name);
	if ( rc ) return rc;



	if ( init_pci_resource(&DG->fpga.regs, dev, ACQ200_FPGA_REG_BAR) ){
		return -ENODEV;
	}

	if ( init_pci_resource(&DG->fpga.fifo, dev, ACQ200_FPGA_FIFO_BAR) ){
		return -ENODEV;
	}
#ifdef ACQ200_FPGA_EXTRA_BAR
	init_pci_resource(&DG->fpga.extra, dev, ACQ200_FPGA_EXTRA_BAR);
#endif
	return 0;
}

static int __devinit
acq200_fpga_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	int rc;
	
	if ((rc = init_pci(dev)) != 0){
		return rc;
	}else if ((rc = init_arbiter()) != 0){
		return rc;
	}else{
		return acqX00_fpga_probe(&dev->dev, dev->irq);
	}
}

static void acq200_fpga_remove (struct pci_dev *dev)
// remove DEVICE resources on device remove
{
	DTACQ_MACH_DRIVER_REMOVE(&dev->dev);
	pci_release_regions(dev);
	/* pci_disable_device(dev); WARNING: do not call, dev alrdy history */
	pci_set_drvdata(dev, NULL);

	acqX00_fpga_remove(&dev->dev, dev->irq);
}

/*
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, String Index }
 */
static struct pci_device_id acq200_fpga_pci_tbl[] __devinitdata = {
	{PCI_VENDOR_ID_XILINX, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{ }
};
static struct pci_driver acq200_fpga_driver = {
	.name     = acq200_fpga_driver_name,
	.id_table = acq200_fpga_pci_tbl,
	.probe    = acq200_fpga_probe,
	.remove   = __devexit_p(acq200_fpga_remove),

	/* Power Managment Hooks */
#ifdef CONFIG_PM
#ifdef PGMCOMOUT_WORKTODO
	.suspend  = acq200_fpga_suspend,
	.resume   = acq200_fpga_resume
#endif
#endif
};


static int __init acq200_fifo_init( void )
{
	int rc;


	acq200_debug = acq200_fifo_debug;

	dbg( 1, " init modules %s %s\n", __DATE__, __TIME__ );

	dbg(1,"init_dg(), init_phases()");
	CAPDEF = DTACQ_MACH_CREATE_CAPDEF();
	init_dg();
	init_phases();

	info( "acq200_debug set %d\n", acq200_debug );

	if ((rc = pci_register_driver( &acq200_fpga_driver )) < 0){
		return rc;
	}
	if ((rc = mk_sysfs( &acq200_fpga_driver.driver)) < 0){
		return rc;
	}
	return 0;
}




static void __exit
acq200_fifo_exit_module(void)
// Remove DRIVER resources on module exit
{
//	unregister_reboot_notifier(&e1000_notifier_reboot);

	dbg( 1, "rm_sysfs" );
	rm_sysfs(&acq200_fpga_driver.driver);
	dbg( 1, "pci_unregister_driver" );
	pci_unregister_driver( &acq200_fpga_driver );
	delete_dg();
	DTACQ_MACH_DESTROY_CAPDEF(CAPDEF);
}





module_init(acq200_fifo_init);
module_exit(acq200_fifo_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 FIFO");

#else
#error FPGA_IS_PCI_DEVICE NOT DEFINED. Do not compile this file directly
#endif   /* #if FPGA_IS_PCI_DEVICE */


