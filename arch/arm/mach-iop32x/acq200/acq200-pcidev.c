#if FPGA_IS_PCI_DEVICE
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
	.suspend  = acq200_fpga_suspend,
	.resume   = acq200_fpga_resume
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

	(rc = pci_register_driver( &acq200_fpga_driver )) >= 0 &&
	(rc = mk_sysfs( &acq200_fpga_driver.driver)) >= 0;
	return rc;
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


