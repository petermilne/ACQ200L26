/* ------------------------------------------------------------------------- */
/* acq200-bridge.c driver for acq200 bridge                                  */
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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#ifdef PGMCOMOUT263
#include <asm-arm/proc-armv/cache.h>
#endif
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#include <linux/moduleparam.h>
#endif

#include <asm/arch-iop32x/iop321.h>

#include "acq200.h"
#include "acq200-bridge.h"

static int acq200_bridge_debug;




char acq200_bridge_driver_name[] = "acq200_bridge";
char acq200_bridge_driver_string[] = "D-TACQ bridge device";
char acq200_bridge_copyright[] = "Copyright (c) 2003 D-TACQ Solutions Ltd";


static struct DevGlobs {
	int major;
	struct B_WINDOW downstream_window;
	struct B_WINDOW upstream_window;
} dg;


int acq200_bridge_get_windows(
	struct B_WINDOW* upstream, struct B_WINDOW* downstream)
{
	if (upstream){
		*upstream = dg.upstream_window;
	}
	if (downstream){
		*downstream = dg.downstream_window;
	}
	return 0;
}

#define PCIDEV(dev) container_of(dev, struct pci_dev, dev)



static int acq200_bridge_open (
	struct inode *inode, struct file *file
)
{
	return 0;
}

static int acq200_bridge_release (
	struct inode *inode, struct file *file
)
{
	return 0;
}

int acq200_bridge_mmap( struct file* filp, struct vm_area_struct* vma )
// mmap the large contig area of mem at dg.bigbuf to vma
{
#ifdef PGMCOMOUT
	return remap_page_range( 
		vma, vma->vm_start, 
		pa_buf( &dg ), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
		);
#else
	return 0;
#endif
}


static void create_proc_entries(void)
{

}


static void delete_proc_entries(void)
{

}


static void run_bridge_mknod_helper(int major)
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};
	static char args[5][5] = {
		{},
		{},
	};
	char *argv[6];
	int rc;

	sprintf( args[1], "%d", major );

        argv[0] = "/sbin/acq200_bridge_helper";
	argv[1] = args[1];  /* major */
	argv[2] = 0;


	dbg( 1, "call_usermodehelper %s\n", argv[0] );

	rc = call_usermodehelper(argv [0], argv, envp, 0);

	if ( rc != 0 ) err( "call done returned %d", rc );
}



#define HINT8_EXT_REG_IDX 0xd3
#define HINT8_EXT_REG_DAT 0xd4

#define HINT8_EXT_REG_STICKY0 0x00
#define HINT8_EXT_REG_UBARTA0 0x08
#define HINT8_EXT_REG_UBARTA1 0x09
#define HINT8_EXT_REG_UBARTA2 0x0a
#define HINT8_EXT_REG_UTRANW  0x0b
#define HINT8_EXT_REG_DBARTA0 0x0c
#define HINT8_EXT_REG_DBARTA1 0x0d
#define HINT8_EXT_REG_DBARTA2 0x0e
#define HINT8_EXT_REG_DTRANW  0x0f

#define HINT8_TRANEN0   (1<<24)
#define HINT8_TRANPFTCH (1<<7)

#define HINT_WINDOW_16M 23     /* 1<<23 => 0x01000000 = 16M */
#define HINT_WINDOW_4K  11

#define XTRANW(xbarta) ((xbarta)+3)

#define NOWHERE 0x00           /* move cfg write address to here for no mess */

static int bit(unsigned len) 
{
	int ibit;

	/** NB: strange bias by +1 _is_ required */
	for (ibit = HINT_WINDOW_4K; (1<<(ibit+1)) < len; ++ibit){
		assert(1<<ibit);
	}
	return ibit;
}

static inline u32 read_hint_ext_reg(struct pci_dev *dev, int reg)
{
	u32 data;
	int rc;

	pci_write_config_byte(dev, HINT8_EXT_REG_IDX, reg);
	rc = pci_read_config_dword(dev, HINT8_EXT_REG_DAT, &data);

	if ( rc ) err("rc %d", rc);

	pci_write_config_byte(dev, NOWHERE, 0);
	return data;
	
}

static void write_hint_ext_reg(struct pci_dev *dev, int reg, u32 data)
{
	dbg(1,"reg %02x data %04x", reg, data);

	pci_write_config_byte(dev, HINT8_EXT_REG_IDX, reg);
	pci_write_config_dword(dev, HINT8_EXT_REG_DAT, data);
	pci_write_config_byte(dev, NOWHERE, 0);
}

static void write_hint_ext_reg_byte(
	struct pci_dev *dev, int reg, int offset, u8 data
	)
{
	dbg(1,"reg %02x.%d data %04x", reg, offset, data);

	pci_write_config_byte(dev, HINT8_EXT_REG_IDX, reg);
	pci_write_config_byte(dev, HINT8_EXT_REG_DAT+offset, data);
	pci_write_config_byte(dev, NOWHERE, 0);
}

static void
setup_address_translation(struct pci_dev *dev)
{
/*
 * suspect that write to whole of DTRANW is enabling the other BARS
 * avoid by writing bytes. Willit Work?
 */
	write_hint_ext_reg_byte(dev, HINT8_EXT_REG_DTRANW, 0, 
				HINT8_TRANPFTCH|HINT_WINDOW_16M );
	write_hint_ext_reg_byte(dev, HINT8_EXT_REG_DTRANW, 3, 1);

	write_hint_ext_reg_byte(dev, HINT8_EXT_REG_UTRANW, 0, HINT_WINDOW_4K);
	write_hint_ext_reg_byte(dev, HINT8_EXT_REG_UTRANW, 3, 1);

	write_hint_ext_reg(dev,HINT8_EXT_REG_UBARTA0, *IOP321_IABAR0&~0x0f);
	info("UBARTA0 = 0x%08x", *IOP321_IABAR0);
	write_hint_ext_reg(dev, NOWHERE, 0);
}


static ssize_t show_bridge_regs(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	struct pci_dev* pci_dev = to_pci_dev(dev);
	int len = 0;
#define SHOWREG( reg ) \
        len += sprintf( buf+len, "%25s: 0x%08x\n", \
                        #reg,read_hint_ext_reg(pci_dev,reg))

	SHOWREG(HINT8_EXT_REG_STICKY0);
	SHOWREG(HINT8_EXT_REG_UBARTA0);
	SHOWREG(HINT8_EXT_REG_UBARTA1);
	SHOWREG(HINT8_EXT_REG_UBARTA2);
	SHOWREG(HINT8_EXT_REG_UTRANW );
	SHOWREG(HINT8_EXT_REG_DBARTA0);
	SHOWREG(HINT8_EXT_REG_DBARTA1);
	SHOWREG(HINT8_EXT_REG_DBARTA1);
	SHOWREG(HINT8_EXT_REG_DTRANW );
	return len;
}
static DEVICE_ATTR(regs, S_IRUGO, show_bridge_regs, 0);



static ssize_t enable_address_translation(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	setup_address_translation(PCIDEV(dev));	
	return strlen(buf);
}
static ssize_t show_address_translation(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%x\n",
		       read_hint_ext_reg(PCIDEV(dev),HINT8_EXT_REG_UBARTA0));
}

static DEVICE_ATTR(enable_address_translation, S_IWUGO|S_IRUGO, 
		   show_address_translation, enable_address_translation);



static ssize_t set_window(	
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count,
	int XBARTA0,
	struct B_WINDOW *gw)
{
	unsigned window_base;
	unsigned window_len;
	int nv;
	
	
	if ((nv = sscanf(buf, "0x%x %x", &window_base, &window_len)) >= 1 ||
	    (nv = sscanf(buf, "%u %u",   &window_base, &window_len)) >= 1    ){
		write_hint_ext_reg(PCIDEV(dev), XBARTA0, window_base);

		info("set %sBARTA0 = 0x%08x", 
		     XBARTA0==HINT8_EXT_REG_DBARTA0?	"D": "U",
		     window_base);

		if (nv == 2){
			u8 flags = XBARTA0==HINT8_EXT_REG_DBARTA0?
						HINT8_TRANPFTCH: 0;

			write_hint_ext_reg_byte(
				PCIDEV(dev), XTRANW(XBARTA0), 0,
				bit(window_len)|flags);
		}
		write_hint_ext_reg(PCIDEV(dev), NOWHERE, 0);

		gw->w_base = window_base;
		gw->w_len  = window_len;
	}else {
		err( "unable to decode \"%s\"\n", buf );
	}
	return strlen(buf);
}

static ssize_t show_window(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf,
	int XBARTA0,
	struct B_WINDOW *gw)
{
        return sprintf(buf, "0x%08x 0x%08x [0x%08x 0x%08x]\n",
		       read_hint_ext_reg(PCIDEV(dev), XBARTA0),
		       read_hint_ext_reg(PCIDEV(dev), XTRANW(XBARTA0)),
		       gw->w_base, gw->w_len
			);
}

static ssize_t set_downstream_window(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	return set_window(
		dev, attr, buf, count, HINT8_EXT_REG_DBARTA0,
		&dg.downstream_window
	);
}


static ssize_t show_downstream_window(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
	       
{
	return show_window(dev, attr, buf, HINT8_EXT_REG_DBARTA0,
		&dg.downstream_window
	); 
}

static DEVICE_ATTR(downstream_window, S_IRUGO|S_IWUGO,
		   show_downstream_window, set_downstream_window);



static ssize_t show_upstream_window(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
	       
{
	return show_window(dev, attr, buf, HINT8_EXT_REG_UBARTA0,
		&dg.upstream_window
	); 
}

static ssize_t set_upstream_window(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	return set_window(dev, attr, buf, count, HINT8_EXT_REG_UBARTA0,
		&dg.upstream_window
		);
}

static DEVICE_ATTR(upstream_window, S_IRUGO|S_IWUGO,
		   show_upstream_window, set_upstream_window);


static void mk_sysfs(struct device *device)
{
	DEVICE_CREATE_FILE(device, &dev_attr_downstream_window);
	DEVICE_CREATE_FILE(device, &dev_attr_upstream_window);
	DEVICE_CREATE_FILE(device, &dev_attr_regs);
	DEVICE_CREATE_FILE(device, &dev_attr_enable_address_translation);
}

static void rm_sysfs(struct device *device)
{
	device_remove_file(device, &dev_attr_downstream_window);
	device_remove_file(device, &dev_attr_upstream_window);
	device_remove_file(device, &dev_attr_regs);
	device_remove_file(device, &dev_attr_enable_address_translation);
}


static int __devinit
acq200_bridge_driver_init( struct pci_dev *dev )
{
	static struct file_operations bridge_fops = {
		.open = acq200_bridge_open,
		.release = acq200_bridge_release,
		.mmap = acq200_bridge_mmap
	};
	int rc = register_chrdev( 
		dg.major = 0, "acq200-bridge", &bridge_fops );

	if (rc < 0){
		err( "can't get major %d", dg.major );
		return rc;
	}else{
		dg.major = rc;
		info( "device major set %d\n", dg.major );
	}

	setup_address_translation(dev);

	create_proc_entries();	
	run_bridge_mknod_helper( dg.major );
	return 0;
} 

static int __devinit
acq200_bridge_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	int rc;
//	acq200_fixup_irqs(dev);

	if (dg.major){
		err("we found our bridge already - ignore this one!");
		return -ENODEV;
	}
	info( "enabling device irq was %d", dev->irq);
	rc = pci_enable_device(dev);	/* XXX check return */
	dbg( 1,"now dev->irq = %d  rc %d", dev->irq, rc);

	rc = pci_request_regions(dev, acq200_bridge_driver_name);
	if ( rc ) return rc;

	mk_sysfs(&dev->dev);

	return acq200_bridge_driver_init(dev);
}

static void acq200_bridge_remove (struct pci_dev *dev)
// remove DEVICE resources on device remove
{
	rm_sysfs(&dev->dev);

	pci_disable_device(dev);

	pci_release_regions(dev);
	pci_set_drvdata(dev, NULL);

	unregister_chrdev( dg.major, "acq200-bridge" );

	delete_proc_entries();
}



/*
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, String Index }
 */
static struct pci_device_id acq200_bridge_pci_tbl[] __devinitdata = {
	{PCI_VENDOR_ID_HINT, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{ }
};
static struct pci_driver acq200_bridge_driver = {
	.name     = acq200_bridge_driver_name,
	.id_table = acq200_bridge_pci_tbl,
	.probe    = acq200_bridge_probe,
	.remove   = __devexit_p(acq200_bridge_remove),

	/* Power Managment Hooks */
#ifdef CONFIG_PM
#ifdef PGMCOMOUT_WORKTODO
	.suspend  = acq200_bridge_suspend,
	.resume   = acq200_bridge_resume
#endif
#endif
};


static int __init acq200_bridge_init( void )
{
	acq200_debug = acq200_bridge_debug;

	return pci_register_driver(&acq200_bridge_driver);
}


static void __exit
acq200_bridge_exit_module(void)
// Remove DRIVER resources on module exit
{

	pci_unregister_driver(&acq200_bridge_driver);
}

EXPORT_SYMBOL_GPL(acq200_bridge_get_windows);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 BRIDGE");
MODULE_VERSION("1.1");


module_param(acq200_bridge_debug, int, 0664);

module_init(acq200_bridge_init);
module_exit(acq200_bridge_exit_module);
