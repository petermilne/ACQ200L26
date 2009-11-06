/* ------------------------------------------------------------------------- */
/* acq200-core.c driver for acq200 core logic fpga load                      */
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
/* 

The sequence is set the PROGRAM low = clear bit 7 at address 18h
Then set PROGRAM high again
Wait until you see INIT high = bit 2 at address 18h
Next start writing to the FPGA data port at address 19h
Check INIT every once in a while if low then a CRC error 
Check for DONE at end - bit 0 at address 18h

*/

#include <asm/uaccess.h>

#ifndef CONFIG_ARCH_ACQ200
#error ERROR: this stuffvalid for acq200 only, sorry
#endif

#include <linux/init.h>


#include <linux/kernel.h>   /* printk() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/mm.h>       /* VMA */
#include <linux/types.h>    /* size_t */
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>    /* O_ACCMODE */

#include <linux/pci.h>

#include <asm/system.h>     /* cli(), *_flags */

#include <asm/hardware.h>
#include <asm/mach/pci.h>

#include <linux/kdev_t.h>   /* MINOR */

#include <asm-arm/mach-types.h>

#include <asm/arch/iop321.h>

#define MODULE
#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include "acq200.h"
#include "acq200_minors.h"

#include "acq196.h"
#include "acq200-sys.h"

struct proc_dir_entry* proc_acq200;
EXPORT_SYMBOL_GPL(proc_acq200);

int acq200_debug = 0;
EXPORT_SYMBOL_GPL(acq200_debug);



#define FPGA_CSR_PROGRAM 0x80	/* "PROGRAM" => clear FPGA memory	*/
#define FPGA_CSR_CS	 0x40	/* ChipSelect needed for S3A		*/

#define FPGA_CSR_INIT    0x04
#define FPGA_CSR_BUSY    0x02
#define FPGA_CSR_DONE    0x01

static int acq200_core_major = 0; /* dynamic alloc */
static int acq200_nbits;


#define FPGA_DEV   2
#define FPGA_DEVFN (FPGA_DEV*8)

#define FPGA_CSR	(volatile u8*)(ACQ200_CPLD+0x18)
#define FPGA_DATA	(volatile u8*)(ACQ200_CPLD+0x19)
#define DIO(dx)		(volatile u8*)(ACQ200_CPLD+0x10+(dx))

static void 
pdev_fixup_irq(struct pci_dev *dev,
	       u8 (*swizzle)(struct pci_dev *, u8 *),
	       int (*map_irq)(struct pci_dev *, u8, u8))
/* blatant crib from setup_irq.c (too bad it was declared static) */
{
	u8 pin, slot;
	int irq;

	pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &pin);

	/* Cope with 0 and illegal. */
	if (pin <= 0 || pin > 4)
		pin = 1;

	slot = (*swizzle)(dev, &pin);
	irq = (*map_irq)(dev, slot, pin);

	dev->irq = irq != -1? irq: 0;

	dbg(1, "PCI fixup irq: (%s) got %d\n", 
	    kobject_name(&dev->dev.kobj), dev->irq);

	/* Always tell the device, so the driver knows what is
	   the real IRQ to use; the device does not use it. */
	pcibios_update_irq(dev, dev->irq);
}


void acq200_fixup_irqs(struct pci_dev* dev)
{
	struct pci_sys_data* sys = dev->sysdata;	

	pdev_fixup_irq( dev, sys->swizzle, sys->map_irq );
}
EXPORT_SYMBOL_GPL(acq200_fixup_irqs);


static int acq200_fpga_hotplug_pci(void)
/*
 * If this is the first time, hotplug the device into the pci subsys
 * If there was already a device there, remove it first
 */
{
	struct pci_dev *old_dev = pci_find_slot( 0, FPGA_DEVFN );
	struct pci_bus *bus = pci_find_bus(0, 0);

	if ( old_dev ){
		pci_remove_bus_device(old_dev);
	}

	if ( pci_scan_slot( bus, FPGA_DEVFN ) ){
		pci_bus_assign_resources(bus);
		pci_bus_add_devices(bus);	
		return 0;
	}else{
		err( "ERROR device not found\n" );
		return -ENODEV;
	}
}


static int acq200_fpga_load_open (struct inode *inode, struct file *file)
{
	volatile u8* fpga_csr = FPGA_CSR;
	int timeout = 100000;

	acq200_nbits = 0;

	
	*fpga_csr = 0;
	*fpga_csr = FPGA_CSR_PROGRAM;

	while( (*fpga_csr&FPGA_CSR_INIT) == 0 ){
		if ( --timeout == 0 ){
			E( "looking for init %x\n", *fpga_csr );
			return -ETIMEDOUT;
		}
	}
	*fpga_csr = FPGA_CSR_PROGRAM|FPGA_CSR_CS;
	return 0;
}

#define CHECK_INIT_HIGH_ONRELEASE

#define NCLOCKS 12

static void add_config_clocks(void)
{
	volatile u8* fpga_data = FPGA_DATA;
	int nclocks = NCLOCKS;

	while(nclocks--){
		*fpga_data = 0x00;
	}
}
static int acq200_fpga_load_release (struct inode *inode, struct file *file)
/*
 * Test that the load has completed OK, then hotplug into pci subsys
 */
{
	volatile u8* fpga_csr = FPGA_CSR;
	u8 status;
	int timeout = 0x10000;
	int rc = 0;

	*fpga_csr = FPGA_CSR_PROGRAM;		/* clears FPGA_CSR_CS */       
	add_config_clocks();	


#ifdef CHECK_INIT_HIGH_ONRELEASE
	dbg(1, "CHECK_INIT_HIGH_ONRELEASE");

	if ( ((status = *fpga_csr)&FPGA_CSR_INIT) == 0 ){
		E( "CRC error INIT LOW %x at %d\n", status,
		   acq200_nbits );
		return -1;
	}			
#endif
	while (((status = *fpga_csr)&FPGA_CSR_DONE) == 0 ){
		if ( (--timeout&0x1fff) == 0 ){
			E( "polling for DONE %02x got %02x\n",
			   FPGA_CSR_DONE, status );
		}
		if ( timeout == 0 ){
			E( "finished but xilinx not ready %x bits %d\n", 
			   status, acq200_nbits );
			rc = -ETIMEDOUT;
			break;
		}
	}

	if (rc == 0 && machine_is_acq200()){
		dbg(1, "call acq200_fpga_hotplug_pci");
	/* run hotplug whether success or not (may be a reset) */
		rc = acq200_fpga_hotplug_pci();
		/** frequently has unknown id 7f with pci host ...
		 *  try again
                 */

		if (rc == -ENODEV){
			rc = acq200_fpga_hotplug_pci();
			if (rc == -ENODEV){
				err("HOTPLUG fail on two attempts");
			}else{
				info("HOTPLUG PASS on second attempt");
			}
		}
	}
	return rc;
}

/* #define CHECK_INIT_HIGH */
/* #define LAZY_CHECKING */

static ssize_t acq200_fpga_load_write ( 
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
{
#if defined(CHECK_INIT_HIGH) || defined(LAZY_CHECKING)
	u8 status;
	volatile u8* fpga_csr = FPGA_CSR;
#endif
	volatile u8* fpga_data = FPGA_DATA;

	int ibuf = 0;
	int timeout = 0x1000;


	for ( ; ibuf != len; ++ibuf, timeout = 0x1000 ){
#ifdef LAZY_CHECKING
		if ( (ibuf&0xfff) == 0 ){
			u8 status = *fpga_csr;
			if ( (status&FPGA_CSR_INIT) == 0 ){
				E( "CRC error %x\n", status );
				return -1;
			}
		}
#endif
#ifdef WAIT_FOR_NOT_BUSY
#error WAIT_FOF_NOT_BUSY
		while ( ((status = *fpga_csr)&FPGA_CSR_BUSY) != 0 ){
			if ( (--timeout&0xff) == 0 ){
				E( "polling for !BUSY %02x got %02x\n",
				   FPGA_CSR_BUSY, status );
			}
			if ( timeout == 0 ){
				E( "too busy, dropping out %02x %d\n",
				   status, acq200_nbits );
				return -ETIMEDOUT;
			}
		}
#endif
#ifdef CHECK_INIT_HIGH
#error CHECK_INIT_HIGH
		if ( ((status = *fpga_csr)&FPGA_CSR_INIT) == 0 ){
			E( "CRC error INIT LOW %x at %d\n", status,
			   acq200_nbits );
			return -1;
		}			
#endif
		*fpga_data = buf[ibuf];
		acq200_nbits += 8;
	}
	
	return len;
}


extern int acq100_is_system_slot_enabled(void);

static int decode_minor(
	struct file *file, unsigned* p_paddr, unsigned *p_len )
{
	unsigned paddr, len;

	switch( MINOR(file->f_dentry->d_inode->i_rdev) ){
	case ACQ200_CORE_FLASH:
		paddr = ACQ200_FLASH_P;	
		len = ACQ200_FLASH_LEN;
		break;
	case ACQ200_CORE_CPLD:
		paddr = ACQ200_CPLD_P;	
		len = ACQ200_CPLD_LEN;
		break;
	case ACQ200_CORE_FPGA:
		paddr = ACQ200_FPGA_P;
		len = ACQ200_FPGA_LEN;
		break;
	case ACQ200_CORE_UART:
		paddr = ACQ200_UART_P; 
		len = ACQ200_UART_LEN;
		break;
	case ACQ200_CORE_LIO:
		paddr = ACQ200_LOCALIO_P; 
		len = ACQ200_LOCALIO_LEN;
		break;
	case ACQ200_CORE_EXIO:
		paddr = ACQ200_EXTERNIO_P; 
		len = ACQ200_EXTERNIO_LEN;
		break;
	case ACQ200_CORE_PCIM:
		if (machine_is_acq100() && !acq100_is_system_slot_enabled()){
			paddr = ACQ100_PCIMEM_P;
			len = 0x04000000;
		}else{
			paddr = ACQ200_PCIMEM;
			len = ACQ200_PCIMEM_SIZE;
		}
		break;
	case ACQ200_CORE_PCIIO:
		paddr = IOP321_PCI_LOWER_IO_PA;
		len = 0x10000;
		break;
	case ACQ200_CORE_PMMR:
		paddr = IOP321_PHYS_MEM_BASE; 
		len = 0x2000;
		break;
	case ACQ200_CORE_IRAM:
		paddr = 0xffff0000;
		len = 0x10000;
		break;
	case ACQ200_CORE_MUMEM: {
		struct resource resource;

		acq200_get_mumem_resource(&resource);
		paddr = virt_to_phys((void*)resource.start);
		len = resource.end - resource.start;
		break;
        }case ACQ200_CORE_BIGBUF: {
		struct resource resource;

		acq200_get_bigbuf_resource(&resource);
		paddr = virt_to_phys((void*)resource.start);
		len = resource.end - resource.start;
		break;
	}default:
		return -ENODEV;
	}

	*p_paddr = paddr;
	*p_len = len;
	return 0;
}

static int acq200_debug_do_mmap(
	struct file *file, struct vm_area_struct *vma,
	unsigned paddr, unsigned len )
{
	if ( vma->vm_end - vma->vm_start < len ){
		len = vma->vm_end - vma->vm_start;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_RESERVED | VM_IO;

	if (io_remap_pfn_range(vma, vma->vm_start, __phys_to_pfn(paddr),
       						len, vma->vm_page_prot)){
		err( "remap_page_range() failed" );
		return -EAGAIN;
	}else{
		return 0;
	}
}

static int acq200_debug_map_mmap(
	struct file *file, struct vm_area_struct *vma )
{
	unsigned paddr, len = 0;
	int rc;

	if ((rc = decode_minor(file, &paddr, &len))){
		return rc;
	}else{
		return acq200_debug_do_mmap(file, vma, paddr, len);
	}
}


static int acq200_debug_map_read( 
	struct file *file, char *buf, size_t len, loff_t *offset
	)
{
	unsigned paddr, source_len;
	int rc;

	if ((rc = decode_minor(file, &paddr, &source_len))){
		return rc;
	}else{	
		char *src;
		len = min( len, (size_t)(source_len - (int)*offset) );


		if (!file->private_data){
			file->private_data = ioremap(paddr, source_len);
		}
		src = (char*)file->private_data;
		if (!src){
			return -ENOMEM;
		}
		
		dbg( 2, "len %d *offset %d", len, (int)*offset );

		if (copy_to_user( buf,  &src[*offset], len )){
			return -EFAULT;
		}
		*offset += len;
		return len;
	}
}


int acq200_core_release (struct inode *inode, struct file *file)
{
	return 0;
}


int acq200_debug_release (struct inode *inode, struct file *file)
{
	if (file->private_data){
		iounmap(file->private_data);
	}
	return acq200_core_release(inode, file);
}

static int acq200_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = 0;
	return 0;
}

static int acq200_core_open (struct inode *inode, struct file *file)
{
	static struct file_operations fpga_load_fops = {
		open: acq200_fpga_load_open,
		release: acq200_fpga_load_release,
		write: acq200_fpga_load_write,

	};

	static struct file_operations debug_map_fops = {
		open: acq200_debug_open,
		mmap: acq200_debug_map_mmap,
		read: acq200_debug_map_read,
		release: acq200_debug_release
	};

        int iminor = MINOR(file->f_dentry->d_inode->i_rdev);

	if (IS_FPGA_LOAD_DEVICE(iminor)){
		file->f_op = &fpga_load_fops;
		return file->f_op->open(inode, file);
	}else if (IS_DEBUG_MAP_DEVICE(iminor)){
		file->f_op = &debug_map_fops;
		return file->f_op->open(inode, file);
	}
	return 0;
}

extern int G_iop321_pci_debug;

static int dma_proc_abort_debug(
	char *buf, char **start, off_t offset, int len, int* eof, void* data )
{
	*eof = 1;

	G_iop321_pci_debug = (int)data;

	return sprintf( buf, "G_iop321_pci_debug %d\n", G_iop321_pci_debug );
}

static u8 dio_route_shadow[6];

static ssize_t show_dio_route(
	int dx,
	struct device * dev, char * buf)
{
        return sprintf(buf,"%02x\n", dio_route_shadow[dx]);
}

static ssize_t store_dio_route(
	int dx,
	struct device * dev, const char * buf, size_t count)
{
	unsigned mask;
	unsigned m1, m2;
	volatile u8* dio = DIO(dx);

/*
 * input 2 digit hex OR 2 dec nums (easier with shell)
 */
	if (sscanf(buf, "%d %d", &m1, &m2) == 2){
		*dio = dio_route_shadow[dx] = (m1<<4) + m2;
	}else if (sscanf(buf, "%x", &mask) == 1 ){
		*dio = dio_route_shadow[dx] = mask;
	}
	
	return strlen(buf);
}


static ssize_t set_debug(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &acq200_debug);
	return strlen(buf);
}
static ssize_t show_debug(
	struct device *dev, 
	struct device_attribute *attr, char * buf)
{
        return sprintf(buf, "%d\n", acq200_debug);
}

static DEVICE_ATTR(debug, S_IWUGO|S_IRUGO, show_debug, set_debug);

#define DECL_ACCESSOR_DIO(dx)					\
static ssize_t show_dio_route##dx(				\
	struct device * dev,					\
	struct device_attribute *attr,				\
	 char * buf)						\
{								\
	return show_dio_route(dx, dev, buf);			\
}								\
static ssize_t store_dio_route##dx(				\
	struct device * dev,					\
	struct device_attribute *attr,				\
	const char * buf, size_t count)				\
{								\
	return store_dio_route(dx, dev, buf, count);		\
}								\
static DEVICE_ATTR(dio_route_d##dx, S_IRUGO|S_IWUGO, 		\
		   show_dio_route##dx, store_dio_route##dx);



DECL_ACCESSOR_DIO(0);
DECL_ACCESSOR_DIO(1);
DECL_ACCESSOR_DIO(2);
DECL_ACCESSOR_DIO(3);
DECL_ACCESSOR_DIO(4);
DECL_ACCESSOR_DIO(5);


extern void acq200_set_user_led(int led4, int on);

#define DECL_ACCESSOR_LED(led4)					\
static ssize_t store_led##led4(				        \
	struct device * dev, struct device_attribute *attr,	\
	const char * buf, size_t count)				\
{								\
        int value;                                              \
        if (sscanf(buf, "%d", &value) == 1){                    \
                acq200_set_user_led(led4==4, value);            \
        }                                                       \
	return strlen(buf);	                        	\
}								\
static DEVICE_ATTR(led##led4, S_IWUGO, 0, store_led##led4)

DECL_ACCESSOR_LED(3);
DECL_ACCESSOR_LED(4);


static ssize_t show_xx_pa(char * buf, struct resource *resource)
{
	u32 pa = (resource->start & ~0xf0000000) | ACQ200_RAMBASE;
	return sprintf(buf,"0x%08x\n", pa);
}

static ssize_t show_bb_pa(
	struct device * device, struct device_attribute *attr, char * buf)
{
	struct resource resource;

	acq200_get_bigbuf_resource(&resource);
        return show_xx_pa(buf, &resource);
}

static DEVICE_ATTR(bb_pa, S_IRUGO, show_bb_pa, 0);

static ssize_t show_tbtmp_pa(
	struct device * device, struct device_attribute *attr, char * buf)
{
	struct resource resource;

	acq200_get_tblock_resource(&resource);
	return show_xx_pa(buf, &resource);
}

static DEVICE_ATTR(tbtmp_pa, S_IRUGO, show_tbtmp_pa, 0);

static ssize_t show_mumem_pa(
	struct device * device, struct device_attribute *attr, char * buf)
{
	struct resource resource;

	acq200_get_mumem_resource(&resource);
	return show_xx_pa(buf, &resource);
}

static DEVICE_ATTR(mumem_pa, S_IRUGO, show_mumem_pa, 0);



static ssize_t show_bb_len(
	struct device * device, struct device_attribute *attr,char * buf)
{
	struct resource resource;
	int len;

	acq200_get_bigbuf_resource(&resource);
	len = resource.end - resource.start;

        return sprintf(buf,"%d 0x%08x %dM\n", len, len, len/0x100000);
}

static DEVICE_ATTR(bb_len, S_IRUGO, show_bb_len, 0);


static  ssize_t show_cpld_rev(
	struct device * device, struct device_attribute *attr, char * buf)
{
        return sprintf(buf,"%d\n", acq100_get_cpld_rev());
}

static DEVICE_ATTR(cpld_rev, S_IRUGO, show_cpld_rev, 0);


static	ssize_t set_hpi_mask(
	struct device *device,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned mask;

	if (sscanf(buf, "0x%02x", &mask) == 1){
		acq200_set_cpld_mask_byte(mask);	
	}
	return count;
}

static ssize_t show_hpi_mask(
	struct device * device, 
	struct device_attribute *attr, 
	char * buf)
{
	return sprintf(buf, "0x%02x\n", acq200_get_cpld_mask_byte());
}

static DEVICE_ATTR(hpi_mask, S_IRUSR|S_IWUSR, show_hpi_mask, set_hpi_mask);

/* 2.6.18 timer tick monitor @@togo */
extern int iop3xx_report_ticks(char* buf, int maxbuf);

static ssize_t show_ticks(
	struct device * device, 
	struct device_attribute *attr, 
	char * buf)
{
	return iop3xx_report_ticks(buf, 4096);
}
static DEVICE_ATTR(ticks, S_IRUGO, show_ticks, 0);

extern unsigned acq200_setAuxClock(unsigned hz);
extern unsigned acq200_getAuxClock(void);

static	ssize_t set_auxClock(
	struct device *device,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned hz;

	if (sscanf(buf, "%u", &hz) == 1){
		acq200_setAuxClock(hz);
	}
	return count;
}

static ssize_t show_auxClock(
	struct device * device, 
	struct device_attribute *attr, 
	char * buf)
{
	return sprintf(buf, "%u\n", acq200_getAuxClock());
}

static DEVICE_ATTR(auxClock, S_IRUSR|S_IWUSR, show_auxClock, set_auxClock);


static  ssize_t show_pci_env(
	struct device * device, struct device_attribute *attr, char * buf)
{
	char* str = "not-supported";
	int rc = -1;

	if (machine_is_acq100() || machine_is_acq132()){
		switch(rc = acq100_get_pci_env()){
		case ACQ100_PCIENV_SSM:
			str = "SSM"; break;
		case ACQ100_PCIENV_PM:
			str = "PM"; break;
		case ACQ100_PCIENV_SAM:
			str = "SAM"; break;
		default:
			;
		}
	}

	return sprintf(buf, "%d %s", rc, str);
}

static DEVICE_ATTR(pci_env, S_IRUGO, show_pci_env, 0);




static ssize_t show_sysslot(
	struct device * device, struct device_attribute *attr, char * buf)
{
	return sprintf(buf, "%d\n", acq200_is_sysslot());
}

static DEVICE_ATTR(sysslot, S_IRUGO, show_sysslot, 0);

static ssize_t show_pmmr(
	struct device * device, char * buf, void *reg)
{
	return sprintf(buf, "0x%08x\n", *(volatile u32*)reg);
}



#define DEF_DEV_ATTR_PMMR(reg)						\
static ssize_t show_##reg(						\
	struct device * device, struct device_attribute *attr, char * buf){ \
	return show_pmmr(device, buf, (void*)IOP321_##reg);		\
}									\
static DEVICE_ATTR(reg, S_IRUGO, show_##reg, 0)


#define MK_DEV_FILE(reg) DEVICE_CREATE_FILE(dev, &dev_attr_##reg)

DEF_DEV_ATTR_PMMR(ATUVID);
DEF_DEV_ATTR_PMMR(ATUCMD);
DEF_DEV_ATTR_PMMR(ASVIR);
DEF_DEV_ATTR_PMMR(PCSR);
 
static void mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_route_d0);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_route_d1);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_route_d2);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_route_d3);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_route_d4);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_route_d5);
	DEVICE_CREATE_FILE(dev, &dev_attr_led3);
	DEVICE_CREATE_FILE(dev, &dev_attr_led4);
	DEVICE_CREATE_FILE(dev, &dev_attr_bb_len);
	DEVICE_CREATE_FILE(dev, &dev_attr_bb_pa);
	DEVICE_CREATE_FILE(dev, &dev_attr_tbtmp_pa);
	DEVICE_CREATE_FILE(dev, &dev_attr_mumem_pa);
	DEVICE_CREATE_FILE(dev, &dev_attr_debug);
	DEVICE_CREATE_FILE(dev, &dev_attr_pci_env);
	DEVICE_CREATE_FILE(dev, &dev_attr_cpld_rev);
	DEVICE_CREATE_FILE(dev, &dev_attr_hpi_mask);
	DEVICE_CREATE_FILE(dev, &dev_attr_ticks);
	DEVICE_CREATE_FILE(dev, &dev_attr_auxClock);
	DEVICE_CREATE_FILE(dev, &dev_attr_sysslot);

	MK_DEV_FILE(ATUVID);
	MK_DEV_FILE(ATUCMD);
	MK_DEV_FILE(ASVIR);
	MK_DEV_FILE(PCSR);
}

static void __init acq200_proc_init(void)
{
	if ( proc_acq200 == 0 ){
		proc_acq200 = proc_mkdir( "acq200", proc_root_driver );
	}

	create_proc_read_entry(
		"abort_debug_off", 0, proc_acq200, dma_proc_abort_debug, 0 );
	create_proc_read_entry(
		"abort_debug_lo", 0, proc_acq200, dma_proc_abort_debug, 
		(void*)1 );
	create_proc_read_entry(
		"abort_debug_hi", 0, proc_acq200, dma_proc_abort_debug, 
		(void*)10 );

	
}


static int acq200_core_probe(struct device * dev)
{
	static struct file_operations core_fops = {
		open: acq200_core_open,
		release: acq200_core_release
	};

	int rc;

	acq200_proc_init();
	mk_dev_sysfs(dev);

	T( "acq200_core_init() %d\n", acq200_core_major  );

	rc = register_chrdev( acq200_core_major = 0, 
			      "acq200-core", 
			      &core_fops );

	if ( rc < 0 ){
		E( "can't get major %d\n", acq200_core_major );
		return rc;
	}else if ( acq200_core_major == 0 ){
		acq200_core_major = rc;
	}
	return 0;
}

static int acq200_core_remove(struct device * dev)
{
	return 0;
}
static struct device_driver core_device_driver = {
	.name  = "acq200core",
	.bus   = &platform_bus_type,
	.probe = acq200_core_probe,
	.remove= acq200_core_remove
};

static struct platform_device core_device = {
	.name  = "acq200core",
	.id    = 0,
	.dev = {
		.kobj.name = "acq200core",
	},
};


static int __init acq200_core_init(void)
{
	int rc = driver_register(&core_device_driver);

	if (rc != 0){
		return rc;
	}else{
		return platform_device_register(&core_device);

	}
}


/*
 * pick up ethernet mac addresses from command line
 * D-TACQ's OUI is 0x002154
 */

#define D_TACQ_MAC_BLOCK  { '\x0', '\x21', '\x54', '\x0', '\x0', '\x0' }

#define MIN(a,b)  ((a)<(b)?(a):(b))

static unsigned char acq200_gEmac0[6] = D_TACQ_MAC_BLOCK;
static unsigned char acq200_gEmac1[6] = D_TACQ_MAC_BLOCK;

int acq200_get_mac( int imac, int nmac, unsigned char the_mac[] )
{
	unsigned char* psrc;
	int ibyte;
	unsigned char tmac[6];
	
	switch( imac ){
	case 0:
		psrc = acq200_gEmac0; break;
	case 1:
		psrc = acq200_gEmac1; 
		if (acq200_gEmac1[3] != '\0' ||
		    acq200_gEmac1[4] != '\0' ||
		    acq200_gEmac1[5] != '\0'    ){
			break;
		}
		/* else fall thru */
	default:
		/* invent a second mac address as f(mac0) */
		psrc = tmac;
		memcpy(tmac, acq200_gEmac0, sizeof(tmac));
		tmac[3] += imac;
		break;
	}

	nmac = MIN( nmac, 6 );

	for ( ibyte = 0; ibyte != nmac; ++ibyte ){
		the_mac[ibyte] = psrc[ibyte];
	}

	return nmac;
}

EXPORT_SYMBOL_GPL(acq200_get_mac);

static int acq200_decode_mac( char* str, unsigned char the_mac[] )
/* str should be a sequence of bytes :XX:XX:XX */
{
#define MACFMT "%x:%x:%x:%x:%x:%x"
#define DECFMT "%d:%d:%d:%d:%d:%d"
	unsigned hmac[6] = { 0, };	/* octets, hex decoded */
	unsigned dmac[6] = { 0, };	/* octets, dec decoded */
	int ito, ifrom;				
	int is_decimal_def = 0;		/* once dec discovered, all dec */
 
/** IEEE standard is to define octets in hex. 
 *  We do this by default
 *  But older D-TACQ u-boot defs were dec. 
 *  We attempt to detect these by looking for rollover past 255.
 *  doesn't work for values < 100, so we trap ls octet < 100 (*)
 *  Only do this for affected machines
 */
	int replace_count = sscanf( str, MACFMT,
			&hmac[0], &hmac[1], &hmac[2],
			&hmac[3], &hmac[4], &hmac[5] );

	sscanf(str, DECFMT,
			&dmac[0], &dmac[1], &dmac[2],
			&dmac[3], &dmac[4], &dmac[5] );


	/* fill the mac from the back. The front side is already there */
	
	for (ifrom = replace_count, ito = 5; ifrom--; ito--){
		if (is_decimal_def == 0 && hmac[ifrom] <= 255){			
			if (ito == 5 &&
                            (machine_is_acq100() || machine_is_acq200()) &&
			    hmac[ifrom] < 100 && dmac[ifrom] > 0	     ){
				/* (*) special case, assume old dec setting */
				the_mac[ito] = dmac[ifrom]; 
				is_decimal_def = 1;

				info("special low octet: [5] %x", dmac[ifrom]);
			}else{
				the_mac[ito] = hmac[ifrom];
			}
		}else if (dmac[ifrom]){
			/* hex overflow, must be dec def > 100 => use dec */
			the_mac[ito] = dmac[ifrom];
			is_decimal_def = 1;

			info("decimal octet definition detected: [%d] %x", 
			     ifrom, dmac[ifrom]);
		}else{
			err("no decimal value for octet %d of %s", ifrom, str);
		}
	}

	info( "[%s] "MACFMT" replaced last %d",
	      str,
		the_mac[0], the_mac[1], the_mac[2],
		the_mac[3], the_mac[4], the_mac[5],
		replace_count );

	return 0;
}

static int __init init_mac0(char *str)
{
	return acq200_decode_mac( str, acq200_gEmac0 );
}

static int __init init_mac1(char *str)
{
	return acq200_decode_mac( str, acq200_gEmac1 );
}

__setup( "gEmac0=", init_mac0 );
__setup( "gEmac1=", init_mac1 );

module_init(acq200_core_init);






