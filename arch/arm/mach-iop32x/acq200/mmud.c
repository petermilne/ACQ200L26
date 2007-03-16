//#define USEBIGBUF

/* ------------------------------------------------------------------------- */
/* acq200-mmud.c driver for acq200/iop321 message unit (MMUD)                    */
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
 * Attempt to decipher the MMU. Feed in a VA, get PA and MMU table chars
 *
 * But results are disappointing.
 * not convinced there is a linear mapping to the pte's.
 * Next time: use the DMAC/AAU (work with physaddrs) to map into
 * a buffer wehere we _can_ manipulate.
 * But - is it worth it? Is life long enough?. adios.
 */

#define VERID "$Revision: 1.3 $ Build 1000 " __DATE__

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

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
#endif



#include "acq200.h"
#include "acq200_debug.h"



static ssize_t show_id(
	struct device_driver *driver, char * buf)
{
	u32 reg;

	asm volatile("mrc p15, 0, %0, c0, c0, 0" : "=r" (reg));
        return sprintf(buf,"0x%08x\n", reg);
}
static DRIVER_ATTR(id,S_IRUGO,show_id,0);

u32 get_translation_base(void)
{
	u32 reg;

	asm volatile("mrc p15, 0, %0, c2, c0, 0" : "=r" (reg));
	return reg;
}

static ssize_t show_version(
	struct device_driver *driver, char * buf)
{
        return sprintf(buf,"%s\n", VERID);
}
static DRIVER_ATTR(version,S_IRUGO,show_version,0);

static ssize_t show_tlb(
	struct device_driver *driver, char * buf)
{
        return sprintf(buf,"0x%08x\n", get_translation_base());
}
static DRIVER_ATTR(tlb,S_IRUGO,show_tlb,0);

static u32 vat;

#define FT_SECTION 2


static ssize_t show_translation(
	struct device_driver *driver, char * buf)
{
	int len = 0;
#define PRT(txt, val) len += sprintf( buf+len, "%20s : 0x%08x\n", #txt, val)
//#define PRT(txt, val) info("%20s : 0x%08x", #txt, val)
	u32 tlb = get_translation_base();
	PRT(va, vat);
	PRT(tlb, tlb);
	u32 tlbva = tlb^0x40000000; /* WARNING: quirk of ACQ200 layout */
	PRT(tlbva, tlbva);
	u32 tbix = (vat>>18)&~3;
	PRT(tbix, tbix);
	u32 *fld = (u32*)(tlbva|tbix);
	PRT(fld, (u32)fld);
	
	int fltyp = *fld&3;
	len += sprintf(buf+len, "fld@0x%p is 0x%08x %s ",
		       fld, *fld,
		       fltyp==0? "fault":
		       fltyp==1? "pte":
		       fltyp==2? "section":
		                 "RESERVED" );
	switch(fltyp){
	case FT_SECTION:
		len += sprintf(buf+len, "AP:%d ", (*fld&0xc00)>>10);
		len += sprintf(buf+len, "DOM:%x ", (*fld&0x1e)>>5);
		len += sprintf(buf+len, "%c%c%c ",
			       *fld&0x10?'I':'i',
			       *fld&0x08?'C':'c',
			       *fld&0x04?'B':'b');
		len += sprintf(buf+len, "pa:0x%08x", tlb+(vat&0x000fffff));
		break;
	default:
		;
	}
	return len += sprintf(buf+len, "\n");
}

static ssize_t set_translate_va(
	struct device_driver *driver, const char * buf, size_t count)
{
	sscanf(buf, "0x%x", &vat) || sscanf(buf, "%x", &vat);
	return strlen(buf);
}

static DRIVER_ATTR(translate, S_IRUGO|S_IWUGO,
		   show_translation, set_translate_va);

 
static ssize_t dump_va(
	struct device_driver *driver, char * buf)
{
	info("dump_va(%p %p %d)", buf, (void*)vat, 64);
	info("%04x %04x %04x %04x", 
	     ((u32*)vat)[0], ((u32*)vat)[1],
	     ((u32*)vat)[2], ((u32*)vat)[3]);
	info("%04x %04x %04x %04x", 
	     ((u32*)vat)[4], ((u32*)vat)[5],
	     ((u32*)vat)[6], ((u32*)vat)[7]);
	info("%04x %04x %04x %04x", 
	     ((u32*)vat)[8], ((u32*)vat)[9],
	     ((u32*)vat)[10], ((u32*)vat)[11]);
	info("%04x %04x %04x %04x", 
	     ((u32*)vat)[12], ((u32*)vat)[13],
	     ((u32*)vat)[14], ((u32*)vat)[15]);
	return copy_to_user(buf, (void*)vat, 64);
}
	
static DRIVER_ATTR(dump,S_IRUGO,dump_va,0);


static void mk_sysfs(struct device *dev)
{
	dbg(1,  "calling device_create_file dev %p", dev );

	driver_create_file(dev->driver, &driver_attr_id);
	driver_create_file(dev->driver, &driver_attr_tlb);
	driver_create_file(dev->driver, &driver_attr_translate);
	driver_create_file(dev->driver, &driver_attr_version);
	driver_create_file(dev->driver, &driver_attr_dump);
}

static void rm_sysfs(struct device *dev)
{
	driver_remove_file(dev->driver, &driver_attr_id);
	driver_remove_file(dev->driver, &driver_attr_tlb);
	driver_remove_file(dev->driver, &driver_attr_translate);
	driver_remove_file(dev->driver, &driver_attr_version);
	driver_remove_file(dev->driver, &driver_attr_dump);
}





static int acq200_mmud_probe(struct device * dev)
{
	mk_sysfs(dev);

	return 0;
}



static int acq200_mmud_remove(struct device * dev)
{
	rm_sysfs(dev);
	return 0;
}





static struct device_driver mmud_device_driver = {
	.name		= "acq200mmud",
	.bus		= &platform_bus_type,
	.probe          = acq200_mmud_probe,
	.remove         = acq200_mmud_remove,
};


static void mmud_release(struct device *dev)
{
	printk("mmud_release\n");
}

static struct platform_device mmud_device = {
	.name		= "acq200mmud",
	.id		= 0,
	.dev = {
		.release   = mmud_release,
		.kobj.name	= "acq200mmud",
	},
};


static int __init acq200_mmud_init( void )
{
	driver_register(&mmud_device_driver);

	return platform_device_register(&mmud_device);
}


static void __exit
acq200_mmud_exit_module(void)
// Remove DRIVER resources on module exit
{
	platform_device_unregister(&mmud_device);	
	driver_unregister(&mmud_device_driver);
}

module_init(acq200_mmud_init);
module_exit(acq200_mmud_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ200 Message Unit");

