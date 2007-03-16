/* ------------------------------------------------------------------------- */
/* iop321-ppmu.c driver for iop321 Peripheral Performance Monitoring Unit    */
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

#define VERID "$Revision: 1.1.1.1 $ Build 1000 " __DATE__

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#include <asm/arch-iop32x/iop321.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif



static int mode;
static int is_running;

void iop321_start_ppmu(void) 
{
#ifdef PGOMCOMOUT
	*IOP321_ESR = 0;
	*IOP321_ESR = mode;
	*IOP321_GTMR &= ~IOP321_GTMR_NGCE;
#endif
	is_running = 1;
}

void iop321_stop_ppmu(void)
{
#ifdef PGMCOMOUT
/* 2.6.18.2 - now used in timer gen */
	*IOP321_GTMR = IOP321_GTMR_NGCE;
#endif
	is_running = 0;
}

EXPORT_SYMBOL(iop321_start_ppmu);
EXPORT_SYMBOL(iop321_stop_ppmu);

static ssize_t show_mode(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "%d\n", mode);
}

static ssize_t set_mode(
	struct device *dev,
	struct device_attribute *attr, 
	const char * buf, size_t count)
{
	sscanf( buf, "%d", &mode);
	return strlen(buf);
}
static DEVICE_ATTR(mode, S_IRUGO|S_IWUGO, show_mode, set_mode);



static ssize_t show_running(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "%d\n", is_running);
}

static ssize_t set_running(
	struct device *dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf( buf, "%d", &is_running);
	if (is_running){
		iop321_start_ppmu();
	}else{
		iop321_stop_ppmu();
	}
	return strlen(buf);
}
static DEVICE_ATTR(running, S_IRUGO|S_IWUGO, show_running, set_running);


static ssize_t show_gtsr_us(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf( buf, "%d\n", *IOP321_GTSR/50 );
}

static DEVICE_ATTR(gtsr_us, S_IRUGO, show_gtsr_us, 0);

struct PPMU_CONTROL {
	int ix;
	char *key;
};

static void tab_results(int mode, char* buf)
{
	static struct PPMU_CONTROL controls[7][14] = {
		{ {}, },      /* mode 0 */
		{
			{1,  "M1_PPCIBus_idle"},
			{2,  "M1_PPCIBus_data"},
			{3,  "M1_PCI_ATU_acq"},
			{4,  "M1_PCI_ATU_own"},
			{5,  "M1_PCI_ATU_data"},
			{6,  "M1_PCI_ATU_gnt"},
			{7,  "M1_PCI_ATU_retry"},
			{9,  "M1_IBus_idle"},
			{10, "M1_IBus_data"},
		},
		{
			{1,  "M2_IBus_idle"},
			{2,  "M2_IBus_data"},
			{3,  "M2_AA_acq"},
			{4,  "M2_AA_own"},
			{5,  "M2_AA_data"},
			{6,  "M2_AA_gnt"},
			{7,  "M2_DMA0_acq"},
			{8,  "M2_DMA0_own"},
			{9,  "M2_DMA0_data"},
			{10, "M2_DMA0_gnt"},
			{11, "M2_DMA1_acq"},
			{12, "M2_DMA1_own"},
			{13, "M2_DMA1_data"},
			{14, "M2_DMA1_gnt"},
		},
		{
			{1,  "M3_IBus_idle"},
			{2,  "M3_IBus_data"},
			{3,  "M3_ATU_acq"},
			{4,  "M3_ATU_own"},
			{5,  "M3_ATU_data"},
			{6,  "M3_ATU_gnt"},
			{7,  "M3_core_acq"},
			{8,  "M3_core_own"},
			{9,  "M3_core_data"},
			{10, "M3_core_gnt"}
		},
		{
			{1,  "M4_PPCIbus_idle"},
			{2,  "M4_PPCIbus_data"},
			{3,  "M4_core_sc"},
			{4,  "M4_core_scd"},
			{5,  "M4_DMA0_sc"},
			{6,  "M4_DMA0_scd"},
			{7,  "M4_DMA1_sc"},
			{8,  "M4_DMA1_scd"},
			{9,  "M4_ATU_sc"},
			{10, "M4_ATU_scd"},
			{11, "M4_ATU_ibsc"},
			{12, "M4_ATU_ibscd"},
			{13, "M4_ATU_splits"},
			{14, "M4_ATU_fwl"}
		},
		{
			{1,  "M5_AA_retry"},
			{2,  "M5_AA_xfer"},
			{3,  "M5_DMA0_retry"},
			{4,  "M5_DMA0_xfer"},
			{5,  "M5_DMA1_retry"},
			{6,  "M5_DMA1_xfer"},
			{7,  "M5_core_retry"},
			{8,  "M5_core_xfer"},
			{11, "M5_ATU_retry"},
			{12, "M5_ATU_xfer"},
		},
		{
			{1,  "M6_MCU_tretry"},
			{2,  "M6_MCU_txfer"},
			{3,  "M6_AA_retry"},
			{4,  "M6_DMA0_retry"},
			{5,  "M6_DMA1_retry"},
			{7,  "M6_core_retry"},
			{8,  "M6_ATU_retry"},
		}
		
	};
	int iresult = 0;
	int ibuf = 0;

	if (mode <0 || mode >6 ) return;

	for (; iresult != 14 && controls[mode][iresult].ix; ++iresult){
		ibuf += sprintf( buf+ibuf, "[%2d] %30s : %10u\n",
				 controls[mode][iresult].ix,
				 controls[mode][iresult].key,
				 (IOP321_PECR0)[controls[mode][iresult].ix] );
	}
}

static ssize_t show_results(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{	
	int len = 0;

	len += sprintf(buf+len, "%5s%30s : %d\n", "", "mode", mode );
	len += sprintf(buf+len, "%5s%30s : %10u\n", "", "GTSR", *IOP321_GTSR );
	tab_results(mode, buf+len);
	return strlen(buf);
}
static DEVICE_ATTR(results, S_IRUGO|S_IWUGO, show_results, 0);
	
static ssize_t show_version(
	struct device_driver *driver, 
	char * buf)
{
        return sprintf(buf,"%s\n", VERID);
}
static DRIVER_ATTR(version,S_IRUGO,show_version,0);



static void mk_sysfs(struct device *dev)
{
	device_create_file(dev, &dev_attr_mode);
	device_create_file(dev, &dev_attr_results);
	device_create_file(dev, &dev_attr_running);
	device_create_file(dev, &dev_attr_gtsr_us);
	driver_create_file(dev->driver, &driver_attr_version);
}
static void rm_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_mode);
	device_remove_file(dev, &dev_attr_results);
	device_remove_file(dev, &dev_attr_running);
	device_remove_file(dev, &dev_attr_gtsr_us);
	driver_remove_file(dev->driver, &driver_attr_version);
}


static int iop321_ppmu_probe(struct device * dev)
{
	mk_sysfs(dev);
	return 0;
}
static int iop321_ppmu_remove(struct device * dev)
{
	rm_sysfs(dev);
	return 0;
}

static struct device_driver ppmu_device_driver = {
	.name		= "iop321ppmu",
	.bus		= &platform_bus_type,
	.probe          = iop321_ppmu_probe,
	.remove         = iop321_ppmu_remove,
};


static struct platform_device ppmu_device = {
	.name		= "iop321ppmu",
	.id		= 0,
	.dev = {
		.kobj.name	= "iop321ppmu",
	},
};



static int __init iop321_ppmu_init( void )
{
	driver_register(&ppmu_device_driver);
/*
 * missing form 2.5.70 ...
	return platform_add_devices(devices, ARRAY_SIZE(devices));
*/
	return platform_device_register(&ppmu_device);
}


static void __exit
iop321_ppmu_exit_module(void)
// Remove DRIVER resources on module exit
{
	platform_device_unregister(&ppmu_device);	
	driver_unregister(&ppmu_device_driver);
}

module_init(iop321_ppmu_init);
module_exit(iop321_ppmu_exit_module);



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for iop321 Peripheral Performance Monitoring Unit");

