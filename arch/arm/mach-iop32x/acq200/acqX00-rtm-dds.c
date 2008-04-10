/* ------------------------------------------------------------------------- */
/* acqX00-rtm.c driver for acq200 RTM DDS                                    */
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
 * Module: provides hook to RTMDDS
 * RTMDDS uses an AD9854 DDS chip
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
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
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#define acq200_debug rtm_debug

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acqX00-rtm-dds.h"

#define IN_RANGE(xx, ll, rr) ((xx)>=(ll)&&(xx)<=(rr))

int rtm_debug;
module_param(rtm_debug, int, 0664);



char acq100_rtm_driver_name[] = "acqX00-rtm-dds";
char acq100_rtm_driver_string[] = "D-TACQ RTM driver";
char acq100_rtm_driver_version[] = "$Revision: 1.8 $ build B1002 " __DATE__;
char acq100_rtm_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";



#define DDS_SRC ((volatile u16*)((unsigned)ACQ200_EXTERNIO+0x20))
#define DDS_DST ((volatile u16*)((unsigned)ACQ200_EXTERNIO+0x24))
#define RIO_CR1 ((volatile u16*)((unsigned)ACQ200_EXTERNIO+0x28))
#define RIO_CR2 ((volatile u16*)((unsigned)ACQ200_EXTERNIO+0x2c))
#define DDS_OUTSEL_NONE 0x7    /* 0..5 => DI0..5 */

/* DI0 == 1 but DO0 == 0 ? this is logic? 
 * How many electronic engineers does it take to screw in a light bulb?
 */

#define DDS_SRC_REFCLK 0
#define DDS_SRC_DI0    1
#define DDS_SRC_DI1    2
#define DDS_SRC_DI2    3
#define DDS_SRC_DI3    4
#define DDS_SRC_DI4    5
#define DDS_SRC_DI5    6
#define DDS_SRC_ECM    7	/* External Clock Multiplier (where fitted) */

#define DDS_SRC_MIN	0
#define DDS_SRC_MAX	7

#define DDS_DST_DO0 0
#define DDS_DST_DO1 1
#define DDS_DST_D02 2
#define DDS_DST_DO3 3
#define DDS_DST_DO4 4
#define DDS_DST_DO5 5


static u8 rio_outputs;

static void set_rio_outputs(u8 outputs) {
	rio_outputs = outputs;

	*RIO_CR1 = outputs&0xf;
	*RIO_CR2 = (outputs&0x3) >> 4;
}


static u8 dds_in;

static void set_dds_in(u8 in) {
	*DDS_SRC = dds_in = in;
}

static u8 dds_out;

static void set_dds_out(u8 out)
{
	*DDS_DST = dds_out = out;
}


#define DDS_BASE (ACQ200_EXTERNIO+0x300)    

#define DDS_REG(offset) ((volatile u16*)((unsigned)DDS_BASE+2*(offset)))

/*
 * name convention ... data is BIG ENDIAN, [0] ... [n] where 0 is MSB
 */

#define DDS_PHASE_ADJUST10 DDS_REG(0x00)
#define DDS_PHASE_ADJUST11 DDS_REG(0x01)

#define DDS_PHASE_ADJUST20 DDS_REG(0x02)
#define DDS_PHASE_ADJUST21 DDS_REG(0x03)

#define DDS_FTW10          DDS_REG(0x04)   /* Frequency Tuning Word */
#define DDS_FTW11          DDS_REG(0x05)
#define DDS_FTW12          DDS_REG(0x05)
#define DDS_FTW13          DDS_REG(0x07)
#define DDS_FTW14          DDS_REG(0x08)
#define DDS_FTW15          DDS_REG(0x09)
#define DDS_FTW1X(r)       DDS_REG(0x04+(r))

#define DDS_FTW20          DDS_REG(0x0a)
#define DDS_FTW21          DDS_REG(0x0b)
#define DDS_FTW22          DDS_REG(0x0c)
#define DDS_FTW23          DDS_REG(0x0d)
#define DDS_FTW24          DDS_REG(0x0e)
#define DDS_FTW25          DDS_REG(0x0f)



#define DDS_DFW0           DDS_REG(0x10)   /* Delta Frequency Word */
#define DDS_DFW1           DDS_REG(0x11)
#define DDS_DFW2           DDS_REG(0x12)
#define DDS_DFW3           DDS_REG(0x13)
#define DDS_DFW4           DDS_REG(0x14)
#define DDS_DFW5           DDS_REG(0x15)

#define DDS_UPDATE_CLK0    DDS_REG(0x16)
#define DDS_UPDATE_CLK1    DDS_REG(0x17
#define DDS_UPDATE_CLK2    DDS_REG(0x18)
#define DDS_UPDATE_CLK3    DDS_REG(0x19)
#define DDS_RAMP_RATE_CLK0 DDS_REG(0x1a)
#define DDS_RAMP_RATE_CLK1 DDS_REG(0x1b)
#define DDS_RAMP_RATE_CLK2 DDS_REG(0x1c)

#define DDS_CSR0           DDS_REG(0x1d)
#define DDS_CSR1           DDS_REG(0x1e)
#define DDS_CSR2           DDS_REG(0x1f)
#define DDS_CSR3           DDS_REG(0x20)

#define DDS_OSKEY_I_M0     DDS_REG(0x21)
#define DDS_OSKEY_I_M1     DDS_REG(0x22)

#define DDS_OSKEY_Q_M0     DDS_REG(0x23)
#define DDS_OSKEY_Q_M1     DDS_REG(0x24)

#define DDS_OSKEY_RR       DDS_REG(0x25)

#define DDS_QDAC_0         DDS_REG(0x26)
#define DDS_QDAC_1         DDS_REG(0x27)

#define DDS_CSR1_HISPEED_PLL 0x40
#define DDS_CSR1_DTACQ_PLL DDS_CSR1_HISPEED_PLL      /** remove PLL bypass */
#define DDS_CSR1_MULT_MIN  4
#define DDS_CSR1_MULT_MAX  20


#define DDS_CSR2_INT_UPDATE_CLK 0x01            /** always set */
#define DDS_CSR2_SRCQDAC        0x10



#define SET_DDS(reg, value) do { \
        dbg(2, "%20s 0x%p = 0x%02x", #reg, (reg), (value)); \
        *(reg) = (value); \
        } while(0)

#define GET_DDS(reg, value) do { \
        (value) = (*reg)&0x00ff; \
        dbg(2, "%20s 0x%p : 0x%02x", #reg, (reg), (value)); \
        } while(0)




static struct device_driver rtm_driver;

static void set_ftw1(unsigned ftw[6])
{
	int iword;

	for (iword = 0; iword != 6; ++iword){
		SET_DDS(DDS_FTW1X(iword), ftw[iword]);
	}
}


int acq200_dds_set_ftw1_bin(const unsigned char ftw[6])
{
	int iword;

	for (iword = 0; iword != 6; ++iword){
		SET_DDS(DDS_FTW1X(iword), ((unsigned char*)ftw)[iword]);
	}
	return 0;
}

int acq200_dds_set_qdac(short value)
{
	value &= 0x0fff;
	SET_DDS(DDS_QDAC_0, value >> 8);
	SET_DDS(DDS_QDAC_1, value & 0x0ff);
	return 0;
}

static void get_ftw1(unsigned ftw[6])
{
	int iword;

	for (iword = 0; iword != 6; ++iword){
		GET_DDS(DDS_FTW1X(iword), ftw[iword]);
	}
}

static int dds_refclk_mult = 4;
static int dds_hispeed = 0;

static void set_csr1(int hispeed) {
	unsigned char csr1 = hispeed? DDS_CSR1_HISPEED_PLL: 0;
	dds_hispeed = hispeed;
	SET_DDS(DDS_CSR1, csr1|dds_refclk_mult);
	dbg(1, "hispeed:0x%02x  write 0x%02x", csr1, csr1|dds_refclk_mult);
}




static void init_dds(void)
{
	/**
	 *  follow recipe by John 11/10/04
         */

	SET_DDS(DDS_CSR0, 0);             /** Comp PD off */
	set_csr1(1);
	SET_DDS(DDS_CSR3, 0x40);          /** default power save mode */
	SET_DDS(DDS_OSKEY_I_M0, 0x0f);
	SET_DDS(DDS_OSKEY_I_M1, 0xc0);
	SET_DDS(DDS_OSKEY_Q_M0, 0x0f);
	SET_DDS(DDS_OSKEY_Q_M1, 0xc0);
}


static inline int is_hexdigit(int d)
{
	return (d >= '0' && d <= '9') || 
	       (d >= 'a' && d <= 'f') ||
	       (d >= 'A' && d <= 'F');
}

static ssize_t store_ftw1(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
#define HBF "%x"
	unsigned ftw[6];
	char bs[4];
	int iw = 0;
	const char *bp = buf;
	char *bsp = bs;

	if (*bp == '+'){
		++bp;
	}
	for (; is_hexdigit(*bp); ++bp){              
		if (bsp - bs == 0){
			*bsp++ = *bp;
		}else if (bsp -bs == 1){
			*bsp++ = *bp;
			*bsp = '\0';
			if (sscanf(bs, HBF, &ftw[iw]) == 1){
				if (++iw == 6){
					set_ftw1(ftw);
					return strlen(buf);
				}else{
					bsp = bs;
				}
			}else{
				dbg(1, "ERROR scan failed");
			}
		}else{
			BUG();
		}
	}

	dbg(1, "ERROR length only %d or not hexdigit", strlen(buf));
	return strlen(buf);
#undef HBF
}



static ssize_t show_ftw1(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
#define HBF "%02x"
	unsigned ftw1[6];

	get_ftw1(ftw1);

	return sprintf(buf, HBF HBF HBF HBF HBF HBF "\n",
		       ftw1[0], ftw1[1], ftw1[2],
		       ftw1[3], ftw1[4], ftw1[5]);
#undef HBF
}


static DEVICE_ATTR(ftw1, S_IRUGO|S_IWUGO, show_ftw1, store_ftw1);

static ssize_t show_hispeed(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d\n", dds_hispeed);
}

static DEVICE_ATTR(hispeed, S_IRUGO, show_hispeed, 0);

static ssize_t store_ftw1_bin(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	if (count == 6){
		acq200_dds_set_ftw1_bin(buf);
	}else{
		err("binary write must be exactly 6 digits");
	}
	return count;
}

static DEVICE_ATTR(ftw1_bin, S_IWUGO, 0, store_ftw1_bin);

static ssize_t store_dds_clksrc(	
	struct device * dev,
	struct device_attribute *attr, 
	const char * buf, 
	size_t count)
{
	int isel;
	int hispeed = 0;

	if (sscanf(buf, "%d", &isel) && 
	    IN_RANGE(isel, DDS_SRC_MIN, DDS_SRC_MAX)){
		switch(isel){
		case DDS_SRC_REFCLK:
			dds_refclk_mult = 4;
			hispeed = 1;
			break;
		case DDS_SRC_ECM:
			dds_refclk_mult = 20;
			break;
		default:
			break;
		}
		set_csr1(hispeed);
		set_dds_in(isel);
	}
            
        return strlen(buf);
}

static ssize_t show_dds_clksrc(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	sprintf(buf, "%d\n", dds_in);

	return strlen(buf);
}

static DEVICE_ATTR(clksrc, S_IRUGO|S_IWUGO, 
		   show_dds_clksrc, store_dds_clksrc);


static ssize_t store_dds_clkdst(	
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int isel;

	if (sscanf(buf, "%d", &isel) && 
	    IN_RANGE(isel, DDS_DST_DO0, DDS_DST_DO5)){
		set_dds_out(isel);
	}
            
        return strlen(buf);
}

static ssize_t show_dds_clkdst(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	sprintf(buf, "%d\n", dds_out);

	return strlen(buf);
}

static DEVICE_ATTR(clkdst, S_IRUGO|S_IWUGO, 
		   show_dds_clkdst, store_dds_clkdst);


static ssize_t store_qdacsrc(	
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int isel;

	if (sscanf(buf, "%d", &isel)){
		int srcqdac = isel? DDS_CSR2_SRCQDAC: 0;
		SET_DDS(DDS_CSR2, DDS_CSR2_INT_UPDATE_CLK|srcqdac);
	}
            
        return strlen(buf);
}


static ssize_t show_qdacsrc(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	unsigned csr2;
	GET_DDS(DDS_CSR2, csr2);
	sprintf(buf, "%d\n", (csr2&DDS_CSR2_SRCQDAC) != 0);

	return strlen(buf);
}

static DEVICE_ATTR(qdacsrc, S_IRUGO|S_IWUGO, 
		   show_qdacsrc, store_qdacsrc);

static ssize_t store_qdac(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int isel;

	if (sscanf(buf, "%d", &isel)){
		acq200_dds_set_qdac((short)isel);
	}

        return strlen(buf);
}

static ssize_t show_qdac(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	unsigned qdac0, qdac1;
	int qdac;

	GET_DDS(DDS_QDAC_0, qdac0);
	GET_DDS(DDS_QDAC_1, qdac1);
	
	qdac = ((qdac0&0x8) != 0? ~0x0fff: 0) | 
		(qdac0&0x0f) << 8 | (qdac1&0x00ff);

	sprintf(buf, "%d\n", qdac);

	return strlen(buf);
}

static DEVICE_ATTR(qdac, S_IRUGO|S_IWUGO, show_qdac, store_qdac);


static ssize_t store_rio_outputs(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int isel = 0;

	if (sscanf(buf, "0x%x", &isel) || sscanf(buf, "%d", &isel)){
		set_rio_outputs(isel);
	}
            
        return strlen(buf);
}

static ssize_t show_rio_outputs(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	sprintf(buf, "%d\n", rio_outputs);

	return strlen(buf);
}

static DEVICE_ATTR(rio_outputs, S_IRUGO|S_IWUGO, 
		   show_rio_outputs, store_rio_outputs);


static ssize_t store_refclk_mult(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	int isel = 0;
	int hispeed = 0;

	if (sscanf(buf, "%d %d", &isel, &hispeed) >= 1){
		if (isel < 4) isel = 4;
		if (isel > 20) isel = 20;
		dds_refclk_mult = isel;
		set_csr1(hispeed);
	}
            
        return strlen(buf);
}

static ssize_t show_refclk_mult(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	sprintf(buf, "%d\n", dds_refclk_mult);

	return strlen(buf);
}

static DEVICE_ATTR(refclk_mult, S_IRUGO|S_IWUGO, 
		   show_refclk_mult, store_refclk_mult);



static void mk_rtm_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_ftw1);
	DEVICE_CREATE_FILE(dev, &dev_attr_ftw1_bin);
	DEVICE_CREATE_FILE(dev, &dev_attr_clksrc);
	DEVICE_CREATE_FILE(dev, &dev_attr_clkdst);
	DEVICE_CREATE_FILE(dev, &dev_attr_rio_outputs);
	DEVICE_CREATE_FILE(dev, &dev_attr_refclk_mult);
	DEVICE_CREATE_FILE(dev, &dev_attr_qdacsrc);
	DEVICE_CREATE_FILE(dev, &dev_attr_qdac);
	DEVICE_CREATE_FILE(dev, &dev_attr_hispeed);
}

static void rtm_dev_release(struct device * dev)
{
	info("");
}


static int rtm_probe(struct device *dev)
{
	info("");
	mk_rtm_sysfs(dev);
	init_dds();
	return 0;
}

static int rtm_remove(struct device *dev)
{
	return 0;
}


static struct device_driver rtm_driver = {
	.name     = "rtmdds",
	.probe    = rtm_probe,
	.remove   = rtm_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device rtm_device = {
	.name = "rtmdds",
	.id   = 0,
	.dev = {
		.release    = rtm_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init rtm_init( void )
{
	int rc = driver_register(&rtm_driver);
	if (rc){
		return rc;
	}
	return platform_device_register(&rtm_device);
}


static void __exit
rtm_exit_module(void)
{
	info("");
	platform_device_unregister(&rtm_device);
	driver_unregister(&rtm_driver);
}

module_init(rtm_init);
module_exit(rtm_exit_module);


EXPORT_SYMBOL_GPL(acq200_dds_set_ftw1_bin);
EXPORT_SYMBOL_GPL(acq200_dds_set_qdac);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for RTM-DDS");


