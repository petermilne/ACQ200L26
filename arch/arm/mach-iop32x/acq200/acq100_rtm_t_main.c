/* ------------------------------------------------------------------------- */
/* acq100_rtm_t.c  - RTM-T adapter		                             */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
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


#define ACQ196

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/moduleparam.h>

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#define acq200_debug rtm_t_debug
#include "acq200.h"
#include "acq200_debug.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"

#include "acq100_rtm_t.h"

#define REVID	"acq100_rtm_t B1001"

int rtm_t_debug;
module_param(rtm_t_debug, int , 0644);


static void __init acq100_redirect(void)
{

	struct resource mumem;

	acq200_get_mumem_resource(&mumem);
	DG->fpga.fifo.pa = virt_to_phys((void*)mumem.start);
	*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_DAC_RTM_T;
	info("set fifo.pa to %x, set SYSCON_DAC_RTM", DG->fpga.fifo.pa);
}
static void __exit acq100_restore(void)
{
	DG->fpga.fifo.pa = ACQ200_FPGA_P+ACQ196_FIFO_OFFSET;
	*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_DAC_RTM_T;
	info("restore fifo.pa to %x", DG->fpga.fifo.pa);
}

#define MBOX_KNOB(MB, reg, perms)					\
static ssize_t store_mbox##MB(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	u32 value = 0;							\
									\
	if (sscanf(buf, "0x%x", &value) || sscanf(buf, "%d", &value)){	\
		*RTMT_REG(reg) = value;				\
	}								\
									\
        return strlen(buf);						\
}									\
									\
static ssize_t show_mbox##MB(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	u32 value = *RTMT_REG(reg);					\
	sprintf(buf, "0x%08x %d\n", value, value);			\
									\
	return strlen(buf);						\
}									\
									\
static DEVICE_ATTR(mbox##MB, (perms), show_mbox##MB, store_mbox##MB)

MBOX_KNOB(Q1, RTMT_Q_MBOX1, S_IRUGO|S_IWUGO);
MBOX_KNOB(Q2, RTMT_Q_MBOX2, S_IRUGO|S_IWUGO);
MBOX_KNOB(H1, RTMT_H_MBOX1, S_IRUGO);
MBOX_KNOB(H2, RTMT_H_MBOX2, S_IRUGO);

static void mk_rtm_t_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxQ1);
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxQ2);
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxH1);
	DEVICE_CREATE_FILE(dev, &dev_attr_mboxH2);
}

static inline int is_magic(u32 mtest)
{
	return mtest == MAGIC1 || mtest == MAGIC2;
}
static int rtm_t_present(void)
{
	return is_magic(*RTMT_REG(RTMT_Q_MBOX1)) && 
	       is_magic(*RTMT_REG(RTMT_Q_MBOX2));
}
static int _rw_test(u32 tv)
{
	u32 rv;

	*RTMT_REG(RTMT_Q_TEST) = tv;
	rv = *RTMT_REG(RTMT_Q_TEST);
	if (rv != tv){
		err("w:%08x r:%08x", tv, rv);		
	}
	return rv != tv;
}
static int bit_test(void)
{
	int rc = 0;
	rc |= _rw_test(0xdeadbeef);
	rc |= _rw_test(0x00000000);
	rc |= _rw_test(0xffffffff);
	rc |= _rw_test(0xaa55aa55);
	rc |= _rw_test(0x55aa55aa);
	rc |= _rw_test(0xcafebabe);
	return rc;
}

extern int rtm_t_spi_master_init(struct device *dev);


static int rtm_t_probe(struct device *dev)
{
	struct platform_device* pd = 
		container_of(dev, struct platform_device, dev);
	int rc;
	info("");

	if (!rtm_t_present()){
		err("RTM-T not found\n");
		return -1;
	}else if (bit_test()){
		err("BIT test failed");
		return -1;
	}else{
		info("RTM-T found, FPGA version %08x id %08x",
		     *RTMT_REG(RTMT_C_REVID), FPGA_ID);	
	}

	mk_rtm_t_sysfs(dev);

	rc = acq100_rtm_t_uart_init();
	if (rc != 0){
		goto uart_fail;
	}
	rc = rtm_t_spi_master_init(dev);

	acq100_redirect();
	return rc;

uart_fail:
	platform_device_unregister(pd);

	return 0;
}

static void rtm_t_dev_release(struct device * dev)
{
	info("");
}

static int rtm_t_remove(struct device *dev)
{
	acq100_rtm_t_uart_exit();
	return 0;
}


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device rtm_t_device = {
	.name	= "acq100_rtm_t",
	.id	= 0,
	.dev	= {
		.release	= rtm_t_dev_release,
		.dma_mask	= &dma_mask
	}
};



static struct device_driver rtm_t_driver = {
	.name	= "acq100_rtm_t",
	.probe	= rtm_t_probe,
	.remove = rtm_t_remove,
	.bus	= &platform_bus_type
};






static int __init acq100_rtm_t_init(void)
{
	int rc;

	info(REVID);

	rc = driver_register(&rtm_t_driver);
	if (rc != 0){
		goto driver_fail;
	}
	rc = platform_device_register(&rtm_t_device);
	if (rc != 0){
		goto device_fail;
	}
	
	return 0;

device_fail:
	driver_unregister(&rtm_t_driver);
driver_fail:
	return rc;
}

static void __exit
acq100_rtm_t_exit_module(void)
{
	platform_device_unregister(&rtm_t_device);
	driver_unregister(&rtm_t_driver);
	acq100_restore();
}

module_init(acq100_rtm_t_init);
module_exit(acq100_rtm_t_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("ACQ100 RTM-T adapter");


