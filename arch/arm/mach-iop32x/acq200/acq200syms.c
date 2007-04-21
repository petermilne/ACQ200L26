/* liunux/arch/arm/mach-iop3xx/acq200syms.c */

#include <linux/module.h>
#include <linux/user.h>

#include <linux/ioport.h>

#include "acq200.h"


void acq200_device_create_file(
	struct device * dev, struct device_attribute * attr,
	const char *file, int line)
{
	if (device_create_file(dev, attr)){
		err("%s:%d device_create_file", file, line);
	}
}

extern void acq200_driver_create_file(
	struct device_driver *drv, struct driver_attribute * attr,
	const char* file, int line)
{
	if (driver_create_file(drv, attr)){
		err("%s:%d driver_create_file", file, line);
	}
}

extern int acq200_copy_to_user(
	void *to, const void *from, unsigned long n, int line
	)
{
	if (copy_to_user(to, from, n)){
		err("FAILED at line %d", line);
		BUG();
	}
	return 0;
}
	
EXPORT_SYMBOL_GPL(acq200_device_create_file);
EXPORT_SYMBOL_GPL(acq200_driver_create_file);

EXPORT_SYMBOL_GPL(acq200_get_bigbuf_resource);
EXPORT_SYMBOL_GPL(acq200_get_mumem_resource);
EXPORT_SYMBOL_GPL(acq200_get_tblock_resource);
EXPORT_SYMBOL_GPL(acq200_mask_irq);
EXPORT_SYMBOL_GPL(acq200_unmask_irq);
