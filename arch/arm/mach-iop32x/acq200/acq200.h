/* acq200.h */



#ifndef __ACQ200_H__
#define __ACQ200_H__

#include <linux/device.h>

#include "acq200_debug.h"

#ifdef __KERNEL__
int acq200_get_bigbuf_resource(struct resource *resource);
int acq200_get_mumem_resource(struct resource *resource);
int acq200_get_tblock_resource(struct resource *resource);
/* WORKTODO - surely a KERNEL API feature ? */
void acq200_mask_irq(int irq);
void acq200_unmask_irq(int irq);
#endif




struct PCI_DMA_BUFFER {
	u32 *va;
	dma_addr_t laddr;
	int direction;
	int mapped;
};

int acq200_post_dmac_request( 
	int channel, 
	u32 laddr,
	u32 offset,
	u32 remaddr, 
	u32 bc, 
	int incoming
);

struct device_attribute;

extern void acq200_device_create_file(
	struct device * dev, struct device_attribute * attr,
	const char *file, int line);

#define DEVICE_CREATE_FILE(dev, attr) \
	acq200_device_create_file(dev, attr, __FILE__, __LINE__)


extern void acq200_driver_create_file(
	struct device_driver *drv, struct driver_attribute * attr,
	const char* file, int line);

#define DRIVER_CREATE_FILE(drv, attr) \
	acq200_driver_create_file(drv, attr, __FILE__, __LINE__)

static inline int PO(int len)
{
	int order = 0;

	for (; (1 << order) * PAGE_SIZE < len; ++order){
		;
	}
	return order;
}

static inline u32 to_mask(u32 mask, u32 value)
{
	int shl;

	if (mask == 0){
		return 0;
	}

	for (shl = 0; ((1<<shl)&mask) == 0; ++shl){
		;
	}
	return (value << shl) & mask;
}

static inline u32 from_mask(u32 mask, u32 value)
{
	int shr;

	if (mask == 0){
		return 0;
	}

	for (shr = 0; ((1<<shr)&mask) == 0; ++shr){
		;
	}
	return (value & mask) >> shr;
}

#endif /* #define __ACQ200_H__ */
