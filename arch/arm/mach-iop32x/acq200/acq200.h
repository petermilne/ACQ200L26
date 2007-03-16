/* acq200.h */



#ifndef __ACQ200_H__
#define __ACQ200_H__

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

#endif /* #define __ACQ200_H__ */
