/* ------------------------------------------------------------------------- */
/* acq200-dmac.h internal defs for acq200 dmac controller                    */
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


#ifndef __ACQ200_DMAC_H__
#define __ACQ200_DMAC_H__

#include <linux/interrupt.h>

struct iop321_dma_desc
{
	u32 NDA;				/* next descriptor address */
	u32 PDA;				/* PCI address */
	u32 PUAD;				/* upper PCI address */
	u32 LAD;				/* local address */
	u32 BC;					/* byte count */
	u32 DC;					/* descriptor control */
	u32 pa;                                 /* phys addr of this entry */
	void *clidat;                           /* blind pointer client data */
#define DD_FIFSTAT PUAD   /* fifstat written to PUAD */
};


struct acq200_dma_ring_buffer {
	unsigned short iput;
	unsigned short iget;
	unsigned nput;
	unsigned nget;
	struct iop321_dma_desc **buffers;      /* -> contiguous vmem block */
	unsigned short hitide;
	unsigned short lotide;
	const char* name;
};


int acq200_rb_init(
	struct acq200_dma_ring_buffer* rb,
	int rblen /* MUST BE POWER OF 2 */);

void acq200_rb_drain(struct acq200_dma_ring_buffer* rb);

/* WARNING: SLOW! */

int acq200_free_rb(struct acq200_dma_ring_buffer *rb);



/*
 * ppmu - functionality supplied by iop321-ppmu.c
 */
extern void iop321_start_ppmu(void);
extern void iop321_stop_ppmu(void);


struct IPC;



struct InterruptSync {
	int irq;
	char channel;
	char interrupted;    // clear before enable
	char requested;
	char enabled;        /* IRQ is enabled */
	spinlock_t bh_lock;  /* lock IRQ for ops on enabled */

	volatile u32* regs;
	u32 flags;
	struct IPC* ipc;
	void (*isr_cb)(struct InterruptSync *self, u32 flags);
	wait_queue_head_t waitq;
	struct tasklet_struct* tasklet;
	unsigned long clidata;
};

struct DmaChannelSync {
	struct InterruptSync eot;
	struct InterruptSync eoc;
	struct InterruptSync err;
};



#define DMA_CHANNEL_PCI 0
#define DMA_CHANNEL_MEM2MEM 0x80
#define DMA_CHANNEL_POLL    0x40
#define DMA_CHANNEL_NOBLOCK 0x100
#define DMA_CHANNEL_POLL_EZ 0x200	/* poll with schedule() */

#define DMA_DCR_MEM2MEM 0x00000040  /* @@todo belongs <asm/arch/iop321-dma.h> */

int acq200_post_dmac_request( 
	int channel, 
	u32 laddr,
	u32 offset,
	u32 remaddr, 
	u32 bc, 
	int incoming
	);


static inline int acq200_post_mem2mem_dmac_request(
	int channel,
	u32 to,
	u32 from,
	u32 bc
	)
{
	return acq200_post_dmac_request(
		channel|DMA_CHANNEL_MEM2MEM,
		to, 0,
		from, bc, 1);
}

struct iop321_dma_desc* acq200_dmad_alloc(void);
void acq200_dmad_free(struct iop321_dma_desc* dmad);
void acq200_dmad_clear(void);

struct DmaChannelSync* acq200_dma_getDmaChannelSync(void);


void acq200_init_interrupt_hook(
	int irq, struct InterruptSync *is, volatile u32* regs);
int acq200_dma_init_interrupt_hook(
	struct InterruptSync* is,
	unsigned int irq,
	irq_handler_t handler,
	const char * devname,
	volatile u32 *regs
	);

#define DMA_REQUEST_IRQ(is_dma, irq, chan, type) \
        do { \
		if (acq200_dma_init_interrupt_hook(\
                        &is_dma[chan].type,\
                        IRQ_IOP321_DMA##chan##_##irq, \
                        dma_irq_##type, \
                        "dma " #chan " " #type, \
                        (void*)IOP321_DMA##chan##_CCR) != 0 ){\
				return -1;\
			}\
	} while(0)

#define DMA_FREE_IRQ(is_dma, type, chan)			 \
        if (is_dma[chan].type.requested ){ \
                free_irq(is_dma[chan].type.irq, &is_dma[chan].type);\
                is_dma[chan].type.requested = 0; \
        }

int acq200_dma_error_count(int channel);

#endif /* ACQ200_DMAC_H__ */
