/* ------------------------------------------------------------------------- */
/* acq200-inline-dma.h driver inline HAL primitives for iop321 DMAC          */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __ACQ200_INLINE_DMA_H__
#define __ACQ200_INLINE_DMA_H__

#include "acq200-dmac.h"
#include <asm/arch/iop321-dma.h>
#ifndef MAXCHAIN
#define MAXCHAIN 8
#endif


struct DmaChannel {
	volatile u32* regs;
	int nchain;
	int id;
	const char* description[MAXCHAIN];      /* for diags only */
	struct iop321_dma_desc* dmad[MAXCHAIN];
};



#define DMA_REG(dmac, reg) ((dmac).regs[(reg)/sizeof(u32)])
#define DMA_REGP(dmacp, reg) ((dmacp)->regs[(reg)/sizeof(u32)])


#define DMA_FIRE(dmac) \
        (DMA_REG(dmac, DMA_CCR) = IOP321_CCR_CE)

#define DMA_RELOAD(dmac) \
	(DMA_REG(dmac, DMA_CCR) = IOP321_CCR_CE|IOP321_CCR_CR)

#define DMA_STA(dmac) \
        (DMA_REG(dmac, DMA_CSR))

#define DMA_DONE(dmac, stat) \
        (((stat = DMA_STA(dmac)) & IOP321_CSR_CAF) == 0)

#define DMA_BUSY(dmac, stat) \
	(((stat = DMA_STA(dmac)) & IOP321_CSR_CAF) != 0)

#define EOX (IOP321_CSR_EOCIF|IOP321_CSR_EOTIF)

static inline void dma_copy_chain(
		struct DmaChannel *to, struct DmaChannel *from, int ii)
{
	int ito = 0;
	for (; ii < from->nchain; ++ii, ++ito){
		to->description[ito] = from->description[ii];
		to->dmad[ito] = from->dmad[ii];
	}
	to->nchain = ito;
}

static inline int dma_eox(struct DmaChannel *dmac, u32 status)
{
	status = DMA_STA(*dmac) & EOX;

	if (status != 0){
		DMA_STA(*dmac) = status;
	}
	return status;
}


#define DMA_PRECHARGE(dmac, pci_addr) \
        (dmac.dmad[0]->PDA = pci_addr)

#define DMA_PRECHARGEN(dmac, ix, pci_addr) \
        (dmac.dmad[ix]->PDA = pci_addr)
#define DMA_DISABLE(dmac) (DMA_REG(dmac, DMA_CCR) = 0)

#define DMA_ARM(dmac)						\
        do {							\
		DMA_DISABLE(dmac);		                \
		DMA_REG(dmac, DMA_NDAR) = (dmac).dmad[0]->pa;	\
	} while(0)

#define DMA_ARM_DIRECT(dmac, _dmad)			\
        do {						\
		DMA_DISABLE(dmac);			\
                dmac.dmad[0] = _dmad;			\
		DMA_REG(dmac, DMA_NDAR) = _dmad->pa;	\
	} while(0)


static inline void dma_cleanup(struct DmaChannel *dmac)
{
	int ic;
	
	for (ic = 0; ic < dmac->nchain; ++ic){
		acq200_dmad_free(dmac->dmad[ic]);
	}
	dmac->nchain = 0;
}

static inline void dma_append_chain(
	struct DmaChannel* channel, 
	struct iop321_dma_desc* dmad,
	const char *description)
{
	int nchain = channel->nchain;

	if (nchain >= MAXCHAIN) BUG();

	if (nchain){
		channel->dmad[nchain-1]->NDA = dmad->pa;
	}
	channel->dmad[nchain] = dmad;
	channel->description[nchain] = description;
	++channel->nchain;
}

static inline void dma_append_chain_recycle(
	struct DmaChannel* channel,
	const char *description)
/* DMAD's are already present .. */
{
	int nchain = channel->nchain;

	if (nchain >= MAXCHAIN) BUG();

	if (nchain){
		channel->dmad[nchain-1]->NDA = channel->dmad[nchain]->pa;
	}

	channel->dmad[nchain]->NDA = 0;
	channel->description[nchain] = description;
	++channel->nchain;
}

static inline struct DmaChannel* dma_allocate_fill_channel(int chan)
{
	struct DmaChannel* channel =
		kzalloc(sizeof(struct DmaChannel), GFP_KERNEL);
	int ii;

	channel->id = chan;
	channel->regs = chan==0? IOP321_DMA0_CCR: IOP321_DMA1_CCR;

	for (ii = 0; ii < MAXCHAIN; ++ii){
		channel->dmad[ii] = acq200_dmad_alloc();
		channel->dmad[ii]->NDA =
		channel->dmad[ii]->PDA =
		channel->dmad[ii]->PUAD =
		channel->dmad[ii]->LAD =
		channel->dmad[ii]->BC =
		channel->dmad[ii]->DC = 0;
	}

	return channel;
}

static inline void dma_free_channel(struct DmaChannel* channel)
{
	int ii;
	for (ii = 0; ii < MAXCHAIN; ++ii){
		if (channel->dmad[ii]){
			acq200_dmad_free(channel->dmad[ii]);
		}
	}
	kfree(channel);
}

#define DEFINE_DMA_CHANNEL(name, ch)		\
static struct DmaChannel name = {		\
	.regs = IOP321_DMA##ch##_CCR,		\
	.id = ch				\
}













#endif /* __ACQ200_INLINE_DMA_H__ */
