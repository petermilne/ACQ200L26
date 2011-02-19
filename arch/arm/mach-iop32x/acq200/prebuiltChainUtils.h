
#ifndef __PREBUILTCHAIN_H__
#define __PREBUILTCHAIN_H__

#define MAXPREBUILT 8

struct PrebuiltChain {
	unsigned char length;
	unsigned char fifo_to_local;          /* indexes the_chain */
	unsigned char local_to_host;          /* indexes the_chain */
	unsigned char local_to_local;	     /* indexes the_chain */
	struct iop321_dma_desc* the_chain[MAXPREBUILT];
	struct iop321_dma_desc desc;            
	void (* insert)(
		struct PrebuiltChain *_this, 
		struct iop321_dma_desc* _new);
	char id[MAXPREBUILT];
};



#define getPBChain(dp) (container_of((dp), struct PrebuiltChain, desc))
#define isPBChainDesc(dp) ((void*)getPBChain(dp) == (dp)->clidat)
#define PBC_SZ (sizeof(struct PrebuiltChain))

static inline void mk_link(
	struct PrebuiltChain *pbc, 
	int ichain,
	struct iop321_dma_desc *dmad, 
	char id
	)
{
	if (ichain && pbc->the_chain[ichain-1]){
		pbc->the_chain[ichain-1]->NDA = dmad->pa;
	}
	dmad->clidat = pbc;
	pbc->the_chain[ichain] = dmad;
	if (id){
		pbc->id[ichain] = id;
	}
}

#ifndef EXCLUDE_PBC_INLINES

static inline void mk_fifo_to_local(
	struct PrebuiltChain *pbc, int ichain)
{
	pbc->fifo_to_local = ichain;   /* dmad provided later */
	pbc->the_chain[ichain] = 0;

	pbc->id[ichain] = 'F';
}

static inline void mk_local_to_local(
	struct PrebuiltChain *pbc, 
	int ichain, 
	u32 pa)
{
	struct iop321_dma_desc* dmad = acq200_dmad_alloc();
	mk_link(pbc, ichain, dmad, 'L');
	pbc->local_to_local = ichain;

	dmad->MM_SRC = 0;		/* insert later */
	dmad->PUAD = 0;
	dmad->MM_DST = pa;
	dmad->BC = DMA_BLOCK_LEN;
	dmad->DC = DMA_DCR_MEM2MEM;	
}
static inline void mk_endstop(
	struct PrebuiltChain *pbc, int ichain, 
	struct iop321_dma_desc* endstop)
{
	mk_link(pbc, ichain, endstop, 'E');
}


#define MK_FIFO_TO_LOCAL(pbc, ichain)		mk_fifo_to_local(pbc, ichain++)
#define MK_LOCAL_TO_LOCAL(pbc, ichain, pa) mk_local_to_local(pbc, ichain++, pa)
#define MK_ENDSTOP(pbc, ichain, es)		mk_endstop(pbc, ichain++, es)

#endif

void prebuilt_insert_local (
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new);

void prebuilt_insert_local_host(
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new);

void prebuilt_insert_local_host_nodata(
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new);


#endif
