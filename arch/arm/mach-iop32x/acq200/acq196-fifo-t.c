/** customise for acq196t - Thomson N sample special ..*/

static void init_endstops_control_target(void);
static void init_control_target(void);


#define DTACQ_MACH_DRIVER_INIT(dev) init_control_target()
#define DEVICE_CREATE_PROC_ENTRIES(root) pbc_create_proc_entries(root)

#define ACQ196T
#include "acq196-fifo.c"

#include "acq100_rtm_t.h"
#include "prebuiltChainUtils.h"

int control_wanted = 0;
module_param(control_wanted, int, 0644);

int control_size = 0x00010000;
module_param(control_size, int, 0644);

int control_block = 0x00000400;
module_param(control_block, int, 0644);

int control_tbix = -1;
module_param(control_tbix, int, 0444);

int control_tboffset = -1;
module_param(control_tboffset, int, 0444);

int control_numblocks = 0;
module_param(control_numblocks, int, 0444);

int set_lowlat = 0;
module_param(set_lowlat, int, 0644);

int odd_word_hole_hack = 0;
module_param(odd_word_hole_hack, int, 0644);

static struct pci_mapping control_target;
static struct pci_mapping t0;		/* times at start */
static struct pci_mapping t1;		/* times at end   */
struct pci_mapping acq196t_t2;		/* times at end of chain 2 */
#define t2 acq196t_t2

#define TS(tx, block) (((u32*)((tx).va))[block])
#define TSP(tx, block) ((tx).pa + (block)*sizeof(u32))

struct PrebuiltChain* pb_chains;

static void* control_va(int iblock)
{
	return control_target.va + iblock*control_block;
}

static u32 control_pa(int iblock)
{
	return control_target.pa + iblock*control_block;
}

static void fill_control_target(void)
{
	int ib;
	unsigned off;

	info("len:%d pa:0x%08x va:%p", 
	     control_target.len, control_target.pa, control_target.va);

	for (ib = off = 0; off < control_size; ib++, off += control_block){
		memset(control_va(ib), ib, control_block);
	}

	control_numblocks = ib;
}


static void init_timestamps(struct pci_mapping* map)
{
	map->len = control_numblocks*sizeof(u32);
	map->va = kzalloc(map->len, GFP_KERNEL);
	
	map->pa = dma_map_single(DG->dev, map->va, map->len, DMA_FROM_DEVICE);	
}


u32 getusecs(u32 t0, u32 t1)
{
	u32 usecs;

	if (t1 > t0){
		usecs = (t1 - t0)/50;
		if (usecs > 100000){
			usecs = 0;
		}
	}else{
		usecs = 0;
	}
	return usecs;
}

int proc_dump_timing(char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	int ib;
	int nb = 0;
	if (eof){
		*eof = 1;
	}
	dma_sync_single_for_cpu(DG->dev, t0.pa, t0.len, DMA_FROM_DEVICE);
	dma_sync_single_for_cpu(DG->dev, t1.pa, t1.len, DMA_FROM_DEVICE);
	dma_sync_single_for_cpu(DG->dev, t2.pa, t2.len, DMA_FROM_DEVICE);

	for (ib = 0; ib < control_numblocks; ++ib){
		unsigned gtsr0 = TS(t0, ib);
		unsigned gtsr1 = TS(t1, ib);
		unsigned gtsr2 = TS(t2, ib);

		nb += snprintf(buf+nb, len - nb, 
			"[%02d] %08x %08x %08x %d %d usecs\n",
			       ib, gtsr0, gtsr1, gtsr2, 
			       getusecs(gtsr0, gtsr1),
			       getusecs(gtsr0, gtsr2));
		if (len - nb == 0){
			break;
		}		
	}

	return nb;
}


static void acq196_t_create_proc_entries(void)
{
	create_proc_read_entry(
		"fifo-t-timing",  S_IRUGO, proc_acq200, proc_dump_timing, 0);
}
static void init_control_target(void)
{
	TBLE* control_mem = acq200_reserveFreeTblock();
	/* round it all up to the nearest control_size boundary */
	u32 mask = ~(control_size-1);
	void *va_block = VA_TBLOCK(control_mem->tblock);
	u32 pa_block = PA_TBLOCK(control_mem->tblock);

	control_numblocks = control_size/control_block;

	assert(control_mem != 0);

	control_target.len = control_size;
	control_target.pa = (pa_block + control_size-1) & mask;
	control_target.va = 
		(void*)((u32)(va_block + control_size-1) & mask);
	control_tbix = control_mem->tblock->iblock;
	control_tboffset = control_target.pa - pa_block;
	fill_control_target();

	*IOP321_IALR2 = mask;
	*IOP321_IATVR2 = control_target.pa;

	info("IATVR2:0x%08x IALR2:0x%08x", *IOP321_IATVR2, *IOP321_IALR2);

	init_timestamps(&t0);
	init_timestamps(&t1);
	init_timestamps(&t2);
	acq196_t_create_proc_entries();	
}

/** hack to avoid the "oddword hole" fault *..
 *  Assume ES,S0,S1. Copy S0+2 to S1 then we have
 *  ES[96]
 *  S0[0,1,2,3,4,5,6...96]  1,3,5 hosed by fault
 *  S1[1,2,3,4,5,6,7...95]  2,4,6 hosed by fault
 */
#define SAMPLE_LEN	192

static inline void mk_local_to_local_offset_hack(
	struct PrebuiltChain *pbc,
	int ichain,
	u32 pa)
{
	struct iop321_dma_desc* dmad = acq200_dmad_alloc();
	unsigned S0 = pa + SAMPLE_LEN;
	unsigned S1 = pa + 2*SAMPLE_LEN;

	mk_link(pbc, ichain, dmad, 'X');
	pbc->local_to_local = ichain;

	dmad->MM_SRC = S0+sizeof(short);
	dmad->PUAD = 0;
	dmad->MM_DST = S1;
	dmad->BC = DMA_BLOCK_LEN-sizeof(short);
	dmad->DC = DMA_DCR_MEM2MEM;
}


#define MK_LOCAL_TO_LOCAL_OFFSET_HACK(pbc, ichain, pa) \
	mk_local_to_local_offset_hack(pbc, ichain++, pa)

static void _init_endstops_control_target(void)
/** init endstops, including control_target chains */
{
	int iblock;

	DG->eoc_int_modulo_mask = 0;
	init_endstops(control_numblocks);

	/** now build numstops chains
         *  - 0..data_blocks with increasing target offset
         *  - data_blocks..stride : unchanged.
	 */

	if (pb_chains == 0){
		pb_chains = kmalloc(control_numblocks*PBC_SZ, GFP_KERNEL);
	}

	for (iblock = 0; iblock < control_numblocks; ++iblock){
		struct PrebuiltChain *pbc = &pb_chains[iblock];
		struct iop321_dma_desc* endstop;
		int ichain = 0;

		memset(pbc, 0, sizeof(struct PrebuiltChain));

		if (!rb_get(&IPC->endstops, &endstop)){
			err("ENDSTOP STARVED");
			finish_with_engines(- __LINE__);
			return;
		}
		if (isPBChainDesc(endstop)){
			err("Already prebuilt");
			finish_with_engines(- __LINE__);
			return;
		}

		/** this idents container of desc */
		pbc->desc.clidat = pbc;
		pbc->iblock = iblock;
		
		MK_GTSR_SNAP(pbc, ichain, TSP(t0, iblock));
		MK_FIFO_TO_LOCAL(pbc, ichain);
		MK_LOCAL_TO_LOCAL(pbc, ichain, control_pa(iblock));
		if (odd_word_hole_hack){
			MK_LOCAL_TO_LOCAL_OFFSET_HACK(
					pbc, ichain, control_pa(iblock));
		}
		MK_ENDSTOP(pbc, ichain, endstop);

		pbc->length = ichain;
		pbc->insert = prebuilt_insert_local;
		rb_put(&IPC->endstops, &pbc->desc);			
	}

	DG->put_max_empties = control_numblocks/2;
	DG->empty_fill_threshold = control_numblocks/2;
}

static void init_endstops_control_target(void)
/** init endstops, including control_target chains */
{
	if (control_wanted){
		_init_endstops_control_target();
		DMC_WO->handleEmpties = dmc_handle_empties_prebuilt;
		if (set_lowlat){
			acq196_syscon_set_all(ACQ196_SYSCON_LOWLAT);
		}else{
			acq196_syscon_clr_all(ACQ196_SYSCON_LOWLAT);
		}
	}else{
		init_endstops(INIT_ENDSTOPS);
		
	}
}

EXPORT_SYMBOL_GPL(acq196t_t2);
EXPORT_SYMBOL_GPL(control_block);
EXPORT_SYMBOL_GPL(control_numblocks);
