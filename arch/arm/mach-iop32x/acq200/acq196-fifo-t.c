/** customise for acq196t - Thomson 4 sample special ..*/

static void init_endstops_control_target(void);
static void init_control_target(void);


#define DTACQ_MACH_DRIVER_INIT(dev) init_control_target()
#define DEVICE_CREATE_PROC_ENTRIES(root) pbc_create_proc_entries(root)

#define ACQ196T
#include "acq196-fifo.c"

#include "prebuiltChainUtils.h"

int control_wanted = 0;
module_param(control_wanted, int, 0644);

int control_size = 0x00010000;
module_param(control_size, int, 0644);

int control_block = 0x00000400;
module_param(control_block, int, 0644);

int control_tbix = -1;
module_param(control_tbix, int, 0444);

int control_numblocks = 0;
module_param(control_numblocks, int, 0444);
static struct pci_mapping control_target;

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

static void init_control_target(void)
{
	TBLE* control_mem = acq200_reserveFreeTblock();
	u32 mask = ~(control_size-1);
	void *va_block = VA_TBLOCK(control_mem->tblock);
	u32 pa_block = PA_TBLOCK(control_mem->tblock);

	assert(control_mem != 0);

	control_target.len = control_size;
	control_target.pa = (pa_block + control_size-1) & mask;
	control_target.va = (void*)((u32)(va_block + control_size-1) & mask);
	control_tbix = control_mem->tblock->iblock;

	*IOP321_IALR2 = mask;
	*IOP321_IATVR2 = control_target.pa;

	info("IATVR2:0x%08x IALR2:0x%08x", *IOP321_IATVR2, *IOP321_IALR2);
	fill_control_target();
}


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

		MK_FIFO_TO_LOCAL(pbc, ichain);
		MK_LOCAL_TO_LOCAL(pbc, ichain, control_pa(iblock));
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
	}else{
		init_endstops(INIT_ENDSTOPS);
		
	}
}

EXPORT_SYMBOL_GPL(control_block);
EXPORT_SYMBOL_GPL(control_numblocks);
