/** customise for acq196t - Thomson 4 sample special ..*/

static void init_endstops_control_target(void);
static void init_control_target(void);


#define DTACQ_MACH_DRIVER_INIT(dev) init_control_target()

#define ACQ196T
#include "acq196-fifo.c"

int control_wanted = 0;
module_param(control_wanted, int, 0644);

int control_size = 0x00010000;
module_param(control_size, int, 0644);

int control_block = 0x00000400;
module_param(control_block, int, 0644);

int control_tbix = -1;
module_param(control_tbix, int, 0444);

static struct pci_mapping control_target;


static void fill_control_target(void)
{
	int ib;
	unsigned off;
	void* va = control_target.va;

	info("len:%d pa:0x%08x va:%p", 
	     control_target.len, control_target.pa, control_target.va);

	for (ib = off = 0; off < control_size; ib++, off += control_block){
		memset(va + off, ib, control_block);
	}
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


static void init_endstops_control_target(void)
/** init endstops, including control_target chains */
{
#if 0
	struct CONTROL_TARGET* ct = &DMC_WO->control_target;

	int modulo = max(ct->stride, ct->data_blocks);
	int numstops = 1024/2;    /** 50msec buffer */
	int istop = 0;
	int iblock = 0;

	while((numstops % modulo) != 0){
		++numstops;
	}

	numstops *= 2;           /** guarantee 2 buffer op returns to start */

	dbg(1, "stride %d blocks %d modulo %d numstops %d",
	    ct->stride, ct->data_blocks, modulo, numstops);

	init_endstops(numstops);

	/** now build numstops chains
         *  - 0..data_blocks with increasing target offset
         *  - data_blocks..stride : unchanged.
	 */

	if (numstops > S_pbstore.count){
		if (S_pbstore.chains){
			kfree(S_pbstore.chains);
		}
		S_pbstore.chains = kmalloc(numstops*PBC_SZ, GFP_KERNEL);
		if (!S_pbstore.chains){
			err("FAILED to allocate PBC %d", numstops*PBC_SZ);
			return;
		}
		S_pbstore.count = numstops;
	}

	nsample = 0;

	for (nsample = 0; istop < numstops; nsample++){
		for (iblock = 0; iblock < modulo; ++iblock, ++istop){
			struct PrebuiltChain *pbc = S_pbstore.chains+istop;
			struct iop321_dma_desc* endstop;
			int ichain = 0;

			memset(pbc, 0, sizeof(struct PrebuiltChain));

			dbg(3, "istop:%3d iblock:%d", istop, iblock);

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
			
			if (iblock < ct->data_blocks){
				if (iblock == ct->data_blocks - 1){
					MK_CAPCOM_TO_LOCAL(pbc, ichain);
					MK_CAPCOM_TEST(pbc, ichain);
				}
				MK_FIFO_TO_LOCAL(pbc, ichain);
				if (LOCAL_TO_HOST_ENABLED){
					MK_LOCAL_TO_HOST(pbc, ichain, iblock);
					pbc->insert = 
						prebuilt_insert_local_host;
				}else{
					pbc->insert=
					 prebuilt_insert_local_host_nodata;
				}
				if (CAPCOM_TO_HOST_ENABLED &&
				    iblock == ct->data_blocks - 1){
					MK_CAPCOM_TO_HOST(pbc, ichain);
				}				
				if (ct->iodd){
					MK_IODD(pbx, ichain);
				}

			}else{
				MK_FIFO_TO_LOCAL(pbc, ichain);
				pbc->insert = prebuilt_insert_local;
			}
			MK_ENDSTOP(pbc, ichain, endstop);
			pbc->length = ichain;

			rb_put(&IPC->endstops, &pbc->desc);
		}
	}

	DG->put_max_empties = numstops/2;
	DG->empty_fill_threshold = numstops/2;
#else
	if (control_wanted){
		;
	}else{
		init_endstops(INIT_ENDSTOPS);
	}
#endif
}

/*
	if (DMC_WO->control_target.pa_data || 
	    DMC_WO->control_target.pa_status ){
#if ISR_ADDS_ENDSTOP 
		err("ISR_ADDS_ENDSTOP set - can't do this, revert to regular");
		init_endstops(INIT_ENDSTOPS);
#else
		init_endstops_control_target();
#endif
		DMC_WO->handleEmpties = dmc_handle_empties_prebuilt;
	}else{
		init_endstops(INIT_ENDSTOPS);
		DMC_WO->handleEmpties = dmc_handle_empties_default;
	}
*/
