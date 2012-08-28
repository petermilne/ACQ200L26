/** customiser for acq196c - UGLEEE - there MUST be a better way ..*/

static void init_endstops_ll(int count);


#define DO_INIT_ENDSTOPS(count) init_endstops_ll(count)

#define ACQ196C
#include "acq196-fifo.c"

int set_lowlat = 0;
module_param(set_lowlat, int, 0644);

static void init_endstops_ll(int count)
/** init endstops, including control_target chains */
{
	if (set_lowlat){
		acq196_syscon_set_all(ACQ196_SYSCON_LOWLAT);
	}else{
		acq196_syscon_clr_all(ACQ196_SYSCON_LOWLAT);
	}
	init_endstops(count);
}
