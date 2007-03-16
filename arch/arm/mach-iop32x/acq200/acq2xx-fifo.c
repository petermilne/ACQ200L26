#define DTACQ_MACH 1
#define ACQ216 

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq216.h"

void acq200_reset_fifo(void)
{
	u32 before, during;

	*ACQ200_FIFCON = 0;
	before = *ACQ200_FIFCON;

	*ACQ200_FIFCON |= ACQ200_FIFCON_HC_RESET;
	during = *ACQ200_FIFCON;
	*ACQ200_FIFCON &= ~ACQ200_FIFCON_HC_RESET;

	dbg(1, "BEFORE: 0x%08x DURING: 0x%08x AFTER :0x%08x", 
	     before, during, *ACQ200_FIFCON);
}

