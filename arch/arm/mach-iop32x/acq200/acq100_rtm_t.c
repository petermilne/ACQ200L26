/* ------------------------------------------------------------------------- */
/* acq100_rtm_t.c  - RTM-T adapter		                             */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
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


#include <linux/kernel.h>

#define ACQ196


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"
#include "acq200-fifo.h"

#include "acq100_rtm_t.h"

#define REVID	"acq100_rtm_t B1000"

static int __init acq100_rtm_t_init(void)
{
	struct resource mumem;

	info(REVID);
	acq200_get_mumem_resource(&mumem);
	DG->fpga.fifo.pa = virt_to_phys((void*)mumem.start);
	*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_DAC_RTM_T;
	info("set fifo.pa to %x, set SYSCON_DAC_RTM", DG->fpga.fifo.pa);
	return 0;
}

static void __exit
acq100_rtm_t_exit_module(void)
{
	DG->fpga.fifo.pa = ACQ200_FPGA_P+ACQ196_FIFO_OFFSET;
	*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_DAC_RTM_T;
	info("restore fifo.pa to %x", DG->fpga.fifo.pa);
}

module_init(acq100_rtm_t_init);
module_exit(acq100_rtm_t_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("ACQ100 RTM-T adapter");


