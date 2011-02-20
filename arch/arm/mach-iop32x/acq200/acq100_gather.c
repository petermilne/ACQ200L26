/* ------------------------------------------------------------------------- */
/* acq100_gather.c gathers data from self and slave devices                  */
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


/** @file acq100_gather.c
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

#include <linux/proc_fs.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/iop321-dma.h>
#include "acq200-dmac.h"
#include "acq200_debug.h"

#include "acq200_hostdrv.h"

/** Globals .. keep to a minimum! */
char acq100_gather_driver_name[] = "acq200";
char acq100_gather_driver_string[] = "D-TACQ intelligent data acquisition device";
char acq100_gather_driver_version[] = "B1000";
char acq100_gather_copyright[] = "Copyright (c) 2011 D-TACQ Solutions Ltd";

int idx;
//DG->eoc_int_modulo_mask = 0;


extern int control_block;
extern int control_numblocks;

int acq100_gather_init_module(void)
{
	int is;

	info("%s\n%s\n%s\n%s", 
	     acq100_gather_driver_name, acq100_gather_driver_string,
	     acq100_gather_driver_version, acq100_gather_copyright);


	info("self: %08x block:%d numblocks:%d",
	     *IOP321_IATVR2, control_block, control_numblocks);

	for (is = 0; ; ++is){
		struct Acq200Device *device = acq200_devices[is];

		if (device == 0){
			break;
		}

		info("device %s pa: 0x%08lx len: %d",
		     device->ram.name, device->ram.pa, device->ram.len);

	}		
	return 0;
}

void acq100_gather_exit_module(void)
{
}


module_init(acq100_gather_init_module);
module_exit(acq100_gather_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("acq100_gather mechanism");
