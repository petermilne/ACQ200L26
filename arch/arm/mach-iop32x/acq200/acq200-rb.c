/* ------------------------------------------------------------------------- */
/* acq200-rb.c generic ring buffer implementation                            */
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

/* @@todo - this include list is REDICULOUS */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#ifdef PGMCOMOUT263
#include <asm-arm/proc-armv/cache.h>
#endif
#include <linux/proc_fs.h>

#include "acq200.h"
#include "acq200_debug.h"
#include "acq200_minors.h"

#include "acq200-dmac.h"


void acq200_rb_clear_counters(struct acq200_dma_ring_buffer* rb)
{
	rb->iput = rb->iget = 0;
	rb->nput = rb->nget = 0;		
	rb->hitide = 0;
	rb->lotide = 0xffffU;
}

int acq200_rb_init(
	struct acq200_dma_ring_buffer* rb,
	int rblen /* MUST BE POWER OF 2 */
)
{
	int vmlen = rblen*sizeof(struct iop321_dma_desc *);
	rb->buffers = kmalloc(vmlen, GFP_KERNEL);

	if ( rb->buffers == 0 ){
		err( "kmalloc %d ) failed", vmlen );
		return -ENOMEM;
	}else{
		dbg(1, "rb->buffers %p, zero it len %d", rb->buffers, vmlen);
		memset(rb->buffers, 0, vmlen);
		dbg(1, "rb->buffers %p, zero done", rb->buffers);

		acq200_rb_clear_counters(rb);
		return 0;
	}
}



int acq200_free_rb( struct acq200_dma_ring_buffer *rb )
{
	dbg(1, "rb->buffers %p, kfree()", rb->buffers);
	kfree( rb->buffers );
	memset( rb, 0, sizeof(struct acq200_dma_ring_buffer *) );
	return 0;
}
