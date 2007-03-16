/* ------------------------------------------------------------------------- */
/* acq200-rb.h generic ring buffer implementation                            */
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


/*
 * WORKTODO: this is NOT as generic as ringbuffer.h
 * in fact this is specific to rb shared with FIQ. ho hum
 */

void acq200_rb_clear_counters(struct acq200_dma_ring_buffer* rb);
void acq200_rb_drain(struct acq200_dma_ring_buffer* rb);
int acq200_rb_init(
	struct acq200_dma_ring_buffer* rb,
	int rblen /* MUST BE POWER OF 2 */
	);
