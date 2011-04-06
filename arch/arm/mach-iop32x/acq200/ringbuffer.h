/* ------------------------------------------------------------------------- */
/* ringbuffer.h generic ring buffer implementation                           */
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


#ifndef __U32_RINGBUFFER_H__
#define __U32_RINGBUFFER_H__ 1

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/iop321-dma.h>
#include "acq200-dmac.h"

struct u32_ringbuffer {
	unsigned short iput;
	unsigned short iget;
	int nput;
	int nget;
	u32 *buffer;      /* -> contiguous vmem block */
	unsigned short hitide;
	unsigned short lotide;
	int nbuf;               /* must be power of 2 */
	int noput;
};

static inline int u32rb_printf(char *buf, struct u32_ringbuffer* rb){
#define RB_PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
	int len = 0;
	
	len += RB_PRINTF("%4s %4s %8s %8s  %4s %4s %4s %4s\n",
			 "iput", "iget", "nput", "nget", "hi", "lo",
			 "nbuf", "noput");

	len += RB_PRINTF("%4d %4d %8d %8d  %4d %4d %4d %4d\n",
			 rb->iput, rb->iget, rb->nput, rb->nget,
			 rb->hitide, rb->lotide, rb->nbuf, rb->noput);
	 return len;
#undef RB_PRINTF	
}
static inline int u32rb_incr(struct u32_ringbuffer* rb, int i){
	return (i+1)&(rb->nbuf-1);
}
static inline int u32rb_is_full(struct u32_ringbuffer* rb){
	return u32rb_incr(rb, rb->iput) == rb->iget;
}
static inline int u32rb_is_empty(struct u32_ringbuffer* rb){
	return rb->iput == rb->iget;
}
static inline int u32rb_put(struct u32_ringbuffer* rb, u32 x)
/* returns 1 on SUCCESS */
{
	if (!u32rb_is_full(rb)){
		rb->buffer[rb->iput] = x;
		rb->iput = u32rb_incr(rb, rb->iput);
		rb->nput++;
		return 1;
	}else{
		rb->noput++;
		return 0;
	}
}
static inline int u32rb_get(struct u32_ringbuffer* rb, u32 *px) 
/* returns > 0 on data, 0 on no data */
{
	if (!u32rb_is_empty(rb)){
		u32 x = rb->buffer[rb->iget];
		rb->iget = u32rb_incr(rb, rb->iget);
		rb->nget++;
		*px = x;
		return 1;
	}else{
		return 0;
	}
}

static inline int u32rb_peek(struct u32_ringbuffer* rb, u32 *px) 
/* returns > 0 on data, 0 on no data */
{
	if (!u32rb_is_empty(rb)){
		u32 x = rb->buffer[rb->iget];
		*px = x;
		return 1;
	}else{
		return 0;
	}
}


static inline int u32rb_get_user(struct u32_ringbuffer* rb, u32 *px) 
/* returns > 0 on data, 0 on no data */
{
	if (!u32rb_is_empty(rb)){
		if (copy_to_user(px, rb->buffer+rb->iget, sizeof(u32))){
			return -EFAULT;
		}
		rb->iget = u32rb_incr(rb, rb->iget);
		rb->nget++;
		return 1;
	}else{
		return 0;
	}
}
static inline void u32rb_init(struct u32_ringbuffer* rb, int nbuf)
{
	int _nbuf;
/* force power of 2 sizing */
	for (_nbuf = 2; _nbuf < nbuf; _nbuf <<=1 ){
		;
	}
	rb->buffer = kmalloc(_nbuf*sizeof(u32), GFP_KERNEL);
	assert(rb->buffer);

	rb->nbuf = _nbuf;
	rb->iget = rb->iput = 0;
	rb->nget = rb->nput = 0;
}

static inline void u32rb_destroy(struct u32_ringbuffer* rb)
{
	rb->iput = rb->iget;
	kfree(rb->buffer);
}


#define RBMASK         (RBLEN-1)
#define RB_IS_EMPTY( rb ) ((rb).iput==(rb).iget)
#define RB_INCR( ii )  (((ii)+1)&RBMASK)
#define RB_DECR( ii )  ((ii)-1 >= 0? (ii)-1: RBMASK)
#define RB_IS_FULL( rb )  (RB_INCR((rb).iput)==(rb).iget)

#define RB_WILL_BE_EMPTY(rb) ((rb).iput==RB_INCR((rb).iget))

/* next is an approximation - nput, nget will overflow */
#define RB_ELEMENT_COUNT(rb) ((rb).nput - (rb).nget)

#define RBLEN          0x4000


#define CHECK_TIDES(rb)					\
do {							\
	unsigned short tide = rb->nput - rb->nget;	\
	if (tide > rb->hitide){				\
		rb->hitide = tide;			\
	}else if (tide < rb->lotide){			\
		rb->lotide = tide;			\
	}						\
} while(0)

#define INIT_TIDES(rb)					\
do {							\
	rb->hitide = rb->lotide = rb->nput - rb->nget;	\
} while(0)

static inline int rb_put( 
	struct acq200_dma_ring_buffer *rb, 
	struct iop321_dma_desc *buf )
{
	if ( !RB_IS_FULL( *rb ) ){
		rb->buffers[rb->iput] = buf;
		rb->iput = RB_INCR(rb->iput);
		rb->nput++;
		CHECK_TIDES(rb);
		return 1;
	}else{
		rb->lotide = 0;
		return 0;
	}
}


static inline int rb_get( 
	struct acq200_dma_ring_buffer* rb, 
	struct iop321_dma_desc** pbuf )
{
	if ( !RB_IS_EMPTY( *rb ) ){
		CHECK_TIDES(rb);
		*pbuf = rb->buffers[rb->iget];
		rb->iget = RB_INCR( rb->iget );
		rb->nget++;
		return 1;
	}else{
		rb->lotide = 0;
		return 0;
	}
}


static inline struct iop321_dma_desc* rb_get_buf(
	struct acq200_dma_ring_buffer* rb
	)
{
	struct iop321_dma_desc* pbuf = 0;

	if ( !RB_IS_EMPTY( *rb ) ){
		CHECK_TIDES(rb);
		pbuf = rb->buffers[rb->iget];
		rb->iget = RB_INCR( rb->iget );
		rb->nget++;
	}else{
		rb->lotide = 0;
	}
	
	return pbuf;
}


#endif /* __U32_RINGBUFFER_H__ */
