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
#endif /* __U32_RINGBUFFER_H__ */
