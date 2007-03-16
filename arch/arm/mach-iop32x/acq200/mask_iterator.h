/* ------------------------------------------------------------------------- */
/* mask_iterator.h mask iterator class def                                   */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __MASK_ITERATOR__
#define __MASK_ITERATOR__

struct MASK_ITERATOR {
    unsigned mask;
    unsigned icursor;
    enum state { MI_IDLE, MI_STARTED, MI_FINISHED } state;
};

static inline int mit_hasNext( struct MASK_ITERATOR* it ) {
    unsigned next_cursor = 0;

    switch( it->state ){
    case MI_IDLE:
	next_cursor = 0x80000000;
	it->icursor = 31;
	break;
    case MI_STARTED:
	next_cursor = (1 << it->icursor) >> 1;
	break;
    case MI_FINISHED:
	return 0;
    }
    for ( ; next_cursor != 0; next_cursor >>= 1 ){
	if ( it->mask & next_cursor ){
	    return 1;
	}
    }
    
    return 0;
}
static inline int mit_getNext( struct MASK_ITERATOR* it ) {
    unsigned next_cursor = 0;

    switch( it->state ){
    case MI_IDLE:
	next_cursor = 0x80000000;
	it->state = MI_STARTED;
	it->icursor = 31;
	break;
    case MI_STARTED:
	next_cursor = 1 << --it->icursor;
	break;
    case MI_FINISHED:
	return -1;
    }
    for ( ; next_cursor != 0; next_cursor >>= 1, --it->icursor ){
	if ( it->mask & next_cursor ){
	    return it->icursor;
	}
    }
    
    it->state = MI_FINISHED;
    return -1;
}

#define MASK_ITERATOR_INIT(name, mask) struct MASK_ITERATOR name = { mask }

#endif /* __MASK_ITERATOR__ */
