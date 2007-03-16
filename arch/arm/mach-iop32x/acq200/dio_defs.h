/* ------------------------------------------------------------------------- */
/* dio_defs.h - application control defs for dio                             */
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


#ifndef __DIO_DEFS_H__
#define __DIO_DEFS_H__

#define DIO_MASK_INPUT     '-'
#define DIO_MASK_OUTPUT1   '1'
#define DIO_MASK_OUTPUT0   '0'
#define DIO_MASK_INPUT0    'L'
#define DIO_MASK_INPUT1    'H'
#define DIO_MASK_OUTPUT_PP 'P'    /* positive pulse */
#define DIO_MASK_OUTPUT_NP 'N'    /* negative pulse */

#define DIO_MASK_INPUT_TOGGLE '|'


#endif
