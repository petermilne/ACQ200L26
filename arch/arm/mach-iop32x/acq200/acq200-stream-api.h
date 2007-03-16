/* ------------------------------------------------------------------------- */
/* acq200-stream-api.h app shared defs for streaming                         */
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


#ifndef __ACQ200_STREAM_API_H__
#define __ACQ200_STREAM_API_H__

/*
 * u32 offsets into ESIG
 */

#ifndef NSUBFRAMES   
#define NSUBFRAMES 0x40
/* @@todo = should really have access to def in acq32busprot.h */
#endif

#define ESIG_ESIG       0x0       /* event signature */
#define ESIG_JIFFIES    0x2       /* jiffies */
#define ESIG_SCC        0x3       /* sample clock count */
#define ESIG_BURST_LEN  0x4      
#define ESIG_BURST_DLY  0x5
#define ESIG_E2_OFFSET  0x6
#define ESIG_DIO        0x7
#define ESIG_EDIO       0x8
#define ESIG_FLAGS      0x9
#define ESIG_MFN        0xa       /* Multi Frame Number */
#define ESIG_TVS        0xb       /* Time of Day secs */
#define ESIG_TVUS       0xc       /* Time of Day usecs */
#define ESIG_ESIG2      0xd

#define ESIG_LAST       0xe

#define ESIG_LEN        (ESIG_LAST * sizeof(u32))  /* this is the sig len */
#endif /* ACQ200_STREAM_API_H__ */
