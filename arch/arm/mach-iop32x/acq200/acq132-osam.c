/* ------------------------------------------------------------------------- */
/* acq132-osam.c		                                             */
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

#define DTACQ_MACH 1
#define ACQ132
#include <linux/seq_file.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq132.h"

static int _decim;

int get_acq132_decim(void)
{
	return _decim == 0 ? 1: _decim;
}

static inline u32 acq132_adc_set_osam(u32 osam, int shl, int nacc)
{
	u32 field = 0;
	osam &= ~(OSAMBITS << shl);

	nacc = max(1, nacc);
	nacc = min(nacc, 16);

	field = (nacc-1)&OSAMBITS;

	osam |= field << shl;

	return osam;
}




static inline u32 acq132_adc_set_shift(u32 osam, int shl, int shift)
{
	u32 field = 0;
	osam &= ~(SHIFTBITS << shl);

	switch(shift){
	case -2:
	default:
		field = SHIFT_M2; break;
	case -1:
		field = SHIFT_M1; break;
	case 1:
		field = SHIFT_P1; break;
	case 2:
		field = SHIFT_P2; break;
	case 0:
		field = SHIFT_0; break;
	}

	osam |= field << shl;
	return osam;
}

void acq132_set_osam_nacc(
	int dev, int lr, int nacc, int shift, int decimate)
{
	u32 osam = *ACQ132_ADC_OSAM(dev);				
	u32 osam1 = osam;						
	osam = acq132_adc_set_osam(osam, ACQ132_ADC_OSAM_R_NACC+lr, nacc);
	osam = acq132_adc_set_shift(osam, ACQ132_ADC_OSAM_R_SHIFT+lr, shift);
	if (decimate){
		osam &= ~(1<<(ACQ132_ADC_OSAM_R_ACCEN+lr));
	}else{
		osam |= 1<<(ACQ132_ADC_OSAM_R_ACCEN+lr);
	}
	*ACQ132_ADC_OSAM(dev) = osam;
	dbg(1, "%p was: 0x%08x set 0x%08x reads 0x%08x ",
	    ACQ132_ADC_OSAM(dev), osam1, osam, *ACQ132_ADC_OSAM(dev));

	_decim = nacc;
}

void acq132_setAllDecimate(int dec)
{
#define DECIM	1
	int block;

	if (dec < 1) dec = 1;
	if (dec > 16) dec = 16;

	dbg(1, "setting decimation to %d", dec);

	for (block = 0; block <= 3; ++block){
		acq132_set_osam_nacc(block, OSAMLR('L'), dec, -2, DECIM);
		acq132_set_osam_nacc(block, OSAMLR('R'), dec, -2, DECIM);
	}
}	

