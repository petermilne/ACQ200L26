/* ------------------------------------------------------------------------- */
/* acq200-sys.h  - acq system cpld interface                                 */
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

extern u8 acq100_get_cpld_rev(void);

#define ACQ100_CPLD_REV_SSM 2


extern u8 acq100_get_pci_env(void);

#define ACQ100_PCIENV_SSM 0           /** system slot master */
#define ACQ100_PCIENV_PM  1           /** peripheral mode    */
#define ACQ100_PCIENV_SAM 2           /** standalone mode    */

extern void acq200_set_cpld_mask_byte(u8 mask);

extern int acq200_is_sysslot(void);
