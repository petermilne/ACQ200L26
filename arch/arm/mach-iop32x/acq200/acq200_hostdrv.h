/* ------------------------------------------------------------------------- */
/* acq200_hostdrv.h skeleton pci driver for slave acq2xx                     */
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


/** @file acq200_hostdrv.h
 */



#define KNAME 32
#define MAXDEV 16



struct Acq200Device {
	int idx;				/**< card index in system. */

	struct pci_dev *pci_dev;		/**< linux pci generic.    */
	int card_type;
	char model_name[KNAME];
	
	struct LogicalDev {
		int major;
		char drv_name[KNAME];
	}
		ldev;

	struct IoMapping {
		u32* va;
		unsigned long pa;
		int len;
		char name[KNAME];
	}
		csr, ram, rom;
};


extern struct Acq200Device* acq200_devices[];

