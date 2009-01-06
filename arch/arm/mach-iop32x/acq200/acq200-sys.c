/* ------------------------------------------------------------------------- */
/* acq200-sys.h  - acq system cpld driver                                    */
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/pci.h>
#include <asm/mach-types.h>

#include <asm/arch/iop321.h>
#include <asm/arch/acq200.h>

#include <asm/setup.h>

#include "acq200_debug.h"

#include "acq200-sys.h"


#define ACQ196_CPLD_RST 2
#define ACQ196_CPLD_REV	3

/* byte regs are 32 bit aligned in ACQ132 */
#define ACQ132_CPLD_RST (2*4)
#define ACQ132_CPLD_REV (3*4)

#define ACQ1xx_CPLD_RST (machine_is_acq132()? ACQ132_CPLD_RST: ACQ196_CPLD_RST)
#define ACQ1xx_CPLD_REV (machine_is_acq132()? ACQ132_CPLD_REV: ACQ196_CPLD_REV)

#define CPLD(reg) *(volatile unsigned char*)(ACQ200_CPLD+ACQ1xx_CPLD_##reg)

u8 acq100_get_cpld_rev(void)
{
	return CPLD(REV) & 0xf;
}

u8 acq100_get_pci_env(void)
{
	if (ACQ100_CPLD_SSM_CAPABLE){
		return (CPLD(RST) >> 1) & 0x3;
        }else{
		return 0xff;
	}
}

#define ACQ200_SYSSLOT() ((*(volatile u8*)(ACQ200_CPLD+3) & 0x01) == 0)

int acq200_is_sysslot(void)
{
	if (machine_is_acq200()){
		return ACQ200_SYSSLOT();		
	}else{
		return acq100_get_pci_env() == ACQ100_PCIENV_SSM;
	}
}

void acq200_arch_reset(char mode)
{

	printk("acq200_arch_reset\n");
	CPLD(RST) = 1;
	printk("**** ERROR **** ALL DONE: now we should be dead\n");
}

