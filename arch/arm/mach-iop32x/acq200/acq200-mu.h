/* ------------------------------------------------------------------------- */
/* acq200-mu.h acq200-mu driver exported symbols                             */
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


#ifndef __ACQ200_MU_H__
#define __ACQ200_MU_H__

#define MFA_MASK 0x00ffffff   /* converts MFA PA to hostmem offset */
#define QP_MASK  0x000fffff   /* converts QPA to Q mumem offset */

typedef u32 MFA;  /* Message Frame address (cookie) - offset in MUMEM */
                  /* MFA's are always NON zero */
typedef u32 RMA;  /* Remote Message address (cooke) - offset in PCI WIN */

u32* mfa2va(MFA mfa);
u32 mfa2pa(MFA mfa);
void* rma2va( RMA rma );
u32 mfa2pci(MFA mfa);
u32 rma2pci( RMA rma );

MFA acq200mu_get_free_ob(void);
int acq200mu_post_ob(MFA mfa);
MFA acq200mu_get_ib(void);
int acq200mu_return_free_ib(MFA mfa);

#endif /* __ACQ200_MU_H__ */
