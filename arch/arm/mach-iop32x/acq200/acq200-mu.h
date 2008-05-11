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



/*
 * Message Buffer
 */

/** Host Buffer LENgth - host buffer size in host slave memory
 *  Host Buffer BLOCK  - basic size division
 *
 */

/* will work Linux 24 as well ... 4MB dynamic buffer */
#define _HBLEN26	 0x0400000
#define _HBBLOCK26       0x0080000
/* V1.0 scheme - relied on 16MB BIGBUF host buffer. BUGBUF!! */
#define _HBLEN24	 0x1000000
#define _HBBLOCK24       0x0100000

extern int HBLEN;
extern int HBBLOCK;

#define NUM_HBBLOCKS  (HBLEN/HBBLOCK)

#define MSGQBASE      ((NUM_HBBLOCKS-1)*HBBLOCK)
#define MSQQLEN       HBBLOCK
#define MSQSIZE       0x400
#define NMSGS         (MSQQLEN/MSQSIZE)
#define NIBMSGS       (NMSGS/2)
#define NOBMSGS       (NMSGS/2)

#define MSGQBASE_IN   MSGQBASE
#define MSGQBASE_OUT  (MSGQBASE+NIBMSGS*MSQSIZE)

#define ACQ200_MU_IFQ_ENTRY(n)  


#define NDATABUFS (NUM_HBBLOCKS-1)
#define DATABUFSZ HBBLOCK
#define DATABUFLN (DATABUFSZ/sizeof(u32))


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

unsigned acq200mu_get_hb_phys(void);
unsigned acq200mu_get_hb_len(void);

unsigned acq200mu_get_hb_mask(void) {
	return ~(acq200mu_get_hb_len() - 1);
}

#define __ACQ200_MU_Q_OFF(ix) (ACQ200_MU_QSZ*(ix))

#define ACQ200_MU_IFQ_FIRST __ACQ200_MU_Q_OFF(0)
#define ACQ200_MU_IPQ_FIRST __ACQ200_MU_Q_OFF(1)
#define ACQ200_MU_OPQ_FIRST __ACQ200_MU_Q_OFF(2)
#define ACQ200_MU_OFQ_FIRST __ACQ200_MU_Q_OFF(3)

#define __ACQ200_MU_Q_LAST(ix) (__ACQ200_MU_Q_OFF((ix)+1)-ACQ200_MU_QINC)

#define ACQ200_MU_IFQ_LAST  __ACQ200_MU_Q_LAST(0)
#define ACQ200_MU_IPQ_LAST  __ACQ200_MU_Q_LAST(1)
#define ACQ200_MU_OPQ_LAST  __ACQ200_MU_Q_LAST(2)
#define ACQ200_MU_OFQ_LAST  __ACQ200_MU_Q_LAST(3)

#define ACQ200_MU_ENTRY(n) (((n)-1)*ACQ200_MU_QINC)

#endif /* __ACQ200_MU_H__ */
