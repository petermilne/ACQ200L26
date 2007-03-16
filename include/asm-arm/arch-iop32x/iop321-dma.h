/* ------------------------------------------------------------------------- */
/* iop321-dma.h - Intel 80321 dma controller bit defs                        */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */


#ifndef _IOP321_DMA_H_
#define _IOP321_DMA_H_

#define IOP321_CCR_CE     0x00000001          /* Channel Enable              */
#define IOP321_CCR_CR     0x00000002          /* Chain Resume                */

#define IOP321_CSR_ERR    0x0000003F          /* Error Mask                  */
#define IOP321_CSR_PXSTE  0x00000002          /* PCI-X Split Transact Error  */
#define IOP321_CSR_PCITAD 0x00000004          /* PCI Target Abort Flag       */
#define IOP321_CSR_PCIMAF 0x00000008          /* PCI Master Abort Flag       */
#define IOP321_CSR_IBMAF  0x00000020          /* Internal Bus Master Abrt Flg*/

#define IOP321_CSR_EOCIF  0x00000100          /* End of Chain Interrupt Flag */
#define IOP321_CSR_EOTIF  0x00000200          /* End of Transfer Int Flag    */
#define IOP321_CSR_CAF    0x00000400          /* Channel Active Flag         */

#define IOP321_DCR_MMTE   0x00000040          /* Mem to Mem Transfer Enable  */
#define IOP321_DCR_DAC    0x00000020          /* dual address enable         */
#define IOP321_DCR_IE     0x00000010          /* enable interrupt            */
#define IOP321_DCR_PCITR  0x0000000F          /* PCI Transaction, one of: */

/* copy from dma.h, thanks MV */
#define DMA_DCR_PCI_IOR		0x00000002	/* I/O Read */
#define DMA_DCR_PCI_IOW		0x00000003	/* I/O Write */
#define DMA_DCR_PCI_MR		0x00000006	/* Memory Read */
#define DMA_DCR_PCI_MW		0x00000007	/* Memory Write */
#define DMA_DCR_PCI_CR		0x0000000A	/* Configuration Read */
#define DMA_DCR_PCI_CW		0x0000000B	/* Configuration Write */
#define DMA_DCR_PCI_MRM		0x0000000C	/* Memory Read Multiple */
#define DMA_DCR_PCI_MRL		0x0000000E	/* Memory Read Line */
#define DMA_DCR_PCI_MWI		0x0000000F	/* Mem Write and Inval */
     
/* Register offsets from base */

#define DMA_CCR                0x00
#define DMA_CSR                0x04
#define DMA_DAR                0x0C
#define DMA_NDAR               0x10
#define DMA_PADR               0x14
#define DMA_PUADR              0x18
#define DMA_LADR               0x1C
#define DMA_BCR                0x20
#define DMA_DCR                0x24

#define DMA_CHANNEL_SW         0x40             /* toggle DMA0/DMA1 */

/* chain offsets from base */

/* this loads to the hardware */

#define DMA_DESCR_NDA         0x00
#define DMA_DESCR_PAD         0x04
#define DMA_DESCR_PUAD        0x08
#define DMA_DESCR_LAD         0x0C
#define DMA_DESCR_BC          0x10
#define DMA_DESCR_DC          0x14

/* this allows the software to coordinate the chain */

#define DMA_DESCR_SW_PA       0x18
#define DMA_DESCR_SW_VA       0x1c    /* consistent mapping VA */


#define DMA_DESCR_SIZE        0x20

#define DMA_DESCR_SW_FLAGS_ID  0xdd00
#define DMA_DESCR_SW_FLAGS_END 0x0001

/*
 * fiq register holds DMAC between FIQ's
 * preload from linux-side
 */
#define ACQ200_DMA_FIQ_ENABLE r8
#define ACQ200_DMA_FIQ_DMAC   r12
#define ACQ200_DMA_FIQ_CHAIN  r11

#define ARM_r11 uregs[11]
#define ARM_r12 uregs[12]

#ifndef __ASSEMBLER__

#define __set_regs( regs, rx, value ) (regs.ARM_##rx = value)
#define ACQ200_DMA_FIQ_SET_DMAC( regs ) \
    __set_regs( regs, r12 /*ACQ200_DMA_FIQ_DMAC*/, (u32)IOP321_DMA0_CCR )
#define ACQ200_DMA_FIQ_SET_CHAIN( regs, chain ) \
    __set_regs( regs, r11 /*ACQ200_DMA_FIQ_CHAIN*/, (u32)chain )
#define ACQ200_DMA_FIQ_SET_ENABLE( regs ) \
    __set_regs( regs, r8/*ACQ200_DMA_FIQ_ENABLE*/, (u32)IOP321_CCR_CE )



#endif


#endif /* _IOP321_DMA_H_ */
