/* ------------------------------------------------------------------------- */
/* rtm-t.h RTM-T PCIe register defs					     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
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


/** @file rtm-t-hostdrv.c D-TACQ RTM-T PCIe regdefs */

#ifndef __RTM_T_H__
#define __RTM_T_H__


// Defines the Vendor ID.  Must be changed if core generated did not set the Vendor ID to the same value
#define PCI_VENDOR_ID_XILINX      0x10ee

// Defines the Device ID.  Must be changed if core generated did not set the Device ID to the same value
#define PCI_DEVICE_ID_XILINX_PCIE 0x0007


#define PCI_SUBVID_DTACQ	0xd1ac
#define PCI_SUBDID_RTMT		0x4000
#define PCI_SUBDID_FHBA		0x4100

/* REGISTERS = offset from BAR0 */
/* NEW: from PRM REV 3 */


/* DUAL CR - regs are mirrored either end, 
 * Writers should assert WANT_CTRL, check HAVE_CTRL.
 * but only H or Q is in charge at any time
 */
#define RTMT_D_TEST		0x20
#define RTMT_D_FCR		0x24
#define RTMT_D_DIO		0x28

/* _H_ HOST: */

#define RTMT_H_PCIE_CSR		0x00
#define RTMT_H_PCIE_CYCLE	0x04
#define RTMT_H_DEBUG		0x08
#define RTMT_H_INT		0x0c
#define RTMT_H_DMACSTA		0x10
#define RTMT_H_DESC_FIFSTA	0x14
#define RTMT_H_TEST_OLD		0x18
#define RTMT_H_DEBUG_COUNTER	0x1c

#define RTMT_H_TEST		RTMT_D_TEST
#define RTMT_H_FCR		RTMT_D_FCR
#define RTMT_H_DIO		RTMT_D_DIO

#define RTMT_H_MBOX1		0x38
#define RTMT_H_MBOX2		0x3c

/* _C_ COMMON: */

#define RTMT_C_REVID		0x40
#define RTMT_C_DATA_FIFSTA	0x44	/* HOST BUFFER FIFO ?? */
#define RTMT_C_PCI		0x48	/* NOT DOCUMENTED */
#define RTMT_C_PCI_CSR		0x48	/* NOT DOCUMENTED */
#define RTMT_C_PCIe_DEV_CSR	0x4c	/* NOT DOCUMENTED */
#define RTMT_C_PCIe_LNK_CSR	0x50	/* NOT DOCUMENTED */

/* _Q_ ACQ (IOP) Control Regs */

#define RTMT_Q_CSR		0x80
#define RTMT_Q_SPI		0x84
#define RTMT_Q_TEST		(0x80+RTMT_D_TEST)
#define RTMT_Q_FCR		(0x80+RTMT_D_FCR)
#define RTMT_Q_DIO		(0x80+RTMT_D_DIO)

#define RTMT_Q_SPI_CTL		0xac
#define RTMT_Q_SPI_DAT		0xb0
#define RTMT_Q_MBOX1		0xb8
#define RTMT_Q_MBOX2		0xbc
#define	RTMT_Q_END		RTMT_Q_MBOX2
#define REGS_LEN	        (0x80+RTMT_Q_END+4)

#define RTMT_H_PCIE_CSR_FPGA_FAMILY	0xff000000
#define RTMT_H_PCIE_CSR_INT_TYPE	0x00070000
#define RTMT_H_PCIE_CSR_VERSION		0x0000ff00
#define RTMT_H_PCIE_CSR_DMA_RESET	0x00000001	/* RW: 1: reset */

#define RTDMAC_RESET		RTMT_H_PCIE_CSR
#define RTDMAC_RESET_RBIT	RTMT_H_PCIE_CSR_DMA_RESET

#define RTMT_H_PCIE_CYCLE_MWR_ENABLE	0x00000001

/* back compatibility ... and the names are shorter as well .. */

#define RTDMAC_TEST		RTMT_H_TEST_OLD
#define TEST_REG		RTMT_H_TEST_OLD
#define RTDMAC_CTRL		RTMT_H_PCIE_CYCLE
#define RTDMAC_CTRL_START	RTMT_H_PCIE_CYCLE_MWR_ENABLE

#define RTDMAC_CTRL_TEST_MODE	(1<<28)		/* RW simulate if set   */
#define RTDMAC_CTRL_TEST_THROT_SHL	24	/* throttle */
#define RTDMAC_CTRL_TEST_THROT_FULL	1
#define RTDMAC_CTRL_TEST_THROT_MIN	15




#define RTMT_D_FCR_HASIT	(1<<31)		/* RO 1: you have control */
/* unused 29-24 */
#define RTMT_D_FCR_PCIE_APP_RDY (1<<23)		/* RO 1: cable plugged in */
/* unused 23-16 */
#define RTMT_D_FCR_WANTIT	(1<<15)		/* 1: request control */
#define RTMT_D_FCR_SIM_THROTTLE_SHL 8
#define RTMT_D_FCR_SIM_THROTTLE_MSK 0xf		/* 0x0 : /1, 0x1 : /2 .. etc */
#define RTMT_D_FCR_SIM_MODE	(1<<7)		/* 1: Simulate on source=ctr */
#define RTMT_D_FCR_PCIE_DATAON	(1<<6)		/* 1: request data frm ACQ */
#define RTMT_D_FCR_LOWLAT	(1<<5)
/* unused 4-1 */
#define RTMT_D_FCR_DMA_SINGLE_RECYCLE	(1<<1)	/* 1: reuse #1 DMA descr forever */
#define RTMT_D_FCR_FIFO_RESET	(1<<0)		/* 1: reset COMMS FIFO Which One? */

#define RTMT_D_FCR_SIM_CLR	\
		(RTMT_D_FCR_SIM_MODE|RTMT_D_FCR_SIM_THROTTLE_MSK<<RTMT_D_FCR_SIM_THROTTLE_SHL)

#define RTMT_D_DIO_DRVON_SHL	24	/* 1: HD15 is OUTPUT */
#define RTMT_D_DIO_SETOUT_SHL	16	/* 1: FPGA is OUTPUT */
#define RTMT_D_DIO_OUTDAT_SHL	 8	/* output value set (when output) */
#define RTMT_D_DIO_INPDAT_SHL	 0

#define RTMT_DIO_MASK		0x00ff	/* 8 bits DIO */
#define MAXDIOBIT		8







/* 31-26 not used */
#define RTMT_Q_CSR_PCIe_RSTn		(1<<25)	/* RO 0: RESET active */
#define RTMT_Q_CSR_PCIe_PRSNT		(1<<24) /* RO 1: CABLE plugged in */
/* 23-18 not used */
#define RTMT_Q_CSR_PG_MGT		(1<<17)	/* RO 1: MGT POWER GOOD */
#define RTMT_Q_CSR_PG_DDR3		(1<<16)	/* RO 1: DDR POWER GOOD */
/* 15-10 not used */
#define RTMT_Q_CSR_PCIe_CABLE_WAKE	(1<<9)	/* QW: wake cable request */
#define RTMT_Q_CSR_PCIe_DRVR_PRSNT	(1<<8)	/* QW IWHAT? */
/* 7-1 not used */
#define RTMT_Q_CSR_BURST		(1<<0)	/* QW PBI Turbo mode */


#define RTMT_XXXX_COUNT_MASK		0xfff0
#define RTMT_XXXX_COUNT_SHL		4
#define RTMT_XXXX_FIFSTA_OVERFLOW	(1<<2)
#define RTMT_XXXX_FIFSTA_EMPTY		(1<<1)

#define RTMT_Q_SPI_BUSY			(1<<31)
#define RTMT_Q_SPI_CTL_START		(1<<7)
#define RTMT_Q_SPI_CS			(1<<0)
#define RTMT_Q_SPI_HOLD			(1<<1)
#define RTMT_Q_SPI_WP			(1<<2)

#define UART_BAR	4
#define TX_FIFO_BAR	2
#define REGS_BAR	0
#define NO_BAR		-1
#define MAP_COUNT	5

/* we wanted an elegant multi-vector solution, but we can only have one int */
#define MSI_DMA		0
#ifdef MSI_BLOCK_WORKS
#define MSI_UART	1
#else
#define MSI_UART	0
#endif

#define RTDMAC_PAGE		0x400		/* 1K pages */
#define RTDMAC_DESC_ADDR_MASK	0xfffffc00	/* base address */

#define RTDMAC_DESC_WRITE	0x00000200	/* Write BIT */
#define RTDMAC_DESC_EOT		0x00000100	/* End Of Transfer interrupt */

#define RTDMAC_DESC_LEN_MASK	0x000000f0	/* length (pages) */
#define RTDMAC_DESC_LEN_SHL	4

#define RTDMAC_DESC_ID		0x0000000f	/* ID 0..15 */

#define EMPTY1	0xee11ee11
#define EMPTY2  0x22ee22ee

#define MAGIC1	0xdeadbeef
#define MAGIC2	0xcafebabe

#define RTDMAC_DATA_FIFO_CNT	0x1000
#define RTDMAC_DESC_FIFO_CNT	0x1000

#define DATA_FIFO_SZ	(RTDMAC_DATA_FIFO_CNT*sizeof(unsigned))
#define DESC_FIFO_SZ	(RTDMAC_DESC_FIFO_CNT*sizeof(unsigned))

#define UART_LEN	512

/* it's actually double, but then we'd have to update stty */
#define RTM_T_UART_XTAL	(14181800)


#define FPGA_ID	0		/* S6 unique ID TBA */


/* LLC SIGNALING */


#define H_MBOX1_LLC_CMD		0xffff0000
#define H_MBOX1_LLC_RUN		0x11c10000
#define H_MBOX1_LLC_STOP	0x11c00000

#define Q_MBOX1_LLC_ACK		0xFFFF0000
#define Q_MBOX1_LLC_SHOT	0x0000ffff

#define __MAKE_LLC_CMD(cmd, clkdiv, clk_pos, trg_pos) \
	((cmd) | ((clkdiv)&0x00ff) |\
	 ((clk_pos)? 0x00008000:0) | ((trg_pos)? 0x00004000:0))

#define MAKE_LLC_RUN(clkdiv, clk_pos, trg_pos) \
		__MAKE_LLC_CMD(H_MBOX1_LLC_RUN, clkdiv, clk_pos, trg_pos)

#define DECODE_LLC_CMD(mbox, clkdiv, clk_pos, trg_pos) \
	do { \
		clkdiv = (mbox)&0x00ff; \
		clk_pos = ((mbox)&0x00008000) != 0; \
		trg_pos = ((mbox)&0x00004000) != 0; \
	} while(0)

#define IS_LLC_RUN(mbox)	(((mbox)&H_MBOX1_LLC_CMD)==H_MBOX1_LLC_RUN)
#define IS_LLC_STOP(mbox)	(((mbox)&H_MBOX1_LLC_CMD)==H_MBOX1_LLC_STOP)

/*  Q_MBOX2 is LLC_TLATCH */

#endif /* __RTM_T_H__ */




