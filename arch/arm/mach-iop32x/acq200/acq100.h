/* ------------------------------------------------------------------------- */
/* acq100.h - common FPGA reg defs for ACQ100 series                         */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2009 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __ACQ100_H__
#define __ACQ100_H__


#ifdef __ASSEMBLER__
#define FPGA_REG(offset)   offset
#else
#define FPGA_REG(offset) ((volatile u32*)((unsigned)ACQ200_FPGA+offset))
#endif

#define ACQ100_SYSCON		FPGA_REG(0x0c)
#define ACQ100_SYSCON_DAC	FPGA_REG(0x18)
#define ACQ200_DIOCON		FPGA_REG(0x24)
#define ACQ100_OFFSET_DACS	FPGA_REG(0x28)
#define ACQ100_WAVLIMIT		FPGA_REG(0x2c)

#define ACQ200_ICR_OFFSET 0x80
#define ACQ200_ICR     FPGA_REG( ACQ200_ICR_OFFSET )

#define ACQ100_BDR_MAGIC	0xdeadbeef

#define ACQ100_SYSCON_EV_MASK    0xf
#define ACQ100_SYSCON_EV_DISABLE 0x0
#define ACQ100_SYSCON_EV_RISING  0x8
#define ACQ100_SYSCON_EV_FALLING 0x0
#define ACQ100_SYSCON_EV_DI0     0x1
#define ACQ100_SYSCON_EV_DI1     0x2
#define ACQ100_SYSCON_EV_DI2     0x3
#define ACQ100_SYSCON_EV_DI3     0x4
#define ACQ100_SYSCON_EV_DI4     0x5
#define ACQ100_SYSCON_EV_DI5     0x6
#define ACQ100_SYSCON_EV_RES1    0x7

#define ACQ100_SYSCON_EV0_SHIFT 8
#define ACQ100_SYSCON_EV1_SHIFT 12
#define ACQ100_SYSCON_TRG_SHIFT 16


#define ACQ100_ICS527_FDW	0x007f0000
#define ACQ100_ICS527_RDW	0x00007f00
#define ACQ100_ICS527_S1S0	0x000000c0

#define ACQ200_DIOCON_SETOUT    0x00ff0000
#define ACQ200_DIOCON_OUTDAT    0x0000ff00
#define ACQ200_DIOCON_INPDAT    0x000000ff

#define ACQ200_DIOCON_SETOUT_SHL 16
#define ACQ200_DIOCON_OUTDAT_SHL 8
#define ACQ200_DIOCON_INPDAT_SHL 0


#define ACQ100_OFFSET_DACS_CHIPSEL 0x80000000
#define ACQ100_OFFSET_DACS_HSHAKE  0x40000000

#define ACQ100_OFFSET_DACS_ADDRX   0xf


#define ACQ100_OFFSET_DACS_ADDR0   0x0

#define DACA 0x1
#define DACB 0x2
#define DACC 0x3
#define DACD 0x4
#define DACE 0x5
#define DACF 0x6
#define DACG 0x7
#define DACH 0x8

#define ACQ100_OFFSET_DACS_XSHIFT  16
#define ACQ100_OFFSET_DACS_YSHIFT   0
#define ACQ100_OFFSET_DACS_ASHIFT  10


#define ACQ100_WAVLIMIT_FAWG_DIV   0x00ff0000U
#define ACQ100_WAVLIMIT_WAVLIMIT   0x000001ffU

#define ACQ100_WAVLIMIT_FAWG_DIV_SHL 16


#ifdef PGMCOMOUT
/*
 * cold FIFO 8K. 
 * in V1, 0x1F = 8K, 0x10=4K 0x8=2K
 */
/*
 * REGISTERS
 */

#define ACQ100_BDR      FPGA_REG(0x00)
#define ACQ100_FIFCON   FPGA_REG(0x04)
#define ACQ100_FIFSTAT  FPGA_REG(0x08)
#define ACQ100_SYSCON   FPGA_REG(0x0c)

#define ACQ200_DIOCON   FPGA_REG( 0x24 )
#define ACQ100_OFFSET_DACS FPGA_REG(0x28)
#define ACQ100_WAVLIMIT FPGA_REG(0x2c)

#define ACQ100_TCR_IMMEDIATE FPGA_REG(0x44)
#define ACQ100_TCR_LATCH     FPGA_REG(0x48)

#define ACQ100_SYSCON_CLKVAL    0x00000008
#define ACQ100_SYSCON_TRIGGERED 0x00000004
#define ACQ100_SYSCON_SOFTTRIG  0x00000002
#define ACQ100_SYSCON_ACQEN     0x00000001

#define ACQ100_FIFSTAT_HOTPOINT   0x0000000f



#define ACQ200_DIOCON_SETOUT    0x00ff0000
#define ACQ200_DIOCON_OUTDAT    0x0000ff00
#define ACQ200_DIOCON_INPDAT    0x000000ff

#define ACQ200_DIOCON_SETOUT_SHL 16
#define ACQ200_DIOCON_OUTDAT_SHL 8
#define ACQ200_DIOCON_INPDAT_SHL 0


/** Counter compatibility with ACQ216: @@worktodo */

#define ACQ216_TCR_IMM  ACQ100_TCR_IMMEDIATE
#define ACQ216_TCR_LAT  ACQ100_TCR_LATCH

#define ACQ216_TCR_RUNNING  0x80000000
#define ACQ216_TCR_UPDCTRL  0x18000000
#define ACQ216_TCR_TCS_MASK 0x07000000
#define ACQ216_TCR_COUNTER  0x00000fff

#define ACQ216_TCR_TCS_SHL 24
#define ACQ216_TCR_UPDCTRL_SHL 27

#ifndef __ASSEMBLER__

extern int disable_acq_debug;
#define DISABLE_ACQ_DEBUG (!disable_acq_debug)

extern void disable_acq(void);
extern void enable_acq(void);


static inline u32 acq100_syscon_set(u32 flags) {
	return *ACQ100_SYSCON |= flags;
}

static inline u32 acq100_syscon_clr(u32 flags){
	return *ACQ100_SYSCON &= ~flags;
}

static inline u32 acq100_fifcon_set(u32 flags){
	return *ACQ100_FIFCON |= flags;
}

static inline u32 acq100_fifcon_clr(u32 flags){
	return *ACQ100_FIFCON &= ~flags;
}
static inline u32 acq100_getSyscon(void) {
	return *ACQ100_SYSCON;
}

#endif

#endif	/* PGMCOMOUT */

#ifndef __ASSEMBLER__


static inline void setFAWG_DIV(unsigned div) {
	u32 limit = *ACQ100_WAVLIMIT;

	div = max(div, 1U);
	div = min(div, 256U);
	
	/** factor 2 adjust reflects realite' */
	div *= 2;
	
	limit &= ~(0xff << ACQ100_WAVLIMIT_FAWG_DIV_SHL);
	limit |= (div-1) << ACQ100_WAVLIMIT_FAWG_DIV_SHL;

	*ACQ100_WAVLIMIT = limit;
}
static inline unsigned getFAWG_DIV(void) {
	unsigned div = (((*ACQ100_WAVLIMIT & ACQ100_WAVLIMIT_FAWG_DIV) >>
		ACQ100_WAVLIMIT_FAWG_DIV_SHL) + 1);
	/** factor 2 adjust reflects realite' */	
	return div / 2;
}

static inline void setWAVLIMIT(unsigned limit) {	
	u32 acq196_wavlimit = *ACQ100_WAVLIMIT;

	limit -= 1;
	limit = min(limit, ACQ100_WAVLIMIT_WAVLIMIT);

	acq196_wavlimit &= ~ACQ100_WAVLIMIT_WAVLIMIT;
	acq196_wavlimit |= limit;

	*ACQ100_WAVLIMIT = acq196_wavlimit;	
}

static inline unsigned getWAVLIMIT(void)
{
	return *ACQ100_WAVLIMIT&ACQ100_WAVLIMIT_WAVLIMIT;
}

static inline u32 acq100_lineCode(int DIx)
{
	return DIx + 1; 
}

extern int disable_acq_debug;
#define DISABLE_ACQ_DEBUG (!disable_acq_debug)

static inline u32 
reg_set_field(volatile u32* reg, int shift, u32 mask, u32 value)
{
	u32 rv = *reg;
	rv &= ~ (mask << shift);
	rv |= (value << shift);
	*reg = rv;
	return rv;
}



#define ACQ216_TCR_RUNNING  0x80000000
#define ACQ216_TCR_UPDCTRL  0x18000000
#define ACQ216_TCR_TCS_MASK 0x07000000
#define ACQ216_TCR_COUNTER  0x00000fff

#define ACQ216_TCR_TCS_SHL 24
#define ACQ216_TCR_UPDCTRL_SHL 27


#ifdef HAS_OB_CLOCK
struct OB_CLOCK_DEF {
	int demand;
	int actual;
	int FDW;
	int RDW;
	int R;
	int Sx;
};

extern struct OB_CLOCK_DEF ob_clock_def;

#define ACQ100_CLK_COUNTER_SRCMASK 0xf0000000
#define ACQ100_CLK_COUNTER_SRCSHL  28
#define ACQ100_CLK_COUNTER_SRC_DIO 0x80000000

#define ACQ100_CLK_COUNTER_COUNT   0x000fffff
#define ACQ100_CLK_COUNTER_PRESCALE	16
#define ACQ100_CLK_COUNTER_ICSSRC	0x8

#endif

#endif	/* __ASSEMBLER__ */

#endif	/*  __ACQ100_H__ */
