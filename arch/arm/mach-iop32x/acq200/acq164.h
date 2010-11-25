/* ------------------------------------------------------------------------- */
/* acq164.h - acq164 FPGA regs                                               */
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

// this change made on branch acq164
// this change made on branch acq164 on harris

#ifndef __ACQ132_H__
#define __ACQ132_H__

#define HAS_OB_CLOCK	1
#include "acq100.h"

#define ACQ164_SAMPLE_WORD_SIZE	4	/* 24 bit sample in 32 bit field */
#define ACQ164_BEST_CLK		32768000

#define ACQ164_RJ_REV		0x304	/* data now right justified */

#define ACQ164_BDR		FPGA_REG(0x00)
#define ACQ164_FIFCON		FPGA_REG(0x04)
#define ACQ164_FIFSTAT		FPGA_REG(0x08)
#define ACQ164_SYSCON		FPGA_REG(0x0c)
#define ACQ164_OSR		FPGA_REG(0x10)
#define ACQ164_ICS527		FPGA_REG(0x14)
#define ACQ164_SYSCONDAC	FPGA_REG(0x18)
#define ACQ164_CLKCON		FPGA_REG(0x1c)
#define ACQ164_DIOCON		ACQ200_DIOCON		/* FPGA_REG(0x24) */
#define ACQ164_OFFSET_DACS	ACQ100_OFFSET_DACS	/* FPGA_REG(0x28) */
#define ACQ164_WAVLIMIT		ACQ100_WAVLIMIT		/* FPGA_REG(0x2c) */
#define ACQ164_TCR_IMMEDIATE	FPGA_REG(0x44)		/* same as 196? */
#define ACQ164_TCR_LATCH	FPGA_REG(0x48)		/* same as 196? */
#define ACQ164_RGATE		FPGA_REG(0x50)		/* same as 196? */
#define ACQ164_CLK_COUNTER	FPGA_REG(0x54)	/* same as 132 */

#define ACQ164_FIFO_OFFSET     0x00100000


#define ACQ164_FIFCON_DAC_IE	  0x80000000
/** not used                      0x40000000 */
#define ACQ164_FIFCON_DAC_RESET   0x20000000
#define ACQ164_FIFCON_DAC_ENABLE  0x10000000
#define ACQ164_FIFCON_DAC_LTIDE   0x0f000000
/** not used			  0x00fe0000 */
#define ACQ164_FIFCON_THROW_EN	  0x00010000
/** not used			  0x0000f000 */

#define ACQ164_FIFCON_ADC2_RESET  0x00000800
#define ACQ164_FIFCON_ADC2_ENABLE 0x00000400
#define ACQ164_FIFCON_ADC1_RESET  0x00000200
#define ACQ164_FIFCON_ADC1_ENABLE 0x00000100
#define ACQ164_FIFCON_HOT_IE      0x00000080
#define ACQ164_FIFCON_HOT_RESET   0x00000040
#define ACQ164_FIFCON_HOT_ENABLE  0x00000020
#define ACQ164_FIFCON_HOT_HITIDE  0x0000001f


#define ACQ164_FIFCON_ADCX_ENABLE \
	(ACQ164_FIFCON_HOT_IE|ACQ164_FIFCON_HOT_ENABLE)
#define ACQ164_FIFCON_ADCX_RESET \
	(ACQ164_FIFCON_ADC2_RESET|ACQ164_FIFCON_ADC1_RESET|\
	ACQ164_FIFCON_HOT_RESET)



#define ACQ164_FIFSTAT_DAC_TR     0x80000000
#define ACQ164_FIFSTAT_DAC_LT     0x40000000
#define ACQ164_FIFSTAT_FAWG_PTR   0x3f000000
#define ACQ164_FIFSTAT_ADC_EV1    0x00800000  /* RC */
#define ACQ164_FIFSTAT_ADC_EV0    0x00400000  /* RC */
#define ACQ164_FIFSTAT_ADC_TR     0x00200000  /* RO */
/** not used			  0x001f0000 */
/* reserved                       0x00008000 */
#define ACQ164_FIFSTAT_ADC2_NE    0x00004000
#define ACQ164_FIFSTAT_ADC2_OVER  0x00002000
#define ACQ164_FIFSTAT_ADC2_UNDER 0x00001000
/* reserved                       0x00000800 */
#define ACQ164_FIFSTAT_ADC1_NE    0x00000400
#define ACQ164_FIFSTAT_ADC1_OVER  0x00000200
#define ACQ164_FIFSTAT_ADC1_UNDER 0x00000100

#define ACQ164_FIFSTAT_HOT_HT     0x00000080
#define ACQ164_FIFSTAT_HOT_OVER   0x00000040
#define ACQ164_FIFSTAT_HOT_UNDER  0x00000020
#define ACQ164_FIFSTAT_HOT_NE     0x00000010
#define ACQ164_FIFSTAT_HOTPOINT   0x0000000f

#define ACQ164_FIFSTAT_ADC_EVX	\
	(ACQ164_FIFSTAT_ADC_EV1|ACQ164_FIFSTAT_ADC_EV0)

#define ACQ164_SYSCON_REV_RESET 0x80000000
/** not used			0x78000000 */
#define ACQ164_SYSCON_OTR_MASK  0x07000000
/** not used			0x00c00000 */
#define ACQ164_SYSCON_CODE_OB   0x00200000
#define ACQ164_SYSCON_LOWLAT	0x00100000
#define ACQ164_SYSCON_TR_RISING 0x00080000
#define ACQ164_SYSCON_TR_DIO    0x00070000
#define ACQ164_SYSCON_EV1_EDGE  0x00008000
#define ACQ164_SYSCON_EV1_DIO   0x00007000
#define ACQ164_SYSCON_EV0_EDGE  0x00000800
#define ACQ164_SYSCON_EV0_DIO   0x00000700
#define ACQ164_SYSCON_LLSYNC	0x00000080
#define ACQ164_SYSCON_CDM	0x00000070
#define ACQ164_SYSCON_CLKDIV	0x00000040
#define ACQ164_SYSCON_ADCMODE	0x00000030
#define ACQ164_SYSCON_CLKVAL    0x00000008
#define ACQ164_SYSCON_TRIGGERED 0x00000004
#define ACQ164_SYSCON_SOFTTRIG  0x00000002
#define ACQ164_SYSCON_ACQEN     0x00000001


#define ACQ164_SYSCON_EV0_SHIFT 8
#define ACQ164_SYSCON_EV1_SHIFT 12
#define ACQ164_SYSCON_TRG_SHIFT 16

#define ACQ196_SYSCON_EC_SHIFT  5


#define ACQ164_SYSCON_OTR_SHIFT	24

#define ACQ164_SYSCON_OTR_DIS	0x0
#define ACQ164_SYSCON_OTR_DO3	0x1
#define ACQ164_SYSCON_OTR_DO4	0x2
#define ACQ164_SYSCON_OTR_DO5	0x3

#define ACQ164_SYSCON_CDM_SHIFT	4
#define ACQ164_SYSCON_CDM_HISPEED_256 0x0
#define ACQ164_SYSCON_CDM_HIRES_512   0x1
#define ACQ164_SYSCON_CDM_LP_256      0x2
#define ACQ164_SYSCON_CDM_LP_512      0x6
#define ACQ164_SYSCON_CDM_LS_512      0x3

/* SYSCONDAC : want exact same as ACQ196 please */

#define ACQ164_CLKCON_SCLK_RESET	0x80000000
#define ACQ164_CLKCON_SCLK_LOCK		0x40000000
/** not used				0x3fff0000 */
#define ACQ164_CLKCON_OIND_RISING	0x00008000
/** not used				0x00004000 */
#define ACQ164_CLKCON_OIND_MASK		0x00003000
#define ACQ164_CLKCON_IND_RISING	0x00000800
/* not used				0x00000400 */
#define ACQ164_CLKCON_IND_MASK		0x00000300
#define ACQ164_CLKCON_OCS_RISING	0x00000080
/* not used				0x00000040 */
#define ACQ164_CLKCON_OCS_MASK		0x00000030
#define ACQ164_CLKCON_CS_RISING		0x00000008
/* not used				0x00000004 */
#define ACQ164_CLKCON_CS_MASK		0x00000003

#define ACQ164_CLKCON_ALL \
	(ACQ164_CLKCON_CS_MASK|ACQ164_CLKCON_OCS_MASK|\
	 ACQ164_CLKCON_IND_MASK|ACQ164_CLKCON_OIND_MASK)

#define ACQ164_CLKCON_OIND_SHIFT	12
#define ACQ164_CLKCON_IND_SHIFT		8
#define ACQ164_CLKCON_OCS_SHIFT		4
#define ACQ164_CLKCON_CS_SHIFT		0

#define ACQ164_CLKCON_DIS		0x0
#define ACQ164_CLKCON_D0		0x1
#define ACQ164_CLKCON_D1		0x2
#define ACQ164_CLKCON_D2		0x3

#define ACQ164_CLKCON_RISING		0x8

#define ACQ164_CLKCON_XX		0xf
#define ACQ164_CLKCON_32768		0x0

#define ACQ100_ICR_HOTEN 0x00000080
#define ACQ100_ICR_DACEN 0x80000000


#define ACQ164_OSR_MASK	0x00ff		/* Over Sampling Register 0=1x */


#ifndef __ASSEMBLER__

static inline u32 acq164_syscon_set(u32 flags) {
	return *ACQ164_SYSCON |= flags;
}

static inline u32 acq164_syscon_clr(u32 flags){
	return *ACQ164_SYSCON &= ~flags;
}

static inline u32 acq164_clkcon_set(u32 flags) {
	return *ACQ164_CLKCON |= flags;
}
static inline u32 acq164_clkcon_clr(u32 flags) {
	return *ACQ164_CLKCON &= ~flags;
}

static inline u32 acq164_fifcon_set(u32 flags){
	return *ACQ164_FIFCON |= flags;
}

static inline u32 acq164_fifcon_clr(u32 flags){
	return *ACQ164_FIFCON &= ~flags;
}
static inline u32 acq164_getSyscon(void) {
	return *ACQ164_SYSCON;
}

static inline u32 acq164_syscon_dac_set(u32 flags) {
	return *ACQ100_SYSCON_DAC |= flags;
}
static inline u32 acq164_syscon_dac_clr(u32 flags){
	return *ACQ100_SYSCON_DAC &= ~flags;
}

static inline void enable_fifo(unsigned mask)
{
	unsigned mask_bits = 
		((mask&0x1)? ACQ164_FIFCON_ADC1_ENABLE: 0)|
		((mask&0x2)? ACQ164_FIFCON_ADC2_ENABLE: 0);
	acq164_fifcon_set(ACQ164_FIFCON_ADCX_ENABLE|mask_bits);	
}
static inline void disable_fifo(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq164_fifcon_clr(ACQ164_FIFCON_ADCX_ENABLE);
}




static inline void stop_capture(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	disable_acq();
	disable_fifo();
}



/** Counter compatibility with ACQ216: @@worktodo */

#define ACQ216_TCR_IMM  ACQ164_TCR_IMMEDIATE
#define ACQ216_TCR_LAT  ACQ164_TCR_LATCH

void acq164_set_obclock(int FDW, int RDW, int R, int Sx);

extern struct OB_CLOCK_DEF ob_clock_def;

#define CLAMP(xx,ll,rr)         ((xx) = (xx)<(ll)? (ll): (xx)>(rr)? (rr): (xx))

static inline void acq164_set_nacc(int nacc)
{
	CLAMP(nacc, 1, 256);
	*ACQ164_OSR = nacc - 1;
}

static inline int acq164_get_nacc(void)
{
	return (*ACQ164_OSR & ACQ164_OSR_MASK) + 1;
}



#endif /* #ifndef __ASSEMBLER__ */


#endif	/*  __ACQ132_H__ */
