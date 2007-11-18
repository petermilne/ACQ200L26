/* ------------------------------------------------------------------------- */
/* acq196.h                                                                  */
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


#ifndef __ACQ196_H__
#define __ACQ196_H__

#ifdef __ASSEMBLER__
#define FPGA_REG(offset)   offset
#else
#define FPGA_REG(offset) ((volatile u32*)((unsigned)ACQ200_FPGA+offset))
#endif


/*
 * cold FIFO 8K. 
 * in V1, 0x1F = 8K, 0x10=4K 0x8=2K
 */
/*
 * REGISTERS
 */

#define ACQ196_BDR      FPGA_REG(0x00)
#define ACQ196_FIFCON   FPGA_REG(0x04)
#define ACQ196_FIFSTAT  FPGA_REG(0x08)
#ifdef __ASSEMBLER__
#define ACQ196_SYSCON   FPGA_REG(0x0c)
#else
#define ACQ196_SYSCON_ADC  FPGA_REG(0x0c)
#define ACQ196_SYSCON_DAC  FPGA_REG(0x18)
#endif
#define ACQ196_CLKCON   FPGA_REG( 0x1c )
#define ACQ200_CLKDAT   FPGA_REG( 0x20 )

#define ACQ200_DIOCON   FPGA_REG( 0x24 )
#define ACQ196_OFFSET_DACS FPGA_REG(0x28)
#define ACQ196_WAVLIMIT FPGA_REG(0x2c)

/* standard FW - pulse generator */
#define ACQ196_PGCSS	FPGA_REG(0x30)
#define ACQ196_PGTIMER	FPGA_REG(0x3c)

/* MAC firmware */
#define ACQ196_MACCON   FPGA_REG(0x30)
#define ACQ196_MACSUB   FPGA_REG(0x34)
#define ACQ196_FIRK     FPGA_REG(0x38)


#define ACQ196_DIO_COPY      FPGA_REG(0x40)
#define ACQ196_TCR_IMMEDIATE FPGA_REG(0x44)
#define ACQ196_TCR_LATCH     FPGA_REG(0x48)
#define ACQ196_LL_AO_SCRATCH FPGA_REG(0x4c)
#define ACQ196_LL_AI_SCRATCH FPGA_REG(0xc0)

#define ACQ200_ICR_OFFSET 0x80
#define ACQ200_ICR     FPGA_REG( ACQ200_ICR_OFFSET )

#define ACQ196_FIFO_OFFSET     0x00100000
#define ACQ196_WAVEFORM_OFFSET 0x00200000
#define ACQ196_MACREF_OFFSET   0x00300000

/*
 * FIELDS
 */

#define ACQ196_BDR_DEFAULT 0xdeadbeef


#define ACQ196_FIFCON_DAC_IE      0x80000000
/** not used                      0x40000000 */
#define ACQ196_FIFCON_DAC_RESET   0x20000000
#define ACQ196_FIFCON_DAC_ENABLE  0x10000000
#define ACQ196_FIFCON_DAC_LTIDE   0x0f000000

#define ACQ196_FIFCON_ADC3_RESET  0x00002000
#define ACQ196_FIFCON_ADC3_ENABLE 0x00001000
#define ACQ196_FIFCON_ADC2_RESET  0x00000800
#define ACQ196_FIFCON_ADC2_ENABLE 0x00000400
#define ACQ196_FIFCON_ADC1_RESET  0x00000200
#define ACQ196_FIFCON_ADC1_ENABLE 0x00000100
#define ACQ196_FIFCON_HOT_IE      0x00000080
#define ACQ196_FIFCON_HOT_RESET   0x00000040
#define ACQ196_FIFCON_HOT_ENABLE  0x00000020
#define ACQ196_FIFCON_HOT_HITIDE  0x0000001f

#define ACQ196_FIFCON_RESET_ALL_COLD   \
        (ACQ196_FIFCON_ADC3_RESET|\
         ACQ196_FIFCON_ADC2_RESET|ACQ196_FIFCON_ADC1_RESET)

#define ACQ196_FIFCON_RESET_ALL \
        (ACQ196_FIFCON_RESET_ALL_COLD|ACQ196_FIFCON_HOT_RESET)

#define ACQ196_FIFCON_ENABLE_ALL_COLD \
        (ACQ196_FIFCON_ADC3_ENABLE|\
	ACQ196_FIFCON_ADC2_ENABLE|ACQ196_FIFCON_ADC1_ENABLE)

#define ACQ196_FIFCON_ENABLE_HOT\
        (ACQ196_FIFCON_HOT_ENABLE|ACQ196_FIFCON_HOT_IE)

#define ACQ196_FIFCON_ENABLE_ALL \
        (ACQ196_FIFCON_ENABLE_ALL_COLD|ACQ196_FIFCON_ENABLE_HOT)

        

#define ACQ196_FIFSTAT_DAC_TR     0x80000000
#define ACQ196_FIFSTAT_DAC_LT     0x40000000
#define ACQ196_FIFSTAT_FAWG_PTR   0x3f000000
#define ACQ196_FIFSTAT_ADC_EV1    0x00800000  /* RC */
#define ACQ196_FIFSTAT_ADC_EV0    0x00400000  /* RC */
#define ACQ196_FIFSTAT_ADC_TR     0x00200000  /* RO */
#define ACQ196_FIFSTAT_ADC_CLKDLY 0x00100000  /* RO - delay from clk expired */
#define ACQ196_FIFSTAT_LL_TIMET   0x00100000  /* alias "" */
/* reserved                       0x00080000 */
#define ACQ196_FIFSTAT_ADC3_NE    0x00040000
#define ACQ196_FIFSTAT_ADC3_OVER  0x00020000
#define ACQ196_FIFSTAT_ADC3_UNDER 0x00010000
/* reserved                       0x00008000 */
#define ACQ196_FIFSTAT_ADC2_NE    0x00004000
#define ACQ196_FIFSTAT_ADC2_OVER  0x00002000
#define ACQ196_FIFSTAT_ADC2_UNDER 0x00001000
/* reserved                       0x00000800 */
#define ACQ196_FIFSTAT_ADC1_NE    0x00000400
#define ACQ196_FIFSTAT_ADC1_OVER  0x00000200
#define ACQ196_FIFSTAT_ADC1_UNDER 0x00000100

#define ACQ196_FIFSTAT_HOT_HT     0x00000080
#define ACQ196_FIFSTAT_HOT_OVER   0x00000040
#define ACQ196_FIFSTAT_HOT_UNDER  0x00000020
#define ACQ196_FIFSTAT_HOT_NE     0x00000010
#define ACQ196_FIFSTAT_HOTPOINT   0x0000000f

#define ACQ196_FIFSTAT_FAWG_COUNT(s) (((s)&ACQ196_FIFSTAT_FAWG_PTR) >> 24)
#define ACQ196_FIFSTAT_ADC_EVX  (ACQ196_FIFSTAT_ADC_EV1|ACQ196_FIFSTAT_ADC_EV0)

#define ACQ196_FIFSTAT_ALL_FLAGS \
        (ACQ196_FIFSTAT_ADC_EV1|ACQ196_FIFSTAT_ADC_EV0| \
         ACQ196_FIFSTAT_ADC1_OVER|ACQ196_FIFSTAT_ADC1_UNDER| \
         ACQ196_FIFSTAT_ADC2_OVER|ACQ196_FIFSTAT_ADC2_UNDER| \
         ACQ196_FIFSTAT_ADC3_OVER|ACQ196_FIFSTAT_ADC3_UNDER| \
         ACQ196_FIFSTAT_HOT_OVER|ACQ196_FIFSTAT_HOT_UNDER)


#define ACQ196_SYSCON_DAC_2CHAN     0x01000000
#define ACQ196_SYSCON_DAC_WAVE_MODE 0x00800000


#define ACQ196_SYSCON_CLKDLY_SHIFT 24
#define ACQ196_SYSCON_LATIMER_SHIFT ACQ196_SYSCON_CLKDLY_SHIFT

#define ACQ196_SYSCON_SIM_MODE  0x00800000
#define ACQ196_SYSCON_SP_ENABLE 0x00400000
#define ACQ196_SYSCON_CODE_OB   0x00200000
#define ACQ196_SYSCON_LOWLAT    0x00100000
#define ACQ196_SYSCON_TR_RISING 0x00080000
#define ACQ196_SYSCON_TR_DIO    0x00070000
#define ACQ196_SYSCON_EV1_EDGE  0x00008000
#define ACQ196_SYSCON_EV1_DIO   0x00007000
#define ACQ196_SYSCON_EV0_EDGE  0x00000800
#define ACQ196_SYSCON_EV0_DIO   0x00000700
#define ACQ196_SYSCON_EC_RISING 0x00000080
#define ACQ196_SYSCON_EC_MASK   0x000000f0
#define ACQ196_SYSCON_EC_DI0    0x00000000
#define ACQ196_SYSCON_EC_DI1    0x00000020
#define ACQ196_SYSCON_EC_DI2    0x00000040
#define ACQ196_SYSCON_EC_SOFT   0x00000060

#define ACQ196_SYSCON_EXTCLK    0x00000010
#define ACQ196_SYSCON_CLKVAL    0x00000008
#define ACQ196_SYSCON_TRIGGERED 0x00000004
#define ACQ196_SYSCON_SOFTTRIG  0x00000002
#define ACQ196_SYSCON_ACQEN     0x00000001

#define ACQ196_SYSCON_EV_MASK    0xf
#define ACQ196_SYSCON_EV_DISABLE 0x0
#define ACQ196_SYSCON_EV_RISING  0x8
#define ACQ196_SYSCON_EV_FALLING 0x0
#define ACQ196_SYSCON_EV_DI0     0x1
#define ACQ196_SYSCON_EV_DI1     0x2
#define ACQ196_SYSCON_EV_DI2     0x3
#define ACQ196_SYSCON_EV_DI3     0x4
#define ACQ196_SYSCON_EV_DI4     0x5
#define ACQ196_SYSCON_EV_DI5     0x6
#define ACQ196_SYSCON_EV_RES1    0x7

#define ACQ196_SYSCON_EV0_SHIFT 8
#define ACQ196_SYSCON_EV1_SHIFT 12
#define ACQ196_SYSCON_TRG_SHIFT 16

#define ACQ196_SYSCON_EC_SHIFT  5

#define ACQ196_WAVE_MAX 512

#define ACQ196_FIFSTAT_ADC_ERR \
        (ACQ196_FIFSTAT_ADC3_OVER|ACQ196_FIFSTAT_ADC3_UNDER|\
	 ACQ196_FIFSTAT_ADC2_OVER|ACQ196_FIFSTAT_ADC2_UNDER|\
	 ACQ196_FIFSTAT_ADC1_OVER|ACQ196_FIFSTAT_ADC1_UNDER|\
	 ACQ196_FIFSTAT_HOT_OVER|ACQ196_FIFSTAT_HOT_UNDER)

#define ACQ196_FIFSTAT_COLD_NE \
       (ACQ196_FIFSTAT_ADC3_NE|ACQ196_FIFSTAT_ADC2_NE|ACQ196_FIFSTAT_ADC1_NE)

#ifndef __ASSEMBLER__
static inline u32 acq196_syscon_set_all(u32 flags) {
	return *ACQ196_SYSCON_ADC |= flags;
}

static inline u32 acq196_syscon_clr_all(u32 flags){
	return *ACQ196_SYSCON_ADC &= ~flags;
}

static inline u32 acq196_fifcon_set_all(u32 flags){
	return *ACQ196_FIFCON |= flags;
}

static inline u32 acq196_fifcon_init_all(u32 flags){
	return *ACQ196_FIFCON = flags;
}

static inline u32 acq196_fifcon_clr_all(u32 flags){
	return *ACQ196_FIFCON &= ~flags;
}
static inline u32 acq196_getSyscon(void) {
	return *ACQ196_SYSCON_ADC;
}

static inline u32 acq196_syscon_dac_set(u32 flags) {
	return *ACQ196_SYSCON_DAC |= flags;
}
static inline u32 acq196_syscon_dac_clr(u32 flags){
	return *ACQ196_SYSCON_DAC &= ~flags;
}


#endif /* __ASSEMBLER__ */

#define ACQ196_HOT_HALF 0x8


/** not used                    0xff000000 */
#define ACQ196_CLKCON_TR_MASK   0x00f00000
#define ACQ196_CLKCON_TRMAS     0x00080000
#define ACQ196_CLKCON_OTR_MASK  0x00070000
/** not used                    0x0000f000 */
#define ACQ196_CLKCON_LLSYNC    0x00000800
#define ACQ196_CLKCON_SCLKSYNC  0x00000400
#define ACQ196_CLKCON_SFTCLKMD  0x00000200
#define ACQ196_CLKCON_SFTCLKS   0x00000100
#define ACQ196_CLKCON_CS_MASK   0x000000f0
#define ACQ196_CLKCON_CLKMAS    0x00000008
#define ACQ196_CLKCON_OCS_MASK  0x00000007

#define ACQ196_CLKCON_TR_SHIFT  20
#define ACQ196_CLKCON_TR_DIx(x) ((0x8 | (x)) << ACQ196_CLKCON_TR_SHIFT)
#define ACQ196_CLKCON_TR_SOFT   ACQ196_CLKCON_TR_DIx(7)

#define ACQ196_CLKCON_OTR_SHIFT 16
#define ACQ196_CLKCON_OTR_DOx(x) ((x) << ACQ196_CLKCON_OTR_SHIFT)

#define ACQ196_CLKCON_CS_SHIFT  4
#define ACQ196_CLKCON_CS_DIx(x) ((0x8 | (x)) << ACQ196_CLKCON_CS_SHIFT)
#define ACQ196_CLKCON_CS_66M    ACQ196_CLKCON_CS_DIx(0xf)

#define ACQ196_CLKCON_OCS_SHIFT 0
#define ACQ196_CLKCON_OCS_DOx(x) ((x) << ACQ196_CLKCON_OCS_SHIFT)

#define ACQ200_DIOCON_SETOUT    0x00ff0000
#define ACQ200_DIOCON_OUTDAT    0x0000ff00
#define ACQ200_DIOCON_INPDAT    0x000000ff

#define ACQ200_DIOCON_SETOUT_SHL 16
#define ACQ200_DIOCON_OUTDAT_SHL 8
#define ACQ200_DIOCON_INPDAT_SHL 0


#define ACQ196_TCR_MASK         0x00000fff   /* 12 bit counter */

#define ACQ196_TCR_RUN          0x80000000   

#define ACQ196_TCS_MASK         0x07000000
#define ACQ196_TCS_DI0          0x00000000
#define ACQ196_TCS_DI1          0x01000000
#define ACQ196_TCS_DI2          0x02000000
#define ACQ196_TCS_DI3          0x03000000
#define ACQ196_TCS_DI4          0x04000000
#define ACQ196_TCS_DI5          0x05000000
#define ACQ196_TCS_INT          0x06000000


#define COLD_FIFO_SZ         8192
#define COLD_FIFO_ENTRIES    16
#define COLD_FIFO_ENTRY_SIZE (COLD_FIFO_SZ/COLD_FIFO_ENTRIES)
#define COLD_FIFO_ENT

#define HOT_FIFO_SZ              4096
#define HOT_FIFO_ENTRIES         16
#define HOT_FIFO_ENTRY_SIZE      (FIFO_SZ/FIFO_ENTRIES)

#define HOT_FIFO_SZK             (HOT_FIFO_SZ/1024)

#if 0
#define COLD_FIFO_HALF(fifcon) (fifcon&ACQ200_FIFCON_COLD_HALF)
#define HOT_FIFO_HALF(fifcon)  (fifcon&ACQ200_FIFCON_HOT_HALF)
#define HOT_FULL(fifcon)       (HOT_FIFO_FULL_ENTRIES(fifcon)==15)
#endif


#define ACQ100_ICR_HOTEN 0x00000080
#define ACQ100_ICR_DACEN 0x80000000


#ifndef __ASSEMBLER__

extern int disable_acq_debug;
#define DISABLE_ACQ_DEBUG (!disable_acq_debug)

static inline void disable_acq(void) 
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq196_syscon_clr_all(ACQ196_SYSCON_ACQEN);
}
static inline void enable_acq(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq196_syscon_set_all(ACQ196_SYSCON_ACQEN);
}

static inline void soft_trig_all(void)
{
	acq196_syscon_set_all(ACQ196_SYSCON_SOFTTRIG);
	acq196_syscon_clr_all(ACQ196_SYSCON_SOFTTRIG);
}

static inline void disable_fifo(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	acq196_fifcon_clr_all(ACQ196_FIFCON_ENABLE_ALL);	
}
static inline void reset_fifo(void)
{
	acq196_fifcon_set_all(ACQ196_FIFCON_RESET_ALL);
	*ACQ196_FIFSTAT = ACQ196_FIFSTAT_ALL_FLAGS;
	acq196_fifcon_clr_all(ACQ196_FIFCON_RESET_ALL);
}
static inline void enable_fifo(unsigned mask)
{
	unsigned enable = ACQ196_FIFCON_ENABLE_HOT;

	if (mask&0x1){
		enable |= ACQ196_FIFCON_ADC1_ENABLE;
	}
	if (mask&0x2){
		enable |= ACQ196_FIFCON_ADC2_ENABLE;
	}
	if (mask&0x4){
		enable |= ACQ196_FIFCON_ADC3_ENABLE;
	}

	dbg(DISABLE_ACQ_DEBUG, "");
	acq196_fifcon_set_all(enable);
}
static inline void stop_capture(void)
{
	dbg(DISABLE_ACQ_DEBUG, "");
	disable_acq();
	disable_fifo();
}


static inline u32 acq196_lineCode(int DIx)
{
	return DIx + 1; 
}

static inline void acq196_set_adc_clkdly(u8 clkdly)
{
	u32 syscon_adc = *ACQ196_SYSCON_ADC;

	syscon_adc &= ~(0xff << ACQ196_SYSCON_CLKDLY_SHIFT);
	syscon_adc |=  (clkdly << ACQ196_SYSCON_CLKDLY_SHIFT);
	*ACQ196_SYSCON_ADC = syscon_adc;
}

static inline u8 acq196_get_adc_clkdly(void)
{
	return *ACQ196_SYSCON_ADC >> ACQ196_SYSCON_CLKDLY_SHIFT;
}


#define ACQ196_OFFSET_DACS_CHIPSEL 0x80000000
#define ACQ196_OFFSET_DACS_HSHAKE  0x40000000

#define ACQ196_OFFSET_DACS_ADDRX   0xf


#define ACQ196_OFFSET_DACS_ADDR0   0x0

#define DACA 0x1
#define DACB 0x2
#define DACC 0x3
#define DACD 0x4
#define DACE 0x5
#define DACF 0x6
#define DACG 0x7
#define DACH 0x8

#define ACQ196_OFFSET_DACS_XSHIFT  16
#define ACQ196_OFFSET_DACS_YSHIFT   0
#define ACQ196_OFFSET_DACS_ASHIFT  10


static inline void __dac_reset(void) {
	acq196_fifcon_set_all(ACQ196_FIFCON_DAC_RESET);
	acq196_fifcon_clr_all(ACQ196_FIFCON_DAC_RESET);
	acq196_fifcon_set_all(ACQ196_FIFCON_DAC_ENABLE);
}
static inline void dac_reset(void) {
	if ((*ACQ196_SYSCON_DAC & ACQ196_SYSCON_TRIGGERED) == 0){
		__dac_reset();
	}
}

static inline void __dac_disarm(void) {
	*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_ACQEN;
}
static inline void __dac_arm(void) {
	*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_ACQEN;
}
static inline void dac_arm(void) {
	if ((*ACQ196_SYSCON_DAC & ACQ196_SYSCON_TRIGGERED) == 0){
		__dac_arm();
	}
}

static inline void __dac_softtrig(void) {
	*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_SOFTTRIG;
	*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_SOFTTRIG;
}

static inline void dac_softtrig(void) {
	if ((*ACQ196_SYSCON_DAC & ACQ196_SYSCON_TRIGGERED) == 0){
		__dac_softtrig();
	}
}

static inline u32 arch_get_int_clk_div(void)
{
	return *ACQ200_CLKDAT;
}

static inline u32 arch_set_int_clk_div(u32 clkdiv)
{
	return *ACQ200_CLKDAT = clkdiv;
}


static inline u8 acq100_get_cpld_rev(void)
{
	return *(volatile unsigned char*)(ACQ200_CPLD+3) & 0xf;
}

#define ACQ100_CPLD_REV_SSM 2

#define ACQ100_CPLD_SSM_CAPABLE \
         (acq100_get_cpld_rev() >= ACQ100_CPLD_REV_SSM)

static inline u8 acq100_get_pci_env(void)
{
	if (ACQ100_CPLD_SSM_CAPABLE){
		return (*(volatile unsigned char*)(ACQ200_CPLD+2) >> 1) & 0x3;
        }else{
		return 0xff;
	}
}

#define ACQ100_PCIENV_SSM 0           /** system slot master */
#define ACQ100_PCIENV_PM  1           /** peripheral mode    */
#define ACQ100_PCIENV_SAM 2           /** standalone mode    */



#endif /* __ASSEMBLER__ */

#define ACQ196_WAVLIMIT_FAWG_DIV   0x00ff0000U
#define ACQ196_WAVLIMIT_WAVLIMIT   0x000001ffU

#define ACQ196_WAVLIMIT_FAWG_DIV_SHL 16

#define ACQ196_MACCON_REF_3SHL  20
#define ACQ196_MACCON_REF_2SHL  18
#define ACQ196_MACCON_REF_1SHL  16

#define ACQ196_MACCON_REFSHL(b) ((b)==1? 16:(b)==2? 18: 20)

#define ACQ196_MACCON_REF_SEL_MASK 0x3
#define ACQ196_MACCON_REF_SEL_ONE  0x0
#define ACQ196_MACCON_REF_SEL_DAC  0x1
#define ACQ196_MACCON_REF_SEL_ADC  0x2


#ifndef __ASSEMBLER__

static inline void setFAWG_DIV(unsigned div) {
	u32 limit = *ACQ196_WAVLIMIT;

	div = max(div, 1U);
	div = min(div, 256U);
	
	/** factor 2 adjust reflects realite' */
	div *= 2;
	
	limit &= ~(0xff << ACQ196_WAVLIMIT_FAWG_DIV_SHL);
	limit |= (div-1) << ACQ196_WAVLIMIT_FAWG_DIV_SHL;

	*ACQ196_WAVLIMIT = limit;
}
static inline unsigned getFAWG_DIV(void) {
	unsigned div = (((*ACQ196_WAVLIMIT & ACQ196_WAVLIMIT_FAWG_DIV) >>
		ACQ196_WAVLIMIT_FAWG_DIV_SHL) + 1);
	/** factor 2 adjust reflects realite' */	
	return div / 2;
}

static inline void setWAVLIMIT(unsigned limit) {	
	u32 acq196_wavlimit = *ACQ196_WAVLIMIT;

	limit -= 1;
	limit = min(limit, ACQ196_WAVLIMIT_WAVLIMIT);

	acq196_wavlimit &= ~ACQ196_WAVLIMIT_WAVLIMIT;
	acq196_wavlimit |= limit;

	*ACQ196_WAVLIMIT = acq196_wavlimit;	
}

static inline unsigned getWAVLIMIT(void)
{
	return *ACQ196_WAVLIMIT&ACQ196_WAVLIMIT_WAVLIMIT;
}
static inline void acq100_set_maccon_ref(unsigned bank, unsigned select) {
	*ACQ196_MACCON &= 
		~(ACQ196_MACCON_REF_SEL_MASK <<ACQ196_MACCON_REFSHL(bank));
	*ACQ196_MACCON |= select << ACQ196_MACCON_REFSHL(bank);
}

static inline unsigned acq100_get_maccon_ref(unsigned bank) {
	return (*ACQ196_MACCON >> ACQ196_MACCON_REFSHL(bank)) & 
		ACQ196_MACCON_REF_SEL_MASK;
}
#endif /* __ASSEMBLER__ */

#define ACQ196_MACCON_OVFL      0x0000e000
#define ACQ196_MACCON_LENGTH    0x00000fff

#define ACQ196_MACCON_OVFL3     0x00008000
#define ACQ196_MACCON_OVFL2     0x00004000
#define ACQ196_MACCON_OVFL1     0x00002000


/** Counter compatibility with ACQ216: @@worktodo */

#define ACQ216_TCR_IMM  ACQ196_TCR_IMMEDIATE
#define ACQ216_TCR_LAT  ACQ196_TCR_LATCH

#define ACQ216_TCR_RUNNING  0x80000000
#define ACQ216_TCR_UPDCTRL  0x18000000
#define ACQ216_TCR_TCS_MASK 0x07000000
#define ACQ216_TCR_COUNTER  0x00000fff

#define ACQ216_TCR_TCS_SHL 24
#define ACQ216_TCR_UPDCTRL_SHL 27



#define ACQ196_PGCSS_GENEN		0x00000100
#define ACQ196_PGCSS_TRIGRISING		0x00000080
#define ACQ196_PGCSS_TRIGSEL		0x00000070
#define ACQ196_PGCSS_DONOT		0x00000008
#define ACQ196_PGCSS_DOSEL		0x00000007

#define ACQ196_PGCSS_TRIGSEL_SHL	4
#define ACQ196_PGCSS_DOSEL_SHL		0

#define ACQ196_PGTIMER_OFFTIME_MASK     0x00000fff
#define ACQ196_PGTIMER_PULSECOUNT_MASK  0x0003f000
#define ACQ196_PGTIMER_ONTIME_MASK	0x01fc0000
#define ACQ196_PGTIMER_DELTIME_MASK     0xfe000000

#define ACQ196_PGTIMER_OFFTIME_SHL	0
#define ACQ196_PGTIMER_PULSECOUNT_SHL	12
#define ACQ196_PGTIMER_ONTIME_SHL	18
#define ACQ196_PGTIMER_DELTIMER_SHL	25


#define ACQ196_PGTIMER_MAX_OFFTIME	0x0fff
#define ACQ196_PGTIMER_MAX_PULSECOUNT   63
#define ACQ196_PGTIMER_MAX_ONTIME	0x007f
#define ACQ196_PGTIMER_MAX_DELAY	0x007f

#endif /* acq196.h */
