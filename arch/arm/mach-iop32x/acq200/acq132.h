/* ------------------------------------------------------------------------- */
/* acq132.h                                                                  */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2007 Peter Milne, D-TACQ Solutions Ltd
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


#ifndef __ACQ132_H__
#define __ACQ132_H__

#include "acq196.h"
#define ACQ132_BDR	ACQ196_BDR
#define ACQ132_FIFCON	ACQ196_FIFCON
#define ACQ132_FIFSTAT  ACQ196_FIFSTAT
#define ACQ132_SYSCON	ACQ196_SYSCON_ADC
/* custom regs */
#define ACQ132_SFPGA_CONF	FPGA_REG(0x10)
#define ACQ132_ICS527		FPGA_REG(0x18)
#define ACQ132_SCAN_LIST_DEF	FPGA_REG(0x50)
#define ACQ132_SCAN_LIST_LEN	FPGA_REG(0x54)



#define ACQ132_GATE_PULSE_FIFO  FPGA_REG(0xc0)
#define ACQ132_ADC_A		FPGA_REG(0x100)
#define ACQ132_ADC_B		FPGA_REG(0x200)
#define ACQ132_ADC_C		FPGA_REG(0x300)
#define ACQ132_ADC_D		FPGA_REG(0x400)

/* Dev 0..3 */
#define ACQ132_ADC_REGS(dev)		FPGA_REG((dev+1)*0x100)
#define ACQ132_ADC_REG(dev, reg)	FPGA_REG((dev+1)*0x100 + (reg))

#define ACQ132_ADC_CTRL_OFFSET	0x04

#define ACQ132_ADC_TEST(dev)	ACQ132_ADC_REG(dev, 0x00)
#define ACQ132_ADC_CTRL(dev)	ACQ132_ADC_REG(dev, ACQ132_ADC_CTRL_OFFSET)
#define ACQ132_ADC_RANGE(dev)	ACQ132_ADC_REG(dev, 0x08)
#define ACQ132_ADC_OSAM(dev)	ACQ132_ADC_REG(dev, 0x0c)
#define ACQ132_ADC_FIFSTA(dev)	ACQ132_ADC_REG(dev, 0x10)

#define BANK_A	0
#define BANK_B  1
#define BANK_C  2
#define BANK_D  3

#define BDR_MAGIC	0xdeadbeef

/* channel mask in bank */
#define MASK_1	0x1
#define MASK_2	0x5
#define MASK_4	0xf


/* custom bits */

#define ACQ132_FIFCON_GPG_RESET	0x00000100

#define ACQ132_SYSCON_REV_RESET 0x80000000
#define ACQ132_SYSCON_GATE_EN	0x10000000
#define ACQ132_SYSCON_GATE_HI	0x08000000
#define ACQ132_SYSCON_GATE_DIO	0x07000000
#define ACQ132_SYSCON_GATE_MASK 0x1f000000
#define ACQ132_SYSCON_GATE_SHL	24

#define ACQ132_SYSCON_GPG_MAS	0x00f00000
#define ACQ132_SYSCON_GPG_MASD7 0x00800000
#define ACQ132_SYSCON_GPG_MASD6 0x00400000
#define ACQ132_SYSCON_GPG_MASD5 0x00200000
#define ACQ132_SYSCON_GPG_MASD4 0x00100000

/* obsoleted by rev 3 */
#define ACQ132_SYSCON_RANGE_HI 0x00800000


#define ACQ132_FIFSTAT_ADC_EV	0x00400000	/* Event or Gate (RGM) */
#define ACQ132_FIFSTAT_ADC_TRG  0x00200000	/* ADC system has triggered */
 
#define ACQ132_FIFSTAT_GPG_NE	0x00020000
#define ACQ132_FIFSTAT_GPG_PTR	0x0001ff00


#define ACQ132_SFPGA_CONF_PROG		31
#define ACQ132_SFPGA_CONF_INIT_LR	29	
#define ACQ132_SFPGA_CONF_DONE_8	20
#define ACQ132_SFPGA_CONF_BUSY		17
#define ACQ132_SFPGA_CONF_UPDATE	16
#define ACQ132_SFPGA_CONF_DATA		0x0000ffff

#define ACQ132_SFPGA_CONF_LRMASK	0x3
#define ACQ132_SFPGA_CONF_8MASK		0xff


#define ACQ132_ICS527_FDW	0x007f0000
#define ACQ132_ICS527_RDW	0x00007f00
#define ACQ132_ICS527_S1S0	0x000000c0
#define ACQ132_ICS527_CLKDIV	0x00000007  
/* the divider is actually in the A_FPGA, 
   but convenient to consider as part of ICS527 */


#define ACQ132_SCAN_S4	0
#define ACQ132_SCAN_S3  2
#define ACQ132_SCAN_S2	4
#define ACQ132_SCAN_S1	6

#define SxA	0x0
#define SxB	0x1
#define SxC	0x2
#define SxD	0x3

#define ACQ132_SCAN_MAX	16	/* max elements in list */

/* CTRL_CHMASK - shifts */

#define ACQ132_ADC_CTRL_LPSshl  24
#define ACQ132_ADC_CTRL_RPSshl  8

#define ACQ132_ADC_CTRL_PSmask	0x7f

#define ACQ132_ADC_CTRL_LMSHFT	20
#define ACQ132_ADC_CTRL_RMSHFT  4

#define ACQ132_ADC_CTRL_LRANGE  0x80008000
#define ACQ132_ADC_CTRL_CMASK	0x00f000f0

#define ACQ132_ADC_CTRL_FIFORES 0x00080008
#define ACQ132_ADC_CTRL_PREPOST 0x00040004
#define ACQ132_ADC_CTRL_SIGEN	0x00020002    /* EventSig (Timestamp) Enable) */
#define ACQ132_ADC_CTRL_ACQEN	0x00010001

#define ACQ132_ADC_RANGE_L4	0x00080000
#define ACQ132_ADC_RANGE_L3	0x00040000
#define ACQ132_ADC_RANGE_L2	0x00020000
#define ACQ132_ADC_RANGE_L1	0x00010000

#define ACQ132_ADC_RANGE_R4	0x00000008
#define ACQ132_ADC_RANGE_R3	0x00000004
#define ACQ132_ADC_RANGE_R2	0x00000002
#define ACQ132_ADC_RANGE_R1	0x00000001

#define ACQ132_ADC_OSAM_L_NACC		28
#define ACQ132_ADC_OSAM_L_SHIFT		24
#define ACQ132_ADC_OSAM_L_ACCEN		16
#define ACQ132_ADC_OSAM_R_NACC		12
#define ACQ132_ADC_OSAM_R_SHIFT		 8
#define ACQ132_ADC_OSAM_R_ACCEN		 0

#define OSAMBITS	0x0fU
#define SHIFTBITS	0x0fU

#define SHIFT_M1 0x0fU
#define SHIFT_M2 0x0eU
#define SHIFT_0	 0x00U
#define SHIFT_P1 0x01U
#define SHIFT_P2 0x02U

#define OSAMLR(lr) (((lr)=='L' || (lr) == 16)? 16: 0)


#define NORMALISE_MASK(mask, shft) (((mask)&(0xfU << (shft))) >> (shft))

/* @todo one bit, all bits. multiple settings todo */
void acq132_set_adc_range(u32 channels);


u32 acq132_get_adc_range(void);

void acq132_set_osam_nacc(
	int dev, int lr, int nacc, int shift, int decimate);

static inline void acq132_adc_set_all(int reg, u32 bits)
{
	int dev;
	for (dev = BANK_A; dev <= BANK_D; ++dev){
		u32 rv = *ACQ132_ADC_REG(dev, reg);
		rv |= bits;

		*ACQ132_ADC_REG(dev, reg) = rv;
	}
}

static inline void acq132_adc_clr_all(int reg, u32 bits)
{
	int dev;
	u32 nbits = ~bits;

	for (dev = BANK_A; dev <= BANK_D; ++dev){
		u32 rv = *ACQ132_ADC_REG(dev, reg);
		rv &= nbits;
		*ACQ132_ADC_REG(dev, reg) = rv;
	}
}

static inline void acq132_set_prescale(int prescale)
{
	unsigned clr_mask = 
		ACQ132_ADC_CTRL_PSmask<<ACQ132_ADC_CTRL_LPSshl |
		ACQ132_ADC_CTRL_PSmask<<ACQ132_ADC_CTRL_RPSshl;
	unsigned set_mask;
	prescale = max(prescale, 1);
	prescale = min(prescale, 128);
	prescale -= 1;

	set_mask = 
		prescale<<ACQ132_ADC_CTRL_LPSshl |
		prescale<<ACQ132_ADC_CTRL_RPSshl;

	acq132_adc_clr_all(ACQ132_ADC_CTRL_OFFSET, clr_mask);
	if (set_mask){
		acq132_adc_set_all(ACQ132_ADC_CTRL_OFFSET, set_mask);
	}	
}

static inline int acq132_get_prescale(void)
/* WARNING: reads BANK_A,L only */
{
	u32 ctrl = *ACQ132_ADC_CTRL(BANK_A);
	u32 prescale = 
		(ctrl>>ACQ132_ADC_CTRL_LPSshl & ACQ132_ADC_CTRL_PSmask) + 1;
	return prescale;
}
static inline void sfpga_conf_clr_all(void) {
	dbg(2, "01");
	*ACQ132_SFPGA_CONF = 0;
}
static inline void sfpga_conf_set_prog(void) {
	dbg(2, "01");
	*ACQ132_SFPGA_CONF |= (1 << ACQ132_SFPGA_CONF_PROG);
}	

static inline void sfpga_conf_clr_prog(void) {
	dbg(2, "01");
	*ACQ132_SFPGA_CONF &= ~(1 << ACQ132_SFPGA_CONF_PROG);
}

static inline void sfpga_conf_set_update(void) {
	dbg(2, "01");
	*ACQ132_SFPGA_CONF |= (1 << ACQ132_SFPGA_CONF_UPDATE);
}

static inline int sfpga_conf_get_busy(void) {
	int rc = (*ACQ132_SFPGA_CONF & (1<<ACQ132_SFPGA_CONF_BUSY)) != 0;
	dbg(2, "busy %d", rc);
	return rc;
}

static inline int _sfpga_conf_get_init(void) {
	u32 conf = *ACQ132_SFPGA_CONF; 
	int lr = (conf >> ACQ132_SFPGA_CONF_INIT_LR) & 0x3;

	dbg(2, "LR return %d", lr );

	return lr;
}

static inline int sfpga_conf_init_is_low(void) {
	return _sfpga_conf_get_init() == 0;	
}

static inline int sfpga_conf_init_is_hi(void) {
	return _sfpga_conf_get_init() == 0x3;
}

static inline void sfpga_conf_send_data(u16 data)
{
	u32 conf = *ACQ132_SFPGA_CONF;
	conf &= ~ACQ132_SFPGA_CONF_DATA;
	conf |= (1 << ACQ132_SFPGA_CONF_UPDATE) | data;


	*ACQ132_SFPGA_CONF = conf;		

	dbg(2, "conf set 0x%08x", conf);
}

static inline int sfpga_conf_done(void) {
	u32 conf = *ACQ132_SFPGA_CONF;
	int done = ((conf>>ACQ132_SFPGA_CONF_DONE_8) & ACQ132_SFPGA_CONF_8MASK)
		== ACQ132_SFPGA_CONF_8MASK;
	
	dbg(2, "conf 0x%08x done %d", conf, done);
	return done;
}

static inline int pulse_fifo_full(void)
{
	u32 ptr = *ACQ132_FIFSTAT & ACQ132_FIFSTAT_GPG_PTR;

	return !(ptr < ACQ132_FIFSTAT_GPG_PTR - 8);
}

extern int acq132_sfpga_get_rev(void);
static inline int acq132_supports_channel_vrange_switch(void)
{
	return acq132_sfpga_get_rev() >= 0x300;
}

void acq132_setScanList(int scanlen, u32 scanlist);

static inline int  acq132_getScanlistLen(void)
{
	return *ACQ132_SCAN_LIST_LEN + 1;
}

static inline int acq132_getScanlistEntry(int n)
{
	unsigned def = *ACQ132_SCAN_LIST_DEF;
	if (n){
		def >>= n*2;
	}
	return BANK_A + (def&0x3);
}

void acq132_set_obclock(int FDW, int RDW, int R, int Sx);
void acq132_set_channel_mask(u32 channel_mask);


int acq132_getRGM(void);
void acq132_setRGM(int enable);

struct OB_CLOCK_DEF {
	int demand;
	int actual;
	int FDW;
	int RDW;
	int R;
	int Sx;
};

extern struct OB_CLOCK_DEF ob_clock_def;

#include <asm/delay.h>

static inline void reset_gpg(void)
{
	u32 fifcon = *ACQ132_FIFCON;
	*ACQ132_FIFCON = fifcon |= ACQ132_FIFCON_GPG_RESET;
	udelay(5);
	*ACQ132_FIFCON = fifcon &= ~ACQ132_FIFCON_GPG_RESET;
	udelay(5);
}

/*
 * acq132 data is presented as 8-sample rows, in ROW_SAM blocks
 *
 * for block in BLOCKS
 *	for row in { A B C D }
 *		for ROW_SAM
 *			acq132_transform_row(to+ROW_OFF(row), from)
 *			update from
 *		update to
 *
 */

#define SWS		2		/* Sample Word Size */
#define MAX_ROWS	4

#define ROW_SAM		128
#define ROW_CHAN	8
#define ROW_WORDS	(ROW_SAM*ROW_CHAN)
#define ROW_SIZE	(ROW_WORDS*SWS)
#define ROW_LONGS	(ROW_SIZE/USS)

#define ROW_CHAN_SZ	(ROW_CHAN*SWS)
#define ROW_CHAN_LONGS  (ROW_CHAN_SZ/sizeof(unsigned))

/* but with channel masking, we have 2 more possibilities: */
#define ROW_CHAN2	4	/* two channels per fpga */
#define ROW_CHAN4	8	/* one channel per fpga */

#define LRCH	2
#define QUADCH	4

typedef unsigned ChannelBank[LRCH][QUADCH];
typedef unsigned * ChannelBankCursors[LRCH][QUADCH];

#define ROWCHAN2LRCH(ix)	((ix)>>2)
#define ROWCHAN2QUADCH(ix)	((ix)&(QUADCH-1))

int getChannelsInMask(int bank, ChannelBank channels);
const char* acq132_getChannelSpeedMask(void);
#endif	/*  __ACQ132_H__ */

