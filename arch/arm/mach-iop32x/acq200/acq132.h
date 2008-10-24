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
#define ACQ132_FIFSTAT  ACQ196_FIFSTAT
#define ACQ132_SYSCON	ACQ196_SYSCON_ADC
/* custom regs */
#define ACQ132_SFPGA_CONF	FPGA_REG(0x10)
#define ACQ132_ICS527		FPGA_REG(0x18)
#define ACQ132_SCAN		FPGA_REG(0x4c)



#define ACQ132_GATE_PULSE_FIFO  FPGA_REG(0xc0)
#define ACQ132_ADC_A		FPGA_REG(0x100)
#define ACQ132_ADC_B		FPGA_REG(0x200)
#define ACQ132_ADC_C		FPGA_REG(0x300)
#define ACQ132_ADC_D		FPGA_REG(0x400)

/* Dev 0..3 */
#define ACQ132_ADC_REGS(dev)		FPGA_REG((dev+1)*0x100)
#define ACQ132_ADC_REG(dev, reg)	FPGA_REG((dev+1)*0x100 + (reg))

#define ACQ132_ADC_TEST(dev)	ACQ132_ADC_REG(dev, 0x00)
#define ACQ132_ADC_CTRL(dev)	ACQ132_ADC_REG(dev, 0x04)
#define ACQ132_ADC_RANGE(dev)	ACQ132_ADC_REG(dev, 0x08)
#define ACQ132_ADC_DECIM(dev)	ACQ132_ADC_REG(dev, 0x0c)
#define ACQ132_ADC_SHIFT(dev)	ACQ132_ADC_REG(dev, 0x10)

#define ACQ132_ADC_OSAM(dev)	ACQ132_ADC_REG(dev, 0x0c)

#define BANK_A	0
#define BANK_B  1
#define BANK_C  2
#define BANK_D  3

#define BDR_MAGIC	0xdeadbeef
/* custom bits */
#define ACQ132_SYSCON_REV_RESET 0x80000000
#define ACQ132_SYSCON_RANGE_HI	0x00800000


#define ACQ132_FIFSTAT_NE	0x00020000
#define ACQ132_FIFSTAT_PULSEP	0x0001ff00


#define ACQ132_SFPGA_CONF_PROG		31
#define ACQ132_SFPGA_CONF_INIT_LR	29	
#define ACQ132_SFPGA_CONF_DONE_8	20
#define ACQ132_SFPGA_CONF_BUSY		17
#define ACQ132_SFPGA_CONF_UPDATE	16
#define ACQ132_SFPGA_CONF_DATA		0x0000ffff

#define ACQ132_SFPGA_CONF_LRMASK	0x3
#define ACQ132_SFPGA_CONF_8MASK		0xff


#define ACQ132_ICS527_FDW	0x003f0000
#define ACQ132_ICS527_RDW	0x00003f00
#define ACQ132_ICS527_S1S0	0x000000c0
#define ACQ132_ICS527_CLKDIV	0x00000007


#define ACQ132_SCAN_S4	0
#define ACQ132_SCAN_S3  2
#define ACQ132_SCAN_S2	4
#define ACQ132_SCAN_S1	6

#define SxA	0x0
#define SxB	0x1
#define SxC	0x2
#define SxD	0x3

#define ACQ132_ADC_CTRL_LMSHFT	20
#define ACQ132_ADC_CTRL_RMSHFT  4
#define ACQ132_ADC_CTRL_ACQEN	0x00010001

#define ACQ132_ADC_RANGE_L4	0x00080000
#define ACQ132_ADC_RANGE_L3	0x00040000
#define ACQ132_ADC_RANGE_L2	0x00020000
#define ACQ132_ADC_RANGE_L1	0x00010000

#define ACQ132_ADC_RANGE_R4	0x00000008
#define ACQ132_ADC_RANGE_R3	0x00000004
#define ACQ132_ADC_RANGE_R2	0x00000002
#define ACQ132_ADC_RANGE_R1	0x00000001

#define ACQ132_ADC_OSAM_L_NACC_SHL	28
#define ACQ132_ADC_OSAM_L_SHIFT		24
#define ACQ132_ADC_OSAM_R_NACC_SHL	12
#define ACQ132_ADC_OSAM_R_SHIFT		 8


static inline void acq132_adc_set_osam(int dev, int shl, int nacc)
{
	u32 osam = *ACQ132_ADC_OSAM(dev);
	u32 field = 0;
	osam &= ~(0xf << shl);

	switch(nacc){
	case 1:
	default:
		break;
	case 2:
		field = 1; break;
	case 4:
		field = 2; break;
	case 8:
		field = 3; break;
	case 16:
		field = 4; break;
	}

	osam |= field << shl;
	*ACQ132_ADC_OSAM(dev) = osam;
}


#define SHIFT_M1 0xf
#define SHIFT_M2 0xe
#define SHIFT_0	 0x0
#define SHIFT_P1 0x1
#define SHIFT_P2 0x2

static inline void acq132_adc_set_shift(int dev, int shl, int shift)
{
	u32 osam = *ACQ132_ADC_OSAM(dev);
	u32 field = 0;
	osam &= ~(0xf << shl);

	switch(shift){
	case -2:
	default:
		field = SHIFT_M2; break;
	case -1:
		field = SHIFT_M1; break;
	case 1:
		field = SHIFT_P1; break;
	case 2:
		field = SHIFT_P2; break;
	}

	osam |= field << shl;
	*ACQ132_ADC_OSAM(dev) = osam;
}

#define ACQ132_SET_OSAM_X_NACC(dev, lr, nacc, shift) do {	       \
	acq132_adc_set_osam(dev, ACQ132_ADC_OSAM_R_NACC_SHL+lr, nacc); \
	acq132_adc_set_shift(dev, ACQ132_ADC_OSAM_R_SHIFT+lr, shift); \
	} while(0)


/* @todo one bit, all bits. multiple settings todo */
void acq132_set_adc_range(u32 channels);


u32 acq132_get_adc_range(void);


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
	u32 ptr = *ACQ132_FIFSTAT & ACQ132_FIFSTAT_PULSEP;

	return !(ptr < ACQ132_FIFSTAT_PULSEP - 8);
}

extern int acq132_sfpga_get_rev(void);
static inline int acq132_supports_channel_vrange_switch(void)
{
	return acq132_sfpga_get_rev() >= 0x300;
}

void acq132_set_obclock(int FDW, int RDW, int R, int Sx);
void acq132_set_channel_mask(u32 channel_mask);
#endif	/*  __ACQ132_H__ */

