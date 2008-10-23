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

#define ACQ132_GATE_PULSE_FIFO  FPGA_REG(0xc0)
#define ACQ132_ADC_A		FPGA_REG(0x100)
#define ACQ132_ADC_B		FPGA_REG(0x200)
#define ACQ132_ADC_C		FPGA_REG(0x300)
#define ACQ132_ADC_D		FPGA_REG(0x400)

/* Dev 0..3 */
#define ACQ132_ADC_REGS(dev)		FPGA_REG((dev+1)*0x100)
#define ACQ132_ADC_REG(dev, reg)	FPGA_REG((dev+1)*0x100 + (reg))

#define ACQ132_ADC_TEST(dev)	ACQ132_ADC_REG(dev, 0x00)
#define ACQ132_ADC_RANGE(dev)	ACQ132_ADC_REG(dev, 0x04)
#define ACQ132_ADC_DECIM(dev)	ACQ132_ADC_REG(dev, 0x08)
#define ACQ132_ADC_SHIFT(dev)	ACQ132_ADC_REG(dev, 0x0c)

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

#define ACQ132_ADC_RANGE_L4	0x00080000
#define ACQ132_ADC_RANGE_L3	0x00040000
#define ACQ132_ADC_RANGE_L2	0x00020000
#define ACQ132_ADC_RANGE_L1	0x00010000

#define ACQ132_ADC_RANGE_R4	0x00000008
#define ACQ132_ADC_RANGE_R3	0x00000004
#define ACQ132_ADC_RANGE_R2	0x00000002
#define ACQ132_ADC_RANGE_R1	0x00000001

/* @todo one bit, all bits. multiple settings todo */
void acq132_set_adc_range(u32 channels);


static inline u32 acq132_get_adc_range(void)
{
	return *ACQ132_SYSCON & ACQ132_SYSCON_RANGE_HI? 0xffffffff: 0;
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
	u32 ptr = *ACQ132_FIFSTAT & ACQ132_FIFSTAT_PULSEP;

	return !(ptr < ACQ132_FIFSTAT_PULSEP - 8);
}

extern int acq132_sfpga_get_rev(void);
static inline int acq132_supports_channel_vrange_switch(void)
{
	return acq132_sfpga_get_rev() >= 0x300;
}

void acq132_set_obclock(int FDW, int RDW, int R, int Sx);

#endif	/*  __ACQ132_H__ */

