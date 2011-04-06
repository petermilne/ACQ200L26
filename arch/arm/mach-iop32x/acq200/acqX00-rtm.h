/* ------------------------------------------------------------------------- */
/* acqX00-rtm.h register access for acq200 DIO32                             */
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

#ifndef __ACQX00_RTM_H__
#define __ACQX00_RTM_H__

static struct DIO32 {
	unsigned is_output;
	unsigned output_values;
	unsigned input_values;
} dio32;


#define RTM_REG(offset) (DIO_REG_TYPE((unsigned)ACQ200_EXTERNIO+(offset)))

#define RTM_DIO_CONTROL RTM_REG(0x00)
#define RTM_DIO_DIR_A   RTM_REG(0x04)
#define RTM_DIO_DIR_B   RTM_REG(0x08)
#define RTM_DIO_DATA_A  RTM_REG(0x0c)
#define RTM_DIO_DATA_B  RTM_REG(0x10)

#define RTM_REGCLR 0x01
#define RTM_DIREN  0x02

#define RTM_REVID(ctrl) (((ctrl)>>4)&0x0f)

#define SET_REG(reg, op, value) \
        *reg op value; \
        dbg(1, "0x%p %s 0x%08x", reg, #op, value)


static inline void init_inputs(void)
{
	SET_REG(RTM_DIO_CONTROL, =, RTM_REGCLR);
	
	SET_REG(RTM_DIO_DIR_A, =, ~(dio32.is_output&0xffff));
	SET_REG(RTM_DIO_DIR_B, =, ~(dio32.is_output >> 16));
	
	SET_REG(RTM_DIO_DATA_A, =, dio32.output_values&0xffff);
	SET_REG(RTM_DIO_DATA_B, =, dio32.output_values >> 16);

	SET_REG(RTM_DIO_CONTROL, =, RTM_REGCLR|RTM_DIREN);
}

static inline unsigned setDO32(u32 output_values)
{
	SET_REG(RTM_DIO_DATA_A, =, output_values&0xffff);
	SET_REG(RTM_DIO_DATA_B, =, output_values >> 16);	
	return output_values;
}

static inline void set_outputs(void)
{
	SET_REG(RTM_DIO_DIR_A, =, ~(dio32.is_output&0xffff));
	SET_REG(RTM_DIO_DIR_B, =, ~(dio32.is_output >> 16));

	setDO32(dio32.output_values);
}


static inline unsigned getDI32(void)
{
	return (*RTM_DIO_DATA_B << 16) | 
		(*RTM_DIO_DATA_A & 0x0ffff);
}
static inline unsigned read_inputs(void)
{
	dio32.input_values = getDI32();
	return dio32.input_values;
}

static inline void DIO_SET_OUTPUT1(int ib) 
{
	dio32.is_output |= 1 <<ib;
	dio32.output_values |= 1 <<ib;
}
static inline void DIO_SET_OUTPUT0(int ib) 
{
	dio32.is_output |= 1 << ib;
        dio32.output_values &= ~(1 << ib);
}
static inline void DIO_SET_INPUT(int ib)
{
	dio32.is_output &= ~(1 << ib);
}
static inline int DIO_IS_OUTPUT(int ib)
{
	return (dio32.is_output & (1 << ib)) != 0;
}
static inline int DIO_IS_OUTPUT1(int ib)
{
	return (dio32.output_values & (1 << ib)) != 0;
}
static inline int DIO_IS_INPUTH(int ib)
{
	return (dio32.input_values & (1 << ib)) != 0;
}

#define MAXDIOBIT 32


#endif                  /* __ACQX00_RTM_H__ */
