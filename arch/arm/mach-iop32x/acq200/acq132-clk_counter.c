/* acq132-clk_counter.c acq132 clock counter module                          */
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
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                   */
/* ------------------------------------------------------------------------- */


#define DTACQ_MACH 2
#define ACQ132
#define ACQ_IS_INPUT 1

#define MODEL_VERID							\
	"$Id: acq132-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $ B1012\n"

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#define ACQ132_VERID "$Revision: 1.13 $ " __DATE__ " " __TIME__

#define MTTR2	0x80

#include <linux/platform_device.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"

#include "acq200-fifo.h"
#include "acq132.h"

#include "boxcar32.h"


#define MAXCOUNT_SHFT	4
#define MAXCOUNT	(1<<MAXCOUNT_SHFT)

#define GTSR_ROLLOVER_MASK 0x00fffffff



static unsigned acq132_getClkCounter(void)
{
	return (*ACQ132_CLK_COUNTER) & ACQ132_CLK_COUNTER_COUNT;
}
static unsigned iop32x_getGTSR(void)
{
	return (*IOP321_GTSR) & GTSR_ROLLOVER_MASK;
}

int clk_dj = 1;
module_param(clk_dj, int, 0444);

int clk_dj_max = 1;
module_param(clk_dj_max, int, 0644);

struct CLKCOUNTER_STATE {
	struct BOXCAR32 data;

	unsigned (* getCount)(void);
	int prescale;	
	unsigned rollover;
	u32 previous;
	u32 result;
};


static struct CLKCOUNTER_STATE clk_probe = {
	.getCount = acq132_getClkCounter,
	.prescale = ACQ132_CLK_COUNTER_PRESCALE,
	.rollover = ACQ132_CLK_COUNTER_COUNT+1
};
static struct CLKCOUNTER_STATE clk_ref = {
	.getCount = iop32x_getGTSR,
	.prescale = 1,
	.rollover = GTSR_ROLLOVER_MASK+1
};


unsigned getHz(struct CLKCOUNTER_STATE *cs)
{
	return (cs->result * cs->prescale * HZ) >> MAXCOUNT_SHFT;
}

int acq132_showClkCounter(char *buf)
{
	unsigned probe_hz = getHz(&clk_probe);
	unsigned ref_hz = getHz(&clk_ref);

	if (clk_dj > 1){
		probe_hz /= clk_dj;
		ref_hz /= clk_dj;
	}
	return sprintf(buf, "%u %u %u\n", probe_hz, ref_hz, clk_dj);
}



static unsigned serviceClkCounter(struct CLKCOUNTER_STATE *st, unsigned cx)
{
	unsigned delta;

	if (cx < st->previous){
		delta = st->rollover - st->previous + cx;
	}else{
		delta = cx - st->previous;
	}
	st->previous = cx;

	st->result = boxcar32_process(&st->data, delta);

	return delta;
}

static struct timer_list clkcounter_timeout;
static int clkcounter_please_stop;

static void monitorClkCounter(unsigned long arg)
{
	unsigned cprobe = clk_probe.getCount();
	unsigned cref = clk_ref.getCount();
	unsigned delta;

	delta = serviceClkCounter(&clk_probe, cprobe);
	serviceClkCounter(&clk_ref, cref);

	if (!clkcounter_please_stop){
		if (delta < 4000 && clk_dj < clk_dj_max){
			++clk_dj;
		}else if (delta > 40000 && clk_dj > 1){
			--clk_dj;
		}
		clkcounter_timeout.expires = jiffies + clk_dj;
		add_timer(&clkcounter_timeout);
	}
}

static void box_init(struct BOXCAR32 *b32)
{
	if (b32->history != 0){
		boxcar32_free(b32);
	}
	boxcar32_init(b32, MAXCOUNT);
}	
void acq132_start_clkCounterMonitor(void)
{
	init_timer(&clkcounter_timeout);
	box_init(&clk_probe.data);
	box_init(&clk_ref.data);
	clkcounter_timeout.function = monitorClkCounter;
	clk_dj = 1;
	clkcounter_timeout.expires = jiffies + clk_dj;
	clkcounter_please_stop = 0;
	add_timer(&clkcounter_timeout);
}

void acq132_stop_clkCounterMonitor(void)
{
	clkcounter_please_stop = 1;
}
