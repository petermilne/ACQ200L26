/* ------------------------------------------------------------------------- */
/* acq200-time.c - timer routine for acq200				     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2007 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    Derived from:
 * arch/arm/plat-iop/time.c
 *
 * Timer code for IOP32x and IOP33x based systems
 *
 * Author: Deepak Saxena <dsaxena@mvista.com>	

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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#ifdef CONFIG_ARCH_IOP32X
#define IRQ_IOP3XX_TIMER0	IRQ_IOP32X_TIMER0
#else
#ifdef CONFIG_ARCH_IOP33X
#define IRQ_IOP3XX_TIMER0	IRQ_IOP33X_TIMER0
#endif
#endif

static unsigned long ticks_per_jiffy;
static unsigned long ticks_per_usec;
static unsigned long next_jiffy_time;

#define MAX_CATCHUP 4
static int insert_histo[MAX_CATCHUP+1];

unsigned long iop_gettimeoffset(void)
{
        unsigned long offset;

        offset = next_jiffy_time - *IOP3XX_TU_TCR1;

        return offset / ticks_per_usec;
}

static unsigned long iop_tick_rate;
unsigned long get_iop_tick_rate(void)
{
	return iop_tick_rate;
}
EXPORT_SYMBOL(get_iop_tick_rate);


extern void iop321_auxtimer_func(void);

static irqreturn_t
iop3xx_timer_interrupt(int irq, void *dev_id)
{
	int tick_insert = 0;

	/* intack */
	write_tisr(1);

	iop321_auxtimer_func();

        while ((signed long)(next_jiffy_time - read_tcr1())
                                                        >= ticks_per_jiffy) {

	        write_seqlock(&xtime_lock);
		timer_tick();
	        write_sequnlock(&xtime_lock);

		if (++tick_insert >= MAX_CATCHUP){
			next_jiffy_time = read_tcr1();
			break;
		}else{
			next_jiffy_time -= ticks_per_jiffy;
		}
        }

	insert_histo[min(tick_insert, MAX_CATCHUP)]++;

        return IRQ_HANDLED;
}

int iop3xx_report_ticks(char* buf, int maxbuf)
{
	int cursor = 0;
	int bin;

	cursor += snprintf(buf, maxbuf, "[");
	
	for (bin = 0; bin <= MAX_CATCHUP; ++bin){
		cursor += snprintf(buf+cursor, maxbuf-cursor,
				   "%6d ", insert_histo[bin]);

		if (cursor > maxbuf-4){
			break;
		}
	}
	cursor += snprintf(buf+cursor, maxbuf-cursor, "]\n");
	return cursor;
}


static struct irqaction iop3xx_timer_irq = {
	.name		= "IOP3XX Timer Tick",
	.handler	= iop3xx_timer_interrupt,
	.flags		= IRQF_DISABLED | IRQF_TIMER,
};

#define AUX_HZ	100

static int aux_hz = AUX_HZ;

unsigned acq200_setAuxClock(unsigned hz)
/* we can run the tick any speed > Hz, because the ISR has absolute ref in tmr1
 */
{
	u32 timer_load;

	if (hz < HZ){
		hz = HZ;
	}

	timer_load = ticks_per_jiffy * HZ/hz;
	write_trr0(timer_load);	
	return hz;
}

unsigned acq200_getAuxClock(void)
{
	return aux_hz;
}

void __init acq200_init_time(unsigned long tick_rate)
{
	u32 timer_ctl;
	u32 timer_load;

	ticks_per_jiffy = (tick_rate + HZ/2) / HZ;
	ticks_per_usec = tick_rate / 1000000;
	next_jiffy_time = 0xffffffff;

	iop_tick_rate = tick_rate;
	printk("iop3xx_init_time tick_rate %lu ticks_per_jiffy %lu usec %lu\n",
	       tick_rate, ticks_per_jiffy, ticks_per_usec);

	timer_ctl = IOP_TMR_EN | IOP_TMR_PRIVILEGED |
			IOP_TMR_RELOAD | IOP_TMR_RATIO_1_1;

	timer_load = ticks_per_jiffy * HZ/AUX_HZ;

	/*
	 * We use timer 0 for our timer interrupt, and timer 1 as
	 * monotonic counter for tracking missed jiffies.
	 */
	write_trr0(timer_load);
	write_tmr0(timer_ctl);
	write_trr1(0xffffffff);
	write_tmr1(timer_ctl);

	setup_irq(IRQ_IOP3XX_TIMER0, &iop3xx_timer_irq);
}

