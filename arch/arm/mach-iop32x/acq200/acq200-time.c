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

static int insert_histo[4];

unsigned long iop3xx_gettimeoffset(void)
{
        unsigned long offset;

        offset = next_jiffy_time - *IOP3XX_TU_TCR1;

        return offset / ticks_per_usec;
}

extern void iop321_auxtimer_func(void);

static irqreturn_t
iop3xx_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	int tick_insert = 0;
        write_seqlock(&xtime_lock);


        iop3xx_cp6_enable();
        asm volatile("mcr p6, 0, %0, c6, c1, 0" : : "r" (1));
        iop3xx_cp6_disable();

	iop321_auxtimer_func();

        while ((signed long)(next_jiffy_time - *IOP3XX_TU_TCR1)
                                                        >= ticks_per_jiffy) {
		timer_tick(regs);
		if (++tick_insert > 2){
                       next_jiffy_time = *IOP3XX_TU_TCR1 - ticks_per_jiffy;
                       break;
		}else{
			next_jiffy_time -= ticks_per_jiffy;
		}
        }

        write_sequnlock(&xtime_lock);

	insert_histo[min(tick_insert, 3)]++;

        return IRQ_HANDLED;
}

int iop3xx_report_ticks(char* buf, int maxbuf)
{
	return snprintf(buf, maxbuf, 
			"[%6d,%6d,%6d,%6d]\n",
			insert_histo[0],
			insert_histo[1],
			insert_histo[2],
			insert_histo[3]
			);
}


static struct irqaction iop3xx_timer_irq = {
	.name		= "IOP3XX Timer Tick",
	.handler	= iop3xx_timer_interrupt,
	.flags		= IRQF_DISABLED | IRQF_TIMER,
};

#define AUX_HZ	1000


void __init iop3xx_init_time(unsigned long tick_rate)
{
	u32 timer_ctl;
	u32 timer_load;

	ticks_per_jiffy = (tick_rate + HZ/2) / HZ;
	ticks_per_usec = tick_rate / 1000000;
	next_jiffy_time = 0xffffffff;

	printk("iop3xx_init_time tick_rate %lu ticks_per_jiffy %lu usec %lu\n",
	       tick_rate, ticks_per_jiffy, ticks_per_usec);

	timer_ctl = IOP3XX_TMR_EN | IOP3XX_TMR_PRIVILEGED |
			IOP3XX_TMR_RELOAD | IOP3XX_TMR_RATIO_1_1;

	timer_load = ticks_per_jiffy * HZ/AUX_HZ;

	/*
	 * We use timer 0 for our timer interrupt, and timer 1 as
	 * monotonic counter for tracking missed jiffies.
	 */
	iop3xx_cp6_enable();
	asm volatile("mcr p6, 0, %0, c4, c1, 0" : : "r" (timer_load));
	asm volatile("mcr p6, 0, %0, c0, c1, 0" : : "r" (timer_ctl));
	asm volatile("mcr p6, 0, %0, c5, c1, 0" : : "r" (0xffffffff));
	asm volatile("mcr p6, 0, %0, c1, c1, 0" : : "r" (timer_ctl));
	iop3xx_cp6_disable();

	setup_irq(IRQ_IOP3XX_TIMER0, &iop3xx_timer_irq);
}

