/* ------------------------------------------------------------------------- */
/* acq200-pulse.c driver for acq200 pulse gen output			     */
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

#ifndef __ACQ200_PULSE_C__
#define __ACQ200_PULSE_C__

/** pulse_timer_hz: if set, use auxtimer at this speed. else use old-style
 *  linux timer
 */
int pulse_timer_hz = 0;
module_param(pulse_timer_hz, int, 0666);


static int using_aux_timer_hz;

/*
 * pulse - capability to generate a pulse on timer
 */


static inline u32 _pulse_output_mask(struct PulseDef *pd)
{
	return (1U << ACQ200_DIOCON_OUTDAT_SHL) << pd->ibit;
}
static inline void _pulse_output_set_active(struct PulseDef *pd)
{
	if (pd->active_high){
		*ACQ200_DIOCON |= _pulse_output_mask(pd);
	}else{
		*ACQ200_DIOCON &= ~_pulse_output_mask(pd);
	}
	DG->stats.local_pulse_count++;
}
static inline void _pulse_output_set_inactive(struct PulseDef *pd)
{
	if (pd->active_high){
		*ACQ200_DIOCON &= ~_pulse_output_mask(pd);
	}else{
		*ACQ200_DIOCON |= _pulse_output_mask(pd);
	}
}


static inline int dj_linux(int delay)
{
	return delay*HZ/1000;
}

static void linuxtimer_pulse_run_after(
	struct PulseDefTimer* pdt, int deltams)
{
	pdt->timer_list.expires = jiffies + dj_linux(deltams);
	add_timer(&pdt->timer_list);
}

#include "iop321-auxtimer.h"

static inline int dj_aux(int delay)
{
	return delay;
}

static unsigned msec_togo;
static void _pulse_func(unsigned long clidata);

/* GTSR counts at 50MHz, or 50k per msec 
   50000 ~= 32768 * 3 / 2
   50000 ~= 262144 / 5 = 2**18 / 5
*/
#define GTSR2MS(gtsr)   ((((gtsr) >> 15) * 5 ) >> 3)
//#define GTSR2MS(gtsr) ((gtsr)/50000)

static void __auxtimer_func(unsigned long clidata)
/* @todo should use GTSR to count off  msecs */
{
	static unsigned my_msec_target;
	static unsigned gtsr1;
	

	if (msec_togo){			
		my_msec_target = msec_togo;
		msec_togo = 0;
		gtsr1 = *IOP321_GTSR;
	}else if (my_msec_target){
		unsigned gtsr2 = *IOP321_GTSR;
		unsigned dg;

		if (likely(gtsr2 > gtsr1)){
			dg = gtsr2 - gtsr1;
		}else{
			dg = 0xffffffffU - gtsr1 + gtsr2;
		}
		
		if (GTSR2MS(dg) > my_msec_target){
			my_msec_target = 0;
			_pulse_func(clidata);
		}
	}else{
		;
	}
}

static void auxtimer_pulse_run_after(struct PulseDefTimer* pdt, int deltams)
{
	msec_togo = dj_aux(deltams);
}

static struct AuxTimerClient atc;

static inline void start_auxtimer(unsigned long clidata)
{
	atc.func = __auxtimer_func;
	atc.clidata = clidata;
	msec_togo = 0;
	iop321_hookAuxTimer(&atc, using_aux_timer_hz);
}

static inline void stop_auxtimer(void) 
{
	iop321_hookAuxTimer(&atc, 0);
}


static void (*_pulse_run_after)(struct PulseDefTimer* pdt, int deltaj);
	
static void _pulse_func(unsigned long clidata)
{
	struct PulseDefTimer* pdt = (struct PulseDefTimer*)clidata;

	switch(pdt->state){
	case PDT_ACTIVE:
		_pulse_output_set_inactive(&pdt->pulse_def);
		if (--pdt->pulses_left > 0){
			pdt->state = PDT_DELAY;
			_pulse_run_after(pdt, pdt->pulse_def.delay_ms);
		}else{
			acq200_set_user_led(1, 0);
			pdt->state = PDT_WAITING_EVENT;
		}
		break;
	case PDT_DELAY:
		_pulse_output_set_active(&pdt->pulse_def);
		pdt->state = PDT_ACTIVE;
		_pulse_run_after(pdt, 10);
		break;
	default:
		;
	}
}

void pulse_init(struct PulseDefTimer* pdt)
{
	using_aux_timer_hz = pulse_timer_hz;
	if (using_aux_timer_hz){	
		_pulse_run_after = auxtimer_pulse_run_after;	
		start_auxtimer((unsigned long)pdt);
	}else{
		_pulse_run_after = linuxtimer_pulse_run_after;
		init_timer(&pdt->timer_list);
	}

	pdt->timer_list.function = _pulse_func;
	pdt->timer_list.data = (unsigned long)pdt;
	pdt->state = PDT_WAITING_EVENT;
}

void pulse_start(struct PulseDefTimer* pdt)
{
/* @@PGMWORKTODO ensure no timer running - this is a long shot */
	pdt->state = PDT_WAITING_EVENT;

#ifdef NOTCALLED_FROM_INTERRUPT
	if (!using_aux_timer_hz){
		del_timer_sync(&pdt->timer_list);
	}
#endif
	if (pdt->pulse_def.pulse_count){
		acq200_set_user_led(1, 1);
		pdt->pulses_left = pdt->pulse_def.pulse_count;
		pdt->state = PDT_DELAY;
		_pulse_run_after(pdt, pdt->pulse_def.start_delay);
	}
}

void pulse_close(struct PulseDefTimer* pdt)
{
	pdt->pulse_def.pulse_count = 0;
	if (!using_aux_timer_hz){
		del_timer(&pdt->timer_list);
	}else{
		stop_auxtimer();
	}
}




#endif /* __ACQ200_PULSE_C__ */

