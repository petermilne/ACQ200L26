/*
 * arch/arm/mach-iop3xx/acq200-leds.c
 *
 * LEDs support for the ACQ200 intelligent digitizer platform
 *
 * Author: Peter Milne <Peter.Milne@d-tacq.com>
 * Copyright (C) 2003 Peter Milne
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * slaveish copy of:
 *
 *  linux/arch/arm/mach-footbridge/ebsa285-leds.c
 *
 *  Copyright (C) 1998-1999 Russell King
 *
 * The EBSA-285 uses the leds as follows:
 *  - Green - toggles state every 50 timer interrupts
 *  - Amber - On if system is not idle
 *  - Red   - currently unused
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/system.h>

#include <asm/arch/iop321.h>

#define LED_STATE_ENABLED	1
#define LED_STATE_CLAIMED	2
static char led_state;
static char hw_led_state;

static spinlock_t leds_lock = SPIN_LOCK_UNLOCKED;


#define XBUS_LED_RED   ACQ200_LED3
#define XBUS_LED_GREEN ACQ200_LED2
#define XBUS_LED_AMBER ACQ200_LED1


static char user_led_flash;
#define USER_LED_3     ACQ200_LED3
#define USER_LED_4     ACQ200_LED4

#define ACQ200_LED3_ON  led_red_on
#define ACQ200_LED3_OFF led_red_off
#define ACQ200_LED4_ON  led_blue_on
#define ACQ200_LED4_OFF led_blue_off

static void acq200_leds_event(led_event_t evt)
{
	unsigned long flags;

	spin_lock_irqsave(&leds_lock, flags);

	switch (evt) {
	case led_start:
		hw_led_state = XBUS_LED_GREEN;
#ifndef CONFIG_LEDS_CPU
		hw_led_state |= XBUS_LED_AMBER;
#endif
		led_state |= LED_STATE_ENABLED;
		break;

	case led_stop:
		led_state &= ~LED_STATE_ENABLED;
		break;

	case led_claim:
		led_state |= LED_STATE_CLAIMED;
		hw_led_state = XBUS_LED_GREEN | XBUS_LED_AMBER;
		break;

	case led_release:
		led_state &= ~LED_STATE_CLAIMED;
		hw_led_state = XBUS_LED_GREEN | XBUS_LED_AMBER;
		break;

#ifdef CONFIG_LEDS_TIMER
	case led_timer:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state ^= XBUS_LED_GREEN;
		if (user_led_flash & ACQ200_LED3){
			hw_led_state ^= ACQ200_LED3;
		}
		if (user_led_flash & ACQ200_LED4){
			hw_led_state ^= ACQ200_LED4;
		}
		break;
#endif

#ifdef CONFIG_LEDS_CPU
	case led_idle_start:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state |= XBUS_LED_AMBER;
		break;

	case led_idle_end:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state &= ~XBUS_LED_AMBER;
		break;
#endif

	case led_halted:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state &= ~XBUS_LED_RED;
		break;

	case led_green_on:
		if (led_state & LED_STATE_CLAIMED)
			hw_led_state &= ~XBUS_LED_GREEN;
		break;

	case led_green_off:
		if (led_state & LED_STATE_CLAIMED)
			hw_led_state |= XBUS_LED_GREEN;
		break;

	case led_amber_on:
		if (led_state & LED_STATE_CLAIMED)
			hw_led_state &= ~XBUS_LED_AMBER;
		break;

	case led_amber_off:
		if (led_state & LED_STATE_CLAIMED)
			hw_led_state |= XBUS_LED_AMBER;
		break;

	case ACQ200_LED3_ON:
		led_state |= LED_STATE_ENABLED;
		user_led_flash &= ~ACQ200_LED3;
		hw_led_state |= ACQ200_LED3;
		break;
	case ACQ200_LED4_ON:
		led_state |= LED_STATE_ENABLED;
		user_led_flash &= ~ACQ200_LED4;
		hw_led_state |= ACQ200_LED4;
		break;
	case ACQ200_LED3_OFF:
		led_state |= LED_STATE_ENABLED;
		user_led_flash &= ~ACQ200_LED3;
		hw_led_state &= ~ACQ200_LED3;
		break;
	case ACQ200_LED4_OFF:
		led_state |= LED_STATE_ENABLED;
		user_led_flash &= ~ACQ200_LED4;
		hw_led_state &= ~ACQ200_LED4;
		break;
	default:
		break;
	}

	if  (led_state & LED_STATE_ENABLED){
		u32 gpod = *IOP321_GPOD;

		gpod &= ~ACQ200_LEDS;
		gpod |= hw_led_state;

		*IOP321_GPOD = gpod;
	}

	spin_unlock_irqrestore(&leds_lock, flags);
}



void acq200_set_user_led(int led4, int on)
{
	char bit = led4? ACQ200_LED4: ACQ200_LED3;

	switch(on)
	{
	case 1:
		user_led_flash &= ~bit;
		leds_event(led4? ACQ200_LED4_ON: ACQ200_LED3_ON);
		break;
	case -1:
		user_led_flash |= bit;
		break;
	case 0:
	default:
		leds_event(led4? ACQ200_LED4_OFF: ACQ200_LED3_OFF);
	}
}

EXPORT_SYMBOL_GPL(acq200_set_user_led);

static int __init leds_init(void)
{
	if (machine_is_acq200()|| machine_is_acq100() || machine_is_acq132()){
		leds_event = acq200_leds_event;

		*IOP321_GPOE &= ~ACQ200_LEDS;

		leds_event(led_start);
	}
	return 0;
}




__initcall(leds_init);
