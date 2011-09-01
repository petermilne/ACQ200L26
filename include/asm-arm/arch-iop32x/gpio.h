/*
 * gpio.h
 *
 *  Created on: May 26, 2011
 *      Author: pgm
 */

#ifndef GPIO_H_
#define GPIO_H_

extern int gpio_get_value(unsigned gpio);
extern void gpio_set_value(unsigned gpio, int value);


static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{

}

static inline int gpio_direction_input(unsigned gpio)
{
	return gpio_get_value(gpio);
}
static inline int gpio_direction_output(unsigned gpio, int value)
{
	gpio_set_value(gpio, value);
	return value;
}

#endif /* GPIO_H_ */
