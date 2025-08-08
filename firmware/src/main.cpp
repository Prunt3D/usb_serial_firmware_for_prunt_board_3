/*
 * USB Serial
 * 
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 * 
 * Main program
 */

#include "common.h"
#include "hardware.h"
#include "usb_conf.h"
#include "usb_serial.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/desig.h>


static void gpio_setup()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOF);

	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0 | GPIO1 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO13 | GPIO14 | GPIO15);
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO13 | GPIO14 | GPIO15);
	gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0 | GPIO1);
}

int main()
{
	common_init();
	gpio_setup();
	qsb_serial_num_init();
	usb_serial.init();

	bool connected = false;

	while (1)
	{
		usb_serial.poll();
	}

	return 0;
}
