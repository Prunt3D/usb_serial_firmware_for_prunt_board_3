/*
 * USB Serial
 * 
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 * 
 * Common functions
 */

#include "common.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

static volatile uint32_t millis_count;

uint32_t millis()
{
	return millis_count;
}

void delay(uint32_t ms)
{
	// Unsigned, so that the subtraction wraps instead of overflowing.
	uint32_t start_time = millis_count;
	while (millis_count - start_time < ms)
		;
}

void common_init()
{
	// Run from HSI48 because CRS trims it to the host's USB SOF packets, whereas HSI drifts with
	// temperature. The USART uses this clock too, and its baud rate is only as accurate as the clock.
	rcc_clock_setup_in_hsi48_out_48mhz();

	// Erasing flash for bootloader entry needs HSI, even though nothing runs from it now.
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	// Initialize SysTick

	// Interrupt every 1ms
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(rcc_ahb_frequency / 1000 - 1);

	// Enable and start
	systick_interrupt_enable();
	systick_counter_enable();
}

// System tick timer interrupt handler
extern "C" void sys_tick_handler()
{
	millis_count++;
}

