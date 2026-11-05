// Stands in for libopencm3's RCC functions, which do nothing here.

#pragma once

#include <stdint.h>

#define RCC_USART1 1
#define RCC_GPIOA 2
#define RCC_DMA 3
#define RCC_USB 4
#define RCC_SYSCFG_COMP 5
#define RCC_HSI48 6
#define RST_USB 7

static uint32_t rcc_apb1_frequency = 48000000;
#define rcc_apb2_frequency rcc_apb1_frequency

static inline void rcc_periph_clock_enable(int) {}
static inline void rcc_periph_reset_pulse(int) {}
static inline void rcc_osc_on(int) {}
static inline void rcc_wait_for_osc_ready(int) {}
static inline void rcc_set_usbclk_source(int) {}
