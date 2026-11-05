// Stands in for libopencm3's RCC functions, which do nothing here.

#pragma once

#include <stdint.h>

#define RCC_USART1 1
#define RCC_GPIOA 2
#define RCC_DMA 3

static uint32_t rcc_apb1_frequency = 48000000;
#define rcc_apb2_frequency rcc_apb1_frequency

static inline void rcc_periph_clock_enable(int) {}
