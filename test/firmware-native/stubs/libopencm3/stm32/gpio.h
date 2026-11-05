// Stands in for libopencm3's GPIO functions, which do nothing here.

#pragma once

#define GPIOA 0
#define GPIO9 1
#define GPIO10 2
#define GPIO12 4
#define GPIO_AF1 1
#define GPIO_MODE_AF 2
#define GPIO_PUPD_PULLUP 1
#define GPIO_OTYPE_PP 0
#define GPIO_OSPEED_MED 1

static inline void gpio_set(int, int) {}
static inline void gpio_set_af(int, int, int) {}
static inline void gpio_mode_setup(int, int, int, int) {}
static inline void gpio_set_output_options(int, int, int, int) {}
