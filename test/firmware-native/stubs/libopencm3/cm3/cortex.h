// Stands in for libopencm3's interrupt masking, as there are no interrupts here.

#pragma once

#include <stdint.h>

static inline uint32_t cm_mask_interrupts(uint32_t) { return 0; }
