/*
 * USB Serial
 * 
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 * 
 * Common declarations
 */

#pragma once

#include <libopencmsis/core_cm3.h>
#include <algorithm>

/**
 * @brief Initializes common services
 */
void common_init();

/**
 * @brief Gets the time.
 * 
 * @return number of milliseconds since a fixed time in the past
 */
uint32_t millis();

/**
 * @brief Delays execution (busy wait)
 * @param ms delay length, in milliseconds
 */
void delay(uint32_t ms);
