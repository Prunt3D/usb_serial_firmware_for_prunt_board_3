//
// QSB USB Device Library for libopencm3
//
// Copyright (c) 2021 Manuel Bleichenbacher
// Copyright (c) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
// Licensed under LGPL License https://opensource.org/licenses/LGPL-3.0
// Derived from libopencm3 (https://github.com/libopencm3/libopencm3)
//

//
// Functions for generating serial number.
//

#include "qsb_device.h"
#include <string.h>


// Unique device ID register of the STM32F0
#define QSB_UID_BASE 0x1FFFF7AC


// Sample for UID:
// 42 00 4B 00 18 51 37 34 35 37 38 32
//
// Structure (see RM0351):
// Wafer x-position (2 byte, little endian)
// Wafer y-position (2 byte, little endian)
// Wafer number (1 byte)
// Lot number (7 bytes, ASCII)

static const char* hex_chars = "0123456789ABCDEF";

char qsb_serial_num[25];

const char* qsb_serial_num_init(void) {
    // get unique device ID
    const uint8_t* uid = (const uint8_t*)QSB_UID_BASE;

    // convert to ASCII string (hex)
    for (int i = 0; i < 12; i++) {
        qsb_serial_num[i * 2] = hex_chars[uid[i] & 0x0f];
        qsb_serial_num[i * 2 + 1] = hex_chars[(uid[i] >> 4) & 0x0f];
    }
    qsb_serial_num[24] = 0;

    return qsb_serial_num;
}
