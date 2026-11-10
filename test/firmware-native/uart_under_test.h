// Compiles the real uart.cpp against the stubs, with its private members open so that a test can
// write to the receive buffer as the DMA controller would. Include this once, instead of uart.h.

#pragma once

#include <stdint.h>

#define private public
#include "uart.h"
#undef private

int stub_rx_cndtr = UART_RX_BUF_LEN;
void (*stub_rx_cndtr_read_hook)() = nullptr;
int stub_tx_dma_done = 0;
int stub_usart_enable_count = 0;
uint32_t stub_usart_isr = 1u << 6; // only TC, as the transmitter is idle
uint32_t stub_usart_icr, stub_usart_cr1, stub_usart_brr, stub_usart_tdr, stub_usart_rdr;

// If set, called once memcpy() has copied its first byte, so that a test can have bytes arrive
// while the receive buffer is being copied
void (*stub_memcpy_hook)() = nullptr;

#include <string.h>

static void *stub_memcpy(void *dst, const void *src, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        ((uint8_t *)dst)[i] = ((const uint8_t *)src)[i];
        if (i == 0 && stub_memcpy_hook)
            stub_memcpy_hook();
    }
    return dst;
}

#define memcpy stub_memcpy
#include "uart.cpp"
#undef memcpy
