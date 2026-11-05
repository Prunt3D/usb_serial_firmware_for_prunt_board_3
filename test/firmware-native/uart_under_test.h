// Compiles the real uart.cpp against the stubs, with its private members open so that a test can
// write to the receive buffer as the DMA controller would. Include this once, instead of uart.h.

#pragma once

#include <stdint.h>

#define private public
#include "uart.h"
#undef private

int stub_rx_cndtr = UART_RX_BUF_LEN;
int stub_tx_dma_done = 0;
int stub_usart_enable_count = 0;
uint32_t stub_usart_isr = 1u << 6; // only TC, as the transmitter is idle
uint32_t stub_usart_icr, stub_usart_cr1, stub_usart_brr, stub_usart_tdr, stub_usart_rdr;

#include "uart.cpp"
