// Stands in for libopencm3's USART registers and functions. The registers are plain variables, and
// the functions do nothing except count how often the USART gets enabled, which is how a test can
// see that a line coding change has been applied.

#pragma once

#include <stdint.h>

#define USART1 1
#define USART6 6

extern uint32_t stub_usart_isr, stub_usart_icr, stub_usart_cr1, stub_usart_brr, stub_usart_tdr, stub_usart_rdr;
extern int stub_usart_enable_count;

#define USART1_RDR stub_usart_rdr
#define USART1_TDR stub_usart_tdr
#define USART_ISR(usart) stub_usart_isr
#define USART_ICR(usart) stub_usart_icr
#define USART_CR1(usart) stub_usart_cr1
#define USART_BRR(usart) stub_usart_brr

#define USART_ISR_TC (1u << 6)
#define USART_ISR_ORE (1u << 3)
#define USART_ISR_NF (1u << 2)
#define USART_ISR_FE (1u << 1)
#define USART_ISR_PE (1u << 0)
#define USART_ICR_ORECF (1u << 3)
#define USART_ICR_NCF (1u << 2)
#define USART_ICR_FECF (1u << 1)
#define USART_ICR_PECF (1u << 0)
#define USART_CR1_OVER8 (1u << 15)

#define USART_STOPBITS_1 0
#define USART_STOPBITS_1_5 1
#define USART_STOPBITS_2 2
#define USART_PARITY_NONE 0
#define USART_PARITY_ODD 1
#define USART_PARITY_EVEN 2
#define USART_MODE_TX_RX 0
#define USART_FLOWCONTROL_NONE 0

static inline void usart_set_mode(int, int) {}
static inline void usart_set_flow_control(int, int) {}
static inline void usart_enable_rx_dma(int) {}
static inline void usart_enable_tx_dma(int) {}
static inline void usart_enable(int) { stub_usart_enable_count++; }
static inline void usart_disable(int) {}
static inline void usart_set_databits(int, int) {}
static inline void usart_set_stopbits(int, int) {}
static inline void usart_set_parity(int, int) {}
