// Stands in for libopencm3's DMA functions. The channels do nothing, and a test plays their part
// through these two variables.

#pragma once

#include <stdint.h>

#define DMA1 0
#define DMA_CCR_MSIZE_8BIT 0
#define DMA_CCR_PL_MEDIUM 0
#define DMA_TCIF 1
#define DMA_TEIF 2

// CNDTR of the receive channel
extern int stub_rx_cndtr;
// If set, called each time CNDTR of the receive channel has been read, so that a test can have bytes
// arrive straight after
extern void (*stub_rx_cndtr_read_hook)();
// Set if the transmit channel has finished
extern int stub_tx_dma_done;

static inline void dma_channel_reset(int, int) {}
static inline void dma_set_peripheral_address(int, int, uint32_t) {}
static inline void dma_set_read_from_memory(int, int) {}
static inline void dma_set_read_from_peripheral(int, int) {}
static inline void dma_enable_memory_increment_mode(int, int) {}
static inline void dma_enable_circular_mode(int, int) {}
static inline void dma_set_memory_size(int, int, int) {}
static inline void dma_set_peripheral_size(int, int, int) {}
static inline void dma_set_priority(int, int, int) {}
static inline void dma_enable_transfer_complete_interrupt(int, int) {}
static inline void dma_set_memory_address(int, int, uint32_t) {}
static inline void dma_set_number_of_data(int, int, int) {}
static inline void dma_enable_channel(int, int) {}
static inline void dma_disable_channel(int, int) {}
static inline int dma_get_number_of_data(int, int)
{
    int cndtr = stub_rx_cndtr;
    if (stub_rx_cndtr_read_hook)
        stub_rx_cndtr_read_hook();
    return cndtr;
}
static inline bool dma_get_interrupt_flag(int, int, int) { return stub_tx_dma_done != 0; }
static inline void dma_clear_interrupt_flags(int, int, int) { stub_tx_dma_done = 0; }
