// Drives the receive path of the real uart.cpp with a model of the circular DMA. Each byte's value
// comes from its position in the stream, so a gap shows up as a wrong value. Fails on data lost
// without an overrun being reported, or on an overrun reported when the ring wasn't overwritten.

#include <stdio.h>
#include <stdlib.h>

#include "uart_under_test.h"

static uint64_t written, consumed;
static int dma_pos;

// 251 doesn't divide the buffer size, so skipping a whole number of laps still changes the values.
static uint8_t byte_at(uint64_t pos)
{
    return (uint8_t)(pos % 251);
}

static void dma_write(int count)
{
    for (int i = 0; i < count; i++) {
        uart.rx_buf[dma_pos] = byte_at(written++);
        dma_pos = (dma_pos + 1) % UART_RX_BUF_LEN;
    }
    stub_rx_cndtr = UART_RX_BUF_LEN - dma_pos;
}

int main()
{
    unsigned long silent_losses = 0, false_overruns = 0, overruns = 0, copies = 0;

    uart.enable();
    srand(12345);

    for (long step = 0; step < 4000000; step++) {
        // Phases: the reader keeps up, the reader stalls, and the reader stays just behind the DMA
        // so that the ring stays nearly full. The order within a step follows the main loop: the
        // overrun check in uart.poll(), then a little later the copy.
        int phase = (step / 4000) % 3;
        int copy_percent = phase == 0 ? 25 : phase == 1 ? 0 : 9;

        dma_write(rand() % 13);

        uint64_t unread = written - consumed;
        uart.poll();
        if (uart.take_errors() & (uint16_t)uart_error::overrun) {
            overruns++;
            if (unread < UART_RX_BUF_LEN)
                false_overruns++;
            consumed = written;
        }

        dma_write(rand() % 3);

        if (rand() % 100 < copy_percent) {
            uint8_t packet[64];
            unread = written - consumed;
            size_t len = uart.copy_rx_data(packet, sizeof(packet));
            copies++;

            if (uart.take_errors() & (uint16_t)uart_error::overrun) {
                overruns++;
                if (unread < UART_RX_BUF_LEN)
                    false_overruns++;
                consumed = written - len;
            } else if (unread >= UART_RX_BUF_LEN && len > 0) {
                // Data has been lost, and the copy has removed the evidence
                silent_losses++;
                consumed = written - len;
            }

            for (size_t i = 0; i < len; i++) {
                if (packet[i] != byte_at(consumed + i)) {
                    silent_losses++;
                    break;
                }
            }
            consumed += len;
        }
    }

    printf("%lu copies, %lu overruns, %lu silent losses, %lu false overruns\n", copies, overruns, silent_losses,
        false_overruns);

    // Make sure the phases really did exercise what they're there for
    if (overruns == 0 || copies == 0) {
        printf("FAILED: the run never overran or never copied\n");
        return 1;
    }

    return silent_losses != 0 || false_overruns != 0;
}
