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

static int bytes_on_next_read, reads_before_arrival;

// Has bytes arrive straight after a later read of CNDTR, once
static void write_after_read()
{
    if (reads_before_arrival-- > 0)
        return;
    stub_rx_cndtr_read_hook = nullptr;
    dma_write(bytes_on_next_read);
}

// Has bytes arrive while memcpy() runs, once
static void write_during_copy()
{
    stub_memcpy_hook = nullptr;
    dma_write(bytes_on_next_read);
}

enum class arrival
{
    after_position_read, // just after the copy first reads where the DMA is
    after_first_byte,    // just after it reads that again, once it has read the first byte
    during_copy,         // while memcpy() runs, once it has read the first byte
};

// Fills the ring until `unread` bytes are waiting, then copies a packet with `arriving` bytes arriving
// at the given point. Returns whether an overrun was reported by the next poll, and sets `intact` to
// whether the packet holds the bytes that were waiting.
static bool copy_while_receiving(int unread, int arriving, arrival when = arrival::after_position_read,
    bool *intact = nullptr)
{
    uint8_t packet[64];
    uart.copy_rx_data(packet, sizeof(packet));
    uart.poll();
    uart.take_errors();
    consumed = written;
    while (uart.rx_data_len() > 0) {
        uart.copy_rx_data(packet, sizeof(packet));
        uart.poll();
    }
    uart.take_errors();

    dma_write(unread);
    uart.poll();
    bytes_on_next_read = arriving;
    if (when == arrival::during_copy) {
        stub_memcpy_hook = write_during_copy;
    } else {
        reads_before_arrival = when == arrival::after_first_byte ? 1 : 0;
        stub_rx_cndtr_read_hook = write_after_read;
    }
    size_t len = uart.copy_rx_data(packet, sizeof(packet));
    uart.poll();
    if (intact) {
        *intact = len == sizeof(packet);
        for (size_t i = 0; i < len; i++)
            if (packet[i] != byte_at(consumed + i))
                *intact = false;
    }
    return (uart.take_errors() & (uint16_t)uart_error::overrun) != 0;
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

    // Bytes arriving while a copy runs. With the ring nearly full, two of them overwrite the first
    // byte to be copied, and one doesn't.
    bool overrun_2_of_1023 = copy_while_receiving(UART_RX_BUF_LEN - 1, 2);
    bool overrun_1_of_1023 = copy_while_receiving(UART_RX_BUF_LEN - 1, 1);
    bool overrun_2_of_1021 = copy_while_receiving(UART_RX_BUF_LEN - 3, 2);
    printf("%s  two bytes arriving during a copy from a ring 1 short of full are reported as an overrun\n",
        overrun_2_of_1023 ? "pass" : "FAIL");
    printf("%s  one byte arriving then isn't\n", !overrun_1_of_1023 ? "pass" : "FAIL");
    printf("%s  two bytes arriving during a copy from a ring 3 short of full aren't\n",
        !overrun_2_of_1021 ? "pass" : "FAIL");

    // Once the copy has read the first byte, the DMA overwriting it loses nothing. The DMA can't get
    // past it before the copy does, so the model doesn't try that.
    bool intact_after_first = false, intact_during = false;
    bool overrun_after_first = copy_while_receiving(UART_RX_BUF_LEN - 1, 2, arrival::after_first_byte,
        &intact_after_first);
    bool overrun_during = copy_while_receiving(UART_RX_BUF_LEN - 1, 2, arrival::during_copy, &intact_during);
    printf("%s  two bytes arriving once the copy has read the first byte aren't reported as an overrun\n",
        !overrun_after_first ? "pass" : "FAIL");
    printf("%s    and the packet holds the bytes that were waiting\n", intact_after_first ? "pass" : "FAIL");
    printf("%s  two bytes arriving while memcpy() runs aren't reported as an overrun\n",
        !overrun_during ? "pass" : "FAIL");
    printf("%s    and the packet holds the bytes that were waiting\n", intact_during ? "pass" : "FAIL");

    return silent_losses != 0 || false_overruns != 0 || !overrun_2_of_1023 || overrun_1_of_1023 || overrun_2_of_1021
        || overrun_after_first || !intact_after_first || overrun_during || !intact_during;
}
