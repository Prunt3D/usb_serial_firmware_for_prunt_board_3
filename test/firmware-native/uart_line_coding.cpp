// Checks how line coding changes behave in the real uart.cpp.

#include "check.h"
#include "uart_under_test.h"

static int dma_pos;

static void receive_byte(uint8_t byte)
{
    uart.rx_buf[dma_pos] = byte;
    dma_pos = (dma_pos + 1) % UART_RX_BUF_LEN;
    stub_rx_cndtr = UART_RX_BUF_LEN - dma_pos;
}

static void restart()
{
    dma_pos = 0;
    stub_rx_cndtr = UART_RX_BUF_LEN;
    uart.enable();
    uart.poll();
}

int main()
{
    uint8_t received[1];
    static const uint8_t tx_data[4] = {1, 2, 3, 4};

    // The USART stays on the old settings while data queued before the request is still going out
    restart();
    uart.transmit(tx_data, sizeof(tx_data));
    int enables = stub_usart_enable_count;
    uart.set_coding(9600, 7, uart_stopbits::_1_0, uart_parity::even);
    uart.poll();
    check("a change to 7E1 waits while TX drains", stub_usart_enable_count == enables);
    check("GET_LINE_CODING reports it straight away", uart.databits() == 7);
    receive_byte(0xC3);
    uart.copy_rx_data(received, 1);
    check("an 8-bit byte stays whole until the change is applied", received[0] == 0xC3);

    stub_tx_dma_done = 1;
    uart.poll();
    check("the change is applied once TX has drained", stub_usart_enable_count == enables + 1);
    receive_byte(0xC3);
    uart.copy_rx_data(received, 1);
    check("a received byte is masked after the change is applied", received[0] == 0x43);

    // A request that changes nothing leaves the USART alone
    restart();
    enables = stub_usart_enable_count;
    uart.set_coding(9600, 8, uart_stopbits::_1_0, uart_parity::none);
    uart.poll();
    check("the same line coding doesn't restart the USART", stub_usart_enable_count == enables);

    // Each of the settings counts as a change
    uart.set_coding(19200, 8, uart_stopbits::_1_0, uart_parity::none);
    uart.poll();
    check("a baud rate change restarts the USART", stub_usart_enable_count == enables + 1);
    check("the new baud rate is reported", uart.baudrate() == 19200);
    uart.set_coding(19200, 8, uart_stopbits::_2_0, uart_parity::none);
    uart.poll();
    check("a stop bits change restarts the USART", stub_usart_enable_count == enables + 2);
    uart.set_coding(19200, 8, uart_stopbits::_2_0, uart_parity::odd);
    uart.poll();
    check("a parity change restarts the USART", stub_usart_enable_count == enables + 3);
    uart.set_coding(19200, 7, uart_stopbits::_2_0, uart_parity::odd);
    uart.poll();
    check("a data bits change restarts the USART", stub_usart_enable_count == enables + 4);

    // Repeating a pending request keeps its place in the TX queue
    restart();
    uart.transmit(tx_data, sizeof(tx_data));
    uart.set_coding(115200, 8, uart_stopbits::_1_0, uart_parity::none);
    int queue_pos = uart.coding_change_tx_pos;
    uart.transmit(tx_data, sizeof(tx_data));
    uart.set_coding(115200, 8, uart_stopbits::_1_0, uart_parity::none);
    check("repeating a pending change keeps its queue position", uart.coding_change_tx_pos == queue_pos);
    check("  and the change is still pending", uart.is_coding_change_pending);

    // Going back to the settings in use cancels a change that's still waiting
    restart();
    uart.transmit(tx_data, sizeof(tx_data));
    enables = stub_usart_enable_count;
    uart.set_coding(115200, 8, uart_stopbits::_1_0, uart_parity::none);
    uart.transmit(tx_data, sizeof(tx_data));
    uart.set_coding(9600, 8, uart_stopbits::_1_0, uart_parity::none);
    check("going back to the settings in use cancels a waiting change", !uart.is_coding_change_pending);
    check("  and GET_LINE_CODING reports them", uart.baudrate() == 9600);
    for (int i = 0; i < 3; i++) {
        stub_tx_dma_done = 1;
        uart.poll();
    }
    check("  and the USART isn't restarted once TX drains",
        uart.tx_buf_tail == uart.tx_buf_head && stub_usart_enable_count == enables);

    // enable() applies the default settings, even if they're already selected
    restart();
    enables = stub_usart_enable_count;
    uart.enable();
    check("enable() applies the default line coding every time", stub_usart_enable_count > enables);
    check("  and leaves nothing pending", !uart.is_coding_change_pending);

    return check_result();
}
