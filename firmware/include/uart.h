/*
 * USB Serial
 *
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * UART interface
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>

#define UART_TX_BUF_LEN 2048
#define UART_RX_BUF_LEN 1024

enum class uart_stopbits
{
    _1_0 = 0,
    _1_5 = 1,
    _2_0 = 2,
};


enum class uart_parity
{
    none = 0,
    odd = 1,
    even = 2,
};

enum class uart_error : uint16_t
{
    framing = 0x10,
    parity = 0x20,
    overrun = 0x40,
};


/**
 * @brief UART implementation
 */
class uart_impl
{
public:
    /// Initializes UART.
    void init();

    // Enable the UART
    void enable();

    /**
     * Polls for new UART events.
     */
    void poll();

    /**
     * @brief Submits the specified data for transmission.
     *
     * The specified data is added to the transmission
     * buffer and transmitted asynchronously.
     *
     * @param data pointer to byte array
     * @param len length of byte array
     */
    void transmit(const uint8_t *data, size_t len);

    /**
     * @brief Copies data from the receive buffer into the specified array.
     *
     * The copied data is removed from the buffer.
     *
     * @param data pointer to byte array
     * @param len length of the byte array
     * @return number of bytes copied to the byte array
     */
    size_t copy_rx_data(uint8_t *data, size_t len);

    /**
     * @brief Returns the length of received data in the receive buffer
     *
     * @return length, in number of bytes
     */
    size_t rx_data_len();

    /**
     * Indicates of an RX buffer overrun has occurred.
     *
     * This function will return `true` once for
     * each occurrence of an overrun.
     *
     * @return `true` if overrun occurred.
     */
    bool has_rx_overrun_occurred();

    /**
     * Gets and clears pending UART hardware/software error flags.
     *
     * The returned bits use the same values as the USB CDC PSTN
     * serial-state error bits.
     *
     * @return bitwise OR of `uart_error` values
     */
    uint16_t take_errors();

    /**
     * @brief Returns the available space in the transmit buffer
     *
     * @return space, in number of bytes
     */
    size_t tx_data_avail();

    /**
     * @brief Sets the line coding.
     *
     * Data queued for transmission before this call is sent with the old settings. The new
     * settings take effect once it has all been sent, and data queued after this call waits
     * for that. The UART also keeps receiving with the old settings until then. The getters
     * below report the new settings right away. Asking for the settings already requested does
     * nothing. Asking for the settings the USART is using, while a change is still waiting,
     * cancels the change, and data queued since it was requested goes out with those settings.
     *
     * @param baudrate baud rate, in bps
     * @param databits data bits per byte
     * @param stopbits length of stop period, in bits
     * @param parity type of party bit
     */
    void set_coding(int baudrate, int databits, uart_stopbits stopbits, uart_parity parity);

    /**
     * @brief Gets the baud rate.
     *
     * @return baud rate, in bps
     */
    int baudrate() { return _baudrate; }

    /**
     * @brief Gets the data bits per byte.
     *
     * @return number of bits
     */
    int databits() { return _databits; }

    /**
     * @brief Gets the length of the stop period
     *
     * @return length, in bits
     */
    uart_stopbits stopbits() { return _stopbits; }

    /**
     * @brief Gets the type of parity bit
     *
     * @return parity type
     */
    uart_parity parity() { return _parity; }

private:
    /// Check if a chunk of data has been transmitted
    void poll_tx_complete();

    /// Try to transmit more data
    void start_transmission();

    /**
     * @brief Checks if RX buffer has been overrun.
     *
     * If it has been overrun, data is discarded and the error state reset.
     *
     * This functions must be called frequently in order to reliably detect an overrun
     * (more often than: RX buffer size * 10 bit/byte / maximum bit rate / 2)
     */
    void check_rx_overrun();

    /// Checks if RX buffer has been overrun, with the DMA at the given position.
    void check_rx_overrun(int buf_head);

    /// The position in the RX buffer that the DMA writes next
    int rx_buf_head();

    /// The amount of data in the RX buffer, with the DMA at the given position
    size_t rx_data_len(int buf_head);

    /// Checks for USART hardware receive errors and clears the peripheral flags.
    void check_usart_errors();

    /// Applies a pending line coding change once the data queued before it has been sent.
    void apply_pending_coding();

    /// Applies the current line coding to the USART, cutting off anything being sent.
    void apply_coding();

    /**
     * @brief Sets the baudrate
     *
     * Uses the highest oversampling mode applicable to the baudrate.
     * Limits the baudrate if set too high.
     *
     * @param baud baudrate (in bps)
     */
    void set_baudrate(int baud);

    /**
     * Deletes the high bit of each byte.
     *
     * It is used to support 7 data bits.
     */
    static void clear_high_bits(uint8_t* buf, int buf_len);

    /// Deletes the high bit of each byte in the receive buffer from `start` up to `end`. `start` can
    /// be negative, to count back from the end of the buffer.
    void clear_high_bits_rx(int start, int end);

    /// Masks received data to the data bits that each byte was received with.
    void mask_rx_data(uint8_t* data, int len);

    // Buffer for data to be transmitted via UART
    //  *  0 <= head < buf_len
    //  *  0 <= tail < buf_len
    //  *  head == tail => empty
    //  *  head + 1 == tail => full (modulo UART_TX_BUF_LEN)
    // `tx_buf_head` points to the positions where the next character
    // should be inserted. `tx_buf_tail` points to the character after
    // the last character that has been transmitted.
    uint8_t tx_buf[UART_TX_BUF_LEN];
    int tx_buf_head;
    int tx_buf_tail;

    // The number of bytes currently being transmitted
    int tx_size;

    // The line coding fields below have been changed, but the USART doesn't use them yet.
    bool is_coding_change_pending;
    // The value of `tx_buf_head` when the line coding was changed. Data before this
    // position is sent with the old settings and data after it with the new ones.
    int coding_change_tx_pos;

    // Buffer of data received via UART
    //  *  0 <= head < buf_len
    //  *  0 <= tail < buf_len
    //  *  head == tail => empty
    //  *  head + 1 == tail => full (modulo UART_RX_BUF_LEN)
    // `rx_buf_head` points to the positions where the next character
    // should be inserted. `rx_buf_tail` points to the character after
    // the last character that has been transmitted.
    uint8_t rx_buf[UART_RX_BUF_LEN];
    // int rx_buf_head: : managed by circular DMA controller
    int rx_buf_tail;

    // Last measured RX buffer size (to detect overrun)
    size_t last_rx_size;

    int _baudrate;
    int _databits;
    uart_stopbits _stopbits;
    uart_parity _parity;

    // The line coding that the USART is using. `_databits` changes as soon as the host asks,
    // which is right for data queued from then on, but the USART keeps receiving with the old
    // settings until the change is applied.
    int usart_baudrate;
    int usart_databits;
    uart_stopbits usart_stopbits;
    uart_parity usart_parity;

    // How many bytes at the start of the receive buffer were received before the last line coding
    // change. They have already been masked to the data bits they were received with.
    int rx_len_before_coding;

    int rx_high_water_mark;
    int tx_max_chunk_size;

    bool is_transmitting;
    bool is_enabled;
    uint16_t pending_errors;
};

/// Global UART instance
extern uart_impl uart;
