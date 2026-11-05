# Firmware Architecture

The firmware takes care of two data paths:

- USB-to-serial path
- serial-to-USB path

The two paths are shown in the figure below. They are mostly independent, both in terms of firmware impelementation and flow control.

![Architecture](images/architecture.svg)


## Software Architecture


### USB-to-serial path

When new data has arrived via USB, the callback `usb_serial_impl::on_usb_data_received()` is called. It copies the data from the PMA buffers into the transmit ring buffer and starts the DMA transfer to the UART data register to transmit the data (unless a DMA operation is already in progress).

When the DMA transfer is complete, it updates the ring buffer accordingly and if more data has arrived in the mean-time, another DMA transfer is started.

### Serial-to-USB path

For UART reception, a permanent circular DMA transfer is set up copying received bytes into the receive ring buffer. `usb_serial_impl::poll()` is called very frequently from the loop in `main()`. It reads the DMA transfer state (number of bytes copied by DMA) to check for additional data that has been copied into the ring buffer.

If data has arrived and if no outgoing USB operation is in progress, the data is put into the PMA buffers so it is transmitted when the host polls the device the next time. Once the data has been transmitted, the callback `usb_serial_impl::on_usb_data_transmitted()` is called.


## Flow control

The software implements flow control to regulate the speed of data transfer in order not to overflow the buffers. The two directions are independent.

The UART side has no flow control: the Prunt Board 3 only connects TX and RX, so the firmware doesn't implement RTS and CTS. On the USB side, standard USB flow control is used.

### USB-to-serial path

When the transmission ring buffer is becoming full, the USB *data out* endpoint is paused, i.e. the firmware stops reading packets from it. Its two PMA buffers fill up, and the host's further packets then get NAK. This tells the host that the device is currently unable to accept more data. The endpoint is paused when less than two packets of space remain, and it's unpaused once there's room again.

### Serial-to-USB path

The sender on the UART can't be paused. The host can normally collect data faster than the UART delivers it, but if it falls behind, the receive ring buffer overruns. The firmware then discards the buffered data and reports an overrun to the host. This can happen at 6 Mbaud with heavy traffic in both directions, as they share the USB bus.

When the host first reads from the *data in* endpoint after configuration, or after clearing a halt, it gets two empty packets. This is how the USB peripheral starts double buffering (see `dbl_buf_queued_tx()`).

## Line coding

The line coding returns to the default, 9600 8N1, at every SET_CONFIGURATION.

A change takes effect once the UART has sent everything queued before the request. Data that arrives after the request waits and goes out with the new settings, and the UART keeps receiving with the old settings until then. GET_LINE_CODING reports the new settings straight away.

A baud rate of 75 selects 6 Mbaud, as Linux doesn't easily allow rates above 4 Mbaud.

## UART errors

Framing, parity and overrun errors are reported to the host with a *SerialState* notification. Noise errors are reported as framing errors. Received data keeps flowing to the host while errors are being reported.

## USB Stack

The firmware uses a custom USB stack instead of the standard *libopencm3* stack as the *libopencm3* stack has several issues:

- Double-buffering is not supported. Thus, it is not able to achieve the maximum data rate of the UART.
- USB flow-control has a design flaw that requires bigger buffers as one cannot pause the endpoint exactly when the buffer is full.
- Bugs in stack affect this firmware.
 