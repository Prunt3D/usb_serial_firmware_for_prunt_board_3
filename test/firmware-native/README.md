# Firmware tests that run on a computer

These tests compile parts of the real firmware for the computer they run on, with the hardware replaced by stubs. They need no board, so they catch logic mistakes early. They need CMake, a C and C++ compiler, and libopencm3's headers, which PlatformIO downloads when the firmware is first built.

```
cd test/firmware-native
cmake -B build
cmake --build build
cd build && ctest --output-on-failure
```

If libopencm3 isn't in PlatformIO's usual place, pass `-DLIBOPENCM3_INCLUDE=<path>` to the first `cmake` command.

## What they cover

| Test | Runs | Checks |
|---|---|---|
| `uart_rx_overrun` | `uart.cpp` | Over 4 million steps with a model of the circular DMA, no data is lost without an overrun being reported, and no overrun is reported without data being lost. |
| `uart_line_coding` | `uart.cpp` | When a line coding change takes effect, how received data is masked until then, and that a request changing nothing leaves the USART alone. |
| `usb_serial_state` | `usb_serial.cpp`, `uart.cpp` | What *SerialState* notifications contain, that one refused by the USB stack stays pending, and that one dropped before the host collects it is sent again. |
| `usb_cdc_requests` | `usb_cdc.cpp` | That CDC class requests are only accepted while the device is configured. |
| `qsb_driver_halt` | `qsb_drv_fsdev_plus.c` | Setting and clearing an endpoint halt: the data toggles, that nothing but the host ends a halt, and that no packets are read during one. |
| `qsb_standard_requests` | `qsb_standard.c` | String descriptor reads of every length, and GET_STATUS on interfaces. |

## What they don't cover

They don't replace testing on a board. Nothing here checks timing, the electrical side, or how a real host behaves.

`qsb_driver_halt` has a small model of the USB endpoint registers, but it only covers what a write does to each bit. It doesn't model USB transactions or the double buffering conflicts in `dbl_buf_queued_tx()`, so most of the data path isn't tested. Earlier attempts to simulate that got it wrong in ways only register dumps from a real F042 showed, so any addition to the model that goes beyond how writes behave should be checked against one.
