
# Data and Control Signals

The data and control signals are derived from the RS-232 standard. In its original incarnation, the standard used +12V and -12V as signal levels. This has evoled into TTL logic levels (+5V and 0V) and 3.3V logic levels (+3.3V and 0V). The hardware of the USB-to-serial adapter uses 3.3V logical levels. Input signals are 5V tolerant.

The standard is based around the idea that a computer is connected to a modem, the modem communicates to another modem, which in turn is connected to a computer. The RS-232 standard would apply to the connection between the computer and the modem. It uses the terms *data terminal equipment (DTE)* for the computer and *data circuit-terminating equipment (DCE)* for the modem.

For simplicity, this document talks about the computer (DTE) and the device (DCE).  The USB-to-serial adapter is seen as part in-between. It implements a number of signals:

## Data Signals

The data singals use 3.3V logic levels:

Data      | Voltage
----------|---------
0 (space) |   0V
1 (mark)  | 3.3V

The control signals adhere to the RS-232 standard (except for using logic level voltages):

Signal   | Direction | Purpose
---------|-----------|---------
TX       | Output    | Data transmission to device.
RX       | Input     | Data reception from device.


## Control Signals

The Prunt Board 3 only connects TX and RX, so this firmware doesn't implement any control signals:

- *SetControlLineState* requests (DTR and RTS) are accepted, and the state is ignored.
- *SerialState* notifications report UART errors. The bits for the input signals (DCD and DSR) are always set, as some systems wait for DCD when opening a port. The serial state is sent after each SET_CONFIGURATION, and again after each *SetControlLineState* request, which hosts send when they open the port. That covers a host driver that's bound again without a new configuration, and starts with DCD off.
- There's no hardware flow control on the serial side. See [Firmware Architecture](firmware.md).
