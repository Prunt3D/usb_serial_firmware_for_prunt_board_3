# Firmware



## Building the firmware

In order to build the firmware, several steps are needed:


### 1. Install [PlatformIO](https://platformio.org/platformio-ide)

These instructions assume that PlatformIO is installed as an extension of *Visual Studio Code*.


### 2. Add board definitions

Add the board definition `genericSTM32F042F6.json` (from the [support](../support) directory) to `boards` directory of the PlatformIO installation (`.platformio/boards` in your home directory). You might need to create the `boards` directory if it doesn't exist yet.


### 3. Build

Either from within Visual Studio Code (click *Build* icon in status bar or click the *Build* in project task in the *PLATFORMIO* view) or from a shell:
```
pio run
```



## Uploading the firmware

In order to upload the firmware, a ST-Link (or a JLink) adapter is needed. Connect the adapter to the SWD debug port and upload, either from PlatformIO (upload icon in status bar, *Upload* task in *PLATFORMIO* view) or by running:

```
pio run -t upload
```

An ST-Link adapter (except the lastet V3 series) can provide sufficient power to the USB-to-Serial adapter for programming it.


## Entering the system bootloader

On the STM32F042, the firmware exposes a vendor-specific control request that disconnects from USB, erases the first user-flash page, and reloads the option bytes. The reload resets the MCU and updates its flash-empty flag, which a normal system reset does not do. Because the first word at `0x08000000` is now erased, the MCU starts the STM32 system memory bootloader.

The application cannot run again until the firmware is reflashed. Erasing the page removes the vector table, so the MCU keeps starting the system bootloader, including after a power cycle. To recover, flash the firmware through the system bootloader or the debug port.

Use [`tools/enter-system-bootloader.c`](../tools/enter-system-bootloader.c) to send the request, passing the board's USB serial number if more than one board is connected. To send it another way, make a control transfer to endpoint 0 with:

- `bmRequestType = 0x40` (vendor, device, host-to-device)
- `bRequest = 0x51`
- `wValue = 0xb007`
- `wIndex = 0xa53b`
- no data stage


## Documentation

- [What you need to know about USB and the STM32 USB peripheral](../doc/usb-facts.md)

- [Firmware Architecture](../doc/firmware.md)

- [Data and Control Signals](../doc/serial-signals.md)
