# Firmware



## Building the firmware

In order to build the firmware, several steps are needed:


### 1. Install [PlatformIO](https://platformio.org/platformio-ide)

These instructions assume that PlatformIO is installed as an extension of *Visual Studio Code*.


### 2. Build

Either from within Visual Studio Code (click *Build* icon in status bar or click the *Build* in project task in the *PLATFORMIO* view) or from a shell:
```
pio run
```

The default environment is `genericSTM32F042C6`, the MCU on the Prunt Board 3. `platformio.ini` finds its board definition in the [support](../support) directory, so there's nothing to install for it.

`platformio.ini` pins the platform, toolchain and libopencm3 versions, so every build uses the same packages. Change them there, not by updating PlatformIO's packages.


### Builds on GitHub

[`.github/workflows/build.yml`](../.github/workflows/build.yml) builds the firmware and runs the [native tests](../test/firmware-native/README.md) on every push and pull request. The firmware is kept as the run's `usb-serial-firmware` artifact. The workflow also compiles [`tools/enter-system-bootloader.c`](../tools/enter-system-bootloader.c).

To make a release, push a tag named after the USB device release, `USB_DEVICE_REL` in [`src/usb_conf.cpp`](src/usb_conf.cpp). This is `bcdDevice`, which is binary-coded decimal, so `0x0200` is tag `v2.0.0`. The workflow stops if the tag doesn't match. Otherwise it makes a draft release with the `.bin` and `.elf` files, which you then publish on GitHub.


## Uploading the firmware

The Prunt Board 3 has no SWD connection to the STM32F042, so the firmware is updated over USB using the DFU mode of the STM32 system bootloader. [Enter the system bootloader](#entering-the-system-bootloader), then flash with [dfu-util](https://dfu-util.sourceforge.net/), writing the first 1 KB page last:

```
cd .pio/build/genericSTM32F042C6
dd if=firmware.bin of=firmware-rest.bin bs=1024 skip=1
dd if=firmware.bin of=firmware-page0.bin bs=1024 count=1
dfu-util -d 0483:df11 -a 0 -s 0x08000400 -D firmware-rest.bin &&
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware-page0.bin
```

The order matters because the MCU only starts the system bootloader while the first page is erased. If flashing stops before that page is written, the board stays in the bootloader and you can flash it again. If the first page were written first, an interrupted update would leave a partial image that the MCU starts instead, with no way back over USB.

So only write the first page once the rest has been written without errors, which is what the `&&` does. If the first `dfu-util` command fails, with any exit code, run it again before going on.

The second command can report an error (exit code 74) as the device leaves DFU mode. This is harmless.


## Entering the system bootloader

On the STM32F042, the firmware exposes a vendor-specific control request that disconnects from USB, erases the first user-flash page, and reloads the option bytes. The reload resets the MCU and updates its flash-empty flag, which a normal system reset does not do. Because the first word at `0x08000000` is now erased, the MCU starts the STM32 system memory bootloader.

The application cannot run again until the firmware is reflashed. Erasing the page removes the vector table, so the MCU keeps starting the system bootloader, including after a power cycle. To recover, flash the firmware through the system bootloader.

Use [`tools/enter-system-bootloader.c`](../tools/enter-system-bootloader.c) to send the request, passing the board's USB serial number if more than one board is connected. To send it another way, make a control transfer to endpoint 0 with:

- `bmRequestType = 0x40` (vendor, device, host-to-device)
- `bRequest = 0x51`
- `wValue = 0xb007`
- `wIndex = 0xa53b`
- no data stage


## Testing

Parts of the firmware can be tested on a computer, with no board. See [`test/firmware-native`](../test/firmware-native/README.md).

