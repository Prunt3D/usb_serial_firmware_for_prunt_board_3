/*
 * Trigger the Prunt Board 3 USB-serial firmware's system bootloader entry.
 *
 * Usage: enter-system-bootloader [SERIAL]
 *
 * This erases the first page of the board's application flash, so the application
 * will not run again until the firmware is reflashed. SERIAL selects the board by
 * its USB serial number and is required if more than one board is connected.
 *
 * Build on Linux with:
 *   cc -Wall -Wextra -O2 -o enter-system-bootloader enter-system-bootloader.c \
 *      $(pkg-config --cflags --libs libusb-1.0)
 *
 * The USB device will disconnect after the request completes. It should then
 * enumerate as the STM32 system memory bootloader.
 */

#include <libusb-1.0/libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint16_t USB_VID = 0x0483;
static const uint16_t USB_PID = 0xa53b;

static const uint8_t REQ_TYPE_ENTER_SYSTEM_BOOTLOADER =
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
static const uint8_t REQ_ENTER_SYSTEM_BOOTLOADER = 0x51;
static const uint16_t REQ_ENTER_SYSTEM_BOOTLOADER_VALUE = 0xb007;
static const uint16_t REQ_ENTER_SYSTEM_BOOTLOADER_INDEX = 0xa53b;
static const unsigned int CONTROL_TIMEOUT_MS = 1000;

static int open_and_read_serial(libusb_device *device,
                                uint8_t serial_index,
                                libusb_device_handle **handle,
                                char *serial,
                                int serial_size)
{
    int result = libusb_open(device, handle);
    if (result != LIBUSB_SUCCESS)
        return result;

    result = libusb_get_string_descriptor_ascii(*handle, serial_index, (unsigned char *)serial, serial_size);
    if (result < 0) {
        libusb_close(*handle);
        *handle = NULL;
        return result;
    }

    return LIBUSB_SUCCESS;
}

/*
 * Opens the board to act on, but only if exactly one board qualifies. A board whose
 * serial number can't be read counts as a possible match when no serial was given,
 * because the request is destructive and could otherwise reach the wrong board.
 */
static bool open_prunt_device(libusb_context *ctx, const char *wanted_serial, libusb_device_handle **handle)
{
    libusb_device **devices = NULL;
    ssize_t count = libusb_get_device_list(ctx, &devices);
    if (count < 0) {
        fprintf(stderr, "Could not list USB devices: %s\n", libusb_error_name((int)count));
        return false;
    }

    int match_count = 0;
    int unreadable_count = 0;

    for (ssize_t i = 0; i < count; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devices[i], &desc) != LIBUSB_SUCCESS)
            continue;

        if (desc.idVendor != USB_VID || desc.idProduct != USB_PID)
            continue;

        const unsigned int bus = libusb_get_bus_number(devices[i]);
        const unsigned int address = libusb_get_device_address(devices[i]);

        libusb_device_handle *candidate = NULL;
        char serial[64];
        int result = open_and_read_serial(devices[i], desc.iSerialNumber, &candidate, serial, sizeof(serial));
        if (result != LIBUSB_SUCCESS) {
            fprintf(stderr, "Could not read the serial number of %04x:%04x on bus %u device %u: %s\n",
                    USB_VID,
                    USB_PID,
                    bus,
                    address,
                    libusb_error_name(result));
            unreadable_count++;
            continue;
        }

        fprintf(stderr, "Found %04x:%04x serial %s on bus %u device %u\n", USB_VID, USB_PID, serial, bus, address);

        if (wanted_serial != NULL && strcmp(serial, wanted_serial) != 0) {
            libusb_close(candidate);
            continue;
        }

        match_count++;
        if (*handle == NULL)
            *handle = candidate;
        else
            libusb_close(candidate);
    }

    libusb_free_device_list(devices, 1);

    const int possible_matches = match_count + (wanted_serial == NULL ? unreadable_count : 0);
    if (match_count == 1 && possible_matches == 1)
        return true;

    if (*handle != NULL) {
        libusb_close(*handle);
        *handle = NULL;
    }

    if (possible_matches > 1)
        fprintf(stderr, "More than one board could match. Pass the serial number of the board to use.\n");
    else if (match_count == 0 && unreadable_count == 0 && wanted_serial != NULL)
        fprintf(stderr, "USB device %04x:%04x with serial %s not found\n", USB_VID, USB_PID, wanted_serial);
    else if (match_count == 0 && unreadable_count == 0)
        fprintf(stderr, "USB device %04x:%04x not found\n", USB_VID, USB_PID);
    else
        fprintf(stderr, "No board could be selected.\n");

    return false;
}

int main(int argc, char **argv)
{
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [SERIAL]\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char *wanted_serial = argc == 2 ? argv[1] : NULL;
    libusb_context *ctx = NULL;
    libusb_device_handle *dev = NULL;

    int result = libusb_init(&ctx);
    if (result != LIBUSB_SUCCESS) {
        fprintf(stderr, "libusb_init failed: %s\n", libusb_error_name(result));
        return EXIT_FAILURE;
    }

    if (!open_prunt_device(ctx, wanted_serial, &dev)) {
        libusb_exit(ctx);
        return EXIT_FAILURE;
    }

    result = libusb_control_transfer(
        dev,
        REQ_TYPE_ENTER_SYSTEM_BOOTLOADER,
        REQ_ENTER_SYSTEM_BOOTLOADER,
        REQ_ENTER_SYSTEM_BOOTLOADER_VALUE,
        REQ_ENTER_SYSTEM_BOOTLOADER_INDEX,
        NULL,
        0,
        CONTROL_TIMEOUT_MS);

    libusb_close(dev);
    libusb_exit(ctx);

    if (result != 0) {
        fprintf(stderr, "control transfer failed: %s\n", libusb_error_name(result));
        return EXIT_FAILURE;
    }

    puts("Bootloader entry request sent.");
    return EXIT_SUCCESS;
}
