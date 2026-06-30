/*
 * Trigger the Prunt Board 3 USB-serial firmware's system bootloader entry.
 *
 * Build on Linux with:
 *   cc -Wall -Wextra -O2 -o enter-system-bootloader enter-system-bootloader.c \
 *      $(pkg-config --cflags --libs libusb-1.0)
 *
 * The USB device will disconnect after the request completes. On STM32F0 builds
 * using the matching firmware, it should then enumerate as the STM32 system
 * memory bootloader.
 */

#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static const uint16_t USB_VID = 0x0483;
static const uint16_t USB_PID = 0xa53b;

static const uint8_t REQ_TYPE_ENTER_SYSTEM_BOOTLOADER =
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
static const uint8_t REQ_ENTER_SYSTEM_BOOTLOADER = 0x51;
static const uint16_t REQ_ENTER_SYSTEM_BOOTLOADER_VALUE = 0xb007;
static const uint16_t REQ_ENTER_SYSTEM_BOOTLOADER_INDEX = 0xa53b;
static const unsigned int CONTROL_TIMEOUT_MS = 1000;

static int open_prunt_device(libusb_context *ctx, libusb_device_handle **handle)
{
    libusb_device **devices = NULL;
    ssize_t count = libusb_get_device_list(ctx, &devices);
    if (count < 0)
        return (int)count;

    int result = LIBUSB_ERROR_NO_DEVICE;

    for (ssize_t i = 0; i < count; i++) {
        struct libusb_device_descriptor desc;
        int desc_result = libusb_get_device_descriptor(devices[i], &desc);
        if (desc_result != LIBUSB_SUCCESS) {
            result = desc_result;
            continue;
        }

        if (desc.idVendor != USB_VID || desc.idProduct != USB_PID)
            continue;

        fprintf(stderr, "Found %04x:%04x on bus %u device %u\n",
                USB_VID,
                USB_PID,
                libusb_get_bus_number(devices[i]),
                libusb_get_device_address(devices[i]));

        result = libusb_open(devices[i], handle);
        if (result == LIBUSB_SUCCESS)
            break;
    }

    libusb_free_device_list(devices, 1);
    return result;
}

int main(void)
{
    libusb_context *ctx = NULL;
    libusb_device_handle *dev = NULL;

    int result = libusb_init(&ctx);
    if (result != LIBUSB_SUCCESS) {
        fprintf(stderr, "libusb_init failed: %s\n", libusb_error_name(result));
        return EXIT_FAILURE;
    }

    result = open_prunt_device(ctx, &dev);
    if (result != LIBUSB_SUCCESS) {
        if (result == LIBUSB_ERROR_NO_DEVICE)
            fprintf(stderr, "USB device %04x:%04x not found\n", USB_VID, USB_PID);
        else
            fprintf(stderr, "USB device %04x:%04x could not be opened: %s\n",
                    USB_VID,
                    USB_PID,
                    libusb_error_name(result));
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

    puts("Bootloader reset request sent.");
    return EXIT_SUCCESS;
}
