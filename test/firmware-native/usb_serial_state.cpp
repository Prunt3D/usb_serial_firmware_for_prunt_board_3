// Runs the real usb_serial.cpp and uart.cpp, capturing the serial state notifications submitted to
// the USB stack. The stack's handling of a halted endpoint is tested in qsb_driver_halt.cpp. Here,
// the notification endpoint holds one packet until the host collects it, and can refuse a packet or
// drop the one it holds, as clearing a halt does.

#include <string.h>

#include "check.h"
#include "uart_under_test.h"
#include "usb_serial.h"

static uint8_t last_notification[10];
static int notification_count;
static bool refuse_notifications;
static bool is_notification_queued;

uint32_t millis()
{
    return 1000;
}

qsb_device* usb_device;

void usb_cdc_init() {}
void usb_cdc_poll() {}
bool usb_cdc_is_connected() { return true; }
void qsb_dev_ep_setup(qsb_device*, uint8_t, uint32_t, int, qsb_dev_ep_callback_fn) {}
uint16_t qsb_dev_ep_read_packet(qsb_device*, uint8_t, uint8_t*, uint16_t) { return 0; }
void qsb_dev_ep_pause(qsb_device*, uint8_t) {}
void qsb_dev_ep_unpause(qsb_device*, uint8_t) {}

uint16_t qsb_dev_ep_transmit_avail(qsb_device*, uint8_t addr)
{
    return addr == 0x83 && is_notification_queued ? 0 : 64;
}

int qsb_dev_ep_transmit_packet(qsb_device*, uint8_t addr, const uint8_t* buf, int len)
{
    if (addr == 0x83) {
        if (refuse_notifications || is_notification_queued)
            return -1;
        memcpy(last_notification, buf, len);
        notification_count++;
        is_notification_queued = true;
    }
    return len;
}

#include "usb_serial.cpp"

// The host collects the queued notification, and the stack reports it as sent
static void collect_notification()
{
    is_notification_queued = false;
    usb_comm_in_cb(usb_device, 0x83, 10);
}

int main()
{
    usb_serial.on_usb_configured();
    usb_serial.send_serial_state();
    check("the initial notification goes out", notification_count == 1);
    check("it's a SERIAL_STATE with 2 bytes of data",
        last_notification[0] == 0xA1 && last_notification[1] == 0x20 && last_notification[6] == 2);
    check("it reports DCD and DSR as active", last_notification[8] == 0x03 && last_notification[9] == 0);

    usb_serial.poll();
    check("nothing more is sent before the host collects it", notification_count == 1);
    collect_notification();
    usb_serial.poll();
    usb_serial.poll();
    check("nothing more is sent while nothing changes", notification_count == 1);

    uart.pending_errors = (uint16_t)uart_error::parity;
    usb_serial.poll();
    check("a UART error is reported", notification_count == 2);
    check("  with DCD and DSR still active", last_notification[8] == 0x23);
    collect_notification();
    usb_serial.poll();
    check("  and only once", notification_count == 2);

    // A notification refused because the endpoint is busy or halted stays pending
    refuse_notifications = true;
    uart.pending_errors = (uint16_t)uart_error::overrun;
    usb_serial.poll();
    usb_serial.poll();
    refuse_notifications = false;
    usb_serial.poll();
    check("a refused notification goes out once it can", notification_count == 3 && last_notification[8] == 0x43);
    collect_notification();
    usb_serial.poll();
    check("  and only once", notification_count == 3);

    // A notification dropped before the host collects it, as when a halt on the endpoint is
    // cleared, is sent again
    uart.pending_errors = (uint16_t)uart_error::framing;
    usb_serial.poll();
    check("a framing error is reported", notification_count == 4 && last_notification[8] == 0x13);
    is_notification_queued = false;
    usb_serial.poll();
    check("  and sent again if it's dropped", notification_count == 5 && last_notification[8] == 0x13);
    collect_notification();
    usb_serial.poll();
    check("  and only once after it's collected", notification_count == 5);

    // A new configuration starts over
    usb_serial.on_usb_configured();
    is_notification_queued = false;
    usb_serial.send_serial_state();
    check("a new configuration sends the state again", notification_count == 6 && last_notification[8] == 0x03);

    // Even if dropping it leaves nothing but DCD and DSR to report
    is_notification_queued = false;
    usb_serial.poll();
    check("  and the state is sent again if that's dropped", notification_count == 7 && last_notification[8] == 0x03);
    collect_notification();
    usb_serial.poll();
    check("  and only once after it's collected", notification_count == 7);

    return check_result();
}
