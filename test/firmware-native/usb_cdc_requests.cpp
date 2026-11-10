// Runs the real usb_cdc.cpp with the USB stack and usb_serial replaced by stubs, to check which CDC
// class requests it accepts in which USB state and with which direction.

#include <stdint.h>

#include "check.h"
#include "bootloader.h"
#include "usb_cdc.h"
#include "usb_conf.h"
#include "usb_serial.h"

static int line_coding_sets;
static int line_coding_gets;
static int line_state_sets;

usb_serial_impl usb_serial;

void usb_serial_impl::on_usb_configured() {}
void usb_serial_impl::send_serial_state() {}
void usb_serial_impl::get_line_coding(qsb_pstn_line_coding*) { line_coding_gets++; }
void usb_serial_impl::set_control_line_state(uint16_t) { line_state_sets++; }

bool usb_serial_impl::set_line_coding(qsb_pstn_line_coding*)
{
    line_coding_sets++;
    return true;
}

void delay(uint32_t) {}
void bootloader_enter_system_memory_boot_mode() {}
qsb_device* usb_conf_init() { return nullptr; }
void qsb_dev_disconnect(qsb_device*, bool) {}
void qsb_dev_poll(qsb_device*) {}
void qsb_dev_register_control_callback(qsb_device*, uint8_t, uint8_t, qsb_dev_control_callback_fn) {}
void qsb_dev_register_reset_callback(qsb_device*, void (*)(void)) {}
void qsb_dev_register_set_config_callback(qsb_device*, qsb_dev_set_config_callback_fn) {}

#include "usb_cdc.cpp"

void crs_autotrim_usb_enable(void) {}

static qsb_request_return_code request(uint8_t type, uint8_t code, uint16_t length)
{
    static uint8_t buffer[16];
    uint8_t* buf = buffer;
    uint16_t len = length;
    qsb_dev_control_completion_callback_fn complete = nullptr;
    qsb_setup_data req = {type, code, 0, 0, length};
    return cdc_control_request(nullptr, &req, &buf, &len, &complete);
}

static qsb_request_return_code set_line_coding() { return request(0x21, QSB_PSTN_REQ_SET_LINE_CODING, 7); }
static qsb_request_return_code get_line_coding() { return request(0xA1, QSB_PSTN_REQ_GET_LINE_CODING, 7); }
static qsb_request_return_code set_line_state() { return request(0x21, QSB_PSTN_REQ_SET_CONTROL_LINE_STATE, 0); }

int main()
{
    cdc_set_config(nullptr, 1);
    check("configured: SET_LINE_CODING works", set_line_coding() == QSB_REQ_HANDLED && line_coding_sets == 1);
    check("configured: GET_LINE_CODING works", get_line_coding() == QSB_REQ_HANDLED && line_coding_gets == 1);
    check("configured: SET_CONTROL_LINE_STATE works", set_line_state() == QSB_REQ_HANDLED && line_state_sets == 1);

    // The stack keeps the handler registered through a USB reset
    usb_reset();
    check("after a reset: SET_LINE_CODING is rejected", set_line_coding() == QSB_REQ_NOTSUPP && line_coding_sets == 1);
    check("after a reset: GET_LINE_CODING is rejected", get_line_coding() == QSB_REQ_NOTSUPP && line_coding_gets == 1);
    check("after a reset: SET_CONTROL_LINE_STATE is rejected",
        set_line_state() == QSB_REQ_NOTSUPP && line_state_sets == 1);

    cdc_set_config(nullptr, 1);
    check("configured again: SET_LINE_CODING works", set_line_coding() == QSB_REQ_HANDLED && line_coding_sets == 2);

    // The direction bit only matters if there's a data stage
    check("SET_LINE_CODING sent as IN is rejected",
        request(0xA1, QSB_PSTN_REQ_SET_LINE_CODING, 7) == QSB_REQ_NOTSUPP && line_coding_sets == 2);
    check("GET_LINE_CODING sent as OUT is rejected",
        request(0x21, QSB_PSTN_REQ_GET_LINE_CODING, 7) == QSB_REQ_NOTSUPP && line_coding_gets == 1);
    check("SET_CONTROL_LINE_STATE sent as IN with no data stage works",
        request(0xA1, QSB_PSTN_REQ_SET_CONTROL_LINE_STATE, 0) == QSB_REQ_HANDLED && line_state_sets == 2);
    check("SET_CONTROL_LINE_STATE sent as IN with a data stage is rejected",
        request(0xA1, QSB_PSTN_REQ_SET_CONTROL_LINE_STATE, 2) == QSB_REQ_NOTSUPP && line_state_sets == 2);

    return check_result();
}
