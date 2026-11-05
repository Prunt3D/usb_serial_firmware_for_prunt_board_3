// Runs the real qsb_standard.c for string descriptors and GET_STATUS on interfaces.

#include <string.h>

#include "check.h"
#include "qsb_private.h"

void qsb_internal_ep_reset(qsb_device* dev) { (void)dev; }
void qsb_dev_ep_stall_set(qsb_device* dev, uint8_t addr, uint8_t stall) { (void)dev; (void)addr; (void)stall; }
uint8_t qsb_dev_ep_stall_get(qsb_device* dev, uint8_t addr) { (void)dev; (void)addr; return 0; }

static struct qsb_internal_device dev;
static uint8_t control_buffer[256] __attribute__((aligned(4)));

static qsb_request_return_code request(
    uint8_t type, uint8_t code, uint16_t value, uint16_t index, uint16_t length, uint8_t** buf, uint16_t* len)
{
    qsb_setup_data req = {0};
    req.bmRequestType = type;
    req.bRequest = code;
    req.wValue = value;
    req.wIndex = index;
    req.wLength = length;

    // Anything the request doesn't fill in shows up as this
    memset(control_buffer, 0xAA, sizeof(control_buffer));
    *buf = control_buffer;
    *len = length;
    return qsb_internal_standard_request(&dev, &req, buf, len);
}

// Checks the bytes the host gets for a read of string descriptor 1
static int is_string_read_correct(uint16_t length, const char* str)
{
    uint8_t* buf;
    uint16_t len;
    uint16_t value = (uint16_t)((QSB_DT_STRING << 8) | 1);
    if (request(0x80, QSB_REQ_GET_DESCRIPTOR, value, QSB_LANGID_ENGLISH_US, length, &buf, &len) != QSB_REQ_HANDLED)
        return 0;

    uint16_t full_length = (uint16_t)(strlen(str) * 2 + 2);
    if (len != (length < full_length ? length : full_length))
        return 0;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t expected = i == 0 ? (uint8_t)full_length
            : i == 1              ? QSB_DT_STRING
            : i % 2 == 0          ? (uint8_t)str[i / 2 - 1]
                                  : 0;
        if (buf[i] != expected)
            return 0;
    }
    return 1;
}

int main(void)
{
    static const char* const strings[] = {"Prunt"};
    static const qsb_interface interfaces[2];
    static const qsb_config_desc config = {.bNumInterfaces = 2, .interface = interfaces};
    dev.strings = strings;
    dev.num_strings = 1;
    dev.config = &config;

    int all_correct = 1;
    for (uint16_t length = 0; length <= 16; length++) {
        if (!is_string_read_correct(length, strings[0])) {
            printf("      wLength %u is wrong\n", length);
            all_correct = 0;
        }
    }
    check("a string descriptor is right for every wLength from 0 to 16", all_correct);
    check("a string descriptor is right for a large wLength", is_string_read_correct(255, strings[0]));

    uint8_t* buf;
    uint16_t len;
    const uint8_t in_standard_interface = 0x81;

    dev.current_config = 0;
    check("GET_STATUS on interface 0 stalls while unconfigured",
        request(in_standard_interface, QSB_REQ_GET_STATUS, 0, 0, 2, &buf, &len) == QSB_REQ_NOTSUPP);

    dev.current_config = 1;
    check("GET_STATUS on interface 0 succeeds",
        request(in_standard_interface, QSB_REQ_GET_STATUS, 0, 0, 2, &buf, &len) == QSB_REQ_HANDLED);
    check("  and returns two zero bytes", len == 2 && buf[0] == 0 && buf[1] == 0);
    check("GET_STATUS on interface 1 succeeds",
        request(in_standard_interface, QSB_REQ_GET_STATUS, 0, 1, 2, &buf, &len) == QSB_REQ_HANDLED);
    check("GET_STATUS on interface 2 stalls",
        request(in_standard_interface, QSB_REQ_GET_STATUS, 0, 2, 2, &buf, &len) == QSB_REQ_NOTSUPP);
    check("GET_STATUS on interface 0x0100 stalls",
        request(in_standard_interface, QSB_REQ_GET_STATUS, 0, 0x100, 2, &buf, &len) == QSB_REQ_NOTSUPP);

    return check_result();
}
