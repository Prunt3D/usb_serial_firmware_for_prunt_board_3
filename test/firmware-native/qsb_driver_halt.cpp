// Runs the endpoint halt handling in the real qsb_drv_fsdev_plus.c against a model of the endpoint
// registers. It's compiled as C++ so that a class can stand in for the registers and see each write.
//
// The model only covers what a write does to each bit. It doesn't model USB transactions or double
// buffering conflicts, so this says nothing about those. See README.md.

#include <stdint.h>

#include "qsb_config.h"
#include "qsb_fsdev.h"

class endpoint_register {
public:
    uint16_t value = 0;

    operator uint32_t() const { return value; }

    endpoint_register& operator=(uint32_t written)
    {
        const uint16_t toggled_by_1 = USB_EP_DTOG_RX | USB_EP_STAT_RX | USB_EP_DTOG_TX | USB_EP_STAT_TX;
        const uint16_t read_write = USB_EP_TYPE | USB_EP_KIND | USB_EP_ADDR;
        const uint16_t cleared_by_0 = USB_EP_CTR_RX | USB_EP_CTR_TX;

        uint16_t next = value & USB_EP_SETUP;
        next |= written & read_write;
        next |= (value ^ written) & toggled_by_1;
        next |= value & written & cleared_by_0;
        value = next;
        return *this;
    }
};

static endpoint_register endpoint_registers[8];

#undef USB_EP
#define USB_EP(EP) endpoint_registers[EP]

#include "qsb_drv_fsdev_btable.h"
#include "qsb_fsdev_ep.h"
#include "qsb_private.h"

static uint32_t pma_writes;

void qsb_fsdev_setup_buf_rx(uint8_t, qsb_buf_desc_offset, uint32_t, uint16_t*) {}
void qsb_fsdev_setup_buf_tx(uint8_t, qsb_buf_desc_offset, uint32_t, uint16_t*) {}
uint32_t qsb_fsdev_get_len(uint8_t, qsb_buf_desc_offset) { return 0; }
void qsb_fsdev_clear_rx_len(uint8_t, qsb_buf_desc_offset) {}
void qsb_fsdev_copy_to_pma(uint8_t, qsb_buf_desc_offset, const uint8_t*, uint32_t) { pma_writes++; }
uint32_t qsb_fsdev_copy_from_pma(uint8_t*, uint32_t, uint8_t, qsb_buf_desc_offset) { return 0; }
void qsb_internal_dev_reset(qsb_device*) {}

#include "qsb_drv_fsdev_plus.c"

#include "check.h"

extern "C" void rcc_periph_clock_enable(enum rcc_periph_clken) {}

static uint32_t stat_tx(int ep) { return endpoint_registers[ep].value & USB_EP_STAT_TX; }
static uint32_t stat_rx(int ep) { return endpoint_registers[ep].value & USB_EP_STAT_RX; }

// What the peripheral does to EP0 on receiving a SETUP packet, which the driver then follows with
// its own reset of the transmit state.
static void receive_setup()
{
    endpoint_register& ep0 = endpoint_registers[0];
    ep0.value &= ~(USB_EP_STAT_TX | USB_EP_STAT_RX);
    ep0.value |= USB_EP_STAT_TX_NAK | USB_EP_STAT_RX_NAK | USB_EP_DTOG_TX | USB_EP_DTOG_RX;
    device_fsdev.ep_state_tx[0] = sgl_buf_0_pkts;
}

static void callback(qsb_device*, uint8_t, uint32_t) {}

static int out_packets;

static void count_out_packet(qsb_device*, uint8_t, uint32_t) { out_packets++; }

int main()
{
    qsb_device* dev = &device_fsdev;
    static const uint8_t packet[10] = {};

    qsb_dev_ep_setup(dev, 0, QSB_ENDPOINT_ATTR_CONTROL, 16, NULL);

    // CLEAR_FEATURE(ENDPOINT_HALT) on EP0: the status stage of the same request must stay DATA1
    receive_setup();
    qsb_dev_ep_stall_set(dev, 0x80, 0);
    int result = qsb_dev_ep_transmit_packet(dev, 0, NULL, 0);
    check("EP0 IN: clearing a halt keeps DTOG_TX at 1 for the status stage",
        (endpoint_registers[0].value & USB_EP_DTOG_TX) != 0);
    check("EP0 IN: the status packet is submitted", result == 0 && stat_tx(0) == USB_EP_STAT_TX_VALID);

    receive_setup();
    qsb_dev_ep_stall_set(dev, 0x00, 0);
    result = qsb_dev_ep_transmit_packet(dev, 0, NULL, 0);
    check("EP0 OUT: clearing a halt keeps DTOG_RX", (endpoint_registers[0].value & USB_EP_DTOG_RX) != 0);
    check("EP0 OUT: the status packet is submitted", result == 0 && stat_tx(0) == USB_EP_STAT_TX_VALID);

    // EP0 still has to respond after a SET_FEATURE(ENDPOINT_HALT) addressed to it
    receive_setup();
    qsb_dev_ep_stall_set(dev, 0x80, 1);
    result = qsb_dev_ep_transmit_packet(dev, 0, NULL, 0);
    check("EP0: the status of SET_FEATURE can still be sent", result == 0 && stat_tx(0) == USB_EP_STAT_TX_VALID);

    // Rejecting a request stalls EP0
    receive_setup();
    qsb_dev_ep_stall_set(dev, 0, 1);
    check("EP0: a rejected request stalls IN", stat_tx(0) == USB_EP_STAT_TX_STALL);

    // Notification endpoint, which is single-buffered
    qsb_dev_ep_setup(dev, 0x83, QSB_ENDPOINT_ATTR_INTERRUPT, 16, callback);
    result = qsb_dev_ep_transmit_packet(dev, 0x83, packet, sizeof(packet));
    check("EP3: a notification goes out normally", result == 10 && stat_tx(3) == USB_EP_STAT_TX_VALID);

    // As after the packet has been sent
    dev->ep_state_tx[3] = sgl_buf_0_pkts;
    endpoint_registers[3].value = (endpoint_registers[3].value & ~USB_EP_STAT_TX) | USB_EP_STAT_TX_NAK;

    qsb_dev_ep_stall_set(dev, 0x83, 1);
    uint32_t pma_writes_before = pma_writes;
    result = qsb_dev_ep_transmit_packet(dev, 0x83, packet, sizeof(packet));
    check("EP3 halted: a notification is refused", result == -1);
    check("EP3 halted: the endpoint stays halted", stat_tx(3) == USB_EP_STAT_TX_STALL);
    check("EP3 halted: GET_STATUS still reports the halt", qsb_dev_ep_stall_get(dev, 0x83) == 1);
    check("EP3 halted: nothing is written to the PMA", pma_writes == pma_writes_before);

    endpoint_registers[3].value |= USB_EP_DTOG_TX; // as after an odd number of packets
    qsb_dev_ep_stall_set(dev, 0x83, 0);
    check("EP3: clearing the halt resets DTOG_TX to DATA0", (endpoint_registers[3].value & USB_EP_DTOG_TX) == 0);
    result = qsb_dev_ep_transmit_packet(dev, 0x83, packet, sizeof(packet));
    check("EP3: notifications resume after the halt is cleared", result == 10 && stat_tx(3) == USB_EP_STAT_TX_VALID);

    // Data IN endpoint, which is double-buffered
    qsb_dev_ep_setup(dev, 0x82, QSB_ENDPOINT_ATTR_BULK, 128, callback);
    qsb_dev_ep_stall_set(dev, 0x82, 1);
    check("EP2: SET_FEATURE halts it", stat_tx(2) == USB_EP_STAT_TX_STALL);
    check("EP2 halted: no room is reported", qsb_dev_ep_transmit_avail(dev, 0x82) == 0);

    endpoint_registers[2].value ^= USB_EP_DTOG_TX; // DTOG_TX and SW_BUF_TX differ, which looks like room
    result = qsb_dev_ep_transmit_packet(dev, 0x82, packet, sizeof(packet));
    check("EP2 halted: a packet is refused", result == -1);
    check("EP2 halted: the endpoint stays halted", stat_tx(2) == USB_EP_STAT_TX_STALL);
    check("EP2 halted: double buffering stays off", (endpoint_registers[2].value & USB_EP_KIND) == 0);

    qsb_dev_ep_stall_set(dev, 0x82, 0);
    check("EP2: clearing the halt restarts it with both buffers looking full",
        stat_tx(2) == USB_EP_STAT_TX_VALID && qsb_dev_ep_transmit_avail(dev, 0x82) == 0);

    // Data OUT endpoint, which is double-buffered
    qsb_dev_ep_setup(dev, 0x01, QSB_ENDPOINT_ATTR_BULK, 128, callback);
    endpoint_registers[1].value |= USB_EP_DTOG_RX;
    qsb_dev_ep_stall_set(dev, 0x01, 0);
    check("EP1: clearing a halt resets DTOG_RX to DATA0", (endpoint_registers[1].value & USB_EP_DTOG_RX) == 0);
    check("EP1: the endpoint is receiving again", stat_rx(1) == USB_EP_STAT_RX_VALID);

    // A packet waiting through a pause is read at the end of it
    qsb_dev_ep_setup(dev, 0x01, QSB_ENDPOINT_ATTR_BULK, 128, count_out_packet);
    qsb_dev_ep_pause(dev, 0x01);
    endpoint_registers[1].value ^= USB_EP_DTOG_RX; // as after a packet arrives
    qsb_dev_ep_unpause(dev, 0x01);
    check("EP1: a packet waiting through a pause is read after it", out_packets == 1);

    // But not if the endpoint was halted in the meantime
    out_packets = 0;
    qsb_dev_ep_setup(dev, 0x01, QSB_ENDPOINT_ATTR_BULK, 128, count_out_packet);
    qsb_dev_ep_pause(dev, 0x01);
    endpoint_registers[1].value ^= USB_EP_DTOG_RX;
    qsb_dev_ep_stall_set(dev, 0x01, 1);
    qsb_dev_ep_unpause(dev, 0x01);
    check("EP1 halted: a waiting packet isn't read", out_packets == 0);
    check("EP1 halted: double buffering stays off", (endpoint_registers[1].value & USB_EP_KIND) == 0);
    check("EP1 halted: the endpoint stays halted", stat_rx(1) == USB_EP_STAT_RX_STALL);

    qsb_dev_ep_stall_set(dev, 0x01, 0);
    dbl_buf_read_rx(dev, 1);
    check("EP1: clearing the halt drops what was waiting", out_packets == 0);
    check("EP1: the endpoint is receiving after the halt", stat_rx(1) == USB_EP_STAT_RX_VALID);

    // As when CTR_RX is handled during a halt, with nothing waiting
    qsb_dev_ep_stall_set(dev, 0x01, 1);
    dbl_buf_read_rx(dev, 1);
    check("EP1 halted: handling CTR_RX leaves double buffering off", (endpoint_registers[1].value & USB_EP_KIND) == 0);

    // SET_CONFIGURATION(0) disables the endpoints, whether they're halted or not
    static qsb_device_desc device_desc = {};
    device_desc.bMaxPacketSize0 = 16;
    dev->desc = &device_desc;
    qsb_dev_ep_setup(dev, 0x82, QSB_ENDPOINT_ATTR_BULK, 128, callback);
    qsb_internal_ep_reset(dev);
    check("resetting endpoints disables EP1, which was halted", stat_rx(1) == USB_EP_STAT_RX_DISABLED);
    check("resetting endpoints disables EP2", stat_tx(2) == USB_EP_STAT_TX_DISABLED);
    check("  and turns double buffering off",
        (endpoint_registers[1].value & USB_EP_KIND) == 0 && (endpoint_registers[2].value & USB_EP_KIND) == 0);

    return check_result();
}
