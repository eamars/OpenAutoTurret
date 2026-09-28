#include <string.h>
#include "tusb.h"
#include "pico/unique_id.h"

// TinyUSB example VID/PID: local development device, not a commercial allocation.
static const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = 64, .idVendor = 0xcafe, .idProduct = 0x4001,
    .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2,
    .iSerialNumber = 3, .bNumConfigurations = 1,
};
static const uint8_t config[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64),
};
const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&device; }
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; return config;
}
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t descriptor[64];
    static char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    if (index == 0) { descriptor[0] = (TUSB_DESC_STRING << 8) | 4; descriptor[1] = 0x0409; return descriptor; }
    pico_get_unique_board_id_string(serial, sizeof(serial));
    const char *strings[] = { "", "Local Pico Bridge", "Pico W PIO UART Bridge", serial, "UART GP0 TX / GP1 RX" };
    if (index >= sizeof(strings) / sizeof(strings[0])) return NULL;
    size_t n = strlen(strings[index]); if (n > 63) n = 63;
    for (size_t i = 0; i < n; ++i) descriptor[i + 1] = strings[index][i];
    descriptor[0] = (TUSB_DESC_STRING << 8) | (2 * n + 2);
    return descriptor;
}
