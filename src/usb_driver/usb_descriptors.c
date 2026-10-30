#include <string.h>
#include <stdio.h>
#include "esp_mac.h"
#include "tusb.h"
#include "usb_audio_descriptors.h"

#define USB_VID                 0x303A
#define USB_PID                 0x8003
#define EPNUM_AUDIO_OUT         0x01
#define EPNUM_AUDIO_FB          0x81
#define EPNUM_AUDIO_IN          0x82
#define EPNUM_AUDIO2_IN         0x83
#define EPNUM_HID_IN            0x84
#define EPNUM_CUSTOM_OUT        0x05
#define EPNUM_CUSTOM_IN         0x85
#define CONFIG_TOTAL_LEN        (TUD_CONFIG_DESC_LEN + USB_AUDIO_DESC_LEN + \
                                 USB_AUDIO2_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

static const uint8_t s_hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};
static const uint8_t s_custom_report_descriptor[] = {
    0x06, 0x00, 0xff, /* Vendor usage page 0xff00 */
    0x09, 0x01, 0xa1, 0x01, /* Usage 1, Application collection */
    0x15, 0x00, 0x26, 0xff, 0x00,
    0x75, 0x08, 0x95, 0x40, /* 64 bytes, no Report ID */
    0x09, 0x01, 0x81, 0x02, /* Input */
    0x09, 0x01, 0x91, 0x02, /* Output */
    0xc0,
};

static const tusb_desc_device_t s_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0104,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const uint8_t s_fs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    USB_AUDIO2_UAC2_CAPTURE_DESCRIPTOR(EPNUM_AUDIO2_IN,
                                       USB_AUDIO_MIC2_EP_SIZE, 1),
    USB_AUDIO_UAC2_DESCRIPTOR(EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN, EPNUM_AUDIO_FB,
                              USB_AUDIO_SPK_EP_SIZE,
                              USB_AUDIO_MIC_EP_SIZE, 1, 1, 1),
    TUD_HID_DESCRIPTOR(ITF_NUM_HID_KEYBOARD, 9, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(s_hid_report_descriptor), EPNUM_HID_IN,
                       8, 10),
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID_CUSTOM, 10, HID_ITF_PROTOCOL_NONE,
        sizeof(s_custom_report_descriptor), EPNUM_CUSTOM_OUT, EPNUM_CUSTOM_IN, 64, 1),
};

TU_VERIFY_STATIC(sizeof(s_fs_configuration) == CONFIG_TOTAL_LEN,
                 "incorrect FS configuration descriptor size");

/*
 * At High Speed, poll speaker OUT every microframe.  Keep capture and the
 * explicit feedback endpoint at 1 ms (bInterval 4).  This matches TinyUSB's
 * HS asynchronous-speaker scheduling and prevents the host OUT pipe from
 * remaining idle after alt setting 1 is selected.
 */
static const uint8_t s_hs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    USB_AUDIO2_UAC2_CAPTURE_DESCRIPTOR(EPNUM_AUDIO2_IN,
                                       USB_AUDIO_MIC2_EP_SIZE, 4),
    USB_AUDIO_UAC2_DESCRIPTOR(EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN, EPNUM_AUDIO_FB,
                              USB_AUDIO_SPK_EP_SIZE,
                              USB_AUDIO_MIC_EP_SIZE, 1, 4, 4),
    TUD_HID_DESCRIPTOR(ITF_NUM_HID_KEYBOARD, 9, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(s_hid_report_descriptor), EPNUM_HID_IN,
                       8, 7),
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID_CUSTOM, 10, HID_ITF_PROTOCOL_NONE,
        sizeof(s_custom_report_descriptor), EPNUM_CUSTOM_OUT, EPNUM_CUSTOM_IN, 64, 4),
};

TU_VERIFY_STATIC(sizeof(s_hs_configuration) == CONFIG_TOTAL_LEN,
                 "incorrect HS configuration descriptor size");

static const tusb_desc_device_qualifier_t s_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved = 0,
};

static uint8_t s_other_speed_configuration[CONFIG_TOTAL_LEN];

uint8_t const *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&s_device_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return tud_speed_get() == TUSB_SPEED_HIGH ? s_hs_configuration
                                               : s_fs_configuration;
}

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (const uint8_t *)&s_device_qualifier;
}

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index;
    const uint8_t *configuration = tud_speed_get() == TUSB_SPEED_HIGH
                                       ? s_fs_configuration
                                       : s_hs_configuration;
    memcpy(s_other_speed_configuration, configuration, CONFIG_TOTAL_LEN);
    s_other_speed_configuration[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
    return s_other_speed_configuration;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return instance == 0 ? s_hid_report_descriptor : instance == 1 ? s_custom_report_descriptor : NULL;
}

static const char *const s_strings[] = {
    (const char[]){0x09, 0x04},
    "Espressif",
    "ESP32-P4 Audio + KWS Control",
    "ESP32P4-UAC2-001",
    "USB Audio",
    "Stereo Playback",
    "8 Channel Capture",
    "USB Audio Capture 2",
    "Stereo Capture",
    "USB HID Keyboard",
    "KWS Control",
};
static uint16_t s_string_descriptor[32];
static char s_serial[13];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t count;
    if (index == 0) {
        memcpy(&s_string_descriptor[1], s_strings[0], 2);
        count = 1;
    } else {
        if (index >= TU_ARRAY_SIZE(s_strings)) {
            return NULL;
        }
        const char *text = s_strings[index];
        if (index == 3) {
            if (!s_serial[0]) {
                uint8_t mac[6]; esp_efuse_mac_get_default(mac);
                snprintf(s_serial,sizeof(s_serial),"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
            }
            text = s_serial;
        }
        count = strlen(text);
        if (count > TU_ARRAY_SIZE(s_string_descriptor) - 1U) {
            count = TU_ARRAY_SIZE(s_string_descriptor) - 1U;
        }
        for (size_t i = 0; i < count; ++i) {
            s_string_descriptor[i + 1U] = (uint8_t)text[i];
        }
    }
    s_string_descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) |
                                        (2U * count + 2U));
    return s_string_descriptor;
}
