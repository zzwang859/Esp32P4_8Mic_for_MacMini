#pragma once

#include "sdkconfig.h"
#include "usb_audio_descriptors.h"

/* ESP32-P4 USB 2.0 High-Speed DWC controller is TinyUSB root hub port 1. */
#define CFG_TUSB_RHPORT1_MODE  (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#define CFG_TUSB_OS            OPT_OS_FREERTOS
#define CFG_TUD_ENABLED        1
#define CFG_TUD_MAX_SPEED      OPT_MODE_HIGH_SPEED
#define CFG_TUSB_DEBUG         0
#define CFG_TUSB_MEM_ALIGN     __attribute__((aligned(64)))
#define CFG_TUD_ENDPOINT0_SIZE 64
/* Use ESP32-P4 HS DWC2 DMA for concurrent isochronous audio endpoints.
 * TinyUSB enables cache synchronization and 64-byte endpoint alignment. */
#define CFG_TUD_DWC2_DMA_ENABLE 1
/* Backported DWC2 periodic-transfer handling for HS isochronous IN. */
#define CFG_TUD_DWC2_PTI_ENABLE 1

#define CFG_TUD_CDC            0
#define CFG_TUD_MSC            0
#define CFG_TUD_HID            2
#define CFG_TUD_HID_EP_BUFSIZE 64
#define CFG_TUD_MIDI           0
#define CFG_TUD_VENDOR         0
#define CFG_TUD_AUDIO          2

#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP                    1
/* Packet sizes are selected from the capture FIFOs in usb_audio.c. */
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL                     0
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                       USB_AUDIO_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS                      1
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                USB_AUDIO_MAX_SAMPLE_RATE_HZ
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                  USB_AUDIO_MIC_CHANNELS
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX                  USB_AUDIO_SPK_CHANNELS
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX USB_AUDIO_BYTES_PER_SAMPLE
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX         USB_AUDIO_SAMPLE_RESOLUTION
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX USB_AUDIO_BYTES_PER_SAMPLE
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX         USB_AUDIO_SAMPLE_RESOLUTION
#define CFG_TUD_AUDIO_ENABLE_EP_IN                          1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT                         1
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX                   USB_AUDIO_MIC_EP_SIZE
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX                  USB_AUDIO_SPK_EP_SIZE
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ                2048
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ               (32U * USB_AUDIO_SPK_EP_SIZE)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT                       1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ                    64

#define CFG_TUD_AUDIO_FUNC_2_DESC_LEN                       USB_AUDIO2_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_2_EP_IN_SZ_MAX                   USB_AUDIO_MIC2_EP_SIZE
#define CFG_TUD_AUDIO_FUNC_2_EP_OUT_SZ_MAX                  0
#define CFG_TUD_AUDIO_FUNC_2_EP_IN_SW_BUF_SZ                512
#define CFG_TUD_AUDIO_FUNC_2_EP_OUT_SW_BUF_SZ               0
#define CFG_TUD_AUDIO_FUNC_2_CTRL_BUF_SZ                    64
