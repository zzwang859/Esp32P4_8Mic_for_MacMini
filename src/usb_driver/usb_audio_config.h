#pragma once

#include <stddef.h>
#include <stdint.h>

/* TinyUSB function indices follow the IAD order in both speed descriptors.
 * Capture1/Capture2 retain their logical raw/processed meanings. */
#define USB_AUDIO_FUNCTION_COUNT  2U
#define USB_AUDIO_PROCESSED_FUNC  0U
#define USB_AUDIO_RAW_FUNC        1U
#define USB_AUDIO_PLAYBACK_FUNC   USB_AUDIO_RAW_FUNC

#define USB_AUDIO_MIC_SAMPLE_RATE_HZ   24000U
#define USB_AUDIO_MIC2_SAMPLE_RATE_HZ  16000U
#define USB_AUDIO_SPK_SAMPLE_RATE_HZ   16000U
#define USB_AUDIO_MAX_SAMPLE_RATE_HZ \
    ((USB_AUDIO_MIC_SAMPLE_RATE_HZ > USB_AUDIO_SPK_SAMPLE_RATE_HZ) \
         ? USB_AUDIO_MIC_SAMPLE_RATE_HZ : USB_AUDIO_SPK_SAMPLE_RATE_HZ)
#define USB_AUDIO_BYTES_PER_SAMPLE     2U
#define USB_AUDIO_SAMPLE_RESOLUTION    16U
#define USB_AUDIO_MIC_CHANNELS         8U
#define USB_AUDIO_MIC2_CHANNELS        2U
#define USB_AUDIO_SPK_CHANNELS         2U

/* Queue operating levels, not capacities. Keep room for producer bursts and
 * scheduling jitter without routinely queuing the entire backing buffer. */
#define USB_AUDIO_RX_DMA_PERIOD_MS     4U
#define USB_AUDIO_MIC_PRIME_MS         4U
#define USB_AUDIO_MIC_LOW_MS           4U
#define USB_AUDIO_MIC_HIGH_MS          12U
/* Dual capture + model recovery measured less than 8 ms of low-water reserve
 * at a 40-ms target. Retain additional margin for NS scheduling jitter. */
#define USB_AUDIO_MIC2_PRIME_MS        48U
/* After an AFE reset, first output bursts do not yet establish its steady
 * cadence. Buffer three 32-ms AFE blocks before ending maintenance silence.
 * Normal streaming keeps the existing 48-ms occupancy target. */
#define USB_AUDIO_MIC2_RECOVERY_MS     96U
#define USB_AUDIO_SPK_PRIME_MS         4U
#define USB_AUDIO_SPK_TARGET_MS        8U

/* Diagnostic capture source: 1 uses the internal LUT tone, 0 uses TDM RX. */
#define USB_AUDIO_MIC_TEST_TONE_ENABLED  0
#define USB_AUDIO_MIC_TEST_TONE_HZ       1000U

#define USB_AUDIO_MIC_FRAME_BYTES \
    (USB_AUDIO_MIC_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE)
#define USB_AUDIO_SPK_FRAME_BYTES \
    (USB_AUDIO_SPK_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE)
#define USB_AUDIO_MIC2_FRAME_BYTES \
    (USB_AUDIO_MIC2_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE)

/* One millisecond of interleaved PCM data. */
#define USB_AUDIO_MIC_MS_BYTES \
    ((USB_AUDIO_MIC_SAMPLE_RATE_HZ / 1000U) * USB_AUDIO_MIC_FRAME_BYTES)
#define USB_AUDIO_SPK_MS_BYTES \
    ((USB_AUDIO_SPK_SAMPLE_RATE_HZ / 1000U) * USB_AUDIO_SPK_FRAME_BYTES)
#define USB_AUDIO_MIC2_MS_BYTES \
    ((USB_AUDIO_MIC2_SAMPLE_RATE_HZ / 1000U) * USB_AUDIO_MIC2_FRAME_BYTES)

/* Endpoint scratch capacity: one millisecond of PCM plus one source frame. */
#define USB_AUDIO_MIC_EP_SIZE \
    ((((USB_AUDIO_MIC_SAMPLE_RATE_HZ + 999U) / 1000U) + 1U) * USB_AUDIO_MIC_FRAME_BYTES)
#define USB_AUDIO_SPK_EP_SIZE \
    ((((USB_AUDIO_SPK_SAMPLE_RATE_HZ + 999U) / 1000U) + 1U) * USB_AUDIO_SPK_FRAME_BYTES)
#define USB_AUDIO_MIC2_EP_SIZE \
    ((((USB_AUDIO_MIC2_SAMPLE_RATE_HZ + 999U) / 1000U) + 1U) * USB_AUDIO_MIC2_FRAME_BYTES)

#define USB_AUDIO_MIC_RING_BYTES       (USB_AUDIO_MIC_MS_BYTES * 50U)
/* The 8-ms I2S DMA can move data in bursts. Leave headroom above the
 * feedback target for those bursts without discarding USB packets. */
#define USB_AUDIO_SPK_RING_BYTES       (USB_AUDIO_SPK_MS_BYTES * 40U)
#define USB_AUDIO_MIC2_RING_BYTES      (USB_AUDIO_MIC2_MS_BYTES * 20U)

_Static_assert(USB_AUDIO_MIC_SAMPLE_RATE_HZ % 1000U == 0U &&
               USB_AUDIO_MIC2_SAMPLE_RATE_HZ % 1000U == 0U &&
               USB_AUDIO_SPK_SAMPLE_RATE_HZ % 1000U == 0U,
               "Audio tasks require an integer number of samples per ms");
_Static_assert(USB_AUDIO_MIC2_SAMPLE_RATE_HZ == 16000U &&
               USB_AUDIO_MIC2_CHANNELS == 2U,
               "Capture2 requires 16-kHz stereo: left audio, right VAD");
_Static_assert(USB_AUDIO_MIC_LOW_MS <= USB_AUDIO_MIC_PRIME_MS &&
               USB_AUDIO_MIC_PRIME_MS < USB_AUDIO_MIC_HIGH_MS &&
               USB_AUDIO_MIC_LOW_MS >= USB_AUDIO_RX_DMA_PERIOD_MS,
               "Raw capture watermarks must cover one TDM DMA period");
_Static_assert(USB_AUDIO_MIC2_PRIME_MS > 32U && USB_AUDIO_MIC2_PRIME_MS < 128U,
               "Capture2 prime must cover an AFE block and fit the stereo FIFO");
_Static_assert(USB_AUDIO_MIC2_RECOVERY_MS >= USB_AUDIO_MIC2_PRIME_MS &&
               USB_AUDIO_MIC2_RECOVERY_MS < 128U,
               "Capture2 recovery must cover priming and fit the stereo FIFO");
_Static_assert(USB_AUDIO_SPK_PRIME_MS >= 4U &&
               USB_AUDIO_SPK_TARGET_MS >= USB_AUDIO_SPK_PRIME_MS,
               "Playback must retain at least two 2-ms DMA periods of reserve");
_Static_assert(USB_AUDIO_SPK_SAMPLE_RATE_HZ == 16000U &&
               USB_AUDIO_SPK_CHANNELS == 2U &&
               USB_AUDIO_SAMPLE_RESOLUTION == 16U,
               "USB playback must be 16-kHz stereo 16-bit PCM");
_Static_assert(USB_AUDIO_MIC_EP_SIZE <= 1023U,
               "Microphone stream exceeds the isochronous endpoint limit");
_Static_assert(USB_AUDIO_SPK_EP_SIZE <= 1023U,
               "Speaker stream exceeds the isochronous endpoint limit");
_Static_assert(USB_AUDIO_MIC2_EP_SIZE <= 1023U,
               "Second microphone stream exceeds the endpoint limit");
