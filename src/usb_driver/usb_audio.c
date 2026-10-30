#include "usb_audio.h"
#include "custom_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "audio_backend.h"
#include "audio_diagnostics.h"
#include "audio_pipeline.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_private/usb_phy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "tusb.h"
#include "usb_audio_config.h"
#include "usb_audio_descriptors.h"
#include "usb_hid_keyboard.h"

static const char *TAG = "usb_audio";
/* Prime and control the average Capture2 occupancy at 48 ms. Individual
 * AFE/NS bursts must not be interpreted as source-clock drift. */
#define MIC2_SAMPLES_PER_MS       (USB_AUDIO_MIC2_SAMPLE_RATE_HZ / 1000U)
#define MIC2_PRIME_SAMPLES        (MIC2_SAMPLES_PER_MS * USB_AUDIO_MIC2_PRIME_MS)
#define SPK_FEEDBACK_TARGET_BYTES (USB_AUDIO_SPK_MS_BYTES * USB_AUDIO_SPK_TARGET_MS)

static usb_phy_handle_t s_phy;
static bool s_maintenance;
static bool s_capture_resuming[2], s_capture_recovering[2];
#if !USB_AUDIO_MIC_TEST_TONE_ENABLED
static StreamBufferHandle_t s_mic_stream[USB_AUDIO_FUNCTION_COUNT];
#endif
static StreamBufferHandle_t s_spk_stream;
static volatile bool s_mic_active[USB_AUDIO_FUNCTION_COUNT];
static volatile bool s_spk_active;
static volatile uint32_t s_spk_generation;
static portMUX_TYPE s_mic_fill_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_spk_usb_packets;
static volatile uint32_t s_spk_usb_bytes;
static volatile uint32_t s_spk_fifo_bytes;
static volatile uint32_t s_spk_empty_callbacks;
static volatile uint32_t s_spk_dropped_bytes;
static volatile uint32_t s_spk_i2s_bytes;
static volatile uint32_t s_spk_underflows;
static volatile uint32_t s_spk_silence_bytes;
static volatile uint32_t s_spk_i2s_errors;
static uint32_t s_spk_feedback_level_q8;
static volatile uint32_t s_spk_feedback_value;
static volatile uint32_t s_spk_queue_peak_bytes;
static TaskHandle_t s_playback_log_task;
enum {
    PLAYBACK_FAULT_OVERFLOW = 1U << 0,
    PLAYBACK_FAULT_UNDERRUN = 1U << 1,
    PLAYBACK_FAULT_I2S = 1U << 2,
};
static bool s_mic_primed[USB_AUDIO_FUNCTION_COUNT];
static volatile uint32_t s_raw_fifo_overflows;
static volatile uint32_t s_i2s_read_anomalies;
static volatile uint32_t s_mic_endpoint_null_packets[USB_AUDIO_FUNCTION_COUNT];
static volatile uint32_t s_mic_endpoint_fifo_backpressure;
static volatile uint32_t s_mic_endpoint_write_shorts;
static bool s_mono_tx_started;
static volatile uint32_t s_mono_tx_packets;
static volatile uint32_t s_mono_tx_zero_packets;
static volatile uint32_t s_mono_tx_zero_scheduled;
static volatile uint32_t s_mono_tx_zero_failed;
static volatile uint32_t s_mono_tx_zero_unexpected;
static volatile uint16_t s_mono_expected_bytes;
static volatile uint32_t s_mono_tx_unexpected_sizes;
static volatile uint32_t s_mono_source_short_reads;
static volatile uint32_t s_mono_source_read_packets;
static volatile uint32_t s_mono_source_zero_packets;
static volatile uint32_t s_mono_source_min_samples = UINT32_MAX;
static volatile uint32_t s_mono_source_frames, s_mono_source_zero_pairs;
static volatile uint32_t s_mono_source_packets_15, s_mono_source_packets_16, s_mono_source_packets_17;
static bool s_mic2_rate_initialized;
static int32_t s_mic2_level_q8, s_mic2_rate_phase_q16;

#if USB_AUDIO_MIC_TEST_TONE_ENABLED
#define MIC_TEST_TONE_LUT_SIZE  24U

/* One 1-kHz period at 24 kHz, amplitude 16000 (about -6.2 dBFS). */
static const int16_t s_mic_test_tone_lut[MIC_TEST_TONE_LUT_SIZE] = {
         0,   4141,   8000,  11314,  13856,  15455,
     16000,  15455,  13856,  11314,   8000,   4141,
         0,  -4141,  -8000, -11314, -13856, -15455,
    -16000, -15455, -13856, -11314,  -8000,  -4141,
};

static uint32_t s_mic_test_tone_phase[USB_AUDIO_FUNCTION_COUNT];

_Static_assert(USB_AUDIO_MIC_SAMPLE_RATE_HZ ==
                   USB_AUDIO_MIC_TEST_TONE_HZ * MIC_TEST_TONE_LUT_SIZE,
               "1-kHz test-tone LUT requires a 24-kHz capture rate");

static void fill_mic_test_tone(uint8_t func_id, int16_t *samples,
                               size_t frame_count, size_t channel_count)
{
    for (size_t frame = 0; frame < frame_count; ++frame) {
        const int16_t positive =
            s_mic_test_tone_lut[s_mic_test_tone_phase[func_id]];
        for (size_t channel = 0; channel < channel_count; ++channel) {
            /* Human CH1/3/5/7 are positive; CH2/4/6/8 are 180 degrees inverted. */
            samples[frame * channel_count + channel] =
                (channel & 1U) == 0U ? positive : (int16_t)-positive;
        }
        s_mic_test_tone_phase[func_id] =
            (s_mic_test_tone_phase[func_id] + 1U) % MIC_TEST_TONE_LUT_SIZE;
    }
}
#endif

static size_t capture2_packet_frames(size_t buffered, size_t nominal)
{
    /* AFE/NS delivers bursts, not a sample-clock measurement every millisecond.
     * Filter 128 USB intervals, then correct occupancy over about two seconds.
     * Fractional accumulation distributes individual +/-1-frame corrections;
     * the 1/16 limit permits +/-3906 ppm at 16 kHz without packet-size bursts. */
    const int32_t target_q8 = (int32_t)MIC2_PRIME_SAMPLES * 256;
    if (!s_mic2_rate_initialized) {
        s_mic2_level_q8 = target_q8;
        s_mic2_rate_phase_q16 = 0;
        s_mic2_rate_initialized = true;
    }
    s_mic2_level_q8 += ((int32_t)buffered * 256 - s_mic2_level_q8) / 128;
    int32_t correction_q16 = (s_mic2_level_q8 - target_q8) / 8;
    if (correction_q16 > 4096) correction_q16 = 4096;
    if (correction_q16 < -4096) correction_q16 = -4096;
    s_mic2_rate_phase_q16 += correction_q16;
    if (s_mic2_rate_phase_q16 >= 65536) {
        s_mic2_rate_phase_q16 -= 65536;
        return nominal + 1U;
    }
    if (s_mic2_rate_phase_q16 <= -65536) {
        s_mic2_rate_phase_q16 += 65536;
        return nominal - 1U;
    }
    return nominal;
}

static void fill_capture2_fifo(void)
{
    const uint8_t func_id = USB_AUDIO_PROCESSED_FUNC;
    _Static_assert(sizeof(audio_pipeline_capture2_frame_t) == USB_AUDIO_MIC2_FRAME_BYTES,
                   "Capture2 audio/VAD frame must match the USB stereo format");
    audio_pipeline_capture2_frame_t packet[USB_AUDIO_MIC2_EP_SIZE / USB_AUDIO_MIC2_FRAME_BYTES];
    const size_t packet_frames = USB_AUDIO_MIC2_SAMPLE_RATE_HZ / 1000U;
    const size_t buffered = audio_pipeline_available();
    if (buffered >= MIC2_PRIME_SAMPLES) s_mic_primed[func_id] = true;
    if (!s_mic_primed[func_id]) {
        s_mic2_rate_initialized = false;
        return;
    }
    if (buffered < packet_frames - 1U) {
        ++s_mic_endpoint_null_packets[func_id];
        s_mic_primed[func_id] = false;
        s_mic2_rate_initialized = false;
        return;
    }
    if (buffered < s_mono_source_min_samples)
        s_mono_source_min_samples = (uint32_t)buffered;
    const bool previous_initialized = s_mic2_rate_initialized;
    const int32_t previous_level = s_mic2_level_q8;
    const int32_t previous_phase = s_mic2_rate_phase_q16;
    size_t wanted = capture2_packet_frames(buffered, packet_frames);
    if (wanted > buffered) wanted = buffered;
    const uint16_t wanted_bytes = (uint16_t)(wanted * USB_AUDIO_MIC2_FRAME_BYTES);
    tu_fifo_t *const usb_fifo = tud_audio_n_get_ep_in_ff(func_id);
    if (!usb_fifo || tu_fifo_remaining(usb_fifo) < wanted_bytes) {
        /* No packet was queued: do not consume a fractional clock correction. */
        s_mic2_rate_initialized = previous_initialized;
        s_mic2_level_q8 = previous_level;
        s_mic2_rate_phase_q16 = previous_phase;
        ++s_mic_endpoint_fifo_backpressure;
        return;
    }
    // Audio and VAD are already paired in the pipeline FIFO, including when
    // one USB packet spans two AFE frames with different VAD decisions.
    const size_t got = audio_pipeline_read(packet, wanted);
    s_mono_source_frames += got;
    for (size_t i = 0; i < got; ++i) {
        if (packet[i].audio == 0 && packet[i].right == 0) ++s_mono_source_zero_pairs;
    }
    if (got == 15U) ++s_mono_source_packets_15;
    else if (got == 16U) ++s_mono_source_packets_16;
    else if (got == 17U) ++s_mono_source_packets_17;
    if (got != wanted) ++s_mono_source_short_reads;
    if (got != 0U) {
        ++s_mono_source_read_packets;
        bool all_zero = true;
        for (size_t i = 0; i < got; ++i) {
            if (packet[i].audio != 0) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) ++s_mono_source_zero_packets;
    }
    const uint16_t bytes = (uint16_t)(got * USB_AUDIO_MIC2_FRAME_BYTES);
    if (tud_audio_n_write(func_id, packet, bytes) != bytes)
        ++s_mic_endpoint_write_shorts;
}

static void fill_mic_fifo_impl(uint8_t func_id, bool in_isr, BaseType_t *task_woken)
{
    (void)in_isr;
    (void)task_woken;
    if (func_id >= USB_AUDIO_FUNCTION_COUNT) {
        return;
    }
    if (s_capture_resuming[func_id]) {
        s_capture_resuming[func_id]=false;
        s_mic_primed[func_id]=false;
        s_mic2_rate_initialized=false;
        if (func_id == USB_AUDIO_PROCESSED_FUNC) {
            audio_pipeline_capture2_frame_t discard[64];
            while (audio_pipeline_read(discard,64) != 0) {}
        }
#if !USB_AUDIO_MIC_TEST_TONE_ENABLED
        else {
            uint8_t discard[USB_AUDIO_MIC_MS_BYTES];
            while ((in_isr ? xStreamBufferReceiveFromISR(s_mic_stream[func_id],discard,sizeof(discard),task_woken) :
                xStreamBufferReceive(s_mic_stream[func_id],discard,sizeof(discard),0)) != 0) {}
        }
#endif
    }
    if (s_capture_recovering[func_id] && !s_maintenance) {
        size_t buffered=func_id == USB_AUDIO_PROCESSED_FUNC ? audio_pipeline_available() : 0;
#if !USB_AUDIO_MIC_TEST_TONE_ENABLED
        if (func_id == USB_AUDIO_RAW_FUNC) buffered=xStreamBufferBytesAvailable(s_mic_stream[func_id])/USB_AUDIO_MIC_FRAME_BYTES;
#else
        if (func_id == USB_AUDIO_RAW_FUNC) s_capture_recovering[func_id]=false;
#endif
        size_t threshold=func_id == USB_AUDIO_PROCESSED_FUNC ?
            USB_AUDIO_MIC2_RECOVERY_MS * MIC2_SAMPLES_PER_MS :
            USB_AUDIO_MIC_PRIME_MS * (USB_AUDIO_MIC_SAMPLE_RATE_HZ / 1000U);
        if (buffered >= threshold) s_capture_recovering[func_id]=false;
    }
    if (s_maintenance || s_capture_recovering[func_id]) {
        int16_t zero[USB_AUDIO_MIC_MS_BYTES / sizeof(int16_t)]={0};
        uint16_t bytes=func_id == USB_AUDIO_PROCESSED_FUNC ? USB_AUDIO_MIC2_MS_BYTES : USB_AUDIO_MIC_MS_BYTES;
        if (tud_audio_n_write(func_id,zero,bytes) != bytes) ++s_mic_endpoint_write_shorts;
        return;
    }
    if (func_id == USB_AUDIO_PROCESSED_FUNC) {
        fill_capture2_fifo();
        return;
    }
#if USB_AUDIO_MIC_TEST_TONE_ENABLED
    int16_t packet[USB_AUDIO_MIC_MS_BYTES / sizeof(int16_t)];
    const size_t channel_count = func_id == USB_AUDIO_RAW_FUNC
                                     ? USB_AUDIO_MIC_CHANNELS
                                     : USB_AUDIO_MIC2_CHANNELS;
    const size_t wanted = func_id == USB_AUDIO_RAW_FUNC
                              ? USB_AUDIO_MIC_MS_BYTES
                              : USB_AUDIO_MIC2_MS_BYTES;
    fill_mic_test_tone(func_id, packet,
                       USB_AUDIO_MIC_SAMPLE_RATE_HZ / 1000U,
                       channel_count);
    (void)tud_audio_n_write(func_id, packet, (uint16_t)wanted);
#else
    int16_t packet[USB_AUDIO_MIC_EP_SIZE / sizeof(int16_t)];
    const size_t packet_frames = USB_AUDIO_MIC_SAMPLE_RATE_HZ / 1000U;
    const size_t buffered = xStreamBufferBytesAvailable(s_mic_stream[func_id]) /
                            USB_AUDIO_MIC_FRAME_BYTES;
    if (buffered >= USB_AUDIO_MIC_PRIME_MS * packet_frames) s_mic_primed[func_id] = true;
    if (!s_mic_primed[func_id]) return;
    size_t wanted = packet_frames;
    if (buffered > USB_AUDIO_MIC_HIGH_MS * packet_frames) ++wanted;
    else if (buffered > 0U && buffered < USB_AUDIO_MIC_LOW_MS * packet_frames) --wanted;
    if (buffered < packet_frames - 1U) {
        ++s_mic_endpoint_null_packets[func_id];
        s_mic_primed[func_id] = false;
        return;
    }
    if (wanted > buffered) wanted = buffered;
    const size_t wanted_bytes = wanted * USB_AUDIO_MIC_FRAME_BYTES;
    tu_fifo_t *const usb_fifo = tud_audio_n_get_ep_in_ff(func_id);
    if (!usb_fifo || tu_fifo_remaining(usb_fifo) < wanted_bytes) {
        ++s_mic_endpoint_fifo_backpressure;
        return;
    }
    const size_t got = in_isr ?
        xStreamBufferReceiveFromISR(s_mic_stream[func_id], packet, wanted_bytes, task_woken) :
        xStreamBufferReceive(s_mic_stream[func_id], packet, wanted_bytes, 0);
    if (tud_audio_n_write(func_id, packet, (uint16_t)got) != got)
        ++s_mic_endpoint_write_shorts;
#endif
}

static void fill_mic_fifo(uint8_t func_id)
{
    const bool in_isr = xPortInIsrContext();
    BaseType_t task_woken = pdFALSE;
    if (in_isr) portENTER_CRITICAL_ISR(&s_mic_fill_lock);
    else portENTER_CRITICAL(&s_mic_fill_lock);
    fill_mic_fifo_impl(func_id, in_isr, &task_woken);
    if (in_isr) portEXIT_CRITICAL_ISR(&s_mic_fill_lock);
    else portEXIT_CRITICAL(&s_mic_fill_lock);
    if (task_woken) portYIELD_FROM_ISR();
}

static void tinyusb_task(void *arg)
{
    (void)arg;
    while (true) {
        tud_task();
    }
}

static void update_speaker_feedback(void)
{
    /* TinyUSB's OUT FIFO is drained in the receive callback. The queue that
     * actually feeds I2S is s_spk_stream, so regulate its target level. */
    const uint32_t target = SPK_FEEDBACK_TARGET_BYTES;
    const uint32_t level = (uint32_t)xStreamBufferBytesAvailable(s_spk_stream);
    /* OUT arrives every 125 us at HS. Smooth over about 8 ms so I2S DMA
     * bursts do not make the advertised sample rate oscillate. */
    const int32_t divisor = tud_speed_get() == TUSB_SPEED_HIGH ? 64 : 8;
    int32_t filtered = (int32_t)s_spk_feedback_level_q8;
    filtered += ((int32_t)(level << 8) - filtered) / divisor;
    s_spk_feedback_level_q8 = (uint32_t)filtered;
    int32_t error = (int32_t)target -
                    (int32_t)((s_spk_feedback_level_q8 + 128U) >> 8);
    if (error > (int32_t)target) error = (int32_t)target;
    if (error < -(int32_t)target) error = -(int32_t)target;
    const uint32_t frame_divisor = tud_speed_get() == TUSB_SPEED_HIGH ? 8U : 1U;
    const int32_t nominal = (int32_t)(USB_AUDIO_SPK_SAMPLE_RATE_HZ /
                                       (1000U * frame_divisor)) << 16;
    /* A full target-level error adjusts the host by at most one sample/ms. */
    const int32_t adjustment = (int32_t)((int64_t)error * 65536 /
                                         ((int64_t)target * frame_divisor));
    s_spk_feedback_value = (uint32_t)(nominal + adjustment);
    (void)tud_audio_n_fb_set(USB_AUDIO_PLAYBACK_FUNC, s_spk_feedback_value);
}

#if !USB_AUDIO_MIC_TEST_TONE_ENABLED
static void capture_task(void *arg)
{
    (void)arg;
    int16_t block[USB_AUDIO_MIC_MS_BYTES / sizeof(int16_t)];
    TickType_t last_silence = xTaskGetTickCount();
    custom_worker_register(CUSTOM_WORK_CAPTURE);
    while (true) {
        if (custom_worker_checkpoint(CUSTOM_WORK_CAPTURE)) {
            /* A maintenance pause is a clock discontinuity. Do not replay
             * seconds of missed silent input ticks at CPU speed on resume. */
            last_silence = xTaskGetTickCount();
        }
        size_t bytes_read = 0;
        esp_err_t err;
        if (audio_pipeline_capture_available()) {
            err = audio_backend_read(block, sizeof(block), &bytes_read, 20);
        } else {
            /* Keep both capture sample clocks alive without a physical codec. */
            memset(block, 0, sizeof(block));
            bytes_read = sizeof(block);
            err = ESP_OK;
            xTaskDelayUntil(&last_silence, pdMS_TO_TICKS(1));
        }
        if (err != ESP_OK || bytes_read == 0U ||
            (bytes_read % USB_AUDIO_MIC_FRAME_BYTES) != 0U) {
            ++s_i2s_read_anomalies;
        }
        /* A timed-out I2S read may still return complete frames. Preserve
         * those samples instead of discarding the whole partial block. */
        bytes_read -= bytes_read % USB_AUDIO_MIC_FRAME_BYTES;
        if (bytes_read == 0U) {
            continue;
        }
        if (s_mic_active[USB_AUDIO_RAW_FUNC]) {
            /* Keep the DSP path independent if the raw USB host stops reading. */
            if (xStreamBufferSpacesAvailable(s_mic_stream[USB_AUDIO_RAW_FUNC]) >= bytes_read) {
                (void)xStreamBufferSend(s_mic_stream[USB_AUDIO_RAW_FUNC],
                                        block, bytes_read, 0);
            } else {
                ++s_raw_fifo_overflows;
            }
        }
        audio_pipeline_capture(block, bytes_read / USB_AUDIO_MIC_FRAME_BYTES);
    }
}
#endif

void usb_audio_get_capture_stats(usb_audio_capture_stats_t *stats)
{
    if (!stats) return;
    stats->raw_fifo_overflows = s_raw_fifo_overflows;
    stats->i2s_read_anomalies = s_i2s_read_anomalies;
    stats->raw_endpoint_null_packets =
        s_mic_endpoint_null_packets[USB_AUDIO_RAW_FUNC];
    stats->mono_endpoint_null_packets =
        s_mic_endpoint_null_packets[USB_AUDIO_PROCESSED_FUNC];
    stats->endpoint_fifo_backpressure = s_mic_endpoint_fifo_backpressure;
    stats->endpoint_write_shorts = s_mic_endpoint_write_shorts;
    stats->mono_tx_packets = s_mono_tx_packets;
    stats->mono_tx_zero_packets = s_mono_tx_zero_packets;
    stats->mono_tx_zero_scheduled = s_mono_tx_zero_scheduled;
    stats->mono_tx_zero_failed = s_mono_tx_zero_failed;
    stats->mono_tx_zero_unexpected = s_mono_tx_zero_unexpected;
    stats->mono_tx_unexpected_sizes = s_mono_tx_unexpected_sizes;
    stats->mono_source_short_reads = s_mono_source_short_reads;
    stats->mono_source_read_packets = s_mono_source_read_packets;
    stats->mono_source_zero_packets = s_mono_source_zero_packets;
    stats->mono_source_min_samples = s_mono_source_min_samples == UINT32_MAX
                                         ? 0U : s_mono_source_min_samples;
    stats->mono_source_frames = s_mono_source_frames;
    stats->mono_source_zero_pairs = s_mono_source_zero_pairs;
    stats->mono_source_packets_15 = s_mono_source_packets_15;
    stats->mono_source_packets_16 = s_mono_source_packets_16;
    stats->mono_source_packets_17 = s_mono_source_packets_17;
    stats->playback_underflows = s_spk_underflows;
    stats->playback_dropped_bytes = s_spk_dropped_bytes;
    stats->playback_i2s_errors = s_spk_i2s_errors;
    stats->playback_peak_bytes = s_spk_queue_peak_bytes;
}

static void notify_playback_fault(uint32_t faults, BaseType_t *task_woken)
{
    if (xPortInIsrContext()) {
        xTaskNotifyFromISR(s_playback_log_task, faults, eSetBits, task_woken);
    } else {
        xTaskNotify(s_playback_log_task, faults, eSetBits);
    }
}

static void playback_log_task(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t faults = 0;
        xTaskNotifyWait(0, UINT32_MAX, &faults, portMAX_DELAY);
        // UART output must never block I2S writes or the USB receive callback.
        if (faults & PLAYBACK_FAULT_OVERFLOW)
            ESP_LOGW(TAG, "playback queue overflow: USB audio dropped");
        if (faults & PLAYBACK_FAULT_UNDERRUN)
            ESP_LOGW(TAG, "playback queue underrun: silence inserted");
        if (faults & PLAYBACK_FAULT_I2S)
            ESP_LOGE(TAG, "playback I2S write failed or returned a short write");
        // Coalesce repeated faults during an error storm. With no new fault,
        // the next notification wait blocks indefinitely and prints nothing.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void playback_task(void *arg)
{
    (void)arg;
    uint8_t block[USB_AUDIO_SPK_MS_BYTES];
    uint32_t generation = s_spk_generation;
    bool primed = false;
    while (true) {
        if (generation != s_spk_generation) {
            generation = s_spk_generation;
            primed = false;
            /* Only this consumer may discard queued audio. Resetting the
             * stream from the control task would race with I2S playback. */
            while (xStreamBufferReceive(s_spk_stream, block, sizeof(block), 0) != 0) {}
        }
        size_t got = 0;
        bool read_attempted = false;
        if (s_spk_active) {
            if (!primed && xStreamBufferBytesAvailable(s_spk_stream) >=
                               USB_AUDIO_SPK_PRIME_MS * sizeof(block)) {
                primed = true;
            }
            if (primed) {
                read_attempted = true;
                const TickType_t start = xTaskGetTickCount();
                const TickType_t timeout = pdMS_TO_TICKS(2);
                while (got < sizeof(block) && s_spk_active &&
                       generation == s_spk_generation) {
                    const TickType_t elapsed = xTaskGetTickCount() - start;
                    if (elapsed >= timeout) break;
                    got += xStreamBufferReceive(s_spk_stream, block + got,
                                                sizeof(block) - got,
                                                timeout - elapsed);
                }
                if (got < sizeof(block)) primed = false;
            }
        }
        if (got < sizeof(block)) {
            /* Output silence when the USB playback stream has insufficient data. */
            memset(block + got, 0, sizeof(block) - got);
            if (s_spk_active) {
                s_spk_silence_bytes += sizeof(block) - got;
                if (read_attempted && generation == s_spk_generation) {
                    ++s_spk_underflows;
                    notify_playback_fault(PLAYBACK_FAULT_UNDERRUN, NULL);
                }
            }
        }
        if (s_maintenance) memset(block,0,sizeof(block));
        size_t written = 0;
        esp_err_t err = audio_backend_write(block, sizeof(block), &written, 20);
        if (err != ESP_OK || written != sizeof(block)) {
            ++s_spk_i2s_errors;
            notify_playback_fault(PLAYBACK_FAULT_I2S, NULL);
        }
        s_spk_i2s_bytes += written;
    }
}

void tud_mount_cb(void)
{
    custom_hid_disconnected();
    // A bus reset may reconfigure the device without an unmount callback.
    // New interfaces start at alt 0; discard the previous stream mask.
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE1, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, false);
    audio_diagnostics_event(ESP_LOG_INFO, TAG, "USB mounted", 0, false);
}

void tud_umount_cb(void)
{
    custom_hid_disconnected();
    audio_pipeline_set_recording(false);
    memset((void *)s_mic_active, 0, sizeof(s_mic_active));
    s_spk_active = false;
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE1, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, false);
    audio_diagnostics_event(ESP_LOG_INFO, TAG, "USB unmounted", 0, false);
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    audio_pipeline_set_recording(false);
    (void)remote_wakeup_en;
    memset((void *)s_mic_active, 0, sizeof(s_mic_active));
    s_spk_active = false;
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE1, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, false);
}

void tud_resume_cb(void)
{
    audio_diagnostics_event(ESP_LOG_INFO, TAG, "USB resumed", 0, false);
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf,
                                  audio_feedback_params_t *params)
{
    if (func_id != USB_AUDIO_PLAYBACK_FUNC) {
        params->method = AUDIO_FEEDBACK_METHOD_DISABLED;
        return;
    }
    (void)alt_itf;
    /* Disabled means no TinyUSB calculation; the application supplies the
     * feedback value from the queue actually consumed by playback_task. */
    params->method = AUDIO_FEEDBACK_METHOD_DISABLED;
    params->sample_freq = USB_AUDIO_SPK_SAMPLE_RATE_HZ;
}

bool tud_audio_set_itf_cb(uint8_t rhport,
                          const tusb_control_request_t *request)
{
    (void)rhport;
    uint8_t itf = tu_u16_low(tu_le16toh(request->wIndex));
    uint8_t alt = tu_u16_low(tu_le16toh(request->wValue));

    if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
        ++s_spk_generation;
        s_spk_active = alt != 0;
        audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, alt != 0);
        if (s_spk_active) {
            s_spk_feedback_level_q8 = SPK_FEEDBACK_TARGET_BYTES << 8;
            s_spk_queue_peak_bytes = 0;
            s_spk_feedback_value = (USB_AUDIO_SPK_SAMPLE_RATE_HZ /
                (tud_speed_get() == TUSB_SPEED_HIGH ? 8000U : 1000U)) << 16;
            (void)tud_audio_n_fb_set(USB_AUDIO_PLAYBACK_FUNC,
                                     s_spk_feedback_value);
            s_spk_usb_packets = 0;
            s_spk_usb_bytes = 0;
            s_spk_fifo_bytes = 0;
            s_spk_empty_callbacks = 0;
            s_spk_dropped_bytes = 0;
            s_spk_i2s_bytes = 0;
            s_spk_underflows = 0;
            s_spk_silence_bytes = 0;
            s_spk_i2s_errors = 0;
        }
        audio_diagnostics_event(ESP_LOG_INFO, TAG,
            s_spk_active ? "speaker stream started" : "speaker stream stopped", 0, false);
    } else if (itf == ITF_NUM_AUDIO_STREAMING_MIC ||
               itf == ITF_NUM_AUDIO2_STREAMING_MIC) {
        const uint8_t func_id = itf == ITF_NUM_AUDIO_STREAMING_MIC
                                    ? USB_AUDIO_RAW_FUNC
                                    : USB_AUDIO_PROCESSED_FUNC;
        audio_diagnostics_set_stream(func_id == USB_AUDIO_RAW_FUNC
            ? AUDIO_DIAG_CAPTURE1 : AUDIO_DIAG_CAPTURE2, alt != 0);
        s_mic_active[func_id] = false;
        s_mic_primed[func_id] = false;
        if (func_id == USB_AUDIO_PROCESSED_FUNC) {
            s_mic2_rate_initialized = false;
            s_mono_tx_started = false;
            s_mono_expected_bytes = 0;
            if (alt != 0U) {
                s_mono_source_min_samples = UINT32_MAX;
                s_mono_tx_packets = 0;
                s_mono_tx_zero_packets = 0;
                s_mono_tx_zero_scheduled = 0;
                s_mono_tx_zero_failed = 0;
                s_mono_tx_zero_unexpected = 0;
                s_mono_tx_unexpected_sizes = 0;
                s_mono_source_short_reads = 0;
                s_mono_source_read_packets = 0;
                s_mono_source_zero_packets = 0;
                s_mono_source_packets_15 = s_mono_source_packets_16 = s_mono_source_packets_17 = 0;
                s_mic_endpoint_null_packets[func_id] = 0;
            }
        }
        if (func_id == USB_AUDIO_PROCESSED_FUNC) audio_pipeline_set_recording(alt != 0);
#if USB_AUDIO_MIC_TEST_TONE_ENABLED
        s_mic_test_tone_phase[func_id] = 0;
#else
        if (func_id == USB_AUDIO_RAW_FUNC) {
            uint8_t discarded[USB_AUDIO_MIC_MS_BYTES];
            while (xStreamBufferReceive(s_mic_stream[func_id], discarded, sizeof(discarded), 0) != 0) {}
        }
#endif
        s_mic_active[func_id] = alt != 0;
        if (s_mic_active[func_id]) {
            fill_mic_fifo(func_id);
        }
        audio_diagnostics_event(ESP_LOG_INFO, TAG,
            s_mic_active[func_id] ? "microphone stream started, device" : "microphone stream stopped, device",
            (int)(func_id + 1U), true);
    }
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport,
                                   const tusb_control_request_t *request)
{
    return tud_audio_set_itf_cb(rhport, request);
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received,
                           uint8_t func_id, uint8_t ep_out,
                           uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)ep_out;
    (void)cur_alt_setting;
    ++s_spk_usb_packets;
    s_spk_usb_bytes += n_bytes_received;

    uint8_t packet[USB_AUDIO_SPK_EP_SIZE];
    BaseType_t task_woken = pdFALSE;
    size_t callback_fifo_bytes = 0;
    while (tud_audio_n_available(func_id) != 0U) {
        uint16_t count = tud_audio_n_read(func_id, packet, sizeof(packet));
        if (count == 0U) {
            break;
        }
        size_t queued = 0;
        const bool playback_active = s_spk_active;
        if (playback_active && xStreamBufferSpacesAvailable(s_spk_stream) >= count) {
            queued = xPortInIsrContext() ?
                xStreamBufferSendFromISR(s_spk_stream, packet, count, &task_woken) :
                xStreamBufferSend(s_spk_stream, packet, count, 0);
        }
        callback_fifo_bytes += count;
        s_spk_fifo_bytes += count;
        s_spk_dropped_bytes += count - queued;
        // Discards while the interface is closed are not queue overflows.
        if (playback_active && queued < count)
            notify_playback_fault(PLAYBACK_FAULT_OVERFLOW, &task_woken);
    }
    if (callback_fifo_bytes == 0U) {
        ++s_spk_empty_callbacks;
    }
    if (s_spk_active) {
        const uint32_t level = (uint32_t)xStreamBufferBytesAvailable(s_spk_stream);
        if (level > s_spk_queue_peak_bytes) s_spk_queue_peak_bytes = level;
        update_speaker_feedback();
    }
    if (task_woken) portYIELD_FROM_ISR();
    return true;
}

bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent,
                           uint8_t func_id, uint8_t ep_in,
                           uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)ep_in;
    (void)cur_alt_setting;
    if (func_id < USB_AUDIO_FUNCTION_COUNT && s_mic_active[func_id]) {
        if (func_id == USB_AUDIO_PROCESSED_FUNC) {
            // TinyUSB reports the transfer it just completed, then our
            // callback refills its FIFO for a later transfer. Ignore only
            // the expected zero-length packets before the first data packet.
            if (n_bytes_sent != 0U) s_mono_tx_started = true;
            if (s_mono_tx_started) {
                ++s_mono_tx_packets;
                if (n_bytes_sent == 0U) ++s_mono_tx_zero_packets;
                else if (n_bytes_sent != (MIC2_SAMPLES_PER_MS - 1U) * USB_AUDIO_MIC2_FRAME_BYTES &&
                         n_bytes_sent != MIC2_SAMPLES_PER_MS * USB_AUDIO_MIC2_FRAME_BYTES &&
                         n_bytes_sent != (MIC2_SAMPLES_PER_MS + 1U) * USB_AUDIO_MIC2_FRAME_BYTES)
                    ++s_mono_tx_unexpected_sizes;
            }
        }
        fill_mic_fifo(func_id);
    }
    return true;
}

/* Called from the locally patched TinyUSB audio driver. One transfer is
 * outstanding per endpoint, so the scheduled length identifies the result. */
void usb_audio_capture_xfer_result_isr(uint8_t func_id, uint8_t result,
                                        uint16_t bytes)
{
    if (func_id != USB_AUDIO_PROCESSED_FUNC ||
        !s_mic_active[func_id] || !s_mono_tx_started || bytes != 0U) return;
    if (result != XFER_RESULT_SUCCESS) ++s_mono_tx_zero_failed;
    else if (s_mono_expected_bytes == 0U) ++s_mono_tx_zero_scheduled;
    else ++s_mono_tx_zero_unexpected;
}

void usb_audio_capture_xfer_scheduled_isr(uint8_t func_id, uint16_t bytes)
{
    if (func_id == USB_AUDIO_PROCESSED_FUNC) s_mono_expected_bytes = bytes;
}

static bool clock_get_request(uint8_t rhport,
                              const audio_control_request_t *request,
                              uint32_t sample_rate_hz)
{
    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_4_t current = {
                .bCur = (int32_t)tu_htole32(sample_rate_hz),
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (const tusb_control_request_t *)request,
                &current, sizeof(current));
        }
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(1) range = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = {
                    .bMin = sample_rate_hz,
                    .bMax = sample_rate_hz,
                    .bRes = 0,
                },
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (const tusb_control_request_t *)request,
                &range, sizeof(range));
        }
    } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
               request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t valid = {.bCur = 1};
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (const tusb_control_request_t *)request,
            &valid, sizeof(valid));
    }
    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 const tusb_control_request_t *request)
{
    const audio_control_request_t *audio_req =
        (const audio_control_request_t *)request;
    if (audio_req->bEntityID == UAC2_ENTITY_SPK_CLOCK) {
        return clock_get_request(rhport, audio_req,
                                 USB_AUDIO_SPK_SAMPLE_RATE_HZ);
    }
    if (audio_req->bEntityID == UAC2_ENTITY_MIC_CLOCK) {
        return clock_get_request(rhport, audio_req,
                                 USB_AUDIO_MIC_SAMPLE_RATE_HZ);
    }
    if (audio_req->bEntityID == UAC2_ENTITY_MIC2_CLOCK) {
        return clock_get_request(rhport, audio_req,
                                 USB_AUDIO_MIC2_SAMPLE_RATE_HZ);
    }
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport,
                                 const tusb_control_request_t *request,
                                 uint8_t *buffer)
{
    (void)rhport;
    const audio_control_request_t *audio_req =
        (const audio_control_request_t *)request;
    if ((audio_req->bEntityID == UAC2_ENTITY_SPK_CLOCK ||
         audio_req->bEntityID == UAC2_ENTITY_MIC_CLOCK ||
         audio_req->bEntityID == UAC2_ENTITY_MIC2_CLOCK) &&
        audio_req->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ &&
        audio_req->bRequest == AUDIO_CS_REQ_CUR &&
        request->wLength == sizeof(audio_control_cur_4_t)) {
        uint32_t rate = tu_le32toh(
            (uint32_t)((audio_control_cur_4_t *)buffer)->bCur);
        uint32_t expected = audio_req->bEntityID == UAC2_ENTITY_SPK_CLOCK
                                ? USB_AUDIO_SPK_SAMPLE_RATE_HZ
                                : audio_req->bEntityID == UAC2_ENTITY_MIC2_CLOCK
                                    ? USB_AUDIO_MIC2_SAMPLE_RATE_HZ : USB_AUDIO_MIC_SAMPLE_RATE_HZ;
        return rate == expected;
    }
    return false;
}

esp_err_t usb_audio_init(void)
{
#if !USB_AUDIO_MIC_TEST_TONE_ENABLED
    /* Only raw capture uses a stream buffer; processed capture reads the pipeline. */
    s_mic_stream[USB_AUDIO_RAW_FUNC] = xStreamBufferCreate(
        USB_AUDIO_MIC_RING_BYTES, USB_AUDIO_MIC_FRAME_BYTES);
    ESP_RETURN_ON_FALSE(s_mic_stream[USB_AUDIO_RAW_FUNC] != NULL, ESP_ERR_NO_MEM,
                        TAG, "allocate raw capture stream buffer");
#endif
    s_spk_stream = xStreamBufferCreate(USB_AUDIO_SPK_RING_BYTES,
                                       USB_AUDIO_SPK_FRAME_BYTES);
    ESP_RETURN_ON_FALSE(s_spk_stream != NULL, ESP_ERR_NO_MEM,
                        TAG, "allocate playback stream buffer");
    BaseType_t ok = xTaskCreatePinnedToCore(playback_log_task, "playback_log", 3072,
        NULL, 2, &s_playback_log_task, 0);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create playback log task");
    usb_phy_config_t phy_config = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_UTMI,
        .otg_speed = USB_PHY_SPEED_HIGH,
    };
    ESP_RETURN_ON_ERROR(usb_new_phy(&phy_config, &s_phy), TAG,
                        "initialize USB HS UTMI PHY");
    ESP_RETURN_ON_FALSE(tusb_init(), ESP_FAIL, TAG,
                        "initialize TinyUSB device stack");

    ok = xTaskCreatePinnedToCore(tinyusb_task, "tinyusb", 4096,
        NULL, CONFIG_USB_AUDIO_USB_TASK_PRIORITY, NULL, 0);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "create TinyUSB task");
    ESP_RETURN_ON_ERROR(usb_hid_keyboard_init(), TAG,
                        "initialize HID keyboard");
#if !USB_AUDIO_MIC_TEST_TONE_ENABLED
    ok = xTaskCreatePinnedToCore(capture_task, "audio_capture", 4096,
        NULL, CONFIG_USB_AUDIO_CAPTURE_TASK_PRIORITY, NULL, tskNO_AFFINITY);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "create capture task");
#endif
    ok = xTaskCreatePinnedToCore(playback_task, "audio_playback", 4096,
        NULL, CONFIG_USB_AUDIO_PLAYBACK_TASK_PRIORITY, NULL, tskNO_AFFINITY);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "create playback task");

    ESP_LOGI(TAG,
             "USB composite ready: capture 1=8ch %u Hz, capture 2=stereo audio+VAD %u Hz; "
             "playback 2ch %u Hz/16-bit; HID keyboard",
             USB_AUDIO_MIC_SAMPLE_RATE_HZ, USB_AUDIO_MIC2_SAMPLE_RATE_HZ, USB_AUDIO_SPK_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "buffer levels ms: capture2 prime/average-target=%u (128-ms filter), raw=%u/%u/%u, playback prime/target=%u/%u",
             USB_AUDIO_MIC2_PRIME_MS,
             USB_AUDIO_MIC_PRIME_MS, USB_AUDIO_MIC_LOW_MS, USB_AUDIO_MIC_HIGH_MS,
             USB_AUDIO_SPK_PRIME_MS, USB_AUDIO_SPK_TARGET_MS);
#if USB_AUDIO_MIC_TEST_TONE_ENABLED
    ESP_LOGI(TAG,
             "capture source: 1-kHz LUT tone, CH1/3/5/7 positive, "
             "CH2/4/6/8 inverted; TDM RX bypassed");
#else
    ESP_LOGI(TAG, "capture source: I2S0 TDM RX");
#endif
    return ESP_OK;
}

unsigned usb_audio_stream_mask(void) {
    return (s_spk_active ? 1U : 0U) | (s_mic_active[USB_AUDIO_RAW_FUNC] ? 2U : 0U) | (s_mic_active[USB_AUDIO_PROCESSED_FUNC] ? 4U : 0U);
}
void usb_audio_set_maintenance(bool active) {
    portENTER_CRITICAL(&s_mic_fill_lock);
    s_maintenance=active;
    for (unsigned i=0;i<2;++i) {
        if (active) (void)tud_audio_n_clear_ep_in_ff(i);
        else { s_capture_resuming[i]=true; s_capture_recovering[i]=true; }
    }
    portEXIT_CRITICAL(&s_mic_fill_lock);
}
