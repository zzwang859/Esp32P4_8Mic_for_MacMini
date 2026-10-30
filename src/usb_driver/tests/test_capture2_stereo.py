"""Check Capture2 packets against mocked FIFOs and FS/HS descriptors in the ELF.

Build firmware first, then run with --cxx <host C++ compiler>.
For Zig, use --cxx <zig executable> c++.
"""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[3]


def function(source, name):
    match = re.search(r"^[^\n]*\b" + name + r"\([^;]*?\)\s*\{", source, re.M)
    if not match:
        raise ValueError(f"Function not found: {name}")
    start = source.index("{", match.start())
    depth, end = 1, start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


MOCKS = r"""
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <vector>
#include <cstdio>
#define _Static_assert static_assert
#include "usb_audio_config.h"
#include "audio_pipeline.h"
#define TUSB_SPEED_HIGH 1
static_assert(USB_AUDIO_MIC2_FRAME_BYTES == 4);
static_assert(USB_AUDIO_MIC2_EP_SIZE == 68);
constexpr size_t kPrime = USB_AUDIO_MIC2_PRIME_MS * (USB_AUDIO_MIC2_SAMPLE_RATE_HZ / 1000);
struct tu_fifo_t {};
static tu_fifo_t fifo;
static bool fifo_present, s_mic_primed[2], s_mic_active[2], s_mono_tx_started;
static unsigned s_mic_endpoint_null_packets[2], s_mic_endpoint_fifo_backpressure;
static unsigned s_mono_source_short_reads, s_mono_source_read_packets;
static unsigned s_mono_source_zero_packets, s_mono_source_min_samples;
static uint32_t s_mono_source_frames, s_mono_source_zero_pairs;
static uint32_t s_mono_source_packets_15, s_mono_source_packets_16, s_mono_source_packets_17;
static bool s_mic2_rate_initialized;
static int32_t s_mic2_level_q8, s_mic2_rate_phase_q16;
static unsigned s_mic_endpoint_write_shorts, s_mono_tx_packets;
static unsigned s_mono_tx_zero_packets, s_mono_tx_unexpected_sizes;
static size_t available, read_limit, capacity, requested, written, write_limit;
static unsigned reads, writes;
static bool high_speed;
static bool s_spk_active;
static size_t speaker_bytes;
static constexpr int s_spk_stream = 0;
static uint32_t s_spk_feedback_level_q8, s_spk_feedback_value;
static int tud_speed_get() { return high_speed ? TUSB_SPEED_HIGH : 0; }
static size_t xStreamBufferBytesAvailable(int) { return speaker_bytes; }
static bool tud_audio_n_fb_set(uint8_t id, uint32_t) { assert(id == USB_AUDIO_PLAYBACK_FUNC); return true; }
static audio_pipeline_capture2_frame_t source[17];
static int16_t output[34];
size_t audio_pipeline_available() { return available; }
size_t audio_pipeline_read(audio_pipeline_capture2_frame_t *packet, size_t count) {
    ++reads; requested = count;
    count = std::min(count, read_limit);
    std::memcpy(packet, source, count * sizeof(*packet));
    return count;
}
static tu_fifo_t *tud_audio_n_get_ep_in_ff(uint8_t id) {
    assert(id == USB_AUDIO_PROCESSED_FUNC); return fifo_present ? &fifo : nullptr;
}
static size_t tu_fifo_remaining(tu_fifo_t *) { return capacity; }
static size_t tud_audio_n_write(uint8_t id, const void *packet, uint16_t bytes) {
    assert(id == USB_AUDIO_PROCESSED_FUNC && bytes <= sizeof(output));
    ++writes; written = bytes; std::memcpy(output, packet, bytes);
    return std::min(size_t(bytes), write_limit);
}
static void fill_mic_fifo(uint8_t) {}
static void reset(size_t buffered = kPrime) {
    available = buffered; reads = writes = 0;
    capacity = write_limit = 68; read_limit = 17; fifo_present = true;
    s_mic_primed[USB_AUDIO_PROCESSED_FUNC] = false; s_mic_active[USB_AUDIO_PROCESSED_FUNC] = true;
    requested = written = 0;
    s_mono_source_short_reads = s_mono_source_read_packets = 0;
    s_mono_source_zero_packets = s_mic_endpoint_fifo_backpressure = 0;
    s_mic_endpoint_null_packets[USB_AUDIO_PROCESSED_FUNC] = s_mic_endpoint_write_shorts = 0;
    s_mono_source_min_samples = UINT32_MAX;
    s_mono_source_frames = s_mono_source_zero_pairs = 0;
    s_mono_source_packets_15 = s_mono_source_packets_16 = s_mono_source_packets_17 = 0;
    s_mic2_rate_initialized = false;
    for (int i = 0; i < 17; ++i)
        source[i] = {int16_t(i * 4095 - 32768), int16_t(i < 7 ? INT16_MIN : INT16_MAX)};
    std::fill_n(output, 34, int16_t(12345));
}
"""

CASES = r"""
int main() {
    // Protocol mask bits describe stream roles, independent of enumeration order.
    for (unsigned mask = 0; mask < 8; ++mask) {
        s_spk_active = (mask & 1U) != 0;
        s_mic_active[USB_AUDIO_RAW_FUNC] = (mask & 2U) != 0;
        s_mic_active[USB_AUDIO_PROCESSED_FUNC] = (mask & 4U) != 0;
        assert(usb_audio_stream_mask() == mask);
    }
    // Nominal, slow and fast USB clock compensation, preserving signed audio.
    const size_t levels[] = {kPrime, (kPrime-257), (kPrime+321)};
    const size_t frames[] = {16, 15, 17};
    for (int payload = 0; payload < 3; ++payload) {
      for (size_t n = 0; n < 3; ++n) {
        reset(levels[n]); s_mic_primed[USB_AUDIO_PROCESSED_FUNC] = true;
        if (n != 0) {
            s_mic2_rate_initialized = true;
            s_mic2_level_q8 = kPrime * 256;
            s_mic2_rate_phase_q16 = n == 1 ? -65535 : 65535;
        }
        for (int i = 0; i < 17; ++i) {
            if (payload == 1) source[i].right = source[i].audio;
            if (payload == 2) source[i].right = int16_t(i * 2000 - 18000);
        }
        fill_capture2_fifo();
        assert(reads == 1 && writes == 1 && requested == frames[n]);
        assert(written == frames[n] * 4);
        for (size_t i = 0; i < frames[n]; ++i) {
            assert(output[2*i] == source[i].audio);
            assert(output[2*i+1] == source[i].right);
        }
        if (frames[n] < 17) assert(output[2*frames[n]] == 12345);
      }
    }
    reset((kPrime-1)); fill_capture2_fifo(); assert(reads == 0 && writes == 0);
    reset(); fill_capture2_fifo(); assert(s_mic_primed[USB_AUDIO_PROCESSED_FUNC] && written == 64);
    reset(14); s_mic_primed[USB_AUDIO_PROCESSED_FUNC] = true; fill_capture2_fifo();
    assert(reads == 0 && s_mic_endpoint_null_packets[USB_AUDIO_PROCESSED_FUNC] == 1);
    assert(!s_mic_primed[USB_AUDIO_PROCESSED_FUNC]);
    available = (kPrime-1); fill_capture2_fifo(); assert(reads == 0);
    available = kPrime; fill_capture2_fifo(); assert(reads == 1 && s_mic_primed[USB_AUDIO_PROCESSED_FUNC]);
    // Stereo capacity must be checked before consuming left-channel samples.
    reset(); capacity = 63; fill_capture2_fifo();
    assert(reads == 0 && writes == 0 && s_mic_endpoint_fifo_backpressure == 1);
    assert(!s_mic2_rate_initialized);
    reset(1000); capacity = 64; s_mic2_rate_initialized = true;
    s_mic2_level_q8 = kPrime * 256; s_mic2_rate_phase_q16 = 65535;
    fill_capture2_fifo();
    assert(reads == 0 && s_mic2_rate_phase_q16 == 65535 && s_mic2_level_q8 == kPrime * 256);
    capacity = 68; fill_capture2_fifo();
    assert(requested == 17 && s_mono_source_packets_17 == 1);
    reset(); fifo_present = false; fill_capture2_fifo(); assert(reads == 0);
    reset(); read_limit = 7; fill_capture2_fifo();
    assert(written == 28 && s_mono_source_short_reads == 1);
    for (int i = 0; i < 7; ++i) assert(output[2*i] == source[i].audio && output[2*i+1] == source[i].right);
    // Full-scale VAD markers must not mask silence on the audio channel.
    reset(); for (auto &frame : source) frame.audio = 0; fill_capture2_fifo();
    assert(s_mono_source_zero_packets == 1 && output[1] == INT16_MIN);
    assert(s_mono_source_frames == 16 && s_mono_source_zero_pairs == 0);
    assert(s_mono_source_packets_16 == 1);
    // One zero stereo frame must be visible even inside a non-silent packet.
    reset(); source[8] = {0, 0}; fill_capture2_fifo();
    assert(s_mono_source_zero_packets == 0 && s_mono_source_zero_pairs == 1);
    assert(s_mono_source_frames == 16);
    reset(1000); s_mic2_rate_initialized = true; s_mic2_level_q8 = kPrime * 256;
    s_mic2_rate_phase_q16 = 65535; source[16] = {0, 0}; fill_capture2_fifo();
    assert(s_mono_source_packets_17 == 1 && s_mono_source_zero_pairs == 1);
    reset((kPrime-257)); s_mic_primed[USB_AUDIO_PROCESSED_FUNC] = true; s_mic2_rate_initialized = true;
    s_mic2_level_q8 = kPrime * 256; s_mic2_rate_phase_q16 = -65535;
    source[14] = {0, 0}; fill_capture2_fifo();
    assert(s_mono_source_packets_15 == 1 && s_mono_source_zero_pairs == 1);
    reset(); read_limit = 0; fill_capture2_fifo();
    assert(written == 0 && s_mono_source_read_packets == 0);
    reset(); write_limit = 60; fill_capture2_fifo(); assert(s_mic_endpoint_write_shorts == 1);
    // Drive the actual packet producer for two simulated minutes. AFE emits
    // 512-sample / 32-ms chunks (queried on hardware); vary jitter by 0..4 ms and the
    // source clock by +/-1000 ppm. No real sleeps or host audio APIs involved.
    for (unsigned jitter : {4U, 16U}) for (bool ns : {false, true}) for (int ppm : {-1000, 0, 1000}) {
        reset(0);
        uint32_t rng = 17;
        std::vector<unsigned> releases(120064);
        unsigned pending = 0;
        for (unsigned chunk = 1; ; ++chunk) {
            rng = rng * 1664525U + 1013904223U;
            const unsigned at = unsigned(uint64_t(chunk) * 32000000 / (1000000 + ppm)) + rng % (jitter + 1);
            if (at >= 120000) break;
            if (!ns) releases[at] += 512;
            else {
                // Reblock 512-point AFE input into 160-point NS output, with
                // two milliseconds of processing/preemption per emitted frame.
                pending += 512;
                for (unsigned offset = 2; pending >= 160; offset += 2) {
                    releases[at + offset] += 160;
                    pending -= 160;
                }
            }
        }
        uint64_t occupancy = 0;
        size_t minimum = SIZE_MAX, maximum = 0;
        unsigned ticks = 0;
        for (unsigned ms = 0; ms < 120000; ++ms) {
            available += releases[ms];
            const unsigned previous_reads = reads;
            fill_capture2_fifo();
            if (reads != previous_reads) {
                minimum = std::min(minimum, available);
                maximum = std::max(maximum, available);
                occupancy += available; ++ticks;
                available -= requested;
            }
        }
        assert(s_mic_endpoint_null_packets[USB_AUDIO_PROCESSED_FUNC] == 0);
        assert(s_mono_source_short_reads == 0 && s_mic_endpoint_write_shorts == 0);
        assert(minimum >= 15 && maximum <= 2048); // Actual 128-ms stereo FIFO capacity.
        // Bursty NS delivery must not cause alternating 15/17-frame traffic
        // at audio-block frequency. Still track a real +/-1000 ppm clock.
        assert(s_mono_source_packets_15 + s_mono_source_packets_17 < ticks / 20);
        std::printf("PASS %s burst/jitter(%u ms) simulation %+d ppm: min/mean/max %.2f/%.2f/%.2f ms, corrected packets %u/%u\n",
                    ns ? "NS160" : "AFE512", jitter, ppm, double(minimum)/16, double(occupancy)/ticks/16,
                    double(maximum)/16, s_mono_source_packets_15 + s_mono_source_packets_17, ticks);
    }
    // Exercise the actual feedback controller with 2-ms I2S consumption
    // bursts at both USB speeds and independent +/-1000-ppm sink clocks.
    for (bool hs : {false, true}) for (int ppm : {-1000, 0, 1000}) {
        high_speed = hs;
        speaker_bytes = USB_AUDIO_SPK_PRIME_MS * USB_AUDIO_SPK_MS_BYTES;
        s_spk_feedback_level_q8 = SPK_FEEDBACK_TARGET_BYTES << 8;
        const unsigned frames_per_tick = hs ? 2 : 16;
        uint32_t host_phase = 0;
        uint64_t sink_phase = 0;
        uint32_t host_rate = frames_per_tick << 16;
        size_t minimum = SIZE_MAX, maximum = 0;
        for (unsigned tick = 1; tick <= (hs ? 160000U : 20000U); ++tick) {
            host_phase += host_rate;
            speaker_bytes += (host_phase >> 16) * USB_AUDIO_SPK_FRAME_BYTES;
            host_phase &= 0xffff;
            sink_phase += uint64_t(frames_per_tick) * (1000000 + ppm);
            if (sink_phase >= 32000000) {
                assert(speaker_bytes >= 32 * USB_AUDIO_SPK_FRAME_BYTES);
                speaker_bytes -= 32 * USB_AUDIO_SPK_FRAME_BYTES;
                sink_phase -= 32000000;
            }
            assert(speaker_bytes <= USB_AUDIO_SPK_RING_BYTES);
            minimum = std::min(minimum, speaker_bytes);
            maximum = std::max(maximum, speaker_bytes);
            update_speaker_feedback();
            if (!hs || tick % 8 == 0) host_rate = s_spk_feedback_value;
        }
        std::printf("PASS playback feedback %s %+d ppm: queue min/max %.2f/%.2f ms\n",
                    hs ? "HS" : "FS", ppm, double(minimum)/64, double(maximum)/64);
    }
    // Packet diagnostics accept 60/64/68 bytes and reject old mono sizes.
    s_mono_tx_started = false;
    for (uint16_t bytes : {uint16_t(0), uint16_t(60), uint16_t(64), uint16_t(68)})
        assert(tud_audio_tx_done_isr(1, bytes, USB_AUDIO_PROCESSED_FUNC, 0x83, 1));
    assert(s_mono_tx_packets == 3 && s_mono_tx_unexpected_sizes == 0);
    for (uint16_t bytes : {uint16_t(30), uint16_t(32), uint16_t(34)})
        tud_audio_tx_done_isr(1, bytes, USB_AUDIO_PROCESSED_FUNC, 0x83, 1);
    assert(s_mono_tx_unexpected_sizes == 3);
    return 0;
}
"""


def check_descriptors(path):
    from elftools.elf.elffile import ELFFile

    with path.open("rb") as stream:
        elf = ELFFile(stream)
        symbols = elf.get_section_by_name(".symtab")

        def data(name):
            symbol, = symbols.get_symbol_by_name(name)
            section = elf.get_section(symbol["st_shndx"])
            offset = symbol["st_value"] - section["sh_addr"]
            return section.data()[offset:offset + symbol["st_size"]]

        device = data("s_device_descriptor")
        assert int.from_bytes(device[10:12], "little") == 0x8003
        assert int.from_bytes(device[12:14], "little") == 0x0104
        report = data("s_custom_report_descriptor")
        assert b"\x06\x00\xff" in report and b"\x95\x40" in report
        assert b"\x81\x02" in report and b"\x91\x02" in report
        for name, interval in (("s_fs_configuration", 1), ("s_hs_configuration", 4)):
            config = data(name)
            assert int.from_bytes(config[2:4], "little") == len(config)
            assert config[4] == 7
            interface, found, pos = None, set(), 0
            interfaces, iads, endpoints, captures = [], [], {}, []
            while pos < len(config):
                size = config[pos]
                assert size >= 2 and pos + size <= len(config)
                desc = config[pos:pos + size]
                if desc[1] == 4:
                    interface = desc[2]
                    interfaces.append((interface, desc[3]))
                    if interface == 6:
                        assert desc[4:8] == bytes((2, 3, 0, 0))
                elif desc[1] == 0x24 and interface == 0 and desc[2] == 2:
                    assert desc[3] == 0x21 and desc[8] == 2
                    assert int.from_bytes(desc[9:13], "little") == 3
                    found.add("terminal")
                elif desc[1] == 0x24 and interface == 1:
                    if desc[2] == 1:
                        assert desc[3] == 0x23 and desc[10] == 2
                        assert int.from_bytes(desc[11:15], "little") == 3
                        found.add("stream")
                    elif desc[2] == 2:
                        assert desc[4:6] == bytes((2, 16))
                        found.add("format")
                elif desc[1] == 5 and interface == 1:
                    assert desc[2] == 0x83 and desc[6] == interval
                    assert int.from_bytes(desc[4:6], "little") == 68
                    found.add("endpoint")
                elif desc[1] == 5 and interface == 5:
                    assert desc[2] == 0x84 and int.from_bytes(desc[4:6], "little") == 8
                    found.add("keyboard")
                elif desc[1] == 5 and interface == 6:
                    assert desc[2] in (0x05, 0x85) and desc[3] & 3 == 3
                    assert int.from_bytes(desc[4:6], "little") == 64 and desc[6] == interval
                    found.add(f"custom-{desc[2]:02x}")
                if desc[1] == 11:
                    iads.append((desc[2], desc[3]))
                elif desc[1] == 5:
                    endpoints.setdefault(interface, []).append((
                        desc[2], desc[3] & 3, int.from_bytes(desc[4:6], "little"), desc[6]))
                elif desc[1] == 0x24 and desc[2] == 1 and interface in (1, 4):
                    captures.append((interface, desc[10]))
                pos += size
            assert found == {"terminal", "stream", "format", "endpoint", "keyboard", "custom-05", "custom-85"}, (name, found)
            assert iads == [(0, 2), (2, 3)], (name, iads)
            assert interfaces == [(0, 0), (1, 0), (1, 1), (2, 0),
                                  (3, 0), (3, 1), (4, 0), (4, 1), (5, 0), (6, 0)]
            assert captures == [(1, 2), (4, 8)], (name, captures)
            assert endpoints[1] == [(0x83, 1, 68, interval)]
            assert endpoints[3] == [(0x01, 1, 68, 1), (0x81, 1, 4, interval)]
            assert endpoints[4] == [(0x82, 1, 400, interval)]
            assert set(endpoints) == {1, 3, 4, 5, 6}
            print(f"PASS {name}: stereo capture first (interface 1), raw 8ch at interface 4; playback/feedback/HID endpoints")
        print("PASS device PID: 0x8003")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/esp32p4_usb_audio_hs.elf")
    parser.add_argument("--cxx", nargs="+", default=["c++"])
    parser.add_argument("--objdump", help="RISC-V objdump to verify the ISR pipeline reader has no FPU instructions")
    args = parser.parse_args()
    usb = (ROOT / "src/usb_driver/usb_audio.c").read_text(encoding="utf-8")
    limits = "\n".join(re.findall(r"^#define (?:MIC2_|SPK_FEEDBACK_TARGET_BYTES).*$", usb, re.M)) + "\n"
    code = (MOCKS + limits + function(usb, "capture2_packet_frames") + function(usb, "fill_capture2_fifo") + function(usb, "tud_audio_tx_done_isr")
            + function(usb, "update_speaker_feedback") + function(usb, "usb_audio_stream_mask") + CASES)
    with tempfile.TemporaryDirectory(prefix="capture2-stereo-") as temp:
        source = Path(temp) / "capture2.cpp"
        binary = Path(temp) / "capture2.exe"
        (Path(temp) / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
        source.write_text(code, encoding="utf-8")
        subprocess.run(args.cxx + ["-std=c++17", "-Wall", "-Wextra", "-Werror",
                                  "-I", str(ROOT / "src/usb_driver"),
                                  "-I", str(ROOT / "src/audio_processing"), "-I", temp, str(source),
                                  "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    print("PASS packets: copy/VAD/DOA right payloads, signed audio, in-packet transitions, 15/16/17 frames, priming, backpressure, short reads/writes, silence")
    check_descriptors(args.elf)
    if args.objdump:
        disassembly = subprocess.check_output(
            [args.objdump, "-d", "--disassemble=audio_pipeline_read", str(args.elf)],
            text=True)
        instructions = re.findall(r"^\s*[0-9a-f]+:\s+[0-9a-f]+\s+(\S+)", disassembly, re.M)
        assert instructions, "Capture2 reader disassembly missing"
        assert all(not op.startswith("f") or op.startswith("fence") for op in instructions), disassembly
        assert "lround" not in disassembly, disassembly
        print("PASS ESP32-P4 ISR reader: paired L/R frames, no FPU instructions")


if __name__ == "__main__":
    main()
