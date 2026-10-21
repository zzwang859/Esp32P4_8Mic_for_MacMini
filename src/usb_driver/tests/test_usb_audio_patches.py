"""Run the generated TinyUSB receive functions against mocked DMA/FIFO state.

Build the firmware first. Pass a host C compiler with --cc.
Use --source-dir managed_components/espressif__tinyusb/src to check upstream
sources and demonstrate the zero-length receive regression before patching.
"""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def function(text, name):
    match = re.search(r"^static [^\n]*\b" + name + r"\([^;]*?\)\s*\{", text, re.M)
    if not match:
        raise ValueError(f"Function not found: {name}")
    start = text.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end]


MOCKS = r"""
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define CFG_TUD_DWC2_PTI_ENABLE 1
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 0
#define USE_LINEAR_BUFFER_RX 1
#define TUSB_DIR_OUT 0
#define TUSB_DIR_IN 1
#define XFER_RESULT_SUCCESS 0
#define XFER_RESULT_FAILED 1
#define DEPCTL_EPTYPE_ISOCHRONOUS 1
#define DIEPCTL_EPENA (1u << 31)
#define TU_VERIFY(expr, ...) do { if (!(expr)) return false; } while (0)

typedef struct { unsigned count; } tu_fifo_t;
typedef struct {
    tu_fifo_t ep_out_ff;
    uint8_t lin_buf_out[68];
    uint8_t ep_out, ep_out_alt;
    uint16_t ep_out_sz;
} audiod_function_t;
typedef union {
    uint32_t value;
    struct { unsigned xfer_size:19, packet_count:10, reserved:3; };
} dwc2_ep_tsize_t;
typedef union {
    uint32_t value;
    struct { unsigned reserved:18, type:2, unused:12; };
} dwc2_depctl_t;
typedef struct {
    bool setup_phase_done, xfer_complete, status_phase_rx;
    bool setup_packet_rx, iso_packet_drop;
} dwc2_doepint_t;
typedef struct { uint32_t diepctl, doepctl, tsiz; } dwc2_dep_t;
typedef struct { dwc2_dep_t epin[2], epout[2]; } dwc2_regs_t;
typedef struct { uint16_t total_len; uint8_t *buffer; } xfer_ctl_t;
static struct { unsigned ep0_pending[2]; } _dcd_data;
static struct { uint8_t setup_packet[8]; } _dcd_usbbuf;
static dwc2_regs_t regs;
static xfer_ctl_t xfers[2];
static audiod_function_t audio;
static unsigned armed, notified, writes, invalidated, payload;
static unsigned completions, completion_bytes, completion_result;
#define DWC2_REG(rhport) (&regs)
#define XFER_CTL_BASE(epnum, dir) (&xfers[epnum])
static bool dma_device_enabled(dwc2_regs_t *r) { (void)r; return true; }
static uint8_t audiod_get_audio_fct_idx(audiod_function_t *a) { (void)a; return 0; }
static unsigned tu_fifo_write_n(tu_fifo_t *f, const void *b, unsigned n) {
    (void)b; ++writes; f->count += n; return n;
}
static bool usbd_edpt_xfer(uint8_t r, uint8_t ep, void *b, uint16_t n) {
    (void)r; (void)ep; (void)b; (void)n; ++armed; return true;
}
static bool tud_audio_rx_done_isr(uint8_t r, uint16_t n, uint8_t f, uint8_t e, uint8_t a) {
    (void)r; (void)f; (void)e; (void)a; ++notified; payload += n; return true;
}
static void edpt_disable(uint8_t r, uint8_t e, bool s) { (void)r; (void)e; (void)s; }
static void dma_setup_prepare(uint8_t r) { (void)r; }
static void dcd_dcache_invalidate(void *p, unsigned n) { (void)p; (void)n; ++invalidated; }
static void dcd_event_setup_received(uint8_t r, uint8_t *p, bool i) { (void)r; (void)p; (void)i; }
static void edpt_schedule_packets(uint8_t r, uint8_t e, uint8_t d) { (void)r; (void)e; (void)d; }
"""

EVENT = r"""
static void dcd_event_xfer_complete(uint8_t r, uint8_t ep, unsigned n, unsigned result, bool isr) {
    (void)isr; ++completions; completion_bytes = n; completion_result = result;
    if (ep == 1) audiod_rx_xfer_isr(r, &audio, (uint16_t)n);
}
"""

CASES = r"""
#define CHECK(expr) do { if (!(expr)) return __LINE__; } while (0)
static void reset(void) {
    armed = notified = writes = invalidated = payload = completions = 0;
    completion_bytes = completion_result = 0;
    audio.ep_out_ff.count = 0;
    audio.ep_out = 1; audio.ep_out_alt = 1; audio.ep_out_sz = 68;
    xfers[1].total_len = 68; xfers[1].buffer = audio.lin_buf_out;
    dwc2_depctl_t ctl = {.value = 0};
    ctl.type = DEPCTL_EPTYPE_ISOCHRONOUS;
    regs.epout[1].doepctl = ctl.value;
    regs.epout[1].tsiz = 0;
}
int main(void) {
    // A zero-length OUT must rearm, then deliver the following real packet.
    reset();
    CHECK(audiod_rx_xfer_isr(1, &audio, 0));
    CHECK(armed == 1 && notified == 1 && writes == 0 && payload == 0);
    CHECK(audiod_rx_xfer_isr(1, &audio, 8));
    CHECK(armed == 2 && notified == 2 && writes == 1 && payload == 8);

    // Repeated empty packets must not break the receive chain.
    for (unsigned i = 0; i < 100; ++i) CHECK(audiod_rx_xfer_isr(1, &audio, 0));
    CHECK(armed == 102 && payload == 8);

    // Dropped PTI packets carry no payload, even with stale residual size.
    reset();
    dwc2_ep_tsize_t size = {.value = 0};
    size.xfer_size = 100; // exceeds the 68-byte request: must not subtract
    regs.epout[1].tsiz = size.value;
    dwc2_doepint_t dropped = {.xfer_complete = true, .iso_packet_drop = true};
    handle_epout_dma(1, 1, dropped);
    CHECK(completions == 1 && completion_bytes == 0 && completion_result == XFER_RESULT_FAILED);
    CHECK(invalidated == 0 && writes == 0 && payload == 0 && armed == 1);
    CHECK(xfers[1].total_len == 68);

    // Normal DMA reception still delivers its actual, short packet size.
    reset();
    size.xfer_size = 60; regs.epout[1].tsiz = size.value;
    dwc2_doepint_t good = {.xfer_complete = true};
    handle_epout_dma(1, 1, good);
    CHECK(completions == 1 && completion_bytes == 8 && completion_result == XFER_RESULT_SUCCESS);
    CHECK(invalidated == 1 && writes == 1 && payload == 8 && armed == 1);

    // Ordinary DMA ZLPs must also rearm OUT.
    reset();
    size.xfer_size = 68; regs.epout[1].tsiz = size.value;
    handle_epout_dma(1, 1, good);
    CHECK(completion_bytes == 0 && armed == 1 && writes == 0 && notified == 1);

    // Non-completion/status events do not produce audio or rearm the data EP.
    reset();
    dwc2_doepint_t idle = {0};
    handle_epout_dma(1, 1, idle);
    good.status_phase_rx = true;
    handle_epout_dma(1, 1, good);
    CHECK(completions == 0 && armed == 0 && writes == 0);
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build"))
    parser.add_argument("--source-dir", type=Path)
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()
    if args.source_dir:
        audio_path = args.source_dir / "class/audio/audio_device.c"
        dwc_path = args.source_dir / "portable/synopsys/dwc2/dcd_dwc2.c"
    else:
        audio_path = args.build_dir / "esp-idf/src/audio_device_capture2.c"
        dwc_path = args.build_dir / "esp-idf/src/dcd_dwc2_capture_pti.c"
    rx = function(audio_path.read_text(encoding="utf-8"), "audiod_rx_xfer_isr")
    dma = function(dwc_path.read_text(encoding="utf-8"), "handle_epout_dma")
    with tempfile.TemporaryDirectory(prefix="usb-audio-regression-") as temp:
        temp = Path(temp)
        source = temp / "regression.c"
        source.write_text(MOCKS + rx + EVENT + dma + CASES, encoding="utf-8")
        binary = temp / "regression.exe"
        subprocess.run([args.cc, "-std=c11", str(source), "-o", str(binary)], check=True)
        result = subprocess.run([str(binary)], check=False)
        if result.returncode:
            lines = source.read_text(encoding="utf-8").splitlines()
            line = result.returncode
            detail = lines[line - 1].strip() if 0 < line <= len(lines) else "abnormal exit"
            raise SystemExit(f"FAIL at generated harness line {line}: {detail}")
    print("PASS: ZLP rearm, repeated ZLP, PTI drop, short DMA receive, DMA ZLP, status filtering")


if __name__ == "__main__":
    main()
