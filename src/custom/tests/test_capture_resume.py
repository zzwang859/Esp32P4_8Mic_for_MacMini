"""A long maintenance pause must not burst synthetic capture into AFE/KWS."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/usb_driver/tests"))
from test_capture2_stereo import function

MOCKS = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include "usb_audio_config.h"
using TickType_t=uint32_t;
using esp_err_t=int;
constexpr unsigned CUSTOM_WORK_CAPTURE=64;
constexpr int ESP_OK=0;
static TickType_t now=100;
static unsigned checkpoints, frames, s_i2s_read_anomalies, s_raw_fifo_overflows;
static bool s_mic_active[2];
static int s_mic_stream[2];
struct Done {};
TickType_t xTaskGetTickCount() { return now; }
void custom_worker_register(unsigned bit) { assert(bit==64); }
bool custom_worker_checkpoint(unsigned bit) {
    assert(bit==64);
    if (++checkpoints==2) { now+=10000; return true; }
    return false;
}
unsigned pdMS_TO_TICKS(unsigned ms) { return ms; }
void xTaskDelayUntil(TickType_t *last,unsigned period) {
    *last+=period;
    assert(*last>now); // A stale pre-pause deadline would generate a catch-up burst.
    now=*last;
}
bool audio_pipeline_capture_available() { return false; }
int audio_backend_read(void *,size_t,size_t *,unsigned) { assert(false); return -1; }
size_t xStreamBufferSpacesAvailable(int) { return 4096; }
size_t xStreamBufferSend(int,const void *,size_t size,unsigned) { return size; }
void audio_pipeline_capture(const int16_t *pcm,size_t count) {
    assert(count==24);
    assert(std::all_of(pcm,pcm+count*8,[](int16_t value){return value==0;}));
    if (++frames==4) throw Done{};
}
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx",nargs="+",required=True)
    args=parser.parse_args()
    source=(ROOT/"src/usb_driver/usb_audio.c").read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="capture-resume-") as folder:
        folder=Path(folder)
        (folder/"sdkconfig.h").write_text("#pragma once\n")
        test=folder/"test.cpp"
        test.write_text(MOCKS+function(source,"capture_task")+r'''
int main() {
    try { capture_task(nullptr); } catch (Done &) {}
    assert(frames==4 && now==10104);
    assert(s_i2s_read_anomalies==0 && s_raw_fifo_overflows==0);
}
''',encoding="utf-8")
        binary=folder/"test.exe"
        subprocess.run(args.cxx+["-std=c++17","-Wall","-Wextra","-Werror","-O2","-UNDEBUG",
            "-I",str(folder),"-I",str(ROOT/"src/usb_driver"),str(test),"-o",str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    print("PASS capture clock rebased after a 10-second maintenance pause")


if __name__=="__main__": main()
