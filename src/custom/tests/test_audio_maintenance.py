"""Run the actual USB refill branch: maintenance/recovery must emit full zero packets."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src/usb_driver/tests"))
from test_capture2_stereo import function

MOCKS=r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
#include "usb_audio_config.h"
#include "audio_pipeline.h"
using BaseType_t=int;
#define USB_AUDIO_FUNCTION_COUNT 2
#define USB_AUDIO_PRIMARY_FUNC 0
#define USB_AUDIO_SECONDARY_FUNC 1
#define MIC2_PRIME_SAMPLES ((USB_AUDIO_MIC2_SAMPLE_RATE_HZ/1000)*USB_AUDIO_MIC2_PRIME_MS)
#define MIC2_SAMPLES_PER_MS (USB_AUDIO_MIC2_SAMPLE_RATE_HZ/1000)
static bool s_maintenance,s_capture_resuming[2],s_capture_recovering[2],s_mic_primed[2],s_mic2_rate_initialized;
static unsigned s_mic_endpoint_write_shorts,s_mic_endpoint_null_packets[2],s_mic_endpoint_fifo_backpressure;
static size_t raw_bytes,processed_frames;
static unsigned reads,normal_calls,writes,last_id;
static int s_mic_stream[2]={1,2};
static std::vector<uint8_t> output;
struct tu_fifo_t {};
static tu_fifo_t fifo;
size_t audio_pipeline_available() { return processed_frames; }
size_t audio_pipeline_read(audio_pipeline_capture2_frame_t *,size_t count) {
    ++reads; size_t n=std::min(count,processed_frames); processed_frames-=n; return n;
}
static size_t xStreamBufferBytesAvailable(int) { return raw_bytes; }
static size_t xStreamBufferReceive(int,void *dst,size_t bytes,unsigned) {
    ++reads; size_t n=std::min(bytes,raw_bytes); std::memset(dst,0x11,n); raw_bytes-=n; return n;
}
static size_t xStreamBufferReceiveFromISR(int stream,void *dst,size_t bytes,BaseType_t *) {
    return xStreamBufferReceive(stream,dst,bytes,0);
}
static tu_fifo_t *tud_audio_n_get_ep_in_ff(uint8_t) { return &fifo; }
static size_t tu_fifo_remaining(tu_fifo_t *) { return 1024; }
static uint16_t tud_audio_n_write(uint8_t id,const void *src,uint16_t size) {
    last_id=id; ++writes; const uint8_t *p=static_cast<const uint8_t *>(src); output.assign(p,p+size); return size;
}
static void fill_capture2_fifo() { ++normal_calls; }
'''

CASES=r'''
int main() {
    BaseType_t woken=0;
    s_maintenance=true;
    for (unsigned id=0;id<2;++id) {
        unsigned before=reads;
        fill_mic_fifo_impl(id,false,&woken);
        assert(last_id==id && reads==before && normal_calls==0);
        assert(output.size()==(id ? USB_AUDIO_MIC2_MS_BYTES : USB_AUDIO_MIC_MS_BYTES));
        assert(std::all_of(output.begin(),output.end(),[](uint8_t b){return b==0;}));
    }
    // USB alone discards old output. It keeps sending complete zero packets
    // until the post-reset reserve is ready. The normal 48-ms threshold
    // must not end recovery while the AFE's burst cadence is still settling.
    s_maintenance=false;
    s_capture_resuming[1]=s_capture_recovering[1]=true; processed_frames=80;
    fill_mic_fifo_impl(1,false,&woken);
    assert(processed_frames==0 && output.size()==USB_AUDIO_MIC2_MS_BYTES && normal_calls==0);
    processed_frames=MIC2_PRIME_SAMPLES-1;
    fill_mic_fifo_impl(1,true,&woken);
    assert(normal_calls==0 && std::all_of(output.begin(),output.end(),[](uint8_t b){return b==0;}));
    processed_frames=MIC2_PRIME_SAMPLES;
    fill_mic_fifo_impl(1,true,&woken);
    assert(normal_calls==0 && s_capture_recovering[1]);
    processed_frames=USB_AUDIO_MIC2_RECOVERY_MS*MIC2_SAMPLES_PER_MS-1;
    fill_mic_fifo_impl(1,true,&woken);
    assert(normal_calls==0 && s_capture_recovering[1]);
    processed_frames=USB_AUDIO_MIC2_RECOVERY_MS*MIC2_SAMPLES_PER_MS;
    fill_mic_fifo_impl(1,true,&woken);
    assert(normal_calls==1 && !s_capture_recovering[1]);
    s_capture_resuming[0]=s_capture_recovering[0]=true; raw_bytes=100;
    fill_mic_fifo_impl(0,false,&woken);
    assert(raw_bytes==0 && output.size()==USB_AUDIO_MIC_MS_BYTES);
    raw_bytes=USB_AUDIO_MIC_PRIME_MS*(USB_AUDIO_MIC_SAMPLE_RATE_HZ/1000)*USB_AUDIO_MIC_FRAME_BYTES;
    fill_mic_fifo_impl(0,false,&woken);
    assert(!s_capture_recovering[0] && output[0]==0x11);
    assert(s_mic_endpoint_write_shorts==0 && s_mic_endpoint_null_packets[0]==0);
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument("--cxx",nargs="+",required=True); args=parser.parse_args()
    source=(ROOT/"src/usb_driver/usb_audio.c").read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="custom-audio-") as folder:
        folder=Path(folder); (folder/"esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n")
        (folder/"sdkconfig.h").write_text("#pragma once\n")
        code=folder/"test.cpp"; code.write_text(MOCKS+function(source,"fill_mic_fifo_impl")+CASES,encoding="utf-8")
        binary=folder/"test.exe"
        subprocess.run(args.cxx+["-std=c++17","-O2","-UNDEBUG","-Wall","-Wextra","-Werror",
            "-I",str(folder),"-I",str(ROOT/"src/usb_driver"),"-I",str(ROOT/"src/audio_processing"),
            str(code),"-o",str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    print("PASS full-size zero PCM on both captures, stale FIFO discard, zero-filled recovery, normal resume")

if __name__=="__main__": main()
