"""COM6 / Windows audio integration test (numpy, hidapi, pyserial, sounddevice).

Uses a 1-kHz diagnostic signal on Capture2, while USB playback drives KWS.
Artifacts (UART, WAV, events, JSON) are written to --output.
"""
from __future__ import annotations

import argparse
import ctypes as c
from ctypes import wintypes as w
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time
import wave

import hid
import numpy as np
import serial
import sounddevice as sd

from kws_control import Client, decode, UPDATE_BEGIN, UPDATE_ABORT, ACTIVATE

ROOT = Path(__file__).resolve().parents[3]
mm = c.WinDLL("winmm")


class Caps(c.Structure):
    _fields_ = [("mid",w.WORD),("pid",w.WORD),("version",w.DWORD),("name",w.WCHAR*32),
                ("formats",w.DWORD),("channels",w.WORD),("reserved",w.WORD)]


class OutCaps(c.Structure):
    _fields_ = Caps._fields_ + [("support",w.DWORD)]


class Format(c.Structure):
    _pack_ = 1
    _fields_ = [("tag",w.WORD),("channels",w.WORD),("rate",w.DWORD),("bytes_per_sec",w.DWORD),
                ("align",w.WORD),("bits",w.WORD),("extra",w.WORD)]


class Header(c.Structure):
    _fields_ = [("data",c.c_void_p),("length",w.DWORD),("recorded",w.DWORD),("user",c.c_size_t),
                ("flags",w.DWORD),("loops",w.DWORD),("next",c.c_void_p),("reserved",c.c_size_t)]


for direction in ("In","Out"):
    cls = Caps if direction=="In" else OutCaps
    getattr(mm,f"wave{direction}GetDevCapsW").argtypes=[c.c_size_t,c.POINTER(cls),w.UINT]
    getattr(mm,f"wave{direction}Open").argtypes=[c.POINTER(c.c_void_p),w.UINT,c.POINTER(Format),c.c_size_t,c.c_size_t,w.DWORD]
    for op in ("PrepareHeader","UnprepareHeader"):
        getattr(mm,f"wave{direction}{op}").argtypes=[c.c_void_p,c.POINTER(Header),w.UINT]
    for op in ("Reset","Close"):
        getattr(mm,f"wave{direction}{op}").argtypes=[c.c_void_p]
mm.waveInAddBuffer.argtypes=mm.waveOutWrite.argtypes=[c.c_void_p,c.POINTER(Header),w.UINT]
mm.waveInStart.argtypes=mm.waveInStop.argtypes=[c.c_void_p]


def check(result):
    if result:
        raise RuntimeError(f"WinMM error {result}")


def devices():
    rows=[]
    for direction,cls in (("In",Caps),("Out",OutCaps)):
        for index in range(getattr(mm,f"wave{direction}GetNumDevs")()):
            caps=cls(); check(getattr(mm,f"wave{direction}GetDevCapsW")(index,c.byref(caps),c.sizeof(caps)))
            rows.append(dict(direction=direction,index=index,name=caps.name,channels=caps.channels))
    return rows


def choose(rows,direction):
    found=[d for d in rows if d["direction"]==direction and "usb audio" in d["name"].lower()
           and (direction=="Out" or "capture 2" in d["name"].lower())]
    if len(found)!=1:
        raise RuntimeError(f"Audio endpoint selection ambiguous: {found}; inventory={rows}")
    return found[0]


class Audio:
    def __init__(self,rows,payload,capture_raw=False):
        self.out_handle=c.c_void_p()
        self.out_buffer=c.create_string_buffer(payload)
        self.out_header=Header(c.cast(self.out_buffer,c.c_void_p),len(payload),0,0,0,0,None,0)
        self.fmt=Format(1,2,16000,64000,4,16,0)
        self.input=choose(rows,"In"); self.output=choose(rows,"Out")
        candidates=[i for i,d in enumerate(sd.query_devices())
                    if sd.query_hostapis(d["hostapi"])["name"]=="Windows WASAPI"
                    and d["max_input_channels"] and "usb audio capture 2" in d["name"].lower()]
        if len(candidates)!=1: raise RuntimeError(f"Exclusive Capture2 selection ambiguous: {candidates}")
        self.capture_device=candidates[0]; self.capture=None; self.blocks=[]; self.capture_faults=[]
        self.raw=None; self.raw_blocks=[]; self.raw_faults=[]; self.raw_device=None
        if capture_raw:
            candidates=[i for i,d in enumerate(sd.query_devices())
                        if sd.query_hostapis(d["hostapi"])["name"]=="Windows WDM-KS"
                        and d["max_input_channels"]>=8 and "usb audio" in d["name"].lower()
                        and "capture 2" not in d["name"].lower()]
            if len(candidates)!=1: raise RuntimeError(f"Raw 8-channel selection ambiguous: {candidates}")
            self.raw_device=candidates[0]

    def start(self):
        def record(data,frames,timing,status):
            if status: self.capture_faults.append(str(status))
            self.blocks.append(bytes(data))
        # Shared WinMM capture can apply microphone noise suppression and
        # erase a steady sine. Exclusive WASAPI preserves the device PCM.
        self.capture=sd.RawInputStream(device=self.capture_device,samplerate=16000,
            channels=2,dtype="int16",blocksize=160,callback=record,
            extra_settings=sd.WasapiSettings(exclusive=True))
        if self.raw_device is not None:
            def record_raw(data,frames,timing,status):
                if status: self.raw_faults.append(str(status))
                self.raw_blocks.append(bytes(data))
            self.raw=sd.RawInputStream(device=self.raw_device,samplerate=24000,
                channels=8,dtype="int16",blocksize=240,callback=record_raw)
            self.raw.start()
        check(mm.waveOutOpen(c.byref(self.out_handle),self.output["index"],c.byref(self.fmt),0,0,0))
        check(mm.waveOutPrepareHeader(self.out_handle,c.byref(self.out_header),c.sizeof(Header)))
        self.capture.start(); self.started=time.monotonic()
        check(mm.waveOutWrite(self.out_handle,c.byref(self.out_header),c.sizeof(Header)))

    def close(self,path):
        if self.raw:
            self.raw.stop(); self.raw.close(); self.raw=None
            with wave.open(str(path.with_name("capture1.wav")),"wb") as output:
                output.setparams((8,2,24000,0,"NONE","not compressed"))
                output.writeframes(b"".join(self.raw_blocks))
        if self.capture:
            self.capture.stop(); self.capture.close(); self.capture=None
            pcm=b"".join(self.blocks)
            with wave.open(str(path),"wb") as output:
                output.setparams((2,2,16000,0,"NONE","not compressed")); output.writeframes(pcm)
        if self.out_handle.value:
            mm.waveOutReset(self.out_handle)
            mm.waveOutUnprepareHeader(self.out_handle,c.byref(self.out_header),c.sizeof(Header))
            mm.waveOutClose(self.out_handle); self.out_handle.value=None

    def stop_playback(self):
        if self.out_handle.value: check(mm.waveOutReset(self.out_handle))


class F24Probe:
    """Observe and consume only the test F24 key; leave every other key alone."""
    def __init__(self):
        self.stop=threading.Event(); self.ready=threading.Event(); self.events=[]; self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start(); self.ready.wait(3)
        if self.error or not self.ready.is_set(): raise RuntimeError(f"F24 hook failed: {self.error}")

    def run(self):
        user=c.WinDLL("user32",use_last_error=True)
        callback_type=c.WINFUNCTYPE(c.c_ssize_t,c.c_int,w.WPARAM,w.LPARAM)
        user.SetWindowsHookExW.argtypes=[c.c_int,callback_type,c.c_void_p,w.DWORD]
        user.SetWindowsHookExW.restype=c.c_void_p
        user.CallNextHookEx.argtypes=[c.c_void_p,c.c_int,w.WPARAM,w.LPARAM]
        user.CallNextHookEx.restype=c.c_ssize_t
        user.UnhookWindowsHookEx.argtypes=[c.c_void_p]
        def callback(code,message,pointer):
            if code>=0 and c.cast(pointer,c.POINTER(w.DWORD))[0]==0x87:
                self.events.append(dict(t=time.monotonic(),message=int(message),down=message in (0x100,0x104)))
                return 1
            return user.CallNextHookEx(None,code,message,pointer)
        self.callback=callback_type(callback)
        handle=user.SetWindowsHookExW(13,self.callback,None,0)
        if not handle:
            self.error=c.get_last_error(); self.ready.set(); return
        self.ready.set(); msg=w.MSG()
        try:
            while not self.stop.is_set():
                while user.PeekMessageW(c.byref(msg),None,0,0,1):
                    user.TranslateMessage(c.byref(msg)); user.DispatchMessageW(c.byref(msg))
                time.sleep(.002)
        finally: user.UnhookWindowsHookEx(handle)

    def close(self):
        self.stop.set(); self.thread.join(3)


def playlist(folder,seconds=240):
    clips=[]; expected=[]; pos=32000
    for word in ("yes","no","up","down","left","right"):
        for path in sorted((folder/word).glob("*.wav"))[:2]:
            with wave.open(str(path),"rb") as source:
                assert source.getparams()[:3]==(1,2,16000)
                pcm=np.frombuffer(source.readframes(source.getnframes()),dtype="<i2").copy()
            clips.append((word,path,pcm))
    cycle=[np.zeros(32000,dtype="<i2")]
    for word,path,pcm in clips:
        expected.append(dict(word=word,file=str(path),t=pos/16000))
        cycle.extend([pcm,np.zeros(16000,dtype="<i2")]); pos+=len(pcm)+16000
    mono=np.concatenate(cycle)
    mono=np.tile(mono,math.ceil(seconds*16000/len(mono)))[:seconds*16000]
    return np.repeat(mono[:,None],2,axis=1).astype("<i2").tobytes(),expected


def tone_analysis(path,stages):
    with wave.open(str(path),"rb") as source:
        x=np.frombuffer(source.readframes(source.getnframes()),dtype="<i2").reshape(-1,2)[:,0].astype(float)
    result={"frames":len(x),"seconds":len(x)/16000,
            "range":[float(x.min()),float(x.max())] if len(x) else [],"stages":[]}
    for stage in stages:
        lo=max(1,int((stage["start"]+.3)*16000)); hi=min(len(x)-1,int((stage["end"]-.3)*16000))
        if hi<=lo: continue
        y=x[lo:hi]
        errors=np.abs(x[lo+1:hi+1]-2*np.cos(np.pi/8)*y+x[lo-1:hi-1])
        bad=np.flatnonzero(errors>50)
        groups=0 if not len(bad) else 1+int(np.count_nonzero(np.diff(bad)>4))
        result["stages"].append(dict(name=stage["name"],frames=len(y),zero_fraction=float(np.mean(y==0)),
            rms=float(np.sqrt(np.mean(y*y))),bad_samples=len(bad),discontinuities=groups,
            max_recurrence_error=float(errors.max()),bad_times=((bad[:30]+lo)/16000).tolist()))
    return result


def assess(summary):
    checks={"no_exception":not summary.get("error"),"no_uart_faults":not summary["faults"],
            "capture_cleanup":not summary.get("capture_cleanup_error")}
    if "tone" in summary:
        normal=[s for s in summary["tone"]["stages"]
                if s["name"] not in ("model_download","model_activate","model_restore","maintenance_pause")
                and not s["name"].startswith(("model_reload_","model_recovery_"))]
        checks["normal_tone_present"]=bool(normal) and all(s["rms"]>1000 for s in normal)
        checks["normal_tone_continuous"]=all(s["discontinuities"]==0 for s in normal)
        checks["capture_no_host_overflow"]=not summary.get("capture_faults")
        usb=summary.get("final_stats",{}).get("usb",{})
        checks["capture_packets_received"]=usb.get("capture2_tx_packets",0)>0
        for key in ("capture2_null_packets","capture2_failed_packets","capture2_unexpected_zero_packets",
                    "capture2_unexpected_sizes","endpoint_backpressure","endpoint_short_writes",
                    "playback_underflows","playback_dropped_bytes","playback_i2s_errors"):
            checks[key+"_unchanged"]=usb.get(key)==summary["stats"]["usb"].get(key)
        checks["kws_no_errors"]=summary.get("final_stats",{}).get("errors")==summary["stats"]["errors"]
        checks["kws_no_dropped_samples"]=summary.get("final_stats",{}).get("dropped_samples")==summary["stats"]["dropped_samples"]
        checks["kws_inference_advanced"]=summary.get("final_stats",{}).get("inference_chunks",0)>summary["stats"]["inference_chunks"]
        checks["kws_ready"]=summary.get("final_status",{}).get("runtime",{}).get("state")=="ready"
        if "capture1" in summary:
            checks["capture1_no_host_overflow"]=not summary["capture1"]["faults"]
            checks["capture1_frames_received"]=summary["capture1"]["frames"]>0
    if "keyboard_events" in summary:
        for stage in summary["stages"]:
            if not stage["name"].startswith("f24_"): continue
            events=[e for e in summary["keyboard_events"] if stage["start"]<=e["t"]<stage["end"]]
            down=sum(e["down"] for e in events); up=len(events)-down
            checks[stage["name"]]=(down==up==0 if stage["name"]=="f24_actions_disabled" else down==up and down>0)
    if "after_activate" in summary:
        checks["activate_without_reboot"]=summary["after_activate"]["boot_id"]==summary["status"]["boot_id"]
        checks["download_keeps_active_model"]=all(summary["after_download"]["active_model"][key]==summary["status"]["active_model"][key]
                                                   for key in ("source","package_sha256","class_count"))
        checks["activate_selected_slot"]=summary["after_activate"]["active_model"]["source"]==("slot_a" if summary["downloaded_slot"]==0 else "slot_b")
        checks["original_model_restored"]=summary["final_status"]["active_model"]["source"]==summary["status"]["active_model"]["source"]
        checks["maintenance_state_reported"]=all(s["value"]["audio"]["capture_output_mode"]=="maintenance_silence"
                                                   for s in summary["during_download"])
        muted=[s for s in summary["tone"]["stages"] if s["name"]=="model_download"]
        checks["maintenance_zero_pcm"]=bool(muted) and all(s["zero_fraction"]==1 and s["rms"]==0 for s in muted)
    if "reloads" in summary:
        checks["repeated_reload_ready"]=all(r["runtime"]["state"]=="ready" for r in summary["reloads"])
        checks["repeated_reload_no_reboot"]=all(r["boot_id"]==summary["status"]["boot_id"] for r in summary["reloads"])
    return checks


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode",choices=["smoke","audio","keyboard","update","maintenance"])
    parser.add_argument("--port",default="COM6"); parser.add_argument("--reset",action="store_true")
    parser.add_argument("--capture-raw",action="store_true",help="Also record the 8-channel raw input through WDM-KS")
    parser.add_argument("--quick",action="store_true",help="Shorten baseline/query stages for fault reproduction")
    parser.add_argument("--all-logs",action="store_true",help="Keep diagnostic UART statistics enabled during USB streams")
    parser.add_argument("--steady-seconds",type=float,default=0,
                        help="Minimum baseline/activated/restored observation time for performance comparisons")
    parser.add_argument("--reload-cycles",type=int,default=0,
                        help="Extra embedded/A/B activations with audio streaming (update mode)")
    parser.add_argument("--source",type=Path,default=Path("E:/Sony/mini_speech_commands"))
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    started=time.monotonic(); notes=[]; stages=[]; summary={}; audio=None; dev=None; client=None
    stop=threading.Event(); uart=serial.Serial(); uart.port=args.port; uart.baudrate=115200; uart.timeout=.05
    uart.dtr=uart.rts=False; uart.open()
    def note(event,**fields):
        row=dict(t=round(time.monotonic()-started,4),event=event,**fields); notes.append(row); print(json.dumps(row,ensure_ascii=True),flush=True)
    def receive():
        with (args.output/"uart.log").open("wb") as output:
            while not stop.is_set():
                data=uart.read(4096)
                if data: output.write(data); output.flush()
    reader=threading.Thread(target=receive,daemon=True); reader.start()
    original=None; touched=False; probe=None
    try:
        if args.reset:
            from esptool.reset import HardReset
            HardReset(uart)()
            if args.all_logs:
                # Enable the explicit ALL mode before USB enumeration can
                # hide startup memory diagnostics under the AUTO gate.
                time.sleep(2); uart.write(b"ll"); time.sleep(10)
            else:
                time.sleep(12)
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            matches=[d for d in hid.enumerate(0x303A,0x8003) if d.get("usage_page")==0xff00 and d.get("usage")==1]
            if len(matches)==1: break
            time.sleep(.25)
        if len(matches)!=1: raise RuntimeError(f"Control HID not found: {hid.enumerate(0x303A,0x8003)}")
        dev=hid.device(); dev.open_path(matches[0]["path"]); client=Client(dev)
        summary["hid"]={k:v for k,v in matches[0].items() if k!="path"}
        for name in ("info","caps","status","classes","bindings","stats"):
            summary[name]=client.read_object(name); note("read",object=name,value=summary[name])
        original=summary["bindings"]["bindings"]
        if summary["status"]["runtime"]["state"]!="ready":
            raise RuntimeError(f"Initial KWS is not ready: {summary['status']['runtime']}")
        if args.mode=="smoke": return
        touched=True
        for binding in original:
            binding=json.loads(json.dumps(binding)); binding["keyboard"]["enabled"]=False
            client.set_binding(binding)
        client.request(0x22,b"\x01")
        # PortAudio enumerates at import time, potentially before a reset or
        # flash has finished re-enumerating USB. Refresh before taking handles.
        sd._terminate(); sd._initialize()
        rows=devices(); summary["audio_devices"]=rows
        payload,expected=playlist(args.source); summary["playlist"]=expected
        audio=Audio(rows,payload,args.capture_raw); audio.start(); summary["audio_start_monotonic"]=audio.started
        if args.all_logs: uart.write(b"ll")
        note("audio_start",input=audio.input,output=audio.output)
        def stage(name,fn):
            begin=time.monotonic()-audio.started; note("stage_start",name=name)
            value=fn(); end=time.monotonic()-audio.started
            stages.append(dict(name=name,start=begin,end=end)); note("stage_end",name=name,duration=end-begin)
            return value
        def wait_ready():
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                status=client.read_object("status")
                if status["runtime"]["state"]=="ready": return status
                if status["runtime"]["state"]=="error": raise RuntimeError(f"KWS recovery failed: {status['runtime']}")
                time.sleep(.05)
            raise TimeoutError("KWS recovery did not reach ready")
        stage("baseline",lambda: time.sleep(max(args.steady_seconds,2 if args.quick else 10)))
        def command_load():
            counts=0; deadline=time.monotonic()+(3 if args.quick else 15)
            while time.monotonic()<deadline:
                for name in ("info","caps","status","classes","bindings","stats"):
                    client.read_object(name); counts+=1
                binding=json.loads(json.dumps(original[0])); binding["keyboard"]["enabled"]=False
                binding["led"]["rgb"]=[counts%256,255-counts%256,32]
                binding["actions_enabled"]=bool(counts%12)
                client.set_binding(binding)
            return counts
        summary["query_objects"]=stage("queries_and_live_bindings",command_load)
        stage("post_queries",lambda: time.sleep(2 if args.quick else 6))
        if args.mode=="maintenance":
            def pause_abort():
                transaction=client.request(UPDATE_BEGIN)["transaction"]
                time.sleep(3)
                summary["during_pause"]=client.read_object("status")
                client.request(UPDATE_ABORT,transaction=transaction)
            stage("maintenance_pause",pause_abort)
            stage("post_maintenance",lambda: time.sleep(10))
        if args.mode=="keyboard":
            probe=F24Probe()
            binding=json.loads(json.dumps(original[0])); binding["actions_enabled"]=True
            binding["led"]["enabled"]=True
            binding["keyboard"]={"enabled":True,"modifier":0,"keycodes":[0x73],"hold_ms":50}
            def observe(seconds):
                deadline=time.monotonic()+seconds
                while time.monotonic()<deadline:
                    data=dev.read(64,100)
                    if data:
                        event=decode(data)
                        if event["kind"]==2:
                            event["received_monotonic"]=time.monotonic(); client.events.append(event)
            client.set_binding(binding); stage("f24_enabled",lambda: observe(28))
            binding["actions_enabled"]=False; client.set_binding(binding)
            stage("f24_actions_disabled",lambda: observe(28))
            binding["actions_enabled"]=True; client.set_binding(binding)
            stage("f24_reenabled",lambda: observe(28))
            summary["keyboard_events"]=[dict(e,t=e["t"]-audio.started) for e in probe.events]
        if args.mode=="update":
            manifest=(ROOT/"src/custom/examples/manifest.json").read_bytes()
            model=(ROOT/"src/kws/model/ds_tc_resnet_backend_w16a16_esp32p4.espdl").read_bytes()
            summary["during_download"]=[]
            def downloading(kind,size):
                value=client.read_object("status")
                summary["during_download"].append(dict(object=kind,bytes=size,value=value))
                note("during_download",object=kind,value=value)
            slot=stage("model_download",lambda: client.download(manifest,model,on_object_started=downloading))
            summary["downloaded_slot"]=slot
            summary["after_download"]=client.read_object("status"); note("after_download",value=summary["after_download"])
            stage("model_recovery_download",wait_ready)
            stage("post_download",lambda: time.sleep(6))
            stage("model_activate",lambda: client.request(ACTIVATE,bytes([slot])))
            summary["after_activate"]=client.read_object("status"); note("after_activate",value=summary["after_activate"])
            stage("model_recovery_activate",wait_ready)
            stage("post_activate",lambda: time.sleep(max(args.steady_seconds,12)))
            old={"embedded":255,"slot_a":0,"slot_b":1}[summary["status"]["active_model"]["source"]]
            if args.reload_cycles:
                targets=[old]+[s["slot"] for s in summary["after_activate"]["slots"] if s["state"]=="valid"]
                summary["reloads"]=[]
                for index in range(args.reload_cycles):
                    target=targets[index%len(targets)]
                    stage(f"model_reload_{index}",lambda: client.request(ACTIVATE,bytes([target])))
                    # Disable keyboard mappings while recognition is warming up.
                    for binding in client.read_object("bindings")["bindings"]:
                        binding["keyboard"]["enabled"]=False; client.set_binding(binding)
                    stage(f"model_recovery_reload_{index}",wait_ready)
                    stage(f"post_reload_{index}",lambda: time.sleep(4))
                    status=client.read_object("status")
                    expected_source={255:"embedded",0:"slot_a",1:"slot_b"}[target]
                    if status["active_model"]["source"]!=expected_source: raise RuntimeError("Reload selected wrong model")
                    summary["reloads"].append(status); note("reload",index=index,value=status)
            # The restored model may have its original keyboard actions enabled.
            # Stop this test's speech before restoring those actions.
            audio.stop_playback(); time.sleep(.1)
            summary["playback_stopped_before_model_restore"]=True
            stage("model_restore",lambda: client.request(ACTIVATE,bytes([old])))
            stage("model_recovery_restore",wait_ready)
            stage("post_restore",lambda: time.sleep(max(args.steady_seconds,8)))
        summary["final_status"]=client.read_object("status"); summary["final_stats"]=client.read_object("stats")
        summary["events"]=[dict(event, payload=event["payload"].hex()) for event in client.events]
        note("audio_finish",stats=summary["final_stats"])
    except Exception as error:
        summary["error"]=repr(error); note("error",detail=repr(error)); raise
    finally:
        if probe: probe.close()
        if audio:
            try:
                audio.close(args.output/"capture2.wav")
                summary["tone"]=tone_analysis(args.output/"capture2.wav",stages)
                summary["capture_faults"]=audio.capture_faults
                if args.capture_raw:
                    raw=np.frombuffer(b"".join(audio.raw_blocks),dtype="<i2")
                    summary["capture1"]={"frames":len(raw)//8,"seconds":len(raw)/8/24000,
                        "faults":audio.raw_faults,"nonzero_samples":int(np.count_nonzero(raw))}
            except Exception as error:
                summary["capture_cleanup_error"]=repr(error)
        if client and original and touched:
            try:
                if args.mode=="update" and summary.get("error"):
                    current=client.read_object("status")["active_model"]["source"]
                    desired=summary["status"]["active_model"]["source"]
                    if current!=desired:
                        client.request(ACTIVATE,bytes([{"embedded":255,"slot_a":0,"slot_b":1}[desired]]))
                        time.sleep(2)
                        summary["restored_model_after_error"]=client.read_object("status")
                current={b["class_index"]:b for b in client.read_object("bindings")["bindings"]}
                for binding in original:
                    if current.get(binding["class_index"])!=binding: client.set_binding(binding)
                summary["restored_bindings"]=True
            except Exception as error: summary["restore_error"]=repr(error)
        if dev: dev.close()
        stop.set(); reader.join(2); uart.close()
        summary["stages"]=stages
        summary["faults"]=[token for token in ("Guru Meditation","assert failed","stack overflow","Task watchdog got triggered","abort() was called")
                           if token in (args.output/"uart.log").read_text(encoding="utf-8",errors="replace")]
        summary["checks"]=assess(summary)
        summary["validation_failures"]=[key for key,value in summary["checks"].items() if not value]
        (args.output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
        (args.output/"timeline.json").write_text(json.dumps(notes,indent=2,ensure_ascii=False),encoding="utf-8")
        print(json.dumps({"result":str(args.output/"summary.json"),"error":summary.get("error"),"tone":summary.get("tone"),"faults":summary["faults"]}),flush=True)
        if summary["validation_failures"] and not summary.get("error"):
            raise RuntimeError(f"Validation failed: {summary['validation_failures']}")


if __name__=="__main__": main()
