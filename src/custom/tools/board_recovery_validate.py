"""Destructive-to-inactive-slot recovery checks on a test board with COM6 + HID.

Run with the embedded model selected, after board_validate.py update populated a valid slot. Tests reset during
download, rejected model output shape, successful A/B activation, and bad SHA.
Restores the initial active model and RAM bindings. No physical power cut.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import threading
import time

import hid
import serial
from esptool.reset import HardReset
from kws_control import Client, ACTIVATE, UPDATE_BEGIN, UPDATE_ABORT, OBJECT_BEGIN, OBJECT_DATA, OBJECT_END

ROOT=Path(__file__).resolve().parents[3]


def connect():
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        matches=[d for d in hid.enumerate(0x303a,0x8003) if d.get("usage_page")==0xff00 and d.get("usage")==1]
        if len(matches)==1:
            dev=hid.device()
            try:
                dev.open_path(matches[0]["path"])
                client=Client(dev); client.read_object("info")
                return dev,client
            except OSError:
                dev.close()
        time.sleep(.2)
    raise TimeoutError("Control HID unavailable")


def upload(client,token,kind,data,sha=None,finish=True):
    client.request(OBJECT_BEGIN,struct.pack("<HI",kind,len(data))+(sha or hashlib.sha256(data).digest()),token)
    for offset in range(0,len(data) if finish else 240,40):
        client.request(OBJECT_DATA,data[offset:offset+40],token,offset)
    if finish: client.request(OBJECT_END,transaction=token)


def expect_error(action,name):
    try: action()
    except RuntimeError as error:
        if str(error)==name: return
        raise
    raise AssertionError(f"Expected {name}")


def ready(client):
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        status=client.read_object("status")
        if status["runtime"]["state"]=="ready": return status
        time.sleep(.1)
    raise TimeoutError(f"KWS did not become ready: {status}")


def save_and_wait(client):
    client.save()
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        status=client.read_object("status")
        if not status["runtime"]["save_pending"]:
            assert not status["runtime"]["config_dirty"]
            return
        time.sleep(.1)
    raise TimeoutError("Config save did not complete within 10 seconds")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",default="COM6"); parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--skip-save",action="store_true",help="Skip the configuration persistence checks")
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    uart=serial.Serial(); uart.port=args.port; uart.baudrate=115200; uart.timeout=.1
    uart.dtr=uart.rts=False; uart.open()
    stop=threading.Event(); log=bytearray(); result={}; dev=None; client=None; initial=None; bindings=None
    def receive():
        while not stop.is_set(): log.extend(uart.read(4096))
    reader=threading.Thread(target=receive,daemon=True); reader.start()
    def note(name,value):
        result[name]=value; print(json.dumps({name:value}),flush=True)
    try:
        dev,client=connect(); initial=client.read_object("status"); bindings=client.read_object("bindings")["bindings"]
        note("initial",initial)
        if initial["active_model"]["source"]!="embedded":
            raise RuntimeError("Recovery checks require the embedded model to be active on this test board")
        valid=[slot["slot"] for slot in initial["slots"] if slot["state"]=="valid"]
        if not valid: raise RuntimeError("Run the update validation first to populate one valid slot")
        active=valid[0]; source="slot_a" if active==0 else "slot_b"
        if initial["active_model"]["source"]!=source: client.request(ACTIVATE,bytes([active]))
        before=ready(client)
        slot_bindings=client.read_object("bindings")["bindings"]
        test_binding=json.loads(json.dumps(slot_bindings[0]))
        test_binding["actions_enabled"]=False
        test_binding["led"]["rgb"]=[17,34,51]
        if not args.skip_save:
            client.set_binding(test_binding); save_and_wait(client)
        else:
            test_binding=slot_bindings[0]
        manifest=(ROOT/"src/custom/examples/manifest.json").read_bytes()
        model=(ROOT/"src/kws/model/ds_tc_resnet_backend_w16a16_esp32p4.espdl").read_bytes()
        token=client.request(UPDATE_BEGIN)["transaction"]
        upload(client,token,0x100,manifest)
        upload(client,token,0x101,model,finish=False)
        note("interrupted",client.read_object("status"))
        dev.close(); dev=None
        HardReset(uart)(); time.sleep(4)
        dev,client=connect(); after=ready(client)
        assert after["boot_id"]!=before["boot_id"]
        assert after["active_model"]["source"]==source and after["runtime"]["state"]=="ready"
        assert after["slots"][1-active]["state"]=="empty_or_invalid"
        note("reset_recovery",after)
        restored_binding=client.read_object("bindings")["bindings"][0]
        assert restored_binding==test_binding
        note("binding_persisted_across_reset",True)
        incompatible=json.loads(manifest)
        incompatible["classes"].append({"class_index":8,"text":"shape_mismatch_test"})
        target=client.download(json.dumps(incompatible).encode(),model)
        expect_error(lambda:client.request(ACTIVATE,bytes([target])),"LOAD_FAILED")
        after=ready(client)
        assert after["active_model"]["source"]==source and after["runtime"]["state"]=="ready"
        note("load_failure_rollback",after)
        if not args.skip_save:
            for binding in slot_bindings: client.set_binding(binding)
            save_and_wait(client)
        token=client.request(UPDATE_BEGIN)["transaction"]
        expect_error(lambda:upload(client,token,0x100,manifest,sha=bytes(32)),"HASH_MISMATCH")
        client.request(UPDATE_ABORT,transaction=token)
        after=ready(client)
        assert after["audio"]["processing_state"]=="running"
        note("bad_hash_rejected",True)
        target=client.download(manifest,model,activate=True)
        after=ready(client)
        assert after["active_model"]["source"]==("slot_a" if target==0 else "slot_b")
        assert after["runtime"]["state"]=="ready"
        note("other_slot_activated",after)
    except Exception as error:
        result["error"]=repr(error); raise
    finally:
        if dev and client and initial:
            try:
                desired={"embedded":255,"slot_a":0,"slot_b":1}[initial["active_model"]["source"]]
                current=client.read_object("status")["active_model"]["source"]
                if current!=initial["active_model"]["source"]: client.request(ACTIVATE,bytes([desired]))
                current_bindings={b["class_index"]:b for b in client.read_object("bindings")["bindings"]}
                for binding in bindings:
                    if current_bindings.get(binding["class_index"])!=binding: client.set_binding(binding)
                note("restored",client.read_object("status"))
            except Exception as error: result["restore_error"]=repr(error)
        if dev: dev.close()
        stop.set(); reader.join(2); uart.close()
        (args.output/"uart.log").write_bytes(log)
        result["faults"]=[token for token in (b"Task watchdog got triggered",b"Guru Meditation",b"abort() was called") if token in log]
        result["faults"]=[token.decode() for token in result["faults"]]
        (args.output/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        if result.get("faults") or result.get("restore_error"): raise RuntimeError("Recovery validation/restore failed")


if __name__=="__main__": main()
