"""Transport tests without a USB device or the optional hidapi package."""
import hashlib
import importlib.util
import json
import struct
import unittest
from pathlib import Path

spec=importlib.util.spec_from_file_location("kws_control",Path(__file__).parents[1]/"tools/kws_control.py")
control=importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


class FakeDevice:
    def __init__(self, timeout_once=False):
        self.requests=[]; self.replies=[]; self.timeout_once=timeout_once
        self.object_data=json.dumps({"classes":[{"class_index":2,"text":"开灯"}]} ,ensure_ascii=False).encode()
        self.offset=0; self.objects=[]; self.data=bytearray()

    def write(self, data):
        assert len(data)==65 and data[0]==0
        request=control.decode(data[1:]); self.requests.append(bytes(data))
        if self.timeout_once:
            self.timeout_once=False
            return 65
        cmd=request["command"]; payload=b""; token=request["transaction"]
        if cmd==control.OPEN_SESSION:
            payload=request["payload"]
        elif cmd==control.READ_BEGIN:
            token=99; payload=struct.pack("<IIII",len(self.object_data),1,1,1)
        elif cmd==control.READ_CHUNK:
            payload=self.object_data[request["offset"]:request["offset"]+40]
        elif cmd==control.UPDATE_BEGIN:
            token=123; payload=b"\x01"
        elif cmd==control.SET_KWS_FILTER:
            payload=struct.pack("<I",8)
        elif cmd==control.OBJECT_BEGIN:
            kind,size=struct.unpack_from("<HI",request["payload"])
            self.expected=(kind,size,request["payload"][6:]); self.offset=0; self.data=bytearray()
        elif cmd==control.OBJECT_DATA:
            assert request["offset"]==self.offset
            self.data.extend(request["payload"]); self.offset+=len(request["payload"])
            payload=struct.pack("<I",self.offset)
        elif cmd==control.OBJECT_END:
            assert len(self.data)==self.expected[1]
            assert hashlib.sha256(self.data).digest()==self.expected[2]
            self.objects.append((self.expected[0],bytes(self.data)))
        self.replies.append(control.encode(cmd,request["sequence"],payload,token,request["offset"],kind=1))
        return 65

    def read(self, size, timeout):
        return self.replies.pop(0) if self.replies else []


class ClientTest(unittest.TestCase):
    def test_frame_and_corruption(self):
        wire=control.encode(0x32,0xffff,b"\0\xff",0x12345678,4090)
        self.assertEqual(len(wire),64)
        self.assertEqual(control.decode(wire)["payload"],b"\0\xff")
        for i in range(64):
            bad=bytearray(wire); bad[i]^=0x80
            with self.assertRaises(ValueError): control.decode(bad)

    def test_snapshot_utf8_and_eof(self):
        device=FakeDevice(); client=control.Client(device)
        self.assertEqual(client.read_object("classes")["classes"][0]["text"],"开灯")
        self.assertEqual(control.decode(device.requests[-1][1:])["command"],control.READ_END)

    def test_timeout_retries_identical_bytes(self):
        device=FakeDevice(timeout_once=True); client=control.Client(device)
        client.session_open=True
        client.request(0x22,b"\x01",timeout=0.001)
        self.assertEqual(device.requests[0],device.requests[1])

    def test_session_timeout_retries_identical_nonce(self):
        device=FakeDevice(timeout_once=True); client=control.Client(device)
        client.request(0x22,b"\x01",timeout=0.001)
        self.assertEqual(device.requests[0],device.requests[1])
        self.assertEqual(control.decode(device.requests[0][1:])["command"],control.OPEN_SESSION)

    def test_reopened_client_starts_a_distinct_session(self):
        device=FakeDevice()
        first=control.Client(device); first.read_object("classes")
        second=control.Client(device); second.read_object("classes")
        sessions=[control.decode(r[1:]) for r in device.requests
                  if control.decode(r[1:])["command"]==control.OPEN_SESSION]
        self.assertEqual(len(sessions),2)
        self.assertNotEqual(sessions[0]["payload"],sessions[1]["payload"])
        self.assertEqual(sessions[0]["sequence"],sessions[1]["sequence"])

    def test_session_ignores_previous_process_response(self):
        device=FakeDevice()
        device.replies.append(control.encode(control.OPEN_SESSION,1,b"old-session-id!!",kind=1))
        client=control.Client(device)
        client.read_object("classes")
        self.assertTrue(client.session_open)

    def test_snapshot_cleanup_preserves_original_failure(self):
        class FailedDevice(FakeDevice):
            def write(self,data):
                command=control.decode(data[1:])["command"]
                if command==control.READ_CHUNK: raise ValueError("Original read failure")
                if command==control.READ_END: raise OSError("Cleanup failed")
                return super().write(data)
        with self.assertRaisesRegex(ValueError,"Original read failure"):
            control.Client(FailedDevice()).read_object("classes")

    def test_complete_download_and_explicit_activation(self):
        device=FakeDevice(); client=control.Client(device)
        manifest=b'{"format_version":1}'; model=b"EDL2"+bytes(range(256))*20
        self.assertEqual(client.download(manifest,model,activate=True),1)
        self.assertEqual(device.objects,[(0x100,manifest),(0x101,model)])
        commands=[control.decode(r[1:])["command"] for r in device.requests]
        self.assertEqual(commands[-2:],[control.UPDATE_COMMIT,control.ACTIVATE])
        self.assertNotIn(control.UPDATE_ABORT,commands)

    def test_binding_wire_layout(self):
        device=FakeDevice()
        device.object_data=json.dumps({"boot_id":1234,"model_epoch":3,"config_revision":7}).encode()
        client=control.Client(device)
        client.set_binding({"class_index":2,"actions_enabled":True,
            "led":{"enabled":True,"rgb":[0,255,0],"duration_ms":600},
            "keyboard":{"enabled":True,"modifier":1,"keycodes":[6],"hold_ms":30}})
        request=control.decode(device.requests[-1][1:])
        self.assertEqual(request["command"],control.SET_BINDING)
        self.assertEqual(len(request["payload"]),29)
        payload=request["payload"]
        self.assertEqual(struct.unpack_from("<IIIH",payload),(1234,3,7,2))
        self.assertEqual(payload[14:25],bytes([7,0,255,0,1,6,0,0,0,0,0]))
        self.assertEqual(struct.unpack_from("<HH",payload,25),(600,30))

    def test_kws_filter_wire_and_ranges(self):
        device=FakeDevice()
        device.object_data=json.dumps({"boot_id":1234,"kws_filter":{"revision":7}}).encode()
        client=control.Client(device)
        with self.assertRaises(ValueError): client.set_kws_filter(101,500,3,False)
        with self.assertRaises(ValueError): client.set_kws_filter(80,0,3,False)
        with self.assertRaises(ValueError): client.set_kws_filter(80,500,0,False)
        with self.assertRaises(ValueError): client.set_kws_filter(80,500,3,1)
        client.set_kws_filter(0,1,1000,True)
        requests=[control.decode(raw[1:]) for raw in device.requests]
        request=next(item for item in requests if item["command"]==control.SET_KWS_FILTER)
        self.assertEqual(struct.unpack("<IIHIHB",request["payload"]),(1234,7,0,1,1000,1))

    def test_failed_download_aborts(self):
        class FailedDevice(FakeDevice):
            def write(self,data):
                result=super().write(data)
                request=control.decode(data[1:])
                if request["command"]==control.OBJECT_END:
                    self.replies[-1]=control.encode(request["command"],request["sequence"],
                        transaction=request["transaction"],kind=1,status=9)
                return result
        device=FailedDevice(); client=control.Client(device)
        with self.assertRaisesRegex(RuntimeError,"HASH_MISMATCH"):
            client.download(b"{}",b"EDL2")
        self.assertEqual(control.decode(device.requests[-1][1:])["command"],control.UPDATE_ABORT)


if __name__=="__main__": unittest.main()
