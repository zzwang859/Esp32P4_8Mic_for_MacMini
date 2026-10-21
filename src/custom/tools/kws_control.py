"""Reference HIDAPI client; protocol codec also works without HIDAPI installed."""
from __future__ import annotations

import argparse
import hashlib
import json
import secrets
import struct
import time
import zlib
from pathlib import Path

HEADER = struct.Struct("<2sBBHHIIHH")
OPEN_SESSION = 0x04
READ_BEGIN, READ_CHUNK, READ_END = 0x10, 0x11, 0x12
SET_BINDING, SAVE_CONFIG = 0x20, 0x21
SET_KWS_FILTER = 0x23
UPDATE_BEGIN, OBJECT_BEGIN, OBJECT_DATA, OBJECT_END = 0x30, 0x31, 0x32, 0x33
UPDATE_COMMIT, UPDATE_ABORT, ACTIVATE = 0x34, 0x35, 0x36
OBJECTS = {"info": 1, "caps": 2, "status": 3, "classes": 4, "bindings": 5, "stats": 6}
ERRORS = {
    2: "BUSY", 3: "BAD_VERSION", 4: "BAD_LENGTH", 5: "BAD_CRC",
    6: "INVALID_STATE", 7: "OUT_OF_RANGE", 8: "OFFSET_MISMATCH",
    9: "HASH_MISMATCH", 10: "MODEL_INCOMPATIBLE", 11: "LOAD_FAILED",
    12: "NO_SPACE", 13: "FLASH_ERROR", 14: "CONFLICT", 15: "NOT_FOUND",
    16: "MODEL_CHANGED", 17: "QUEUE_FULL", 18: "BAD_COMMAND",
}


def encode(command, sequence, payload=b"", transaction=0, offset=0, kind=0, status=0):
    if len(payload) > 40:
        raise ValueError("Payload exceeds 40 bytes")
    body = HEADER.pack(b"KC", 1, kind, command, sequence, transaction, offset, len(payload), status)
    body += payload.ljust(40, b"\0")
    return body + struct.pack("<I", zlib.crc32(body))


def decode(wire):
    wire = bytes(wire)
    if len(wire) != 64:
        raise ValueError("Report must contain 64 bytes")
    magic, version, kind, command, sequence, transaction, offset, size, status = HEADER.unpack_from(wire)
    if magic != b"KC" or version != 1 or size > 40:
        raise ValueError("Invalid report header")
    if zlib.crc32(wire[:60]) != struct.unpack_from("<I", wire, 60)[0]:
        raise ValueError("Invalid CRC32")
    if any(wire[20 + size:60]):
        raise ValueError("Nonzero report padding")
    return dict(kind=kind, command=command, sequence=sequence, transaction=transaction,
                offset=offset, status=status, payload=wire[20:20 + size])


class Client:
    def __init__(self, device):
        self.device = device
        self.sequence = 0
        self.events = []
        self.session = secrets.token_bytes(16)
        self.session_open = False

    def request(self, command, payload=b"", transaction=0, offset=0, timeout=15.0):
        if not self.session_open and command != OPEN_SESSION:
            result = self.request(OPEN_SESSION, self.session, timeout=timeout)
            if result["payload"] != self.session:
                raise ValueError("Session nonce mismatch")
            self.session_open = True
        self.sequence = (self.sequence + 1) & 0xFFFF
        wire = encode(command, self.sequence, payload, transaction, offset)
        # A timeout retries exactly the same frame, including sequence and offset.
        for _ in range(3):
            if self.device.write(b"\0" + wire) != 65:
                raise OSError("Incomplete HID write")
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                data = self.device.read(64, 100)
                if not data:
                    continue
                result = decode(data)
                if result["kind"] == 2:
                    result["received_monotonic"] = time.monotonic()
                    self.events.append(result)
                    continue
                if result["kind"] != 1 or result["sequence"] != self.sequence or result["command"] != command:
                    continue
                if command == OPEN_SESSION and result["status"] == 0 and result["payload"] != payload:
                    continue  # A stale response from a previously closed process.
                if result["status"] > 1:
                    raise RuntimeError(ERRORS.get(result["status"], f"Error {result['status']}"))
                return result
        raise TimeoutError(f"Command 0x{command:04x}, seq {self.sequence}")

    def read_object(self, name):
        result = self.request(READ_BEGIN, struct.pack("<H", OBJECTS[name]))
        token = result["transaction"]
        size, = struct.unpack_from("<I", result["payload"])
        data = bytearray()
        try:
            while len(data) < size:
                chunk = self.request(READ_CHUNK, transaction=token, offset=len(data))["payload"]
                if not chunk or len(data) + len(chunk) > size:
                    raise ValueError("Invalid object chunk")
                data.extend(chunk)
        except BaseException:
            try:
                self.request(READ_END, transaction=token)
            except Exception:
                pass  # Preserve the read error if cleanup also loses USB.
            raise
        else:
            self.request(READ_END, transaction=token)
        return json.loads(data.decode("utf-8"))

    def set_binding(self, binding):
        current = self.read_object("status")
        led, key = binding.get("led", {}), binding.get("keyboard", {})
        flags = int(bool(binding.get("actions_enabled", False)))
        flags |= int(bool(led.get("enabled", False))) << 1
        flags |= int(bool(key.get("enabled", False))) << 2
        if led.get("effect", "flash") != "flash":
            raise ValueError("Only the flash LED effect is supported")
        rgb = led.get("rgb", [0, 0, 0])
        keys = key.get("keycodes", [])
        if len(rgb) != 3 or len(keys) > 6:
            raise ValueError("Expected 3 RGB bytes and at most 6 keycodes")
        payload = struct.pack("<IIIHB3BB6BHH", current["boot_id"], current["model_epoch"],
                              current["config_revision"], binding["class_index"], flags,
                              *rgb, key.get("modifier", 0), *(keys + [0] * (6 - len(keys))),
                              led.get("duration_ms", 600), key.get("hold_ms", 0))
        return self.request(SET_BINDING, payload)

    def save(self):
        current = self.read_object("status")
        return self.request(SAVE_CONFIG, struct.pack("<III", current["boot_id"],
                            current["model_epoch"], current["config_revision"]))

    def set_kws_filter(self, threshold_percent: int, cooldown_ms: int,
                       stable_frames: int, need_vad_checked: bool):
        values = (threshold_percent, cooldown_ms, stable_frames)
        if any(type(value) is not int for value in values) or not (
            0 <= threshold_percent <= 100 and 1 <= cooldown_ms <= 60000 and
            1 <= stable_frames <= 1000 and type(need_vad_checked) is bool
        ):
            raise ValueError("KWS filter requires threshold 0..100, cooldown 1..60000 ms, "
                             "frames 1..1000 and a boolean VAD setting")
        current = self.read_object("status")
        revision = current["kws_filter"]["revision"]
        payload = struct.pack("<IIHIHB", current["boot_id"], revision,
                              threshold_percent, cooldown_ms, stable_frames,
                              int(need_vad_checked))
        result = self.request(SET_KWS_FILTER, payload)
        return {"filter_revision": struct.unpack("<I", result["payload"])[0],
                "kws_filter": self.read_object("status")["kws_filter"]}

    def download(self, manifest: bytes, model: bytes, activate=False, on_object_started=None):
        result = self.request(UPDATE_BEGIN)
        transaction, slot = result["transaction"], result["payload"][0]
        try:
            for kind, data in ((0x100, manifest), (0x101, model)):
                self.request(OBJECT_BEGIN, struct.pack("<HI", kind, len(data)) + hashlib.sha256(data).digest(), transaction)
                if on_object_started:
                    on_object_started(kind, len(data))
                for offset in range(0, len(data), 40):
                    ack = self.request(OBJECT_DATA, data[offset:offset + 40], transaction, offset)
                    if struct.unpack("<I", ack["payload"])[0] != min(offset + 40, len(data)):
                        raise ValueError("Device accepted an unexpected offset")
                self.request(OBJECT_END, transaction=transaction)
            self.request(UPDATE_COMMIT, transaction=transaction)
        except BaseException:
            try:
                self.request(UPDATE_ABORT, transaction=transaction)
            except Exception:
                pass  # Preserve the download error if the device disconnected.
            raise
        if activate:
            self.request(ACTIVATE, bytes([slot]))
        return slot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial")
    sub = parser.add_subparsers(dest="command", required=True)
    get = sub.add_parser("get"); get.add_argument("object", choices=OBJECTS)
    bind = sub.add_parser("bind"); bind.add_argument("file", type=Path)
    filter_command = sub.add_parser("filter")
    filter_command.add_argument("threshold_percent", type=int)
    filter_command.add_argument("cooldown_ms", type=int)
    filter_command.add_argument("stable_frames", type=int)
    filter_command.add_argument("need_vad_checked", choices=["true", "false", "1", "0"])
    sub.add_parser("save")
    update = sub.add_parser("download")
    update.add_argument("model", type=Path); update.add_argument("manifest", type=Path)
    update.add_argument("--activate", action="store_true")
    activate = sub.add_parser("activate"); activate.add_argument("slot", choices=["a", "b", "factory"])
    args = parser.parse_args()
    import hid  # pip install hidapi; optional until actual device access
    matches = [d for d in hid.enumerate(0x303A, 0x8003)
               if d.get("usage_page") == 0xFF00 and d.get("usage") == 1
               and (not args.serial or d.get("serial_number") == args.serial)]
    if len(matches) != 1:
        raise SystemExit(f"Expected one control HID, found {len(matches)}; select --serial if needed")
    device = hid.device(); device.open_path(matches[0]["path"])
    try:
        client = Client(device)
        if args.command == "get": result = client.read_object(args.object)
        elif args.command == "bind": result = client.set_binding(json.loads(args.file.read_text(encoding="utf-8")))
        elif args.command == "filter": result = client.set_kws_filter(
            args.threshold_percent, args.cooldown_ms, args.stable_frames,
            args.need_vad_checked in ("true", "1"))
        elif args.command == "save": result = client.save()
        elif args.command == "activate": result = client.request(ACTIVATE, bytes([{"a": 0, "b": 1, "factory": 255}[args.slot]]))
        else: result = {"downloaded_slot": client.download(args.manifest.read_bytes(), args.model.read_bytes(), args.activate)}
        print(json.dumps(result, ensure_ascii=False, indent=2, default=lambda x: x.hex() if isinstance(x, bytes) else str(x)))
    finally:
        device.close()


if __name__ == "__main__":
    main()
