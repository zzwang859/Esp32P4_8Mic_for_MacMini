"""Validate SAVE_CONFIG with USB audio open, then reset and verify persistence.

--expect-deferred reproduces the old playback gate without changing bindings.
Normal mode restores the supplied scene's mappings and leaves boot action flags off.
Requires hidapi, pyserial, sounddevice and esptool. Release the serial monitor first.
"""
import argparse
import copy
import json
from pathlib import Path
import struct
import threading
import time

import serial
import sounddevice as sd
from esptool.reset import HardReset
from board_recovery_validate import connect, ready


class Streams:
    def __init__(self, mask):
        self.streams = []
        self.frames = {"playback": 0, "capture1": 0, "capture2": 0}
        self.faults = []
        self.blocks = []
        devices = list(sd.query_devices())
        apis = sd.query_hostapis()

        def choose(kind):
            found = []
            for i, d in enumerate(devices):
                name = d["name"].lower()
                if "usb audio" not in name or "ab13x" in name:
                    continue
                api = apis[d["hostapi"]]["name"]
                if kind == "playback" and d["max_output_channels"] and api == "Windows WASAPI":
                    found.append(i)
                if kind == "capture2" and "capture 2" in name and d["max_input_channels"] and api == "Windows WASAPI":
                    found.append(i)
                if kind == "capture1" and "capture 2" not in name and d["max_input_channels"] >= 8 and api == "Windows WDM-KS":
                    found.append(i)
            if len(found) != 1:
                raise RuntimeError(f"Ambiguous {kind} endpoint: {found}")
            return found[0]

        def callback(kind):
            def run(data, frames, timing, status):
                if kind == "playback":
                    data[:] = bytes(len(data))  # Exercise an open playback stream without audible sound.
                elif kind == "capture2":
                    self.blocks.append(bytes(data))
                self.frames[kind] += frames
                if status:
                    self.faults.append({"t": time.monotonic(), "stream": kind, "status": str(status)})
            return run

        try:
            for bit, kind, rate, channels in ((1, "playback", 16000, 2),
                                               (2, "capture1", 24000, 8),
                                               (4, "capture2", 16000, 2)):
                if not mask & bit:
                    continue
                cls = sd.RawOutputStream if kind == "playback" else sd.RawInputStream
                extra = {} if kind == "capture1" else {"extra_settings": sd.WasapiSettings(exclusive=True)}
                stream = cls(device=choose(kind), samplerate=rate, channels=channels,
                             dtype="int16", blocksize=rate // 100, callback=callback(kind), **extra)
                self.streams.append(stream)
                stream.start()
        except BaseException:
            self.close()
            raise

    def close(self):
        for stream in reversed(self.streams):
            try:
                stream.stop()
            finally:
                stream.close()
        self.streams.clear()


def wait_save(client, timeout=10):
    started = time.monotonic()
    while time.monotonic() - started < timeout:
        status = client.read_object("status")
        if not status["runtime"]["save_pending"]:
            if status["runtime"]["config_dirty"]:
                raise AssertionError(f"Save ended but current config is still dirty: {status}")
            return status, round(time.monotonic() - started, 4)
        time.sleep(.03)
    raise TimeoutError(f"SAVE_CONFIG did not finish with streams open: {status}")


def wait_streams(client, mask):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        status = client.read_object("status")
        if status["audio"]["stream_mask"] & mask == mask:
            return status
        time.sleep(.05)
    raise AssertionError(f"Expected stream bits {mask:#x}: {status['audio']}")


def disabled(binding):
    binding = copy.deepcopy(binding)
    binding["actions_enabled"] = False
    binding["led"]["enabled"] = False
    binding["keyboard"]["enabled"] = False
    return binding


def model_identity(status):
    model = status["active_model"]
    return {key: model[key] for key in ("source", "package_sha256", "model_bytes", "class_count")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--expect-deferred", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {"stages": []}
    uart = serial.Serial()
    uart.port, uart.baudrate, uart.timeout = args.port, 115200, .05
    uart.dtr = uart.rts = False
    uart.open()
    stop = threading.Event()
    log = bytearray()

    def receive():
        while not stop.is_set():
            log.extend(uart.read(4096))

    reader = threading.Thread(target=receive, daemon=True)
    reader.start()
    dev = client = audio = None
    original = None
    touched = False
    try:
        dev, client = connect()
        initial = {name: client.read_object(name) for name in ("info", "status", "bindings", "stats")}
        summary["initial"] = initial
        (args.output / "initial.json").write_text(json.dumps(initial, indent=2), encoding="utf-8")
        scene = json.loads(args.scene.read_text(encoding="utf-8-sig")) if args.scene else initial
        original = scene["bindings"]["bindings"]
        assert model_identity(initial["status"]) == model_identity(scene["status"])
        if args.expect_deferred:
            audio = Streams(1)
            summary["opened"] = wait_streams(client, 1)
            summary["accepted"] = client.save()["status"]
            time.sleep(1)
            summary["pending"] = client.read_object("status")
            assert summary["accepted"] == 1 and summary["pending"]["runtime"]["save_pending"]
            try:
                client.save()
            except RuntimeError as error:
                assert str(error) == "BUSY"
                summary["second_save"] = "BUSY"
            else:
                raise AssertionError("Old firmware did not reproduce BUSY")
            audio.close()
            audio = None
            summary["completed_after_close"], _ = wait_save(client)
            summary["reproduced"] = True
        else:
            desired = [disabled(b) for b in original]
            assert initial["bindings"]["bindings"] == desired, "Firmware reset must retain scene parameters with all flags off"
            touched = True
            for mask in (1, 4, 6, 7):
                audio = Streams(mask)
                stage = {"requested_mask": mask, "opened": wait_streams(client, mask), "saves": []}
                time.sleep(.5)
                stage["before_stats"] = client.read_object("stats")
                fault_start = len(audio.faults)
                frames_start = dict(audio.frames)
                for attempt in range(2):
                    probe = copy.deepcopy(desired[0])
                    probe["led"]["rgb"] = [31 + mask, 79 + attempt, 127]
                    probe["keyboard"]["modifier"] = 3
                    probe["keyboard"]["keycodes"] = [6, 0, 0, 0, 0, 0]
                    probe["keyboard"]["hold_ms"] = 30
                    client.set_binding(probe)
                    response = client.save()
                    assert response["status"] == 1
                    status, duration = wait_save(client)
                    assert status["audio"]["stream_mask"] & mask == mask
                    assert status["audio"]["processing_state"] == "running"
                    assert status["audio"]["capture_output_mode"] == "normal"
                    assert client.read_object("bindings")["bindings"][0] == probe
                    events = [e for e in client.events if e["command"] == 0x8002 and
                              struct.unpack("<II", e["payload"])[1] == status["config_revision"]]
                    assert events and events[-1]["status"] == 0
                    stage["saves"].append({"status": status, "seconds": duration, "event_status": 0})
                time.sleep(.5)
                stage["after_stats"] = client.read_object("stats")
                stage["frames_during_saves"] = {key: audio.frames[key] - frames_start[key] for key in audio.frames}
                stage["faults_during_saves"] = audio.faults[fault_start:]
                assert not stage["faults_during_saves"], stage["faults_during_saves"]
                assert stage["after_stats"]["inference_chunks"] > stage["before_stats"]["inference_chunks"]
                for key in ("errors", "resets", "dropped_samples"):
                    assert stage["after_stats"][key] == stage["before_stats"][key], key
                for key in ("raw_null_packets", "capture2_null_packets", "capture2_failed_packets",
                            "capture2_unexpected_zero_packets", "capture2_unexpected_sizes", "endpoint_backpressure",
                            "endpoint_short_writes", "playback_underflows", "playback_dropped_bytes", "playback_i2s_errors"):
                    assert stage["after_stats"]["usb"][key] == stage["before_stats"]["usb"][key], key
                for bit, kind in ((1, "playback"), (2, "capture1"), (4, "capture2")):
                    if mask & bit:
                        assert stage["frames_during_saves"][kind] > 0
                summary["stages"].append(stage)
                print(json.dumps({"mask": mask, "actual_mask": stage["opened"]["audio"]["stream_mask"],
                                  "save_seconds": [s["seconds"] for s in stage["saves"]],
                                  "frames": stage["frames_during_saves"], "faults": stage["faults_during_saves"]}), flush=True)
                audio.close()
                audio = None
                time.sleep(.3)
            for binding in desired:
                client.set_binding(binding)
            client.save()
            summary["restored_before_reset"], _ = wait_save(client)
            assert client.read_object("bindings")["bindings"] == desired
            before_boot = summary["restored_before_reset"]["boot_id"]
            dev.close()
            dev = None
            HardReset(uart)()
            time.sleep(3)
            dev, client = connect()
            summary["after_reset"] = ready(client)
            summary["bindings_after_reset"] = client.read_object("bindings")
            assert summary["after_reset"]["boot_id"] != before_boot
            assert summary["bindings_after_reset"]["bindings"] == desired
            assert model_identity(summary["after_reset"]) == model_identity(scene["status"])
            assert summary["after_reset"]["slots"] == scene["status"]["slots"]
            assert not summary["after_reset"]["runtime"]["save_pending"]
            touched = False
            summary["passed"] = True
    except BaseException as error:
        summary["error"] = repr(error)
        raise
    finally:
        if audio:
            audio.close()
        if touched and dev and original:
            try:
                for binding in original:
                    client.set_binding(disabled(binding))
                client.save()
                summary["cleanup_restore"], _ = wait_save(client)
            except Exception as error:
                summary["restore_error"] = repr(error)
        if dev:
            dev.close()
        time.sleep(.1)
        stop.set()
        reader.join(2)
        uart.close()
        summary["uart_faults"] = [s for s in ("Guru Meditation", "assert failed", "stack overflow", "Task watchdog got triggered", "abort() was called")
                                  if s.encode() in log]
        (args.output / "uart.log").write_bytes(log)
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        if summary["uart_faults"]:
            raise RuntimeError(f"UART faults: {summary['uart_faults']}")
    print(json.dumps({key: summary[key] for key in ("reproduced", "passed") if key in summary}), flush=True)


if __name__ == "__main__":
    main()
