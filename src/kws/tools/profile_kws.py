"""Capture KWS memory/CPU diagnostics, or summarize an existing UART log.

Use the ESP-IDF Python environment for --port/--flash (pyserial, esptool).
Keep USB audio interfaces closed during the baseline so diagnostics are visible.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import sys
import time


PERF = re.compile(r"kws: perf chunks=(\d+) avg=([\d.]+) ms max=([\d.]+) ms RTF=([\d.]+) dropped=(\d+) resets=(\d+) errors=(\d+)")
CPU = re.compile(r"kws: cpu task_avg=([\d.]+) ms task_RTF=([\d.]+) core([01])_share=([\d.]+)% interval=([\d.]+) s")
BEAMFORMER = re.compile(r"audio_dsp: (DAS|MVDR) task CPU=([\d.]+)% runtime=([\d.]+) ms interval=([\d.]+) s")
FAILURES = ("Guru Meditation", "assert failed", "stack overflow", "Task watchdog got triggered")


def summarize(text: str) -> dict:
    text = re.sub(r"\x1b\[[0-9;]*m", "", text)
    rows = []
    memory = []
    for line in text.splitlines():
        if "kws_model:" in line and any(s in line for s in (
                "heap before", "MFCC context=", "tensor internal SRAM budget=", "model variable:",
                "model parameter:", "model parameter_copy:", "tensor regions:")):
            memory.append(line)
        if match := PERF.search(line):
            values = match.groups()
            rows.append(dict(chunks=int(values[0]), wall_avg_ms=float(values[1]),
                             wall_max_ms=float(values[2]), wall_rtf=float(values[3]),
                             dropped_samples=int(values[4]), resets=int(values[5]), errors=int(values[6])))
        elif (match := CPU.search(line)) and rows and "task_rtf" not in rows[-1]:
            values = match.groups()
            core = int(values[2])
            rows[-1].update(task_avg_ms=float(values[0]), task_rtf=float(values[1]),
                            kws_core=core, kws_cpu_pct=float(values[3]), interval_s=float(values[4]))
            rows[-1][f"kws_core{core}_pct"] = float(values[3])
        elif (match := BEAMFORMER.search(line)) and rows and "beamformer_core1_pct" not in rows[-1]:
            rows[-1]["beamformer"] = match[1]
            rows[-1]["beamformer_core1_pct"] = float(match[2])
            # Retain named fields for comparisons with historical captures.
            rows[-1][match[1].lower() + "_core1_pct"] = float(match[2])
    complete = [row for row in rows if "task_rtf" in row and row["chunks"]]
    return dict(memory=memory, windows=rows, paired_windows=len(complete),
                firmware_faults=[token for token in FAILURES if token in text])


def markdown(data: dict) -> str:
    lines = ["# KWS diagnostic capture", "", "## Memory placement", "", "```text"]
    lines.extend(data["memory"] or ["No memory diagnostics captured; capture from reset."])
    lines += ["```", "", "## Windows", "",
              "| Window | Chunks | Wall RTF | Task CPU RTF | KWS CPU % (core) | Beamformer CPU1 % | Dropped total | Resets total |",
              "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for n, row in enumerate(data["windows"], 1):
        lines.append(f"| {n} | {row['chunks']} | {row['wall_rtf']:.3f} | {row.get('task_rtf', 'missing')} | "
                     f"{row.get('kws_cpu_pct', 'missing')} (core{row.get('kws_core', '?')}) | {row.get('beamformer', '')} {row.get('beamformer_core1_pct', 'missing')} | "
                     f"{row['dropped_samples']} | {row['resets']} |")
    lines += ["", "Task CPU RTF measures the whole KWS task, including reset/conversion/queue overhead and failed calls.",
              "It is a window estimate: scheduler counters are committed at context switches, and ISR time may be charged to the task.",
              "The first window includes startup; use subsequent windows for steady-state comparisons.", "",
              "A large wall/task RTF gap indicates scheduling or blocking time. A high task RTF indicates substantial CPU execution cost;",
              "memory placement and CPU/cache contention then require a controlled comparison. Memory location alone does not prove causality.", ""]
    if data["firmware_faults"]:
        lines.append("Firmware faults: " + ", ".join(data["firmware_faults"]))
    if not data["paired_windows"]:
        lines.append("No paired wall/CPU samples. Verify diagnostic firmware, UART access and USB quiet policy.")
    return "\n".join(lines) + "\n"


def flash(build: Path, port: str, before: str) -> None:
    config = json.loads((build / "flasher_args.json").read_text(encoding="utf-8"))
    if config["extra_esptool_args"]["chip"] != "esp32p4":
        raise ValueError("Expected an ESP32-P4 build")
    app = config["app"]
    image = build / app["file"]
    if not image.is_file():
        raise FileNotFoundError(image)
    # esptool verifies the connected chip type before writing. Only update the
    # application; this diagnostic changes neither bootloader nor partition map.
    command = [sys.executable, "-m", "esptool", "--chip", "esp32p4", "--port", port,
               "--baud", "460800", "--before", before, "--after", "watchdog-reset", "write-flash",
               *config["write_flash_args"], app["offset"], str(image)]
    subprocess.run(command, check=True)


def capture(port: str, seconds: float, path: Path, reset: bool = True) -> str:
    import serial
    from esptool.reset import HardReset

    uart = serial.Serial()
    uart.port, uart.baudrate, uart.timeout = port, 115200, 0.25
    uart.dtr = uart.rts = False
    try:
        uart.open()
        # Keep the UART open through reset, preserving startup memory records.
        if reset:
            HardReset(uart)()
        print(f"Capturing {port} for {seconds:g}s to {path}", flush=True)
        end = time.monotonic() + seconds
        with path.open("wb") as output:
            while time.monotonic() < end:
                block = uart.read(4096)
                if block:
                    output.write(block)
                    output.flush()
                    print(block.decode("utf-8", errors="replace"), end="", flush=True)
    finally:
        if uart.is_open:
            uart.close()
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> None:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument("--input-log", type=Path)
    parser.add_argument("--flash", action="store_true")
    parser.add_argument("--before", choices=("default-reset", "no-reset"), default="default-reset",
                        help="Use no-reset after manually entering BOOT/download mode")
    parser.add_argument("--build", type=Path, default=root / "build")
    parser.add_argument("--seconds", type=float, default=65)
    parser.add_argument("--output", type=Path, default=root / ".codex-tmp" / f"kws-profile-{datetime.now():%Y%m%d-%H%M%S}")
    args = parser.parse_args()
    if args.flash and not args.port:
        parser.error("--flash requires --port")
    if args.seconds < 1:
        parser.error("--seconds must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.input_log:
        text = args.input_log.read_text(encoding="utf-8", errors="replace")
    else:
        # Fail before touching flash if another monitor owns the UART.
        import serial
        uart = serial.Serial()
        uart.port, uart.baudrate = args.port, 115200
        uart.dtr = uart.rts = False
        uart.open()
        uart.close()
        if args.flash:
            flash(args.build, args.port, args.before)
        # Flash uses a stub watchdog reset, including adapters without RTS/EN
        # wiring. Boot's PSRAM/AFE init precedes KWS memory logs; open promptly.
        text = capture(args.port, args.seconds, args.output.with_suffix(".log"), reset=not args.flash)
    result = summarize(text)
    args.output.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    args.output.with_suffix(".md").write_text(markdown(result), encoding="utf-8")
    print(f"\nPaired windows: {result['paired_windows']}; report: {args.output.with_suffix('.md')}")
    if result["firmware_faults"] or not result["paired_windows"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
