"""Small HID configuration window for the persistent KWS filter."""
from __future__ import annotations

import argparse
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

from kws_control import Client


def connect(serial):
    import hid  # pip install hidapi

    matches = [item for item in hid.enumerate(0x303A, 0x8003)
               if item.get("usage_page") == 0xFF00 and item.get("usage") == 1
               and (not serial or item.get("serial_number") == serial)]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one control HID, found {len(matches)}; use --serial")
    device = hid.device()
    device.open_path(matches[0]["path"])
    return device


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="device serial number when several boards are connected")
    args = parser.parse_args()

    root = tk.Tk()
    root.title("KWS Filter")
    root.resizable(False, False)
    fields = (
        ("Keyword probability threshold (%)", "threshold", "80"),
        ("Global cooldown after a keyword trigger (ms)", "cooldown", "500"),
        ("Consecutive keyword frames to trigger (10 ms/frame)", "frames", "3"),
    )
    vars_ = {key: tk.StringVar(value=default) for _, key, default in fields}
    vad = tk.BooleanVar(value=False)
    for row, (label, key, _) in enumerate(fields):
        ttk.Label(root, text=label).grid(row=row, column=0, padx=12, pady=7, sticky="w")
        ttk.Entry(root, textvariable=vars_[key], width=12).grid(
            row=row, column=1, padx=12, pady=7, sticky="e")
    ttk.Checkbutton(root, text="Need-VAD-Checked", variable=vad).grid(
        row=3, column=0, columnspan=2, padx=12, pady=7, sticky="w")
    status = tk.StringVar(value="Connecting…")
    ttk.Label(root, textvariable=status).grid(row=5, column=0, columnspan=2,
                                               padx=12, pady=(4, 12), sticky="w")
    responses = queue.Queue()
    busy = False

    def run(operation, values=None):
        nonlocal busy
        if busy:
            return
        busy = True
        refresh_button.configure(state="disabled")
        save_button.configure(state="disabled")
        status.set("Saving to device…" if operation == "save" else "Reading device…")

        def worker():
            device = None
            try:
                device = connect(args.serial)
                client = Client(device)
                result = (client.set_kws_filter(*values)["kws_filter"]
                          if operation == "save" else client.read_object("status")["kws_filter"])
                responses.put((operation, result, None))
            except Exception as exc:
                responses.put((operation, None, str(exc)))
            finally:
                if device is not None:
                    device.close()

        threading.Thread(target=worker, daemon=True).start()

    def save():
        try:
            values = (int(vars_["threshold"].get()), int(vars_["cooldown"].get()),
                      int(vars_["frames"].get()), bool(vad.get()))
            if not (0 <= values[0] <= 100 and 1 <= values[1] <= 60000 and
                    1 <= values[2] <= 1000):
                raise ValueError("Threshold: 0–100; cooldown: 1–60000 ms; frames: 1–1000")
        except ValueError as exc:
            messagebox.showerror("Invalid KWS filter", str(exc), parent=root)
            return
        run("save", values)

    buttons = ttk.Frame(root)
    buttons.grid(row=4, column=0, columnspan=2, pady=6)
    refresh_button = ttk.Button(buttons, text="Read device", command=lambda: run("read"))
    refresh_button.pack(side="left", padx=8)
    save_button = ttk.Button(buttons, text="Save to device", command=save)
    save_button.pack(side="left", padx=8)

    def poll():
        nonlocal busy
        try:
            operation, result, error = responses.get_nowait()
        except queue.Empty:
            pass
        else:
            busy = False
            refresh_button.configure(state="normal")
            save_button.configure(state="normal")
            if error:
                status.set(error)
                messagebox.showerror("KWS Filter", error, parent=root)
            else:
                vars_["threshold"].set(str(result["keyword_threshold_percent"]))
                vars_["cooldown"].set(str(result["cooldown_ms"]))
                vars_["frames"].set(str(result["stable_frames"]))
                vad.set(bool(result["need_vad_checked"]))
                status.set(("Saved to NVS" if operation == "save" else "Read from device") +
                           f" · revision {result['revision']}")
        root.after(100, poll)

    root.after(100, poll)
    run("read")
    root.mainloop()


if __name__ == "__main__":
    main()
