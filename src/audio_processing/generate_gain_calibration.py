"""Generate firmware float gains from the local array calibration JSON."""

import argparse
import json
import math
from pathlib import Path
import struct


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number in dB")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def generate(source, destination):
    config = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("Gain calibration must be a JSON object")
    common = finite_number(config.get("common_base_gain_db"), "common_base_gain_db")
    trims = config.get("channel_trim_gain_db")
    if not isinstance(trims, list) or len(trims) != 8:
        raise ValueError("channel_trim_gain_db must contain exactly 8 values")
    gains = []
    for index, trim in enumerate(trims):
        trim = finite_number(trim, f"channel_trim_gain_db[{index}]")
        # Match Python GainCalibration's float32 coefficients without NumPy.
        gain = struct.unpack("f", struct.pack("f", 10.0 ** ((common + trim) / 20.0)))[0]
        if not math.isfinite(gain) or gain <= 0:
            raise ValueError(f"Channel {index} gain is outside positive float32 range")
        gains.append(f"{gain:.9e}f")
    text = (
        "// Generated from src/audio_processing/gain_cali.json. Do not edit.\n"
        '#pragma once\n#include "array_dsp.hpp"\n\n'
        "namespace array_dsp {\n"
        f"inline constexpr float common_input_gain_db = {common:.9e}f;\n"
        "inline constexpr ChannelGains calibrated_channel_gains{\n    "
        + ",\n    ".join(gains)
        + "\n};\n} // namespace array_dsp\n"
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or destination.read_text(encoding="utf-8") != text:
        destination.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    generate(args.source, args.destination)
