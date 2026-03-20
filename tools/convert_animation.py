#!/usr/bin/env python3
"""
convert_animation.py
====================
Converts the original fixed-timestep stream animation (animations.h, 8 bytes/frame
at LOOPDELAY ms/step) into the new event-based delta format and writes
include/animation_data.h.

Usage:
    python3 tools/convert_animation.py \
        --input  animations_original.h \
        --output include/animation_data.h \
        --timestep 50

The script can also be imported as a module; call convert(frames, timestep_ms).
"""

import argparse
import re
import sys
from pathlib import Path


CHANNELS = 8
END_MARKER = 0xFFFF


def parse_stream(header_text: str) -> list[list[int]]:
    """Extract the flat byte list from the PROGMEM array in the .h file."""
    # grab everything between { and };
    m = re.search(r'=\s*\{([^}]+)\}', header_text, re.DOTALL)
    if not m:
        raise ValueError("Could not find array data in header file")
    numbers = [int(x.strip()) for x in m.group(1).split(',') if x.strip().isdigit()]
    if len(numbers) % CHANNELS != 0:
        raise ValueError(f"Data length {len(numbers)} is not a multiple of {CHANNELS}")
    return [numbers[i:i+CHANNELS] for i in range(0, len(numbers), CHANNELS)]


def convert(frames: list[list[int]], timestep_ms: int) -> bytes:
    """
    Convert a list of frames (each a list of CHANNELS byte values) into the
    packed event-based binary format described in Animations.h.

    Returns the raw bytes (suitable for embedding in a C array).
    """
    out = bytearray()
    prev = None
    accumulated_ms = 0

    for frame in frames:
        if prev is None:
            # First frame: emit all channels as a single event at t=0
            mask = 0xFF
            out += (0).to_bytes(2, 'big')   # deltaMs = 0
            out.append(mask)
            out += bytes(frame)
        else:
            changed_mask = 0
            for ch in range(CHANNELS):
                if frame[ch] != prev[ch]:
                    changed_mask |= (1 << ch)

            accumulated_ms += timestep_ms

            if changed_mask:
                delta = min(accumulated_ms, 0xFFFE)   # cap below end marker
                out += delta.to_bytes(2, 'big')
                out.append(changed_mask)
                for ch in range(CHANNELS):
                    if changed_mask & (1 << ch):
                        out.append(frame[ch])
                accumulated_ms = 0

        prev = frame[:]

    # End marker
    out += END_MARKER.to_bytes(2, 'big')
    return bytes(out)


def bytes_to_c_array(data: bytes, name: str = "animation") -> str:
    """Format raw bytes as a PROGMEM C array declaration."""
    lines = []
    lines.append("// AUTO-GENERATED — do not edit by hand.")
    lines.append("// Re-generate with:  python3 tools/convert_animation.py")
    lines.append(f"// Raw size: {len(data)} bytes")
    lines.append("#pragma once")
    lines.append("#include <avr/pgmspace.h>")
    lines.append("")
    lines.append(f"static const uint16_t {name}_size PROGMEM = {len(data)};")
    lines.append(f"static const byte {name}[] PROGMEM = {{")

    ROW = 16
    for i in range(0, len(data), ROW):
        chunk = data[i:i+ROW]
        hex_vals = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"    {hex_vals},")

    lines.append("};")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description="Convert stream animation to event format")
    parser.add_argument("--input",     default="animations_original.h",
                        help="Original animations.h (stream format)")
    parser.add_argument("--output",    default="include/animation_data.h",
                        help="Output header path")
    parser.add_argument("--timestep",  type=int, default=50,
                        help="Original loop delay in ms (default 50)")
    parser.add_argument("--name",      default="animation",
                        help="C symbol name for the array")
    args = parser.parse_args()

    src = Path(args.input).read_text()
    frames = parse_stream(src)
    print(f"Parsed {len(frames)} frames × {CHANNELS} channels "
          f"({len(frames)*CHANNELS} bytes original)", file=sys.stderr)

    data = convert(frames, args.timestep)
    ratio = len(data) / (len(frames) * CHANNELS)
    print(f"Event stream: {len(data)} bytes  ({ratio:.1%} of original)", file=sys.stderr)

    out_text = bytes_to_c_array(data, args.name)
    Path(args.output).write_text(out_text)
    print(f"Written to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
