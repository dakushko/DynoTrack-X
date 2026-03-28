#!/usr/bin/env python3
"""Regenerate src/logo_png.h from assets/logo.png (run from project root)."""
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
PNG = ROOT / "assets" / "logo.png"
OUT = ROOT / "src" / "logo_png.h"

if __name__ == "__main__":
    b = PNG.read_bytes()
    lines = [
        "#pragma once",
        "#include <Arduino.h>",
        "",
        "static const uint8_t kLogoPng[] PROGMEM = {",
    ]
    for i in range(0, len(b), 16):
        chunk = b[i : i + 16]
        lines.append("  " + ", ".join(f"0x{x:02x}" for x in chunk) + ",")
    lines.append("};")
    lines.append("")
    lines.append(f"static const size_t kLogoPngLen = {len(b)};")
    lines.append("")
    OUT.write_text("\n".join(lines))
    print(f"Wrote {OUT} ({len(b)} bytes from {PNG})")
