#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Find I2C OLED address and verify luma can draw (no ROS).

  ros2 run gps_field_subscriber field_oled_selftest

Typical SSD1306: 0x3C / 0x3D. Other addresses on the bus are often IMU, PMIC, etc.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from typing import List, Set


def _run_i2cdetect(port: int) -> str:
    print(f"\n--- i2cdetect -y {port} ---")
    try:
        r = subprocess.run(
            ["i2cdetect", "-y", str(port)],
            capture_output=True,
            text=True,
            timeout=5,
        )
        out = r.stdout or r.stderr or ""
        print(out or "(no output)")
        return out
    except FileNotFoundError:
        print("i2cdetect not found. Install: sudo apt install i2c-tools")
        return ""
    except Exception as e:
        print(e)
        return ""


def _addresses_from_i2cdetect_text(text: str) -> Set[int]:
    """Parse i2cdetect table; only rows where token index matches column (works for row 60+)."""
    found: Set[int] = set()
    for line in text.splitlines():
        line = line.strip()
        m = re.match(r"^([0-9a-f]{2}):", line)
        if not m:
            continue
        base = int(m.group(1), 16)
        tokens = line.split()[1:]
        if len(tokens) > 16:
            tokens = tokens[:16]
        for col, tok in enumerate(tokens):
            if tok in ("--", "UU"):
                continue
            if len(tok) == 2:
                try:
                    int(tok, 16)
                except ValueError:
                    continue
                found.add(base + col)
    return found


def _scan_smbus(port: int) -> List[int]:
    """List devices that accept a 1-byte read (same idea as i2cdetect)."""
    try:
        import smbus2
    except ImportError:
        return []
    found: List[int] = []
    bus = smbus2.SMBus(port)
    for addr in range(0x08, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    return found


def _try_draw(port: int, addr: int, device: str, width: int, height: int) -> bool:
    try:
        from gps_field_subscriber.oled_display import OledI2cDisplay
    except ImportError:
        print("Import failed. Source install/setup.bash or run from package path.")
        return False
    try:
        d = OledI2cDisplay(
            i2c_port=port,
            i2c_address=addr,
            device_name=device,
            width=width,
            height=height,
            contrast=255,
        )
        d.show_lines(
            [
                f"OK 0x{addr:02X}",
                f"{device}",
                f"{width}x{height}",
                "GPS OLED test",
            ]
        )
        print(f"  Draw OK at 0x{addr:02X} ({device})")
        return True
    except Exception as e:
        print(f"  0x{addr:02X} ({device}): {e}")
        return False


def main() -> int:
    p = argparse.ArgumentParser(description="I2C OLED self-test for Pinky GPS subscriber")
    p.add_argument("--port", type=int, default=1, help="I2C bus (Pi 4/5 usually 1)")
    p.add_argument(
        "--addresses",
        type=str,
        default="",
        help="Extra comma-separated decimal addresses (e.g. 107). Empty = auto from bus scan",
    )
    p.add_argument("--device", choices=("ssd1306", "sh1106", "both"), default="both")
    p.add_argument("--width", type=int, default=128)
    p.add_argument("--height", type=int, default=64)
    p.add_argument("--no-scan", action="store_true", help="Skip i2cdetect print only")
    p.add_argument(
        "--port0-also",
        action="store_true",
        help="Also run i2cdetect -y 0 (some HATs use bus 0)",
    )
    args = p.parse_args()

    to_try: List[int] = [0x3C, 0x3D]
    extra: List[int] = []
    for part in args.addresses.split(","):
        part = part.strip()
        if part:
            extra.append(int(part, 0) if part.startswith("0x") else int(part))

    if not args.no_scan:
        txt = _run_i2cdetect(args.port)
        parsed = _addresses_from_i2cdetect_text(txt)
        smbus_found = set(_scan_smbus(args.port))
        merged = parsed | smbus_found | set(to_try) | set(extra)
        to_try = sorted(merged)
        print(
            f"\nAddresses to probe (merge i2cdetect parse + smbus scan + 3C/3D + extras): "
            f"{', '.join(f'0x{a:02X}' for a in to_try)}"
        )
        if args.port0_also:
            t0 = _run_i2cdetect(0)
            to_try = sorted(
                set(to_try)
                | _addresses_from_i2cdetect_text(t0)
                | set(_scan_smbus(0))
            )
        if not any(a in (0x3C, 0x3D) for a in parsed | smbus_found):
            print(
                "\n*** No 0x3C / 0x3D on this bus. Typical I2C OLED uses those. ***"
                "\n    0x6B is often an IMU or other sensor, not SSD1306."
                "\n    If the front panel never shows 3c/3d, it may be SPI/DSI/HDMI, not this I2C driver."
            )
    else:
        to_try = sorted(set(to_try) | set(extra))

    print("\nTrying luma draw (ssd1306 and/or sh1106)...")
    ok_any = False
    drivers = (
        ("ssd1306", "sh1106") if args.device == "both" else (args.device,)
    )
    for a in to_try:
        for drv in drivers:
            if _try_draw(args.port, a, drv, args.width, args.height):
                ok_any = True
                break

    if not ok_any:
        print(
            "\nNo SSD1306/SH1106 responded. Next steps:"
            "\n  - Confirm the front display is I2C SSD1306/SH1106 (not SPI)."
            "\n  - Try: --port 0   or   ros2 run ... field_oled_selftest -- --port0-also"
            "\n  - 128x32 module: --height 32"
            "\n  - Other I2C bus:  i2cdetect -y 0 / -y 13 (CM4) etc."
        )
        return 1

    print(
        "\nUse the working address with field_gps_subscriber, e.g."
        "\n  -p oled_i2c_address:=<decimal>   (60 = 0x3C)"
        "\n  -p oled_device:=ssd1306   or   sh1106"
        f"\n  -p oled_i2c_port:={args.port}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
