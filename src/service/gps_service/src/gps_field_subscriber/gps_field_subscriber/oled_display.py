# -*- coding: utf-8 -*-
"""128x64 (or similar) I2C OLED via luma.oled (SSD1306 / SH1106)."""

from __future__ import annotations

import threading
from typing import List

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306, sh1106
    from PIL import ImageFont

    _LUMA_AVAILABLE = True
except ImportError:
    _LUMA_AVAILABLE = False


class OledI2cDisplay:
    """Thread-safe draw of text lines on a small OLED."""

    def __init__(
        self,
        i2c_port: int = 1,
        i2c_address: int = 0x3C,
        device_name: str = "ssd1306",
        rotate: int = 0,
        width: int = 128,
        height: int = 64,
        contrast: int = 0,
    ) -> None:
        if not _LUMA_AVAILABLE:
            raise RuntimeError(
                "luma.oled not installed. On the Pi: pip install luma.oled "
                "(use same Python as ROS: e.g. python3 -m pip install --user luma.oled)"
            )
        serial = i2c(port=i2c_port, address=i2c_address)
        name = device_name.lower().strip()
        if name == "sh1106":
            self._dev = sh1106(serial, width=width, height=height, rotate=rotate)
        else:
            self._dev = ssd1306(serial, width=width, height=height, rotate=rotate)
        if contrast > 0:
            try:
                self._dev.contrast(contrast)
            except Exception:
                pass
        self._lock = threading.Lock()
        self._font = self._load_font()

    def _load_font(self):
        for path, size in (
            ("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11),
            ("/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf", 11),
        ):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
        return ImageFont.load_default()

    def show_lines(self, lines: List[str]) -> None:
        """Draw up to ~6 lines; strings truncated to fit width."""
        with self._lock:
            with canvas(self._dev) as draw:
                draw.rectangle(self._dev.bounding_box, outline="black", fill="black")
                y = 0
                line_step = 12
                for raw in lines[:6]:
                    text = raw.replace("\n", " ")
                    if len(text) > 21:
                        text = text[:20] + "."
                    draw.text((0, y), text, font=self._font, fill="white")
                    y += line_step

    @staticmethod
    def luma_available() -> bool:
        return _LUMA_AVAILABLE
