# -*- coding: utf-8 -*-
"""Pinky front SPI LCD via ``pinky_lcd.LCD`` (same as notebook: PIL Image + img_show)."""

from __future__ import annotations

import threading
from typing import Optional, Tuple

try:
    from PIL import Image, ImageDraw, ImageFont

    _PIL_OK = True
except ImportError:
    _PIL_OK = False


class PinkySpiLcdPanel:
    """Thread-safe updates on the SPI TFT (requires ``pip install pinky_lcd`` on device)."""

    def __init__(
        self,
        header_label: str,
        img_size: Tuple[int, int] = (320, 240),
    ) -> None:
        if not _PIL_OK:
            raise RuntimeError("PIL required for Pinky SPI LCD")
        try:
            from pinky_lcd import LCD
        except ImportError as e:
            raise RuntimeError(
                "pinky_lcd not importable. Install the Pinky SPI LCD package "
                "(e.g. from pinkylib/lcd or pip)."
            ) from e
        self._header_label = header_label
        self._img_w, self._img_h = img_size
        self._lcd = LCD()
        self._lock = threading.Lock()
        self._load_fonts()
        self.show_waiting()

    def _load_fonts(self) -> None:
        try:
            self._font_title = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24
            )
            self._font_body = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 20
            )
        except OSError:
            self._font_title = ImageFont.load_default()
            self._font_body = self._font_title

    def _new_image(self, bg: Tuple[int, int, int] = (15, 25, 45)) -> Image.Image:
        return Image.new("RGB", (self._img_w, self._img_h), color=bg)

    def show_waiting(self) -> None:
        img = self._new_image()
        dr = ImageDraw.Draw(img)
        dr.text(
            (12, 20),
            f"Pinky GPS  {self._header_label}",
            fill=(0, 220, 180),
            font=self._font_title,
        )
        dr.text(
            (12, 70),
            "Waiting for gps_pos...",
            fill=(200, 220, 255),
            font=self._font_body,
        )
        with self._lock:
            self._lcd.img_show(img)

    def show_gps(
        self,
        x_mm: float,
        y_mm: float,
        yaw_deg: float,
        stamp_sec: int,
        stamp_nsec: int,
    ) -> None:
        img = self._new_image()
        dr = ImageDraw.Draw(img)
        dr.text(
            (12, 8),
            f"Pinky GPS  {self._header_label}",
            fill=(0, 255, 200),
            font=self._font_title,
        )
        lines = [
            f"x_mm   {x_mm:8.1f}",
            f"y_mm   {y_mm:8.1f}",
            f"yaw    {yaw_deg:8.1f} deg",
            f"stamp  {stamp_sec}.{stamp_nsec:09d}",
        ]
        y0 = 55
        for line in lines:
            dr.text((12, y0), line, fill=(230, 240, 255), font=self._font_body)
            y0 += 36
        with self._lock:
            self._lcd.img_show(img)

    def close(self) -> None:
        with self._lock:
            try:
                self._lcd.close()
            except Exception:
                pass
