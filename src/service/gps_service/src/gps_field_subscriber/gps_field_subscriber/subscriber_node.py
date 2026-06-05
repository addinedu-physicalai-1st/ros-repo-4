#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Subscribe /pinky_{pinky_id}/gps_pos (default); Tk / OLED / SPI LCD.

NOTE: DDS 네트워크 분리는 ROS_DOMAIN_ID로 하고, 토픽의 <id>는 pinky_id를 사용한다.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Optional

import rclpy
from rclpy.node import Node

from gps_field_msgs.msg import PinkyGps
from gps_field_subscriber.ros_domain_topic import resolve_gps_topic, ros_domain_id_str

if TYPE_CHECKING:
    from gps_field_subscriber.oled_display import OledI2cDisplay
    from gps_field_subscriber.pinky_spi_lcd import PinkySpiLcdPanel


def _try_tk():
    import tkinter as tk
    from tkinter import font as tkfont

    return tk, tkfont


class LcdWindow:
    """Tk window; works on X11 (e.g. local display or ssh -X)."""

    def __init__(
        self,
        field_w: float,
        field_h: float,
        domain_label: str,
        topic_str: str,
        filter_id: int,
    ) -> None:
        tk, tkfont = _try_tk()
        self._tk = tk
        self.root = tk.Tk()
        self.root.title(
            f"Pinky GPS — PINKY_ID={filter_id} (field {field_w:.0f}x{field_h:.0f} mm)"
        )
        self.root.geometry("520x280")
        self.root.configure(bg="#0a1628")

        title_font = tkfont.Font(family="DejaVu Sans", size=16, weight="bold")
        body_font = tkfont.Font(family="DejaVu Sans Mono", size=18)

        if filter_id >= 0:
            sub_line = f"Filter: pinky_id == {filter_id}"
        else:
            sub_line = "No pinky_id filter (single-robot topic)"
        self._lbl_title = tk.Label(
            self.root,
            text=f"{sub_line}\n{topic_str}",
            font=title_font,
            fg="#7ecbff",
            bg="#0a1628",
            justify="left",
        )
        self._lbl_title.pack(pady=(16, 8))

        self._lbl_coords = tk.Label(
            self.root,
            text="Waiting (no messages yet)\nx_mm: -\ny_mm: -\nyaw_deg: -",
            font=body_font,
            fg="#e8f4ff",
            bg="#102240",
            justify="left",
            padx=24,
            pady=20,
        )
        self._lbl_coords.pack(fill="both", expand=True, padx=12, pady=12)

        self._lbl_meta = tk.Label(
            self.root,
            text=(
                f"PINKY_ID={filter_id}\n"
                f"topic: {topic_str}"
            ),
            font=tkfont.Font(family="DejaVu Sans", size=10),
            fg="#6a8aaa",
            bg="#0a1628",
        )
        self._lbl_meta.pack(pady=(0, 10))

    def set_coords(
        self,
        x_mm: float,
        y_mm: float,
        yaw_deg: float,
        stamp_sec: int,
        stamp_nsec: int,
    ) -> None:
        text = (
            f"x_mm:   {x_mm:8.1f}\n"
            f"y_mm:   {y_mm:8.1f}\n"
            f"yaw:    {yaw_deg:8.1f} deg\n"
            f"stamp:  {stamp_sec}.{stamp_nsec:09d}"
        )
        self._lbl_coords.config(text=text)

    def run(self) -> None:
        self.root.mainloop()

    def close(self) -> None:
        try:
            self.root.destroy()
        except Exception:
            pass


class FieldGpsSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("field_gps_subscriber")

        self.declare_parameter("topic_name", "")
        self.declare_parameter("topic_pattern", "/pinky_{pinky_id}/gps_pos")
        # >= 0: only show messages with this pinky_id; -1: show all on this topic
        self.declare_parameter("pinky_id", -1)
        self.declare_parameter("field_width_mm", 1880.0)
        self.declare_parameter("field_height_mm", 1410.0)
        self.declare_parameter("show_tk", True)
        self.declare_parameter("show_oled", False)
        self.declare_parameter("oled_i2c_port", 1)
        self.declare_parameter("oled_i2c_address", 60)
        self.declare_parameter("oled_device", "ssd1306")
        self.declare_parameter("oled_rotate", 0)
        self.declare_parameter("oled_width", 128)
        self.declare_parameter("oled_height", 64)
        self.declare_parameter("oled_contrast", 255)
        self.declare_parameter("show_pinky_spi", False)
        self.declare_parameter("pinky_lcd_img_width", 320)
        self.declare_parameter("pinky_lcd_img_height", 240)

        explicit_topic = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )
        topic_pattern = (
            self.get_parameter("topic_pattern").get_parameter_value().string_value
        )
        self._filter_id = int(
            self.get_parameter("pinky_id").get_parameter_value().integer_value
        )
        self._filter_by_pinky_id = self._filter_id >= 0
        if self._filter_id < 0 and not (explicit_topic or "").strip():
            self.get_logger().warning(
                "pinky_id=-1 and topic_name is empty; "
                "set pinky_id (e.g. 24) or pass topic_name explicitly."
            )
            topic = "/pinky_0/gps_pos"
        else:
            pid_for_topic = self._filter_id if self._filter_id >= 0 else 0
            topic = resolve_gps_topic(
                explicit_topic,
                topic_pattern,
                pinky_id=pid_for_topic,
                logger=self.get_logger(),
            )
        domain_label = ros_domain_id_str()
        fw = self.get_parameter("field_width_mm").get_parameter_value().double_value
        fh = self.get_parameter("field_height_mm").get_parameter_value().double_value
        show_tk = self.get_parameter("show_tk").get_parameter_value().bool_value
        show_oled = self.get_parameter("show_oled").get_parameter_value().bool_value
        oled_port = int(
            self.get_parameter("oled_i2c_port").get_parameter_value().integer_value
        )
        oled_addr = int(
            self.get_parameter("oled_i2c_address").get_parameter_value().integer_value
        )
        oled_dev = self.get_parameter("oled_device").get_parameter_value().string_value
        oled_rot = int(
            self.get_parameter("oled_rotate").get_parameter_value().integer_value
        )
        oled_w = int(
            self.get_parameter("oled_width").get_parameter_value().integer_value
        )
        oled_h = int(
            self.get_parameter("oled_height").get_parameter_value().integer_value
        )
        oled_contrast = int(
            self.get_parameter("oled_contrast").get_parameter_value().integer_value
        )
        show_pinky_spi = self.get_parameter("show_pinky_spi").get_parameter_value().bool_value
        plcd_w = int(
            self.get_parameter("pinky_lcd_img_width").get_parameter_value().integer_value
        )
        plcd_h = int(
            self.get_parameter("pinky_lcd_img_height").get_parameter_value().integer_value
        )

        self._lcd: Optional[LcdWindow] = None
        self._oled: Optional[OledI2cDisplay] = None
        self._pinky_spi: Optional[PinkySpiLcdPanel] = None

        if show_tk:
            try:
                self._lcd = LcdWindow(fw, fh, domain_label, topic, self._filter_id)
            except Exception as e:
                self.get_logger().error(f"Tk window failed: {e}")
                if not show_oled and not show_pinky_spi:
                    raise

        if show_oled:
            from gps_field_subscriber.oled_display import OledI2cDisplay

            try:
                self._oled = OledI2cDisplay(
                    i2c_port=oled_port,
                    i2c_address=oled_addr,
                    device_name=oled_dev,
                    rotate=oled_rot,
                    width=oled_w,
                    height=oled_h,
                    contrast=oled_contrast,
                )
                self._oled.show_lines(
                    [
                        f"pinky_id {self._filter_id}",
                        "Waiting...",
                    ]
                )
                self.get_logger().info(
                    f"OLED: port={oled_port} addr=0x{oled_addr:02X} device={oled_dev} "
                    f"{oled_w}x{oled_h} contrast={oled_contrast}. "
                    "If screen stays blank, run: ros2 run gps_field_subscriber field_oled_selftest"
                )
            except Exception as e:
                self.get_logger().error(f"OLED init failed: {e}")
                if self._lcd is None and not show_pinky_spi:
                    raise RuntimeError("No Tk, OLED failed, and Pinky SPI disabled") from e

        if show_pinky_spi:
            from gps_field_subscriber.pinky_spi_lcd import PinkySpiLcdPanel

            try:
                self._pinky_spi = PinkySpiLcdPanel(
                    f"pinky_id {self._filter_id}", img_size=(plcd_w, plcd_h)
                )
                self.get_logger().info(
                    f"Pinky SPI LCD (pinky_lcd) {plcd_w}x{plcd_h} — same as notebook LCD()"
                )
            except Exception as e:
                self.get_logger().error(f"Pinky SPI LCD init failed: {e}")
                if self._lcd is None and self._oled is None:
                    raise RuntimeError(
                        "Pinky SPI LCD required but init failed (need GPIO/SPI, pinky_lcd)"
                    ) from e

        if self._lcd is None and self._oled is None and self._pinky_spi is None:
            raise RuntimeError(
                "Enable at least one: show_tk, show_oled, or show_pinky_spi"
            )

        self._sub = self.create_subscription(PinkyGps, topic, self._on_gps, 10)
        self.get_logger().info(
            f"Subscribe topic={topic} pinky_id_filter={self._filter_id} "
            f"(>=0 filter, -1 accept all) "
            f"tk={self._lcd is not None} oled={self._oled is not None} "
            f"pinky_spi={self._pinky_spi is not None}"
        )

    def _on_gps(self, msg: PinkyGps) -> None:
        if self._filter_by_pinky_id and int(msg.pinky_id) != self._filter_id:
            return

        x = float(msg.x_mm)
        y = float(msg.y_mm)
        yaw = float(msg.yaw_deg)
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)

        if self._oled is not None:
            lines = [
                f"id {int(msg.pinky_id)}",
                f"x {x:.0f} mm",
                f"y {y:.0f} mm",
                f"yaw {yaw:.0f} deg",
                f"t {sec}",
            ]
            try:
                self._oled.show_lines(lines)
            except Exception as e:
                self.get_logger().warning(f"OLED draw failed: {e}")

        if self._pinky_spi is not None:
            try:
                self._pinky_spi.show_gps(x, y, yaw, sec, nsec)
            except Exception as e:
                self.get_logger().warning(f"Pinky SPI LCD draw failed: {e}")

        if self._lcd is not None:

            def ui_update() -> None:
                self._lcd.set_coords(x, y, yaw, sec, nsec)

            try:
                self._lcd.root.after(0, ui_update)
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = FieldGpsSubscriber()
    except Exception as e:
        print("field_gps_subscriber failed to start:", e)
        rclpy.shutdown()
        return

    if node._lcd is not None:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        def on_closing() -> None:
            node.destroy_node()
            rclpy.shutdown()
            node._lcd.close()
            if node._pinky_spi is not None:
                node._pinky_spi.close()

        node._lcd.root.protocol("WM_DELETE_WINDOW", on_closing)
        try:
            node._lcd.run()
        except KeyboardInterrupt:
            on_closing()
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if node._pinky_spi is not None:
                node._pinky_spi.close()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
