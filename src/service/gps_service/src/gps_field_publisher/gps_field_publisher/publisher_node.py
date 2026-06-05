#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish PinkyGps on /pinky_{ROS_DOMAIN_ID}/gps_pos by default (sim)."""

import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from gps_field_msgs.msg import PinkyGps

from gps_field_publisher.ros_domain_topic import resolve_gps_topic


class FieldGpsPublisher(Node):
    def __init__(self) -> None:
        super().__init__("field_gps_publisher")
        self.declare_parameter("topic_name", "")
        self.declare_parameter("topic_pattern", "/pinky_{domain_id}/gps_pos")
        self.declare_parameter("field_width_mm", 1880.0)
        self.declare_parameter("field_height_mm", 1410.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("pinky_ids", [])
        self.declare_parameter("frame_id", "field_1880x1410")

        explicit = self.get_parameter("topic_name").get_parameter_value().string_value
        pattern = self.get_parameter("topic_pattern").get_parameter_value().string_value
        topic = resolve_gps_topic(explicit, pattern, self.get_logger())
        self._w = self.get_parameter("field_width_mm").get_parameter_value().double_value
        self._h = self.get_parameter("field_height_mm").get_parameter_value().double_value
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        pv = self.get_parameter("pinky_ids").get_parameter_value()
        if pv.integer_array_value:
            self._ids = list(pv.integer_array_value)
        else:
            try:
                d = int(os.environ.get("ROS_DOMAIN_ID", "0")) & 0xFF
            except ValueError:
                d = 0
            self._ids = [d]

        self._pub = self.create_publisher(PinkyGps, topic, 10)
        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)
        self._t = 0.0

        self.get_logger().info(
            f"Publishing topic={topic} field={self._w}x{self._h} mm pinky_ids={self._ids} "
            f"rate={rate_hz} Hz"
        )

    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _tick(self) -> None:
        self._t += 0.15
        stamp = self.get_clock().now().to_msg()
        for i, pid in enumerate(self._ids):
            # Simple sim: elliptic path per ID inside field
            ph = float(i) * 1.7
            x = self._w * 0.5 + (self._w * 0.5 - 80.0) * math.sin(self._t * 0.4 + ph)
            y = self._h * 0.5 + (self._h * 0.5 - 80.0) * math.cos(self._t * 0.35 + ph * 0.8)
            x = self._clamp(x, 0.0, self._w)
            y = self._clamp(y, 0.0, self._h)
            yaw = math.degrees(math.atan2(
                (self._h * 0.5 - y) * 0.01,
                (x - self._w * 0.5) * 0.01,
            ))

            msg = PinkyGps()
            msg.header = Header()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.pinky_id = int(pid) & 0xFF
            msg.x_mm = x
            msg.y_mm = y
            msg.yaw_deg = float(yaw)
            # New fields (backward compatible defaults for sim publisher)
            msg.confidence = 100
            msg.is_valid = True
            self._pub.publish(msg)
            self.get_logger().debug(
                f"pinky_id={msg.pinky_id} x={x:.1f} y={y:.1f} yaw={yaw:.1f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
