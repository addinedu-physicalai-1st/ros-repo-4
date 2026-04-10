#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2에서 ``gps_field_msgs/PinkyGps`` 를 **현재 프로세스의** ``ROS_DOMAIN_ID`` 에 맞는
토픽(기본 ``/pinky_{domain}/gps_pos``)으로 발행하는 최소 예제입니다.

**도메인 격리 테스트 (요청하신 시나리오)**

- 이 스크립트: ``export ROS_DOMAIN_ID=22`` 후 실행 → 토픽은 ``/pinky_22/gps_pos``
- 핑키 LCD/구독: ``ROS_DOMAIN_ID=23`` 으로 실행 → ``/pinky_23/gps_pos`` 만 구독
- → 22번 도메인에서 아무리 쏴도 **23번 LCD 숫자는 바뀌지 않는 것이 정상**입니다.

실행::

    source /opt/ros/<distro>/setup.bash
    source ~/dalimi/GPSTopicTest/install/setup.bash   # 또는 ~/GPSTopicTest/install
    export ROS_DOMAIN_ID=22
    python3 tools/ros2_pinky_gps_publish_domain22_demo.py --rate 2

다른 터미널에서 구독 확인(22번만 보임)::

    export ROS_DOMAIN_ID=22
    ros2 topic echo /pinky_22/gps_pos
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

try:
    from gps_field_msgs.msg import PinkyGps
except ImportError as e:
    raise SystemExit(
        "gps_field_msgs 를 찾을 수 없습니다. "
        "GPSTopicTest 를 colcon build 후 install/setup.bash 를 source 하세요.\n"
        f"원인: {e}"
    ) from e


def resolve_gps_topic(explicit_topic: str, pattern: str) -> str:
    exp = (explicit_topic or "").strip()
    if exp:
        return exp if exp.startswith("/") else f"/{exp}"
    raw = os.environ.get("ROS_DOMAIN_ID", "0")
    try:
        domain_id = int(raw)
    except ValueError:
        domain_id = 0
    t = pattern.format(domain_id=domain_id)
    return t if t.startswith("/") else f"/{t}"


class DomainDemoPublisher(Node):
    def __init__(self, rate_hz: float = 2.0, pinky_id: int = 0) -> None:
        super().__init__("pinky_gps_domain_demo_publisher")
        self.declare_parameter("topic_name", "")
        self.declare_parameter("topic_pattern", "/pinky_{domain_id}/gps_pos")
        self.declare_parameter("publish_rate_hz", float(max(0.1, rate_hz)))
        self.declare_parameter("field_width_mm", 1880.0)
        self.declare_parameter("field_height_mm", 1410.0)
        self.declare_parameter("frame_id", "field_1880x1410")
        self.declare_parameter("pinky_id", int(pinky_id) & 0xFF)

        explicit = self.get_parameter("topic_name").get_parameter_value().string_value
        pattern = self.get_parameter("topic_pattern").get_parameter_value().string_value
        topic = resolve_gps_topic(explicit, pattern)
        rate_hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self._w = float(self.get_parameter("field_width_mm").get_parameter_value().double_value)
        self._h = float(self.get_parameter("field_height_mm").get_parameter_value().double_value)
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._pinky_id = int(self.get_parameter("pinky_id").get_parameter_value().integer_value) & 0xFF

        self._pub = self.create_publisher(PinkyGps, topic, 10)
        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)
        self._t = 0.0

        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        self.get_logger().info(
            f"ROS_DOMAIN_ID={domain} → topic={topic}  "
            f"(도메인 23 LCD는 이 토픽을 구독하지 않으면 값이 변하지 않아야 함)"
        )

    def _tick(self) -> None:
        self._t += 0.2
        x = self._w * 0.5 + (self._w * 0.45) * math.sin(self._t)
        y = self._h * 0.5 + (self._h * 0.45) * math.cos(self._t * 0.9)
        x = max(0.0, min(self._w, x))
        y = max(0.0, min(self._h, y))
        yaw = math.degrees(math.sin(self._t * 0.5)) * 45.0

        msg = PinkyGps()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pinky_id = self._pinky_id
        msg.x_mm = float(x)
        msg.y_mm = float(y)
        msg.yaw_deg = float(yaw)
        self._pub.publish(msg)
        self.get_logger().info(
            f"publish pinky_id={msg.pinky_id} x_mm={x:.1f} y_mm={y:.1f} yaw_deg={yaw:.1f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--rate", type=float, default=2.0, help="발행 Hz")
    parser.add_argument("--pinky-id", type=int, default=0, help="메시지 pinky_id (0–255)")
    args, ros_argv = parser.parse_known_args()

    rclpy.init(args=ros_argv)
    node = DomainDemoPublisher(rate_hz=args.rate, pinky_id=args.pinky_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
