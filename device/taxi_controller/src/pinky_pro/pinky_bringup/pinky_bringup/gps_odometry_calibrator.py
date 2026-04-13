#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from gps_field_msgs.msg import PinkyGps


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def pose_to_2d(position, orientation) -> Pose2D:
    _, _, yaw = euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ])
    return Pose2D(position.x, position.y, yaw)


def compose_pose(a: Pose2D, b: Pose2D) -> Pose2D:
    cos_yaw = math.cos(a.yaw)
    sin_yaw = math.sin(a.yaw)
    return Pose2D(
        x=a.x + cos_yaw * b.x - sin_yaw * b.y,
        y=a.y + sin_yaw * b.x + cos_yaw * b.y,
        yaw=normalize_angle(a.yaw + b.yaw),
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return Pose2D(
        x=-(cos_yaw * pose.x + sin_yaw * pose.y),
        y=-(-sin_yaw * pose.x + cos_yaw * pose.y),
        yaw=normalize_angle(-pose.yaw),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_pose(current: Pose2D, target: Pose2D, translation_gain: float, yaw_gain: float) -> Pose2D:
    return Pose2D(
        x=current.x + translation_gain * (target.x - current.x),
        y=current.y + translation_gain * (target.y - current.y),
        yaw=normalize_angle(current.yaw + yaw_gain * normalize_angle(target.yaw - current.yaw)),
    )


class GpsOdometryCalibrator(Node):
    def __init__(self):
        super().__init__("gps_odometry_calibrator")

        self.declare_parameter("raw_odom_topic", "odom")
        self.declare_parameter("pinky_id", 0 & 0xFF)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("correction_period_sec", 2.0)
        self.declare_parameter("gps_timeout_sec", 5.0)
        self.declare_parameter("gps_min_confidence", 90)
        self.declare_parameter("translation_smoothing_gain", 0.35)
        self.declare_parameter("yaw_smoothing_gain", 0.35)

        raw_odom_topic = self.get_parameter("raw_odom_topic").value
        pinky_id = self.get_parameter("pinky_id").value
        gps_topic = f"/pinky_{pinky_id}/gps_pos"

        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        correction_period_sec = float(self.get_parameter("correction_period_sec").value)
        self.gps_timeout_sec = float(self.get_parameter("gps_timeout_sec").value)
        self.gps_min_confidence = float(self.get_parameter("gps_min_confidence").value)
        self.translation_smoothing_gain = float(
            self.get_parameter("translation_smoothing_gain").value
        )
        self.yaw_smoothing_gain = float(
            self.get_parameter("yaw_smoothing_gain").value
        )

        self.latest_raw_odom: Optional[Odometry] = None
        self.latest_gps_pose: Optional[PinkyGps] = None
        self.map_to_odom: Optional[Pose2D] = None
        self.initialized = False
        self.last_gps_stamp = None

        self.raw_odom_sub = self.create_subscription(
            Odometry,
            raw_odom_topic,
            self.raw_odom_callback,
            10,
        )
        self.gps_sub = self.create_subscription(
            PinkyGps,
            gps_topic,
            self.gps_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(correction_period_sec, self.calibration_timer_callback)

        self.get_logger().info(
            f"GPS odometry calibrator started. raw_odom='{raw_odom_topic}', "
            f"gps='{gps_topic}', pinky_id={pinky_id}, map_frame='{self.map_frame_id}', "
            f"odom_frame='{self.odom_frame_id}', "
            f"period={correction_period_sec:.1f}s"
        )

    def raw_odom_callback(self, msg: Odometry) -> None:
        self.latest_raw_odom = msg
        if self.initialized:
            self.publish_map_to_odom_tf(msg.header.stamp)

    def gps_callback(self, msg: PinkyGps) -> None:
        if not msg.is_valid or msg.confidence < self.gps_min_confidence:
            return
        self.latest_gps_pose = msg
        self.last_gps_stamp = Time.from_msg(msg.header.stamp)

    def calibration_timer_callback(self) -> None:
        if self.latest_raw_odom is None or self.latest_gps_pose is None:
            return

        if self.last_gps_stamp is None:
            return

        gps_age = (self.get_clock().now() - self.last_gps_stamp).nanoseconds / 1e9
        if gps_age > self.gps_timeout_sec:
            self.get_logger().warn(
                f"Skipping GPS correction because the latest GPS sample is stale ({gps_age:.2f}s)."
            )
            return

        raw_pose = self.get_latest_raw_pose()
        gps_pose = self.get_latest_gps_pose()

        if not self.initialized:
            self.map_to_odom = compose_pose(gps_pose, inverse_pose(raw_pose))
            self.initialized = True
            self.get_logger().info(
                f"Initialized '{self.map_frame_id}' -> '{self.odom_frame_id}' from GPS frame "
                f"'{self.latest_gps_pose.header.frame_id or 'unknown'}'."
            )
        else:
            assert self.map_to_odom is not None
            target_map_to_odom = self.compute_target_map_to_odom(raw_pose, gps_pose)
            translation_gain = max(0.0, min(1.0, self.translation_smoothing_gain))
            yaw_gain = max(0.0, min(1.0, self.yaw_smoothing_gain))
            self.map_to_odom = interpolate_pose(
                self.map_to_odom,
                target_map_to_odom,
                translation_gain,
                yaw_gain,
            )

            self.get_logger().info(
                "Applied GPS map correction: "
                f"map->odom x={self.map_to_odom.x:.3f}m, "
                f"y={self.map_to_odom.y:.3f}m, "
                f"yaw={math.degrees(self.map_to_odom.yaw):.2f}deg"
            )

    def get_latest_raw_pose(self) -> Pose2D:
        assert self.latest_raw_odom is not None
        return pose_to_2d(
            self.latest_raw_odom.pose.pose.position,
            self.latest_raw_odom.pose.pose.orientation,
        )

    def get_latest_gps_pose(self) -> Pose2D:
        assert self.latest_gps_pose is not None
        return Pose2D(
            x=self.latest_gps_pose.x_mm / 1000.0,
            y=self.latest_gps_pose.y_mm / 1000.0,
            yaw=math.radians(self.latest_gps_pose.yaw_deg),
        )

    def compute_target_map_to_odom(self, raw_pose: Pose2D, gps_pose: Pose2D) -> Pose2D:
        return compose_pose(gps_pose, inverse_pose(raw_pose))

    def publish_map_to_odom_tf(self, stamp) -> None:
        if not self.initialized or self.map_to_odom is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame_id
        transform.child_frame_id = self.odom_frame_id
        transform.transform.translation.x = self.map_to_odom.x
        transform.transform.translation.y = self.map_to_odom.y
        transform.transform.translation.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.map_to_odom.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdometryCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
