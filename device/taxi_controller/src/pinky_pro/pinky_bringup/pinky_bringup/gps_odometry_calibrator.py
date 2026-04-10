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


class GpsOdometryCalibrator(Node):
    def __init__(self):
        super().__init__("gps_odometry_calibrator")

        self.declare_parameter("raw_odom_topic", "odom/raw")
        self.declare_parameter("corrected_odom_topic", "odom")
        self.declare_parameter("gps_topic", "gps/pose")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("correction_period_sec", 2.0)
        self.declare_parameter("gps_timeout_sec", 5.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_gps_orientation", True)

        raw_odom_topic = self.get_parameter("raw_odom_topic").value
        corrected_odom_topic = self.get_parameter("corrected_odom_topic").value
        gps_topic = self.get_parameter("gps_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        correction_period_sec = float(self.get_parameter("correction_period_sec").value)
        self.gps_timeout_sec = float(self.get_parameter("gps_timeout_sec").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_gps_orientation = bool(self.get_parameter("use_gps_orientation").value)

        self.latest_raw_odom: Optional[Odometry] = None
        self.latest_gps_pose: Optional[PoseStamped] = None
        self.gps_frame_to_odom: Optional[Pose2D] = None
        self.odom_correction = Pose2D(0.0, 0.0, 0.0)
        self.last_gps_stamp = None

        self.corrected_odom_pub = self.create_publisher(Odometry, corrected_odom_topic, 10)
        self.raw_odom_sub = self.create_subscription(
            Odometry,
            raw_odom_topic,
            self.raw_odom_callback,
            10,
        )
        self.gps_sub = self.create_subscription(
            PoseStamped,
            gps_topic,
            self.gps_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(correction_period_sec, self.calibration_timer_callback)

        self.get_logger().info(
            f"GPS odometry calibrator started. raw_odom='{raw_odom_topic}', "
            f"gps='{gps_topic}', corrected_odom='{corrected_odom_topic}', "
            f"period={correction_period_sec:.1f}s"
        )

    def raw_odom_callback(self, msg: Odometry):
        self.latest_raw_odom = msg
        corrected_pose = compose_pose(
            self.odom_correction,
            pose_to_2d(msg.pose.pose.position, msg.pose.pose.orientation),
        )
        self.publish_corrected_odometry(msg, corrected_pose)

    def gps_callback(self, msg: PoseStamped):
        self.latest_gps_pose = msg
        self.last_gps_stamp = Time.from_msg(msg.header.stamp)

    def calibration_timer_callback(self):
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

        raw_pose = pose_to_2d(
            self.latest_raw_odom.pose.pose.position,
            self.latest_raw_odom.pose.pose.orientation,
        )
        gps_pose = pose_to_2d(
            self.latest_gps_pose.pose.position,
            self.latest_gps_pose.pose.orientation,
        )

        if not self.use_gps_orientation:
            gps_pose = Pose2D(gps_pose.x, gps_pose.y, raw_pose.yaw)

        if self.gps_frame_to_odom is None:
            self.gps_frame_to_odom = compose_pose(raw_pose, inverse_pose(gps_pose))
            self.get_logger().info(
                f"Locked GPS frame '{self.latest_gps_pose.header.frame_id or 'unknown'}' "
                f"to odom frame '{self.odom_frame_id}'."
            )

        gps_pose_in_odom = compose_pose(self.gps_frame_to_odom, gps_pose)
        self.odom_correction = compose_pose(gps_pose_in_odom, inverse_pose(raw_pose))

        self.get_logger().info(
            "Applied GPS correction: "
            f"dx={self.odom_correction.x:.3f}m, "
            f"dy={self.odom_correction.y:.3f}m, "
            f"dyaw={math.degrees(self.odom_correction.yaw):.2f}deg"
        )

        self.publish_corrected_odometry(self.latest_raw_odom, compose_pose(self.odom_correction, raw_pose))

    def publish_corrected_odometry(self, raw_odom: Odometry, corrected_pose: Pose2D):
        odom_msg = Odometry()
        odom_msg.header.stamp = raw_odom.header.stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.covariance = raw_odom.pose.covariance
        odom_msg.twist = raw_odom.twist

        odom_msg.pose.pose.position.x = corrected_pose.x
        odom_msg.pose.pose.position.y = corrected_pose.y
        odom_msg.pose.pose.position.z = raw_odom.pose.pose.position.z

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, corrected_pose.yaw)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        self.corrected_odom_pub.publish(odom_msg)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = odom_msg.header
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = corrected_pose.x
            transform.transform.translation.y = corrected_pose.y
            transform.transform.translation.z = raw_odom.pose.pose.position.z
            transform.transform.rotation = odom_msg.pose.pose.orientation
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
