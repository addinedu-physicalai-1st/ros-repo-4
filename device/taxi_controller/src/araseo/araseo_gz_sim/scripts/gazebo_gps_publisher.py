#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from gps_field_msgs.msg import PinkyGps
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboGpsPublisher(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_gps_publisher")

        self.declare_parameter("gazebo_pose_topic", "gazebo/pose")
        self.declare_parameter("gps_topic_pattern", "/pinky_{pinky_id}/gps_pos")
        self.declare_parameter("pinky_id", 0)
        self.declare_parameter("field_width_mm", 1880.0)
        self.declare_parameter("field_height_mm", 1410.0)
        self.declare_parameter("world_min_x_m", -0.93)
        self.declare_parameter("world_max_x_m", 0.93)
        self.declare_parameter("world_min_y_m", -0.70)
        self.declare_parameter("world_max_y_m", 0.70)
        self.declare_parameter("frame_id", "field_1880x1410")

        gazebo_pose_topic = self.get_parameter("gazebo_pose_topic").value
        gps_topic_pattern = self.get_parameter("gps_topic_pattern").value
        self.pinky_id = int(self.get_parameter("pinky_id").value) & 0xFF
        self.field_width_mm = float(self.get_parameter("field_width_mm").value)
        self.field_height_mm = float(self.get_parameter("field_height_mm").value)
        self.world_min_x_m = float(self.get_parameter("world_min_x_m").value)
        self.world_max_x_m = float(self.get_parameter("world_max_x_m").value)
        self.world_min_y_m = float(self.get_parameter("world_min_y_m").value)
        self.world_max_y_m = float(self.get_parameter("world_max_y_m").value)
        self.frame_id = self.get_parameter("frame_id").value
        gps_topic = gps_topic_pattern.format(pinky_id=self.pinky_id)

        self.gps_pub = self.create_publisher(PinkyGps, gps_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            gazebo_pose_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f"Publishing Gazebo pose '{gazebo_pose_topic}' to PinkyGps '{gps_topic}' "
            f"for pinky_id={self.pinky_id}"
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        x_mm = self.scale_to_mm(
            msg.pose.position.x,
            self.world_min_x_m,
            self.world_max_x_m,
            self.field_width_mm,
        )
        y_mm = self.scale_to_mm(
            msg.pose.position.y,
            self.world_min_y_m,
            self.world_max_y_m,
            self.field_height_mm,
        )
        yaw = yaw_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        gps_msg = PinkyGps()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = self.frame_id
        gps_msg.pinky_id = self.pinky_id
        gps_msg.x_mm = x_mm
        gps_msg.y_mm = y_mm
        gps_msg.yaw_deg = math.degrees(yaw)
        gps_msg.confidence = 100
        gps_msg.is_valid = True
        self.gps_pub.publish(gps_msg)

    @staticmethod
    def scale_to_mm(value_m: float, world_min_m: float, world_max_m: float, field_size_mm: float) -> float:
        if world_max_m <= world_min_m:
            return 0.0
        ratio = (value_m - world_min_m) / (world_max_m - world_min_m)
        ratio = max(0.0, min(1.0, ratio))
        return ratio * field_size_mm

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
