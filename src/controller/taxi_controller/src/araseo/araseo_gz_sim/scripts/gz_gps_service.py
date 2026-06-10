#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from araseo_msgs.srv import TaxiGps


class GazeboGpsService(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_gps_service")

        self.declare_parameter("gazebo_pose_topic", "gazebo/pose")
        self.declare_parameter("service_name", "/gps/taxi")
        self.declare_parameter("pinky_id", 0)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("origin_offset_x_m", 0.93)
        self.declare_parameter("origin_offset_y_m", 0.70)

        self.pinky_id = int(self.get_parameter("pinky_id").value) & 0xFF
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.origin_offset_x_m = float(self.get_parameter("origin_offset_x_m").value)
        self.origin_offset_y_m = float(self.get_parameter("origin_offset_y_m").value)
        gazebo_pose_topic = self.get_parameter("gazebo_pose_topic").value
        service_name = self.get_parameter("service_name").value

        self.latest_gps: Optional[PoseStamped] = None

        self.pose_sub = self.create_subscription(
            PoseStamped,
            gazebo_pose_topic,
            self.pose_callback,
            10,
        )
        self.gps_service = self.create_service(TaxiGps, service_name, self.handle_gps_request)

        self.get_logger().info(
            f"Serving Gazebo pose '{gazebo_pose_topic}' as TaxiGps '{service_name}' "
            f"for pinky_id={self.pinky_id}"
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        gps = PoseStamped()
        gps.header.stamp = msg.header.stamp
        gps.header.frame_id = self.map_frame_id
        gps.pose.position.x = msg.pose.position.x + self.origin_offset_x_m
        gps.pose.position.y = msg.pose.position.y + self.origin_offset_y_m
        gps.pose.position.z = msg.pose.position.z
        gps.pose.orientation = msg.pose.orientation
        self.latest_gps = gps

    def handle_gps_request(self, request: TaxiGps.Request, response: TaxiGps.Response) -> TaxiGps.Response:
        request_id = int(request.id) & 0xFF
        if request_id != self.pinky_id:
            response.success = False
            response.confidence = 0
            response.gps = PoseStamped()
            return response

        if self.latest_gps is None:
            response.success = False
            response.confidence = 0
            response.gps = PoseStamped()
            return response

        response.success = True
        response.confidence = 100
        response.gps = self.latest_gps
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboGpsService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
