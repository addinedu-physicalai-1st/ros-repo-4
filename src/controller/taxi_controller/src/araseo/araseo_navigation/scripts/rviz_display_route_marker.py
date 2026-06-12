#!/usr/bin/env python3

import json

from action_msgs.msg import GoalStatus
from araseo_msgs.action import NavigateToNearestStop
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class RvizDisplayRouteMarker(Node):

    def __init__(self):
        super().__init__('rviz_display_route_marker')

        self.stops_file = self.declare_parameter('stops_file', '').value
        self.route_graph_file = self.declare_parameter(
            'route_graph_file',
            '',
        ).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.marker_topic = self.declare_parameter(
            'marker_topic',
            'route_graph_markers',
        ).value
        self.goal_pose_topic = self.declare_parameter(
            'goal_pose_topic',
            'goal_pose',
        ).value
        self.navigate_action_name = self.declare_parameter(
            'navigate_action_name',
            'navigate_to_nearest_stop',
        ).value
        self.plan_topic = self.declare_parameter('plan_topic', 'plan').value
        self.local_plan_topic = self.declare_parameter(
            'local_plan_topic',
            'local_plan',
        ).value

        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            marker_qos,
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToNearestStop,
            self.navigate_action_name,
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.on_goal_pose,
            10,
        )
        self.plan_sub = self.create_subscription(
            Path,
            self.plan_topic,
            self.on_global_plan,
            10,
        )
        self.local_plan_sub = self.create_subscription(
            Path,
            self.local_plan_topic,
            self.on_local_plan,
            10,
        )

        self.stops = self.load_stops()
        self.route_graph = self.load_route_graph()
        self.global_plan = None
        self.local_plan = None
        self.navigate_goal_active = False
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f"RViz goal bridge: '{self.goal_pose_topic}' -> "
            f"'{self.navigate_action_name}'"
        )

    def load_stops(self):
        if not self.stops_file:
            self.get_logger().warn('No stops file provided')
            return []
        with open(self.stops_file, 'r', encoding='utf-8') as stop_stream:
            data = yaml.safe_load(stop_stream) or {}
        stops = data.get('stops', [])
        self.get_logger().info(f'Loaded {len(stops)} stops')
        return stops

    def load_route_graph(self):
        if not self.route_graph_file:
            self.get_logger().warn('No route graph file provided')
            return None
        with open(self.route_graph_file, 'r', encoding='utf-8') as graph_stream:
            graph = json.load(graph_stream)
        self.get_logger().info(f'Loaded route graph from: {self.route_graph_file}')
        return graph

    def on_global_plan(self, path_msg):
        self.global_plan = path_msg

    def on_local_plan(self, path_msg):
        self.local_plan = path_msg

    def on_goal_pose(self, pose_msg):
        if self.navigate_goal_active:
            self.get_logger().warn(
                'Ignoring RViz goal because navigate_to_nearest_stop is active'
            )
            return

        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Action server '{self.navigate_action_name}' is unavailable"
            )
            return

        goal = NavigateToNearestStop.Goal()
        goal.query_pose = pose_msg
        if not goal.query_pose.header.frame_id:
            goal.query_pose.header.frame_id = self.map_frame

        self.global_plan = None
        self.local_plan = None
        self.publish_clear_plan_markers()

        self.navigate_goal_active = True
        self.get_logger().info(
            'Sending RViz goal pose to nearest-stop navigator: '
            f'x={goal.query_pose.pose.position.x:.3f}, '
            f'y={goal.query_pose.pose.position.y:.3f}, '
            f"frame='{goal.query_pose.header.frame_id}'"
        )
        send_future = self.navigate_client.send_goal_async(
            goal,
            feedback_callback=self.on_navigate_feedback,
        )
        send_future.add_done_callback(self.on_navigate_goal_response)

    def on_navigate_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.stop_id:
            self.get_logger().debug(
                f"Nearest stop candidate: stop_id='{feedback.stop_id}', "
                f'distance={feedback.stop_distance:.3f} m'
            )

    def on_navigate_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - keep ROS callback alive
            self.navigate_goal_active = False
            self.get_logger().error(f'Failed to send navigation goal: {exc}')
            return

        if not goal_handle.accepted:
            self.navigate_goal_active = False
            self.get_logger().warn('navigate_to_nearest_stop rejected RViz goal')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_navigate_result)

    def on_navigate_result(self, future):
        self.navigate_goal_active = False
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001 - keep ROS callback alive
            self.get_logger().error(f'Failed to get navigation result: {exc}')
            return

        result = wrapped_result.result
        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.success
        ):
            self.get_logger().info(
                f"Arrived at nearest stop '{result.stop_id}' "
                f'(route_node_id={result.route_node_id})'
            )
            return

        self.get_logger().warn(
            f'Nearest-stop navigation finished with status={wrapped_result.status}, '
            f"success={result.success}, message='{result.message}'"
        )

    def build_delete_marker(self, marker_id, namespace, now):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = now
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def publish_clear_plan_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_array.markers.append(
            self.build_delete_marker(100, 'global_plan', now)
        )
        marker_array.markers.append(
            self.build_delete_marker(101, 'local_plan', now)
        )
        self.marker_pub.publish(marker_array)

    def build_path_marker(
        self,
        marker_id,
        namespace,
        path_msg,
        color,
        z_offset,
        line_width,
    ):
        marker = Marker()
        marker.header.frame_id = path_msg.header.frame_id or self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD if path_msg.poses else Marker.DELETE
        marker.scale.x = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.orientation.w = 1.0

        for pose_stamped in path_msg.poses:
            point = Point()
            point.x = pose_stamped.pose.position.x
            point.y = pose_stamped.pose.position.y
            point.z = pose_stamped.pose.position.z + z_offset
            marker.points.append(point)

        return marker

    def append_stop_label_markers(self, marker_array, now):
        for marker_id, stop in enumerate(self.stops, start=1):
            pose = stop.get('pose', {})
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now
            marker.ns = 'stop_labels'
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(pose['x'])
            marker.pose.position.y = float(pose['y'])
            marker.pose.position.z = float(pose.get('z', 0.0)) + 0.16
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.12
            marker.color.r = 0.85
            marker.color.g = 1.0
            marker.color.b = 0.85
            marker.color.a = 0.95
            marker.text = stop.get('stop_id', f'stop_{marker_id}')
            marker_array.markers.append(marker)

    def build_stop_marker(self, now):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = now
        marker.ns = 'stops'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0

        for stop in self.stops:
            pose = stop.get('pose', {})
            point = Point()
            point.x = float(pose['x'])
            point.y = float(pose['y'])
            point.z = float(pose.get('z', 0.04))
            marker.points.append(point)

        return marker

    def append_route_graph_markers(self, marker_array, now):
        if not self.route_graph:
            return

        points_marker = Marker()
        points_marker.header.frame_id = self.map_frame
        points_marker.header.stamp = now
        points_marker.ns = 'route_points'
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.12
        points_marker.scale.y = 0.12
        points_marker.color.r = 1.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.pose.orientation.w = 1.0

        lines_marker = Marker()
        lines_marker.header.frame_id = self.map_frame
        lines_marker.header.stamp = now
        lines_marker.ns = 'route_lines'
        lines_marker.id = 1
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.scale.x = 0.04
        lines_marker.color.r = 0.05
        lines_marker.color.g = 0.05
        lines_marker.color.b = 0.05
        lines_marker.color.a = 0.2
        lines_marker.pose.orientation.w = 1.0

        for feature in self.route_graph.get('features', []):
            geom = feature.get('geometry', {})
            geom_type = geom.get('type')
            coords = geom.get('coordinates', [])
            if geom_type == 'Point':
                point = Point()
                point.x = float(coords[0])
                point.y = float(coords[1])
                points_marker.points.append(point)
            elif geom_type == 'LineString':
                self.append_line_segments(lines_marker, coords)
            elif geom_type == 'MultiLineString':
                for line in coords:
                    self.append_line_segments(lines_marker, line)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(lines_marker)

    def append_line_segments(self, marker, coords):
        for idx in range(len(coords) - 1):
            start = Point()
            start.x = float(coords[idx][0])
            start.y = float(coords[idx][1])
            end = Point()
            end.x = float(coords[idx + 1][0])
            end.y = float(coords[idx + 1][1])
            marker.points.append(start)
            marker.points.append(end)

    def publish_markers(self):
        if (
            not self.route_graph
            and not self.stops
            and not self.global_plan
            and not self.local_plan
        ):
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        self.append_route_graph_markers(marker_array, now)

        if self.stops:
            marker_array.markers.append(self.build_stop_marker(now))
            self.append_stop_label_markers(marker_array, now)

        if self.global_plan is not None:
            marker_array.markers.append(
                self.build_path_marker(
                    100,
                    'global_plan',
                    self.global_plan,
                    (1.0, 0.5, 0.0, 1.0),
                    0.05,
                    0.06,
                )
            )
        if self.local_plan is not None:
            marker_array.markers.append(
                self.build_path_marker(
                    101,
                    'local_plan',
                    self.local_plan,
                    (0.0, 0.67, 1.0, 1.0),
                    0.07,
                    0.04,
                )
            )

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RvizDisplayRouteMarker()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
