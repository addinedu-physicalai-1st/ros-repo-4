#!/usr/bin/env python3

import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from araseo_navigation.action import NavigateToStation
class StationInteractiveMarker(Node):
    def __init__(self):
        super().__init__('station_interactive_marker')

        self.stations_file = self.declare_parameter('stations_file', '').value
        self.route_graph_file = self.declare_parameter('route_graph_file', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.plan_topic = self.declare_parameter('plan_topic', '/plan').value
        self.local_plan_topic = self.declare_parameter('local_plan_topic', '/local_plan').value

        self.get_logger().info(f'Loading stations from: "{self.stations_file}"')
        self.get_logger().info(f'Loading route graph from: "{self.route_graph_file}"')
        self.get_logger().info(f'Subscribing to global plan topic: "{self.plan_topic}"')
        self.get_logger().info(f'Subscribing to local plan topic: "{self.local_plan_topic}"')

        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.server = InteractiveMarkerServer(self, 'station_markers')
        self.marker_pub = self.create_publisher(MarkerArray, 'route_graph_markers', marker_qos)
        self.action_client = ActionClient(self, NavigateToStation, 'navigate_to_station')
        self.plan_sub = self.create_subscription(Path, self.plan_topic, self.on_global_plan, 10)
        self.local_plan_sub = self.create_subscription(Path, self.local_plan_topic, self.on_local_plan, 10)

        self.global_plan = None
        self.local_plan = None

        self.load_stations()
        self.load_route_graph()
        
        self.create_timer(1.0, self.publish_route_graph)

    def load_stations(self):
        if not self.stations_file:
            self.get_logger().warn('No stations file provided')
            return

        try:
            with open(self.stations_file, 'r') as f:
                data = yaml.safe_load(f)
                self.stations = data.get('stations', [])
            self.get_logger().info(f'Loaded {len(self.stations)} stations')
        except Exception as e:
            self.get_logger().error(f'Failed to load stations file: {e}')
            return

        for station in self.stations:
            self.make_station_marker(station)
        self.server.applyChanges()

    def make_station_marker(self, station):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.map_frame
        int_marker.name = station['station_id']
        int_marker.description = station['display_name']
        int_marker.scale = 0.6
        int_marker.pose.position.x = float(station['pose']['x'])
        int_marker.pose.position.y = float(station['pose']['y'])
        int_marker.pose.position.z = 0.2
        int_marker.pose.orientation.w = 1.0

        # Sphere marker for the station
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0

        control = InteractiveMarkerControl()
        control.name = f"{station['station_id']}_button"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info(f'Station {feedback.marker_name} clicked. Navigating...')
            self.send_goal(feedback.marker_name)

    def send_goal(self, station_id):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        goal_msg = NavigateToStation.Goal()
        goal_msg.station_id = station_id
        
        self.get_logger().info(f'Sending goal to station: {station_id}')
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)

    def load_route_graph(self):
        if not self.route_graph_file:
            self.get_logger().warn('No route graph file provided')
            self.route_graph = None
            return

        with open(self.route_graph_file, 'r') as f:
            self.route_graph = json.load(f)

    def on_global_plan(self, path_msg):
        self.global_plan = path_msg

    def on_local_plan(self, path_msg):
        self.local_plan = path_msg

    def build_path_marker(self, marker_id, namespace, path_msg, color, z_offset, line_width):
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

    def publish_route_graph(self):
        if not self.route_graph and not self.global_plan and not self.local_plan:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Points
        points_marker = Marker()
        points_marker.header.frame_id = self.map_frame
        points_marker.header.stamp = now
        points_marker.ns = 'route_points'
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.14
        points_marker.scale.y = 0.14
        points_marker.color.r = 1.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.pose.position.z = 0.0
        points_marker.pose.orientation.w = 1.0

        # Lines
        lines_marker = Marker()
        lines_marker.header.frame_id = self.map_frame
        lines_marker.header.stamp = now
        lines_marker.ns = 'route_lines'
        lines_marker.id = 1
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.scale.x = 0.05
        lines_marker.color.r = 0.05
        lines_marker.color.g = 0.05
        lines_marker.color.b = 0.05
        lines_marker.color.a = 0.08
        lines_marker.pose.position.z = 0.0
        lines_marker.pose.orientation.w = 1.0

        if self.route_graph:
            for feature in self.route_graph.get('features', []):
                geom = feature.get('geometry', {})
                if geom.get('type') == 'Point':
                    p = Point()
                    p.x = float(geom['coordinates'][0])
                    p.y = float(geom['coordinates'][1])
                    p.z = 0.0
                    points_marker.points.append(p)
                elif geom.get('type') == 'LineString':
                    coords = geom['coordinates']
                    for i in range(len(coords) - 1):
                        p1 = Point()
                        p1.x = float(coords[i][0])
                        p1.y = float(coords[i][1])
                        p1.z = 0.0
                        p2 = Point()
                        p2.x = float(coords[i+1][0])
                        p2.y = float(coords[i+1][1])
                        p2.z = 0.0
                        lines_marker.points.append(p1)
                        lines_marker.points.append(p2)

            marker_array.markers.append(points_marker)
            marker_array.markers.append(lines_marker)

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
    node = StationInteractiveMarker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
