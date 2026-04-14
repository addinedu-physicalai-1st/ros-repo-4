#!/usr/bin/env python3

import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from geometry_msgs.msg import Point
from araseo_navigation.action import NavigateToStation
class StationInteractiveMarker(Node):
    def __init__(self):
        super().__init__('station_interactive_marker')

        self.stations_file = self.declare_parameter('stations_file', '').value
        self.route_graph_file = self.declare_parameter('route_graph_file', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value

        self.get_logger().info(f'Loading stations from: "{self.stations_file}"')
        self.get_logger().info(f'Loading route graph from: "{self.route_graph_file}"')


        self.server = InteractiveMarkerServer(self, 'station_markers')
        self.marker_pub = self.create_publisher(MarkerArray, 'route_graph_markers', 10)
        self.action_client = ActionClient(self, NavigateToStation, 'navigate_to_station')

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

    def publish_route_graph(self):
        if not self.route_graph:
            return

        marker_array = MarkerArray()
        
        # Points
        points_marker = Marker()
        points_marker.header.frame_id = self.map_frame
        points_marker.ns = 'route_points'
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.1
        points_marker.scale.y = 0.1
        points_marker.color.r = 1.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.pose.orientation.w = 1.0

        # Lines
        lines_marker = Marker()
        lines_marker.header.frame_id = self.map_frame
        lines_marker.ns = 'route_lines'
        lines_marker.id = 1
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.scale.x = 0.05
        lines_marker.color.r = 0.5
        lines_marker.color.g = 0.5
        lines_marker.color.b = 0.5
        lines_marker.color.a = 0.8
        lines_marker.pose.orientation.w = 1.0

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
                    p2 = Point()
                    p2.x = float(coords[i+1][0])
                    p2.y = float(coords[i+1][1])
                    lines_marker.points.append(p1)
                    lines_marker.points.append(p2)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(lines_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = StationInteractiveMarker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
