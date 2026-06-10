#!/usr/bin/env python3

from dataclasses import dataclass
import json
import math
from pathlib import Path as FilePath

from araseo_navigation.action import NavigateToNearestStop
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, FollowPath
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


@dataclass(frozen=True)
class Stop:
    stop_id: str
    route_node_id: int
    pose: PoseStamped


@dataclass(frozen=True)
class RouteNode:
    node_id: int
    x: float
    y: float


def quaternion_from_yaw(yaw):
    pose = PoseStamped()
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose.pose.orientation


class NearestStopNavigator(Node):

    def __init__(self):
        super().__init__('nearest_stop_navigator')

        self.stops_file = self.declare_parameter('stops_file', '').value
        self.route_graph_file = self.declare_parameter(
            'route_graph_file',
            '',
        ).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter(
            'base_frame',
            'base_footprint',
        ).value
        self.compute_route_action_name = self.declare_parameter(
            'compute_route_action_name',
            'compute_route',
        ).value
        self.follow_path_action_name = self.declare_parameter(
            'follow_path_action_name',
            'follow_path',
        ).value
        self.controller_id = self.declare_parameter(
            'controller_id',
            'FollowPath',
        ).value
        self.goal_checker_id = self.declare_parameter(
            'goal_checker_id',
            'general_goal_checker',
        ).value
        self.progress_checker_id = self.declare_parameter(
            'progress_checker_id',
            'progress_checker',
        ).value
        self.max_stop_distance = float(
            self.declare_parameter('max_stop_distance', 0.0).value
        )

        self.callback_group = ReentrantCallbackGroup()
        self.compute_route_client = ActionClient(
            self,
            ComputeRoute,
            self.compute_route_action_name,
            callback_group=self.callback_group,
        )
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            self.follow_path_action_name,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            NavigateToNearestStop,
            'navigate_to_nearest_stop',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_in_progress = False
        self.active_goal_handle = None
        self.active_route_goal = None
        self.active_follow_goal = None
        self.stops = self.load_stops(self.stops_file)
        self.route_nodes = self.load_route_nodes(self.route_graph_file)
        self.get_logger().info(
            f'Loaded {len(self.stops)} stops; '
            f'{len(self.route_nodes)} route nodes; '
            f"route_action='{self.compute_route_action_name}', "
            f"follow_action='{self.follow_path_action_name}', "
            f"controller='{self.controller_id}', "
            f"goal_checker='{self.goal_checker_id}', "
            f"progress_checker='{self.progress_checker_id}'"
        )

    def load_stops(self, stops_file):
        if not stops_file:
            raise RuntimeError('stops_file parameter must not be empty')

        with open(stops_file, 'r', encoding='utf-8') as stop_stream:
            data = yaml.safe_load(stop_stream) or {}

        stops_node = data.get('stops')
        if not isinstance(stops_node, list) or not stops_node:
            raise RuntimeError(
                "stops file must contain a non-empty top-level 'stops' list"
            )

        stops = []
        seen = set()
        for stop_node in stops_node:
            stop_id = stop_node['stop_id']
            if stop_id in seen:
                raise RuntimeError(f'duplicate stop_id: {stop_id}')
            seen.add(stop_id)

            pose_node = stop_node['pose']
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = float(pose_node['x'])
            pose.pose.position.y = float(pose_node['y'])
            pose.pose.position.z = float(pose_node.get('z', 0.0))
            pose.pose.orientation = quaternion_from_yaw(float(pose_node['yaw']))
            stops.append(
                Stop(
                    stop_id=stop_id,
                    route_node_id=int(stop_node['route_node_id']),
                    pose=pose,
                )
            )

        return stops

    def load_route_nodes(self, route_graph_file):
        if not route_graph_file:
            raise RuntimeError('route_graph_file parameter must not be empty')

        graph = json.loads(FilePath(route_graph_file).read_text(encoding='utf-8'))
        route_nodes = []
        for feature in graph.get('features', []):
            if feature.get('geometry', {}).get('type') != 'Point':
                continue
            coords = feature['geometry']['coordinates']
            route_nodes.append(
                RouteNode(
                    node_id=int(feature.get('properties', {})['id']),
                    x=float(coords[0]),
                    y=float(coords[1]),
                )
            )

        if not route_nodes:
            raise RuntimeError('route graph must contain point route nodes')

        return route_nodes

    def goal_callback(self, goal_request):
        del goal_request
        if self.goal_in_progress:
            self.get_logger().warn('Rejecting goal because navigation is active')
            return GoalResponse.REJECT

        self.goal_in_progress = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        if self.active_route_goal is not None:
            self.active_route_goal.cancel_goal_async()
        if self.active_follow_goal is not None:
            self.active_follow_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def nearest_stop(self, query_pose):
        qx = query_pose.pose.position.x
        qy = query_pose.pose.position.y
        return min(
            (
                (
                    math.hypot(
                        qx - stop.pose.pose.position.x,
                        qy - stop.pose.pose.position.y,
                    ),
                    stop,
                )
                for stop in self.stops
            ),
            key=lambda item: item[0],
        )

    def nearest_route_node(self, x, y):
        return min(
            self.route_nodes,
            key=lambda node: math.hypot(x - node.x, y - node.y),
        )

    def current_route_node(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        translation = transform.transform.translation
        return self.nearest_route_node(translation.x, translation.y)

    async def execute_callback(self, goal_handle):
        self.active_goal_handle = goal_handle
        try:
            return await self.navigate_to_nearest_stop(goal_handle)
        finally:
            self.active_goal_handle = None
            self.active_route_goal = None
            self.active_follow_goal = None
            self.goal_in_progress = False

    async def navigate_to_nearest_stop(self, goal_handle):
        result = NavigateToNearestStop.Result()
        query_pose = goal_handle.request.query_pose
        query_frame = query_pose.header.frame_id

        if query_frame and query_frame != self.map_frame:
            result.success = False
            result.error_code = ComputeRoute.Result.UNKNOWN
            result.message = (
                f"query_pose frame '{query_frame}' does not match "
                f"map_frame '{self.map_frame}'"
            )
            goal_handle.abort()
            return result

        stop_distance, stop = self.nearest_stop(query_pose)
        result.stop_id = stop.stop_id
        result.route_node_id = stop.route_node_id
        result.stop_pose = stop.pose

        feedback = NavigateToNearestStop.Feedback()
        feedback.stop_id = stop.stop_id
        feedback.stop_pose = stop.pose
        feedback.stop_distance = stop_distance
        goal_handle.publish_feedback(feedback)

        if self.max_stop_distance > 0.0 and stop_distance > self.max_stop_distance:
            result.success = False
            result.error_code = ComputeRoute.Result.NO_VALID_ROUTE
            result.message = (
                f"Nearest stop '{stop.stop_id}' is {stop_distance:.3f} m away, "
                f'over max_stop_distance={self.max_stop_distance:.3f}'
            )
            goal_handle.abort()
            return result

        try:
            start_node = self.current_route_node()
        except TransformException as exc:
            result.success = False
            result.error_code = ComputeRoute.Result.TF_ERROR
            result.message = (
                f'failed to get current robot pose from '
                f'{self.map_frame} to {self.base_frame}: {exc}'
            )
            goal_handle.abort()
            return result

        if not self.compute_route_client.wait_for_server(timeout_sec=10.0):
            result.success = False
            result.error_code = ComputeRoute.Result.UNKNOWN
            result.message = 'compute_route action server unavailable'
            goal_handle.abort()
            return result

        route_goal = ComputeRoute.Goal()
        route_goal.start_id = start_node.node_id
        route_goal.goal_id = stop.route_node_id
        route_goal.use_start = True
        route_goal.use_poses = False

        self.get_logger().info(
            f"Routing to stop_id='{stop.stop_id}', "
            f'start_node_id={start_node.node_id}, '
            f'route_node_id={stop.route_node_id}, '
            f'clicked_distance={stop_distance:.3f} m'
        )
        route_goal_handle = await self.compute_route_client.send_goal_async(
            route_goal
        )
        if not route_goal_handle.accepted:
            result.success = False
            result.error_code = ComputeRoute.Result.UNKNOWN
            result.message = 'compute_route rejected goal'
            goal_handle.abort()
            return result

        self.active_route_goal = route_goal_handle
        route_result = await route_goal_handle.get_result_async()
        self.active_route_goal = None

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = 'Navigation canceled during route computation'
            goal_handle.canceled()
            return result

        if (
            route_result.result is None
            or route_result.result.error_code != ComputeRoute.Result.NONE
        ):
            result.success = False
            result.error_code = (
                route_result.result.error_code
                if route_result.result is not None
                else ComputeRoute.Result.UNKNOWN
            )
            result.message = f'compute_route failed with {result.error_code=}'
            goal_handle.abort()
            return result

        path = route_result.result.path
        if not path.poses:
            result.success = False
            result.error_code = ComputeRoute.Result.NO_VALID_ROUTE
            result.message = 'compute_route returned an empty path'
            goal_handle.abort()
            return result

        self.plan_pub.publish(path)

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = 'Navigation canceled before path following'
            goal_handle.canceled()
            return result

        if not self.follow_path_client.wait_for_server(timeout_sec=10.0):
            result.success = False
            result.error_code = FollowPath.Result.UNKNOWN
            result.message = 'follow_path action server unavailable'
            goal_handle.abort()
            return result

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = self.controller_id
        follow_goal.goal_checker_id = self.goal_checker_id
        follow_goal.progress_checker_id = self.progress_checker_id

        def follow_feedback_callback(feedback_msg):
            follow_feedback = feedback_msg.feedback
            feedback.path_distance_remaining = follow_feedback.distance_to_goal
            feedback.speed = follow_feedback.speed
            goal_handle.publish_feedback(feedback)

        follow_goal_handle = await self.follow_path_client.send_goal_async(
            follow_goal,
            feedback_callback=follow_feedback_callback,
        )
        if not follow_goal_handle.accepted:
            result.success = False
            result.error_code = FollowPath.Result.INVALID_CONTROLLER
            result.message = 'follow_path rejected goal'
            goal_handle.abort()
            return result

        self.active_follow_goal = follow_goal_handle
        follow_result = await follow_goal_handle.get_result_async()
        self.active_follow_goal = None

        if goal_handle.is_cancel_requested:
            result.success = False
            result.error_code = (
                follow_result.result.error_code
                if follow_result.result is not None
                else FollowPath.Result.UNKNOWN
            )
            result.message = 'Navigation canceled'
            goal_handle.canceled()
            return result

        if (
            follow_result.result is not None
            and follow_result.result.error_code == FollowPath.Result.NONE
        ):
            result.success = True
            result.error_code = FollowPath.Result.NONE
            result.message = 'Arrived at nearest stop'
            goal_handle.succeed()
            return result

        result.success = False
        result.error_code = (
            follow_result.result.error_code
            if follow_result.result is not None
            else FollowPath.Result.UNKNOWN
        )
        result.message = (
            follow_result.result.error_msg
            if follow_result.result is not None
            else 'follow_path failed without result'
        )
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NearestStopNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
