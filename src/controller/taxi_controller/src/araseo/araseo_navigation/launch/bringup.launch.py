import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString, RewrittenYaml

from pinky_bringup.robot_identity import (
    build_robot_identity,
    load_pinky_id_from_params_file,
)


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    pinky_params_file = LaunchConfiguration('pinky_params_file').perform(context)
    pinky_id_override = LaunchConfiguration('pinky_id').perform(context).strip()
    pinky_id = (
        int(pinky_id_override)
        if pinky_id_override
        else load_pinky_id_from_params_file(pinky_params_file)
    )
    identity = build_robot_identity(pinky_id)

    configured_params = ReplaceString(
        source_file=LaunchConfiguration('params_file'),
        replacements={
            '__MAP_FRAME__': identity['map_frame_id'],
            '__ODOM_FRAME__': identity['odom_frame_id'],
            '__BASE_FRAME__': identity['base_frame_id'],
            '__SCAN_TOPIC__': 'scan',
        },
    )
    namespaced_params = RewrittenYaml(
        source_file=configured_params,
        root_key=identity['namespace'],
        param_rewrites={},
        convert_types=True,
    )

    navigation_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('araseo_navigation'),
                'launch',
                'navigation_launch.xml',
            )
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': namespaced_params,
            'route_graph_file': LaunchConfiguration('route_graph_file'),
            'stops_file': LaunchConfiguration('stops_file'),
            'map_frame': identity['map_frame_id'],
            'base_frame': identity['base_frame_id'],
            'use_composition': LaunchConfiguration('use_composition'),
            'container_name': LaunchConfiguration('container_name'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'log_level': LaunchConfiguration('log_level'),
            'lifecycle_nodes': LaunchConfiguration('lifecycle_nodes_nav'),
        }.items(),
    )

    return [
        GroupAction(
            [
                PushRosNamespace(identity['namespace']),
                Node(
                    package='rclcpp_components',
                    executable='component_container_isolated',
                    name=LaunchConfiguration('container_name'),
                    output='screen',
                    arguments=[
                        '--ros-args',
                        '--log-level',
                        LaunchConfiguration('log_level'),
                    ],
                    parameters=[
                        namespaced_params,
                        {
                            'autostart': LaunchConfiguration('autostart'),
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                        },
                    ],
                    condition=IfCondition(LaunchConfiguration('use_composition')),
                ),
                navigation_launch,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('pinky_id', default_value=''),
            DeclareLaunchArgument(
                'pinky_params_file',
                default_value=os.path.join(
                    get_package_share_directory('pinky_bringup'),
                    'config',
                    'pinky_params.yaml',
                ),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('araseo_navigation'),
                    'params',
                    'nav_params.yaml',
                ),
            ),
            DeclareLaunchArgument(
                'route_graph_file',
                default_value=os.path.join(
                    get_package_share_directory('araseo_navigation'),
                    'params',
                    'route_graph.geojson',
                ),
            ),
            DeclareLaunchArgument(
                'stops_file',
                default_value=os.path.join(
                    get_package_share_directory('araseo_navigation'),
                    'params',
                    'stops.yaml',
                ),
            ),
            DeclareLaunchArgument('autostart', default_value='True'),
            DeclareLaunchArgument('container_name', default_value='nav2_container'),
            DeclareLaunchArgument('use_composition', default_value='False'),
            DeclareLaunchArgument('use_respawn', default_value='False'),
            DeclareLaunchArgument('log_level', default_value='debug'),
            DeclareLaunchArgument(
                'lifecycle_nodes_nav',
                default_value=(
                    "['controller_server', 'route_server', "
                    "'velocity_smoother']"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
