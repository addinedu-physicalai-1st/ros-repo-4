import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from pinky_bringup.robot_identity import build_robot_identity, load_pinky_id_from_params_file


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    params_file = LaunchConfiguration("params_file").perform(context)
    pinky_id_override = LaunchConfiguration("pinky_id").perform(context).strip()
    pinky_id = int(pinky_id_override) if pinky_id_override else load_pinky_id_from_params_file(params_file)
    identity = build_robot_identity(pinky_id)

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("pinky_description"),
                "launch",
                "upload_robot.launch.py",
            )
        ),
        launch_arguments={
            "namespace": identity["namespace"],
            "is_sim": use_sim_time,
        }.items(),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sllidar_ros2"),
                "launch",
                "sllidar_c1_launch.py",
            )
        ),
        launch_arguments={
            "serial_port": "/dev/ttyAMA0",
            "frame_id": identity["lidar_frame_id"],
            "inverted": "false",
            "angle_compensate": "true",
            "scan_mode": "DenseBoost",
        }.items(),
    )

    return [
        upload_robot,
        GroupAction(
            [
                PushRosNamespace(identity["namespace"]),
                lidar,
                Node(
                    package="pinky_bringup",
                    executable="bringup",
                    parameters=[
                        params_file,
                        {
                            "wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation,
                            "use_sim_time": use_sim_time,
                            "pinky_id": identity["pinky_id"],
                            "odom_frame_id": identity["odom_frame_id"],
                            "base_frame_id": identity["base_frame_id"],
                        },
                    ],
                ),
                Node(
                    package="pinky_bringup",
                    executable="battery_publisher",
                ),
                Node(
                    package="pinky_bringup",
                    executable="gps_odometry_calibrator",
                    parameters=[
                        params_file,
                        {
                            "use_sim_time": use_sim_time,
                            "pinky_id": identity["pinky_id"],
                            "raw_odom_topic": "odom",
                            "map_frame_id": identity["map_frame_id"],
                            "odom_frame_id": identity["odom_frame_id"],
                            "base_frame_id": identity["base_frame_id"],
                        },
                    ],
                ),
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("wheel_radius", default_value="0.027"),
            DeclareLaunchArgument("wheel_separation", default_value="0.0961"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    get_package_share_directory("pinky_bringup"),
                    "config",
                    "pinky_params.yaml",
                ),
            ),
            DeclareLaunchArgument("pinky_id", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
