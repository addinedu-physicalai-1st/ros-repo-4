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

    pinky_params_file = LaunchConfiguration("pinky_params").perform(context)
    bridge_params_file = LaunchConfiguration("bridge_params").perform(context)
    pinky_id_override = LaunchConfiguration("pinky_id").perform(context).strip()
    pinky_id = int(pinky_id_override) if pinky_id_override else load_pinky_id_from_params_file(pinky_params_file)
    identity = build_robot_identity(pinky_id)
    robot_name = f"{identity['namespace']}_pinky"
    world_path = os.path.join(get_package_share_directory("araseo_gz_sim"), "worlds", "minicity.sdf")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r -v4 {world_path}",
                "on_exit_shutdown": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("pinky_description"),
                    "launch",
                    "upload_robot.launch.py",
                )
            ),
            launch_arguments={
                "namespace": identity["namespace"],
                "is_sim": LaunchConfiguration("use_sim_time"),
                "cam_tilt_deg": LaunchConfiguration("cam_tilt_deg"),
            }.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name=f"{identity['namespace']}_create",
            output="screen",
            arguments=[
                "-name",
                robot_name,
                "-topic",
                f"/{identity['namespace']}/robot_description",
                "-x",
                LaunchConfiguration("spawn_x"),
                "-y",
                LaunchConfiguration("spawn_y"),
                "-z",
                LaunchConfiguration("spawn_z"),
                "-Y",
                LaunchConfiguration("spawn_yaw"),
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        GroupAction(
            [
                PushRosNamespace(identity["namespace"]),
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="parameter_bridge",
                    output="screen",
                    arguments=["--ros-args", "-p", f"config_file:={bridge_params_file}"],
                    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                ),
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gps_pose_bridge",
                    output="screen",
                    arguments=[f"/model/{robot_name}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"],
                    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                    remappings=[(f"/model/{robot_name}/pose", "gazebo/pose")],
                ),
                Node(
                    package="araseo_gz_sim",
                    executable="gazebo_gps_publisher.py",
                    name="gazebo_gps_publisher",
                    output="screen",
                    parameters=[
                        LaunchConfiguration("pinky_params"),
                        {
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "pinky_id": identity["pinky_id"],
                        },
                    ],
                ),
                Node(
                    package="pinky_bringup",
                    executable="gps_odometry_calibrator",
                    name="gps_odometry_calibrator",
                    output="screen",
                    parameters=[
                        LaunchConfiguration("pinky_params"),
                        {
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "pinky_id": identity["pinky_id"],
                            "raw_odom_topic": "odom",
                            "map_frame_id": identity["map_frame_id"],
                            "odom_frame_id": identity["odom_frame_id"],
                            "base_frame_id": identity["base_frame_id"],
                        },
                    ],
                ),
                Node(
                    package="ros_gz_image",
                    executable="image_bridge",
                    name="image_bridge_raw",
                    output="screen",
                    arguments=["camera/image_raw"],
                    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                ),
                Node(
                    package="ros_gz_image",
                    executable="image_bridge",
                    name="image_bridge",
                    output="screen",
                    arguments=["camera"],
                    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                ),
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pinky_id", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("cam_tilt_deg", default_value="25"),
            DeclareLaunchArgument(
                "bridge_params",
                default_value=os.path.join(
                    get_package_share_directory("araseo_gz_sim"),
                    "params",
                    "pinky_bridge.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "pinky_params",
                default_value=os.path.join(
                    get_package_share_directory("pinky_bringup"),
                    "config",
                    "pinky_params.yaml",
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
