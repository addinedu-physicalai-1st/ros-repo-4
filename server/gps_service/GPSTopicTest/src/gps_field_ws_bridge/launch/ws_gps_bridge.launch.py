from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ws_port",
                default_value="8765",
                description="WebSocket listen port",
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value="",
                description="Override topic; empty → /pinky_{ROS_DOMAIN_ID}/gps_pos",
            ),
            DeclareLaunchArgument(
                "topic_pattern",
                default_value="/pinky_{domain_id}/gps_pos",
                description="Used when topic_name is empty",
            ),
            DeclareLaunchArgument(
                "domain_ids_env_file",
                default_value="",
                description="Optional .env path; non-empty loads PINKY_GPS_PUBLISH_DOMAIN_IDS list for multi-publish",
            ),
            DeclareLaunchArgument(
                "domain_ids_env_key",
                default_value="PINKY_GPS_PUBLISH_DOMAIN_IDS",
                description="Key inside .env for comma-separated domain ids",
            ),
            DeclareLaunchArgument(
                "domain_ids_env_filter_by_ros_main_id",
                default_value="false",
                description="If true, JSON ros_main_id publishes to one topic only if in .env list",
            ),
            Node(
                package="gps_field_ws_bridge",
                executable="ws_gps_publisher",
                name="ws_gps_publisher",
                output="screen",
                parameters=[
                    {
                        "ws_port": ParameterValue(
                            LaunchConfiguration("ws_port"), value_type=int
                        ),
                        "topic_name": LaunchConfiguration("topic_name"),
                        "topic_pattern": LaunchConfiguration("topic_pattern"),
                        "domain_ids_env_file": LaunchConfiguration(
                            "domain_ids_env_file"
                        ),
                        "domain_ids_env_key": LaunchConfiguration(
                            "domain_ids_env_key"
                        ),
                        "domain_ids_env_filter_by_ros_main_id": ParameterValue(
                            LaunchConfiguration(
                                "domain_ids_env_filter_by_ros_main_id"
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
