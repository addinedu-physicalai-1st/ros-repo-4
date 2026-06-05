from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    topic_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="",
        description="Empty → /pinky_{ROS_DOMAIN_ID}/gps_pos",
    )
    pattern_arg = DeclareLaunchArgument(
        "topic_pattern",
        default_value="/pinky_{domain_id}/gps_pos",
        description="With empty topic_name",
    )

    publisher = Node(
        package="gps_field_publisher",
        executable="field_gps_publisher",
        name="field_gps_publisher",
        output="screen",
        parameters=[
            {
                "topic_name": LaunchConfiguration("topic_name"),
                "topic_pattern": LaunchConfiguration("topic_pattern"),
                "field_width_mm": 1880.0,
                "field_height_mm": 1410.0,
                "publish_rate_hz": 5.0,
                "pinky_ids": [],
                "frame_id": "field_1880x1410",
            }
        ],
    )

    return LaunchDescription([topic_arg, pattern_arg, publisher])
