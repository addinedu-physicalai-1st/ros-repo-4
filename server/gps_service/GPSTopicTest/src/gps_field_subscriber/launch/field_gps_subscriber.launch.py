from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topic_name",
                default_value="",
                description="Empty → /pinky_{pinky_id}/gps_pos (requires pinky_id >= 0)",
            ),
            DeclareLaunchArgument(
                "topic_pattern",
                default_value="/pinky_{pinky_id}/gps_pos",
                description="With empty topic_name, format with pinky_id (legacy {domain_id} supported)",
            ),
            DeclareLaunchArgument(
                "pinky_id",
                default_value="-1",
                description=">=0 filter by msg.pinky_id; -1 accept all on topic",
            ),
            DeclareLaunchArgument(
                "show_tk",
                default_value="true",
                description="Desktop Tk window",
            ),
            DeclareLaunchArgument(
                "show_oled",
                default_value="false",
                description="I2C OLED (luma.oled; pip install -r requirements-oled.txt)",
            ),
            DeclareLaunchArgument(
                "show_pinky_spi",
                default_value="false",
                description="Pinky front SPI TFT (pinky_lcd + PIL; same as notebook LCD)",
            ),
            DeclareLaunchArgument(
                "pinky_lcd_img_width",
                default_value="320",
                description="PIL image width before img_show (notebook-style)",
            ),
            DeclareLaunchArgument(
                "pinky_lcd_img_height",
                default_value="240",
                description="PIL image height before img_show",
            ),
            DeclareLaunchArgument(
                "oled_i2c_port",
                default_value="1",
                description="I2C bus number (usually 1 on Pi)",
            ),
            DeclareLaunchArgument(
                "oled_i2c_address",
                default_value="60",
                description="Decimal address (60 = 0x3C)",
            ),
            DeclareLaunchArgument(
                "oled_device",
                default_value="ssd1306",
                description="ssd1306 or sh1106",
            ),
            Node(
                package="gps_field_subscriber",
                executable="field_gps_subscriber",
                name="field_gps_subscriber",
                output="screen",
                parameters=[
                    {
                        "topic_name": LaunchConfiguration("topic_name"),
                        "topic_pattern": LaunchConfiguration("topic_pattern"),
                        "pinky_id": ParameterValue(
                            LaunchConfiguration("pinky_id"), value_type=int
                        ),
                        "field_width_mm": 1880.0,
                        "field_height_mm": 1410.0,
                        "show_tk": ParameterValue(
                            LaunchConfiguration("show_tk"), value_type=bool
                        ),
                        "show_oled": ParameterValue(
                            LaunchConfiguration("show_oled"), value_type=bool
                        ),
                        "show_pinky_spi": ParameterValue(
                            LaunchConfiguration("show_pinky_spi"), value_type=bool
                        ),
                        "pinky_lcd_img_width": ParameterValue(
                            LaunchConfiguration("pinky_lcd_img_width"), value_type=int
                        ),
                        "pinky_lcd_img_height": ParameterValue(
                            LaunchConfiguration("pinky_lcd_img_height"), value_type=int
                        ),
                        "oled_i2c_port": ParameterValue(
                            LaunchConfiguration("oled_i2c_port"), value_type=int
                        ),
                        "oled_i2c_address": ParameterValue(
                            LaunchConfiguration("oled_i2c_address"), value_type=int
                        ),
                        "oled_device": LaunchConfiguration("oled_device"),
                    }
                ],
            ),
        ]
    )
