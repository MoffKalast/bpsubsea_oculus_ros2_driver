from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sonar_ip = LaunchConfiguration("sonar_ip")
    frame_id = LaunchConfiguration("frame_id")
    start_fan_image = LaunchConfiguration("start_fan_image")
    fan_output_width = LaunchConfiguration("fan_output_width")
    fan_output_height = LaunchConfiguration("fan_output_height")
    fan_meters_per_pixel = LaunchConfiguration("fan_meters_per_pixel")
    fan_max_range = LaunchConfiguration("fan_max_range")
    fan_center_x = LaunchConfiguration("fan_center_x")
    fan_center_y = LaunchConfiguration("fan_center_y")
    fan_palette = LaunchConfiguration("fan_palette")
    fan_flip_x = LaunchConfiguration("fan_flip_x")
    fan_flip_y = LaunchConfiguration("fan_flip_y")
    fan_head_down = LaunchConfiguration("fan_head_down")
    fan_background_value = LaunchConfiguration("fan_background_value")

    return LaunchDescription([
        DeclareLaunchArgument("sonar_ip", default_value="auto"),
        DeclareLaunchArgument("frame_id", default_value="oculus"),
        DeclareLaunchArgument("start_fan_image", default_value="true"),
        DeclareLaunchArgument("fan_output_width", default_value="800"),
        DeclareLaunchArgument("fan_output_height", default_value="800"),
        DeclareLaunchArgument("fan_meters_per_pixel", default_value="0.0"),
        DeclareLaunchArgument("fan_max_range", default_value="0.0"),
        DeclareLaunchArgument("fan_center_x", default_value="-1.0"),
        DeclareLaunchArgument("fan_center_y", default_value="-1.0"),
        DeclareLaunchArgument("fan_palette", default_value="mono8"),
        DeclareLaunchArgument("fan_flip_x", default_value="false"),
        DeclareLaunchArgument("fan_flip_y", default_value="false"),
        DeclareLaunchArgument("fan_head_down", default_value="true"),
        DeclareLaunchArgument("fan_background_value", default_value="0"),
        Node(
            package="bpsubsea_oculus_ros2_driver",
            executable="oculus_driver_node",
            name="oculus_driver",
            output="screen",
            parameters=[{
                "sonar_ip": sonar_ip,
                "frame_id": frame_id,
            }],
        ),
        Node(
            package="bpsubsea_oculus_ros2_driver",
            executable="oculus_fan_image_node",
            name="oculus_fan_image",
            output="screen",
            condition=IfCondition(start_fan_image),
            parameters=[{
                "output_width": fan_output_width,
                "output_height": fan_output_height,
                "meters_per_pixel": fan_meters_per_pixel,
                "max_range": fan_max_range,
                "center_x": fan_center_x,
                "center_y": fan_center_y,
                "palette": fan_palette,
                "flip_x": fan_flip_x,
                "flip_y": fan_flip_y,
                "head_down": fan_head_down,
                "background_value": fan_background_value,
            }],
        ),
    ])
