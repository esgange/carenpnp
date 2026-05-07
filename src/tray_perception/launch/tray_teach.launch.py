from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/erds/DOBOT_pickn_place/src/tray_perception/config/tray_detector.yaml",
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value="/camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
        ),
        Node(
            package="tray_perception",
            executable="tray_teach_node",
            name="tray_teach",
            output="screen",
            parameters=[
                params_file,
                {
                    "color_topic": color_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                },
            ],
        ),
    ])
