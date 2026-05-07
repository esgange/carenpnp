from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_debug',
            executable='motion_debug_gui',
            name='motion_debug_gui',
            output='screen',
        ),
    ])
