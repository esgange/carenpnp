from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    do_service = LaunchConfiguration('do_service')
    auto_off_on_exit = LaunchConfiguration('auto_off_on_exit')

    return LaunchDescription([
        DeclareLaunchArgument(
            'do_service',
            default_value='/dobot_bringup_ros2/srv/DO',
            description='Dobot DO service name',
        ),
        DeclareLaunchArgument(
            'auto_off_on_exit',
            default_value='true',
            description='Turn active outputs OFF when closing GUI',
        ),
        Node(
            package='gripper_control',
            executable='gripper_control_gui',
            name='gripper_control_gui',
            output='screen',
            parameters=[
                {
                    'do_service': do_service,
                    'auto_off_on_exit': auto_off_on_exit,
                }
            ],
        ),
    ])
