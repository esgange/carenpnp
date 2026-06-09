from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path

def _ros_domain_action():
    import importlib.util

    helper_candidates = []
    for parent in Path(__file__).resolve().parents:
        helper_candidates.extend([
            parent / 'src' / 'dobot_bringup_v4' / 'launch' / 'ros_domain.py',
            parent / 'install' / 'cr_robot_ros2' / 'share' / 'cr_robot_ros2' / 'launch' / 'ros_domain.py',
            parent / 'cr_robot_ros2' / 'share' / 'cr_robot_ros2' / 'launch' / 'ros_domain.py',
            parent / 'share' / 'cr_robot_ros2' / 'launch' / 'ros_domain.py',
        ])

    for helper_path in helper_candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location('_dobot_ros_domain', helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module.ros_domain_action()

    raise RuntimeError('Could not find ros_domain.py helper for ROS_DOMAIN_ID')


def generate_launch_description():
    do_service = LaunchConfiguration('do_service')
    di_status_topic = LaunchConfiguration('di_status_topic')
    auto_off_on_exit = LaunchConfiguration('auto_off_on_exit')

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument(
            'do_service',
            default_value='/dobot_bringup_ros2/srv/DO',
            description='Dobot DO service name',
        ),
        DeclareLaunchArgument(
            'di_status_topic',
            default_value='/dobot_bringup_ros2/DIStatus_200mS',
            description='Dobot 30005 DI status topic containing digital_input_bits',
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
                    'di_status_topic': di_status_topic,
                    'auto_off_on_exit': ParameterValue(auto_off_on_exit, value_type=bool),
                }
            ],
        ),
    ])
