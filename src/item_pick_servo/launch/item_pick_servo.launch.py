import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _item_pick_servo_pythonpath() -> str:
    paths = []
    for parent in Path(__file__).resolve().parents:
        build_path = parent / 'build' / 'item_pick_servo'
        source_path = parent / 'src' / 'item_pick_servo'
        if build_path.exists() or source_path.exists():
            if build_path.exists():
                paths.append(str(build_path))
            if source_path.exists():
                paths.append(str(source_path))
            break

    current_pythonpath = os.environ.get('PYTHONPATH', '')
    if current_pythonpath:
        paths.append(current_pythonpath)
    return os.pathsep.join(paths)

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
    return LaunchDescription([
        DeclareLaunchArgument(
            'item_approach_servo_p_t_sec',
            default_value='1.5',
            description='ServoP point runtime in seconds for item approach.',
        ),
        DeclareLaunchArgument(
            'item_final_z_up_servo_p_t_sec',
            default_value='1.0',
            description='ServoP point runtime in seconds for item final Z-up.',
        ),
        DeclareLaunchArgument(
            'return_to_tray_teach_servo_j_t_sec',
            default_value='1.5',
            description='ServoJ joint runtime in seconds for returning to tray perception teach.',
        ),
        _ros_domain_action(),
        Node(
            package='item_pick_servo',
            executable='item_pick_servo',
            name='item_pick_servo',
            output='screen',
            parameters=[{
                'item_approach_servo_p_t_sec': LaunchConfiguration('item_approach_servo_p_t_sec'),
                'item_final_z_up_servo_p_t_sec': LaunchConfiguration('item_final_z_up_servo_p_t_sec'),
                'return_to_tray_teach_servo_j_t_sec': LaunchConfiguration(
                    'return_to_tray_teach_servo_j_t_sec'
                ),
            }],
            additional_env={
                'PYTHONPATH': _item_pick_servo_pythonpath(),
            },
        ),
    ])
