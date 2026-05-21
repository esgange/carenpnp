import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


STRING_PARAMS = (
    'runtime_settings_file',
    'item_profile_state_file',
    'motion_service_root',
    'gripper_do_service',
    'item_pose_topic',
    'start_sequence_service',
    'track_service',
    'track_status_service',
    'item_seek_complete_service',
    'robot_goal_frame_id',
    'robot_gripper_frame_id',
    'camera_safety_frame_id',
)

BOOL_PARAMS = (
    'headless',
    'load_runtime_settings',
    'tf_only_mode',
    'publish_goal_debug_tf',
    'prefer_camera_inside_bin',
)

FLOAT_PARAMS = (
    'item_pose_wait_timeout_sec',
    'ee_intercept_speed_mm_s',
    'item_x_offset_mm',
    'item_y_offset_mm',
    'item_standoff_z_mm',
    'approach_z_up_mm',
    'final_z_up_mm',
    'pre_pick_settling_time_sec',
    'pick_settling_time_sec',
    'command_hysteresis_sec',
    'tool_offset_x_mm',
    'tool_offset_y_mm',
    'tool_offset_z_mm',
    'tool_offset_rx_deg',
    'tool_offset_ry_deg',
    'tool_offset_rz_deg',
    'camera_bin_safe_margin_mm',
)


def _item_pick_pythonpath() -> str:
    paths = []
    for parent in Path(__file__).resolve().parents:
        build_path = parent / 'build' / 'item_pick'
        source_path = parent / 'src' / 'item_pick'
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


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _bool_arg(value: str, name: str) -> bool:
    lowered = value.lower()
    if lowered in ('1', 'true', 'yes', 'on'):
        return True
    if lowered in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(f'{name} must be a boolean, got {value!r}')


def _float_arg(value: str, name: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(f'{name} must be a number, got {value!r}') from exc


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    params = {}
    for name in STRING_PARAMS:
        value = _arg(context, name)
        if value:
            params[name] = value
    for name in BOOL_PARAMS:
        value = _arg(context, name)
        if value:
            params[name] = _bool_arg(value, name)
    for name in FLOAT_PARAMS:
        value = _arg(context, name)
        if value:
            params[name] = _float_arg(value, name)

    return [
        Node(
            package='item_pick',
            executable='item_pick',
            name='item_pick',
            output='screen',
            parameters=[params] if params else [],
            additional_env={
                'PYTHONPATH': _item_pick_pythonpath(),
            },
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        _ros_domain_action(),
        *[DeclareLaunchArgument(name, default_value='') for name in STRING_PARAMS],
        *[DeclareLaunchArgument(name, default_value='') for name in BOOL_PARAMS],
        *[DeclareLaunchArgument(name, default_value='') for name in FLOAT_PARAMS],
        OpaqueFunction(function=_launch_setup),
    ])
