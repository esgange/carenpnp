import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


STRING_PARAMS = (
    'runtime_settings_file',
    'motion_service_root',
    'tray_vector_topic',
    'tray_axis_overlay_topic',
    'start_sequence_service',
    'track_service',
    'track_status_service',
    'tray_dimensions_service',
    'tray_seek_complete_service',
    'robot_goal_frame_id',
)

BOOL_PARAMS = (
    'headless',
    'load_runtime_settings',
    'tf_only_mode',
    'release_grip_enabled',
    'publish_goal_debug_tf',
)

FLOAT_PARAMS = (
    'tray_vector_wait_timeout_sec',
    'ee_intercept_speed_mm_s',
    'ee_final_pose_angle_deg',
    'tray_intercept_x_offset_mm',
    'tray_intercept_y_offset_mm',
    'tray_standoff_z_mm',
    'follow_distance_mm',
    'post_follow_z_up_mm',
    'command_hysteresis_sec',
    'goal_tf_lookup_timeout_sec',
    'tray_prediction_max_lead_sec',
    'preview_tray_length_mm',
    'preview_tray_width_mm',
)


def _tray_intercept_pythonpath() -> str:
    paths = []
    for parent in Path(__file__).resolve().parents:
        build_path = parent / 'build' / 'tray_intercept'
        source_path = parent / 'src' / 'tray_intercept'
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
            package='tray_intercept',
            executable='tray_intercept',
            name='tray_intercept',
            output='screen',
            parameters=[params] if params else [],
            additional_env={
                'PYTHONPATH': _tray_intercept_pythonpath(),
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
