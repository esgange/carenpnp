import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def _normalize_calibration_mode(value):
    mode = str(value or '').strip().lower().replace('-', '_')
    if mode in ('eye_to_hand', 'eyetohand'):
        return 'eye_to_hand'
    return 'eye_on_hand'


def _default_camera_prefix(calibration_mode):
    return 'bin_camera' if _normalize_calibration_mode(calibration_mode) == 'eye_to_hand' else 'robot_camera'


def _default_camera_frame(calibration_mode, camera_prefix):
    if _normalize_calibration_mode(calibration_mode) == 'eye_to_hand':
        prefix = str(camera_prefix or '').strip().strip('/') or 'bin_camera'
        return f'{prefix}_color_optical_frame'
    return 'camera_link'


def _topic_for_prefix(camera_prefix, suffix):
    prefix = str(camera_prefix or '').strip().strip('/')
    if not prefix:
        prefix = 'robot_camera'
    return f'/{prefix}/{suffix}'

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


def _launch_setup(context, *args, **kwargs):
    calibration_mode_value = _normalize_calibration_mode(
        LaunchConfiguration('calibration_mode').perform(context)
    )
    camera_prefix = LaunchConfiguration('camera_prefix').perform(context).strip().strip('/')
    if not camera_prefix or camera_prefix.lower() == 'auto':
        camera_prefix = _default_camera_prefix(calibration_mode_value)

    color_topic_value = LaunchConfiguration('color_topic').perform(context).strip()
    depth_topic_value = LaunchConfiguration('depth_topic').perform(context).strip()
    camera_info_topic_value = LaunchConfiguration('camera_info_topic').perform(context).strip()
    if not color_topic_value:
        color_topic_value = _topic_for_prefix(camera_prefix, 'color/image_raw')
    if not depth_topic_value:
        depth_topic_value = _topic_for_prefix(camera_prefix, 'depth/image_raw')
    if not camera_info_topic_value:
        camera_info_topic_value = _topic_for_prefix(camera_prefix, 'color/camera_info')
    camera_frame_value = LaunchConfiguration('camera_frame').perform(context).strip()
    if not camera_frame_value or camera_frame_value.lower() == 'auto':
        camera_frame_value = _default_camera_frame(calibration_mode_value, camera_prefix)

    print(
        '[camera_calibration.launch] '
        f'mode={calibration_mode_value} camera_prefix={camera_prefix} '
        f'color_topic={color_topic_value} depth_topic={depth_topic_value} '
        f'camera_info_topic={camera_info_topic_value} camera_frame={camera_frame_value}'
    )

    gui = Node(
        package='camera_calibration',
        executable='camera_calibration_gui',
        name='camera_calibration_gui',
        output='screen',
        parameters=[{
            'calibration_mode': calibration_mode_value,
            'camera_prefix': camera_prefix,
            'camera_frame': camera_frame_value,
            'color_topic': color_topic_value,
            'depth_topic': depth_topic_value,
            'camera_info_topic': camera_info_topic_value,
        }],
    )

    calibration_tf = Node(
        package='camera_calibration',
        executable='calibration_perception',
        name='calibration_perception',
        output='screen',
        parameters=[{
            'marker_prefix': 'aruco_marker',
            'marker_ids': [1, 2, 3, 4],
            'parent_frame': camera_frame_value,
            'output_frame': 'tag_frame',
            'publish_rate': 20.0,
            'lookup_timeout': 0.05,
            'max_marker_age_sec': 1.5,
        }],
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('aruco_perception'),
                'launch',
                'aruco_perception.launch.py')),
        launch_arguments={
            'use_calibration': 'false',
            'parent_frame': camera_frame_value,
            'child_frame': 'calibrated_camera_link',
            'show_overlay_window': 'false',
            'color_topic': color_topic_value,
            'depth_topic': depth_topic_value,
            'camera_info_topic': camera_info_topic_value,
        }.items(),
    )
    return [gui, calibration_tf, aruco_launch]


def generate_launch_description():
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='eye_on_hand',
        description='Calibration mode for GUI/calibrator: eye_on_hand or eye_to_hand.',
    )
    camera_prefix_arg = DeclareLaunchArgument(
        'camera_prefix',
        default_value='auto',
        description='Camera topic prefix. auto selects robot_camera for eye_on_hand and bin_camera for eye_to_hand.',
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='auto',
        description='Raw camera TF frame for calibration. auto uses camera_link for eye_on_hand and <prefix>_color_optical_frame for eye_to_hand.',
    )
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='',
        description='Explicit RGB image topic override. Empty derives from camera_prefix.',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='',
        description='Explicit depth image topic override. Empty derives from camera_prefix.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='',
        description='Explicit camera info topic override. Empty derives from camera_prefix.',
    )
    return LaunchDescription([
        _ros_domain_action(),
        calibration_mode_arg,
        camera_prefix_arg,
        camera_frame_arg,
        color_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        OpaqueFunction(function=_launch_setup),
    ])
