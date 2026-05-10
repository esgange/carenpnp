import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
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
    calibration_mode = LaunchConfiguration('calibration_mode')
    color_topic = LaunchConfiguration('color_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='eye_on_hand',
        description='Calibration mode for GUI/calibrator: eye_on_hand or eye_to_hand.',
    )
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/color/image_raw',
        description='RGB image topic for the camera stream used during calibration.',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Depth image topic for the camera stream used during calibration.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='Camera info topic aligned with the RGB image stream used during calibration.',
    )

    gui = Node(
        package='camera_calibration',
        executable='camera_calibration_gui',
        name='camera_calibration_gui',
        output='screen',
        parameters=[{
            'calibration_mode': calibration_mode,
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
            'parent_frame': 'camera_link',
            'output_frame': 'tag_frame',
            'publish_rate': 20.0,
            'lookup_timeout': 0.05,
        }],
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aruco_perception'),
                         'launch', 'aruco_perception.launch.py')),
        launch_arguments={
            'use_calibration': 'false',
            'parent_frame': 'camera_link',
            'child_frame': 'calibrated_camera_link',
            'show_overlay_window': 'false',
            'color_topic': color_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
        }.items(),
    )

    return LaunchDescription([
        _ros_domain_action(),
        calibration_mode_arg,
        color_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        gui,
        calibration_tf,
        aruco_launch,
    ])
