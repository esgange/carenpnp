import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists() and
            (
                (path / "README.md").exists()
                or (path / "docker-compose.yml").exists()
                or (path / "src" / "dobot_msgs_v4").exists()
            )
        )

    for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()

    for start in (Path.cwd(), Path(__file__).resolve()):
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
    return Path.cwd().resolve()


def _repo_path(*parts: str) -> str:
    return str(_workspace_root().joinpath(*parts))

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
    platform_name = LaunchConfiguration("platform_name")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    observed_board_frame = LaunchConfiguration("observed_board_frame")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    overlay_topic = LaunchConfiguration("overlay_topic")
    use_aruco_overlay = LaunchConfiguration("use_aruco_overlay")
    calibration_parent_frame = LaunchConfiguration("calibration_parent_frame")
    calibration_child_frame = LaunchConfiguration("calibration_child_frame")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")
    platform_calibration_dir = LaunchConfiguration("platform_calibration_dir")
    platform_calibration_file = LaunchConfiguration("platform_calibration_file")
    marker_prefix = LaunchConfiguration("marker_prefix")
    lookup_timeout = LaunchConfiguration("lookup_timeout")
    stability_window_sec = LaunchConfiguration("stability_window_sec")
    stability_translation_tolerance_m = LaunchConfiguration("stability_translation_tolerance_m")
    stability_rotation_tolerance_deg = LaunchConfiguration("stability_rotation_tolerance_deg")

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("aruco_perception"),
                "launch",
                "aruco_perception.launch.py",
            )
        ),
        launch_arguments={
            "use_calibration": "true",
            "parent_frame": calibration_parent_frame,
            "child_frame": calibration_child_frame,
            "calibration_dir": calibration_dir,
            "calibration_file": calibration_file,
            "show_overlay_window": "false",
            "publish_overlay": "true",
            "overlay_rate_hz": "10.0",
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
        }.items(),
    )

    platform_teach = Node(
        package="camera_calibration",
        executable="platform_teach",
        name="platform_teach",
        output="screen",
        parameters=[{
            "platform_name": platform_name,
            "base_frame": base_frame,
            "camera_frame": camera_frame,
            "observed_board_frame": observed_board_frame,
            "marker_prefix": marker_prefix,
            "marker_ids": [1, 2, 3, 4],
            "color_topic": color_topic,
            "overlay_topic": overlay_topic,
            "use_aruco_overlay": use_aruco_overlay,
            "platform_calibration_dir": platform_calibration_dir,
            "platform_calibration_file": platform_calibration_file,
            "lookup_timeout": lookup_timeout,
            "stability_window_sec": stability_window_sec,
            "stability_translation_tolerance_m": stability_translation_tolerance_m,
            "stability_rotation_tolerance_deg": stability_rotation_tolerance_deg,
            "delete_existing_on_save": True,
        }],
    )

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument("platform_name", default_value="robot_platform_1"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_frame", default_value="calibrated_camera_link"),
        DeclareLaunchArgument("observed_board_frame", default_value="platform_board_observed"),
        DeclareLaunchArgument("color_topic", default_value="/robot_camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/robot_camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/robot_camera/color/camera_info"),
        DeclareLaunchArgument("overlay_topic", default_value="/aruco_overlay"),
        DeclareLaunchArgument("use_aruco_overlay", default_value="true"),
        DeclareLaunchArgument("calibration_parent_frame", default_value="Link6"),
        DeclareLaunchArgument("calibration_child_frame", default_value="calibrated_camera_link"),
        DeclareLaunchArgument("calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("platform_calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("platform_calibration_file", default_value=""),
        DeclareLaunchArgument("marker_prefix", default_value="aruco_marker"),
        DeclareLaunchArgument("lookup_timeout", default_value="0.15"),
        DeclareLaunchArgument("stability_window_sec", default_value="1.0"),
        DeclareLaunchArgument("stability_translation_tolerance_m", default_value="0.001"),
        DeclareLaunchArgument("stability_rotation_tolerance_deg", default_value="1.0"),
        aruco_launch,
        platform_teach,
    ])
