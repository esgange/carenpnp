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
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_calibration = LaunchConfiguration("use_calibration")
    calibration_parent_frame = LaunchConfiguration("calibration_parent_frame")
    calibration_child_frame = LaunchConfiguration("calibration_child_frame")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")
    target_frame = LaunchConfiguration("target_frame")
    marker_parent_frame = LaunchConfiguration("marker_parent_frame")
    base_frame = LaunchConfiguration("base_frame")
    gripper_frame = LaunchConfiguration("gripper_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    use_platform_calibration = LaunchConfiguration("use_platform_calibration")
    auto_discover_platform_calibration = LaunchConfiguration("auto_discover_platform_calibration")
    platform_calibration_dir = LaunchConfiguration("platform_calibration_dir")
    platform_calibration_file = LaunchConfiguration("platform_calibration_file")
    marker_prefix = LaunchConfiguration("marker_prefix")
    overlay_topic = LaunchConfiguration("overlay_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    bin_teach_dir = LaunchConfiguration("bin_teach_dir")
    output_dir = LaunchConfiguration("output_dir")
    bin_name = LaunchConfiguration("bin_name")
    show_aruco_overlay = LaunchConfiguration("show_aruco_overlay")
    publish_aruco_overlay = LaunchConfiguration("publish_aruco_overlay")
    aruco_overlay_rate_hz = LaunchConfiguration("aruco_overlay_rate_hz")
    use_aruco_overlay = LaunchConfiguration("use_aruco_overlay")
    motion_service_root = LaunchConfiguration("motion_service_root")
    align_distance_mm = LaunchConfiguration("align_distance_mm")
    align_pose_speed_percent = LaunchConfiguration("align_pose_speed_percent")
    align_visible_max_age_sec = LaunchConfiguration("align_visible_max_age_sec")
    align_initial_timeout_sec = LaunchConfiguration("align_initial_timeout_sec")
    align_min_base_z_mm = LaunchConfiguration("align_min_base_z_mm")
    align_goal_pos_tol_mm = LaunchConfiguration("align_goal_pos_tol_mm")
    align_goal_rot_tol_deg = LaunchConfiguration("align_goal_rot_tol_deg")
    align_up_max_distance_mm = LaunchConfiguration("align_up_max_distance_mm")
    align_up_speed_factor_percent = LaunchConfiguration("align_up_speed_factor_percent")
    align_up_timeout_sec = LaunchConfiguration("align_up_timeout_sec")
    align_up_user_index = LaunchConfiguration("align_up_user_index")
    align_restore_speed_factor_percent = LaunchConfiguration("align_restore_speed_factor_percent")

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("aruco_perception"),
                "launch",
                "aruco_perception.launch.py",
            )
        ),
        launch_arguments={
            "use_calibration": use_calibration,
            "parent_frame": calibration_parent_frame,
            "child_frame": calibration_child_frame,
            "calibration_dir": calibration_dir,
            "calibration_file": calibration_file,
            "show_overlay_window": show_aruco_overlay,
            "publish_overlay": publish_aruco_overlay,
            "overlay_rate_hz": aruco_overlay_rate_hz,
            "detections_topic": detections_topic,
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
        }.items(),
    )

    bin_teach = Node(
        package="item_perception_yolo",
        executable="bin_teach",
        name="bin_teach",
        output="screen",
        parameters=[{
            "parent_frame": marker_parent_frame,
            "target_frame": target_frame,
            "base_frame": base_frame,
            "gripper_frame": gripper_frame,
            "camera_frame": camera_frame,
            "use_platform_calibration": use_platform_calibration,
            "auto_discover_platform_calibration": auto_discover_platform_calibration,
            "platform_calibration_dir": platform_calibration_dir,
            "platform_calibration_file": platform_calibration_file,
            "color_topic": color_topic,
            "marker_prefix": marker_prefix,
            "overlay_topic": overlay_topic,
            "use_aruco_overlay": use_aruco_overlay,
            "detections_topic": detections_topic,
            "bin_teach_dir": bin_teach_dir,
            "output_dir": output_dir,
            "bin_name": bin_name,
            "motion_service_root": motion_service_root,
            "align_distance_mm": align_distance_mm,
            "align_pose_speed_percent": align_pose_speed_percent,
            "align_visible_max_age_sec": align_visible_max_age_sec,
            "align_initial_timeout_sec": align_initial_timeout_sec,
            "align_min_base_z_mm": align_min_base_z_mm,
            "align_goal_pos_tol_mm": align_goal_pos_tol_mm,
            "align_goal_rot_tol_deg": align_goal_rot_tol_deg,
            "align_up_max_distance_mm": align_up_max_distance_mm,
            "align_up_speed_factor_percent": align_up_speed_factor_percent,
            "align_up_timeout_sec": align_up_timeout_sec,
            "align_up_user_index": align_up_user_index,
            "align_restore_speed_factor_percent": align_restore_speed_factor_percent,
        }],
    )

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument("color_topic", default_value="/robot_camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/robot_camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/robot_camera/color/camera_info"),
        DeclareLaunchArgument("use_calibration", default_value="true"),
        DeclareLaunchArgument("calibration_parent_frame", default_value="Link6"),
        DeclareLaunchArgument("calibration_child_frame", default_value="calibrated_camera_link"),
        DeclareLaunchArgument("calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("target_frame", default_value="bin_teach_target"),
        DeclareLaunchArgument("marker_parent_frame", default_value="calibrated_camera_link"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("gripper_frame", default_value="Link6"),
        DeclareLaunchArgument("camera_frame", default_value="calibrated_camera_link"),
        DeclareLaunchArgument("use_platform_calibration", default_value="true"),
        DeclareLaunchArgument("auto_discover_platform_calibration", default_value="true"),
        DeclareLaunchArgument("platform_calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("platform_calibration_file", default_value=""),
        DeclareLaunchArgument("marker_prefix", default_value="aruco_marker"),
        DeclareLaunchArgument("overlay_topic", default_value="/aruco_overlay"),
        DeclareLaunchArgument("detections_topic", default_value="/aruco_detections"),
        DeclareLaunchArgument(
            "bin_teach_dir",
            default_value=_repo_path("teach", "bin_teach"),
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=bin_teach_dir,
        ),
        DeclareLaunchArgument("bin_name", default_value="bin"),
        DeclareLaunchArgument("show_aruco_overlay", default_value="false"),
        DeclareLaunchArgument("publish_aruco_overlay", default_value="true"),
        DeclareLaunchArgument("aruco_overlay_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("use_aruco_overlay", default_value="true"),
        DeclareLaunchArgument("motion_service_root", default_value="/dobot_bringup_ros2/srv"),
        DeclareLaunchArgument("align_distance_mm", default_value="300.0"),
        DeclareLaunchArgument("align_pose_speed_percent", default_value="100"),
        DeclareLaunchArgument("align_visible_max_age_sec", default_value="0.75"),
        DeclareLaunchArgument("align_initial_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("align_min_base_z_mm", default_value="200.0"),
        DeclareLaunchArgument("align_goal_pos_tol_mm", default_value="8.0"),
        DeclareLaunchArgument("align_goal_rot_tol_deg", default_value="3.0"),
        DeclareLaunchArgument("align_up_max_distance_mm", default_value="400.0"),
        DeclareLaunchArgument("align_up_speed_factor_percent", default_value="5"),
        DeclareLaunchArgument("align_up_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument("align_up_user_index", default_value="0"),
        DeclareLaunchArgument("align_restore_speed_factor_percent", default_value="100"),
        aruco_launch,
        bin_teach,
    ])
