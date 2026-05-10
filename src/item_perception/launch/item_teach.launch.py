import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    params_file = LaunchConfiguration("params_file")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_calibration = LaunchConfiguration("use_calibration")
    publish_static_calibration_tf = LaunchConfiguration("publish_static_calibration_tf")
    calibration_parent_frame = LaunchConfiguration("calibration_parent_frame")
    calibration_child_frame = LaunchConfiguration("calibration_child_frame")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")
    auto_discover_calibration = LaunchConfiguration("auto_discover_calibration")
    publish_item_pose_array = LaunchConfiguration("publish_item_pose_array")
    item_pose_array_topic = LaunchConfiguration("item_pose_array_topic")
    align_item_z_axis_to_depth_plane = LaunchConfiguration("align_item_z_axis_to_depth_plane")
    bin_teach_dir = LaunchConfiguration("bin_teach_dir")
    motion_service_root = LaunchConfiguration("motion_service_root")
    bin_roi_move_speed_percent = LaunchConfiguration("bin_roi_move_speed_percent")

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument(
            "params_file",
            default_value=_repo_path("src", "item_perception", "config", "item_teach.yaml"),
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value="/camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
        ),
        DeclareLaunchArgument(
            "use_calibration",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "publish_static_calibration_tf",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "calibration_parent_frame",
            default_value="Link6",
        ),
        DeclareLaunchArgument(
            "calibration_child_frame",
            default_value="calibrated_camera_link",
        ),
        DeclareLaunchArgument(
            "calibration_dir",
            default_value=_repo_path("calibration"),
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value="",
        ),
        DeclareLaunchArgument(
            "auto_discover_calibration",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "publish_item_pose_array",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "item_pose_array_topic",
            default_value="bin_item_poses",
        ),
        DeclareLaunchArgument(
            "align_item_z_axis_to_depth_plane",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "bin_teach_dir",
            default_value=_repo_path("teach", "bin_teach"),
        ),
        DeclareLaunchArgument(
            "motion_service_root",
            default_value="/dobot_bringup_ros2/srv",
        ),
        DeclareLaunchArgument(
            "bin_roi_move_speed_percent",
            default_value="100",
        ),
        Node(
            package="item_perception",
            executable="item_teach",
            name="item_teach",
            output="screen",
            parameters=[
                params_file,
                {
                    "color_topic": color_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "use_calibration": use_calibration,
                    "publish_static_calibration_tf": publish_static_calibration_tf,
                    "calibration_parent_frame": calibration_parent_frame,
                    "calibration_child_frame": calibration_child_frame,
                    "calibration_dir": calibration_dir,
                    "calibration_file": calibration_file,
                    "auto_discover_calibration": auto_discover_calibration,
                    "publish_item_pose_array": publish_item_pose_array,
                    "item_pose_array_topic": item_pose_array_topic,
                    "align_item_z_axis_to_depth_plane": align_item_z_axis_to_depth_plane,
                    "bin_teach_dir": bin_teach_dir,
                    "motion_service_root": motion_service_root,
                    "bin_roi_move_speed_percent": bin_roi_move_speed_percent,
                },
            ],
        ),
    ])
