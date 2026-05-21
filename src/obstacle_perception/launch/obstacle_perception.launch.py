import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def _workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists() and
            (
                (path / "README.md").exists()
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
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    color_topic = LaunchConfiguration("color_topic")
    enable_memory = LaunchConfiguration("enable_memory")
    memory_voxel_size = LaunchConfiguration("memory_voxel_size")
    memory_decay = LaunchConfiguration("memory_decay")
    memory_max_voxels = LaunchConfiguration("memory_max_voxels")
    memory_color_r = LaunchConfiguration("memory_color_r")
    memory_color_g = LaunchConfiguration("memory_color_g")
    memory_color_b = LaunchConfiguration("memory_color_b")
    memory_min_hits = LaunchConfiguration("memory_min_hits")
    target_frame = LaunchConfiguration("target_frame")
    frame_id_override = LaunchConfiguration("frame_id_override")
    voxel_size = LaunchConfiguration("voxel_size")
    pixel_stride = LaunchConfiguration("pixel_stride")
    min_range = LaunchConfiguration("min_range")
    max_range = LaunchConfiguration("max_range")
    marker_lifetime = LaunchConfiguration("marker_lifetime")
    min_points_per_voxel = LaunchConfiguration("min_points_per_voxel")
    publish_pointcloud = LaunchConfiguration("publish_pointcloud")
    publish_markers = LaunchConfiguration("publish_markers")
    memory_publish_rate = LaunchConfiguration("memory_publish_rate")
    frustum_enable = LaunchConfiguration("frustum_enable")
    frustum_frame = LaunchConfiguration("frustum_frame")
    frustum_near = LaunchConfiguration("frustum_near")
    frustum_far = LaunchConfiguration("frustum_far")
    frustum_hfov_deg = LaunchConfiguration("frustum_hfov_deg")
    frustum_vfov_deg = LaunchConfiguration("frustum_vfov_deg")
    calibration_parent_frame = LaunchConfiguration("calibration_parent_frame")
    calibration_child_frame = LaunchConfiguration("calibration_child_frame")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")

    return LaunchDescription(
        [
            _ros_domain_action(),
            DeclareLaunchArgument(
                "color_topic", default_value="/robot_camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/robot_camera/depth/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/robot_camera/color/camera_info"
            ),
            DeclareLaunchArgument("enable_memory", default_value="true"),
            DeclareLaunchArgument("memory_voxel_size", default_value="0.03"),
            DeclareLaunchArgument("memory_decay", default_value="0.0"),
            DeclareLaunchArgument("memory_max_voxels", default_value="400000"),
            DeclareLaunchArgument("memory_color_r", default_value="0"),
            DeclareLaunchArgument("memory_color_g", default_value="100"),
            DeclareLaunchArgument("memory_color_b", default_value="255"),
            DeclareLaunchArgument("memory_min_hits", default_value="30"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("frame_id_override", default_value=""),
            DeclareLaunchArgument("memory_publish_rate", default_value="5.0"),
            DeclareLaunchArgument("memory_skip_live", default_value="true"),
            DeclareLaunchArgument("memory_blue_tint", default_value="0.3"),
            DeclareLaunchArgument("memory_skip_live_volume", default_value="true"),
            DeclareLaunchArgument("frustum_enable", default_value="true"),
            DeclareLaunchArgument("frustum_frame", default_value="calibrated_camera_link"),
            DeclareLaunchArgument("frustum_near", default_value="0.1"),
            DeclareLaunchArgument("frustum_far", default_value="3.0"),
            DeclareLaunchArgument("frustum_hfov_deg", default_value="65.0"),
            DeclareLaunchArgument("frustum_vfov_deg", default_value="50.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.03"),
            DeclareLaunchArgument("pixel_stride", default_value="4"),
            DeclareLaunchArgument("min_range", default_value="0.15"),
            DeclareLaunchArgument("max_range", default_value="2.5"),
            DeclareLaunchArgument("marker_lifetime", default_value="300.0"),
            DeclareLaunchArgument("min_points_per_voxel", default_value="3"),
            DeclareLaunchArgument("publish_pointcloud", default_value="true"),
            DeclareLaunchArgument("publish_markers", default_value="true"),
            DeclareLaunchArgument("calibration_parent_frame", default_value="Link6"),
            DeclareLaunchArgument("calibration_child_frame", default_value="calibrated_camera_link"),
            DeclareLaunchArgument(
                "calibration_dir", default_value=_repo_path("calibration")
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description="Optional explicit calibration YAML; empty means auto-discover newest eye-on-hand calibration in calibration_dir.",
            ),
            IncludeLaunchDescription(
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
                    "color_topic": color_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                }.items(),
            ),
            Node(
                package="obstacle_perception",
                executable="obstacle_perception_node",
                name="obstacle_perception",
                output="screen",
                parameters=[
                    {
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "color_topic": color_topic,
                        "voxel_size": voxel_size,
                        "pixel_stride": pixel_stride,
                        "min_range": min_range,
                        "max_range": max_range,
                        "marker_lifetime": marker_lifetime,
                        "min_points_per_voxel": min_points_per_voxel,
                        "publish_pointcloud": publish_pointcloud,
                        "publish_markers": publish_markers,
                    }
                ],
            ),
            Node(
                condition=IfCondition(enable_memory),
                package="obstacle_perception",
                executable="obstacle_memory_node",
                name="obstacle_memory",
                output="screen",
                parameters=[
                    {
                        "input_cloud_topic": "/obstacles/points",
                        "output_cloud_topic": "/obstacles/memory_points",
                        "voxel_size": memory_voxel_size,
                        "decay_seconds": memory_decay,
                        "max_voxels": memory_max_voxels,
                        "publish_rate": memory_publish_rate,
                        "color_r": memory_color_r,
                        "color_g": memory_color_g,
                        "color_b": memory_color_b,
                        "min_hits": memory_min_hits,
                        "target_frame": target_frame,
                        "frame_id_override": frame_id_override,
                        "skip_if_live": LaunchConfiguration("memory_skip_live"),
                        "blue_tint": LaunchConfiguration("memory_blue_tint"),
                        "skip_live_volume": LaunchConfiguration("memory_skip_live_volume"),
                        "frustum_enable": frustum_enable,
                        "frustum_frame": frustum_frame,
                        "frustum_near": frustum_near,
                        "frustum_far": frustum_far,
                        "frustum_hfov_deg": frustum_hfov_deg,
                        "frustum_vfov_deg": frustum_vfov_deg,
                    }
                ],
            ),
        ]
    )
