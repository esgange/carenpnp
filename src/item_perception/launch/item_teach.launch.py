import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BIN_CAMERA_COLOR_TOPIC = "/bin_camera/color/image_raw"
BIN_CAMERA_DEPTH_TOPIC = "/bin_camera/depth/image_raw"
BIN_CAMERA_INFO_TOPIC = "/bin_camera/color/camera_info"
BIN_CAMERA_CONTROL_SERVICE_ROOT = "/bin_camera"


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


def _calibration_selection_helper():
    import importlib.util

    helper_candidates = []
    for parent in Path(__file__).resolve().parents:
        helper_candidates.extend([
            parent / "src" / "dobot_bringup_v4" / "launch" / "calibration_selection.py",
            parent / "install" / "cr_robot_ros2" / "share" / "cr_robot_ros2" / "launch" / "calibration_selection.py",
            parent / "cr_robot_ros2" / "share" / "cr_robot_ros2" / "launch" / "calibration_selection.py",
            parent / "share" / "cr_robot_ros2" / "launch" / "calibration_selection.py",
        ])

    for helper_path in helper_candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location("_dobot_calibration_selection", helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

    raise RuntimeError("Could not find calibration_selection.py helper")


def _manual_calibration_setup(context, *args, **kwargs):
    del args, kwargs
    use_calibration = LaunchConfiguration("use_calibration").perform(context).strip().lower()
    explicit_file = LaunchConfiguration("calibration_file").perform(context).strip()
    if use_calibration not in ("1", "true", "yes", "on") or explicit_file:
        return []

    selection = _calibration_selection_helper()
    robot_ip_address = selection.resolve_robot_ip_address(
        LaunchConfiguration("robot_ip_address").perform(context)
    )
    if not selection.requires_manual_selection(robot_ip_address):
        return []

    selected_file = selection.choose_required_calibration(
        calibration_dir=LaunchConfiguration("calibration_dir").perform(context),
        filename_pattern="axab_calibration_eyetohand_*.yaml",
        calibration_label="eye-to-hand calibration",
        launch_label="item_teach.launch",
        robot_ip_address=robot_ip_address,
    )
    return [
        SetLaunchConfiguration("calibration_file", selected_file),
        SetLaunchConfiguration("auto_discover_calibration", "false"),
    ]


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
    use_calibration = ParameterValue(LaunchConfiguration("use_calibration"), value_type=bool)
    publish_static_calibration_tf = ParameterValue(
        LaunchConfiguration("publish_static_calibration_tf"),
        value_type=bool,
    )
    calibration_parent_frame = LaunchConfiguration("calibration_parent_frame")
    calibration_child_frame = LaunchConfiguration("calibration_child_frame")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")
    robot_ip_address = LaunchConfiguration("robot_ip_address")
    auto_discover_calibration = ParameterValue(
        LaunchConfiguration("auto_discover_calibration"),
        value_type=bool,
    )
    publish_item_pose_array = ParameterValue(
        LaunchConfiguration("publish_item_pose_array"),
        value_type=bool,
    )
    item_pose_array_topic = LaunchConfiguration("item_pose_array_topic")
    align_item_z_axis_to_depth_plane = ParameterValue(
        LaunchConfiguration("align_item_z_axis_to_depth_plane"),
        value_type=bool,
    )
    camera_control_service_root = LaunchConfiguration("camera_control_service_root")
    color_exposure_min_us = ParameterValue(LaunchConfiguration("color_exposure_min_us"), value_type=int)
    color_exposure_max_us = ParameterValue(LaunchConfiguration("color_exposure_max_us"), value_type=int)
    depth_exposure_min_us = ParameterValue(LaunchConfiguration("depth_exposure_min_us"), value_type=int)
    depth_exposure_max_us = ParameterValue(LaunchConfiguration("depth_exposure_max_us"), value_type=int)
    bin_teach_dir = LaunchConfiguration("bin_teach_dir")
    profiles_dir = LaunchConfiguration("profiles_dir")
    motion_service_root = LaunchConfiguration("motion_service_root")
    bin_roi_move_speed_percent = ParameterValue(
        LaunchConfiguration("bin_roi_move_speed_percent"),
        value_type=int,
    )

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("item_perception"),
                "config",
                "item_teach.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value=BIN_CAMERA_COLOR_TOPIC,
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value=BIN_CAMERA_DEPTH_TOPIC,
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value=BIN_CAMERA_INFO_TOPIC,
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
            default_value="base_link",
        ),
        DeclareLaunchArgument(
            "calibration_child_frame",
            default_value="bin_calibrated_camera_link",
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
            "robot_ip_address",
            default_value="",
            description="Robot controller IP used to decide whether manual calibration selection is required.",
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
            "camera_control_service_root",
            default_value=BIN_CAMERA_CONTROL_SERVICE_ROOT,
        ),
        DeclareLaunchArgument(
            "color_exposure_min_us",
            default_value="1",
        ),
        DeclareLaunchArgument(
            "color_exposure_max_us",
            default_value="100",
        ),
        DeclareLaunchArgument(
            "depth_exposure_min_us",
            default_value="1",
        ),
        DeclareLaunchArgument(
            "depth_exposure_max_us",
            default_value="32000",
        ),
        DeclareLaunchArgument(
            "bin_teach_dir",
            default_value=_repo_path("teach", "bin_teach"),
        ),
        DeclareLaunchArgument(
            "profiles_dir",
            default_value=_repo_path("teach", "item_teach"),
        ),
        DeclareLaunchArgument(
            "motion_service_root",
            default_value="/dobot_bringup_ros2/srv",
        ),
        DeclareLaunchArgument(
            "bin_roi_move_speed_percent",
            default_value="100",
        ),
        OpaqueFunction(function=_manual_calibration_setup),
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
                    "robot_ip_address": robot_ip_address,
                    "auto_discover_calibration": auto_discover_calibration,
                    "publish_item_pose_array": publish_item_pose_array,
                    "item_pose_array_topic": item_pose_array_topic,
                    "align_item_z_axis_to_depth_plane": align_item_z_axis_to_depth_plane,
                    "camera_control_service_root": camera_control_service_root,
                    "color_exposure_min_us": color_exposure_min_us,
                    "color_exposure_max_us": color_exposure_max_us,
                    "depth_exposure_min_us": depth_exposure_min_us,
                    "depth_exposure_max_us": depth_exposure_max_us,
                    "bin_teach_dir": bin_teach_dir,
                    "profiles_dir": profiles_dir,
                    "motion_service_root": motion_service_root,
                    "bin_roi_move_speed_percent": bin_roi_move_speed_percent,
                },
            ],
        ),
    ])
