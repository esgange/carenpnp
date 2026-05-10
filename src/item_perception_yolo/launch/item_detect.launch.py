import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def _to_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _show_missing_calibration_dialog(message: str) -> None:
    try:
        import tkinter as tk
        from tkinter import messagebox

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        messagebox.showerror("Calibration File Missing", message + "\n\nClick OK to close launch.")
        root.destroy()
    except Exception as exc:
        print(f"[item_detect.launch] Could not open GUI dialog: {exc}")
        print(message)


def _find_latest_calibration(calibration_dir: str) -> str:
    try:
        base = Path(calibration_dir).expanduser()
        if not base.exists() or not base.is_dir():
            return ""
        yaml_files = [
            p for p in base.iterdir()
            if p.is_file() and p.suffix == ".yaml" and p.stat().st_size > 0
        ]
        if not yaml_files:
            return ""
        latest = max(yaml_files, key=lambda p: p.stat().st_mtime)
        return str(latest)
    except Exception as exc:
        print(f"[item_detect.launch] Failed to search calibrations in {calibration_dir}: {exc}")
        return ""


def _calibration_file_is_usable(path: str) -> bool:
    try:
        p = Path(path).expanduser()
        return p.exists() and p.is_file() and p.stat().st_size > 0
    except Exception:
        return False


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    profiles_dir = LaunchConfiguration("profiles_dir").perform(context)
    model_root = LaunchConfiguration("model_root").perform(context)
    color_topic = LaunchConfiguration("color_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    bin_pose_topic = LaunchConfiguration("bin_pose_topic").perform(context)
    bin_item_pose_array_topic = LaunchConfiguration("bin_item_pose_array_topic").perform(context)
    seek_service = LaunchConfiguration("seek_service").perform(context)
    seek_complete_service = LaunchConfiguration("seek_complete_service").perform(context)
    seek_status_service = LaunchConfiguration("seek_status_service").perform(context)
    go_to_teach_service = LaunchConfiguration("go_to_teach_service").perform(context)
    movj_service = LaunchConfiguration("movj_service").perform(context)

    use_calibration = _to_bool(LaunchConfiguration("use_calibration").perform(context))
    parent_frame = LaunchConfiguration("parent_frame").perform(context)
    child_frame = LaunchConfiguration("child_frame").perform(context)
    calibration_dir = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
    calibration_file = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))
    camera_frame_override = LaunchConfiguration("camera_frame").perform(context).strip()
    start_visualization = _to_bool(LaunchConfiguration("start_visualization").perform(context))
    align_item_z_axis_to_depth_plane = _to_bool(
        LaunchConfiguration("align_item_z_axis_to_depth_plane").perform(context)
    )
    python_executable = LaunchConfiguration("python_executable").perform(context)
    yolo_imgsz = LaunchConfiguration("yolo_imgsz")
    yolo_conf = LaunchConfiguration("yolo_conf")
    yolo_iou = LaunchConfiguration("yolo_iou")
    max_inference_hz = LaunchConfiguration("max_inference_hz")
    onnxruntime_threads = LaunchConfiguration("onnxruntime_threads")
    seek_window_sec = LaunchConfiguration("seek_window_sec")
    seek_decay_sec = LaunchConfiguration("seek_decay_sec")

    selected_file = ""
    if use_calibration:
        if calibration_file:
            selected_file = calibration_file
            if not _calibration_file_is_usable(selected_file):
                msg = (
                    "[item_detect.launch] calibration_file is set but missing/empty: "
                    f"{selected_file}"
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        else:
            selected_file = _find_latest_calibration(calibration_dir)
            if not selected_file:
                msg = (
                    "[item_detect.launch] No non-empty calibration YAML found in "
                    f"{calibration_dir}. Provide one via calibration_file:=<path>."
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        print(f"[item_detect.launch] Using calibration file: {selected_file}")

    bin_camera_frame = camera_frame_override
    if not bin_camera_frame and use_calibration:
        bin_camera_frame = child_frame

    parameter_sources = [
        {
            "profiles_dir": profiles_dir,
            "model_root": model_root,
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "bin_pose_topic": bin_pose_topic,
            "bin_item_pose_array_topic": bin_item_pose_array_topic,
            "seek_service": seek_service,
            "seek_complete_service": seek_complete_service,
            "seek_status_service": seek_status_service,
            "go_to_teach_service": go_to_teach_service,
            "movj_service": movj_service,
            "camera_frame": bin_camera_frame,
            "start_visualization": start_visualization,
            "use_calibration": use_calibration,
            "publish_static_calibration_tf": use_calibration,
            "calibration_parent_frame": parent_frame,
            "calibration_child_frame": child_frame,
            "calibration_dir": calibration_dir,
            "calibration_file": selected_file,
            "auto_discover_calibration": False,
            "align_item_z_axis_to_depth_plane": align_item_z_axis_to_depth_plane,
            "yolo_imgsz": ParameterValue(yolo_imgsz, value_type=int),
            "yolo_conf": ParameterValue(yolo_conf, value_type=float),
            "yolo_iou": ParameterValue(yolo_iou, value_type=float),
            "max_inference_hz": ParameterValue(max_inference_hz, value_type=float),
            "onnxruntime_threads": ParameterValue(onnxruntime_threads, value_type=int),
            "seek_window_sec": ParameterValue(seek_window_sec, value_type=float),
            "seek_decay_sec": ParameterValue(seek_decay_sec, value_type=float),
        }
    ]
    if params_file:
        parameter_sources.insert(0, params_file)

    return [
        Node(
            package="item_perception_yolo",
            executable="item_detect_yolo_node.py",
            name="item_detect",
            output="screen",
            prefix=[python_executable, " "],
            parameters=parameter_sources,
        )
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
    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument(
            "params_file",
            default_value="",
        ),
        DeclareLaunchArgument(
            "profiles_dir",
            default_value=_repo_path("teach", "bins_yolo", "profiles"),
        ),
        DeclareLaunchArgument(
            "model_root",
            default_value=_repo_path("teach", "bins_yolo", "models"),
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
            "bin_pose_topic",
            default_value="bin_seek_pose",
        ),
        DeclareLaunchArgument(
            "bin_item_pose_array_topic",
            default_value="bin_item_poses",
        ),
        DeclareLaunchArgument(
            "seek_service",
            default_value="item_detect/seek",
        ),
        DeclareLaunchArgument(
            "seek_complete_service",
            default_value="item_detect/seek_complete",
        ),
        DeclareLaunchArgument(
            "seek_status_service",
            default_value="item_detect/seek_status",
        ),
        DeclareLaunchArgument(
            "go_to_teach_service",
            default_value="item_detect/go_to_teach",
        ),
        DeclareLaunchArgument(
            "movj_service",
            default_value="/dobot_bringup_ros2/srv/MovJ",
        ),
        DeclareLaunchArgument(
            "use_calibration",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "parent_frame",
            default_value="Link6",
        ),
        DeclareLaunchArgument(
            "child_frame",
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
            "camera_frame",
            default_value="",
        ),
        DeclareLaunchArgument(
            "start_visualization",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "align_item_z_axis_to_depth_plane",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "python_executable",
            default_value=_repo_path(".venv", "bin", "python"),
        ),
        DeclareLaunchArgument("yolo_imgsz", default_value="640"),
        DeclareLaunchArgument("yolo_conf", default_value="0.35"),
        DeclareLaunchArgument("yolo_iou", default_value="0.45"),
        DeclareLaunchArgument("max_inference_hz", default_value="8.0"),
        DeclareLaunchArgument("onnxruntime_threads", default_value="0"),
        DeclareLaunchArgument("seek_window_sec", default_value="60.0"),
        DeclareLaunchArgument("seek_decay_sec", default_value="1.0"),
        OpaqueFunction(function=_launch_setup),
    ])
