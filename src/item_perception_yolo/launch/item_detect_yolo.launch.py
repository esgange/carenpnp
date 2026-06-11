import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BIN_CAMERA_COLOR_TOPIC = "/bin_camera/color/image_raw"
BIN_CAMERA_DEPTH_TOPIC = "/bin_camera/depth/image_raw"
BIN_CAMERA_INFO_TOPIC = "/bin_camera/color/camera_info"


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
        print(f"[item_detect_yolo.launch] Could not open GUI dialog: {exc}")
        print(message)


def _unquote_config_value(value: str) -> str:
    text = str(value or "").strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        text = text[1:-1]
    return text.strip()


def _station_config_value(*keys: str) -> str:
    try:
        values = {}
        with Path(_repo_path("station_config")).open("r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = _unquote_config_value(value)
    except OSError:
        return ""

    for key in keys:
        value = values.get(key)
        if value:
            return value
    return ""


def _resolve_robot_ip_address(value: str = "") -> str:
    requested = str(value or "").strip()
    if requested:
        return requested
    env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
    if env_ip:
        return env_ip
    return _station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def _sanitize_filename_token(value: str) -> str:
    token = []
    previous_underscore = False
    for ch in str(value or "").strip():
        if ch.isalnum() or ch in "._-":
            token.append(ch)
            previous_underscore = False
        elif not previous_underscore:
            token.append("_")
            previous_underscore = True
    return "".join(token).strip("_")


def _looks_like_ip_token(token: str) -> bool:
    return "." in token and all(ch.isdigit() or ch == "." for ch in token)


def _classify_robot_file(path: Path, robot_ip_address: str) -> str:
    ip_token = _sanitize_filename_token(robot_ip_address)
    if not ip_token:
        return "legacy"
    stem = path.stem
    if stem.endswith(f"_{ip_token}"):
        return "exact"
    last_token = stem.rsplit("_", 1)[-1]
    if _looks_like_ip_token(last_token):
        return "different"
    return "legacy"


def _find_latest_calibration(calibration_dir: str, robot_ip_address: str = "") -> str:
    try:
        base = Path(calibration_dir).expanduser()
        if not base.exists() or not base.is_dir():
            return ""
        exact_files = []
        legacy_files = []
        for path in base.iterdir():
            if not path.is_file() or path.suffix != ".yaml" or path.stat().st_size <= 0:
                continue
            name = path.name
            if name.startswith("axab_calibration_eyetohand_"):
                classification = _classify_robot_file(path, robot_ip_address)
                if classification == "exact":
                    exact_files.append(path)
                elif classification == "legacy":
                    legacy_files.append(path)
        candidates = exact_files or legacy_files
        if not candidates:
            return ""
        latest = max(candidates, key=lambda p: p.stat().st_mtime)
        if not exact_files and robot_ip_address:
            print(
                "[item_detect_yolo.launch] No eye-to-hand calibration tagged for robot IP "
                f"{robot_ip_address}; using legacy unsuffixed file: {latest}"
            )
        return str(latest)
    except Exception as exc:
        print(f"[item_detect_yolo.launch] Failed to search calibrations in {calibration_dir}: {exc}")
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
    selected_model_path = os.path.expanduser(
        LaunchConfiguration("selected_model_path").perform(context).strip()
    )
    selected_profile_path = os.path.expanduser(
        LaunchConfiguration("selected_profile_path").perform(context).strip()
    )
    runtime_settings_file = os.path.expanduser(
        LaunchConfiguration("runtime_settings_file").perform(context).strip()
    )
    selected_model_export_file = os.path.expanduser(
        LaunchConfiguration("selected_model_export_file").perform(context).strip()
    )
    selected_profile_export_file = os.path.expanduser(
        LaunchConfiguration("selected_profile_export_file").perform(context).strip()
    )
    selected_profile_topic = LaunchConfiguration("selected_profile_topic").perform(context).strip()
    color_topic = LaunchConfiguration("color_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    publish_overlay = _to_bool(LaunchConfiguration("publish_overlay").perform(context))
    use_profile_camera_topics = _to_bool(
        LaunchConfiguration("use_profile_camera_topics").perform(context)
    )
    bin_pose_topic = LaunchConfiguration("bin_pose_topic").perform(context)
    bin_item_pose_array_topic = LaunchConfiguration("bin_item_pose_array_topic").perform(context)
    seek_service = LaunchConfiguration("seek_service").perform(context)
    repick_service = LaunchConfiguration("repick_service").perform(context)
    seek_complete_service = LaunchConfiguration("seek_complete_service").perform(context)
    seek_status_service = LaunchConfiguration("seek_status_service").perform(context)
    go_to_teach_service = LaunchConfiguration("go_to_teach_service").perform(context)
    movj_service = LaunchConfiguration("movj_service").perform(context)

    use_calibration = _to_bool(LaunchConfiguration("use_calibration").perform(context))
    parent_frame = LaunchConfiguration("parent_frame").perform(context)
    child_frame = LaunchConfiguration("child_frame").perform(context)
    calibration_dir = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
    calibration_file = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))
    robot_ip_address = _resolve_robot_ip_address(
        LaunchConfiguration("robot_ip_address").perform(context)
    )
    camera_frame_override = LaunchConfiguration("camera_frame").perform(context).strip()
    start_visualization = _to_bool(LaunchConfiguration("start_visualization").perform(context))
    headless = _to_bool(LaunchConfiguration("headless").perform(context))
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
                    "[item_detect_yolo.launch] calibration_file is set but missing/empty: "
                    f"{selected_file}"
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        else:
            selection = _calibration_selection_helper()
            if selection.requires_manual_selection(robot_ip_address):
                selected_file = selection.choose_required_calibration(
                    calibration_dir=calibration_dir,
                    filename_pattern="axab_calibration_eyetohand_*.yaml",
                    calibration_label="eye-to-hand calibration",
                    launch_label="item_detect_yolo.launch",
                    robot_ip_address=robot_ip_address,
                )
            else:
                selected_file = _find_latest_calibration(calibration_dir, robot_ip_address)
            if not selected_file:
                msg = (
                    "[item_detect_yolo.launch] No non-empty eye-to-hand calibration YAML "
                    f"for robot IP {robot_ip_address or 'auto'} found in "
                    f"{calibration_dir}. Provide one via calibration_file:=<path>."
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        print(f"[item_detect_yolo.launch] Using calibration file: {selected_file}")

    bin_camera_frame = camera_frame_override
    if not bin_camera_frame and use_calibration:
        bin_camera_frame = child_frame

    parameter_sources = [
        {
            "profiles_dir": profiles_dir,
            "model_root": model_root,
            "selected_model_path": selected_model_path,
            "selected_profile_path": selected_profile_path,
            "runtime_settings_file": runtime_settings_file,
            "selected_model_export_file": selected_model_export_file,
            "selected_profile_export_file": selected_profile_export_file,
            "selected_profile_topic": selected_profile_topic,
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "publish_overlay": publish_overlay,
            "use_profile_camera_topics": use_profile_camera_topics,
            "bin_pose_topic": bin_pose_topic,
            "bin_item_pose_array_topic": bin_item_pose_array_topic,
            "seek_service": seek_service,
            "repick_service": repick_service,
            "seek_complete_service": seek_complete_service,
            "seek_status_service": seek_status_service,
            "go_to_teach_service": go_to_teach_service,
            "movj_service": movj_service,
            "camera_frame": bin_camera_frame,
            "start_visualization": start_visualization,
            "headless": headless,
            "use_calibration": use_calibration,
            "publish_static_calibration_tf": use_calibration,
            "calibration_parent_frame": parent_frame,
            "calibration_child_frame": child_frame,
            "calibration_dir": calibration_dir,
            "calibration_file": selected_file,
            "robot_ip_address": robot_ip_address,
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
            default_value=_repo_path("teach", "item_teach_yolo"),
        ),
        DeclareLaunchArgument(
            "model_root",
            default_value=_repo_path("teach", "item_teach_yolo"),
        ),
        DeclareLaunchArgument(
            "selected_model_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "selected_profile_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "runtime_settings_file",
            default_value=_repo_path("config", "item_perception_yolo", "item_detect_yolo_runtime_settings.yaml"),
        ),
        DeclareLaunchArgument(
            "selected_model_export_file",
            default_value=_repo_path("config", "item_perception_yolo", "item_detect_yolo_selected_model.txt"),
        ),
        DeclareLaunchArgument(
            "selected_profile_export_file",
            default_value=_repo_path("config", "item_perception", "item_detect_selected_profile.txt"),
        ),
        DeclareLaunchArgument(
            "selected_profile_topic",
            default_value="item_detect/selected_profile",
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
            "publish_overlay",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_profile_camera_topics",
            default_value="true",
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
            "repick_service",
            default_value="item_detect/repick",
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
            default_value="base_link",
        ),
        DeclareLaunchArgument(
            "child_frame",
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
            description="Robot controller IP for calibration file discovery. Empty uses ROBOT_IP_ADDRESS/station_config.",
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
            "headless",
            default_value="false",
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
