import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def _to_int(value: str, name: str) -> int:
    try:
        return int(value.strip())
    except ValueError as exc:
        raise RuntimeError(f"{name} must be an integer, got: {value!r}") from exc


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
        print(f"[tray_detect.launch] Could not open GUI dialog: {exc}")
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


def _calibration_matches_robot_ip(path: Path, robot_ip_address: str) -> bool:
    ip_token = _sanitize_filename_token(robot_ip_address)
    if not ip_token:
        return False
    return path.stem.endswith(f"_{ip_token}")


def _find_latest_calibration(calibration_dir: str, robot_ip_address: str = "") -> str:
    try:
        robot_ip_address = str(robot_ip_address or "").strip()
        if not robot_ip_address:
            print("[tray_detect.launch] Robot IP is not set; cannot auto-select an exact calibration file.")
            return ""
        base = Path(calibration_dir).expanduser()
        if not base.exists() or not base.is_dir():
            return ""
        exact_files = []
        for path in base.iterdir():
            if not path.is_file() or path.suffix != ".yaml" or path.stat().st_size <= 0:
                continue
            name = path.name
            if name.startswith("axab_calibration_eyeonhand_"):
                if _calibration_matches_robot_ip(path, robot_ip_address):
                    exact_files.append(path)
        if not exact_files:
            return ""
        latest = max(exact_files, key=lambda p: p.stat().st_mtime)
        return str(latest)
    except Exception as exc:
        print(f"[tray_detect.launch] Failed to search calibrations in {calibration_dir}: {exc}")
        return ""


def _calibration_file_is_usable(path: str) -> bool:
    try:
        p = Path(path).expanduser()
        return p.exists() and p.is_file() and p.stat().st_size > 0
    except Exception:
        return False


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    profiles_dir = os.path.expanduser(LaunchConfiguration("profiles_dir").perform(context).strip())
    selected_profile_path = os.path.expanduser(
        LaunchConfiguration("selected_profile_path").perform(context).strip()
    )
    runtime_settings_file = os.path.expanduser(
        LaunchConfiguration("runtime_settings_file").perform(context).strip()
    )
    color_topic = LaunchConfiguration("color_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    overlay_topic = LaunchConfiguration("overlay_topic").perform(context)
    publish_overlay = _to_bool(LaunchConfiguration("publish_overlay").perform(context))
    camera_control_service_root = LaunchConfiguration("camera_control_service_root").perform(context)
    color_exposure_min_us = _to_int(
        LaunchConfiguration("color_exposure_min_us").perform(context),
        "color_exposure_min_us",
    )
    color_exposure_max_us = _to_int(
        LaunchConfiguration("color_exposure_max_us").perform(context),
        "color_exposure_max_us",
    )
    depth_exposure_min_us = _to_int(
        LaunchConfiguration("depth_exposure_min_us").perform(context),
        "depth_exposure_min_us",
    )
    depth_exposure_max_us = _to_int(
        LaunchConfiguration("depth_exposure_max_us").perform(context),
        "depth_exposure_max_us",
    )
    tray_pose_topic = LaunchConfiguration("tray_pose_topic").perform(context)
    tray_axis_overlay_topic = LaunchConfiguration("tray_axis_overlay_topic").perform(context)
    tray_vector_topic = LaunchConfiguration("tray_vector_topic").perform(context)

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

    selected_file = ""
    if use_calibration:
        if calibration_file:
            selected_file = calibration_file
            if not _calibration_file_is_usable(selected_file):
                msg = (
                    "[tray_detect.launch] calibration_file is set but missing/empty: "
                    f"{selected_file}"
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        else:
            selection = _calibration_selection_helper()
            if selection.requires_manual_selection(robot_ip_address):
                selected_file = selection.choose_required_calibration(
                    calibration_dir=calibration_dir,
                    filename_pattern="axab_calibration_eyeonhand_*.yaml",
                    calibration_label="eye-on-hand calibration",
                    launch_label="tray_detect.launch",
                    robot_ip_address=robot_ip_address,
                )
            else:
                selected_file = _find_latest_calibration(calibration_dir, robot_ip_address)
            if not selected_file:
                if robot_ip_address:
                    msg = (
                        "[tray_detect.launch] No non-empty eye-on-hand calibration YAML "
                        f"tagged for robot IP {robot_ip_address} found in {calibration_dir}. "
                        "Provide one via calibration_file:=<path>."
                    )
                else:
                    msg = (
                        "[tray_detect.launch] No robot IP was resolved for calibration "
                        "auto-selection. Provide robot_ip_address:=<ip> or "
                        "calibration_file:=<path>."
                    )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        print(f"[tray_detect.launch] Using calibration file: {selected_file}")

    tray_camera_frame = camera_frame_override
    if not tray_camera_frame and use_calibration:
        tray_camera_frame = child_frame

    parameter_sources = []
    if params_file and Path(os.path.expanduser(params_file)).exists():
        parameter_sources.append(params_file)
    parameter_sources.append(
        {
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "overlay_topic": overlay_topic,
            "publish_overlay": publish_overlay,
            "camera_control_service_root": camera_control_service_root,
            "color_exposure_min_us": color_exposure_min_us,
            "color_exposure_max_us": color_exposure_max_us,
            "depth_exposure_min_us": depth_exposure_min_us,
            "depth_exposure_max_us": depth_exposure_max_us,
            "tray_pose_topic": tray_pose_topic,
            "tray_axis_overlay_topic": tray_axis_overlay_topic,
            "tray_vector_topic": tray_vector_topic,
            "profiles_dir": profiles_dir,
            "selected_profile_path": selected_profile_path,
            "runtime_settings_file": runtime_settings_file,
            "camera_frame": tray_camera_frame,
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
        }
    )

    return [
        Node(
            package="tray_perception",
            executable="tray_detect_node",
            name="tray_detect",
            output="screen",
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
            default_value=_repo_path("teach", "tray_teach"),
        ),
        DeclareLaunchArgument(
            "selected_profile_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "runtime_settings_file",
            default_value=_repo_path("config", "tray_perception", "tray_detect_runtime_settings.yaml"),
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value="/robot_camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/robot_camera/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/robot_camera/color/camera_info",
        ),
        DeclareLaunchArgument(
            "overlay_topic",
            default_value="tray_overlay",
        ),
        DeclareLaunchArgument(
            "publish_overlay",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "camera_control_service_root",
            default_value="/robot_camera",
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
            "tray_pose_topic",
            default_value="tray_pose",
        ),
        DeclareLaunchArgument(
            "tray_axis_overlay_topic",
            default_value="tray_axis_overlay",
        ),
        DeclareLaunchArgument(
            "tray_vector_topic",
            default_value="tray_vector",
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
            default_value="arm_calibrated_camera_link",
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
        OpaqueFunction(function=_launch_setup),
    ])
