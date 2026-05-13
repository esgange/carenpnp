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
        print(f"[tray_detect.launch] Could not open GUI dialog: {exc}")
        print(message)


def _find_latest_calibration(calibration_dir: str) -> str:
    try:
        base = Path(calibration_dir).expanduser()
        if not base.exists() or not base.is_dir():
            return ""
        yaml_files = []
        for path in base.iterdir():
            if not path.is_file() or path.suffix != ".yaml" or path.stat().st_size <= 0:
                continue
            name = path.name
            if name.startswith("axab_calibration_eyeonhand_"):
                yaml_files.append(path)
        if not yaml_files:
            return ""
        latest = max(yaml_files, key=lambda p: p.stat().st_mtime)
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
    color_topic = LaunchConfiguration("color_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    tray_pose_topic = LaunchConfiguration("tray_pose_topic").perform(context)
    tray_vector_topic = LaunchConfiguration("tray_vector_topic").perform(context)

    use_calibration = _to_bool(LaunchConfiguration("use_calibration").perform(context))
    parent_frame = LaunchConfiguration("parent_frame").perform(context)
    child_frame = LaunchConfiguration("child_frame").perform(context)
    calibration_dir = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
    calibration_file = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))
    camera_frame_override = LaunchConfiguration("camera_frame").perform(context).strip()
    start_visualization = _to_bool(LaunchConfiguration("start_visualization").perform(context))

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
            selected_file = _find_latest_calibration(calibration_dir)
            if not selected_file:
                msg = (
                    "[tray_detect.launch] No non-empty calibration YAML found in "
                    f"{calibration_dir}. Provide one via calibration_file:=<path>."
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        print(f"[tray_detect.launch] Using calibration file: {selected_file}")

    tray_camera_frame = camera_frame_override
    if not tray_camera_frame and use_calibration:
        tray_camera_frame = child_frame

    return [
        Node(
            package="tray_perception",
            executable="tray_detect_node",
            name="tray_detect",
            output="screen",
            parameters=[
                params_file,
                {
                    "color_topic": color_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "tray_pose_topic": tray_pose_topic,
                    "tray_vector_topic": tray_vector_topic,
                    "camera_frame": tray_camera_frame,
                    "start_visualization": start_visualization,
                    "use_calibration": use_calibration,
                    "publish_static_calibration_tf": use_calibration,
                    "calibration_parent_frame": parent_frame,
                    "calibration_child_frame": child_frame,
                    "calibration_dir": calibration_dir,
                    "calibration_file": selected_file,
                    "auto_discover_calibration": False,
                },
            ],
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
            default_value=_repo_path("config", "tray_perception", "tray_teach_settings.yaml"),
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
            "tray_pose_topic",
            default_value="tray_pose",
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
        OpaqueFunction(function=_launch_setup),
    ])
