import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    color_topic = LaunchConfiguration("color_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    bin_pose_topic = LaunchConfiguration("bin_pose_topic").perform(context)
    bin_item_pose_array_topic = LaunchConfiguration("bin_item_pose_array_topic").perform(context)

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
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "bin_pose_topic": bin_pose_topic,
            "bin_item_pose_array_topic": bin_item_pose_array_topic,
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
        }
    ]
    if params_file:
        parameter_sources.insert(0, params_file)

    return [
        Node(
            package="bin_perception",
            executable="item_detect",
            name="item_detect",
            output="screen",
            parameters=parameter_sources,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="",
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
            default_value="~/DOBOT_pickn_place/calibration",
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
        OpaqueFunction(function=_launch_setup),
    ])
