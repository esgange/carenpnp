import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def repo_path(*parts: str) -> str:
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
      return str(Path(value).expanduser().resolve().joinpath(*parts))

  for start in (Path.cwd(), Path(__file__).resolve()):
    path = start.expanduser().resolve()
    if path.is_file():
      path = path.parent
    for candidate in (path, *path.parents):
      if looks_like_root(candidate):
        return str(candidate.joinpath(*parts))
  return str(Path.cwd().resolve().joinpath(*parts))


def show_missing_calibration_dialog(message: str) -> None:
  try:
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showerror("Calibration File Missing", message + "\n\nClick OK to close launch.")
    root.destroy()
  except Exception as exc:
    print(f"[aruco_perception.launch] Could not open GUI dialog: {exc}")
    print(message)

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
  return LaunchDescription(
    [
      _ros_domain_action(),
      DeclareLaunchArgument("use_calibration", default_value="true",
                            description="Require and load calibration YAML by default; set false only for raw-frame calibration workflow."),
      DeclareLaunchArgument("parent_frame", default_value="Link6",
                            description="Parent frame for calibrated camera."),
      DeclareLaunchArgument("child_frame", default_value="calibrated_camera_link",
                            description="Name of calibrated camera frame."),
      DeclareLaunchArgument("calibration_dir", default_value=repo_path("calibration"),
                            description="Directory to search for calibration YAMLs."),
      DeclareLaunchArgument("calibration_file", default_value="",
                            description="Explicit calibration YAML path (overrides discovery)."),
      DeclareLaunchArgument("calibration_file_prefix", default_value="axab_calibration_eyeonhand_",
                            description="Filename prefix used when auto-discovering calibration YAMLs."),
      DeclareLaunchArgument("show_overlay_window", default_value="true",
                            description="Show the detector's standalone OpenCV overlay window."),
      DeclareLaunchArgument("publish_overlay", default_value="true",
                            description="Publish the RGB/depth debug overlay image."),
      DeclareLaunchArgument("overlay_rate_hz", default_value="10.0",
                            description="Maximum overlay publish/window update rate; <=0 means camera rate."),
      DeclareLaunchArgument("detections_topic", default_value="/aruco_detections",
                            description="Current-frame ArUco detection output topic."),
      DeclareLaunchArgument("color_topic", default_value="/robot_camera/color/image_raw",
                            description="RGB image topic for ArUco detection."),
      DeclareLaunchArgument("depth_topic", default_value="/robot_camera/depth/image_raw",
                            description="Depth image topic for ArUco detection."),
      DeclareLaunchArgument("camera_info_topic", default_value="/robot_camera/color/camera_info",
                            description="Camera info topic for ArUco detection."),
      OpaqueFunction(function=launch_setup),
    ]
  )


def launch_setup(context, *args, **kwargs):
  use_calibration = LaunchConfiguration("use_calibration").perform(context).lower() == "true"
  parent_frame = LaunchConfiguration("parent_frame").perform(context)
  child_frame = LaunchConfiguration("child_frame").perform(context)
  calibration_dir = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
  explicit_file = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))
  calibration_file_prefix = LaunchConfiguration("calibration_file_prefix").perform(context).strip()
  show_overlay_window = LaunchConfiguration("show_overlay_window")
  publish_overlay = LaunchConfiguration("publish_overlay")
  overlay_rate_hz = LaunchConfiguration("overlay_rate_hz")
  detections_topic = LaunchConfiguration("detections_topic")
  color_topic = LaunchConfiguration("color_topic")
  depth_topic = LaunchConfiguration("depth_topic")
  camera_info_topic = LaunchConfiguration("camera_info_topic")

  selected_file = ""
  if use_calibration:
    if explicit_file:
      selected_file = explicit_file
      if not calibration_file_is_usable(selected_file):
        msg = (
          "[aruco_perception.launch] calibration_file is set but missing/empty: "
          f"{selected_file}"
        )
        show_missing_calibration_dialog(msg)
        raise RuntimeError(
          msg
        )
    else:
      selected_file = find_latest_calibration(calibration_dir, calibration_file_prefix)
      if not selected_file:
        msg = (
          f"[aruco_perception.launch] No non-empty calibration YAML matching {calibration_file_prefix!r} found in "
          f"{calibration_dir}. Provide one via calibration_file:=<path>."
        )
        show_missing_calibration_dialog(msg)
        raise RuntimeError(
          msg
        )
    print(f"[aruco_perception.launch] Using calibration file: {selected_file}")
    camera_frame = child_frame
  else:
    print(
      "[aruco_perception.launch] Calibration disabled (use_calibration:=false). "
      f"Marker poses will be published in frame '{parent_frame}'."
    )
    camera_frame = parent_frame

  nodes = [
    Node(
      package="aruco_perception",
      executable="aruco_detector_node",
      name="aruco_perception",
      output="screen",
      parameters=[{
        "camera_frame": camera_frame,
        "use_calibration": use_calibration,
        "publish_static_calibration_tf": use_calibration,
        "calibration_file": selected_file,
        "calibration_parent_frame": parent_frame,
        "calibration_child_frame": child_frame,
        "show_overlay_window": ParameterValue(show_overlay_window, value_type=bool),
        "publish_overlay": ParameterValue(publish_overlay, value_type=bool),
        "overlay_rate_hz": ParameterValue(overlay_rate_hz, value_type=float),
        "detections_topic": detections_topic,
        "color_topic": color_topic,
        "depth_topic": depth_topic,
        "camera_info_topic": camera_info_topic,
      }],
    )
  ]
  return nodes


def find_latest_calibration(calibration_dir: str, filename_prefix: str) -> str:
  try:
    base = Path(calibration_dir).expanduser()
    if not base.exists() or not base.is_dir():
      return ""
    yaml_files = []
    for path in base.iterdir():
      if not path.is_file() or path.suffix != ".yaml" or path.stat().st_size <= 0:
        continue
      name = path.name
      if name.startswith(filename_prefix):
        yaml_files.append(path)
    if not yaml_files:
      return ""
    latest = max(yaml_files, key=lambda p: p.stat().st_mtime)
    return str(latest)
  except Exception as exc:
    print(f"[aruco_perception.launch] Failed to search calibrations in {calibration_dir}: {exc}")
    return ""


def calibration_file_is_usable(path: str) -> bool:
  try:
    p = Path(path).expanduser()
    return p.exists() and p.is_file() and p.stat().st_size > 0
  except Exception:
    return False
