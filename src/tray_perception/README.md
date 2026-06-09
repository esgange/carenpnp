# tray_perception

`tray_perception` provides the RGB-D teach and detect workflow for trays. A tray
profile is taught with `tray_teach_node`, then `tray_detect_node` loads the
profile to publish tray pose, tray velocity, and tray dimensions for downstream
motion packages.

## Nodes

| Node | Executable | Purpose |
| --- | --- | --- |
| `tray_teach` | `tray_teach_node` | Interactive OpenCV UI for teaching tray ROI, edge settings, and tray-plane reference. |
| `tray_detect` | `tray_detect_node` | Runtime detector for tray pose, seek confidence, tray vector, and dimensions service. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select tray_perception
source install/setup.bash
```

## Launch

Teach:

```bash
ros2 launch tray_perception tray_teach.launch.py
```

Detect:

```bash
ros2 launch tray_perception tray_detect.launch.py
```

Both tray launch files expose `profiles_dir`. `tray_detect.launch.py` also
exposes `selected_profile_path` and `runtime_settings_file`, so Robot Cell Orchestrator can
launch offline tray detection from a selected teach file or online detection
from the root `runtime/` handoff folder.

Production/service mode can run without the OpenCV operator window:

```bash
ros2 launch tray_perception tray_detect.launch.py headless:=true start_visualization:=false
```

Camera topic overrides:

```bash
ros2 launch tray_perception tray_detect.launch.py \
  color_topic:=/robot_camera/color/image_raw \
  depth_topic:=/robot_camera/depth/image_raw \
  camera_info_topic:=/robot_camera/color/camera_info
```

## Teach Workflow

1. Enter a tray name.
2. Select the view that makes the tray edge easiest to tune.
3. Add the tray ROI.
4. Tune thresholds, RGB exposure, and edge/ray settings.
5. Select the tray-plane ROI when prompted.
6. Verify the overlay and save the tray profile.

Saved profiles use RGB detection. Tray dimensions are measured from the RGB tray
corners projected onto the saved tray plane, then saved as `tray_width_mm` and
`tray_height_mm`. Runtime detection loads the saved plane instead of remeasuring
tray depth every frame.

RGB exposure uses the `RGB Exposure us` slider in `tray_teach`: `0` keeps camera
auto exposure enabled and `1-100` sends that value directly as microseconds.
Depth exposure remains auto. Saved tray profiles include the RGB exposure value,
and `tray_detect` applies it when the selected profile is loaded.

## Detect Behavior

`tray_detect_node`:

- loads valid tray profiles from the profiles directory;
- uses `Open Teach` to browse for a tray teach YAML file instead of selecting
  from an in-window dropdown;
- keeps final seek debug PNG capture off by default; `Debug Img` is not saved to
  runtime settings as a production/headless safety exception, because persisting
  it can flood the repo with debug images on unattended runs;
- uses RGB edge detection and the saved tray plane for metric dimensions/pose;
- publishes only after `seek_valid_frames_confidence` continuous valid frames;
- resets seek evidence after `seek_decay_sec` without a valid frame;
- publishes first/last seek artifacts for successful confidence runs;
- uses the lower-left tray corner as the pose and overlay origin;
- sets `+X` along the tray long side and `+Y` along the short side;
- derives `+Z` naturally from `X cross Y`, so Z may point up or down
  depending on the tray orientation in the image.
- keeps that lower-left origin fixed; seek direction no longer flips the axes.

## Calibration

`tray_detect.launch.py` loads calibration YAML directly and publishes the static
camera transform when calibration is enabled.

Defaults:

| Setting | Default |
| --- | --- |
| `use_calibration` | `true` |
| `parent_frame` | `Link6` |
| `child_frame` | `arm_calibrated_camera_link` |
| `calibration_dir` | `WORKSPACE_ROOT/calibration` |

If calibration is enabled and no usable YAML is available, launch shows an error
dialog and exits.

## Inputs

| Topic | Type |
| --- | --- |
| `/robot_camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/robot_camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/robot_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

## Outputs

| Output | Type | Notes |
| --- | --- | --- |
| `tray_overlay` | `sensor_msgs/msg/Image` | Runtime debug/preview image. |
| `tray_pose` | `geometry_msgs/msg/PoseStamped` | Filtered canonical tray pose: lower-left origin, tray edge X/Y, natural right-handed Z. |
| `tray_axis_overlay` | `geometry_msgs/msg/PolygonStamped` | The same 2D origin and X/Y unit directions drawn on the camera overlay. |
| `tray_vector` | `dobot_msgs_v4/msg/TrayVector` | Pose, timing, velocity, speed, and direction. |
| `tray_cube_marker` | `visualization_msgs/msg/Marker` | RViz cube marker. |
| `tray_detect/get_tray_dimensions` | `dobot_msgs_v4/srv/GetTrayDimensions` | Taught tray dimensions from the active profile. |

## Profile Files

Profiles are stored in:

```text
WORKSPACE_ROOT/teach/tray_teach
```

Current dated profiles use:

```text
tray_<name>_<ddmmyyyy>.yaml
```

Runtime state and the latest-profile alias are stored under:

```text
WORKSPACE_ROOT/config/tray_perception
```

The latest-profile alias is:

```text
tray_teach_settings.yaml
```

Seek artifacts are written under:

```text
WORKSPACE_ROOT/debug files/seek_frames
```

## Key Runtime Defaults

| Parameter | Default |
| --- | --- |
| `seek_window_sec` | `60.0` |
| `seek_decay_sec` | `1.0` |
| `seek_valid_frames_confidence` | `5` |
| `tray_dimension_tolerance_percent` | `15` |
| `depth_threshold_mm` | `10` |
| `depth_edge_offset_px` | `4` |
| `camera_control_service_root` | `/robot_camera` |
| `color_exposure_min_us` | `1` |
| `color_exposure_max_us` | `100` |
| `pose_filter_window_sec` | `0.8` |
| `pose_filter_min_samples` | `3` |
| `tray_thickness_mm` | `15.0` |

## Quick Checks

```bash
ros2 topic hz /tray_overlay
ros2 topic hz /tray_pose
ros2 topic echo /tray_vector --once
ros2 service call /tray_detect/get_tray_dimensions dobot_msgs_v4/srv/GetTrayDimensions {}
```

## Notes

- Restart `tray_detect` after manually editing or deleting profile YAML files.
- Use `Get Tray Size` in `tray_intercept` to pull dimensions from this package.
- Re-teach a tray profile when the tray, camera geometry, or detection mode
  changes.
