# tray_perception

`tray_perception` provides the RGB-D teach and detect workflow for trays. A tray
profile is taught with `tray_teach_node`, then `tray_detect_node` loads the
profile to publish tray pose, tray velocity, and tray dimensions for downstream
motion packages.

## Nodes

| Node | Executable | Purpose |
| --- | --- | --- |
| `tray_teach` | `tray_teach_node` | Interactive OpenCV UI for teaching tray ROI, edge settings, and optional depth-plane reference. |
| `tray_detect` | `tray_detect_node` | Runtime detector for tray pose, seek confidence, tray vector, and dimensions service. |

## Build

```bash
cd /home/erds/DOBOT_pickn_place
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

Camera topic overrides:

```bash
ros2 launch tray_perception tray_detect.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

## Teach Workflow

1. Enter a tray name.
2. Select RGB or depth view.
3. Add the tray ROI.
4. Tune thresholds and edge/ray settings.
5. In depth mode, select the depth-plane ROI when prompted.
6. Verify the overlay and save the tray profile.

Detection mode is determined by the view used when placing the ROI:

- ROI added in `RGB` view saves an RGB detection profile.
- ROI added in `Depth` view saves a depth detection profile.

Depth profiles save fixed depth-plane coefficients. Runtime detection loads the
saved plane instead of recomputing it every frame.

## Detect Behavior

`tray_detect_node`:

- loads valid tray profiles from the profiles directory;
- supports RGB or depth detection based on the selected profile;
- publishes only after `seek_valid_frames_confidence` continuous valid frames;
- resets seek evidence after `seek_decay_sec` without a valid frame;
- publishes first/last seek artifacts for successful confidence runs;
- uses the lower-left tray corner as the pose and overlay origin;
- sets `+X` along the tray long side and `+Y` along the short side.
- keeps that lower-left origin fixed; seek direction no longer flips the axes.

## Calibration

`tray_detect.launch.py` loads calibration YAML directly and publishes the static
camera transform when calibration is enabled.

Defaults:

| Setting | Default |
| --- | --- |
| `use_calibration` | `true` |
| `parent_frame` | `Link6` |
| `child_frame` | `calibrated_camera_link` |
| `calibration_dir` | `~/DOBOT_pickn_place/calibration` |

If calibration is enabled and no usable YAML is available, launch shows an error
dialog and exits.

## Inputs

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

## Outputs

| Output | Type | Notes |
| --- | --- | --- |
| `tray_overlay` | `sensor_msgs/msg/Image` | Runtime debug/preview image. |
| `tray_pose` | `geometry_msgs/msg/PoseStamped` | Filtered canonical tray pose: overlay origin/X, robot/base-up Z. |
| `tray_axis_overlay` | `geometry_msgs/msg/PolygonStamped` | The same 2D origin and X/Y unit directions drawn on the camera overlay. |
| `tray_vector` | `dobot_msgs_v4/msg/TrayVector` | Pose, timing, velocity, speed, and direction. |
| `tray_cube_marker` | `visualization_msgs/msg/Marker` | RViz cube marker. |
| `tray_detect/get_tray_dimensions` | `dobot_msgs_v4/srv/GetTrayDimensions` | Live dimensions or taught-profile fallback. |

## Profile Files

Profiles are stored in:

```text
/home/erds/DOBOT_pickn_place/config/trays
```

Current dated profiles use:

```text
tray_<name>_<ddmmyyyy>.yaml
```

The compatibility/latest file is:

```text
tray_teach_settings.yaml
```

Seek artifacts are written under:

```text
/home/erds/DOBOT_pickn_place/debug files/seek_frames
```

## Key Runtime Defaults

| Parameter | Default |
| --- | --- |
| `seek_window_sec` | `60.0` |
| `seek_decay_sec` | `1.0` |
| `seek_valid_frames_confidence` | `5` |
| `area_tolerance_percent` | `15` |
| `depth_threshold_mm` | `10` |
| `depth_edge_offset_px` | `4` |
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
