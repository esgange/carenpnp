# camera_calibration

`camera_calibration` provides the hand-eye calibration tools used to align the
camera frame with the DOBOT robot. It supports both eye-on-hand and eye-to-hand
`AX=XB` workflows and writes calibration YAML files consumed by the perception
packages.

## Executables

| Executable | Purpose |
| --- | --- |
| `eye_on_hand_calibrator` | C++ calibration solver and service node. |
| `calibration_perception` | Fits one depth-derived board pose from the four configured ArUco markers. |
| `camera_calibration_gui` | GUI for manual sample capture, solving, and saving YAML. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_calibration
source install/setup.bash
```

## Launch

Recommended full workflow:

```bash
ros2 launch camera_calibration camera_calibration.launch.py
```

Start the Orbbec camera launcher before calibration when the cameras are not
already running:

```bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

The camera launcher scans saved serial-number mappings on startup. If at least
one configured camera is detected, it starts the camera node immediately; missing
configured cameras are reported in the launcher scan log instead of blocking on a
warning dialog.

GUI-only launch:

```bash
ros2 launch camera_calibration camera_calibration_gui.launch.py
```

Mode override:

```bash
ros2 launch camera_calibration camera_calibration.launch.py calibration_mode:=eye_to_hand
```

Camera calibration chooses topics from `camera_prefix`. With
`camera_prefix:=auto`, `eye_on_hand` uses `robot_camera` and `eye_to_hand` uses
`bin_camera`:

```bash
ros2 launch camera_calibration camera_calibration.launch.py \
  calibration_mode:=eye_to_hand \
  camera_prefix:=bin_camera
```

Custom four-marker board IDs:

```bash
ros2 launch camera_calibration camera_calibration.launch.py aruco_ids:=1,2,3,4
```

`aruco_ids` must contain exactly four unique IDs from OpenCV `DICT_5X5_50`
(`0..49`) in top-left, top-right, bottom-left, bottom-right order. The detector
uses depth-derived marker centers/corners to fit one shared board pose as
`tag_frame`.

Explicit camera topic overrides:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  device_preset:='High Accuracy' \
  enable_color:=true \
  enable_depth:=true \
  depth_registration:=true \
  align_target_stream:=COLOR \
  align_mode:=SW \
  enable_frame_sync:=true \
  enable_temporal_filter:=true \
  color_width:=848 color_height:=480 color_fps:=30 \
  depth_width:=848 depth_height:=480 depth_fps:=30 \
  enable_point_cloud:=false
```

Teach the robot platform reference after eye-to-hand camera calibration with
the separate `platform_calibration` package:

```bash
ros2 launch platform_calibration platform_calibration.launch.py
```

## Calibration Modes

| Mode | Use Case |
| --- | --- |
| `eye_on_hand` | Camera is mounted on the robot; calibration target is fixed in the workspace. |
| `eye_to_hand` | Camera is fixed; calibration target is mounted on the end effector. |

`eye_on_hand` is the default.

## Full Launch Composition

`camera_calibration.launch.py` starts:

- `camera_calibration_gui`;
- `calibration_perception`, fitting one depth board pose from the four configured
  `aruco_ids` into `tag_frame`;
- `aruco_perception.launch.py` with `use_calibration=false`, so samples are
  collected in the raw camera frame.

Default frames for the normal flow:

| Setting | Default |
| --- | --- |
| Base frame | `base_link` |
| Gripper frame | `Link6` |
| Eye-on-hand camera frame | `robot_camera_color_optical_frame` |
| Eye-to-hand camera frame | `bin_camera_color_optical_frame` |
| ArUco IDs | `1,2,3,4` |
| Target frame | `tag_frame` fixed |

## Operator Flow

1. Start the RGB-D camera and DOBOT bringup.
2. Launch `camera_calibration.launch.py`.
3. In the GUI, select the calibration mode and confirm frame names, ArUco IDs,
   and minimum sample count.
4. Hand-guide the robot to a calibration pose where all 4 configured markers are
   visible and the tag gate shows `READY`.
5. Click `Get Sample`.
6. Repeat at varied robot poses until the minimum sample count is reached. The
   GUI previews after 3 samples and computes automatically once enough samples
   are collected.
7. Use `Undo Last` to remove the most recent sample, or `Reset Samples` to clear
   all samples after confirming the warning dialog.
8. Save the YAML.

Both modes acquire the tag the same way: all 4 configured markers must produce a
fresh, stable `camera_frame -> tag_frame` transform before `Get Sample` is
enabled. The GUI starts the calibrator service node as soon as it opens and
restarts it when mode/frame/sample-count settings change. Eye-on-hand no longer
generates poses or moves the robot; use drag mode or hand-guiding to choose each
sample pose. Both modes record samples through `add_sample` and request live
preview through the same `preview_calibration` service.

## Services

The solver exposes relative services. When the GUI starts the calibrator, it
puts them under the private namespace shown in the status log, such as
`/camera_calibration_gui_<pid>/add_sample`. If the calibrator executable is run
without a namespace, the same services appear at the root names below.

| Service | Type |
| --- | --- |
| `/add_sample` | `std_srvs/srv/Trigger` |
| `/preview_calibration` | `std_srvs/srv/Trigger` |
| `/compute_calibration` | `std_srvs/srv/Trigger` |
| `/save_calibration` | `std_srvs/srv/Trigger` |
| `/remove_last_sample` | `std_srvs/srv/Trigger` |
| `/reset_samples` | `std_srvs/srv/Trigger` |

Example:

```bash
ros2 service call /compute_calibration std_srvs/srv/Trigger {}
```

## Output Files

Default output path:

```text
WORKSPACE_ROOT/calibration/axab_calibration_<mode>_<ddmmyyyy>_<robot_ip>.yaml
```

Mode tokens are `eyeonhand` and `eyetohand`, for example
`axab_calibration_eyeonhand_09052026_192.168.20.202.yaml`. The robot IP is
resolved from `robot_ip_address`, `ROBOT_IP_ADDRESS`, then root `station_config`;
if none is available, the IP suffix is omitted.

Saved AX=XB camera calibration YAML is intentionally minimal:

```yaml
parameters:
  name: cr10_orbbec335
  calibration_type: eye_in_hand
  robot_base_frame: base_link
  robot_effector_frame: Link6
  tracking_base_frame: camera_color_optical_frame
  tracking_marker_frame: charuco_target
  freehand_robot_movement: true
  move_group_namespace: /
  move_group: manipulator
transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

Mode comes from the filename. Camera calibration readers expect this `transform`
schema.

The output is compatible with `aruco_perception`, `tray_perception`,
`item_perception`, and `obstacle_perception`.

Saving a camera calibration deletes older files only when both the calibration
mode and robot-IP filename suffix match. Legacy no-IP files and files for other
robot IPs are preserved. If the robot IP cannot be resolved, no older files are
deleted.

## Quick Checks

```bash
ros2 topic echo /aruco_overlay --once
ros2 run tf2_ros tf2_echo robot_camera_color_optical_frame tag_frame
ros2 run tf2_ros tf2_echo Link6 arm_calibrated_camera_link
```

## Notes

- Use a stable, visible ArUco target before clicking `Get Sample`.
- Move the robot manually between samples so the solver sees varied positions
  and orientations.
- Re-run calibration when the camera mount, lens, resolution, or robot TCP setup
  changes.
