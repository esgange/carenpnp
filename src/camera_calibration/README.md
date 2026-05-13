# camera_calibration

`camera_calibration` provides the hand-eye calibration tools used to align the
camera frame with the DOBOT robot. It supports both eye-on-hand and eye-to-hand
`AX=XB` workflows and writes calibration YAML files consumed by the perception
packages.

## Executables

| Executable | Purpose |
| --- | --- |
| `eye_on_hand_calibrator` | C++ calibration solver and service node. |
| `calibration_perception` | Averages multiple ArUco marker TFs into one stable target frame. |
| `platform_teach` | GUI for saving the calibration board pose as the robot platform reference. |
| `camera_calibration_gui` | GUI for generating poses, running capture, solving, and saving YAML. |

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

GUI-only launch:

```bash
ros2 launch camera_calibration camera_calibration_gui.launch.py
```

Mode override:

```bash
ros2 launch camera_calibration camera_calibration.launch.py calibration_mode:=eye_to_hand
```

Camera topic overrides:

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

Teach the robot platform reference after camera calibration:

```bash
ros2 launch camera_calibration platform_teach.launch.py
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
- `calibration_perception`, averaging `aruco_marker_1` through
  `aruco_marker_4` into `tag_frame`;
- `aruco_perception.launch.py` with `use_calibration=false`, so samples are
  collected in the raw camera frame.

Default frames for the normal flow:

| Setting | Default |
| --- | --- |
| Base frame | `base_link` |
| Gripper frame | `Link6` |
| Camera frame | `camera_link` |
| Target frame | `tag_frame` |

## Operator Flow

1. Start the RGB-D camera and DOBOT bringup.
2. Launch `camera_calibration.launch.py`.
3. In the GUI, confirm frame names and the output path.
4. Click `Generate Pose` to create candidate robot poses.
5. Optionally click `Align` to move the camera to the closest reachable,
   centered view of the tag.
6. Click `Start` to move, settle, and collect samples.
7. Click `Compute` and review the result.
8. Save the YAML.

The generated motion goals aim the configured camera frame at the tag and are
constrained by tag distance, tag tilt, look-up bias, minimum base-frame Z
height, and IK availability.

## Services

The solver exposes:

| Service | Type |
| --- | --- |
| `/add_sample` | `std_srvs/srv/Trigger` |
| `/compute_calibration` | `std_srvs/srv/Trigger` |
| `/save_calibration` | `std_srvs/srv/Trigger` |
| `/reset_samples` | `std_srvs/srv/Trigger` |

Example:

```bash
ros2 service call /compute_calibration std_srvs/srv/Trigger {}
```

## Output Files

Default output path:

```text
WORKSPACE_ROOT/calibration/axab_calibration_<mode>_<ddmmyyyy>.yaml
```

Mode tokens are `eyeonhand` and `eyetohand`, for example
`axab_calibration_eyeonhand_09052026.yaml`.

Saved AX=XB camera calibration YAML is intentionally minimal:

```yaml
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

Only one active camera calibration YAML per mode is kept in the calibration
directory. Saving a new eye-on-hand calibration deletes older eye-on-hand files;
saving a new eye-to-hand calibration deletes older eye-to-hand files.

## Platform Reference

`platform_teach.launch.py` reuses the same four-marker calibration board, but
loads the current camera calibration first. `platform_teach` now averages marker
TFs `1, 2, 3, 4` internally into `platform_board_observed`, then saves the board
pose as:

```text
WORKSPACE_ROOT/calibration/platform_calibration_<platform_name>.yaml
```

Only one platform calibration is kept in the calibration directory. Saving again deletes
older `platform_calibration_*.yaml` files and writes the new one. Save becomes
available after the internal board pose stays within 1 mm and 1 degree for one
second. The YAML stores `base_link -> <platform_name>` as
top-level `transform`, with metadata such as `platform_name`,
`transform_parent_frame`, `transform_child_frame`, `observed_board_frame`,
marker IDs, and timestamp.

`bin_teach` auto-loads this platform file by default and saves bin transforms in
the platform frame, while ROI dots remain normal RGB image pixel points.

## Quick Checks

```bash
ros2 topic echo /aruco_overlay --once
ros2 run tf2_ros tf2_echo camera_link tag_frame
ros2 run tf2_ros tf2_echo Link6 calibrated_camera_link
```

## Notes

- Use a stable, visible ArUco target before starting automatic capture.
- Keep robot motion clear of obstacles during generated-pose capture.
- Re-run calibration when the camera mount, lens, resolution, or robot TCP setup
  changes.
