# aruco_perception

`aruco_perception` detects ArUco markers from an RGB-D camera stream and
publishes marker poses, marker TF frames, and an optional debug overlay. It is
used directly during hand-eye calibration and as a runtime calibrated-camera TF
source for the perception stack.

## Executables

| Executable | Purpose |
| --- | --- |
| `aruco_detector_node` | Detects markers from RGB, samples depth, publishes marker poses and TFs. |
| `perception_calibration` | Advanced helper for manual calibration-file generation. Runtime workflows normally do not use it. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select aruco_perception
source install/setup.bash
```

## Launch

```bash
ros2 launch aruco_perception aruco_perception.launch.py
```

Common overrides:

```bash
ros2 launch aruco_perception aruco_perception.launch.py \
  calibration_file:=/abs/path/to/axab_calibration.yaml \
  show_overlay_window:=false
```

For raw-camera calibration collection, disable calibration:

```bash
ros2 launch aruco_perception aruco_perception.launch.py \
  use_calibration:=false \
  parent_frame:=camera_link \
  show_overlay_window:=false
```

## Camera Inputs

Default topics:

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

If the camera namespace changes, pass matching launch overrides:

```bash
ros2 launch aruco_perception aruco_perception.launch.py \
  color_topic:=/my_camera/color/image_raw \
  depth_topic:=/my_camera/depth/image_raw \
  camera_info_topic:=/my_camera/color/camera_info
```

## Outputs

| Output | Type | Notes |
| --- | --- | --- |
| `marker_pose` | `geometry_msgs/msg/PoseStamped` | Pose of the most recent detected marker. |
| `/aruco_overlay` | `sensor_msgs/msg/Image` | RGB/depth debug view. |
| `camera_frame -> aruco_marker_<id>` | TF | One TF per detected marker. |
| `parent_frame -> child_frame` | static TF | Published when `use_calibration=true`. |

## Calibration Behavior

- With `use_calibration=true`, launch loads the newest non-empty YAML in
  `WORKSPACE_ROOT/calibration` unless `calibration_file` is set.
- If calibration is enabled and no usable file exists, launch fails early with a
  clear error.
- The loaded YAML is expected to include `calibration_transform.rotation` and
  `calibration_transform.translation`.
- When calibration is enabled, marker poses are published in
  `calibrated_camera_link` by default.
- When calibration is disabled, marker poses are published in `parent_frame`.

## Quick Checks

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /marker_pose --once
ros2 topic hz /aruco_overlay
ros2 run tf2_ros tf2_echo calibrated_camera_link aruco_marker_1
```

## Notes

- Start the camera before launching this package.
- Use calibration-disabled mode only for raw-frame calibration workflows.
- Other packages, including `obstacle_perception`, may include this launch file
  to publish the calibrated camera frame.
