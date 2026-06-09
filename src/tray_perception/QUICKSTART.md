# tray_perception Quickstart

This is the short operator path from RGB-D camera images to a published tray
pose and tray vector. See `README.md` for the full behavior and interface
contract.

## 1. Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select tray_perception
source install/setup.bash
```

Required camera inputs:

- `/robot_camera/color/image_raw`
- `/robot_camera/depth/image_raw`
- `/robot_camera/color/camera_info`

## 2. Teach a Tray Profile

```bash
ros2 launch tray_perception tray_teach.launch.py
```

Use custom camera topics when needed:

```bash
ros2 launch tray_perception tray_teach.launch.py \
  color_topic:=/my_camera/color/image_raw \
  depth_topic:=/my_camera/depth/image_raw \
  camera_info_topic:=/my_camera/color/camera_info
```

Teach flow:

1. Enter a tray name.
2. Choose RGB or depth view.
3. Add the tray ROI.
4. Tune thresholds, RGB exposure, and edge/ray controls until the tray outline is stable.
5. If using depth mode, select the depth-plane ROI when prompted.
6. Verify the overlay.
7. Save the tray profile.

The `RGB Exposure us` slider uses `0` for auto exposure and `1-100` as direct
microseconds. Depth exposure stays auto. Detection applies the saved RGB exposure
from the selected tray profile.

Saved profiles are written to:

```text
WORKSPACE_ROOT/teach/tray_teach/tray_<name>_<ddmmyyyy>.yaml
```

The compatibility/latest profile is also written as:

```text
WORKSPACE_ROOT/config/tray_perception/tray_teach_settings.yaml
```

## 3. Run Detection

```bash
ros2 launch tray_perception tray_detect.launch.py
```

Use the same camera overrides if the camera topics are not the defaults:

```bash
ros2 launch tray_perception tray_detect.launch.py \
  color_topic:=/my_camera/color/image_raw \
  depth_topic:=/my_camera/depth/image_raw \
  camera_info_topic:=/my_camera/color/camera_info
```

Detect mode:

- loads valid tray profiles from `WORKSPACE_ROOT/teach/tray_teach`;
- supports RGB or depth detection based on the selected profile;
- publishes `tray_pose` after the seek confidence threshold is met;
- publishes `tray_vector` with timing and velocity metadata;
- exposes `tray_detect/get_tray_dimensions` for `tray_intercept`.

## 4. Check Outputs

```bash
ros2 topic hz /tray_overlay
ros2 topic echo /tray_pose --once
ros2 topic echo /tray_vector --once
ros2 service call /tray_detect/get_tray_dimensions dobot_msgs_v4/srv/GetTrayDimensions {}
```

## Common Issues

- No profile selected: use `Open Teach` to choose a tray teach YAML, or teach
  and save a profile first.
- Missing calibration: pass `calibration_file:=/abs/path/to/file.yaml` or add a
  valid YAML to `WORKSPACE_ROOT/calibration`.
- Overlay appears but no `tray_pose`: loosen area/depth tolerance or retune the
  tray ROI and thresholds.
- Unstable output: re-teach the profile with a cleaner ROI and stronger edge or
  depth-plane separation.
