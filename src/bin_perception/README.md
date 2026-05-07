# bin_perception

`bin_perception` provides the RGB-D teach and detect workflow used by the
DOBOT bin picking stack. An item profile is taught once with `item_teach`, saved as
a dated YAML file, and then loaded by `item_detect` for runtime masking, item pose
estimation, and optional robot seek/go-to-teach behavior.

## Nodes

| Node | Purpose |
| --- | --- |
| `item_teach` | Interactive OpenCV UI for teaching ROI, RGB thresholds, depth plane, depth mask tuning, and item pose references. |
| `item_detect` | Runtime detector that loads taught profiles, tracks the selected bin, publishes poses and overlays, and manages profile selection/deletion. |
| `bin_teach` | ArUco-assisted bin-frame teaching utility. |

## Build

```bash
cd /home/erds/DOBOT_pickn_place
source /opt/ros/humble/setup.bash
colcon build --packages-select bin_perception
source install/setup.bash
```

## Launch

Teach a profile:

```bash
ros2 launch bin_perception item_teach.launch.py
```

`item_teach` opens the interactive teach UI.

Run detection:

```bash
ros2 launch bin_perception item_detect.launch.py
```

Teach a bin frame from ArUco markers:

```bash
ros2 launch bin_perception bin_teach.launch.py
```

By default, `bin_teach` now auto-loads the current platform calibration from:

```text
~/DOBOT_pickn_place/config/platform/platform_calibration_<platform_name>.yaml
```

Create or update that file first with:

```bash
ros2 launch camera_calibration platform_teach.launch.py platform_name:=robot_platform_1
```

Set `use_platform_calibration:=false` only for the legacy camera-relative
bin-teach workflow.
Set `platform_calibration_file:=/abs/path/to/file.yaml` only when you want to
override auto-discovery.

Common launch overrides:

```bash
ros2 launch bin_perception item_teach.launch.py calibration_file:=/abs/path/to/calibration.yaml
ros2 launch bin_perception item_detect.launch.py align_item_z_axis_to_depth_plane:=true
```

## Calibration

Both nodes default to the current eye-on-hand calibration workflow.

- Calibration files are discovered from `~/DOBOT_pickn_place/calibration` unless
  `calibration_file` is provided.
- `item_teach` and `item_detect` publish the static calibration transform in-node
  when calibration is enabled.
- Item poses are parented to `calibrated_camera_link`.
- If no usable calibration YAML is available and calibration is enabled, launch
  fails early with a clear error.

## Teach Workflow

1. Load a saved `bin_teach` profile, or select the RGB region of interest manually.
2. Tune RGB thresholds and choose `Focus White` or `Focus Black`.
3. Use the bin-teach reference depth plane, or manually select four depth-plane corners when no bin plane is available.
4. Tune depth null fill, depth window, depth hole fill, and depth trim.
5. Enter the pose stage.
6. Teach the item pose reference:
   - single-blob items use one saved blob reference;
   - pair items teach blob `1/2` and blob `2/2` as separate real references.
7. Verify the overlay and save the item profile.

When a `bin_teach` profile is loaded, the same four ArUco-corner points become
the RGB ROI and the saved bin reference depth plane becomes the depth-normalize
plane for `item_teach`. The bin plane is learned once in `bin_teach` from the
ArUco marker center depths in normalized image coordinates, so item profiles for
the same bin share a consistent depth baseline. The bin name and item name stay independent. The saved item
profile records the associated bin, and the item profile filename includes the
bin suffix so the same item can be taught for multiple bins without overwriting
another profile from the same day. If the bin was taught at a different camera
resolution, `item_teach` scales the saved normalized bin ROI to the current
color/depth frame instead of rejecting it.

`item_teach` can delete the selected bin-teach YAML from the bin selector row
after a second confirmation click, then refreshes the selector immediately. The
Back button steps through the teach stages so earlier knobs can be adjusted; when
the depth plane came from `bin_teach`, stepping back from depth tuning preserves
that shared plane.

The teach-mode edge tolerance slider is for post-teach preview only. It is not
saved into the item profile; `item_detect` keeps its own runtime tolerance.

The teach preview mirrors the runtime mask order used by `item_detect`:

```text
ROI -> RGB mask -> optional RGB cleanup -> depth-plane mask -> depth window/trim -> pose detection
```

For depth peak selection, detect uses only pixels inside:

```text
RGB mask AND ROI AND above normalized depth plane
```

## Profile Files

Item profiles are saved in:

```text
/home/erds/DOBOT_pickn_place/config/bins
```

Each saved profile uses this name pattern:

```text
item_<name>_<ddmmyyyy>.yaml
item_<name>_bin_<associated_bin>_<ddmmyyyy>.yaml
```

The detector only lists real profile YAML files from the profiles directory.
Legacy aggregate files such as `item_teach_settings.yaml` and
`bin_teach_settings.yaml` are ignored and should not appear in the dropdown.
Existing legacy `bin_<name>_<ddmmyyyy>.yaml` profiles remain loadable.

Bin-teach profiles are saved in:

```text
/home/erds/DOBOT_pickn_place/config/bin_teach
```

New bin-teach files use the name you enter:

```text
<name>.yaml
```

With platform calibration enabled, each bin-teach YAML saves the bin transform in
the loaded platform frame, for example `robot_platform_1 -> bin_blue_bin_frame`,
and records the platform calibration file under `platform_reference`. The saved
`roi_points` are still the four image-space corner dots used by `item_teach`.
Each file also saves the bin reference depth plane as `depth_plane_*` fields
using the same `a*x_norm + b*y_norm + c` model consumed by item profiles.

Older `bin_<name>_teach.yaml` bin-teach files remain loadable by `item_teach`.

Runtime state files:

| File | Owner | Purpose |
| --- | --- | --- |
| `item_teach_runtime.yaml` | `item_teach` | Stores teach UI/runtime preferences. |
| `item_detect_runtime_settings.yaml` | `item_detect` | Stores detect UI/runtime preferences, including the selected profile path. |
| `item_detect_selected_profile.txt` | `item_detect` | Exports the current selected profile path for other nodes. |

Deleting a profile from `item_detect` removes the selected dated YAML file and
refreshes the dropdown from disk.

## Saved Profile Contract

`item_teach` saves the runtime parameters consumed by `item_detect`, including:

- ROI points and RGB threshold settings.
- Focus mode and RGB cleanup parameters.
- Depth mask parameters.
- Normalized depth plane fields:
  - `depth_plane_enabled`
  - `depth_plane_a`
  - `depth_plane_b`
  - `depth_plane_c`
  - `depth_plane_reference_depth_m`
  - `depth_plane_roi`
- Z-axis alignment policy:
  - `align_item_z_axis_to_depth_plane`
- Pose template data for single or pair references.

Teach preview tolerance is intentionally excluded from saved profiles. Runtime
detect tolerance is controlled by `item_detect` and its runtime settings.

Pair references are saved with explicit pair fields so `item_detect` can match
blob `2/2` independently, not only predict it from blob `1/2`:

- `pose_template_mode: pair`
- `pose_group_member_count: 2`
- `anchor_hull`
- `companion_hull`
- `companion_area_px`
- `companion_aspect_ratio`
- `companion_fill_ratio`
- `group_hull`
- `group_area_px`
- `group_aspect_ratio`
- `member_centers_norm`
- `anchor_center_norm`

Legacy pose keys may still be loaded for compatibility, but newly saved profiles
use the explicit fields above.

## Pose Orientation

Item pose axes follow this policy:

- `X` follows the detected long-side direction.
- `Y` follows the detected short-side direction.
- `Z` aligns to the taught normalized depth-plane normal when
  `align_item_z_axis_to_depth_plane=true` and the saved depth plane is valid.
- If the depth plane is missing or invalid, the original detected pose
  orientation is preserved. There is no robot-base-normal fallback.

`bin_item_poses` uses the same aligned orientation computed internally by the
detector.

## Outputs

Default runtime outputs:

| Topic | Type | Notes |
| --- | --- | --- |
| `bin_overlay` | `sensor_msgs/msg/Image` | Debug/preview image. |
| `bin_item_poses` | `geometry_msgs/msg/PoseArray` | All matched item poses. |
| `bin_pose` | `geometry_msgs/msg/PoseStamped` | Selected/primary bin pose. |
| `bin_cube_marker` | `visualization_msgs/msg/Marker` | RViz marker for bin visualization. |

## Maintenance Notes

- Rebuild after changing C++ or launch files:

```bash
colcon build --packages-select bin_perception
source install/setup.bash
```

- Restart `item_detect` after manually editing or deleting profile YAML files.
- Prefer creating new dated teach profiles instead of editing old legacy
  profiles by hand.
- Old single-blob profiles remain supported. Old pair profiles without complete
  companion hull or depth-plane data should be retaught so the detector has the
  same assumptions used by current teach mode.
