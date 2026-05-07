# bin_perception Quickstart

This quickstart covers the normal teach-then-detect flow. See
`README.md` for the full profile contract and runtime behavior.

## 1. Build

```bash
cd /home/erds/DOBOT_pickn_place
source /opt/ros/humble/setup.bash
colcon build --packages-select bin_perception
source install/setup.bash
```

## 2. Teach an Item Profile

If this robot setup does not already have a platform reference, create it once:

```bash
ros2 launch camera_calibration platform_teach.launch.py platform_name:=robot_platform_1
```

Then teach or update the bin:

```bash
ros2 launch bin_perception bin_teach.launch.py
```

`bin_teach` auto-loads the one active platform calibration from
`~/DOBOT_pickn_place/config/platform` and saves the bin pose in that platform
frame.

```bash
ros2 launch bin_perception item_teach.launch.py
```

This opens the interactive teach UI.

Teach flow:

1. Load a saved `bin_teach` profile, or select the RGB ROI manually.
2. Tune RGB thresholds and select `Focus White` or `Focus Black`.
3. Use the loaded bin-teach reference depth plane, or manually select four depth-plane corners if no bin plane is available.
4. Tune depth null fill, depth window, depth hole fill, and depth trim.
5. Enter the pose stage.
6. Teach either one single blob or a two-blob pair (`1/2` and `2/2`).
7. Confirm the overlay, including ROI, depth plane, blob hulls, group hull, and pose axes.
8. Save the item profile.

Saved profiles are written to:

```text
/home/erds/DOBOT_pickn_place/config/bins/item_<name>_<ddmmyyyy>.yaml
/home/erds/DOBOT_pickn_place/config/bins/item_<name>_bin_<associated_bin>_<ddmmyyyy>.yaml
```

The item name and bin name are independent. Loading `blue_bin` in `item_teach`
associates the saved item profile with that bin, reuses/scales its ROI, and uses
the same four corner points for the depth-normalize plane.

New bin-teach profiles are saved as:

```text
/home/erds/DOBOT_pickn_place/config/bin_teach/<name>.yaml
```

The bin-teach file stores both the ROI corners and the bin reference depth plane.
`item_teach` inherits that plane so every item taught in the same bin uses the
same depth baseline.

In `item_teach`, use `Delete` beside the Bin Teach dropdown, then confirm with a
second click, to remove the selected bin-teach file. Use `Back` to revisit
earlier teach stages; inherited bin depth planes stay loaded when you step back
to tune RGB settings.

## 3. Run Detection

```bash
ros2 launch bin_perception item_detect.launch.py
```

Default outputs:

- `/bin_overlay`
- `/bin_item_poses`
- `/bin_pose`
- `/bin_cube_marker`

Use the top dropdown to select a profile. Use `Delete Item` to remove the
selected dated profile from disk and refresh the dropdown.
