# item_perception_yolo Quickstart

This quickstart covers the YOLO/SAM2 teach-then-detect flow.

## 1. Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select item_perception_yolo
source install/setup.bash
```

## 2. Teach A Bin ROI

If this robot setup does not already have a platform reference, create it once:

```bash
ros2 launch camera_calibration platform_teach.launch.py platform_name:=robot_platform_1
```

Then teach or update the bin:

```bash
ros2 launch item_perception_yolo bin_teach.launch.py
```

## 3. Teach A YOLO Item

```bash
ros2 launch item_perception_yolo item_teach_yolo.launch.py item_name:=paper_cutlery
```

Teach flow:

1. Load the saved bin teach profile.
2. Left-click positive SAM2 prompts on the item.
3. Right-click negative prompts if the mask includes unwanted pixels.
4. Save one or more samples.
5. Train YOLO11.

The YOLO profile is written to:

```text
WORKSPACE_ROOT/teach/bins_yolo/profiles/item_<item>[_bin_<bin>]_yolo_<date>.yaml
```

The trained model bundle is written under:

```text
WORKSPACE_ROOT/teach/bins_yolo/models
```

## 4. Run YOLO Detection

```bash
ros2 launch item_perception_yolo item_detect_yolo.launch.py
```

Default outputs:

- `/bin_overlay`
- `/bin_item_poses`
- `/bin_seek_pose`
- `/bin_cube_marker`

Use the top dropdown to select a YOLO profile. Use `Delete Item` to remove the
selected YOLO profile and associated model bundle from disk.
