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
ros2 launch platform_calibration platform_calibration.launch.py
```

Then teach or update the bin:

```bash
ros2 launch item_perception bin_teach.launch.py
```

## 3. Teach A YOLO Item

```bash
ros2 launch item_perception_yolo item_teach_yolo.launch.py
```

Teach flow:

1. Load the saved bin teach profile.
2. Enter the item name in the teach UI.
3. Left-click positive SAM2 prompts on the item.
4. Right-click negative prompts if the mask includes unwanted pixels.
5. Save item samples.
6. Save background samples for empty-bin or non-target views.
7. Train YOLO11.

For stable backgrounds, keep background images around 10-20% of the total
training images. `Save BG` writes an empty YOLO label file for that ROI crop.
Use `Save Session` to keep both the generated dataset and ROI review progress.
Reloaded sessions resume saved ROI captures with annotated/skipped frame status
intact, and annotated frames show their saved mask overlay.

The final YOLO teach bundle is written to:

```text
WORKSPACE_ROOT/teach/item_teach_yolo/item_<item>[_bin_<bin>]_<ddmmyyyy>/
```

That folder contains the detector profile and final model files:

```text
item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml
best.onnx
```

## 4. Run YOLO Detection

```bash
ros2 launch item_perception_yolo item_detect_yolo.launch.py
```

YOLO detect defaults to the same `/bin_camera` topics used by YOLO teach. Use
`Open Model` to select a YOLO ONNX/model bundle; when the bundle has its sibling
YAML, detect also loads the saved ROI, camera topics, bin plane, and teach
joints.

Default outputs:

- `/bin_overlay`
- `/bin_item_poses`
- `/bin_seek_pose`
- `/bin_cube_marker`

Use `Delete Item` to remove the selected YOLO profile and associated model
bundle from disk.
