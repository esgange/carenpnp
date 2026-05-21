# item_perception_yolo

`item_perception_yolo` is the YOLO/SAM2 branch of the DOBOT item perception
stack. It teaches ROI-cropped YOLO11 segmentation samples from SAM2 prompts,
trains a YOLO11-seg model, exports an ONNX model, and runs the YOLO detector.

## Nodes

| Node | Purpose |
| --- | --- |
| `bin_teach` | ArUco-assisted bin-frame teaching utility used to provide the ROI and depth plane for YOLO teach. |
| `item_teach_yolo_node.py` | SAM2 prompt-based teach UI that saves YOLO11 segmentation samples and trains a YOLO11-seg model. |
| `item_detect_yolo_node.py` | Runtime YOLO11 segmentation detector. It loads ONNX profiles, runs ROI-crop inference on CPU, and publishes item pose outputs/services. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select item_perception_yolo
source install/setup.bash
```

## Python Dependencies

YOLO/SAM2 teach and detect nodes should run from the frozen workspace-local
Python environment:

```bash
tools/deps/install_offline_deps.sh --python-only
source tools/deps/source_third_party_env.sh
python -c "import torch, torchvision, ultralytics, onnxruntime, sam2"
```

## Launch

Teach or update a bin ROI first:

```bash
ros2 launch item_perception_yolo bin_teach.launch.py
```

Teach YOLO11 segmentation samples with SAM2 prompts:

```bash
ros2 launch item_perception_yolo item_teach_yolo.launch.py
```

Enter the item name in the teach UI before saving samples or training.

YOLO teach defaults to the bin camera stream under `/bin_camera`. Override the
topics when using a different camera namespace:

```bash
ros2 launch item_perception_yolo item_teach_yolo.launch.py \
  color_topic:=/custom_camera/color/image_raw \
  depth_topic:=/custom_camera/depth/image_raw \
  camera_info_topic:=/custom_camera/color/camera_info
```

Run YOLO detection:

```bash
ros2 launch item_perception_yolo item_detect_yolo.launch.py
```

The YOLO detector keeps the external ROS node name `item_detect`, so existing
topics/services such as `item_detect/seek`, `bin_seek_pose`, and
`bin_item_poses` remain compatible.

## YOLO Teach Workflow

1. Load a saved `bin_teach` profile.
2. Add positive SAM2 prompts with left-click.
3. Add negative SAM2 prompts with right-click when needed.
4. Save item segmentation samples.
5. Save background samples for empty-bin or non-target views.
6. Train YOLO11.
7. Use the generated YOLO profile in `item_detect_yolo`.

The teach UI crops samples to the selected bin ROI and blacks out pixels outside
that ROI. It also saves the current six robot joint angles when
`/joint_states_robot` is available, so the detector can offer `Go To Teach`.

Background samples are saved with empty YOLO label files. Keep them around
10-20% of the total training images when the background is mostly stable.

## Generated Files

YOLO teach creates scratch runtime sessions under:

```text
WORKSPACE_ROOT/config/item_perception_yolo/item_teach_yolo_runtime
```

The runtime folder is cleared on each fresh teach launch by default. After
training, the final detector-ready bundle is promoted into one folder under:

```text
WORKSPACE_ROOT/teach/item_teach_yolo
```

Bundle folders follow the item/bin/date naming standard:

```text
item_<item>[_bin_<bin>]_<ddmmyyyy>/
```

Each bundle contains `best.pt`, `best.onnx`, and
`item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml`. The profile stores camera topics,
ROI points, bin association, depth plane data from `bin_teach`, model paths,
training metadata, item/background sample counts, and teach joints.

## Detection

The detector loads YOLO profiles from `teach/item_teach_yolo`, runs ONNX
Runtime CPU inference on the selected bin ROI, selects the highest-confidence
mask while Seek is armed, and publishes:

| Topic | Type | Notes |
| --- | --- | --- |
| `bin_overlay` | `sensor_msgs/msg/Image` | Debug/preview image. |
| `bin_item_poses` | `geometry_msgs/msg/PoseArray` | All valid per-mask item poses. |
| `bin_seek_pose` | `geometry_msgs/msg/PoseStamped` | Selected seek pose. |
| `bin_cube_marker` | `visualization_msgs/msg/Marker` | RViz marker for visualization. |

Detection defaults to the robot camera stream under `/robot_camera`. Override
`color_topic`, `depth_topic`, and `camera_info_topic` if needed.

## Maintenance Notes

- Rebuild after changing package source or launch files:

```bash
colcon build --packages-select item_perception_yolo --symlink-install
source install/setup.bash
```

- Restart `item_detect_yolo` after manually editing or deleting profile YAML
  files.
