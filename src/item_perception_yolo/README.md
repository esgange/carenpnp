# item_perception_yolo

`item_perception_yolo` is the YOLO/SAM2 branch of the DOBOT item perception
stack. It teaches ROI-masked YOLO11 segmentation samples from SAM2 prompts,
trains a YOLO11-seg model, exports an ONNX model, and runs the YOLO detector.

## Nodes

| Node | Purpose |
| --- | --- |
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

Teach or update a bin ROI first with the classic item perception package:

```bash
ros2 launch item_perception bin_teach.launch.py
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

When color/depth/camera-info topics are not broadcasting, teach and detect show
the same explicit `no camera topics...` overlay style used by the other camera
UIs.

Run YOLO detection:

```bash
ros2 launch item_perception_yolo item_detect_yolo.launch.py
```

YOLO detection uses the fixed `/bin_camera`, so calibration auto-discovery
selects the newest eye-to-hand file tagged for the current robot IP. If no
exact-IP calibration file exists, launch fails and prompts for an explicit file.
Override the selection with `calibration_file:=/abs/path/to/calibration.yaml`.
For the shared/default robot IP `192.168.200.1`, automatic selection is disabled
and launch asks the operator to choose an eye-to-hand file. Headless launches
must pass `calibration_file` explicitly.

The YOLO detector keeps the external ROS node name `item_detect`, so existing
topics/services such as `item_detect/seek`, `item_detect/repick`,
`item_detect/seek_complete`, `item_detect/seek_status`, `bin_seek_pose`, and
`bin_item_poses` remain compatible with `item_pick`.

With `use_calibration:=true`, YOLO detect must publish item poses in the
calibrated bin-camera child frame, normally `bin_calibrated_camera_link`, and
publish the static `base_link -> bin_calibrated_camera_link` transform from the
selected eye-to-hand calibration YAML. Do not publish pick handoff poses in the
raw camera frame such as `bin_camera_color_optical_frame`; `item_pick` expects a
TF path from `base_link` to the pose frame.

For Item Pick handoff, YOLO detect also publishes the loaded profile YAML on
the latched `item_detect/selected_profile` topic and writes the same path to
`config/item_perception/item_detect_selected_profile.txt`. The ONNX model path
is still written separately to the YOLO-specific selected-model file.

Seek behavior matches non-YOLO item detect: Seek toggles ON/OFF with
`item_detect/seek`, the first valid pose is published once to `bin_seek_pose`,
Seek stays logically ON in a latched handoff state, `item_detect/repick`
reacquires without toggling Seek OFF, and only `item_detect/seek_complete`
releases Seek after Item Pick final Z-up.

By default, YOLO detect uses the same `/bin_camera` topic family as YOLO teach.
When a trained profile is selected, detect also follows the camera topics saved
inside that profile, so a custom-topic teach profile runs on the same streams
without extra launch arguments.

The detect window mirrors the classic non-YOLO `item_detect` operator UI:
`View`, `Overlay`, `Seek`, `Debug Img`, `Go To Teach`, `Open Model`, and
`Delete Item` live in the same top-bar layout. `Open Model` selects a YOLO
ONNX/model bundle; if a sibling YAML profile exists, detect uses it for ROI,
bin depth plane, camera topics, item name, and teach joints. Without that YAML,
detect can still run the model on the full frame, but plane alignment and
`Go To Teach` metadata are unavailable.

## YOLO Teach Workflow

1. Load a saved `bin_teach` profile.
2. Add positive SAM2 prompts with left-click.
3. Add negative SAM2 prompts with right-click when needed.
4. Save item segmentation samples.
5. Save background samples for empty-bin or non-target views.
6. Train YOLO11.
7. Use the generated YOLO profile in `item_detect_yolo`.

Important ROI terminology: in this package, "cropped ROI" means a full camera
resolution image with pixels outside the selected bin ROI erased to black. It
does not mean saving or reviewing a physically smaller image. `Capture ROI`
saves only the ROI-masked full-frame image. `Review Images` should show that
ROI-masked image at the same resolution as the raw camera frame. The SAM2/YOLO
annotation logic may use the ROI rectangle internally, but the review image stays
full-frame with black outside the ROI.

The teach UI saves samples from the selected bin ROI and blacks out pixels
outside that ROI. It also saves the current six robot joint angles when
`/joint_states_robot` is available, so the detector can offer `Go To Teach`.

Background samples are saved with empty YOLO label files. Keep them around
10-20% of the total training images when the background is mostly stable.

Use `Save Session` before leaving a weak or unfinished training run if you want
to continue it later. `Load Session` lists saved sessions by item name and
sample counts, and its dropdown also includes a delete action for saved sessions.

## Generated Files

YOLO teach creates scratch runtime sessions under:

```text
WORKSPACE_ROOT/config/item_perception_yolo/item_teach_yolo_runtime
```

The runtime folder is cleared on each fresh teach launch by default, and
unsaved runtime sessions are removed when the item name changes or the node
exits. Saved sessions are kept under:

```text
WORKSPACE_ROOT/config/item_perception_yolo/item_teach_yolo_saved_sessions
```

After training, the final detector-ready bundle is promoted into one folder
under:

```text
WORKSPACE_ROOT/teach/item_teach_yolo
```

Bundle folders follow the item/bin/date naming standard:

```text
item_<item>[_bin_<bin>]_<ddmmyyyy>/
```

Each bundle contains `best.onnx` and
`item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml`. The profile stores camera topics,
ROI points, bin association, depth plane data from `bin_teach`, model paths,
training metadata, item/background sample counts, and teach joints.

## Detection

The detector auto-loads YOLO bundles from `teach/item_teach_yolo`, can open a
YOLO ONNX model directly with `Open Model`, runs ONNX Runtime CPU inference on
the selected bin ROI or full frame, selects the highest-confidence mask while
Seek is armed, and publishes:

| Topic | Type | Notes |
| --- | --- | --- |
| `bin_overlay` | `sensor_msgs/msg/Image` | Debug/preview image. |
| `bin_item_poses` | `geometry_msgs/msg/PoseArray` | All valid per-mask item poses. |
| `bin_seek_pose` | `geometry_msgs/msg/PoseStamped` | Selected seek pose. |
| `bin_cube_marker` | `visualization_msgs/msg/Marker` | RViz marker for visualization. |

Detection defaults to the bin camera stream under `/bin_camera`. Override
`color_topic`, `depth_topic`, and `camera_info_topic` if needed, or launch with
`use_profile_camera_topics:=false` when you want launch arguments to override
the topics saved in the trained profile.

## Maintenance Notes

- Rebuild after changing package source or launch files:

```bash
colcon build --packages-select item_perception_yolo --symlink-install
source install/setup.bash
```

- Restart `item_detect_yolo` after manually editing or deleting profile YAML
  files.
