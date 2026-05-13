# DOBOT Pick and Place

ROS 2 workspace for DOBOT robot bringup, RGB-D perception, calibration,
operator tools, and pick/intercept workflows.

## Package Map

| Package | Path | Purpose |
| --- | --- | --- |
| `cr_robot_ros2` | `src/dobot_bringup_v4` | TCP bridge to DOBOT controllers; publishes robot state and exposes command services. |
| `dobot_msgs_v4` | `src/dobot_msgs_v4` | Custom DOBOT messages and service definitions. |
| `dobot_rviz` | `src/dobot_rviz` | URDFs, meshes, robot state publisher, and RViz configuration. |
| `orbbec_camera_launcher` | `src/orbbec_camera_launcher` | Tkinter operator GUI for scanning, naming, and launching two Orbbec cameras. |
| `camera_calibration` | `src/camera_calibration` | Eye-on-hand and eye-to-hand calibration tools. |
| `aruco_perception` | `src/aruco_perception` | RGB-D ArUco marker pose detection and calibrated camera TF publishing. |
| `obstacle_perception` | `src/obstacle_perception` | Live depth obstacles and persistent obstacle memory. |
| `tray_perception` | `src/tray_perception` | Tray teach/detect workflow with tray pose, vector, and dimensions output. |
| `tray_intercept` | `src/tray_intercept` | Operator console for intercepting moving trays from `tray_vector`. |
| `tray_intercept_servo` | `src/tray_intercept_servo` | Servo-enabled tray intercept workflow with live preview and startup motion profile support. |
| `item_perception` | `src/item_perception` | Item teach/detect workflow using bin ROI profiles and item pose output. |
| `item_perception_yolo` | `src/item_perception_yolo` | YOLO/SAM2 item perception experiments using the bin ROI workflow. |
| `item_pick` | `src/item_pick` | Operator GUI and motion sequence for picking from `item_detect` output. |
| `item_pick_servo` | `src/item_pick_servo` | Servo-enabled item pick workflow for `bin_seek_pose` outputs. |
| `pick_cycle` | `src/pick_cycle` | Higher-level pick/place cycle GUI that coordinates item pick and tray intercept services. |
| `debug_servop` | `src/debug_servop` | ServoP debug GUI with live TCP markers for tuning servo motion. |
| `motion_debug` | `src/motion_debug` | Live robot debug GUI and motion script editor/player. |
| `movement_calibration` | `src/movement_calibration` | Speed calibration for linear movement scripts. |
| `gripper_control` | `src/gripper_control` | GUI for DOBOT digital output gripper channels. |

Each package has its own `README.md` with launch commands, interfaces, and
operational notes.

## Recent Workflow Updates

- Runtime/config folders are now grouped by owner under `config/`, including
  `item_perception`, `tray_perception`, `robot_bringup`, `camera_bringup`, and
  `motion_calibrate`.
- Teach outputs now use purpose-named folders: `teach/item_teach` for item
  profiles/tool sidecars and `teach/tray_teach` for tray profiles.
- Platform calibration now lives with the other calibration outputs in
  `calibration/platform_calibration_<platform_name>.yaml`, using a top-level
  `transform` block plus metadata.
- `ros_domain_id` was removed from the default robot config. `ROS_DOMAIN_ID` is
  normally inherited from the shell environment, while optional legacy configs
  can still provide `ros_domain_id`.
- `pick_cycle` shows a live robot status in the mini GUI: Stop, Picking,
  Placing, and the reserved On Pause state.
- `item_teach` now reuses loaded ROI points for the depth-plane overlay and can
  derive the depth-normalize plane directly from ROI points.
- The item depth plane is now a reference surface only. The depth window scans
  finite depth inside the RGB mask and ROI across the selected 1-100 mm window
  instead of clamping at the plane.
- `item_pick` now separates pre-pick settling from pickup-depth settling and
  uses `MovLIO` to trigger suction at the start of the 6% pickup descent.
- `tray_detect` publishes natural tray edge axes, and `tray_intercept` projects
  tray X/Y motion into robot base XY while applying standoff in base +Z.
- The old `Clear Depth` teach button was removed because the ROI/depth workflow
  no longer needs that manual reset step.

## Prerequisites

- ROS 2 Humble-style workspace environment.
- `colcon` and `rosdep`.
- C++ compiler and Python 3.
- RGB-D camera publishing aligned color, depth, and camera-info topics.
- DOBOT controller reachable from the host machine.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -r -y
colcon build --symlink-install
source install/setup.bash
```

## Offline Frozen Dependencies

The repo can be packaged with a frozen Ubuntu 22.04 + ROS 2 Humble dependency
bundle under `third_party/`. On an internet-connected machine, populate the
bundle:

```bash
tools/deps/fetch_offline_deps.sh
```

The preferred way to move the system to another PC is now the Docker image
export/load workflow below. The native offline install path is still available
when you intentionally want ROS and Orbbec installed onto the host system.

Copy the repo, including `third_party/debs`, `third_party/wheels`,
`third_party/sam2`, and `third_party/yolo`, to the target PC. Then install
without network access:

```bash
tools/deps/install_offline_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
tools/deps/verify_offline_env.sh
```

Use this in shells that need the frozen Python AI environment:

```bash
source tools/deps/source_third_party_env.sh
```

The frozen apt package set is listed in
`tools/deps/apt-packages.freeze.txt`; Python wheels are locked by
`requirements.lock.txt`. RViz and Orbbec are frozen as local ROS `.deb`
packages, not source-built under `third_party`.

`requirements.txt` tracks the direct pip dependencies for the workspace-local
AI/perception environment, including YOLO, ONNX Runtime, CPU PyTorch, and the
vendored SAM2 runtime dependencies. `requirements.lock.txt` is the full offline
wheel closure used by `tools/deps/install_offline_deps.sh`. ROS Python modules
such as `rclpy`, `launch`, `cv_bridge`, `python_qt_binding`, and Tkinter come
from the frozen apt/ROS bundle, so they are intentionally not pip requirements.

The large `.deb`, wheel, checkpoint, and vendored source payloads are meant for
release archives or local transfer bundles. A plain git clone may not include
all ignored third-party binaries.

The workspace robot config controls `ROS_LOCALHOST_ONLY` through
`config/robot_bringup/param.json`. `ROS_DOMAIN_ID` is normally left to the
shell environment; an optional legacy `ros_domain_id` field is still honored if
present in a custom config.

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

Build one package:

```bash
colcon build --packages-select item_perception
source install/setup.bash
```

## SAM2 Workspace Install

SAM2 is installed from the vendored checkout in the frozen third-party Python
environment:

```bash
tools/deps/install_offline_deps.sh --python-only
source tools/deps/source_third_party_env.sh
python -c "import torch, torchvision, sam2; print(torch.__version__, torchvision.__version__, sam2.__file__)"
```

The editable checkout lives at `third_party/sam2`, and the starter SAM 2.1 tiny
checkpoint is at:

```text
third_party/sam2/checkpoints/sam2.1_hiera_tiny.pt
```

The default lock uses CPU PyTorch (`torch==2.5.1+cpu`,
`torchvision==0.20.1+cpu`). CUDA should be added later as a separate optional
lock file and wheel bundle.

## YOLO11 Workspace Install

YOLO11 is installed through Ultralytics in the same workspace-local Python
environment:

```bash
source tools/deps/source_third_party_env.sh
python -c "from ultralytics import YOLO; model = YOLO('third_party/yolo/checkpoints/yolo11n-seg.pt'); print(model.task)"
```

The starter YOLO11 segmentation nano checkpoint is at:

```text
third_party/yolo/checkpoints/yolo11n-seg.pt
```

Example CLI use:

```bash
yolo segment predict model=third_party/yolo/checkpoints/yolo11n-seg.pt source=/path/to/image.jpg device=cpu
```

## Camera Topic Contract

RGB-D perception packages default to:

| Stream | Topic |
| --- | --- |
| Color image | `/robot_camera/color/image_raw` |
| Depth image | `/robot_camera/depth/image_raw` |
| Camera info | `/robot_camera/color/camera_info` |

Direct camera consumers:

- `aruco_perception`
- `camera_calibration`
- `obstacle_perception`
- `tray_perception`
- `item_perception`
- `item_perception_yolo`

Indirect consumers:

- `tray_intercept` consumes `tray_vector`.
- `item_pick` consumes `bin_seek_pose`.

If the camera namespace changes, pass matching `color_topic`, `depth_topic`, and
`camera_info_topic` launch overrides to the relevant packages.
The eye-to-hand item detector is the current exception: `item_detect_eyetohand`
defaults to the dual-camera bin stream under `/bin_camera`.

For example:

```bash
ros2 launch tray_perception tray_detect.launch.py \
  color_topic:=/custom_camera/color/image_raw \
  depth_topic:=/custom_camera/depth/image_raw \
  camera_info_topic:=/custom_camera/color/camera_info
```

## Recommended Runtime Order

1. Start the RGB-D camera.
2. Start DOBOT bringup:

```bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

3. Start RViz when visualization is needed:

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

4. Calibrate the camera if needed:

```bash
ros2 launch camera_calibration camera_calibration.launch.py
```

5. Run perception:

```bash
ros2 launch aruco_perception aruco_perception.launch.py
ros2 launch obstacle_perception obstacle_perception.launch.py
ros2 launch tray_perception tray_detect.launch.py
ros2 launch item_perception item_detect.launch.py
```

6. Run operator workflows:

```bash
ros2 launch tray_intercept tray_intercept.launch.py
ros2 launch item_pick item_pick.launch.py
ros2 launch motion_debug motion_debug.launch.py
ros2 launch gripper_control gripper_control.launch.py
```

## Calibration Files

Calibration YAML files are stored in:

```text
WORKSPACE_ROOT/calibration
```

Perception launches that use calibration normally auto-discover the newest
non-empty eye-on-hand file named `axab_calibration_eyeonhand_<ddmmyyyy>.yaml`
in that directory.
Platform calibration is saved in the same directory as
`platform_calibration_<platform_name>.yaml`.
You can override discovery with:

```bash
calibration_file:=/abs/path/to/axab_calibration_eyeonhand_09052026.yaml
```

If calibration is enabled and no usable file exists, the launch fails early
instead of silently running with an invalid transform.

## Generated Runtime Data

Common generated paths:

| Path | Owner | Purpose |
| --- | --- | --- |
| `teach/item_teach` | `item_perception`, `item_perception_yolo`, `item_pick`, `item_pick_servo` | Item teach profiles and tool teach sidecars. |
| `config/item_perception` | `item_perception`, `item_perception_yolo`, `item_pick`, `item_pick_servo`, `tray_intercept_servo` | Item runtime settings and active profile selection. |
| `teach/tray_teach` | `tray_perception` | Dated tray teach profiles. |
| `config/tray_perception` | `tray_perception`, `tray_intercept`, `tray_intercept_servo`, `item_pick_servo` | Tray runtime settings and active/latest tray config. |
| `teach/bin_teach` | `item_perception`, `item_perception_yolo` | Bin teach profiles. |
| `teach/bins_yolo` | `item_perception_yolo` | YOLO teach sessions, profiles, and model bundles. |
| `config/robot_bringup/param.json` | `dobot_bringup_v4` | Robot connection config. |
| `config/camera_bringup/orbbec_cameras.yaml` | `orbbec_camera_launcher` | Orbbec camera serial/name mapping. |
| `debug files/seek_frames` | `item_perception`, `tray_perception` | First/last frame seek artifacts. |
| `debug files/pick_cycle_movement_deltas` | `pick_cycle` | One movement delta debug text file per cycle. |
| `WORKSPACE_ROOT/calibration` | `camera_calibration`, `movement_calibration` | Calibration YAML, JSON, and CSV files. |
| `WORKSPACE_ROOT/config/motion_calibrate` | `motion_debug`, `movement_calibration` | Motion script JSON files. |

## Fresh Start for Teach/Detect State

To clear generated bin/tray profiles and runtime state:

```bash
find WORKSPACE_ROOT/teach/item_teach -maxdepth 1 -type f -delete
find WORKSPACE_ROOT/teach/tray_teach -maxdepth 1 -type f -delete
find WORKSPACE_ROOT/teach/bin_teach -maxdepth 1 -type f -delete
rm -f WORKSPACE_ROOT/config/item_perception/item_detect_runtime_settings.yaml
rm -f WORKSPACE_ROOT/config/tray_perception/tray_detect_runtime_settings.yaml
rm -f WORKSPACE_ROOT/config/item_perception/item_pick_runtime_settings.json
```

Then rebuild the affected packages:

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select item_perception tray_perception item_pick --symlink-install
source install/setup.bash
```

## Orbbec Gemini 330 Series Example

The dual-camera launcher reads serial/name mappings and common Orbbec launch
arguments from `config/camera_bringup/orbbec_cameras.yaml`.

```bash
source /opt/ros/humble/setup.bash
source WORKSPACE_ROOT/install/setup.bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

The workspace does not require the camera point cloud because perception nodes
consume image, depth, and camera-info topics directly.

## Docker

The recommended transfer workflow is a sealed Docker runtime image. The image
contains ROS 2 Humble, the frozen Orbbec/RViz/system `.deb` packages, this
workspace built without `--symlink-install`, and the frozen Python AI
environment. The remote PC only needs Ubuntu 22.04, Docker, and the saved image
tar; host ROS is not used by the container.

Before using GUI tools such as RViz or the camera launcher, allow local Docker
containers to use the host X11 display:

```bash
xhost +local:docker
```

Build and verify the image on this PC:

```bash
cd WORKSPACE_ROOT
docker compose build
docker compose run --rm dobot tools/docker/verify_container.sh
```

Export the image for transfer:

```bash
tools/docker/export_image.sh
```

Copy these files to the remote PC:

```text
dobot_pickn_place_humble.tar
dobot_pickn_place_humble.tar.sha256
docker-compose.yml
```

On the remote PC:

```bash
sha256sum -c dobot_pickn_place_humble.tar.sha256
docker load -i dobot_pickn_place_humble.tar
docker compose run --rm dobot bash
```

Useful checks inside the container:

```bash
ros2 pkg prefix orbbec_camera
ros2 pkg prefix orbbec_camera_launcher
python -c "import torch, torchvision, ultralytics, onnxruntime, sam2, cv2, yaml"
ros2 run orbbec_camera list_devices_node
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

## Notes

- Source code is tracked; `build`, `install`, and `log` directories are
  generated.
- Keep robot model, calibration files, and camera topics consistent across the
  stack.
- Read the package-level README before operating a workflow that sends robot
  motion commands.
