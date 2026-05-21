# DOBOT Pick and Place

ROS 2 workspace for DOBOT robot bringup, RGB-D perception, calibration,
operator tools, pick/intercept workflows, and RabbitMQ-facing external
orchestration.

The workspace is intended to run as a host-native ROS 2 Humble install on
Ubuntu 22.04. For client/arm PCs, the repo can be shipped with an offline
dependency bundle under `third_party/` so a target machine only needs Ubuntu,
ROS 2 Humble, and the copied workspace bundle. See [INSTALL.md](INSTALL.md) for
the full client-PC procedure.

## Package Map

| Package | Path | Purpose |
| --- | --- | --- |
| `cr_robot_ros2` | `src/dobot_bringup_v4` | TCP bridge to DOBOT controllers; publishes robot state and exposes command services. |
| `dobot_msgs_v4` | `src/dobot_msgs_v4` | Custom DOBOT messages and service definitions. |
| `dobot_rviz` | `src/dobot_rviz` | URDFs, meshes, robot state publisher, and RViz configuration. |
| `orbbec_camera_launcher` | `src/orbbec_camera_launcher` | Tkinter operator GUI for scanning, naming, and launching two Orbbec cameras. |
| `cell_external_bridge` | `cell_external_bridge` | RabbitMQ-facing external bridge for one arm; calls the Robot Cell Orchestrator online API. |
| `camera_calibration` | `src/camera_calibration` | Eye-on-hand and eye-to-hand calibration tools. |
| `aruco_perception` | `src/aruco_perception` | RGB-D ArUco marker pose detection and calibrated camera TF publishing. |
| `obstacle_perception` | `src/obstacle_perception` | Live depth obstacles and persistent obstacle memory. |
| `tray_perception` | `src/tray_perception` | Tray teach/detect workflow with tray pose, vector, and dimensions output. |
| `tray_intercept` | `src/tray_intercept` | Operator console for intercepting moving trays from `tray_vector`. |
| `item_perception` | `src/item_perception` | Item teach/detect workflow using bin ROI profiles and item pose output. |
| `item_perception_yolo` | `src/item_perception_yolo` | YOLO/SAM2 item perception experiments using the bin ROI workflow. |
| `item_pick` | `src/item_pick` | Operator GUI and motion sequence for picking from `item_detect` output. |
| `robot_cell_orchestrator` | `src/robot_cell_orchestrator` | Main robot-cell controller GUI and online API for coordinating pick/place flow. |
| `motion_debug` | `src/motion_debug` | Live robot debug GUI and motion script editor/player. |
| `movement_calibration` | `src/movement_calibration` | Speed calibration for linear movement scripts. |
| `gripper_control` | `src/gripper_control` | GUI for DOBOT digital output gripper channels. |

Each package has its own `README.md` with launch commands, interfaces, and
operational notes.

## Recent Workflow Updates

- Runtime/config folders are now grouped by owner under `config/`, including
  `item_perception`, `tray_perception`, `robot_bringup`, `camera_bringup`, and
  `motion_calibrate`.
- Station-level runtime setup now lives in root `station_config`. It is used by
  Cell External Bridge and by Robot Cell Orchestrator when launching Robot Bringup.
- Online production is now Robot Cell Orchestrator-owned: Cell External Bridge sends
  `/robot_cell_orchestrator/load_online_program`, `/robot_cell_orchestrator/validate_online_program`,
  `/robot_cell_orchestrator/start_online`, and `/robot_cell_orchestrator/place_online`, then watches
  `/robot_cell_orchestrator/events`.
- Cell External Bridge is now included in the offline Python bundle. The frozen
  wheel set includes the RabbitMQ client stack (`aio-pika` / `aiormq`) and the
  offline installer installs `cell_external_bridge` into `third_party/.venv`.
- A root [INSTALL.md](INSTALL.md) documents the full client-PC install flow:
  package the repo, copy it to a machine with Ubuntu 22.04 + ROS 2 Humble, run
  bundled dependency install, build, configure `station_config`, and smoke test.
- Switching Robot Cell Orchestrator to Online changes orchestration mode only.
  Start production support nodes from the Robot Cell Orchestrator Node Launcher
  or with `robot_runtime_headless.launch.py`.
- Teach outputs now use purpose-named folders: `teach/item_teach` for item
  profiles with embedded tool teach data and `teach/tray_teach` for tray
  profiles.
- Platform calibration now lives with the other calibration outputs in
  `calibration/platform_calibration_<platform_name>.yaml`, using a top-level
  `transform` block plus metadata.
- `ros_domain_id` was removed from the default robot config. `ROS_DOMAIN_ID` is
  normally inherited from the shell environment, while optional legacy configs
  can still provide `ros_domain_id`.
- `robot_cell_orchestrator` shows a live robot status in the controller GUI: Stop, Picking,
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
- Generated teach/runtime/model outputs are kept under `teach/`, `config/`,
  `runtime/`, `calibration/`, `debug files/`, or `Log/`. The `src/` tree should
  remain source code plus intentional assets such as RViz robot meshes.

## Prerequisites

- Ubuntu 22.04 host-native install.
- ROS 2 Humble installed at `/opt/ros/humble`.
- For development or online dependency refresh: `colcon`, `rosdep`, C++
  compiler, Python 3, and network access.
- For client/arm PC deployment: copy the full repo bundle, including
  `third_party/debs`, `third_party/wheels`, `third_party/sam2`, and
  `third_party/yolo`, then use [INSTALL.md](INSTALL.md).
- RGB-D camera publishing aligned color, depth, and camera-info topics.
- DOBOT controller reachable from the host machine.
- RabbitMQ broker reachable on the conveyor/master PC when running
  `cell-external-bridge`.

## Build

For development machines with normal package access, build from the workspace
root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -r -y
colcon build --symlink-install
source install/setup.bash
```

For offline client PCs, do not use `rosdep`; follow [INSTALL.md](INSTALL.md) and
run `tools/deps/install_offline_deps.sh` first.

## Offline Frozen Dependencies

The repo can be packaged with a frozen Ubuntu 22.04 + ROS 2 Humble dependency
bundle under `third_party/`. On an internet-connected machine, populate the
bundle:

```bash
tools/deps/fetch_offline_deps.sh
```

Copy the repo, including `third_party/debs`, `third_party/wheels`,
`third_party/sam2`, `third_party/yolo`, and `cell_external_bridge`, to the
target PC. Then install without network access:

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
runtime, bridge, AI/perception, and test environment, including RabbitMQ client
support, YOLO, ONNX Runtime, CPU PyTorch, and the vendored SAM2 runtime
dependencies. `requirements.lock.txt` is the full offline wheel closure used by
`tools/deps/install_offline_deps.sh`. ROS Python modules such as `rclpy`,
`launch`, `cv_bridge`, `python_qt_binding`, and Tkinter come from the frozen
apt/ROS bundle, so they are intentionally not pip requirements.

The installer also installs local editable packages from `third_party/sam2` and
`cell_external_bridge`, so the `sam2` import and `cell-external-bridge` command
are available from `third_party/.venv`.

The large `.deb`, wheel, checkpoint, and vendored source payloads are meant for
release archives or local transfer bundles. A plain git clone may not include
all ignored third-party binaries.

For the full client-PC process, including packaging, target install,
station-specific configuration, and smoke tests, see [INSTALL.md](INSTALL.md).

The workspace station config controls `ROS_LOCALHOST_ONLY` through
`station_config`. `ROS_DOMAIN_ID` is normally left to the shell environment; an
optional legacy `ros_domain_id` field is still honored if present in a custom
Robot Bringup JSON config.

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
`item_detect` defaults to the dual-camera bin stream under `/bin_camera`.

For example:

```bash
ros2 launch tray_perception tray_detect.launch.py \
  color_topic:=/custom_camera/color/image_raw \
  depth_topic:=/custom_camera/depth/image_raw \
  camera_info_topic:=/custom_camera/color/camera_info
```

## Recommended Runtime Order

For the production external-bridge path:

1. Edit `station_config` for the arm PC.
2. Build/source the workspace and source the frozen Python environment:

```bash
export DOBOT_PICKN_PLACE_ROOT="$PWD"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
source tools/deps/source_third_party_env.sh
```

3. Start Robot Cell Orchestrator:

```bash
ros2 launch robot_cell_orchestrator robot_cell_orchestrator.launch.py
```

4. Start support nodes either from the Robot Cell Orchestrator Node Launcher
   or from sourced terminals. For the terminal path:

```bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py station_config:=station_config
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py mode:=online
```

5. In Robot Cell Orchestrator, switch to Online and validate calibration,
   runtime files, and services.
6. Start Cell External Bridge from a second sourced host shell:

```bash
set -a
source station_config
set +a
cell-external-bridge
```

7. Let the external master send `cmd.load_program`, then `cmd.pick`, then
   `cmd.place`.

For manual/lab operation, launch the same components directly:

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
non-empty file for the required calibration mode in that directory. Robot-camera
workflows use `axab_calibration_eyeonhand_<ddmmyyyy>.yaml`; fixed/bin-camera
item workflows use `axab_calibration_eyetohand_<ddmmyyyy>.yaml`.
Platform calibration is saved in the same directory as
`platform_calibration_<platform_name>.yaml`.
You can override discovery with:

```bash
calibration_file:=/abs/path/to/axab_calibration_eyeonhand_09052026.yaml
```

If calibration is enabled and no usable file exists, the launch fails early
instead of silently running with an invalid transform.

## Generated Runtime Data

Generated runtime data should not live under `src/`. The source tree contains
ROS packages and intentional assets, especially RViz robot meshes under
`src/dobot_rviz/meshes`. Python `__pycache__` directories may appear locally
after launches or tests, but they are ignored and can be deleted.

Common generated paths:

| Path | Owner | Purpose |
| --- | --- | --- |
| `teach/item_teach` | `item_perception`, `item_pick` | Item teach profiles with embedded tool teach data. |
| `config/item_perception` | `item_perception`, `item_pick`, `robot_cell_orchestrator` | Item runtime settings and active profile selection. |
| `teach/tray_teach` | `tray_perception` | Dated tray teach profiles. |
| `config/tray_perception` | `tray_perception`, `tray_intercept`, `robot_cell_orchestrator` | Tray runtime settings and active/latest tray config. |
| `teach/bin_teach` | `item_perception`, `item_perception_yolo` | Dated bin teach profiles named `bin_<name>_<ddmmyyyy>.yaml`. |
| `teach/item_teach_yolo` | `item_perception_yolo` | Final YOLO teach bundles named like classic item teach profiles. |
| `config/item_perception_yolo` | `item_perception_yolo` | Scratch YOLO teach runtime sessions and training datasets. |
| `runtime` | `robot_cell_orchestrator`, `item_perception`, `tray_perception` | Online active bin/item/tray YAML set copied by `/robot_cell_orchestrator/load_online_program`. |
| `config/robot_cell_orchestrator` | `robot_cell_orchestrator` | Headless runtime-stack launch settings. |
| `station_config` | `cell_external_bridge`, `dobot_bringup_v4`, `robot_cell_orchestrator` | Station identity, RabbitMQ bridge config, and Robot Bringup connection config. |
| `config/robot_bringup/param.json` | `dobot_bringup_v4` | Legacy fallback Robot Bringup JSON config. |
| `config/camera_bringup/orbbec_cameras.yaml` | `orbbec_camera_launcher` | Orbbec camera serial/name mapping. |
| `debug files/seek_frames` | `item_perception`, `tray_perception` | First/last frame seek artifacts. |
| `debug files/robot_cell_orchestrator_movement_deltas` | `robot_cell_orchestrator` | One movement delta debug text file per cycle. |
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

## Notes

- Source code is tracked; `build`, `install`, and `log` directories are
  generated.
- Large offline dependency payloads under `third_party/debs`, `third_party/wheels`,
  `third_party/sam2`, and `third_party/yolo` are intended for release archives or
  full-folder transfer bundles, not a plain git clone.
- `src/dobot_rviz/meshes` is the main size contributor inside `src/`; those are
  robot visualization assets, not generated runtime artifacts.
- Keep robot model, calibration files, and camera topics consistent across the
  stack.
- Read the package-level README before operating a workflow that sends robot
  motion commands.
