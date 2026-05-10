# DOBOT Pick and Place

ROS 2 workspace for DOBOT robot bringup, RGB-D perception, calibration,
operator tools, and pick/intercept workflows.

## Package Map

| Package | Path | Purpose |
| --- | --- | --- |
| `cr_robot_ros2` | `src/dobot_bringup_v4` | TCP bridge to DOBOT controllers; publishes robot state and exposes command services. |
| `dobot_msgs_v4` | `src/dobot_msgs_v4` | Custom DOBOT messages and service definitions. |
| `dobot_rviz` | `src/dobot_rviz` | URDFs, meshes, robot state publisher, and RViz configuration. |
| `camera_calibration` | `src/camera_calibration` | Eye-on-hand and eye-to-hand calibration tools. |
| `aruco_perception` | `src/aruco_perception` | RGB-D ArUco marker pose detection and calibrated camera TF publishing. |
| `obstacle_perception` | `src/obstacle_perception` | Live depth obstacles and persistent obstacle memory. |
| `tray_perception` | `src/tray_perception` | Tray teach/detect workflow with tray pose, vector, and dimensions output. |
| `tray_intercept` | `src/tray_intercept` | Operator console for intercepting moving trays from `tray_vector`. |
| `item_perception` | `src/item_perception` | Item teach/detect workflow using bin ROI profiles and item pose output. |
| `item_perception_yolo` | `src/item_perception_yolo` | YOLO/SAM2 item perception experiments using the bin ROI workflow. |
| `item_pick` | `src/item_pick` | Operator GUI and motion sequence for picking from `item_detect` output. |
| `motion_debug` | `src/motion_debug` | Live robot debug GUI and motion script editor/player. |
| `movement_calibration` | `src/movement_calibration` | Speed calibration for linear movement scripts. |
| `gripper_control` | `src/gripper_control` | GUI for DOBOT digital output gripper channels. |

Each package has its own `README.md` with launch commands, interfaces, and
operational notes.

## Recent Workflow Updates

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

The workspace ROS environment is controlled by
`config/dobot_bringup_v4/param.json` via `ros_domain_id` and
`ros_localhost_only`. Sourcing `install/setup.bash` exports
`ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` from that config, and repo launch
files set the same values before starting nodes.

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

SAM2 is installed in a workspace-local Python environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -c "import torch, torchvision, sam2; print(torch.__version__, torchvision.__version__, sam2.__file__)"
```

The editable checkout lives at `third_party/sam2`, and the starter SAM 2.1 tiny
checkpoint is at:

```text
third_party/sam2/checkpoints/sam2.1_hiera_tiny.pt
```

This host currently uses CPU PyTorch (`torch==2.5.1+cpu`,
`torchvision==0.20.1+cpu`) because no CUDA runtime was detected. Reinstall
PyTorch with CUDA and rerun the SAM2 editable install if this workspace is moved
to a GPU host.

## YOLO11 Workspace Install

YOLO11 is installed through Ultralytics in the same workspace-local Python
environment:

```bash
source .venv/bin/activate
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
| Color image | `/camera/color/image_raw` |
| Depth image | `/camera/depth/image_raw` |
| Camera info | `/camera/color/camera_info` |

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
non-empty `.yaml` file in that directory. You can override discovery with:

```bash
calibration_file:=/abs/path/to/axab_calibration.yaml
```

If calibration is enabled and no usable file exists, the launch fails early
instead of silently running with an invalid transform.

## Generated Runtime Data

Common generated paths:

| Path | Owner | Purpose |
| --- | --- | --- |
| `teach/items` | `item_perception`, `item_pick` | Item teach profiles and item-pick tool teach sidecars. |
| `config/bins` | `item_perception`, `item_pick` | Item runtime settings and active profile selection. |
| `teach/trays` | `tray_perception` | Dated tray teach profiles. |
| `config/trays` | `tray_perception` | Tray runtime settings and active/latest tray config. |
| `teach/bin_teach` | `item_perception` | Bin teach profiles. |
| `teach/bins_yolo` | `item_perception_yolo` | YOLO teach sessions, profiles, and model bundles. |
| `config/dobot_bringup_v4/param.json` | `dobot_bringup_v4` | Robot connection config. |
| `debug files/seek_frames` | `item_perception`, `tray_perception` | First/last frame seek artifacts. |
| `debug files/pick_cycle_movement_deltas` | `pick_cycle` | One movement delta debug text file per cycle. |
| `WORKSPACE_ROOT/calibration` | `camera_calibration`, `movement_calibration` | Calibration YAML, JSON, and CSV files. |
| `WORKSPACE_ROOT/config/motion_debug_scripts` | `motion_debug`, `movement_calibration` | Motion script JSON files. |

## Fresh Start for Teach/Detect State

To clear generated bin/tray profiles and runtime state:

```bash
find WORKSPACE_ROOT/teach/items -maxdepth 1 -type f -delete
find WORKSPACE_ROOT/teach/trays -maxdepth 1 -type f -delete
find WORKSPACE_ROOT/teach/bin_teach -maxdepth 1 -type f -delete
rm -f WORKSPACE_ROOT/config/bins/item_detect_runtime_settings.yaml
rm -f WORKSPACE_ROOT/config/trays/tray_detect_runtime_settings.yaml
rm -f WORKSPACE_ROOT/config/bins/item_pick_runtime_settings.json
```

Then rebuild the affected packages:

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select item_perception tray_perception item_pick --symlink-install
source install/setup.bash
```

## Orbbec Gemini 330 Series Example

```bash
source /opt/ros/humble/setup.bash
source WORKSPACE_ROOT/install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  device_preset:='High Accuracy' \
  enable_color:=true \
  enable_depth:=true \
  depth_registration:=true \
  align_target_stream:=COLOR \
  align_mode:=SW \
  enable_frame_sync:=true \
  enable_temporal_filter:=true \
  color_width:=848 color_height:=480 color_fps:=30 \
  depth_width:=848 depth_height:=480 depth_fps:=30 \
  enable_point_cloud:=false
```

The workspace does not require the camera point cloud because perception nodes
consume image, depth, and camera-info topics directly.

## Docker

The repository includes a Docker Compose setup for a full workspace build with
Orbbec packages.

Build the image:

```bash
cd WORKSPACE_ROOT
sudo docker compose build
```

Start a shell:

```bash
sudo docker compose run --rm dobot bash
```

Build inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/<repo>
rosdep install --from-paths src --ignore-src -r -y --skip-keys="opencv2 message_generation joint_state_publisher"
colcon build --symlink-install
```

For RViz GUI forwarding from Docker, allow local X11 access on the host first:

```bash
xhost +local:docker
```

## Notes

- Source code is tracked; `build`, `install`, and `log` directories are
  generated.
- Keep robot model, calibration files, and camera topics consistent across the
  stack.
- Read the package-level README before operating a workflow that sends robot
  motion commands.
