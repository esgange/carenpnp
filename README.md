# DOBOT Pick and Place

ROS 2 Humble workspace for a DOBOT pick-and-place cell: robot bringup,
RGB-D perception, camera/platform calibration, tray intercept, item detection,
item picking, operator consoles, and an optional RabbitMQ bridge for external
production orchestration.

The normal target is simple on purpose:

- Ubuntu 22.04
- ROS 2 Humble
- Native host install, no Docker required
- Core robot/camera runtime from apt/ROS packages
- Optional Python AI/RabbitMQ bundle only when YOLO/SAM2 or the external bridge
  is needed

For an offline arm/client PC, copy the repo together with the prepared
`third_party/` dependency bundle and follow [INSTALL.md](INSTALL.md).

## Quick Start

Use this path on an Ubuntu 22.04 machine that already has ROS 2 Humble
installed at `/opt/ros/humble`.

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  libyaml-cpp-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-tk \
  python3-vcstool \
  python3-yaml \
  qtbase5-dev

sudo rosdep init 2>/dev/null || true
rosdep update
```

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

If the Orbbec driver packages are not available through your configured apt/ROS
repositories, install them from the prepared offline bundle or from your local
Orbbec ROS 2 package source before building packages that depend on camera
messages. The expected package names are `ros-humble-orbbec-camera`,
`ros-humble-orbbec-camera-msgs`, and `ros-humble-orbbec-description`.

Configure this robot station:

```bash
cp -n station_config.example station_config
nano station_config
set -a
source station_config
set +a
```

Build/source terminals that should see the workspace:

```bash
cd ~/DOBOT_pickn_place
source /opt/ros/humble/setup.bash
source install/setup.bash
set -a
source station_config
set +a
```

Start the main operator console:

```bash
ros2 launch robot_cell_orchestrator robot_cell_orchestrator.launch.py
```

Or start the configured production support stack headlessly:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py mode:=online
```

## Dependency Model

Core ROS runtime does not require pip, a Python virtual environment, Docker, or
internet model downloads. It is built from the ROS packages in `src/` plus
system/ROS dependencies resolved by `rosdep` or by the frozen apt bundle.

Optional pieces are isolated:

| Need | Extra dependency path |
| --- | --- |
| YOLO/SAM2 item teach/detect | `third_party/.venv`, `third_party/sam2`, `third_party/yolo`, `requirements.lock.txt` |
| RabbitMQ external bridge | `cell_external_bridge` installed into `third_party/.venv` |
| Fully offline client PC | `third_party/debs`, `third_party/wheels`, `tools/deps/install_offline_deps.sh` |

On an internet-connected staging machine, prepare the offline bundle with:

```bash
tools/deps/fetch_offline_deps.sh
```

On a core-only offline/client PC:

```bash
tools/deps/install_offline_deps.sh --system-only
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

On a full offline/client PC that needs YOLO/SAM2 or the external bridge:

```bash
tools/deps/install_offline_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
tools/deps/verify_offline_env.sh
```

Only source the third-party Python environment in terminals that need YOLO/SAM2
or the RabbitMQ bridge:

```bash
source tools/deps/source_third_party_env.sh
```

## Package Map

| Package | Path | Purpose |
| --- | --- | --- |
| `cr_robot_ros2` | `src/dobot_bringup_v4` | TCP bridge to DOBOT controllers, robot state publishers, and command services. |
| `dobot_msgs_v4` | `src/dobot_msgs_v4` | Custom DOBOT messages and services used by robot-side packages. |
| `dobot_rviz` | `src/dobot_rviz` | URDFs, meshes, robot state publisher launch, and RViz config. |
| `orbbec_camera_launcher` | `src/orbbec_camera_launcher` | Tkinter GUI/headless launcher for named Orbbec cameras. |
| `camera_calibration` | `src/camera_calibration` | Eye-on-hand and eye-to-hand camera calibration. |
| `platform_calibration` | `src/platform_calibration` | Fixed platform/bin reference calibration. |
| `aruco_perception` | `src/aruco_perception` | RGB-D ArUco pose detection and calibrated camera TF publishing. |
| `obstacle_perception` | `src/obstacle_perception` | Live depth obstacles and persistent obstacle memory. |
| `tray_perception` | `src/tray_perception` | Tray teach/detect with tray pose, vector, and dimension output. |
| `tray_intercept` | `src/tray_intercept` | Operator console and motion service for tray interception. |
| `item_perception` | `src/item_perception` | Classic item teach/detect using bin ROI profiles and depth geometry. |
| `item_perception_yolo` | `src/item_perception_yolo` | Optional YOLO/SAM2 item teach/detect using the same item-pick handoff protocol. |
| `item_pick` | `src/item_pick` | Operator GUI and motion sequence for item pose pickup. |
| `robot_cell_orchestrator` | `src/robot_cell_orchestrator` | Main cell GUI, node launcher, runtime validation, and online API. |
| `gripper_control` | `src/gripper_control` | GUI for DOBOT gripper digital outputs. |
| `motion_debug` | `src/motion_debug` | Motion debug GUI and script player. |
| `movement_calibration` | `src/movement_calibration` | Movement speed calibration tools. |
| `cell_external_bridge` | `cell_external_bridge` | Optional RabbitMQ-facing bridge for one arm/client PC. |

Each package has a local `README.md` with launch arguments and interface notes.

## Station Config

`station_config` is the per-machine runtime file. It is sourced by shells and
read by launch files that need robot identity, ROS locality, and external bridge
settings.

Important fields:

```bash
ROS_LOCALHOST_ONLY=true
ROBOT_IP_ADDRESS=192.168.200.1
ROBOT_TYPE=cr10
ARM_NUMBER=1
ROBOT_ARM_ID=arm_01
```

`ROS_LOCALHOST_ONLY=true` keeps ROS discovery on this PC. The launch files also
set `ROS_LOCALHOST_ONLY=1` for the orchestrated nodes, so the default cell
runtime stays local unless you deliberately change the station config.

Robot Cell Orchestrator also exposes this robot IP in its **Robot Connection**
panel. Edit the value there and press **Save** to update `ROBOT_IP_ADDRESS` in
`station_config`; stop and relaunch Robot Bringup after changing it. The LAN2
debug/default controller IP is `192.168.200.1`.

The orchestrator camera feeds are opened from the **Camera Views** section with
**Open Camera Window**. The camera viewer stays in its own window so the main
operator controls remain visible; clicking the button again brings the camera
window to the front.

`ROS_DOMAIN_ID` is normally inherited from the shell environment. Set it before
launching if you need a non-default domain:

```bash
export ROS_DOMAIN_ID=0
```

## Normal Runtime

The orchestrator is the preferred entry point for users. It launches and monitors
the robot cell pieces from one console.

```bash
ros2 launch robot_cell_orchestrator robot_cell_orchestrator.launch.py
```

Typical manual/lab order when launching pieces yourself:

```bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py station_config:=station_config
ros2 launch dobot_rviz dobot_rviz.launch.py
ros2 launch item_perception item_detect.launch.py
ros2 launch item_pick item_pick.launch.py
ros2 launch tray_perception tray_detect.launch.py
ros2 launch tray_intercept tray_intercept.launch.py
```

The Orbbec camera launcher scans the saved camera serial mappings on startup.
When at least one configured camera is detected, it starts valid camera nodes
immediately; missing configured cameras are logged in the launcher instead of
blocking startup behind a warning click.

Headless production support stack:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py mode:=online
```

External RabbitMQ bridge, only when the conveyor/master PC is part of the run:

```bash
source tools/deps/source_third_party_env.sh
set -a
source station_config
set +a
cell-external-bridge
```

## Camera Topics

The two-camera setup normally uses these namespaces:

| Camera role | Color | Depth | Camera info |
| --- | --- | --- | --- |
| Bin camera | `/bin_camera/color/image_raw` | `/bin_camera/depth/image_raw` | `/bin_camera/color/camera_info` |
| Robot camera | `/robot_camera/color/image_raw` | `/robot_camera/depth/image_raw` | `/robot_camera/color/camera_info` |

Most perception launch files accept `color_topic`, `depth_topic`, and
`camera_info_topic` overrides:

```bash
ros2 launch item_perception item_detect.launch.py \
  color_topic:=/custom_camera/color/image_raw \
  depth_topic:=/custom_camera/depth/image_raw \
  camera_info_topic:=/custom_camera/color/camera_info
```

## Calibration And Teach Files

Generated robot-cell data is kept outside `src/`.

| Path | Owner | Purpose |
| --- | --- | --- |
| `calibration/` | camera/platform/movement calibration | Camera calibration YAML, platform calibration YAML, and movement calibration data. |
| `teach/bin_teach/` | item/tray perception | Bin profiles and ROI/depth-plane teach files. |
| `teach/item_teach/` | classic item perception, item pick | Item teach profiles, dimensions, and item-pick tool/EE settings. |
| `teach/item_teach_yolo/` | YOLO item perception | Final YOLO item profiles and model bundles. |
| `teach/tray_teach/` | tray perception | Tray profiles and taught tray dimensions. |
| `config/item_perception/` | item perception, item pick | Runtime settings and selected item profile handoff. |
| `config/tray_perception/` | tray perception, tray intercept | Runtime settings and selected tray profile handoff. |
| `config/camera_bringup/` | Orbbec launcher | Camera serial/name mapping and Orbbec launch options. |
| `config/robot_cell_orchestrator/` | orchestrator | Headless/runtime-stack launch settings. |
| `runtime/` | orchestrator and runtime nodes | Active online program files and process/runtime state. |
| `debug files/` and `Log/` | runtime nodes | Local debug captures and logs. |

Calibration discovery is robot-IP aware where the calibration filename includes
the IP. You can still override a calibration path explicitly with launch
arguments such as:

```bash
calibration_file:=/abs/path/to/axab_calibration_eyeonhand_09062026_192.168.200.1.yaml
```

Robot Cell Orchestrator calibration pickers filter by the expected file class:
eye-on-hand pickers show `axab_calibration_eyeonhand_*.yaml`, eye-to-hand
pickers show `axab_calibration_eyetohand_*.yaml`, and platform pickers show
`platform_calibration_*.yaml`. The selected file is still validated by YAML
metadata before it is accepted.

## Item Detect To Item Pick Handoff

Classic item detect and YOLO item detect both expose the same external protocol
for `item_pick`:

- Item detect publishes a selected teach/profile path for item pick settings.
- Seek ON performs one detection handoff and publishes the item target pose.
- Item pick arms, receives one pose, executes one pick routine, then returns to
  standby after final Z-up.
- If suction feedback fails after retract and Auto Repick is enabled, item pick
  releases, returns to the saved start joints, and calls the item detect repick
  service so detect reacquires without a manual Seek OFF/ON cycle.
- TF-only mode does not send motion commands. It publishes the goal TF so the
  pose can be checked in RViz, then returns item pick to standby.

See [src/item_perception/README.md](src/item_perception/README.md),
[src/item_perception_yolo/README.md](src/item_perception_yolo/README.md), and
[src/item_pick/README.md](src/item_pick/README.md) for the full topic/service
contract.

## YOLO/SAM2 Optional Flow

YOLO/SAM2 is not required for the core robot cell. Use it only in terminals that
source the frozen third-party Python environment:

```bash
source tools/deps/source_third_party_env.sh
ros2 launch item_perception_yolo item_teach_yolo.launch.py
ros2 launch item_perception_yolo item_detect_yolo.launch.py
```

The default CPU checkpoints live in the offline bundle:

```text
third_party/sam2/checkpoints/sam2.1_hiera_tiny.pt
third_party/yolo/checkpoints/yolo11n-seg.pt
```

YOLO detect keeps the ROS node name `item_detect`, so it can replace the classic
detect node for item-pick handoff when only one detect node is running.

## Repo Hygiene

Keep source and generated data separate:

- Commit source code, launch files, package manifests, docs, and intentional
  assets such as RViz meshes.
- Do not commit `build/`, `install/`, `log/`, `.venv/`, generated teach files,
  runtime configs, model training outputs, or local calibration captures unless
  you are deliberately packaging a release artifact.
- Large payloads under `third_party/debs`, `third_party/wheels`,
  `third_party/sam2`, and `third_party/yolo` are for offline transfer bundles.
  A plain git clone should be usable for source work, while a client-PC archive
  should include those ignored payloads.
- Prefer package-level README files for operator details, because robot motion
  workflows are safer when the launch commands and IO assumptions stay close to
  the package that owns them.

## Useful Checks

List ROS packages in the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon list
```

Build one package:

```bash
colcon build --packages-select item_pick --symlink-install
source install/setup.bash
```

Check ROS graph locality:

```bash
echo "$ROS_LOCALHOST_ONLY"
echo "$ROS_DOMAIN_ID"
ros2 node list
```

Verify the full offline bundle:

```bash
tools/deps/verify_offline_env.sh
```

## Troubleshooting

- If `ros2` is missing, source `/opt/ros/humble/setup.bash`.
- If a package cannot be found after a build, source `install/setup.bash`.
- If camera packages are missing, install `ros-humble-orbbec-camera` and
  `ros-humble-orbbec-camera-msgs`, or use the frozen apt bundle.
- If YOLO/SAM2 imports fail, source `tools/deps/source_third_party_env.sh` or
  reinstall the Python bundle with `tools/deps/install_offline_deps.sh --python-only`.
- If ROS nodes cannot see each other, check `ROS_LOCALHOST_ONLY`,
  `ROS_DOMAIN_ID`, and whether both terminals sourced the same workspace.
- If a workflow sends robot motion, verify robot IP, robot type, tool settings,
  calibration file, and camera topics before arming motion.
