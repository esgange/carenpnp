# Client PC Install Guide

This guide is for an arm/client PC that already has:

- Ubuntu 22.04, x86_64.
- ROS 2 Humble installed at `/opt/ros/humble`.
- This workspace copied onto the machine with the offline dependency bundle.

After the workspace is copied, the install does not need internet access. The
remaining system packages come from `third_party/debs`. Python wheels under
`third_party/wheels` are only needed when the PC will run YOLO/SAM2 item
perception or the RabbitMQ-facing `cell-external-bridge`.

## What Must Be In The Copied Workspace

A plain git clone is enough for normal source work, but it is not enough for an
offline client PC because large payloads are ignored by git.

For a core robot-cell client, the copied workspace needs:

```text
DOBOT_pickn_place/
  config/
  src/
  station_config.example
  third_party/debs/
  tools/deps/
```

For YOLO/SAM2 or the external bridge, also include:

```text
DOBOT_pickn_place/
  cell_external_bridge/
  third_party/wheels/
  third_party/sam2/
  third_party/yolo/
```

For full GUI remote desktop access to the robot/node PC, include `NoMachine/`.
The repository copy can then run:

```bash
./NoMachine/remote_ready_ubuntu22.sh
```

Quick check from the workspace root:

```bash
test -d third_party/debs
test -f third_party/manifest.yaml
```

Full bundle quick checks:

```bash
test -d third_party/wheels
test -f third_party/sam2/checkpoints/sam2.1_hiera_tiny.pt
test -f third_party/yolo/checkpoints/yolo11n-seg.pt
```

## Prepare The Bundle On The Source PC

Run this only on the development/staging PC, not on the offline client PC:

```bash
cd /home/erds/DOBOT_pickn_place
tools/deps/fetch_offline_deps.sh
```

For a full YOLO/SAM2 or bridge bundle, also run
`tools/deps/verify_offline_env.sh` after the bundle is installed.

Create a transfer archive from the parent directory. This keeps the dependency
payloads but leaves out local build output and the local virtual environment,
which will be recreated on the client PC:

```bash
cd /home/erds
tar \
  --exclude='DOBOT_pickn_place/build' \
  --exclude='DOBOT_pickn_place/install' \
  --exclude='DOBOT_pickn_place/log' \
  --exclude='DOBOT_pickn_place/third_party/.venv' \
  --exclude='DOBOT_pickn_place/third_party/.apt-cache' \
  --exclude='DOBOT_pickn_place/third_party/.apt-state' \
  -czf DOBOT_pickn_place_client_bundle.tar.gz \
  DOBOT_pickn_place
sha256sum DOBOT_pickn_place_client_bundle.tar.gz > DOBOT_pickn_place_client_bundle.tar.gz.sha256
```

Move `DOBOT_pickn_place_client_bundle.tar.gz` to the client PC by USB drive,
local LAN copy, or whatever transfer method is available.

## Install On The Client PC

Unpack the workspace:

```bash
cd ~
tar -xzf /path/to/DOBOT_pickn_place_client_bundle.tar.gz
cd ~/DOBOT_pickn_place
export DOBOT_PICKN_PLACE_ROOT="$PWD"
```

Install the frozen local system dependencies for the core robot cell:

```bash
tools/deps/install_offline_deps.sh --system-only
```

For a full client PC that will run YOLO/SAM2 or `cell-external-bridge`, install
both system and Python dependencies:

```bash
tools/deps/install_offline_deps.sh
```

The script uses local files only:

- `third_party/debs/*.deb` through `dpkg` and `apt-get --no-download`.
- Optional `third_party/wheels/*` through `pip --no-index`.
- Optional `third_party/sam2` as an editable local package.
- Optional `cell_external_bridge` as an editable local package.

Build the ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify the full offline Python/AI/bridge install:

```bash
tools/deps/verify_offline_env.sh
```

Expected final line:

```text
Offline environment verification passed.
```

## Configure The Arm PC

Edit the station file:

```bash
cp -n station_config.example station_config
nano station_config
```

Set these values for the specific arm:

```bash
RABBITMQ_URL=amqp://catarm-server:<password>@<conveyor-pc-ip>:5672/%2Fcatarm
RABBITMQ_EXCHANGE=catarm.events
MESSAGE_SIGNING_KEY=<same-key-as-conveyor-pc>
BELT_ID=belt-a
CELL_BRIDGE_ID=edge_01
ROBOT_ARM_ID=arm_01
ARM_NUMBER=1
ROS_LOCALHOST_ONLY=true
ROBOT_IP_ADDRESS=192.168.200.1
ROBOT_TYPE=cr10
```

Use a unique `ARM_NUMBER`, `CELL_BRIDGE_ID`, and `ROBOT_ARM_ID` per robot arm.
`MESSAGE_SIGNING_KEY` must match the conveyor/master PC exactly.

## Source The Environment In Every Terminal

Use this from the workspace root:

```bash
cd ~/DOBOT_pickn_place
export DOBOT_PICKN_PLACE_ROOT="$PWD"
source tools/deps/source_third_party_env.sh
set -a
source station_config
set +a
```

`source_third_party_env.sh` sources ROS, the workspace overlay, and
`third_party/.venv` when they exist. For core-only terminals, sourcing
`/opt/ros/humble/setup.bash`, `install/setup.bash`, and `station_config` is
enough.

## Run The System

Start the full runtime stack in one terminal:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py
```

Or start the operator GUI:

```bash
ros2 launch robot_cell_orchestrator robot_cell_orchestrator.launch.py
```

Start the RabbitMQ-facing bridge in another sourced terminal:

```bash
cell-external-bridge
```

The bridge should connect to RabbitMQ, bind the arm command routing keys, and
publish heartbeat telemetry.

## Smoke Test Without Robot Motion

Use this when the conveyor RabbitMQ broker is reachable but the robot/perception
stack is not ready yet:

```bash
cd ~/DOBOT_pickn_place
export DOBOT_PICKN_PLACE_ROOT="$PWD"
source tools/deps/source_third_party_env.sh
set -a
source station_config
set +a
export CELL_BRIDGE_SIMULATE_MODE=success
cell-external-bridge
```

The conveyor side should see heartbeat/status messages and fake successful
responses for accepted `pick` and `place` commands.

## Notes And Boundaries

- Do not run `rosdep install` on the client PC unless it has internet access.
  Use `tools/deps/install_offline_deps.sh` for the frozen local install.
- The arm PC does not need to install Python packages from the internet.
- The RabbitMQ broker itself is not bundled here. It normally runs on the
  conveyor/master PC.
- CUDA/GPU Python packages are not bundled. The current offline lock is CPU
  PyTorch.
- Hardware still has to be reachable: DOBOT controller over the robot network,
  Orbbec cameras over USB/network as configured, and RabbitMQ over LAN.

## Quick Troubleshooting

If `ros2` is missing:

```bash
source /opt/ros/humble/setup.bash
```

If a ROS package is missing after build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If `cell-external-bridge` is missing:

```bash
tools/deps/install_offline_deps.sh --python-only
source tools/deps/source_third_party_env.sh
command -v cell-external-bridge
```

If RabbitMQ connection fails, first check the configured URL:

```bash
grep '^RABBITMQ_URL=' station_config
```

If `nc` is installed, also check the broker port from the arm PC:

```bash
nc -vz <conveyor-pc-ip> 5672
```

If workspace-root paths look wrong, make sure this is set before launching:

```bash
export DOBOT_PICKN_PLACE_ROOT="$PWD"
```
