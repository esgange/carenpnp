# Offline Dependency Bundle

This workspace targets Ubuntu 22.04 and ROS 2 Humble. The core robot-cell
runtime uses apt/ROS packages. CPU-only AI packages are bundled separately for
YOLO/SAM2 item perception and the optional RabbitMQ bridge.

The offline bundle lives under `third_party/` and is used for native host
installs without internet access.

For the full client-PC install flow, see the root [INSTALL.md](../../INSTALL.md).

## Internet-connected Machine

```bash
tools/deps/fetch_offline_deps.sh
```

This populates:

- `third_party/debs`
- `third_party/wheels`
- `third_party/manifest.yaml`

`third_party/debs` is the core ROS/system package bundle. `third_party/wheels`,
`third_party/sam2`, and `third_party/yolo` are only needed for YOLO/SAM2 or
`cell-external-bridge`.

The large binary payloads are intended for release archives or local transfer
bundles. Do not assume a plain git clone includes every ignored `.deb`, wheel,
checkpoint, or vendored source checkout.

## Offline Target Machine

Core robot-cell system packages only:

```bash
tools/deps/install_offline_deps.sh --system-only
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Full bundle, including YOLO/SAM2 and the external bridge:

```bash
tools/deps/install_offline_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
tools/deps/verify_offline_env.sh
```

Use `source tools/deps/source_third_party_env.sh` in shells that need the frozen
Python environment.

## Requirement Files

`requirements.txt` is the direct pip dependency list for the optional bridge,
AI/perception, test code, and the vendored SAM2 checkout. `requirements.lock.txt`
is the complete wheel closure used for full offline installs. ROS Python
modules, Tkinter, RViz, Orbbec, OpenCV C++ libraries, and build tools are frozen
as apt/ROS `.deb` packages instead.
