# Offline Dependency Bundle

This workspace targets Ubuntu 22.04, ROS 2 Humble, and CPU-only AI packages by
default. The offline bundle lives under `third_party/` and is used for native
host installs.

For the full client-PC install flow, see the root [INSTALL.md](../../INSTALL.md).

## Internet-connected Machine

```bash
tools/deps/fetch_offline_deps.sh
```

This populates:

- `third_party/debs`
- `third_party/wheels`
- `third_party/manifest.yaml`

The large binary payloads are intended for release archives or local transfer
bundles. Do not assume a plain git clone includes every ignored `.deb`, wheel,
checkpoint, or vendored source checkout.

## Offline Target Machine

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

`requirements.txt` is the direct pip dependency list for runtime, bridge,
AI/perception, test code, and the vendored SAM2 checkout. `requirements.lock.txt`
is the complete wheel closure used for offline installs. ROS Python modules,
Tkinter, RViz, Orbbec, OpenCV C++ libraries, and build tools are frozen as
apt/ROS `.deb` packages instead.
