# NoMachine Remote Desktop

This folder contains the helper script for enabling full remote desktop control
of this station PC over the LAN. Run it on the PC you want to control, meaning
the robot/cell node that runs this workspace.

NoMachine is separate from `cell_external_bridge`: it controls the whole Linux
desktop, including terminals, GUI tools, and ROS windows. It does not change the
bridge code or RabbitMQ behavior.

## Install On The Remote Node

For the expected Ryzen 7 node PC (`amd64` / `x86_64`) on Ubuntu 22.04 with
ROS 2 already installed, run the one-click bootstrap from the repository root:

```bash
./NoMachine/remote_ready_ubuntu22.sh
```

That script uses the bundled NoMachine package in `NoMachine/debs/`, installs
XFCE only if no desktop environment is present, opens TCP/4000 in UFW when UFW
is active, and enables NoMachine virtual-display mode when no monitor is
detected.

The lower-level installer is also available:

```bash
chmod +x NoMachine/install_nomachine_remote_access.sh
./NoMachine/install_nomachine_remote_access.sh
```

Robot Cell Orchestrator also runs the one-click bootstrap automatically on GUI startup
when NoMachine is not already ready. The **Services** panel shows
**NoMachine Remote Access** as the top row and turns it green when
`/etc/NX/nxserver --status` reports ready.

On first install, the GUI opens a terminal and the script may ask for the
Linux sudo password. After NoMachine is installed, later Robot Cell
Orchestrator launches only start/status-check the NoMachine service.
The bootstrap starts NoMachine with `--start-mode manual`, so it is available
when Robot Cell Orchestrator launches but is not configured as an always-on
boot service.

If the node is a minimal/headless Ubuntu install with no desktop environment
and you are using the lower-level installer directly:

```bash
./NoMachine/install_nomachine_remote_access.sh --install-xfce
```

If UFW is active and blocks LAN access:

```bash
./NoMachine/install_nomachine_remote_access.sh --open-ufw
```

If there is no monitor and NoMachine connects to a black, white, or frozen
screen, force NoMachine to create its own virtual display:

```bash
./NoMachine/install_nomachine_remote_access.sh --force-virtual-display
```

That command stops the local display manager, so it logs out any physical GUI
session. For robot/lab PCs, the most reliable headless setup is still an HDMI
dummy plug plus NoMachine.

## Connect From Your PC

Install NoMachine on your own PC too, then connect to:

```text
nx://REMOTE_NODE_IP:4000
```

Find the remote node IP with:

```bash
hostname -I
```

Login with the normal Linux username and password for the remote node.

## Notes

- The repo currently bundles the official NoMachine `amd64` DEB for Ryzen/x86_64
  nodes: `NoMachine/debs/nomachine_9.6.3_1_amd64.deb`.
- The installer prefers bundled DEBs from `NoMachine/debs/` and only downloads
  from NoMachine when the needed architecture is not bundled and `--offline-only`
  is not set.
- Refresh or add cached architectures with:

  ```bash
  ./NoMachine/cache_nomachine_debs.sh amd64
  ./NoMachine/cache_nomachine_debs.sh amd64 arm64 armhf
  ```

- It requires `sudo`. NoMachine itself is bundled for `amd64`; Ubuntu packages
  such as `xfce4`, `xfce4-terminal`, `dbus-x11`, `ufw`, `curl`, or
  `ca-certificates` are installed from the Ubuntu apt repositories when missing.
- NoMachine needs a desktop environment. Use `--install-xfce` if this machine
  only has a terminal/server install.
- Keep port `4000/tcp` on the LAN only. Do not expose it directly to the
  internet.
