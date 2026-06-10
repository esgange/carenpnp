# orbbec_camera_launcher

`orbbec_camera_launcher` provides a Tkinter operator GUI for scanning and
launching two Orbbec cameras by serial number.

When the GUI starts, it reads the saved camera slots and automatically scans for
connected Orbbec devices. Each configured slot is checked by serial number. If
only one configured camera is connected, the GUI logs which camera is missing and
still launches the connected camera without waiting for a dialog click. If no
configured cameras are connected, the GUI reports the condition and does not
launch any camera nodes.
Detected cameras are launched one at a time: the second camera starts only after
the first camera is verified by its ROS image topics.

## Run

```bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

Headless camera launch using the saved YAML mapping:

```bash
ros2 launch orbbec_camera_launcher camera_headless.launch.py
```

The headless launch scans connected Orbbec devices before starting camera
drivers. It starts only configured serial numbers that are currently detected;
if none of the selected configured cameras are connected, the launch exits
without creating camera nodes or empty topics. Limit the launch to specific
slots, names, or serial numbers with `enabled_cameras:=1,robot_camera`, and
override Orbbec launch arguments directly, for example:

```bash
ros2 launch orbbec_camera_launcher camera_headless.launch.py enabled_cameras:=robot_camera color_fps:=15
```

Set `require_connected:=false` only when you intentionally want the old behavior
of starting the Orbbec driver without a preflight serial scan.

## Dependencies

This package has no pip-only runtime dependencies. It uses:

- `ros-humble-orbbec-camera` for `gemini_330_series.launch.py`
- `ros2launch` to spawn one Orbbec launch process per configured camera
- A terminal emulator (`gnome-terminal`, `xterm`, `xfce4-terminal`, `konsole`,
  or `mate-terminal`) so launched camera drivers are visible and not hidden
- `python3-tk` for the operator GUI
- `python3-yaml` for reading and writing camera config

From the workspace root, those dependencies are installed from the frozen
apt/ROS bundle by:

```bash
tools/deps/install_offline_deps.sh --system-only
```

## Runtime Config

The camera serial/name mapping is read from and saved to:

```text
WORKSPACE_ROOT/config/camera_bringup/orbbec_cameras.yaml
```

The same file also stores the Orbbec launch arguments used for each camera:

```yaml
orbbec_launch_args:
  device_preset: High Accuracy
  enable_color: true
  enable_depth: true
  depth_registration: true
  align_target_stream: COLOR
  align_mode: SW
  enable_frame_sync: true
  enable_temporal_filter: true
  color_width: 848
  color_height: 480
  color_fps: 30
  depth_width: 848
  depth_height: 480
  depth_fps: 30
  enable_point_cloud: false
```

The GUI creates `WORKSPACE_ROOT/config/camera_bringup` when saving mappings and
preserves the launch argument block when serial/name mappings are updated.

Set `DOBOT_PICKN_PLACE_ROOT` or `DOBOT_WORKSPACE_ROOT` when launching from a
non-standard shell so the GUI can find the workspace-local config directory.

## Startup Behavior

The default slot names are:

| Slot | Camera name | Default topics |
| --- | --- | --- |
| 1 | `bin_camera` | `/bin_camera/color/image_raw`, `/bin_camera/depth/image_raw` |
| 2 | `robot_camera` | `/robot_camera/color/image_raw`, `/robot_camera/depth/image_raw` |

On startup auto-launch, any warnings about missing configured cameras are written
to the scan log without blocking; if at least one configured camera is detected,
that camera starts immediately. On manual `Launch Cameras`, warnings are still
shown to the operator before launch continues.

On startup and on manual `Launch Cameras`, the GUI:

1. Scans connected Orbbec devices with `ros2 run orbbec_camera list_devices_node`.
2. Compares detected serial numbers with configured slots.
3. If no configured cameras are connected, reports the condition and launches
   nothing.
4. If only one configured camera is connected, reports which camera
   is missing and launches only the connected camera.
5. Launches detected configured slots sequentially, one terminal per camera.
   The terminal title and banner identify the exact camera node running in that
   terminal.
6. Waits for `/CAMERA_NAME/color/image_raw` and
   `/CAMERA_NAME/depth/image_raw` before marking that camera as running and
   before starting the next camera.
7. Warns and stops that launch if the process exits or the required topics do
   not appear within the readiness timeout, then continues with any remaining
   detected configured camera.

The running status shows which camera is starting and which cameras are verified.
`Launch Cameras` is disabled while any tracked camera node is running or while a
launch sequence is in progress. `Stop Cameras` stays available; if no camera is
running, it reports that there are no camera nodes to stop. Window close sends
shutdown signals to the tracked process groups, so child camera driver nodes are
cleaned up with their launch terminal. If no supported terminal emulator is
available, the GUI refuses to launch camera drivers instead of creating hidden
background nodes.
