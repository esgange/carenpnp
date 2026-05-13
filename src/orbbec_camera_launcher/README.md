# orbbec_camera_launcher

`orbbec_camera_launcher` provides a Tkinter operator GUI for scanning and
launching two Orbbec cameras by serial number.

When the GUI starts, it reads the saved camera slots and automatically scans for
connected Orbbec devices. Any configured slot whose serial number is detected is
launched automatically. If only one configured camera is connected, that camera
still starts and the GUI warns which slot is missing. If a launch process exits
during startup, the GUI reports the failed slot and keeps any successfully
started camera running.

## Run

```bash
ros2 launch orbbec_camera_launcher camera_launcher.launch.py
```

## Dependencies

This package has no pip-only runtime dependencies. It uses:

- `ros-humble-orbbec-camera` for `gemini_330_series.launch.py`
- `ros2launch` to spawn one Orbbec launch process per configured camera
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

On startup and on manual `Launch Cameras`, the GUI:

1. Scans connected Orbbec devices with `ros2 run orbbec_camera list_devices_node`.
2. Compares detected serial numbers with configured slots.
3. Launches every configured slot that is currently detected.
4. Warns for unconfigured slots or configured serial numbers that are not
   detected.
5. Warns if a launched camera process exits during the startup grace period.

The running status shows whether one or two camera launch processes are active.
