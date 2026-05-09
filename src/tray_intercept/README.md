# tray_intercept

`tray_intercept` is the operator package for intercepting a moving tray from
`tray_perception` output. It waits for a fresh `tray_vector`, predicts an
intercept target, and dispatches a staged robot motion sequence.

## Executable

| Executable | Purpose |
| --- | --- |
| `tray_intercept` | Tkinter operator console and service endpoint for tray intercept motion. |

## Build

```bash
cd /home/erds/DOBOT_pickn_place
source /opt/ros/humble/setup.bash
colcon build --packages-select tray_intercept
source install/setup.bash
```

## Run

```bash
ros2 launch tray_intercept tray_intercept.launch.py
```

Direct run:

```bash
ros2 run tray_intercept tray_intercept
```

## Inputs

| Input | Type | Source |
| --- | --- | --- |
| `tray_vector` | `dobot_msgs_v4/msg/TrayVector` | `tray_detect` motion estimate. |
| `tray_axis_overlay` | `geometry_msgs/msg/PolygonStamped` | Live 2D tray origin and X/Y axes for the GUI preview. |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` | DOBOT bringup TCP feedback. |

The GUI automatically calls `tray_detect/get_tray_dimensions` to keep the tray
preview size synced when the service is available.

## Services

Service exposed by this package:

| Service | Type | Purpose |
| --- | --- | --- |
| `tray_intercept/track` | `std_srvs/srv/Trigger` | Arms the same intercept sequence as the GUI track button. |
| `tray_intercept/track_status` | `std_srvs/srv/Trigger` | Returns success while track is armed and waiting for a fresh tray vector. |
| `tray_intercept/start_sequence` | `dobot_msgs_v4/srv/TrayInterceptStart` | Arms and starts the intercept sequence. |

Robot services called under `/dobot_bringup_ros2/srv`:

- `CP`
- `SpeedFactor`
- `Stop`
- `MovL`
- `MovLIO`

Example:

```bash
ros2 service call /tray_intercept/start_sequence dobot_msgs_v4/srv/TrayInterceptStart \
"{tray_vector_wait_timeout_sec: 60.0, ee_intercept_speed_mm_s: 650.0, tray_intercept_x_offset_mm: 0.0, tray_intercept_y_offset_mm: 0.0, tray_standoff_z_mm: 100.0, follow_distance_mm: 200.0, post_follow_z_up_mm: 300.0, troubleshoot_tf_only: false}"
```

## Runtime Flow

When armed, the node:

1. Waits for a fresh `tray_vector`.
2. Sends `Stop`.
3. Computes an intercept goal in `base_link`.
4. Queues `MovL` to the intercept pose.
5. Queues `MovL` or `MovLIO` to follow in the tray motion direction.
6. Queues a post-follow Z-up move with continuous tray-direction follow.

Troubleshoot mode publishes goal TFs only and does not send robot motion.
The tray intercept move uses a fixed `650 mm/s` EE speed; the
`ee_intercept_speed_mm_s` service field is kept for compatibility. The GUI
angle control sets a manual `-90..90 deg` final EE pose angle: negative rotates
CCW, positive rotates CW, and zero preserves the current TCP orientation instead
of aligning the EE to the tray axes. The tray-direction follow move still uses
detected tray speed, and post-follow Z-up uses the configured arm max speed.
The tray standoff Z offset is applied in robot/base +Z, so positive Z remains
an upward standoff even if the detected tray frame has a downward natural Z.
Tray X/Y offsets are projected into the robot base XY plane before motion.

The preview origin is fixed at the lower-left tray corner. X/Y preview clicks
are converted from that displayed bottom origin and sent directly in the
canonical tray frame from `tray_detect`. The flat top-down preview uses live
2D axes from `tray_axis_overlay`, so it follows runtime tray orientation without
waiting for a seek `tray_vector`.

## Debug TF Frames

Published in the robot goal frame, default `base_link`:

- `tray_movel_goal_tcp`
- `tray_follow_goal_tcp`
- `tray_post_follow_zup_goal_tcp`

## Motion Calibration

The node auto-loads speed mapping from the newest non-empty file matching:

```text
~/DOBOT_pickn_place/calibration/relmovl_speed_calibration*.json
```

It also reads startup `CP` and `SpeedFactor` values from the same calibration
file and applies them once before the first real motion command.

## Tray Velocity Handling

- Speeds below `5 mm/s` are treated as noise.
- Very low-speed trays produce no follow translation, while the sequence and
  debug outputs remain valid.
- Prediction includes tray message age, decay, and command hysteresis.

## Notes

- Run `tray_detect` before arming so `tray_vector` is available.
- Use TF-only troubleshoot mode to validate intercept frames before real robot
  motion.
- Keep movement calibration current when robot speed behavior changes.
