# item_pick_servo

`item_pick_servo` is the operator node for executing a pick sequence from
`item_detect` output. It consumes the selected item pose, applies a
profile-specific tool teach offset, and sends the robot through approach, pick,
retract, and final Z-up motions.

## Executable

| Executable | Purpose |
| --- | --- |
| `item_pick_servo` | Tkinter operator GUI and service endpoint for item-pick execution. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select item_pick_servo
source install/setup.bash
```

## Run

```bash
ros2 launch item_pick_servo item_pick_servo.launch.py
```

ServoP point runtimes can be adjusted from launch:

```bash
ros2 launch item_pick_servo item_pick_servo.launch.py \
  item_approach_servo_p_t_sec:=1.5 \
  item_final_z_up_servo_p_t_sec:=1.0 \
  return_to_tray_teach_servo_j_t_sec:=1.5
```

Direct run:

```bash
ros2 run item_pick_servo item_pick_servo
```

## Inputs

| Input | Type | Source |
| --- | --- | --- |
| `bin_seek_pose` | `geometry_msgs/msg/PoseStamped` | `item_detect` selected item pose. |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` | DOBOT bringup TCP feedback. |
| `item_detect_selected_profile.txt` | text file | Active profile exported by `item_detect`. |
| `tray_detect_runtime_settings.yaml` or `tray_teach_settings.yaml` | YAML file | Tray perception teach joints for the final teach-return move. |

The active profile export is read from:

```text
WORKSPACE_ROOT/config/item_perception/item_detect_selected_profile.txt
```

GUI runtime settings are saved to:

```text
WORKSPACE_ROOT/config/item_perception/item_pick_servo_runtime_settings.json
```

Tray teach/runtime state for the final teach-return move is read from:

```text
WORKSPACE_ROOT/config/tray_perception
```

## Services

`item_pick_servo` exposes:

| Service | Type | Purpose |
| --- | --- | --- |
| `item_pick_servo/track` | `std_srvs/srv/Trigger` | Arms the same pick sequence as the GUI track button. |
| `item_pick_servo/track_status` | `std_srvs/srv/Trigger` | Returns success while track is armed and waiting for a fresh item pose. |
| `item_pick_servo/start_sequence` | `dobot_msgs_v4/srv/TrayInterceptStart` | Arms and starts the pick sequence with explicit settings. |

The `start_sequence` service type is shared with `tray_intercept`, so field
names contain `tray_*`; in this package those values are interpreted as
item-pick settings.

Example:

```bash
ros2 service call /item_pick_servo/start_sequence dobot_msgs_v4/srv/TrayInterceptStart \
"{tray_vector_wait_timeout_sec: 60.0, ee_intercept_speed_mm_s: 350.0, tray_intercept_x_offset_mm: 0.0, tray_intercept_y_offset_mm: 0.0, ee_final_pose_angle_deg: 0.0, tray_standoff_z_mm: 100.0, follow_distance_mm: 200.0, post_follow_z_up_mm: 300.0, troubleshoot_tf_only: false}"
```

Robot service clients use the DOBOT bringup service root:

```text
/dobot_bringup_ros2/srv
```

Main robot services used:

- `Stop`
- `MovL`
- `MovLIO`
- `ServoP`
- `ServoJ`
- `DO`

## Tool Teach Sidecars

Each item-detect profile requires a saved tool teach sidecar before arming. The
sidecar stores the Link6/tool offset, operator pick heights, pre-pick settling,
and pickup-depth settling for the active item teach.

Sidecar pattern:

```text
WORKSPACE_ROOT/teach/item_teach/<item_name>_tool.yaml
```

The GUI can:

- load the sidecar for the active `item_detect` profile;
- preview the tool offset TF in RViz;
- save updated tool teach values with `Save Tool Teach`;
- block arming when the active item profile has no saved tool teach.

## Motion Sequence

On trigger, the node arms for a fresh `bin_seek_pose`. When the pose arrives, it:

1. Builds the two valid long-axis item poses: preferred and 180-degree flipped.
2. Prefers the pose that keeps `calibrated_camera_link` inside the active bin
   teach footprint. If both are outside, it logs a warning and continues with
   the preferred pose anyway.
3. Moves with `ServoP` to the approach pose above the pick goal, using
   `t=item_approach_servo_p_t_sec` default `1.5`, `aheadtime=50`, and `gain=500`.
4. Opens the gripper with suction off, then waits the configured pre-pick
   settling time.
5. Uses `MovLIO` at 6% speed factor to move to pick depth while triggering
   suction at the start of the descent, waits for TCP reach, then waits the
   configured pick settling time at pickup depth.
6. Closes the gripper, retracts to approach with `MovL`, waits for TCP reach,
   then moves to final Z-up with `ServoP` using
   `t=item_final_z_up_servo_p_t_sec` default `1.0`.
7. Reads the active tray perception teach profile and queues a final `ServoJ`
   return to that tray teach joint pose using
   `t=return_to_tray_teach_servo_j_t_sec` default `1.5`.

Camera-bin pose preference can be disabled with `prefer_camera_inside_bin:=false`.
The checked frames default to `Link6` and `calibrated_camera_link`.

## Debug TF Frames

When debug TF output is enabled, the node publishes target and tool-offset
preview frames including:

- `item_pick_servo_tool_offset_preview`
- `item_movel_goal_tool_offset`
- `item_movel_goal_tool_axis_x_tip`
- `item_movel_goal_tool_axis_y_tip`
- `item_movel_goal_tool_axis_z_tip`

## Notes

- Run `item_detect` first so `bin_seek_pose` and the active profile export are
  available.
- Save a tool teach sidecar for each new item profile before running a real pick.
- Use troubleshoot/TF-only mode to validate target frames before enabling robot
  motion.
