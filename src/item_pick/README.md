# item_pick

`item_pick` is the operator node for executing a pick sequence from
`item_detect` output. It consumes the selected item pose, applies a
profile-specific tool teach offset, and sends the robot through approach, pick,
retract, and final Z-up motions.

## Executable

| Executable | Purpose |
| --- | --- |
| `item_pick` | Tkinter operator GUI and service endpoint for item-pick execution. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select item_pick
source install/setup.bash
```

## Run

```bash
ros2 launch item_pick item_pick.launch.py
```

Headless service mode, with no Tkinter window:

```bash
ros2 launch item_pick item_pick.launch.py headless:=true
```

Important launch arguments are `runtime_settings_file`,
`item_profile_state_file`, `selected_profile_topic`, `motion_service_root`, `gripper_do_service`,
`di_status_topic`, `item_pose_topic`, `track_service`,
`track_status_service`, `item_seek_complete_service`,
`item_repick_service`, `auto_repick_service`, `repick_start_stability_sec`, and
`pick_motion_speed_percent`. Failed-pick retry is controlled by
`auto_repick_on_failed_suction` and defaults to `true`.
When `load_runtime_settings:=true`, the JSON runtime settings are loaded at
startup. In headless mode the JSON file must exist and include the complete
runtime key set; launch arguments are treated as overrides, not the normal place
to keep motion settings.

Direct run:

```bash
ros2 run item_pick item_pick
```

## Inputs

| Input | Type | Source |
| --- | --- | --- |
| `bin_seek_pose` | `geometry_msgs/msg/PoseStamped` | `item_detect` selected item pose. |
| `item_detect/selected_profile` | `std_msgs/msg/String` | Latched path of the item teach currently loaded by `item_detect`. |
| `/dobot_bringup_ros2/DIStatus_200mS` | `std_msgs/msg/String` | Slow DI status stream used for DI1 suction confirmation after retract. |
| `item_detect_selected_profile.txt` | text file | Active profile exported by `item_detect`. |

The selected-profile topic is the primary live handoff. The active profile
export remains a startup and backward-compatible fallback and is read from:

```text
WORKSPACE_ROOT/config/item_perception/item_detect_selected_profile.txt
```

GUI runtime settings are saved to:

```text
WORKSPACE_ROOT/config/item_perception/item_pick_runtime_settings.json
```

## Services

`item_pick` exposes:

| Service | Type | Purpose |
| --- | --- | --- |
| `item_pick/track` | `std_srvs/srv/Trigger` | Arms the same pick sequence as the GUI track button. |
| `item_pick/track_status` | `std_srvs/srv/Trigger` | Returns success while track is armed and waiting for a fresh item pose. |
| `item_pick/start_sequence` | `dobot_msgs_v4/srv/TrayInterceptStart` | Arms and starts the pick sequence with explicit settings. |
| `item_pick/set_auto_repick` | `std_srvs/srv/SetBool` | Enables/disables Auto Repick after a failed pickup. |

The `start_sequence` service type is shared with `tray_intercept`, so field
names contain `tray_*`; in this package those values are interpreted as
item-pick settings.

Example:

```bash
ros2 service call /item_pick/start_sequence dobot_msgs_v4/srv/TrayInterceptStart \
"{tray_vector_wait_timeout_sec: 60.0, ee_intercept_speed_mm_s: 350.0, tray_intercept_x_offset_mm: 0.0, tray_intercept_y_offset_mm: 0.0, ee_final_pose_angle_deg: 0.0, tray_standoff_z_mm: 100.0, follow_distance_mm: 200.0, post_follow_z_up_mm: 300.0, troubleshoot_tf_only: false}"
```

Robot service clients use the DOBOT bringup service root:

```text
/dobot_bringup_ros2/srv
```

Main robot services used:

- `GetPose` (one-shot current TCP read before pick-orientation selection)
- `GetAngle`
- `MovJ`
- `MovL`
- `MovLIO`
- `DO`

Item-detect coordination services used:

- `item_detect/seek_complete`
- `item_detect/repick`

## IO Map

| Channel | Purpose |
| --- | --- |
| `DO1` | Gripper close |
| `DO2` | Gripper open |
| `DO3` | Suction/vacuum on |
| `DO4` | Suction cup exhaust/purge |
| `DI1` | Active-high suction status input, read after retract |

The DI status topic defaults to `/dobot_bringup_ros2/DIStatus_200mS`.
`DI1` is read from bit `0` of the `digital_input_bits` JSON field.

## Tool Teach

Each item-detect profile requires saved tool teach data before arming. The
`tool_teach` block inside the item profile stores the Link6/tool offset,
operator pick heights, pick move speed, and pick-depth suction settle time for
that active item teach.

Embedded profile block:

```text
WORKSPACE_ROOT/teach/item_teach/item_<item_name>[_bin_<bin_name>]_<ddmmyyyy>.yaml
```

The GUI can:

- load embedded tool teach data for the active `item_detect` profile;
- preview the tool offset TF in RViz;
- save the EE position settings and tool offset with `Save EE + Tool Teach`;
- show a timestamped, scrolling Item Datalog of motion, IO, DI1, and repick steps;
- block arming when the active item profile has no saved tool teach.

Legacy `<item_name>_tool.yaml` sidecars are still readable for older profiles,
but new saves update the item profile directly.

## Motion Sequence

On trigger, the node arms for a fresh `bin_seek_pose`. When the pose arrives, it:

1. Saves the current six robot joints as the repick start position.
2. Builds the two valid long-axis item poses: preferred and 180-degree flipped.
3. Prefers the pose that keeps `arm_calibrated_camera_link` inside the active bin
   teach footprint. If both are outside, it logs a warning and continues with
   the preferred pose anyway.
4. Moves with `MovJ` to the approach pose above the pick goal.
5. Opens the gripper with suction and exhaust off.
6. Uses `MovLIO` at the configured pick motion speed to move to pick depth.
   The GUI/launch setting accepts `1..100%` and defaults to `10%`. `DO4`
   exhaust is forced off at the start of descent, and `DO3` suction turns on
   halfway down.
7. Waits for the descent to finish, then holds at pick depth with suction active
   for the configured `0.1..1.0s` suction settle time.
8. Queues a `MovL` return to the approach pose at the same speed, with the
   gripper still open and suction active. `RobotMode` must show the queued
   motion has finished and the controller has been idle for 100 ms.
9. Reads the first fresh `DI1` sample only after the retract motion is finished.
   When `DI1` is active, the node closes the gripper, queues final Z-up, and
   only then calls `item_detect/seek_complete`.
10. When that fresh `DI1` sample is inactive, final Z-up is skipped immediately
   and Seek remains ON. With **Auto Repick** enabled, the node automatically
   performs the same 300 ms release pulse as the **Release 300ms** button,
   returns by joint-mode `MovJ` to the saved repick start, re-arms itself, and
   calls `item_detect/repick`. Item Detect then publishes a newly acquired pose
   without toggling Seek OFF. With Auto Repick disabled, the node purges and
   returns to standby while Seek remains ON.

TF-only preview, motion errors, failed suction, and manual Stop do not call
`seek_complete`. The orchestrator therefore advances to placement only after
final Z-up has been accepted. While Seek is ON, the pose watchdog reports each
configured wait interval but remains armed instead of timing out the pick cycle.

Camera-bin pose preference can be disabled with `prefer_camera_inside_bin:=false`.
The checked frames default to `Link6` and `arm_calibrated_camera_link`.

## Debug TF Frames

When debug TF output is enabled, the node publishes target and tool-offset
preview frames including:

- `item_pick_tool_offset_preview`
- `item_movel_goal_tool_offset`
- `item_movel_goal_tool_axis_x_tip`
- `item_movel_goal_tool_axis_y_tip`
- `item_movel_goal_tool_axis_z_tip`

## Notes

- Run `item_detect` first so `bin_seek_pose` and the active profile export are
  available.
- Save tool teach for each new item profile before running a real pick.
- Use troubleshoot/TF-only mode to validate target frames before enabling robot
  motion.
