# gripper_control

`gripper_control` is a small operator diagnostic GUI for the DOBOT gripper and
suction tooling. It sends `dobot_msgs_v4/srv/DO` requests for output control
and subscribes to the slow 30005 DI stream for DI1 suction feedback.

## Executable

| Executable | Purpose |
| --- | --- |
| `gripper_control_gui` | Tkinter GUI for toggling `DO1`-`DO4`, viewing DI1 suction status, and running grip/release actions. |

## IO Map

| Channel | Purpose |
| --- | --- |
| `DO1` | Gripper close |
| `DO2` | Gripper open/release pulse |
| `DO3` | Suction/vacuum on |
| `DO4` | Suction cup exhaust |
| `DI1` | Active-high suction status input |

The `Suction Status` LED reads `digital_input_bits` from
`/dobot_bringup_ros2/DIStatus_200mS`. `DI1` is displayed from bit `0`, using
the usual `DI N` to bit `N-1` mapping.

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select gripper_control
source install/setup.bash
```

## Run

```bash
ros2 launch gripper_control gripper_control.launch.py
```

Direct run:

```bash
ros2 run gripper_control gripper_control_gui
```

Common overrides:

```bash
ros2 launch gripper_control gripper_control.launch.py \
  do_service:=/dobot_bringup_ros2/srv/DO \
  di_status_topic:=/dobot_bringup_ros2/DIStatus_200mS \
  auto_off_on_exit:=true
```

## Interface

| Interface | Type | Default |
| --- | --- | --- |
| `do_service` | `dobot_msgs_v4/srv/DO` | `/dobot_bringup_ros2/srv/DO` |
| `di_status_topic` | `std_msgs/msg/String` JSON payload | `/dobot_bringup_ros2/DIStatus_200mS` |
| `auto_off_on_exit` | boolean | `true` |

Expected DI status payload:

```json
{"digital_input_bits":1,"source_port":30005}
```

## GUI Behavior

- Provides independent controls for `DO1`, `DO2`, `DO3`, and `DO4`.
- Shows one read-only `Suction Status` LED from `DI1`.
- Sends `status=1` for ON and `status=0` for OFF.
- Uses immediate DO calls with `time=0`.
- Supports per-channel auto-off timing in milliseconds.
- Can force active outputs OFF on exit when `auto_off_on_exit=true`.

Quick actions:

| Action | Behavior |
| --- | --- |
| `Grip` | Sets `DO2` OFF, sets `DO4` OFF, sets `DO1` ON, and sets `DO3` ON for suction. |
| `Release` | Sets `DO1` OFF, sets `DO3` OFF, pulses `DO4` for suction exhaust, and pulses `DO2` for gripper release. |

Release timing:

| Output | Pulse |
| --- | --- |
| `DO4` exhaust | `250 ms` |
| `DO2` open/release | `100 ms` |

## Diagnostics

Check the DI1 source topic:

```bash
ros2 topic echo --field data /dobot_bringup_ros2/DIStatus_200mS
```

For DI1 active, `digital_input_bits` should include bit `0`, which is `1`.

## Notes

- Start `cr_robot_ros2` before opening the GUI so the DO service and 30005 DI
  status topic are available.
- Confirm output channel wiring before enabling suction or gripper hardware.
- Service response `res != -1` is treated as success.
