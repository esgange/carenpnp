# gripper_control

`gripper_control` is a small operator GUI for DOBOT digital outputs used by the
gripper and suction tooling. It sends `dobot_msgs_v4/srv/DO` requests to the
bringup service and provides quick grip/release actions.

## Executable

| Executable | Purpose |
| --- | --- |
| `gripper_control_gui` | Tkinter GUI for toggling DO channels and quick gripper actions. |

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
  auto_off_on_exit:=true
```

## Interface

| Interface | Type | Default |
| --- | --- | --- |
| `do_service` | `dobot_msgs_v4/srv/DO` | `/dobot_bringup_ros2/srv/DO` |

## GUI Behavior

- Provides independent controls for `DO1`, `DO2`, and `DO3`.
- Sends `status=1` for ON and `status=0` for OFF.
- Uses immediate DO calls with `time=0`.
- Supports per-channel auto-off timing in milliseconds.
- Can force active outputs OFF on exit when `auto_off_on_exit=true`.

Quick actions:

| Action | Behavior |
| --- | --- |
| `Grip` | Sets `DO1` ON and `DO3` ON for suction. |
| `Release` | Sets `DO1` OFF, sets `DO3` OFF, and pulses `DO2` for vent/release. |

## Notes

- Start `cr_robot_ros2` before opening the GUI.
- Confirm output channel wiring before enabling suction or gripper hardware.
- Service response `res != -1` is treated as success.
