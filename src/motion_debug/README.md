# motion_debug

`motion_debug` is the primary operator GUI for live DOBOT state, manual motion
commands, IO/motion service calls, and motion script editing/playback. It is a
diagnostic and commissioning tool, not an autonomous pick workflow.

## Executable

| Executable | Purpose |
| --- | --- |
| `motion_debug_gui` | Tkinter GUI for status, manual control, and script playback. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select motion_debug
source install/setup.bash
```

## Run

```bash
ros2 launch motion_debug motion_debug.launch.py
```

Direct run:

```bash
ros2 run motion_debug motion_debug_gui
```

## Inputs

| Topic | Type |
| --- | --- |
| `/joint_states_robot` | `sensor_msgs/msg/JointState` |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` |
| `dobot_msgs_v4/msg/RobotStatus` | `dobot_msgs_v4/msg/RobotStatus` |

## Robot Services

The GUI calls services under:

```text
/dobot_bringup_ros2/srv
```

Common services used:

- `EnableRobot`
- `DisableRobot`
- `ClearError`
- `Stop`
- `MoveJog`
- `StopMoveJog`
- `StartDrag`
- `StopDrag`
- `Tool`
- `SetTool`
- `SetPayload`
- `CP`
- `SpeedFactor`
- `VelJ`
- `VelL`
- `AccJ`
- `AccL`
- `MovJ`
- `MovL`

## Motion Scripts

Scripts are stored by default in:

```text
WORKSPACE_ROOT/config/motion_debug_scripts
```

Script files are JSON and can include a top-level speed profile:

```json
{
  "speed_profile": {
    "cp": 80,
    "speed_factor": 40
  }
}
```

Legacy scripts with top-level `cp` and `speed_factor` are still loaded.

Script behavior:

- loading a script updates the GUI CP and SpeedFactor controls;
- running a script applies CP and SpeedFactor first;
- script points execute in order;
- the run button becomes a stop control while playback is active.

## Relationship to Other Packages

- `tray_intercept` handles tray tracking and intercept motion.
- `item_pick` handles item-pick execution.
- `movement_calibration` consumes motion scripts created here.

## Notes

- Start `cr_robot_ros2` before using live robot commands.
- Use conservative speed settings during commissioning.
- Motion scripts should be reviewed before playback because they send real robot
  motion commands.
