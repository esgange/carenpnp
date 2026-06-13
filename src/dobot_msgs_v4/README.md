# dobot_msgs_v4

`dobot_msgs_v4` defines the custom ROS 2 messages and services used by the
DOBOT workspace. It mirrors the DOBOT controller command surface and adds a few
workspace-specific interfaces for tray and bin workflows.

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select dobot_msgs_v4
source install/setup.bash
```

## Messages

| Message | Purpose |
| --- | --- |
| `RobotStatus.msg` | Robot status, mode, safety, and controller state fields. |
| `ToolVectorActual.msg` | Actual TCP/tool pose feedback from the robot controller. |
| `TrayVector.msg` | Tray pose, timing, velocity, and motion direction used by tray intercept. |

`RobotStatus.msg` includes controller connectivity, the raw DOBOT
`robot_mode`, and `is_idle` for the normal enabled-idle mode (`5`).

`TrayVector.msg` includes:

- tray pose in millimeters and degrees;
- first/last observation timestamps;
- observation window timing;
- velocity vector, speed, and direction unit vector.

## Services

The package contains the DOBOT command service definitions used by
`cr_robot_ros2`, including:

| Family | Examples |
| --- | --- |
| Robot state and safety | `EnableRobot`, `DisableRobot`, `ClearError`, `RobotMode`, `EmergencyStop` |
| Motion | `MovJ`, `MovL`, `MovJIO`, `MovLIO`, `Arc`, `Circle`, `MoveJog` |
| Relative motion | `RelMovJTool`, `RelMovLTool`, `RelMovJUser`, `RelMovLUser`, `RelJointMovJ` |
| Speed and acceleration | `SpeedFactor`, `VelJ`, `VelL`, `AccJ`, `AccL`, `CP` |
| IO and Modbus | `DI`, `DO`, `AI`, `AO`, `GetCoils`, `SetCoils`, `GetHoldRegs`, `SetHoldRegs` |
| Kinematics and frames | `PositiveKin`, `InverseKin`, `CalcUser`, `CalcTool`, `SetUser`, `SetTool` |
| Workspace workflows | `GetTrayDimensions`, `TrayInterceptStart` |

## Using These Interfaces

For a C++ package:

```xml
<!-- package.xml -->
<depend>dobot_msgs_v4</depend>
```

```cmake
# CMakeLists.txt
find_package(dobot_msgs_v4 REQUIRED)
ament_target_dependencies(your_target dobot_msgs_v4)
```

For a Python package:

```xml
<!-- package.xml -->
<exec_depend>dobot_msgs_v4</exec_depend>
```

```python
from dobot_msgs_v4.srv import MovL
from dobot_msgs_v4.msg import ToolVectorActual
```

## Related Packages

- `cr_robot_ros2` implements service servers for the controller API.
- `motion_debug` uses core motion, state, and IO services.
- `tray_perception` publishes `TrayVector`.
- `tray_intercept` consumes `TrayVector` and exposes `TrayInterceptStart`.
- `item_pick` reuses `TrayInterceptStart` for its operator service.

## Notes

- Build this package before packages that depend on its generated interfaces.
- Keep service definitions stable when possible; many operator GUIs and
  perception nodes import these types directly.
