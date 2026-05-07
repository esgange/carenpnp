# dobot_bringup_v4

`dobot_bringup_v4` contains the ROS package `cr_robot_ros2`, which bridges ROS 2
to a DOBOT CR-series controller over TCP. It exposes the DOBOT command API as ROS
services and publishes robot state for the rest of the workspace.

## Package Name

The folder and ROS package names differ:

| Folder | ROS Package |
| --- | --- |
| `src/dobot_bringup_v4` | `cr_robot_ros2` |

Use `cr_robot_ros2` in launch commands and package dependencies.

## Build

```bash
cd /home/erds/DOBOT_pickn_place
source /opt/ros/humble/setup.bash
colcon build --packages-select cr_robot_ros2
source install/setup.bash
```

## Configuration

Robot connection settings are stored in:

```text
src/dobot_bringup_v4/config/param.json
```

Installed launches read the installed copy under the package share directory.
With `--symlink-install`, source-side config edits are picked up directly.

Important fields:

| Field | Meaning |
| --- | --- |
| `robot_number` | Number of robot entries in the file. |
| `current_robot` | 1-based index of the active robot entry. |
| `node_info[].ip_address` | Robot controller IP address. |
| `node_info[].robot_type` | Robot model, such as `cr5`, `cr10`, `cr16`, `me6`, or `nova5`. |
| `node_info[].trajectory_duration` | Default trajectory duration parameter. |
| `node_info[].robot_node_name` | ROS node name for bringup. |

## Launch

```bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

Use a custom config:

```bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py \
  config:=/absolute/path/to/param.json
```

## Published State

| Topic | Type | Notes |
| --- | --- | --- |
| `joint_states_robot` | `sensor_msgs/msg/JointState` | Robot joint state stream. |
| `dobot_msgs_v4/msg/RobotStatus` | `dobot_msgs_v4/msg/RobotStatus` | Robot status and mode fields. |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` | TCP pose feedback. |
| `/dobot_bringup_ros2/msg/FeedInfo` | `std_msgs/msg/String` | JSON-like feedback payload. |

## Services

Services are exposed under:

```text
/dobot_bringup_ros2/srv
```

Common examples:

- `EnableRobot`
- `DisableRobot`
- `ClearError`
- `Stop`
- `MovJ`
- `MovL`
- `MovLIO`
- `SpeedFactor`
- `CP`
- `DO`

Example:

```bash
ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot {}
```

Motion service requests can pass controller argument tokens through
`param_value`, for example `v=`, `a=`, `tool=`, and `user=`.

## Related Packages

- `dobot_msgs_v4` defines the message and service interfaces.
- `dobot_rviz` visualizes robot state from `joint_states_robot`.
- `motion_debug`, `tray_intercept`, `item_pick`, and
  `movement_calibration` call the services exposed here.

## Notes

- Confirm the controller IP address before launching robot bringup.
- Start bringup before launching operator tools that call robot services.
- Keep the configured `robot_type` aligned with the physical robot so RViz and
  motion assumptions match the hardware.
