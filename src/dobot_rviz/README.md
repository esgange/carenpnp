# dobot_rviz

`dobot_rviz` provides robot URDF assets and an RViz configuration for the DOBOT
workspace. It reads the active robot model from `cr_robot_ros2` configuration,
starts `robot_state_publisher`, and opens RViz with the project visualization
layout.

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select dobot_rviz
source install/setup.bash
```

## Launch

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

## What the Launch Does

- Reads `robot_type` from `WORKSPACE_ROOT/config/dobot_bringup_v4/param.json`.
- Selects the matching URDF from `urdf/`.
- Falls back to `cr5_robot.urdf` if the configured model has no URDF.
- Starts `robot_state_publisher`.
- Remaps `joint_states` to `/joint_states_robot`.
- Publishes a static `world -> base_link` transform.
- Starts RViz with `rviz/urdf.rviz`.

## Supported URDF Models

- `cr3`
- `cr5`
- `cr7`
- `cr10`
- `cr12`
- `cr16`
- `cr20`
- `cr30h`
- `me6`
- `nova2`
- `nova5`

## Default RViz Displays

The provided RViz config includes displays for:

- robot model and TF tree;
- tray pose and tray cube marker;
- live obstacle markers and point cloud;
- persistent obstacle memory point cloud;
- common debug TF frames from tray and bin workflows.

Useful debug TFs include:

- `tray_movel_goal_tcp`
- `tray_follow_goal_tcp`
- `tray_post_follow_zup_goal_tcp`
- item-pick tool offset preview frames when `item_pick` publishes them.

## Notes

- Keep `robot_type` in `WORKSPACE_ROOT/config/dobot_bringup_v4/param.json` aligned with the
  physical robot.
- Run perception packages alongside RViz to populate obstacle, tray, bin, and
  debug overlays.
- If RViz opens without robot motion, confirm `/joint_states_robot` is being
  published by bringup.
