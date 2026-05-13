# ServoP Debug Linear Planner

Small ROS 2 GUI for testing a straight-line Cartesian `ServoP` trajectory from Point A to Point B.

The ROS package name is lowercase for ROS compatibility:

```bash
ros2 launch debug_servop debug_servop.launch.py
```

## Flow

1. Click `Get Pose` to copy the current TCP pose format: `x,y,z,rx,ry,rz`.
2. Paste/edit Point A and Point B.
3. Set `ServoP total time` in seconds.
4. Set `Acceleration level (%)` from 1% to 20%.
5. Set `Plan/send interval` from 200 ms to 400 ms.
6. Click `TF Only / Visualize` to publish the planned waypoints to RViz without robot motion.
7. After verifying the path, click `Run ServoP Planner`.

The motion test sends commands in this order:

- `MovJ(mode=false)` to Point A.
- A short settle delay.
- Wait for fresh TCP feedback to confirm the robot has reached Point A.
- Sequential `ServoP` commands along a straight-line A-to-B waypoint plan.

`MovL` is no longer used.

## Linear ServoP planner

The planner creates a straight Cartesian path from A to B and time-scales it with a trapezoidal velocity profile:

- The path variable `s` goes from `0.0` at Point A to `1.0` at Point B.
- Each waypoint linearly interpolates XYZ only: `x,y,z`.
- EE orientation is locked to Point A: `rx,ry,rz` do not rotate during the planned path or at the final waypoint.
- The planner/send interval is selected in the GUI from `200 ms` to `400 ms`.
- The selected interval is now treated as the target interval. The planner chooses an integer segment count and spreads all segments evenly across the requested total time. This removes the old short leftover final segment and makes the acceleration and deceleration sample spacing symmetric.
- Each sent `ServoP` command uses `t=<actual equalized segment time>`.


## Plan/send interval slider

The `Plan/send interval (ms)` slider controls the target waypoint spacing and streaming period used by the planner.

- Minimum target: `200 ms` (`~5 Hz`)
- Maximum target: `400 ms` (`~2.5 Hz`)
- Default target: `200 ms`

The planner now avoids a short final remainder segment. For example, if the total time and selected interval do not divide evenly, it picks the closest integer segment count and uses one equalized interval for every segment. A `1.4 s` move with a `300 ms` target creates `5` equal segments at `0.28 s` each instead of `0.3, 0.3, 0.3, 0.3, 0.2`. This makes the deceleration side match the acceleration side more closely.

## Orientation lock

The planner intentionally prevents end-effector spin by keeping the orientation from Point A for every planned waypoint:

```text
planned_pose = interpolated_x, interpolated_y, interpolated_z, point_a_rx, point_a_ry, point_a_rz
```

This means Point B orientation values are currently ignored during `ServoP` linear planning. The final waypoint also keeps Point A orientation, so the robot should not perform a hidden final 360 degree rotation.

## Acceleration level (%) slider

The slider does not change robot firmware gain. It changes how long the generated trajectory spends accelerating and decelerating:

- Lower acceleration percentage = longer ramp time = smaller position deltas near start/end = smoother.
- Higher acceleration percentage = shorter ramp time = sharper acceleration/deceleration.
- Segment timing is equalized across the whole move before the ramp is sampled, so deceleration no longer gets a short leftover interval at the end.

Current mapping:

- `1%` acceleration -> accel/decel ramp is `45%` of total time on each side.
- `20%` acceleration -> accel/decel ramp is `10%` of total time on each side.

The GUI logs the number of generated points, `dt`, ramp percentage, first/middle/last points, and streamed ServoP progress.

## Point A reach check before ServoP

Before any ServoP waypoint is sent, the node now verifies that the actual TCP feedback is close to Point A. This prevents the stream from starting while the robot is still traveling to the start pose.

Defaults in `debug_servop/servo_p_debug_gui.py`:

```python
POINT_A_REACH_TIMEOUT_SEC = 10.0
POINT_A_POSITION_TOLERANCE_MM = 2.0
POINT_A_ORIENTATION_TOLERANCE_DEG = 2.0
POINT_A_FEEDBACK_MAX_AGE_SEC = 1.0
```

The check uses Euclidean XYZ error and shortest-angle rx/ry/rz error. If Point A is not reached before timeout, the ServoP stream is aborted and no ServoP calls are sent.

## RViz visualization

The `TF Only / Visualize` button does not move the robot. It only publishes the generated plan.

In RViz:

1. Set Fixed Frame to the same frame used by the node, default: `base_link`.
2. Add a `TF` display. The generated frames use stable names like `servop_plan_000`, `servop_plan_001`, etc. A new plan replaces the active TF set instead of creating a new generation.
3. Add a `MarkerArray` display on topic:

```text
/debug_servop/trajectory_markers
```

Every new `TF Only / Visualize` or `Run ServoP Planner` action first publishes a marker `DELETEALL` and clears the active TF waypoint list, then publishes the new plan. The marker line and sphere list show the exact current waypoint path. TF frames are decimated if the generated plan has more than `MAX_TF_FRAMES = 120` points.

By default, Dobot TCP `x/y/z` values are assumed to be millimeters and are scaled to RViz meters with:

```python
TF_POSITION_SCALE_DEFAULT = 0.001
```

You can change the base frame and scale with ROS parameters:

```bash
ros2 run debug_servop servo_p_debug_gui --ros-args \
  -p tf_frame_id:=base_link \
  -p tf_position_scale:=0.001
```

## Fixed ServoP parameters

ServoP gain is intentionally not exposed in the GUI.

The planner still sends:

- `aheadtime=50`
- `gain=500`

These constants are in `debug_servop/servo_p_debug_gui.py`:

```python
SERVO_P_AHEADTIME = 50.0
SERVO_P_GAIN = 500.0
```

## Interfaces

Robot interfaces:

- TCP feedback: `/dobot_msgs_v4/msg/ToolVectorActual`
- MovJ service: `/dobot_bringup_ros2/srv/MovJ`
- ServoP service: `/dobot_bringup_ros2/srv/ServoP`

Visualization interfaces:

- MarkerArray topic: `/debug_servop/trajectory_markers`
- Dynamic TF child frames: `servop_plan_<index>`

## Notes

A successful ROS service response means the command was accepted by the Dobot interface; it is not a full guarantee that the physical robot has completed the whole motion. Test with conservative times and lower acceleration levels first.

If RViz still shows waypoint TF frames from an older version of this node, restart RViz once. Older code published generation-named static TF frames, and static TF frames cannot be deleted by a later node run.
