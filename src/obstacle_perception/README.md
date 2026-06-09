# obstacle_perception

`obstacle_perception` converts RGB-D camera data into live obstacle voxels and
an optional persistent obstacle memory cloud. It is intended for visualization,
scene awareness, and keeping remembered obstacles in a fixed frame while the
camera moves with the robot.

## Executables

| Executable | Purpose |
| --- | --- |
| `obstacle_perception_node` | Projects depth pixels into 3D, voxelizes them, and publishes live obstacles. |
| `obstacle_memory_node` | Accumulates stable obstacle voxels in a target frame. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select obstacle_perception
source install/setup.bash
```

## Launch

```bash
ros2 launch obstacle_perception obstacle_perception.launch.py
```

Disable persistent memory:

```bash
ros2 launch obstacle_perception obstacle_perception.launch.py enable_memory:=false
```

Use a specific calibration:

```bash
ros2 launch obstacle_perception obstacle_perception.launch.py \
  calibration_file:=/abs/path/to/axab_calibration_eyeonhand_09052026.yaml
```

## Launch Composition

The launch file starts:

- `aruco_perception.launch.py` with calibration enabled, so
  `Link6 -> arm_calibrated_camera_link` is published from the latest calibration;
- `obstacle_perception_node` for live obstacles;
- `obstacle_memory_node` when `enable_memory=true`.

If calibration is enabled and no usable calibration YAML exists, launch fails
early with a clear error.

## Inputs

| Topic | Type |
| --- | --- |
| `/robot_camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/robot_camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/robot_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

Camera topics can be overridden with `color_topic`, `depth_topic`, and
`camera_info_topic`.

## Outputs

| Topic | Type | Notes |
| --- | --- | --- |
| `/obstacles/points` | `sensor_msgs/msg/PointCloud2` | Live obstacle cloud. |
| `/obstacles/markers` | `visualization_msgs/msg/MarkerArray` | Live voxel markers. |
| `/obstacles/memory_points` | `sensor_msgs/msg/PointCloud2` | Persistent memory cloud when enabled. |

Live outputs are published in `arm_calibrated_camera_link` by default. Memory output
is transformed into `target_frame`, which defaults to `base_link`.

## Important Parameters

Live node:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `voxel_size` | `0.03` | Live voxel size in meters. |
| `pixel_stride` | `4` | Depth sampling stride. |
| `min_range` | `0.15` | Minimum accepted depth in meters. |
| `max_range` | `2.5` | Maximum accepted depth in meters. |
| `min_points_per_voxel` | `3` | Live voxel filtering threshold. |
| `publish_pointcloud` | `true` | Enable `/obstacles/points`. |
| `publish_markers` | `true` | Enable `/obstacles/markers`. |

Memory node:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `memory_voxel_size` | `0.03` | Memory voxel size in meters. |
| `memory_min_hits` | `30` | Hits required before memory publish. |
| `memory_max_voxels` | `400000` | Memory cap. |
| `memory_publish_rate` | `5.0` | Publish rate in Hz. |
| `target_frame` | `base_link` | Memory accumulation frame. |
| `frustum_enable` | `true` | Skip remembered points inside current camera frustum. |

## RViz

`dobot_rviz` already includes displays for:

- `/obstacles/points`;
- `/obstacles/markers`;
- `/obstacles/memory_points`.

## Quick Checks

```bash
ros2 topic hz /obstacles/points
ros2 topic hz /obstacles/markers
ros2 topic hz /obstacles/memory_points
ros2 run tf2_ros tf2_echo Link6 arm_calibrated_camera_link
```

## Notes

- Start the camera before launching obstacle perception.
- Keep calibration current when the camera mount moves.
- Increase `pixel_stride` or `voxel_size` to reduce CPU load.
- Tune frustum settings when memory obstacles disappear too aggressively or
  remain in the current line of sight.
