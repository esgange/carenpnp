# platform_calibration

GUI tool for saving the fixed platform reference used by bin teach.

It consumes an eye-to-hand camera calibration
`axab_calibration_eyetohand_*.yaml`, starts ArUco/depth detection on the bin
camera, observes the same four-marker calibration board, and saves:

```text
WORKSPACE_ROOT/calibration/platform_calibration_<platform_name>_<ddmmyyyy>_<robot_ip>.yaml
```

The robot IP comes from `robot_ip_address`, `ROBOT_IP_ADDRESS`, then the root
`station_config`. Auto-discovery only considers files matching that current
robot IP. A legacy or custom-named file can still be supplied explicitly with
`calibration_file:=<path>`.

For the shared/default robot IP `192.168.200.1`, the input eye-to-hand
calibration is never auto-selected. Launch opens a file chooser, or requires
`calibration_file:=<path>` when no GUI is available.

Saving deletes older platform calibration files only when their filename has
the same robot-IP suffix. Legacy no-IP files and files for other robot IPs are
preserved. If the robot IP cannot be resolved, no older files are deleted.

Launch:

```bash
ros2 launch platform_calibration platform_calibration.launch.py
```

Default frame path:

```text
base_link -> bin_calibrated_camera_link -> aruco_marker_1..4
```

The saved YAML is a `base_link -> <platform_name>` transform. `bin_teach`
can use that platform frame when teaching bin poses.
