# movement_calibration

`movement_calibration` characterizes DOBOT `MovL`/`RelMovL` speed behavior using
motion scripts created in `motion_debug`. It writes calibration JSON and optional
TCP trace CSV files that downstream motion packages can load.

## Executables

| Executable | Purpose |
| --- | --- |
| `movement_calibration` | Calibration node that runs scripts, measures TCP motion, and writes fit output. |
| `movement_calibration_gui` | Lightweight GUI wrapper for launching the calibration node with common settings. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select movement_calibration
source install/setup.bash
```

## Run

Recommended node launch:

```bash
ros2 launch movement_calibration movement_calibration.launch.py
```

Direct node run:

```bash
ros2 run movement_calibration movement_calibration
```

GUI wrapper:

```bash
ros2 launch movement_calibration movement_calibration_gui.launch.py
```

or:

```bash
ros2 run movement_calibration movement_calibration_gui
```

## Required Preparation

Create calibration scripts with `motion_debug` in:

```text
WORKSPACE_ROOT/config/motion_debug_scripts
```

Default script names:

- `x_calibrate.json`
- `y_calibrate.json`
- `z_calibrate.json`

Each script should move primarily along one axis, keep orientation stable, and
include multiple speed factors. A practical starting range is `v=5` through
`v=60` in increments of `5`.

## Inputs and Services

| Interface | Default | Purpose |
| --- | --- | --- |
| `dobot_msgs_v4/msg/ToolVectorActual` | `dobot_msgs_v4/msg/ToolVectorActual` | TCP pose feedback. |
| `service_root` | `/dobot_bringup_ros2/srv` | Root for robot command services. |

Services used:

- `CP`
- `SpeedFactor`
- `MovL`

## Common Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| `scripts_dir` | `WORKSPACE_ROOT/config/motion_debug_scripts` | Folder containing calibration scripts. |
| `script_names_csv` | `x_calibrate,y_calibrate,z_calibrate` | Scripts executed in order. |
| `startup_cp` | `100` | Applied before calibration. |
| `startup_speed_factor` | `50` | Applied before calibration. |
| `goal_tolerance_mm` | `2.0` | Motion convergence tolerance. |
| `settle_time_sec` | `0.15` | Settling time before measuring. |
| `segment_timeout_sec` | `20.0` | Per-segment timeout. |
| `output_file` | dated calibration JSON | Empty value uses default dated output path. |
| `save_raw_trace` | `true` | Writes TCP trace CSV. |

Example:

```bash
ros2 launch movement_calibration movement_calibration.launch.py \
  startup_cp:=80 \
  startup_speed_factor:=40
```

## Output Files

Default JSON output:

```text
WORKSPACE_ROOT/calibration/relmovl_speed_calibration_<ddmmyyyy>.json
```

Default raw trace:

```text
WORKSPACE_ROOT/calibration/relmovl_speed_calibration_<ddmmyyyy>_tcp_trace.csv
```

The JSON includes:

- axis models for `x`, `y`, and `z`;
- a global model;
- startup CP and SpeedFactor settings;
- measurement filters;
- plateau/saturation diagnostics;
- raw segment measurements.

## Fit Filtering

Default filters exclude weak or saturated samples:

- command speed `v=100` is excluded;
- minimum commanded distance is `10 mm`;
- minimum measured distance is `5 mm`;
- minimum travel ratio is `0.25`;
- plateau exclusion is enabled by default.

## Notes

- Start `cr_robot_ros2` before running calibration.
- Review generated scripts in `motion_debug` before execution.
- Re-run calibration when robot speed settings, tooling, payload, controller
  configuration, or motion scripts change.
