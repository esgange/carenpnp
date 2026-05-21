# robot_cell_orchestrator

Robot cell orchestrator GUI for coordinating the item-pick, tray-intercept,
runtime, and online external-bridge flow.

Cycle sequence:

Readiness gate:

- Create the service clients once when the node starts.
- Scan calibration files, teach/runtime files, and service availability while
  the GUI is open.
- Re-check all required services at the start of every cycle; stop the cycle if
  any required service is unavailable.

1. `/item_detect/go_to_teach`
2. `/item_pick/track`
3. Verify `/item_pick/track_status` reports armed
4. Monitor robot TCP feedback until it is stable
5. `/item_detect/seek`
6. Wait for `/item_detect/seek_status` to turn on, then off
7. `/tray_detect/go_to_teach`
8. `/tray_intercept/start_sequence` with the orchestrator tray X/Y/RZ settings
9. Verify `/tray_intercept/track_status` reports armed
10. Monitor robot TCP feedback until it is stable
11. `/tray_detect/seek`
12. Wait for `/tray_detect/seek_status` to turn on, then off

Item services and seek services are sent as virtual clicks. Tray arm uses
`dobot_msgs_v4/srv/TrayInterceptStart` so the orchestrator can pass tray intercept
X/Y offsets in millimeters and final EE RZ angle in degrees. The GUI ignores
data returned by seek services and no longer uses pose topics, fixed seek
timeouts, fixed robot-stop waits, or a required motion-detected gate. It waits
for each detect node's Seek status to turn on after the seek command, then turn
off again, so an old OFF state cannot be mistaken for completion. The detect
node's own configurable seek window controls the maximum seek time. It only watches
`/dobot_msgs_v4/msg/ToolVectorActual`, the same TCP feedback topic used by
`item_pick` and `tray_intercept`. After each arm/start call, the GUI verifies
the corresponding armed-status service before waiting for TCP stability and
sending seek.

The robot is treated as stable when TCP feedback stays within 1 mm linear and
1 degree rotational for the selected stability time. The stability timer is
based on live feedback time, not a fixed number of frames. A 30 second internal
watchdog prevents robot monitoring from hanging forever and logs the last
observed TCP delta when it cannot classify stability.

## Launch

Source the workspace first:

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the Robot Cell Orchestrator GUI:

```bash
ros2 launch robot_cell_orchestrator robot_cell_orchestrator.launch.py
```

Launch the runtime stack without Robot Cell Orchestrator GUI:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py
```

This starts the configured Orbbec cameras, `item_detect`, `item_pick` in
headless service mode, `tray_detect`, and `tray_intercept` in headless service
mode. RViz is available as a launch argument and is off by default:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py launch_rviz:=true
```

The headless stack defaults to `mode:=online`, so item/tray detect profiles come
from `WORKSPACE_ROOT/runtime`. Use offline teach folders instead with:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py mode:=offline
```

The normal headless values are stored in:

```text
WORKSPACE_ROOT/config/robot_cell_orchestrator/robot_runtime_headless_settings.yaml
```

That file owns camera launch selection, RViz on/off, online/offline profile
dirs, topics, service names, calibration paths, and child runtime settings file
paths. Launch arguments are overrides only, for example:

```bash
ros2 launch robot_cell_orchestrator robot_runtime_headless.launch.py launch_rviz:=true mode:=offline
```

The motion settings for headless `item_pick` and `tray_intercept` are loaded
from their JSON runtime files in `config/item_perception` and
`config/tray_perception`. In headless mode those JSON files must exist and
contain all required keys; the nodes will not silently continue with launch
defaults when runtime settings are incomplete.

Direct run commands are also available:

```bash
ros2 run robot_cell_orchestrator robot_cell_orchestrator_gui
```

`robot_cell_orchestrator_gui` arms `item_pick` and `tray_intercept`, then coordinates the
shared `item_detect` and `tray_detect` seek services.

Robot Cell Orchestrator GUI runtime knobs and window size are saved in:

```text
WORKSPACE_ROOT/config/robot_cell_orchestrator/robot_cell_orchestrator_runtime_settings.yaml
```

This file stores Step Mode, tray seek stability, tray placement X/Y/RZ offsets,
and the Robot Cell Orchestrator window geometry. The loop and Auto Repick
checkboxes are session-only; Auto Repick is currently UI-only.

Switching the Robot Cell Orchestrator GUI between **Offline** and **Online**
only changes the orchestration mode. It does not launch or stop robot,
perception, RViz, or camera nodes. Start and stop support nodes from the
right-side **Node Launcher** when you want them running:

- `Robot Bringup`
- `RViz`
- `Item Pick`
- `Item Detect`
- `Tray Intercept`
- `Tray Detect`

When the Node Launcher headless toggle is ON, `Item Pick` and `Tray Intercept`
use `headless:=true` and load their JSON runtime settings. `Item Detect` and
`Tray Detect` use `headless:=true` and receive the active teach file through
`selected_profile_path:=...`. Loading an online program updates runtime files and
settings only; it does not auto-start or auto-restart those nodes. Use
**Validate** or **Start Cycle** to check service readiness after the needed
nodes are launched manually.

## Online and Offline Modes

`robot_cell_orchestrator_gui` defaults to **Offline** mode. Offline uses:

```text
WORKSPACE_ROOT/teach/bin_teach
WORKSPACE_ROOT/teach/item_teach
WORKSPACE_ROOT/teach/tray_teach
```

The offline dropdowns choose the bin, item, and tray teach files used for the
manual cycle gate. When Robot Cell Orchestrator launches offline item/tray detect nodes, it
passes the selected item/tray teach file as `selected_profile_path:=...`. It does
not copy teach files or remember the selected teach in detect runtime settings.

The right panel also has an **External Bridge** control above **Node Launcher**.
When Robot Cell Orchestrator is in Offline mode, that button starts `cell_external_bridge` in
its own terminal in offline debug mode. It listens to the external RabbitMQ
command stream, writes JSONL datalogs under
`debug files/cell_external_bridge/`, and publishes fake
`load_program_ok` / `pick_ok` / `place_ok` responses. Offline debug does not
write `runtime/` and does not call Robot Cell Orchestrator ROS services.

**Online** mode reads only:

```text
WORKSPACE_ROOT/runtime
```

That folder is owned by Robot Cell Orchestrator in online mode. The external server sends
teach filenames through `/robot_cell_orchestrator/load_online_program`; Robot Cell Orchestrator validates
the requested local teach files, clears old runtime YAMLs, and copies the active
bin/item/tray set into `runtime/`. Tool teach data is normally embedded inside
the item profile.
Online runtime validation requires exactly one YAML of each type:

- root key `bin_teach`
- root key `item_detect`
- root key `tray_detect`

The item profile must include a top-level `tool_teach` block. A single legacy
YAML with root key `tool_teach_version` is still accepted for older profiles.

Unknown YAML files block online readiness. Non-YAML files are ignored.

Cycle start is blocked until all three calibration classes are present:

```text
WORKSPACE_ROOT/calibration/axab_calibration_eyeonhand_*.yaml
WORKSPACE_ROOT/calibration/axab_calibration_eyetohand_*.yaml
WORKSPACE_ROOT/calibration/platform_calibration_*.yaml
```

The external online start API is:

```bash
ros2 service call /robot_cell_orchestrator/start_online std_srvs/srv/Trigger {}
```

The external online program-load API is:

```bash
ros2 service call /robot_cell_orchestrator/load_online_program dobot_msgs_v4/srv/LoadOnlineProgram \
"{qqc_id: water_bottle, bin_teach_file: bin_blue_bin_08052026.yaml, item_teach_file: item_paper_cutlery_bin_blue_bin_08052026.yaml, tray_teach_file: tray_blue_tray_06052026.yaml, tray_x_mm: 25.0, tray_y_mm: 30.0, tray_rz_deg: 5.0}"
```

The external online place release API is:

```bash
ros2 service call /robot_cell_orchestrator/place_online std_srvs/srv/Trigger {}
```

The online validation API is:

```bash
ros2 service call /robot_cell_orchestrator/validate_online_program std_srvs/srv/Trigger {}
```

Validation succeeds only in Online mode when calibration, runtime files, and
required services are ready. The start service also requires the GUI to already
be in Online mode; Offline mode is treated as maintenance override and is not
controlled by Cell External Bridge.

If the External Bridge control is started while Robot Cell Orchestrator is in Online mode, it
runs the normal Cell External Bridge path. Changing Robot Cell Orchestrator mode stops any
running bridge process so the bridge cannot keep using an old Offline Debug or
Online behavior.

Service-triggered Online mode is command-gated:

1. `/robot_cell_orchestrator/start_online` starts or releases the pick side.
2. Robot Cell Orchestrator publishes `moving_to_tray` when the pick side is complete and the
   robot is back at the conveyor side, ready above the conveyor.
3. Robot Cell Orchestrator waits there indefinitely until `/robot_cell_orchestrator/place_online` is
   called.
4. The place service releases the tray/place side and Robot Cell Orchestrator publishes
   `moving_to_bin` when it is ready for the next pick command.

Online phase events are published as JSON strings on:

```text
/robot_cell_orchestrator/events
```

Cell External Bridge uses `moving_to_tray` to publish external `pick_ok` and
`moving_to_bin` to publish external `place_ok`. Online mode has no
`cycle_complete` event and does not continue past `pick_ok` without
`cmd.place`.
