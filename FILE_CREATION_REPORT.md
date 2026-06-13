# File Creation And Teach File Report

Date reviewed: 2026-05-18
Communication guide note added: 2026-05-21

Documentation note: external RabbitMQ/server communication is documented in
`cell_external_bridge/docs/external-rabbitmq-communication-guide.md`. That guide
keeps Robot Cell Orchestrator as the abstraction layer between the high level
server/dashboard and the lower ROS2 nodes.

Scope: project code under `src/`, launch files, package manifests, and checked-in
config defaults. Generated build/install folders, vendor code, and old runtime
dump contents were ignored while tracing ownership.

Path policy after cleanup: code defaults now resolve under the detected workspace
repo root, shown below as `WORKSPACE_ROOT`. The root is found from the current
working directory, the running source/launch file, or colcon environment paths.
`DOBOT_PICKN_PLACE_ROOT` or `DOBOT_WORKSPACE_ROOT` can override it.

## Main Output Areas

| Area | Default path | Primary owners |
| --- | --- | --- |
| Camera calibration | `WORKSPACE_ROOT/calibration` | `camera_calibration` |
| Platform calibration | `WORKSPACE_ROOT/calibration` | `platform_calibration` |
| Bin teach files | `WORKSPACE_ROOT/teach/bin_teach` | `item_perception/bin_teach` |
| Item teach profiles | `WORKSPACE_ROOT/teach/item_teach` | `item_perception/item_teach`, `item_pick` tool teach |
| Item runtime state | `WORKSPACE_ROOT/config/item_perception` | `item_perception/item_detect`, `item_pick`, `robot_cell_orchestrator` |
| YOLO item final bundles | `WORKSPACE_ROOT/teach/item_teach_yolo` | `item_perception_yolo/item_teach_yolo_node.py`, `item_detect_yolo_node.py` |
| YOLO item scratch/saved sessions | `WORKSPACE_ROOT/config/item_perception_yolo` | `item_perception_yolo/item_teach_yolo_node.py` |
| Tray teach profiles | `WORKSPACE_ROOT/teach/tray_teach` | `tray_perception/tray_teach_node`, `tray_detect_node` |
| Tray runtime/config state | `WORKSPACE_ROOT/config/tray_perception` | `tray_perception/tray_teach_node`, `tray_detect_node`, `tray_intercept`, `robot_cell_orchestrator` |
| Online runtime handoff | `WORKSPACE_ROOT/runtime` | `robot_cell_orchestrator` |
| Seek debug dumps | `WORKSPACE_ROOT/debug files/seek_frames` | `item_perception/item_detect`, `tray_perception/tray_detect_node` |
| Robot cell orchestrator movement delta logs | `WORKSPACE_ROOT/debug files/robot_cell_orchestrator_movement_deltas` | `robot_cell_orchestrator/robot_cell_orchestrator_gui` |
| Motion scripts | `WORKSPACE_ROOT/config/motion_calibrate` | `motion_debug`, `movement_calibration` |
| Motion launch logs | `WORKSPACE_ROOT/Log/motion_debug` | `motion_debug` |
| Station config | `WORKSPACE_ROOT/station_config` | `cell_external_bridge`, `dobot_bringup_v4`, `robot_cell_orchestrator` |
| Legacy robot bringup config | `WORKSPACE_ROOT/config/robot_bringup/param.json` | `dobot_bringup_v4` fallback |
| Orbbec camera serial/name map | `WORKSPACE_ROOT/config/camera_bringup/orbbec_cameras.yaml` | `orbbec_camera_launcher` |

## Nodes And Packages That Create Or Mutate Files

### `camera_calibration`

Executable: `eye_on_hand_calibrator`

- Creates the calibration directory on startup if possible.
- Writes camera calibration YAML to `output_path`, defaulting by mode to:
  `WORKSPACE_ROOT/calibration/axab_calibration_eyeonhand_<ddmmyyyy>.yaml` or
  `WORKSPACE_ROOT/calibration/axab_calibration_eyetohand_<ddmmyyyy>.yaml`.
- Camera calibration YAMLs contain only `transform.translation` and
  `transform.rotation`.
- Normalizes empty or wrong-mode `output_path` names back to the active
  mode-specific default.
- Deletes older calibration YAMLs only when the active mode and robot-IP
  filename suffix both match. Legacy no-IP and other-robot files are preserved.
- Trigger: `save_calibration` service, usually reached from the GUI Save YAML flow.

Executable: `platform_calibration`

- Writes platform calibration YAML to
  `WORKSPACE_ROOT/calibration/platform_calibration_<platform_name>_<ddmmyyyy>_<robot_ip>.yaml`.
- Platform calibration YAMLs use top-level `transform.translation` and
  `transform.rotation`, plus `metadata` for platform frame names and teach
  context.
- If `platform_calibration_file` is provided, writes that exact file instead.
- `platform_calibration.launch.py` sets `delete_existing_on_save=true`, so old
  platform calibration YAMLs with the same robot-IP filename suffix are removed
  before saving. Legacy no-IP and other-robot files are preserved.
- Trigger: platform calibration GUI save action.

Executable: `camera_calibration_gui`

- Creates `WORKSPACE_ROOT/calibration` while constructing the mode-specific
  default output path.
- Persists GUI preferences through Qt `QSettings` under the platform-dependent
  `DOBOT/camera_calibration_gui` settings store.
- Actual calibration YAML writing is delegated to `eye_on_hand_calibrator`.

### `item_perception`

Executable: `bin_teach`

- Writes one bin teach YAML per saved bin:
  `WORKSPACE_ROOT/teach/bin_teach/bin_<bin_name>_<ddmmyyyy>.yaml`.
- The output directory can be changed by `bin_teach_dir` or `output_dir`.
- File root key: `bin_teach`.
- Contents include bin name, teach date, ROI points, marker geometry, arm pose at
  save, platform calibration references, and depth plane data used by item
  teach.
- Trigger: `Save bin_teach` button.

Executable: `item_teach`

- Writes item teach runtime state to
  `WORKSPACE_ROOT/config/item_perception/item_teach_runtime.yaml`.
- Writes dated item detect profile YAMLs to
  `WORKSPACE_ROOT/teach/item_teach/item_<item_name>[_bin_<bin_name>]_<ddmmyyyy>.yaml`.
- File root key: `item_detect`.
- Profile content includes thresholds, ROI, depth plane, bin association, pose
  references, item name, teach date, and teach joints.
- Saved item profiles also include footprint metadata from the visible pose
  rectangle projected onto the loaded bin/depth plane:
  `item_length_mm`, `item_width_mm`, `item_dimensions_mm`, and compatibility
  `taught_item_average_*` fields. The dimension source is recorded as
  `rgb_pose_rectangle_on_bin_plane` when the plane came from `bin_teach`.
- Can delete a selected bin teach file from `bin_teach_dir`.
- Triggers: runtime tuning changes, `Save Item`, and Bin Teach delete button.

Executable: `item_detect`

- Writes runtime UI state to
  `WORKSPACE_ROOT/config/item_perception/item_detect_runtime_settings.yaml`.
- Writes the active item profile pointer to
  `WORKSPACE_ROOT/config/item_perception/item_detect_selected_profile.txt`.
- Writes seek debug data to `WORKSPACE_ROOT/debug files/seek_frames`:
  `seek_<stamp>_last.png` and `seek_<stamp>_pose.yaml`.
- Deletes the selected dated item profile YAML when the UI delete flow is
  confirmed.
- Runtime item pose still uses live measured depth, with the saved depth plane
  used for residual/window filtering and optional Z-axis alignment. The saved
  item length/width fields are profile metadata in the current detect path.

### `item_perception_yolo`

The YOLO package consumes bin teach YAMLs from `WORKSPACE_ROOT/teach/bin_teach`
but does not install its own `bin_teach` executable or launch file. Use
`item_perception/bin_teach` to teach or update bin ROI/depth-plane profiles, and
use the Python YOLO teach/detect nodes below for the YOLO/SAM2 workflow.
- `item_perception_yolo/launch/item_detect_yolo.launch.py` launches the Python
  YOLO detect node.

Executable: `item_teach_yolo_node.py`

- Clears and creates one scratch runtime session directory per fresh teach
  launch:
  `WORKSPACE_ROOT/config/item_perception_yolo/item_teach_yolo_runtime/<item>_<timestamp>`.
- Inside each session it creates:
  `dataset/images/train`, `dataset/labels/train`, `masks`, `previews`,
  `prompts`, `models`, `dataset.yaml`, and `session.yaml`.
- Each saved sample writes `sample_*.png`, YOLO label `.txt`, mask `.png`,
  overlay preview `.png`, and prompt `.yaml`.
- Training writes under the scratch session `models/train` directory through
  Ultralytics.
- Promotion creates a final detector-ready teach bundle under
  `WORKSPACE_ROOT/teach/item_teach_yolo/item_<item>[_bin_<bin>]_<ddmmyyyy>`.
- The final bundle contains `best.onnx` and
  `item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml`.
- Saved teach sessions are stored under
  `WORKSPACE_ROOT/config/item_perception_yolo/item_teach_yolo_saved_sessions`.
  The UI can save the active session, load a session by item name, and delete
  saved sessions from the Load Session dropdown.
- Unsaved runtime sessions are removed when the item name changes or when the
  node exits. Saved sessions are kept until explicitly deleted.

Executable: `item_detect_yolo_node.py`

- Writes runtime UI state to
  `WORKSPACE_ROOT/config/item_perception_yolo/item_detect_yolo_runtime_settings.yaml`
  and exports the selected model path to
  `WORKSPACE_ROOT/config/item_perception_yolo/item_detect_yolo_selected_model.txt`.
- Defaults to the same `/bin_camera` topics as YOLO teach and, by default, uses
  the camera topics stored in the selected trained profile.
- Uses the classic non-YOLO item-detect top-bar UI but opens YOLO ONNX/model
  bundles with `Open Model`; sibling YAML metadata supplies ROI, bin plane,
  camera topics, item name, and teach joints when available.
- Deletes selected YOLO teach bundles from
  `WORKSPACE_ROOT/teach/item_teach_yolo` when the selected profile points at a
  safe path inside that teach root.

### `tray_perception`

Executable: `tray_teach_node`

- Writes tray teach runtime state to
  `WORKSPACE_ROOT/config/tray_perception/tray_teach_runtime.yaml`.
- Writes dated tray profile YAMLs to
  `WORKSPACE_ROOT/teach/tray_teach/tray_<tray_name>_<ddmmyyyy>.yaml`.
- Also overwrites the latest settings alias:
  `WORKSPACE_ROOT/config/tray_perception/tray_teach_settings.yaml`.
- File root key: `tray_detect`.
- Profile content includes thresholds, ROI, tray plane coefficients/ROI points,
  tray name, teach date, teach joints, and `tray_width_mm`/`tray_height_mm`
  measured from RGB tray corners projected onto the tray plane.
- Trigger: `Save Tray` in the tray teach UI.

Executable: `tray_detect_node`

- Writes runtime UI state to
  `WORKSPACE_ROOT/config/tray_perception/tray_detect_runtime_settings.yaml`.
- Writes seek debug data to `WORKSPACE_ROOT/debug files/seek_frames`:
  `seek_<stamp>_first.png`, `seek_<stamp>_last.png`, and
  `seek_<stamp>_pose.yaml`.
- Deletes selected dated tray profile YAMLs.
- If the deleted profile matches `tray_teach_settings.yaml`, the node copies the
  newest remaining profile over `tray_teach_settings.yaml`; if no profile
  remains, it removes the alias file.

### `item_pick`

Executable: `item_pick`

- Reads active item profile state from
  `WORKSPACE_ROOT/config/item_perception/item_detect_selected_profile.txt`.
- Writes runtime GUI state to
  `WORKSPACE_ROOT/config/item_perception/item_pick_runtime_settings.json`.
- Writes the active profile's embedded `tool_teach` block in place.
- Legacy `<item_teach_name>_tool.yaml` sidecars remain readable for older
  profiles but are no longer the normal write target.
- Trigger: `Save Tool Teach` and runtime GUI setting changes.
- The saved tool teach data is also used to sync DOBOT Tool 1 on profile
  load/save/arm; that controller service sync does not create project files.

### `orbbec_camera_launcher`

Executable: `camera_launcher_gui`

- Creates `WORKSPACE_ROOT/config/camera_bringup` when saving camera mappings.
- Reads/writes the two-camera serial/name map at
  `WORKSPACE_ROOT/config/camera_bringup/orbbec_cameras.yaml`.
- Preserves and uses the `orbbec_launch_args` block in the same YAML as the
  editable `orbbec_camera gemini_330_series.launch.py` launch argument set.
- Trigger: `Save Mapping` and `Launch Cameras`; launch saves the mapping before
  starting the two Orbbec launch processes.
- Starts `orbbec_camera gemini_330_series.launch.py` subprocesses by
  `serial_number` and `camera_name`, but does not write camera image/depth data.

### `motion_debug`

Executable: `motion_debug_gui`

- Creates one launch diagnostics log per GUI start:
  `WORKSPACE_ROOT/Log/motion_debug/log_<yyyymmdd_hhmmss>.txt`.
- Appends diagnostic events to that log during operation.
- Creates motion script directory `WORKSPACE_ROOT/config/motion_calibrate`.
- Writes saved motion scripts as
  `WORKSPACE_ROOT/config/motion_calibrate/<script_name>.json`.
- Deletes selected motion script JSONs.
- These script JSONs are read by `movement_calibration`.

### `movement_calibration`

Executable: `movement_calibration`

- Writes calibration JSON to `output_file`, defaulting to
  `WORKSPACE_ROOT/calibration/relmovl_speed_calibration_<ddmmyyyy>.json`.
- If `save_raw_trace=true` and raw trace rows exist, writes CSV to
  `<output_file_stem>_tcp_trace.csv`, unless `raw_trace_file` is provided.
- Reads motion scripts from `WORKSPACE_ROOT/config/motion_calibrate`.

Executable: `movement_calibration_gui`

- Does not directly write the calibration JSON/CSV.
- Defaults its script picker to `WORKSPACE_ROOT/config/motion_calibrate`.
- Launches `movement_calibration` and reads/preflights output and motion script
  files.

### `tray_intercept`

Executable: `tray_intercept`

- Writes runtime GUI state to
  `WORKSPACE_ROOT/config/tray_perception/tray_intercept_runtime_settings.json`.
- The runtime settings do not include an EE alignment toggle. The node always
  aligns the final TCP/Link6 yaw to the tray Y axis as a parallel line and then
  applies the operator angle offset.
- Reads movement calibration JSON from
  `WORKSPACE_ROOT/calibration/relmovl_speed_calibration.json` or newest
  `WORKSPACE_ROOT/calibration/relmovl_speed_calibration*.json`.
- It does not create movement calibration files.
- Trigger: runtime GUI setting changes and shutdown save.

### `robot_cell_orchestrator`

Executable: `robot_cell_orchestrator_gui`

- Handles `/robot_cell_orchestrator/load_online_program` by resolving requested
  local bin/item/tray teach YAML basenames, clearing existing YAMLs from
  `WORKSPACE_ROOT/runtime`, and copying the selected YAMLs into that folder.
- If the selected item profile has embedded `tool_teach`, the item YAML is the
  only tool-teach file copied. A legacy tool sidecar is still copied when it is
  needed for an older item profile.
- Writes selected online item/tray profile paths into:
  `WORKSPACE_ROOT/config/item_perception/item_detect_runtime_settings.yaml`,
  `WORKSPACE_ROOT/config/item_perception/item_detect_selected_profile.txt`, and
  `WORKSPACE_ROOT/config/tray_perception/tray_detect_runtime_settings.yaml`.
- Writes online tray placement X/Y/RZ into
  `WORKSPACE_ROOT/config/tray_perception/tray_intercept_runtime_settings.json`.
- Writes Robot Cell Orchestrator GUI/runtime settings to
  `WORKSPACE_ROOT/config/robot_cell_orchestrator/robot_cell_orchestrator_runtime_settings.yaml`.
- Writes one movement delta debug text file per observed cycle:
  `WORKSPACE_ROOT/debug files/robot_cell_orchestrator_movement_deltas/<timestamp>_cycle_<n>_movement_deltas.txt`.
- The file is created on the first completed movement tracker event for that
  cycle and updated as more movement delta events arrive.
- Contents include cycle number, file start/update timestamps, node name, and
  movement tracker TCP delta lines.
- Trigger: passive movement tracker events after `ToolVectorActual` indicates a
  movement has settled. These debug writes do not send robot commands.

## Packages Audited With No Direct File Creation Found

- `aruco_perception`: `aruco_detector_node` and `perception_calibration` read
  calibration files and publish data, but no direct writes were found.
- `dobot_bringup_v4`: launch reads `WORKSPACE_ROOT/station_config` first and
  falls back to `WORKSPACE_ROOT/config/robot_bringup/param.json`; robot node
  does not directly write project files.
  `ros_domain_id` is no longer required in the default workspace config; launch
  and setup hooks only export `ROS_DOMAIN_ID` when that optional legacy field is
  present.
- `dobot_msgs_v4`: message definitions only.
- `dobot_rviz`: launch reads URDF/RViz assets. RViz itself may write user config
  if the operator saves from RViz, but this package does not.
- `gripper_control`: GUI/service interaction only in this repo scan.
- `obstacle_perception`: in-memory obstacle state only.

## Teach File Inventory

| Teach/profile type | Pattern | Writer |
| --- | --- | --- |
| Eye-on-hand camera calibration | `calibration/axab_calibration_eyeonhand_<ddmmyyyy>.yaml` | `camera_calibration/eye_on_hand_calibrator` |
| Eye-to-hand camera calibration | `calibration/axab_calibration_eyetohand_<ddmmyyyy>.yaml` | `camera_calibration/eye_on_hand_calibrator` |
| Platform calibration | `calibration/platform_calibration_<name>_<ddmmyyyy>_<robot_ip>.yaml` | `platform_calibration` |
| Bin teach | `teach/bin_teach/bin_<bin>_<ddmmyyyy>.yaml` | `item_perception/bin_teach` |
| Item teach/profile | `teach/item_teach/item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml` | `item_perception/item_teach` |
| Item YOLO teach bundle | `teach/item_teach_yolo/item_<item>[_bin_<bin>]_<ddmmyyyy>/...` | `item_perception_yolo/item_teach_yolo_node.py` |
| Tray teach/profile | `teach/tray_teach/tray_<tray>_<ddmmyyyy>.yaml` | `tray_perception/tray_teach_node` |
| Tray latest alias | `config/tray_perception/tray_teach_settings.yaml` | `tray_perception/tray_teach_node`, sometimes `tray_detect_node` after delete |
| Item pick tool teach | embedded `tool_teach` block in `teach/item_teach/item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml` | `item_pick` |
| Orbbec camera serial/name map | `config/camera_bringup/orbbec_cameras.yaml` | `orbbec_camera_launcher` |
| Offline dependency manifest | `third_party/manifest.yaml` | `tools/deps/audit_dependencies.py` |
| Offline apt bundle | `third_party/debs/*.deb` | `tools/deps/fetch_offline_deps.sh` |
| Offline Python wheel bundle | `third_party/wheels/*` | `tools/deps/fetch_offline_deps.sh` |
| Motion debug script | `config/motion_calibrate/<script_name>.json` | `motion_debug_gui` |
| Movement calibration | `calibration/relmovl_speed_calibration_<ddmmyyyy>.json` | `movement_calibration` |
| Robot cell orchestrator movement delta debug | `debug files/robot_cell_orchestrator_movement_deltas/<timestamp>_cycle_<n>_movement_deltas.txt` | `robot_cell_orchestrator_gui` |

## Existing Data Migration Note

Some generated metadata under `teach/` and `config/` still stores absolute file
references. Those are profile, runtime, dataset, and model metadata records, not
code defaults. They should be migrated deliberately if the repo is renamed or
moved, because some entries point to trained model bundles and dataset snapshots.

## Organization Notes

- `teach/item_teach` now holds long-lived item profiles with embedded tool teach
  data.
  `config/item_perception` holds runtime state and active profile selection.
- `teach/tray_teach` now holds dated tray profiles. `config/tray_perception` holds runtime
  state and the active/latest tray settings alias.
- Platform calibration moved out of `teach/` and now lives with other
  calibration outputs under
  `calibration/platform_calibration_<platform>_<ddmmyyyy>_<robot_ip>.yaml`.
- `debug files/seek_frames` is shared by item seek and tray seek. Separate
  `debug files/item_seek_frames` and `debug files/tray_seek_frames` would make
  cleanup safer.
- `debug files/robot_cell_orchestrator_movement_deltas` is debug-only and stores one text
  file per robot cycle, updated as movement tracker events arrive.
- `item_perception_yolo` reads classic bin-teach YAMLs from `teach/bin_teach`,
  but its final YOLO item bundles live under
  `teach/item_teach_yolo` and do not write classic item profiles under
  `teach/item_teach`.
- `item_pick` now stores tool teach data inside each dated item profile. Legacy
  `<item>_tool.yaml` sidecars are still readable for old profiles.
- `config/camera_bringup` is runtime/local state and should stay ignored so real camera
  serial numbers are not accidentally committed.
- `third_party/debs` and `third_party/wheels` are the offline dependency depot.
  The directories are part of the repo layout, while large binary payloads are
  intended for release archives or local transfer bundles.
- Calibration save nodes delete older matching YAMLs only for the same robot-IP
  filename suffix. Camera calibration also requires the same eye-on-hand or
  eye-to-hand mode. Legacy no-IP files and files for other robots are preserved.
