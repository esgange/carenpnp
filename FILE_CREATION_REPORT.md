# File Creation And Teach File Report

Date reviewed: 2026-05-10

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
| Platform calibration | `WORKSPACE_ROOT/teach/platform` | `camera_calibration/platform_teach` |
| Bin teach files | `WORKSPACE_ROOT/teach/bin_teach` | `item_perception/bin_teach`, `item_perception_yolo/bin_teach` |
| Item teach profiles | `WORKSPACE_ROOT/teach/items` | `item_perception/item_teach`, C++ `item_perception_yolo/item_teach`, `item_pick` tool teach |
| Item runtime state | `WORKSPACE_ROOT/config/bins` | `item_perception/item_detect`, `item_pick` |
| YOLO item datasets, profiles, models | `WORKSPACE_ROOT/teach/bins_yolo` | `item_perception_yolo/item_teach_yolo_node.py`, `item_detect_yolo_node.py` |
| Tray teach profiles | `WORKSPACE_ROOT/teach/trays` | `tray_perception/tray_teach_node`, `tray_detect_node` |
| Tray runtime/config state | `WORKSPACE_ROOT/config/trays` | `tray_perception/tray_teach_node`, `tray_detect_node`, `tray_intercept` |
| Seek debug dumps | `WORKSPACE_ROOT/debug files/seek_frames` | `item_perception/item_detect`, `tray_perception/tray_detect_node` |
| Pick cycle movement delta logs | `WORKSPACE_ROOT/debug files/pick_cycle_movement_deltas` | `pick_cycle/pick_cycle_gui`, `pick_cycle/pick_cycle_gui_servo` |
| Motion scripts | `WORKSPACE_ROOT/config/motion_debug_scripts` | `motion_debug`, `movement_calibration` |
| Motion launch logs | `WORKSPACE_ROOT/Log/motion_debug` | `motion_debug` |
| Robot bringup config | `WORKSPACE_ROOT/config/dobot_bringup_v4/param.json` | `dobot_bringup_v4`, `dobot_rviz`, `motion_debug` |

## Nodes And Packages That Create Or Mutate Files

### `camera_calibration`

Executable: `eye_on_hand_calibrator`

- Creates the calibration directory on startup if possible.
- Writes camera calibration YAML to `output_path`, defaulting to
  `WORKSPACE_ROOT/calibration/axab_calibration.yaml`.
- Deletes other `axab_calibration.yaml` or `axab_calibration_*.yaml` files in the
  same directory before writing the new output.
- Trigger: `save_calibration` service, usually reached from the GUI Save YAML flow.

Executable: `platform_teach`

- Writes platform teach/calibration YAML to
  `WORKSPACE_ROOT/teach/platform/platform_calibration_<platform_name>.yaml`.
- If `platform_calibration_file` is provided, writes that exact file instead.
- `platform_teach.launch.py` sets `delete_existing_on_save=true`, so old platform
  calibration YAMLs in the output directory are removed before saving.
- Trigger: Platform Teach GUI save action.

Executable: `camera_calibration_gui`

- Creates `WORKSPACE_ROOT/calibration` while constructing the default output path.
- Persists GUI preferences through Qt `QSettings` under the platform-dependent
  `DOBOT/camera_calibration_gui` settings store.
- Actual calibration YAML writing is delegated to `eye_on_hand_calibrator`.

### `item_perception`

Executable: `bin_teach`

- Writes one bin teach YAML per saved bin:
  `WORKSPACE_ROOT/teach/bin_teach/<bin_name>.yaml`.
- The output directory can be changed by `bin_teach_dir` or `output_dir`.
- File root key: `bin_teach`.
- Contents include bin name, ROI points, marker geometry, arm pose at save,
  platform calibration references, and depth plane data used by item teach.
- Trigger: `Save bin_teach` button.

Executable: `item_teach`

- Writes item teach runtime state to
  `WORKSPACE_ROOT/config/bins/item_teach_runtime.yaml`.
- Writes dated item detect profile YAMLs to
  `WORKSPACE_ROOT/teach/items/item_<item_name>[_bin_<bin_name>]_<ddmmyyyy>.yaml`.
- File root key: `item_detect`.
- Profile content includes thresholds, ROI, depth plane, bin association, pose
  references, item name, teach date, and teach joints.
- Can delete a selected bin teach file from `bin_teach_dir`.
- Triggers: runtime tuning changes, `Save Item`, and Bin Teach delete button.

Executable: `item_detect`

- Writes runtime UI state to
  `WORKSPACE_ROOT/config/bins/item_detect_runtime_settings.yaml`.
- Writes the active item profile pointer to
  `WORKSPACE_ROOT/config/bins/item_detect_selected_profile.txt`.
- Writes seek debug data to `WORKSPACE_ROOT/debug files/seek_frames`:
  `seek_<stamp>_last.png` and `seek_<stamp>_pose.yaml`.
- Deletes the selected dated item profile YAML when the UI delete flow is
  confirmed.

### `item_perception_yolo`

Installed C++ executables: `bin_teach`, `item_teach`, `item_detect`

- These are copied/parallel C++ nodes with the same file writing behavior as
  `item_perception`, now with the same workspace-root defaults.
- `item_perception_yolo/launch/bin_teach.launch.py` and
  `item_perception_yolo/launch/item_teach.launch.py` launch the C++ teach nodes.
- `item_perception_yolo/launch/item_detect.launch.py` launches the Python YOLO
  detect node by default, not the installed C++ `item_detect` executable.

Executable: `item_teach_yolo_node.py`

- Creates one runtime session directory per item:
  `WORKSPACE_ROOT/teach/bins_yolo/runtime/<item>_<timestamp>`.
- Inside each session it creates:
  `dataset/images/train`, `dataset/labels/train`, `masks`, `previews`,
  `prompts`, `models`, `dataset.yaml`, and `session.yaml`.
- Each saved sample writes `sample_*.png`, YOLO label `.txt`, mask `.png`,
  overlay preview `.png`, and prompt `.yaml`.
- Training writes under the session `models/train` directory through Ultralytics.
- Promotion creates a model bundle under
  `WORKSPACE_ROOT/teach/bins_yolo/models/item_<item>[_bin_<bin>]_<date>`.
- Writes YOLO item profile YAMLs under
  `WORKSPACE_ROOT/teach/bins_yolo/profiles/item_<item>[_bin_<bin>]_yolo_<date>.yaml`.
- Cleans the old runtime session directory when the item name is changed.

Executable: `item_detect_yolo_node.py`

- Does not create steady-state runtime files in the current code path.
- Deletes selected YOLO profile YAMLs from
  `WORKSPACE_ROOT/teach/bins_yolo/profiles`.
- Deletes associated model artifacts under
  `WORKSPACE_ROOT/teach/bins_yolo/models` when the selected profile points at a
  safe path inside that model root.

### `tray_perception`

Executable: `tray_teach_node`

- Writes tray teach runtime state to
  `WORKSPACE_ROOT/config/trays/tray_teach_runtime.yaml`.
- Writes dated tray profile YAMLs to
  `WORKSPACE_ROOT/teach/trays/tray_<tray_name>_<ddmmyyyy>.yaml`.
- Also overwrites the latest/legacy settings file:
  `WORKSPACE_ROOT/config/trays/tray_teach_settings.yaml`.
- File root key: `tray_detect`.
- Profile content includes thresholds, ROI, depth plane, tray name, teach date,
  teach joints, edge lengths, and area.
- Trigger: `Save Tray` in the tray teach UI.

Executable: `tray_detect_node`

- Writes runtime UI state to
  `WORKSPACE_ROOT/config/trays/tray_detect_runtime_settings.yaml`.
- Writes seek debug data to `WORKSPACE_ROOT/debug files/seek_frames`:
  `seek_<stamp>_first.png`, `seek_<stamp>_last.png`, and
  `seek_<stamp>_pose.yaml`.
- Deletes selected dated tray profile YAMLs.
- If the deleted profile matches `tray_teach_settings.yaml`, the node copies the
  newest remaining profile over `tray_teach_settings.yaml`; if no profile
  remains, it removes the legacy file.

### `item_pick`

Executable: `item_pick`

- Reads active item profile state from
  `WORKSPACE_ROOT/config/bins/item_detect_selected_profile.txt`.
- Writes runtime GUI state to
  `WORKSPACE_ROOT/config/bins/item_pick_runtime_settings.json`.
- Writes one tool teach sidecar next to the active item profile:
  `<active_profile_dir>/<item_teach_name>_tool.yaml`.
- Example default destination:
  `WORKSPACE_ROOT/teach/items/<item_name>_tool.yaml`.
- Trigger: `Save Tool Teach` and runtime GUI setting changes.

### `motion_debug`

Executable: `motion_debug_gui`

- Creates one launch diagnostics log per GUI start:
  `WORKSPACE_ROOT/Log/motion_debug/log_<yyyymmdd_hhmmss>.txt`.
- Appends diagnostic events to that log during operation.
- Creates motion script directory `WORKSPACE_ROOT/config/motion_debug_scripts`.
- Writes saved motion scripts as
  `WORKSPACE_ROOT/config/motion_debug_scripts/<script_name>.json`.
- Deletes selected motion script JSONs.
- These script JSONs are read by `movement_calibration`.

### `movement_calibration`

Executable: `movement_calibration`

- Writes calibration JSON to `output_file`, defaulting to
  `WORKSPACE_ROOT/calibration/relmovl_speed_calibration_<ddmmyyyy>.json`.
- If `save_raw_trace=true` and raw trace rows exist, writes CSV to
  `<output_file_stem>_tcp_trace.csv`, unless `raw_trace_file` is provided.
- Reads motion scripts from `WORKSPACE_ROOT/config/motion_debug_scripts`.

Executable: `movement_calibration_gui`

- Does not directly write the calibration JSON/CSV.
- Defaults its script picker to `WORKSPACE_ROOT/config/motion_debug_scripts`.
- Launches `movement_calibration` and reads/preflights output and motion script
  files.

### `tray_intercept`

Executable: `tray_intercept`

- Writes runtime GUI state to
  `WORKSPACE_ROOT/config/trays/tray_intercept_runtime_settings.json`.
- Reads movement calibration JSON from
  `WORKSPACE_ROOT/calibration/relmovl_speed_calibration.json` or newest
  `WORKSPACE_ROOT/calibration/relmovl_speed_calibration*.json`.
- It does not create movement calibration files.
- Trigger: runtime GUI setting changes and shutdown save.

### `pick_cycle`

Executables: `pick_cycle_gui`, `pick_cycle_gui_servo`

- Writes one movement delta debug text file per observed cycle:
  `WORKSPACE_ROOT/debug files/pick_cycle_movement_deltas/<timestamp>_cycle_<n>_movement_deltas.txt`.
- The file is created on the first completed movement tracker event for that
  cycle and updated as more movement delta events arrive.
- Contents include cycle number, file start/update timestamps, node name, and
  movement tracker TCP delta lines.
- Trigger: passive movement tracker events after `ToolVectorActual` indicates a
  movement has settled. These debug writes do not send robot commands.

## Packages Audited With No Direct File Creation Found

- `aruco_perception`: `aruco_detector_node` and `perception_calibration` read
  calibration files and publish data, but no direct writes were found.
- `dobot_bringup_v4`: launch reads
  `WORKSPACE_ROOT/config/dobot_bringup_v4/param.json`; robot node does not
  directly write project files.
- `dobot_msgs_v4`: message definitions only.
- `dobot_rviz`: launch reads URDF/RViz assets. RViz itself may write user config
  if the operator saves from RViz, but this package does not.
- `gripper_control`: GUI/service interaction only in this repo scan.
- `obstacle_perception`: in-memory obstacle state only.

## Teach File Inventory

| Teach/profile type | Pattern | Writer |
| --- | --- | --- |
| Camera calibration | `calibration/axab_calibration.yaml` | `camera_calibration/eye_on_hand_calibrator` |
| Platform calibration | `teach/platform/platform_calibration_<name>.yaml` | `camera_calibration/platform_teach` |
| Bin teach | `teach/bin_teach/<bin_name>.yaml` | `item_perception/bin_teach`, `item_perception_yolo/bin_teach` |
| Item teach/profile | `teach/items/item_<item>[_bin_<bin>]_<ddmmyyyy>.yaml` | `item_perception/item_teach`, C++ YOLO copy if used |
| Item YOLO profile | `teach/bins_yolo/profiles/item_<item>[_bin_<bin>]_yolo_<date>.yaml` | `item_perception_yolo/item_teach_yolo_node.py` |
| YOLO model bundle | `teach/bins_yolo/models/item_<item>[_bin_<bin>]_<date>/...` | `item_perception_yolo/item_teach_yolo_node.py` |
| Tray teach/profile | `teach/trays/tray_<tray>_<ddmmyyyy>.yaml` | `tray_perception/tray_teach_node` |
| Tray latest alias | `config/trays/tray_teach_settings.yaml` | `tray_perception/tray_teach_node`, sometimes `tray_detect_node` after delete |
| Item pick tool teach | `teach/items/<item_teach_name>_tool.yaml` | `item_pick` |
| Motion debug script | `config/motion_debug_scripts/<script_name>.json` | `motion_debug_gui` |
| Movement calibration | `calibration/relmovl_speed_calibration_<ddmmyyyy>.json` | `movement_calibration` |
| Pick cycle movement delta debug | `debug files/pick_cycle_movement_deltas/<timestamp>_cycle_<n>_movement_deltas.txt` | `pick_cycle_gui`, `pick_cycle_gui_servo` |

## Existing Data Migration Note

Some generated metadata under `teach/` and `config/` still stores absolute file
references. Those are profile, runtime, dataset, and model metadata records, not
code defaults. They should be migrated deliberately if the repo is renamed or
moved, because some entries point to trained model bundles and dataset snapshots.

## Organization Notes

- `teach/items` now holds long-lived item profiles and tool teach sidecars.
  `config/bins` holds runtime state and active profile selection.
- `teach/trays` now holds dated tray profiles. `config/trays` holds runtime
  state and the active/latest tray settings alias.
- `debug files/seek_frames` is shared by item seek and tray seek. Separate
  `debug files/item_seek_frames` and `debug files/tray_seek_frames` would make
  cleanup safer.
- `debug files/pick_cycle_movement_deltas` is debug-only and stores one text
  file per pick cycle, updated as movement tracker events arrive.
- `item_perception_yolo` installs copied C++ teach/detect nodes that write to the
  same default paths as `item_perception`. Running both packages can overwrite or
  delete the same profiles unless launch arguments isolate their directories.
- `item_pick` tool teach sidecars are named from item teach name, not the full
  dated profile filename. Multiple profiles with the same item name can therefore
  share one `<item>_tool.yaml`.
- Calibration teach nodes intentionally enforce a single active file by deleting
  older matching calibration YAMLs in the same directory. That is useful for
  active-state clarity, but it matters for backups.
