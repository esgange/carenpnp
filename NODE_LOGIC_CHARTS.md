# DOBOT Pick-and-Place Node Logic Charts

This document maps the ROS 2 nodes in this workspace, what each node does, and
how the nodes communicate.

## Diagram Legend

- Solid arrow: topic, TF, or hardware data stream
- Dotted arrow: ROS service, file/configuration handoff, or process control;
  the edge label identifies which one
- Names shown are the default names. Most can be overridden by launch arguments
  or runtime settings.

## 1. Complete Runtime Communication Map

```mermaid
flowchart LR
    Operator["Operator / Cell External Bridge"]
    Orchestrator["robot_cell_orchestrator_gui"]

    CameraLauncher["orbbec_camera_launcher"]
    BinCamera["Orbbec bin_camera driver"]
    RobotCamera["Orbbec robot_camera driver"]

    Bringup["cr_robot_ros2"]
    Controller["DOBOT controller"]
    RSP["robot_state_publisher"]
    RViz["rviz2"]

    ItemDetect["item_detect<br/>classic or YOLO"]
    ItemPick["item_pick"]
    TrayDetect["tray_detect"]
    TrayIntercept["tray_intercept"]

    Aruco["aruco_perception"]
    ObstacleLive["obstacle_perception"]
    ObstacleMemory["obstacle_memory"]

    Operator -. "online ROS API / GUI actions" .-> Orchestrator
    Orchestrator -. "launch/stop processes" .-> CameraLauncher
    Orchestrator -. "launch/stop processes" .-> Bringup
    Orchestrator -. "launch/stop processes" .-> ItemDetect
    Orchestrator -. "launch/stop processes" .-> ItemPick
    Orchestrator -. "launch/stop processes" .-> TrayDetect
    Orchestrator -. "launch/stop processes" .-> TrayIntercept

    CameraLauncher -. "starts configured serials" .-> BinCamera
    CameraLauncher -. "starts configured serials" .-> RobotCamera

    BinCamera -->|"/bin_camera color + depth + info"| ItemDetect
    RobotCamera -->|"/robot_camera color + depth + info"| TrayDetect
    RobotCamera -->|"RGB-D"| Aruco
    RobotCamera -->|"RGB-D"| ObstacleLive

    Controller <-->|"TCP dashboard / motion / feedback"| Bringup
    Bringup -->|"/joint_states_robot"| RSP
    RSP -->|"/tf"| RViz
    Bringup -->|"DIStatus"| ItemPick
    Orchestrator -. "GetAngle" .-> Bringup

    ItemDetect -->|"bin_seek_pose + selected_profile"| ItemPick
    ItemDetect -->|"bin_overlay"| Orchestrator
    ItemPick -. "motion, pose, mode, tool, and DO services" .-> Bringup
    ItemPick -. "seek_complete / repick" .-> ItemDetect

    TrayDetect -->|"tray_vector + tray_axis_overlay"| TrayIntercept
    TrayDetect -->|"tray_overlay"| Orchestrator
    TrayIntercept -. "motion, pose, speed, and DO services" .-> Bringup
    TrayIntercept -. "seek_complete" .-> TrayDetect

    Orchestrator -. "seek, status, repick, go_to_teach" .-> ItemDetect
    Orchestrator -. "track, track_status, auto_repick" .-> ItemPick
    Orchestrator -. "seek, status, go_to_teach" .-> TrayDetect
    Orchestrator -. "start_sequence, track_status" .-> TrayIntercept

    Aruco -->|"marker poses + marker TFs"| RViz
    ObstacleLive -->|"/obstacles/points"| ObstacleMemory
    ObstacleLive -->|"/obstacles/markers"| RViz
    ObstacleMemory -->|"/obstacles/memory_points"| RViz
```

## 2. Robot Bringup and Visualization

### `cr_robot_ros2`

```mermaid
flowchart TD
    Start["Start cr_robot_ros2"]
    Config["Read robot IP, type, and publish rate"]
    Connect["Connect to DOBOT ports<br/>dashboard, motion, feedback, DI"]
    Services["Expose /dobot_bringup_ros2/srv/*"]
    Poll["Poll controller state"]
    Publish["Publish joint, TCP, robot status,<br/>feedback JSON, and DI JSON"]
    Client["Motion / calibration / GUI client"]
    Robot["DOBOT controller"]

    Start --> Config --> Connect
    Connect <--> Robot
    Connect --> Services
    Connect --> Poll --> Publish --> Poll
    Client -. "service call" .-> Services
    Services -. "controller command" .-> Robot
```

Published state:

| Topic | Main consumers |
| --- | --- |
| `/joint_states_robot` | `robot_state_publisher`, `motion_debug`, YOLO teach |
| `dobot_msgs_v4/msg/RobotStatus` | `motion_debug` |
| `dobot_msgs_v4/msg/ToolVectorActual` | calibration, bin teach, movement calibration, motion debug |
| `/dobot_bringup_ros2/msg/FeedInfo` | diagnostics |
| `/dobot_bringup_ros2/DIStatus_200mS` | `item_pick`, `gripper_control` |

The node exposes the controller command set under
`/dobot_bringup_ros2/srv`, including `MovJ`, `MovL`, `MovLIO`, `Stop`,
`GetPose`, `GetAngle`, `RobotMode`, `SpeedFactor`, `CP`, `DO`, and the
remaining services defined in `dobot_msgs_v4`.

### `robot_state_publisher`, `world_to_base`, and `rviz2`

```mermaid
flowchart LR
    Bringup["cr_robot_ros2"]
    RSP["robot_state_publisher"]
    Static["static_transform_publisher"]
    TF["/tf and /tf_static"]
    RViz["rviz2"]

    Bringup -->|"/joint_states_robot"| RSP
    RSP -->|"URDF link transforms"| TF
    Static -->|"world -> base_link"| TF
    TF --> RViz
```

## 3. Camera Launch and Camera Data

`orbbec_camera_launcher` is a launcher node/process, not an image-processing
node. It scans USB devices, matches serial numbers from
`config/camera_bringup/orbbec_cameras.yaml`, starts the Orbbec launch file, and
waits for fresh color and depth messages before starting the next configured
camera. The watchdog continues checking stream freshness and relaunches a
camera driver when its process exits or either image stream becomes stale.

```mermaid
flowchart TD
    Start["camera_launcher_gui or camera_headless.launch.py"]
    Load["Load saved camera names, serials, and driver arguments"]
    Scan["Run Orbbec device scan"]
    Match{"Configured camera connected?"}
    Watchdog["Start camera watchdog"]
    Launch["Watchdog launches Orbbec driver"]
    Wait{"Fresh color and depth messages?"}
    Ready["Mark camera ready"]
    Monitor{"Streams remain fresh?"}
    Backoff["Stop driver and wait with backoff"]
    Next["Start next configured camera"]
    Skip["Log missing camera and continue"]

    Start --> Load --> Scan --> Match
    Match -->|Yes| Watchdog --> Launch --> Wait
    Wait -->|Yes| Ready --> Monitor
    Monitor -->|Yes| Monitor
    Monitor -->|No| Backoff --> Launch
    Ready --> Next
    Wait -->|No / timeout| Backoff
    Match -->|No| Skip
```

Default camera outputs:

| Camera | Topics |
| --- | --- |
| `bin_camera` | `/bin_camera/color/image_raw`, `/bin_camera/depth/image_raw`, `/bin_camera/color/camera_info` |
| `robot_camera` | `/robot_camera/color/image_raw`, `/robot_camera/depth/image_raw`, `/robot_camera/color/camera_info` |

Watchdog interfaces:

| Type | Default name |
| --- | --- |
| Publisher | `/camera_watchdog/status` (`diagnostic_msgs/msg/DiagnosticArray`) |
| Publisher | `/camera_watchdog/healthy` (`std_msgs/msg/Bool`) |
| Service | `/camera_watchdog/restart_all` (`std_srvs/srv/Trigger`) |

## 4. ArUco and Camera Calibration

### `aruco_perception`

```mermaid
flowchart TD
    RGBD["Color + depth + CameraInfo"]
    Detect["Detect configured ArUco IDs"]
    Depth["Sample marker depth and estimate 3D poses"]
    Calib{"Calibration enabled?"}
    Transform["Transform output into calibrated camera frame"]
    Raw["Keep output in raw/parent frame"]
    Outputs["Publish marker_pose,<br/>/aruco_detections, /aruco_overlay,<br/>and optional marker TFs"]

    RGBD --> Detect --> Depth --> Calib
    Calib -->|Yes| Transform --> Outputs
    Calib -->|No| Raw --> Outputs
```

`aruco_perception/perception_calibration` is a small static-TF helper. It loads
an eye-on-hand calibration YAML and publishes
`Link6 -> arm_calibrated_camera_link`.

### Calibration Workflow Nodes

```mermaid
flowchart LR
    Camera["Camera RGB-D"]
    Aruco["aruco_perception"]
    Detections["/aruco_detections"]
    Board["calibration_perception"]
    TagTF["camera_frame -> tag_frame TF"]
    GUI["camera_calibration_gui"]
    Solver["eye_on_hand_calibrator"]
    Bringup["cr_robot_ros2 services"]
    YAML["axab_calibration_*.yaml"]

    Camera --> Aruco --> Detections --> Board --> TagTF
    TagTF --> GUI
    GUI -. "starts process" .-> Solver
    GUI -. "MovJ / Stop / InverseKin" .-> Bringup
    GUI -. "add / preview / compute / save / reset" .-> Solver
    Solver -->|"TF lookups: robot, camera, tag"| Solver
    Solver -. "save calibration" .-> YAML
```

Node responsibilities:

| Node | Responsibility |
| --- | --- |
| `camera_calibration_gui` | Operator UI, motion generation, IK checks, overlay display, and solver service client |
| `calibration_perception` | Fits one board pose from `/aruco_detections` and broadcasts `camera_frame -> tag_frame` |
| `eye_on_hand_calibrator` | Collects robot/tag TF samples, solves AX=XB, previews the result, and writes calibration YAML |
| `aruco_perception/perception_calibration` | Loads a saved YAML and publishes the calibrated camera static TF |

### `platform_calibration`

```mermaid
flowchart TD
    EyeToHand["Eye-to-hand YAML"]
    StaticTF["base_link -> bin_calibrated_camera_link"]
    Aruco["aruco_perception<br/>bin camera"]
    Markers["ArUco marker TFs + overlay"]
    Platform["platform_calibration"]
    Stable{"Board pose stable?"}
    Save["Save base_link -> platform reference YAML"]
    BinTeach["bin_teach"]

    EyeToHand -.-> StaticTF
    StaticTF --> Aruco
    Aruco --> Markers --> Platform --> Stable
    Stable -->|Yes, operator saves| Save
    Save -. "platform_calibration_*.yaml" .-> BinTeach
```

## 5. Teaching Nodes

### `bin_teach` (classic and YOLO packages)

```mermaid
flowchart TD
    Aruco["/aruco_detections and optional /aruco_overlay"]
    Color["Bin-camera color image"]
    Robot["ToolVectorActual / GetAngle"]
    Platform["Platform calibration YAML"]
    Teach["bin_teach"]
    Align["Optional MovJ / RelMovLUser alignment"]
    Save["Save bin frame, ROI, plane,<br/>camera topics, and robot joints"]

    Aruco --> Teach
    Color --> Teach
    Robot --> Teach
    Platform -.-> Teach
    Teach -. "Stop / SpeedFactor / motion" .-> Align
    Align --> Teach --> Save
```

### `item_teach`

```mermaid
flowchart TD
    Camera["Bin-camera RGB-D + CameraInfo"]
    BinProfile["Selected bin-teach YAML"]
    Robot["GetAngle / MovJ"]
    UI["item_teach OpenCV UI"]
    Tune["Teach ROI, thresholds, depth plane,<br/>mask, item axes, and tool association"]
    Preview["Publish bin_overlay and optional bin_item_poses"]
    Save["Save item_detect YAML profile"]

    Camera --> UI
    BinProfile -.-> UI
    Robot -.-> UI
    UI --> Tune --> Preview --> Save
```

### `item_teach_yolo`

```mermaid
flowchart TD
    Camera["Bin-camera color + joint states"]
    BinProfile["Bin-teach YAML"]
    Prompt["SAM2 point/box prompts"]
    Samples["Generate YOLO segmentation samples"]
    Train["Train YOLO segmentation model"]
    Export["Save best.pt, best.onnx, and item YAML"]

    Camera --> Prompt
    BinProfile -.-> Prompt
    Prompt --> Samples --> Train --> Export
```

### `tray_teach`

```mermaid
flowchart TD
    Camera["Robot-camera RGB-D + CameraInfo"]
    Robot["GetAngle"]
    UI["tray_teach OpenCV UI"]
    Tune["Teach ROI, RGB/edge settings,<br/>tray plane, dimensions, and origin"]
    Preview["Publish tray_overlay"]
    Save["Save tray_detect YAML profile"]

    Camera --> UI
    Robot -.-> UI
    UI --> Tune --> Preview --> Save
```

## 6. Runtime Perception Nodes

### `item_detect` (classic or YOLO)

Both implementations keep the external node name and handoff protocol
compatible with `item_pick` and the orchestrator.

```mermaid
stateDiagram-v2
    [*] --> LoadProfile
    LoadProfile --> Idle: profile and calibration valid
    Idle --> Acquiring: item_detect/seek
    Acquiring --> Acquiring: invalid frame or confidence not reached
    Acquiring --> Latched: valid item pose
    Latched --> Acquiring: item_detect/repick
    Latched --> Idle: item_detect/seek_complete
    Acquiring --> Idle: seek toggled off

    state Acquiring {
        [*] --> ReadRGBD
        ReadRGBD --> Segment
        Segment --> EstimatePose
        EstimatePose --> FilterEvidence
        FilterEvidence --> ReadRGBD
    }
```

Interfaces:

| Direction | Default interface |
| --- | --- |
| Subscribe | bin-camera color, depth, and camera info |
| Publish | `bin_overlay`, `bin_seek_pose`, `bin_item_poses`, `bin_cube_marker` |
| Publish, latched | `item_detect/selected_profile` |
| Provide services | `item_detect/seek`, `repick`, `seek_complete`, `seek_status`, `go_to_teach` |
| Call services | `/dobot_bringup_ros2/srv/MovJ`, camera exposure services |

### `tray_detect`

```mermaid
stateDiagram-v2
    [*] --> LoadProfile
    LoadProfile --> Idle: profile and calibration valid
    Idle --> Acquiring: tray_detect/seek
    Acquiring --> Acquiring: confidence below threshold
    Acquiring --> Latched: stable tray pose/vector published
    Latched --> Idle: tray_detect/seek_complete
    Acquiring --> Idle: seek toggled off

    state Acquiring {
        [*] --> ReadRGBD
        ReadRGBD --> DetectEdges
        DetectEdges --> ProjectToSavedPlane
        ProjectToSavedPlane --> FilterPoseAndVelocity
        FilterPoseAndVelocity --> ConfidenceGate
        ConfidenceGate --> ReadRGBD
    }
```

Interfaces:

| Direction | Default interface |
| --- | --- |
| Subscribe | robot-camera color, depth, and camera info |
| Publish | `tray_overlay`, `tray_pose`, `tray_axis_overlay`, `tray_vector`, `tray_cube_marker` |
| Provide services | `tray_detect/get_tray_dimensions`, `seek`, `seek_complete`, `seek_status`, `go_to_teach` |
| Call services | `/dobot_bringup_ros2/srv/MovJ`, camera exposure services |

## 7. Item Pick Logic

```mermaid
flowchart TD
    Arm["GUI, orchestrator, or start_sequence arms item_pick"]
    Wait["Wait for fresh bin_seek_pose"]
    Save["Save current joints for possible repick"]
    Goals["Apply item profile, tool offset,<br/>approach, and final Z-up"]
    Preview{"TF-only mode?"}
    Approach["MovJ to approach pose"]
    IOOpen["DO: open gripper, suction/exhaust off"]
    Descend["MovLIO descend<br/>turn suction on during descent"]
    Retract["MovL retract to approach"]
    Idle["Wait for RobotMode idle and stable"]
    DI{"Fresh DI1 suction active?"}
    Grip["DO: close gripper"]
    Up["MovL final Z-up"]
    Complete["Call item_detect/seek_complete"]
    Auto{"Auto Repick enabled?"}
    Purge["Release/purge, MovJ to saved joints"]
    Repick["Re-arm and call item_detect/repick"]
    Standby["Return to standby; Seek remains ON"]

    Arm --> Wait --> Save --> Goals --> Preview
    Preview -->|Yes| Standby
    Preview -->|No| Approach --> IOOpen --> Descend --> Retract --> Idle --> DI
    DI -->|Yes| Grip --> Up --> Complete
    DI -->|No| Auto
    Auto -->|Yes| Purge --> Repick --> Wait
    Auto -->|No| Standby
```

`item_pick` subscribes to `bin_seek_pose`,
`item_detect/selected_profile`, and
`/dobot_bringup_ros2/DIStatus_200mS`. It provides `item_pick/track`,
`item_pick/track_status`, `item_pick/start_sequence`, and
`item_pick/set_auto_repick`. Its DOBOT clients are `MovJ`, `MovL`, `MovLIO`,
`GetAngle`, `GetPose`, `RobotMode`, `SetTool`, `Tool`, `Stop`, and `DO`.

## 8. Tray Intercept Logic

```mermaid
flowchart TD
    Arm["GUI, orchestrator, or start_sequence arms tray_intercept"]
    Wait["Wait for fresh tray_vector"]
    Stop["Call Stop"]
    Predict["Predict intercept from tray pose,<br/>velocity, offsets, and lead time"]
    Preview{"TF-only mode?"}
    Intercept["Queue MovL to intercept pose"]
    Follow{"Release IO enabled?"}
    MovL["Queue MovL along tray direction"]
    MovLIO["Queue MovLIO follow + release outputs"]
    ZUp["Queue post-follow Z-up<br/>while continuing tray direction"]
    Complete["Call tray_detect/seek_complete"]
    PreviewDone["Publish goal TFs only<br/>Seek remains latched"]

    Arm --> Wait --> Stop --> Predict --> Preview
    Preview -->|Yes| PreviewDone
    Preview -->|No| Intercept --> Follow
    Follow -->|No| MovL --> ZUp --> Complete
    Follow -->|Yes| MovLIO --> ZUp --> Complete
```

`tray_intercept` subscribes to `tray_vector` and `tray_axis_overlay`, calls
`tray_detect/get_tray_dimensions`, and provides `tray_intercept/track`,
`tray_intercept/track_status`, and `tray_intercept/start_sequence`. Its DOBOT
clients are `MovL`, `MovLIO`, `Stop`, `CP`, `DO`, `GetPose`, and
`SpeedFactor`.

## 9. Robot Cell Orchestrator

### Offline Cycle

```mermaid
flowchart TD
    Start["Start Cycle"]
    Ready{"Runtime files, calibration,<br/>and required services ready?"}
    Auto["Set item auto-repick"]
    ItemTeach["item_detect/go_to_teach"]
    ItemArm["item_pick/track"]
    ItemSeek["item_detect/seek"]
    ItemDone["Wait for item seek status to turn OFF<br/>after successful pick"]
    Step{"Offline step mode?"}
    TrayTeach["tray_detect/go_to_teach"]
    TrayArm["tray_intercept/start_sequence"]
    TraySeek["tray_detect/seek"]
    TrayDone["Wait for tray seek status to turn OFF"]
    Loop{"Loop enabled?"}
    Stop["Stop / report failure"]

    Start --> Ready
    Ready -->|No| Stop
    Ready -->|Yes| Auto --> ItemTeach --> ItemArm --> ItemSeek --> ItemDone --> Step
    Step -->|Wait for operator| Step
    Step -->|Continue / disabled| TrayTeach --> TrayArm --> TraySeek --> TrayDone --> Loop
    Loop -->|Yes| Ready
    Loop -->|No| Stop
```

### Online Cycle

```mermaid
sequenceDiagram
    participant B as Cell External Bridge
    participant O as robot_cell_orchestrator_gui
    participant I as item_detect + item_pick
    participant T as tray_detect + tray_intercept

    B->>O: load_online_program
    O-->>B: runtime files copied/validated
    B->>O: validate_online_program
    B->>O: start_online (cmd.pick)
    O->>I: go_to_teach, arm, seek
    I-->>O: pick completes and seek turns OFF
    O-->>B: event moving_to_tray
    Note over O: Wait indefinitely for cmd.place
    B->>O: place_online
    O->>T: go_to_teach, arm, seek
    T-->>O: placement completes and seek turns OFF
    O-->>B: event moving_to_bin
```

Orchestrator ROS interfaces:

| Type | Default name |
| --- | --- |
| Service | `robot_cell_orchestrator/load_online_program` |
| Service | `robot_cell_orchestrator/validate_online_program` |
| Service | `robot_cell_orchestrator/start_online` |
| Service | `robot_cell_orchestrator/place_online` |
| Publisher | `robot_cell_orchestrator/events` |
| Subscribers | `bin_overlay`, `tray_overlay` |
| Robot client | `/dobot_bringup_ros2/srv/GetAngle` |

`robot_runtime_headless.launch.py` is a launch composition, not a ROS node. It
can start the configured camera drivers, `item_detect`, `item_pick`,
`tray_detect`, `tray_intercept`, and RViz using the shared orchestrator runtime
settings.

## 10. External RabbitMQ Bridge

```mermaid
sequenceDiagram
    participant M as Conveyor / master RabbitMQ
    participant C as cell_external_bridge controller
    participant R as ROS bridge client node
    participant O as robot_cell_orchestrator_gui

    M->>C: cmd.load_program
    C->>R: parsed local teach filenames + tray placement
    R->>O: load_online_program service
    O-->>R: accepted runtime file set
    C-->>M: load_program_ok or load_program_failed

    M->>C: cmd.pick
    C->>R: run_pick
    R->>O: start_online service
    O-->>R: moving_to_tray event
    C-->>M: pick_ok or pick_failed

    M->>C: cmd.place
    C->>R: run_place
    R->>O: place_online service
    O-->>R: moving_to_bin event
    C-->>M: place_ok or place_failed
```

The ROS client node created inside the bridge is named
`cell_external_bridge_robot_cell_orchestrator_client`. RabbitMQ communication
does not talk directly to motion or perception nodes; all physical-cycle
commands pass through the orchestrator API.

## 11. Obstacle Perception

```mermaid
flowchart LR
    Camera["Robot-camera synchronized<br/>color + depth + CameraInfo"]
    Live["obstacle_perception"]
    Cloud["/obstacles/points"]
    Markers["/obstacles/markers"]
    Memory["obstacle_memory"]
    TF["TF to base_link"]
    Persistent["/obstacles/memory_points"]
    RViz["rviz2"]

    Camera --> Live
    Live --> Cloud --> Memory
    Live --> Markers --> RViz
    TF --> Memory
    Memory --> Persistent --> RViz
```

`obstacle_perception` projects depth pixels into 3D, voxelizes them, removes
small floating groups, and publishes live data. `obstacle_memory` transforms
the live cloud into `base_link`, accumulates stable voxels, and can suppress
points still visible in the current camera frustum.

## 12. Diagnostic and Utility Nodes

| Node | Subscribes / reads | Calls / produces |
| --- | --- | --- |
| `gripper_control_gui` | `/dobot_bringup_ros2/DIStatus_200mS` | `/dobot_bringup_ros2/srv/DO` |
| `motion_debug_gui` | joint state, TCP state, robot status | enable/disable, jog, drag, tool, payload, speed, acceleration, `MovJ`, `MovL`, and stop services |
| `movement_calibration` | `ToolVectorActual` | `CP`, `SpeedFactor`, and `MovL`; writes speed calibration JSON |
| `movement_calibration_gui` | Operator settings | Launches `movement_calibration` |

```mermaid
flowchart LR
    Bringup["cr_robot_ros2"]
    Gripper["gripper_control_gui"]
    Debug["motion_debug_gui"]
    MoveCal["movement_calibration"]
    MoveGUI["movement_calibration_gui"]
    File["relmovl speed calibration JSON"]

    Bringup -->|"DIStatus"| Gripper
    Gripper -. "DO" .-> Bringup
    Bringup -->|"joint / TCP / status"| Debug
    Debug -. "diagnostic motion services" .-> Bringup
    MoveGUI -. "launch with parameters" .-> MoveCal
    Bringup -->|"ToolVectorActual"| MoveCal
    MoveCal -. "CP / SpeedFactor / MovL" .-> Bringup
    MoveCal -.-> File
```

## 13. Node and Executable Inventory

| Package | Node / process | Executable |
| --- | --- | --- |
| `cr_robot_ros2` | DOBOT controller bridge | `cr_robot_ros2_node` |
| `dobot_rviz` | Robot model TF publisher | `robot_state_publisher` |
| `dobot_rviz` | World anchor | `static_transform_publisher` |
| `dobot_rviz` | Visualizer | `rviz2` |
| `orbbec_camera_launcher` | Camera launcher GUI | `camera_launcher_gui` |
| `aruco_perception` | ArUco RGB-D detector | `aruco_detector_node` |
| `aruco_perception` | Saved calibration TF helper | `perception_calibration` |
| `camera_calibration` | Calibration GUI | `camera_calibration_gui` |
| `camera_calibration` | Four-marker board fitter | `calibration_perception` |
| `camera_calibration` | AX=XB solver | `eye_on_hand_calibrator` |
| `platform_calibration` | Platform calibration GUI/node | `platform_calibration` |
| `obstacle_perception` | Live obstacle projector | `obstacle_perception_node` |
| `obstacle_perception` | Persistent obstacle map | `obstacle_memory_node` |
| `item_perception` | Bin teaching | `bin_teach` |
| `item_perception` | Classic item teaching | `item_teach` |
| `item_perception` | Classic item detection | `item_detect` |
| `item_perception_yolo` | Bin teaching | `bin_teach` |
| `item_perception_yolo` | SAM2/YOLO teaching | `item_teach_yolo_node.py` |
| `item_perception_yolo` | YOLO runtime detection | `item_detect_yolo_node.py` |
| `tray_perception` | Tray teaching | `tray_teach_node` |
| `tray_perception` | Tray runtime detection | `tray_detect_node` |
| `item_pick` | Item motion and GUI/service endpoint | `item_pick` |
| `tray_intercept` | Tray motion and GUI/service endpoint | `tray_intercept` |
| `robot_cell_orchestrator` | Main cell coordinator | `robot_cell_orchestrator_gui` |
| `gripper_control` | Manual gripper GUI | `gripper_control_gui` |
| `motion_debug` | Robot commissioning GUI | `motion_debug_gui` |
| `movement_calibration` | Speed calibration worker | `movement_calibration` |
| `movement_calibration` | Calibration launcher GUI | `movement_calibration_gui` |
| `cell_external_bridge` | RabbitMQ controller | `cell-external-bridge` |

`dobot_msgs_v4` contains message and service definitions; it does not start a
node.
