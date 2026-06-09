# External RabbitMQ Communication Guide

This guide defines how a high level server or dashboard communicates with one
robot arm through RabbitMQ. The important rule is that the high level system
does not control low level ROS2 nodes directly. It sends business-level
commands to the Cell External Bridge, and the bridge talks only to Robot Cell
Orchestrator. Robot Cell Orchestrator is the abstraction layer that owns the
ROS2 node details, runtime files, readiness checks, and robot cycle state.

```text
External server/dashboard
  -> RabbitMQ command/status messages
  -> Cell External Bridge
  -> Robot Cell Orchestrator ROS API
  -> item/tray detect, item pick, tray intercept, robot bringup, and motion nodes
```

## Boundary Rule

- External systems publish only RabbitMQ messages to the arm command routing
  keys.
- Cell External Bridge receives those messages and translates them to Robot Cell
  Orchestrator service calls.
- Robot Cell Orchestrator is the only component that should decide which lower
  ROS2 nodes are launched, armed, configured, or sequenced.
- New high level features should add a Robot Cell Orchestrator API first, then
  expose that API through Cell External Bridge.

This keeps the external contract stable even when item detect, tray detect,
item pick, tray intercept, or robot movement internals change.

## RabbitMQ Topology

The bridge uses a topic exchange. The default exchange is `catarm.events`, set
by `RABBITMQ_EXCHANGE`.

| Purpose | Routing key |
| --- | --- |
| Commands into one arm | `belt.{belt_id}.arm.{arm_number}.cmd.{command}` |
| Status responses from one arm | `belt.{belt_id}.arm.{arm_number}.status` |
| Heartbeat telemetry from one arm | `belt.{belt_id}.arm.{arm_number}.telemetry` |
| Conveyor state broadcast to all arms on a belt | `belt.{belt_id}.conveyor.state.changed` |

The current command suffixes are `load_program`, `pick`, and `place`.

## Message Envelope

Commands are JSON payloads. Responses always include the standard arm envelope:

```json
{
  "event": "pick_ok",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:30:00+00:00"
}
```

Additional fields are added for each command response. Fields with no value are
omitted.

If `MESSAGE_SIGNING_KEY` is configured, both sides must sign message bodies with
the `x-signature` header. Unsigned messages are accepted only when signing is not
configured.

## Commands And Responses

### `cmd.load_program`

Use this before pick/place to tell the cell which local teach files and tray
placement should be active for the next production run.

Routing key:

```text
belt.{belt_id}.arm.{arm_number}.cmd.load_program
```

Canonical payload:

```json
{
  "event": "load_program",
  "qqc_id": "meal_kit_a",
  "xml_version": 7,
  "bin_teach_file": "bin_blue_bin_21052026.yaml",
  "item_teach_file": "item_cup_bin_blue_bin_21052026.yaml",
  "tray_teach_file": "tray_blue_tray_21052026.yaml",
  "tray_x_mm": 25.0,
  "tray_y_mm": 30.0,
  "tray_rz_deg": 5.0
}
```

Nested teach and placement fields are also accepted:

```json
{
  "event": "load_program",
  "qqc_id": "meal_kit_a",
  "teach_files": {
    "bin": "bin_blue_bin_21052026.yaml",
    "item": "item_cup_bin_blue_bin_21052026.yaml",
    "tray": "tray_blue_tray_21052026.yaml"
  },
  "tray_placement": {
    "x": 25.0,
    "y": 30.0,
    "rz": 5.0
  }
}
```

The teach file values are local YAML basenames. They are not file contents and
they must not include directory paths. Robot Cell Orchestrator resolves them
from the local teach folders, validates them, clears old runtime YAMLs, copies
the accepted files into `runtime/`, writes the selected item/tray runtime state,
and applies the tray placement for tray intercept.

Success response:

```json
{
  "event": "load_program_ok",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:30:00+00:00",
  "qqc_id": "meal_kit_a",
  "xml_version": 7,
  "files": [
    "bin_blue_bin_21052026.yaml",
    "item_cup_bin_blue_bin_21052026.yaml",
    "tray_blue_tray_21052026.yaml"
  ],
  "bin_teach_file": "bin_blue_bin_21052026.yaml",
  "item_teach_file": "item_cup_bin_blue_bin_21052026.yaml",
  "tray_teach_file": "tray_blue_tray_21052026.yaml",
  "tray_x_mm": 25.0,
  "tray_y_mm": 30.0,
  "tray_rz_deg": 5.0
}
```

Failure response:

```json
{
  "event": "load_program_failed",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:30:00+00:00",
  "qqc_id": "meal_kit_a",
  "xml_version": 7,
  "reason": "robot_cell_orchestrator_rejected"
}
```

Common load failures include missing `qqc_id`, missing teach file names, invalid
tray placement numbers, unavailable Robot Cell Orchestrator services, or a Robot
Cell Orchestrator validation rejection.

### `cmd.pick`

Use this to release one pick cycle after a program has been loaded.

Routing key:

```text
belt.{belt_id}.arm.{arm_number}.cmd.pick
```

Payload:

```json
{
  "event": "pick",
  "qqc_id": "meal_kit_a",
  "tray_id": "tray_00042",
  "task_id": "task_00042_pick"
}
```

`qqc_id`, `tray_id`, and `task_id` are optional, but sending them makes logs and
dashboard correlation easier. When `qqc_id` is present it must match the most
recent accepted `cmd.load_program`.

Bridge behavior:

- Check that a QQC is loaded and that any requested `qqc_id` matches it.
- Call `robot_cell_orchestrator/start_online`.
- Wait for the Robot Cell Orchestrator `moving_to_tray` event.
- Publish `pick_ok` or `pick_failed`.

Success response:

```json
{
  "event": "pick_ok",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:31:00+00:00",
  "attempt": 1,
  "qqc_id": "meal_kit_a",
  "tray_id": "tray_00042",
  "task_id": "task_00042_pick",
  "cycle_index": 12,
  "phase_id": 344
}
```

Failure response:

```json
{
  "event": "pick_failed",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:31:00+00:00",
  "attempt": 1,
  "qqc_id": "meal_kit_a",
  "tray_id": "tray_00042",
  "task_id": "task_00042_pick",
  "reason": "robot_cell_orchestrator_timeout",
  "retryable": false
}
```

Common pick failures include `no_qqc_loaded`, `qqc_mismatch`,
`service_unavailable`, `robot_cell_orchestrator_rejected`,
`robot_cell_orchestrator_timeout`, `bridge_unavailable`, and `cancelled`.

### `cmd.place`

Use this after `pick_ok` when the external system is ready for the robot to
place the item and return toward the bin side.

Routing key:

```text
belt.{belt_id}.arm.{arm_number}.cmd.place
```

Payload:

```json
{
  "event": "place",
  "qqc_id": "meal_kit_a",
  "tray_id": "tray_00042",
  "task_id": "task_00042_place"
}
```

Bridge behavior:

- Check that a QQC is loaded and that any requested `qqc_id` matches it.
- Confirm a pick phase was seen first.
- Call `robot_cell_orchestrator/place_online`.
- Wait for the Robot Cell Orchestrator `moving_to_bin` event.
- Publish `place_ok` or `place_failed`.

Success response:

```json
{
  "event": "place_ok",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:32:00+00:00",
  "attempt": 1,
  "qqc_id": "meal_kit_a",
  "tray_id": "tray_00042",
  "task_id": "task_00042_place",
  "cycle_index": 12,
  "phase_id": 358
}
```

Failure response:

```json
{
  "event": "place_failed",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-21T10:32:00+00:00",
  "attempt": 1,
  "qqc_id": "meal_kit_a",
  "tray_id": "tray_00042",
  "task_id": "task_00042_place",
  "reason": "robot_cell_orchestrator_rejected",
  "retryable": false
}
```

Common place failures are the same as pick, plus the case where `cmd.place`
arrives before a successful `cmd.pick`.

## Conveyor State Broadcast

The external conveyor/server can broadcast state changes to all arms on a belt:

```text
belt.{belt_id}.conveyor.state.changed
```

Payload:

```json
{
  "state": "stopped",
  "triggered_by": "operator_stop"
}
```

`state=stopped` asks each bridge to cancel any in-flight Robot Cell Orchestrator
cycle at the next polling boundary. New pick/place work is normally paused by
the server simply not publishing new commands until the conveyor is running
again.

## Robot Cell Orchestrator API Used By The Bridge

Cell External Bridge should call these Robot Cell Orchestrator endpoints only.
Lower ROS2 nodes stay behind this API.

| API | Type | Used for |
| --- | --- | --- |
| `robot_cell_orchestrator/load_online_program` | `dobot_msgs_v4/srv/LoadOnlineProgram` | Select local teach files, write runtime handoff, validate readiness |
| `robot_cell_orchestrator/validate_online_program` | `std_srvs/srv/Trigger` | Check online readiness |
| `robot_cell_orchestrator/start_online` | `std_srvs/srv/Trigger` | Release the pick side of the online cycle |
| `robot_cell_orchestrator/place_online` | `std_srvs/srv/Trigger` | Release the place side of the online cycle |
| `robot_cell_orchestrator/events` | `std_msgs/msg/String` JSON | Publish phase events such as `moving_to_tray`, `moving_to_bin`, and `timeout` |

## Adding More Functions Later

Use this pattern when adding a new high level server/dashboard command:

1. Define the operator meaning first, for example `pause`, `resume`, `reject_tray`,
   `home_robot`, or `reload_program`.
2. Add or extend a Robot Cell Orchestrator ROS API that owns the lower ROS2 node
   behavior.
3. Add a client method in `cell_external_bridge/src/cell_external_bridge/bridge.py`.
4. Add the RabbitMQ command dispatch and response events in
   `cell_external_bridge/src/cell_external_bridge/robot_arm_controller.py`.
5. Add tests under `cell_external_bridge/tests`.
6. Document the new command, payload, success response, and failure response in
   this guide.

Use the same response style: `{command}_ok` for success and `{command}_failed`
for failure. Include `reason`, `retryable`, and correlation fields such as
`qqc_id`, `tray_id`, or `task_id` whenever they help the dashboard explain what
happened.

