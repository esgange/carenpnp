# Robot Arm Team Hand-off

Cell External Bridge talks to the running ROS 2 Robot Cell Orchestrator node through services and
phase events. Robot Cell Orchestrator owns online/offline mode, runtime validation,
readiness gates, and motion. Cell External Bridge owns RabbitMQ, runtime file
materialization, and external acknowledgements.

## What Runs The Cycle

```text
RabbitMQ cmd.load_program
        |
        v
cell_external_bridge.runtime_program.materialize_runtime_program
        |
        v
WORKSPACE_ROOT/runtime/*.yaml
        |
        v
Robot Cell Orchestrator /robot_cell_orchestrator/validate_online_program

RabbitMQ cmd.pick / cmd.place
        |
        v
cell_external_bridge.robot_arm_controller.RobotArmController
        |
        v
cell_external_bridge.arm_communication.ArmCommunication._do_pick / _do_place
        |
        v
cell_external_bridge.bridge.RosCycleClient
        |
        v
Robot Cell Orchestrator /robot_cell_orchestrator/start_online + /robot_cell_orchestrator/place_online + /robot_cell_orchestrator/events
```

`cmd.pick` calls `/robot_cell_orchestrator/start_online`, waits for `moving_to_tray`, and
publishes `pick_ok`. Robot Cell Orchestrator then waits at the conveyor side with the robot
ready above the conveyor. `cmd.place` calls `/robot_cell_orchestrator/place_online`, waits
for `moving_to_bin`, and publishes `place_ok`.

## Files You Usually Edit

- Root Robot Cell Orchestrator source:
  `src/robot_cell_orchestrator/robot_cell_orchestrator/robot_cell_orchestrator_gui.py`
- Root item/tray/robot service packages:
  `src/item_pick/`, `src/tray_intercept/`, `src/dobot_bringup_v4/`
- Cell External Bridge ROS client only when the ROS API contract changes:
  `cell_external_bridge/src/cell_external_bridge/bridge.py`
- Runtime file payload handling:
  `cell_external_bridge/src/cell_external_bridge/runtime_program.py`

`arm_communication.py` should stay small: it only chooses simulate mode or the
ROS bridge call.

## Bridge Return Contract

`RosCycleClient.run_pick` / `run_place` return:

| Return value | Wrapper behavior |
| --- | --- |
| `{"status": "ok"}` | Publish `pick_ok` / `place_ok`. |
| `{"status": "error", "reason": "<snake_case>"}` | Publish `{op}_failed`. |

The v1 online no-sensor path does not emit conveyor-stop, issue-resolved, or
manual-intervention events for timeout/failure handling.

## Failure Reasons

| Reason | Meaning | Retryable |
| --- | --- | --- |
| `service_unavailable` | Required Robot Cell Orchestrator service was not reachable. | `True` |
| `robot_cell_orchestrator_rejected` | Robot Cell Orchestrator rejected validation/start/place, often offline/maintenance. | `False` |
| `robot_cell_orchestrator_timeout` | Robot Cell Orchestrator did not emit the expected phase event before timeout. | `False` |
| `bridge_unavailable` | `rclpy` or the Robot Cell Orchestrator client could not be imported/constructed. | varies |
| `ros_cycle_failed` | Catch-all when no specific reason was matched. | `True` |

Add new reasons in `bridge.py` and document them here.

## Runtime Program Contract

`cmd.load_program` must include at least one YAML file in `runtime_files`,
`program_files`, or `teach_files`. Cell External Bridge only checks file safety and
writes the files. Robot Cell Orchestrator validates semantic content, including required
teach types.

Accepted file object fields:

- `filename`, `name`, or `path` for the basename.
- `content_b64`, `b64`, or `content` for file content.

Unsafe paths and non-YAML files are rejected before anything is written.

## CELL_BRIDGE_SIMULATE_MODE

`arm_communication._do_pick` / `_do_place` short-circuit to
`RobotArmController._simulate(...)` when `CELL_BRIDGE_SIMULATE_MODE` is
non-empty.
Modes: `success`, `always_fail`, `always_fail_hard`, `fail_then_pass`,
`fail_twice_then_pass`.

This mode is for RabbitMQ/status smoke tests and unit tests. It does not touch
the ROS bridge.

## Tests

- `tests/test_robot_arm_controller.py` covers load-program materialization,
  online status events, telemetry, and QQC preconditions.
- `tests/test_arm_communication_bridge.py` covers the bridge wiring and
  fallback when ROS imports are unavailable.
