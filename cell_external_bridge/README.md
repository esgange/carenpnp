# Cell External Bridge

Cell External Bridge is the RabbitMQ-facing process for one robotic arm. It is the
external communicator only: external master/conveyor commands arrive over
RabbitMQ, Cell External Bridge forwards online program selections to Robot Cell Orchestrator,
and Robot Cell Orchestrator owns the root `runtime/` folder, readiness, and motion through ROS
services.

## Runtime Role

One Cell External Bridge process owns one arm:

```text
Conveyor/master
  -> RabbitMQ cmd.load_program / cmd.pick / cmd.place
  -> cell_external_bridge.robot_arm_controller.RobotArmController
  -> Robot Cell Orchestrator ROS API
  -> root runtime/ YAML handoff
  -> RabbitMQ load_program_ok / pick_ok / place_ok or *_failed
```

Robot Cell Orchestrator remains the authority for online/offline mode, calibration, runtime
teach-file validation, service readiness, and the pick/place sequence.

For the full external server/dashboard contract, see
[`external-rabbitmq-communication-guide.md`](docs/external-rabbitmq-communication-guide.md).

## Online Flow

`cmd.load_program`

- Receives `qqc_id`, local teach filenames for bin/item/tray, and tray placement
  `x`, `y`, `rz`.
- Calls `/robot_cell_orchestrator/load_online_program`.
- Robot Cell Orchestrator finds those files in the local teach folders, copies the selected
  bin/item/tray YAMLs plus the matching item tool sidecar into `runtime/`, and
  applies the tray placement.
- Publishes `load_program_ok` only when Robot Cell Orchestrator accepts the requested files
  and online readiness passes.

`cmd.pick`

- Uses the currently loaded `qqc_id`.
- Calls `/robot_cell_orchestrator/start_online`.
- Waits for Robot Cell Orchestrator event `moving_to_tray`.
- Publishes `pick_ok`.
- If `cmd.place` never arrives, Robot Cell Orchestrator stays waiting above the conveyor.

`cmd.place`

- Uses the currently loaded `qqc_id`.
- Calls `/robot_cell_orchestrator/place_online`.
- Waits for Robot Cell Orchestrator event `moving_to_bin`.
- Publishes `place_ok`.

Timeouts publish only `pick_failed` or `place_failed` in v1. The online
no-sensor path does not emit conveyor-stop, issue-resolved, or
manual-intervention events.

## Message Shape

All arm-published messages share this envelope:

```json
{
  "event": "pick_ok",
  "belt_id": "belt-a",
  "edge_node_id": "cell_bridge_01",
  "robot_arm_id": "arm_01",
  "arm_number": 1,
  "timestamp": "2026-05-08T18:30:00Z"
}
```

`cmd.load_program` example:

```json
{
  "event": "load_program",
  "qqc_id": "water_bottle",
  "xml_version": 7,
  "xml_sha256": "optional-metadata",
  "bin_teach_file": "bin_blue_bin_08052026.yaml",
  "item_teach_file": "item_paper_cutlery_bin_blue_bin_08052026.yaml",
  "tray_teach_file": "tray_blue_tray_06052026.yaml",
  "tray_x_mm": 25.0,
  "tray_y_mm": 30.0,
  "tray_rz_deg": 5.0
}
```

Nested form is also accepted:

```json
{
  "event": "load_program",
  "qqc_id": "water_bottle",
  "teach_files": {
    "bin": "bin_blue_bin_08052026.yaml",
    "item": "item_paper_cutlery_bin_blue_bin_08052026.yaml",
    "tray": "tray_blue_tray_06052026.yaml"
  },
  "tray_placement": {
    "x": 25.0,
    "y": 30.0,
    "rz": 5.0
  }
}
```

`cmd.pick` / `cmd.place` example:

```json
{
  "event": "pick"
}
```

`cmd.pick` and `cmd.place` do not need teach filenames, tray placement, tray id,
or task id. They operate on the program loaded by the most recent accepted
`cmd.load_program`.

## RabbitMQ Topology

- Commands:
  `belt.{belt_id}.arm.{arm_number}.cmd.{load_program|pick|place}`
- Status:
  `belt.{belt_id}.arm.{arm_number}.status`
- Telemetry:
  `belt.{belt_id}.arm.{arm_number}.telemetry`
- Conveyor state broadcast:
  `belt.{belt_id}.conveyor.state.changed`

`MESSAGE_SIGNING_KEY` enables HMAC-SHA256 verification through
`packages/shared-contracts`. Production should configure the same key on the
conveyor/master and every Cell External Bridge process.

## Running

From the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/deps/source_third_party_env.sh
set -a
source station_config
set +a
cell-external-bridge
```

For a second arm overlay:

```text
CELL_BRIDGE_ID=cell_bridge_02
ROBOT_ARM_ID=arm_02
ARM_NUMBER=2
```

For a pure RabbitMQ smoke test with no ROS bridge import:

```bash
set -a
source station_config
set +a
export CELL_BRIDGE_SIMULATE_MODE=success
cell-external-bridge
```

For lab debugging from Robot Cell Orchestrator GUI, click **Cell External Bridge** while the
GUI is in Offline mode. Robot Cell Orchestrator starts this package as a normal Python
process with:

```text
CELL_BRIDGE_OFFLINE_DEBUG=1
CELL_BRIDGE_DATALOG_DIR=debug files/cell_external_bridge
```

Offline debug still connects to RabbitMQ and verifies signed messages, but it
does not write `runtime/` or call Robot Cell Orchestrator ROS services. It logs received
commands/state broadcasts as JSONL and immediately publishes fake
`load_program_ok`, `pick_ok`, and `place_ok` responses.

For editable development:

```bash
source /opt/ros/humble/setup.bash
source <workspace-install>/setup.bash
cd /home/erds/DOBOT_pickn_place
pip install -e "cell_external_bridge[test]"
set -a
source station_config
set +a
export CELL_BRIDGE_SIMULATE_MODE=success
cell-external-bridge
```

## Configuration

The main station setup file is [station_config](/home/erds/DOBOT_pickn_place/station_config).
It uses simple `KEY=value` lines so it can be sourced by a shell before running
station processes. Environment variables still work as temporary one-command
overrides.

Normal `station_config` keys:

| Key / YAML key | Purpose | Default |
| --- | --- | --- |
| `BELT_ID` / `belt_id` | Belt this arm belongs to | `belt-a` |
| `CELL_BRIDGE_ID` / `edge_node_id` | Bridge identifier; published as `edge_node_id` for external compatibility | `cell_bridge_01` |
| `ROBOT_ARM_ID` / `robot_arm_id` | Arm identifier used in logs/status | `arm_01` |
| `ARM_NUMBER` / `arm_number` | Arm number on this belt | `1` |
| `RABBITMQ_URL` / `rabbitmq_url` | AMQP URL | `amqp://guest:guest@localhost:5672/` |
| `RABBITMQ_EXCHANGE` / `exchange_name` | Topic exchange | `catarm.events` |
| `ROBOT_NUMBER` / `robot_number` | Robot count passed to bringup | `1` |
| `ROS_LOCALHOST_ONLY` / `ros_localhost_only` | Restrict ROS discovery to this computer | `false` |
| `ROBOT_IP_ADDRESS` / `node_info[].ip_address` | DOBOT controller IP address | required for station_config bringup |
| `ROBOT_TYPE` / `node_info[].robot_type` | DOBOT model, e.g. `cr5`, `cr10`, `cr16` | `cr5` |

Optional/debug keys:

| Key / YAML key | Purpose | Default |
| --- | --- | --- |
| `ROBOT_CELL_ORCHESTRATOR_LOAD_PROGRAM_SERVICE` / `robot_cell_orchestrator_load_program_service` | Local teach selection + tray placement service | `robot_cell_orchestrator/load_online_program` |
| `ROBOT_CELL_ORCHESTRATOR_VALIDATE_SERVICE` / `robot_cell_orchestrator_validate_service` | Runtime/readiness validation service | `robot_cell_orchestrator/validate_online_program` |
| `ROBOT_CELL_ORCHESTRATOR_START_SERVICE` / `robot_cell_orchestrator_start_service` | Online pick-side release service | `robot_cell_orchestrator/start_online` |
| `ROBOT_CELL_ORCHESTRATOR_PLACE_SERVICE` / `robot_cell_orchestrator_place_service` | Online place-side release service | `robot_cell_orchestrator/place_online` |
| `ROBOT_CELL_ORCHESTRATOR_EVENTS_TOPIC` / `robot_cell_orchestrator_events_topic` | Robot Cell Orchestrator phase events | `robot_cell_orchestrator/events` |
| `ROBOT_CELL_ORCHESTRATOR_PHASE_TIMEOUT_S` / `robot_cell_orchestrator_phase_timeout_s` | Phase wait timeout | `120.0` |
| `CELL_BRIDGE_CONFIG_PATH` | Optional YAML overlay | unset |
| `CELL_BRIDGE_SIMULATE_MODE` | Deterministic motion simulation | unset |
| `RUNTIME_DIR` / `runtime_dir` | Legacy/dev fallback for old direct-write tests; normally unset | `/ws/runtime` |
| `CELL_BRIDGE_OFFLINE_DEBUG` / `offline_debug` | RabbitMQ datalog + fake OK responses for Offline GUI debugging | `false` |
| `CELL_BRIDGE_DATALOG_DIR` / `datalog_dir` | Offline debug JSONL output directory | `debug files/cell_external_bridge` |
| `CELL_BRIDGE_WARMUP_TIMEOUT_S` / `bridge_warmup_timeout_s` | Startup service readiness wait | `30.0` |
| `HEARTBEAT_INTERVAL_S` / `heartbeat_interval_s` | Telemetry interval; `0` disables | `5.0` |
| `LOG_LEVEL` | Python log level | `INFO` |
| `STATION_CONFIG_PATH` | Optional alternate config file path | `station_config` |

Do not put `ROS_DOMAIN_ID` in `station_config`; ROS domain setup belongs to robot
bringup or the shell that starts the ROS graph.

Legacy `EDGE_NODE_ID`, `EDGE_CONFIG_PATH`, `EDGE_SIMULATE_MODE`, and
`EDGE_BRIDGE_WARMUP_TIMEOUT_S` are still accepted as fallbacks, but new arm
setups should use the `CELL_BRIDGE_*` names in `station_config`.

`pyproject.toml` is still required. It defines the Python package, dependencies,
and the `cell-external-bridge` command installed by `pip` or the colcon Python
package build.

## Directory Layout

```text
station_config
cell_external_bridge/
  pyproject.toml
  README.md
  config/
    station_01.yaml
    station_02.yaml
  docs/
    distributed-deployment.md
    external-rabbitmq-communication-guide.md
    robot_arm_team_handoff.md
  src/cell_external_bridge/
    __init__.py
    arm_communication.py
    bridge.py
    config.py
    messaging.py
    robot_arm_controller.py
    runtime_program.py
  tests/
    test_arm_communication_bridge.py
    test_robot_arm_controller.py
```

## Tests

```bash
cd /home/erds/DOBOT_pickn_place
pip install -e "cell_external_bridge[test]"
PYTHONPATH=cell_external_bridge/src python -m pytest cell_external_bridge/tests/ -v
```

The bridge tests stub the ROS client where needed, so the focused unit suite can
run on a host without a live robot or Robot Cell Orchestrator graph.
