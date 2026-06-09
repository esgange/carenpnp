# Distributed Deployment (LAN)

The CATARM arm-side stack runs natively on each station PC. The conveyor/master
PC owns RabbitMQ and the external orchestration services; each arm PC runs one
Cell External Bridge process plus the local ROS 2 robot/perception stack.

```text
                 +-----------------------------+
                 |        Conveyor PC          |
                 |                             |
                 |  RabbitMQ  :5672 (LAN)      |
                 |  Conveyor/master services   |
                 |  Database / storage stack   |
                 +--------+----------+---------+
                          |          ^
   cmd.load_program|pick|place       | arm.*.status
                          |          | arm.*.telemetry
   belt.X.arm.N.cmd.*     v          |
                  +-------+-+--+-----+--------+
                  |          LAN              |
                  +-+------+-----+------+-----+
                    |      |     |      |
                    v      v     v      v
                +-------+-------+-------+-------+
                | Arm PC| Arm PC| Arm PC| Arm PC| ...
                |  N=1  |  N=2  |  N=3  |  N=8  |
                |bridge |bridge |bridge |bridge |
                | + ROS | + ROS | + ROS | + ROS |
                +-------+-------+-------+-------+
```

All command/feedback traffic flows through one RabbitMQ topic exchange
(`catarm.events`) on the conveyor PC. The conveyor server should bind queues for
`belt.{belt_id}.arm.*.status` / `.telemetry` / `.result`. Each arm PC must
publish with the right routing keys and HMAC key for feedback to work
end-to-end.

## Conveyor PC Setup

1. Install and start RabbitMQ plus the conveyor/master services by the conveyor
   project’s normal host-native process manager.
2. Configure credentials, `MESSAGE_SIGNING_KEY`, belt IDs, and service secrets.
3. Optionally restrict which NIC RabbitMQ accepts on.
4. Open the firewall for the arm-PC subnet:
   - TCP/5672 (AMQP, required)
   - TCP/15672 (RabbitMQ management UI, optional)
   - Conveyor UI/API ports used by operators.

Verify the broker is reachable from the LAN:

```bash
nc -vz <conveyor-pc-ip> 5672
```

## Arm PC Setup

1. Install this workspace with the host-native process in the root
   [INSTALL.md](../../INSTALL.md).
2. Fill in the per-arm root config file:

   ```bash
   code station_config
   ```

   Critical fields:

   | Variable | Example | Notes |
   | --- | --- | --- |
   | `RABBITMQ_URL` | `amqp://catarm-server:<pass>@192.168.1.10:5672/%2Fcatarm` | Full AMQP URL for the conveyor PC broker. |
   | `MESSAGE_SIGNING_KEY` | `<32-byte hex>` | Byte-identical to the conveyor PC value. |
   | `BELT_ID` | `belt-a` | Same belt id used by the conveyor server for this belt. |
   | `ARM_NUMBER` | `1`-`8` | Unique per arm on the belt; drives every routing key. |
   | `ROBOT_ARM_ID` | `arm_01` | Identifier the robot ships with; appears in load XML and logs. |
   | `CELL_BRIDGE_ID` | `cell_bridge_01` | Bridge identifier published as `edge_node_id` for compatibility. |
   | `RUNTIME_DIR` | `runtime` | Online program handoff folder shared with Robot Cell Orchestrator. |
   | `ROBOT_IP_ADDRESS` | `192.168.200.1` | DOBOT controller IP used by Robot Bringup. |
   | `ROBOT_TYPE` | `cr10` | DOBOT model used by Robot Bringup. |
   | `ROBOT_NUMBER` | `1` | Robot count passed to Robot Bringup. |
   | `ROS_LOCALHOST_ONLY` | `true` | Keep ROS discovery local to this station PC. |

3. Source the workspace and start the bridge:

   ```bash
   cd /home/erds/DOBOT_pickn_place
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   source tools/deps/source_third_party_env.sh
   set -a
   source station_config
   set +a
   cell-external-bridge
   ```

You should see the controller connect to RabbitMQ, bind command routing keys,
and begin publishing heartbeat telemetry.

## Verifying Feedback

The fastest end-to-end smoke test is to simulate motion on the arm side and
watch the conveyor receive `pick_ok` / `place_ok`.

On the arm PC:

```bash
set -a
source station_config
set +a
export CELL_BRIDGE_SIMULATE_MODE=success
cell-external-bridge
```

On the conveyor PC, tail the conveyor server logs and look for the arm id,
`status`, `pick_ok`, and `place_ok`.

Any of the following prove the feedback path works:

- `arm.heartbeat` events arriving every `HEARTBEAT_INTERVAL_S`.
- `pick_ok` / `place_ok` after a synthetic cycle.
- `{op}_failed` after a forced failure (`CELL_BRIDGE_SIMULATE_MODE=fail_then_pass`).

The online no-sensor path does not emit conveyor-stop, issue-resolved, or
manual-intervention events for v1 timeout/failure handling.

## Troubleshooting

- `connection.refused` on the arm PC: the conveyor PC firewall is blocking
  5672, or `RABBITMQ_URL` points to the wrong host. Run
  `nc -vz <conveyor-host> 5672` from the arm PC.
- `ACCESS_REFUSED` / `Login was refused`: credentials in the arm PC's
  `station_config` do not match the conveyor PC credentials.
- The conveyor logs `Rejected ... -- bad HMAC` for every message:
  `MESSAGE_SIGNING_KEY` differs between PCs.
- The arm publishes `pick_ok` but the conveyor never reacts: confirm `BELT_ID`
  matches between the conveyor server and the arm PC. Routing keys are
  belt-scoped, so a typo silently swallows everything.
- ROS DDS discovery failures across arm PCs: handle `ROS_DOMAIN_ID` in robot
  bringup or in the shell that starts the ROS graph. It is not a Cell External
  Bridge setting.
