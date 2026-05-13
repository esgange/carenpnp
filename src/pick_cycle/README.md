# pick_cycle

Mini GUI for automating the existing item-pick and tray-intercept cycle.

Default non-servo sequence:

Startup gate:

- Create the service clients once when the node starts
- During node startup, wait up to 5.5 seconds for the configured trigger
  services and tray start service to be ready
- Cache that startup result and reuse those clients for the whole GUI session
- Re-check all required services at the start of every cycle; stop the cycle if
  any required service is unavailable

1. `/item_detect/go_to_teach`
2. `/item_pick/track`
3. Verify `/item_pick/track_status` reports armed
4. Monitor robot TCP feedback until it is stable
5. `/item_detect/seek`
6. Wait for `/item_detect/seek_status` to turn on, then off
7. `/tray_detect/go_to_teach`
8. `/tray_intercept/start_sequence` with the mini GUI tray X/Y/RZ settings
9. Verify `/tray_intercept/track_status` reports armed
10. Monitor robot TCP feedback until it is stable
11. `/tray_detect/seek`
12. Wait for `/tray_detect/seek_status` to turn on, then off

Servo GUI sequence:

1. `/item_pick_servo/track`
2. Verify `/item_pick_servo/track_status` reports armed
3. Monitor robot TCP feedback until it is stable
4. `/item_detect/seek`
5. Wait for `/item_detect/seek_status` to turn on, then off
6. `/tray_intercept_servo/start_sequence` with the mini GUI tray X/Y/RZ settings
7. Verify `/tray_intercept_servo/track_status` reports armed
8. Monitor robot TCP feedback until it is stable
9. `/tray_detect/seek`
10. Wait for `/tray_detect/seek_status` to turn on, then off

For the servo GUI, teach-return motion lives inside the motion packages:
`item_pick_servo` returns to the active tray teach pose after item pick, and
`tray_intercept_servo` returns to the active item teach pose after tray intercept.

Item services and seek services are sent as virtual clicks. Tray arm uses
`dobot_msgs_v4/srv/TrayInterceptStart` so the mini GUI can pass tray intercept
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

Launch the non-servo pick-cycle GUI:

```bash
ros2 launch pick_cycle pick_cycle.launch.py
```

Launch the servo pick-cycle GUI:

```bash
ros2 launch pick_cycle pick_cycle_servo.launch.py
```

To open both GUIs at the same time, use two terminals:

```bash
# Terminal 1
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pick_cycle pick_cycle.launch.py
```

```bash
# Terminal 2
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pick_cycle pick_cycle_servo.launch.py
```

Direct run commands are also available:

```bash
ros2 run pick_cycle pick_cycle_gui
ros2 run pick_cycle pick_cycle_gui_servo
```

`pick_cycle_gui` arms `item_pick` and `tray_intercept`. `pick_cycle_gui_servo`
uses the shared `item_detect` and `tray_detect` seek services, but no longer
calls either perception Go To Teach service directly. It arms `item_pick_servo`
and `tray_intercept_servo`; those motion packages handle the servo teach-return
moves. Do not run both cycles actively at the same time against the same robot.

## Background movement timing

The GUI now includes a passive movement tracker. It observes the configured robot
TCP feedback topic in the background and logs a timing line after each detected
physical movement settles, for example:

```text
Movement tracker: Cycle 1: Arm item pick -> Cycle 1: Seek item detect travel took 2.34s (TCP delta 185.6mm, 0.0deg)
```

This tracker does not send robot commands, add sleeps, or change the cycle
sequence. It only watches TCP feedback and uses the same stability window chosen
in the GUI to decide when a detected movement has finished.

Each cycle also writes one debug text file containing all completed movement
delta events for that cycle under:

```text
WORKSPACE_ROOT/debug files/pick_cycle_movement_deltas
```

File names use `YYYYMMDD_HHMMSS_cycle_<n>_movement_deltas.txt` and are updated
as movement tracker events arrive during that cycle.
