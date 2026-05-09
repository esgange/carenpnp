# pick_cycle

Mini GUI for automating the existing item-pick and tray-intercept cycle.

Default sequence:

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

Run:

```bash
ros2 launch pick_cycle pick_cycle.launch.py
```
