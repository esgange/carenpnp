import time

import rclpy

from orbbec_camera_launcher import camera_watchdog


class FakeProcess:
    next_pid = 43000

    def __init__(self, *_args, **_kwargs):
        self.pid = FakeProcess.next_pid
        FakeProcess.next_pid += 1
        self.returncode = None

    def poll(self):
        return self.returncode

    def send_signal(self, _signal):
        self.returncode = 0


def test_watchdog_restarts_stale_and_exited_camera(monkeypatch):
    monkeypatch.setattr(camera_watchdog.subprocess, 'Popen', FakeProcess)
    rclpy.init(args=[
        '--ros-args',
        '-p', "camera_names:=['test_camera']",
        '-p', "serial_numbers:=['TEST123']",
        '-p', 'startup_timeout_sec:=1.0',
        '-p', 'health_timeout_sec:=1.0',
        '-p', 'restart_delay_sec:=0.1',
        '-p', 'restart_backoff_max_sec:=0.2',
        '-p', 'shutdown_timeout_sec:=0.1',
    ])
    node = None
    try:
        node = camera_watchdog.CameraWatchdog()
        state = node._states['test_camera']
        first_pid = state.process.pid

        node._record_image('test_camera', 'color')
        node._record_image('test_camera', 'depth')
        node._check_cameras()
        assert state.phase == 'healthy'

        state.last_depth_at = time.monotonic() - 2.0
        node._check_cameras()
        assert state.phase == 'waiting'
        assert state.restart_count == 1

        state.next_start_at = 0.0
        node._check_cameras()
        assert state.phase == 'starting'
        assert state.process.pid != first_pid

        state.process.returncode = 7
        node._check_cameras()
        assert state.phase == 'waiting'
        assert state.restart_count == 2
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
