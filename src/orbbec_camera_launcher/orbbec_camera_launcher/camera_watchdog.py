from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@dataclass
class CameraState:
    name: str
    serial_number: str
    phase: str = 'waiting'
    process: subprocess.Popen | None = None
    process_group_id: int | None = None
    launch_started_at: float | None = None
    last_color_at: float | None = None
    last_depth_at: float | None = None
    next_start_at: float = 0.0
    restart_count: int = 0
    consecutive_failures: int = 0
    last_error: str = ''


class CameraWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('supervisor')

        self.declare_parameter('camera_names', [''])
        self.declare_parameter('serial_numbers', [''])
        self.declare_parameter('orbbec_launch_file', 'gemini_330_series.launch.py')
        self.declare_parameter('device_num', '1')
        self.declare_parameter('launch_args_json', '{}')
        self.declare_parameter('workspace_root', '')
        self.declare_parameter('startup_timeout_sec', 20.0)
        self.declare_parameter('health_timeout_sec', 5.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('restart_delay_sec', 3.0)
        self.declare_parameter('restart_backoff_max_sec', 30.0)
        self.declare_parameter('shutdown_timeout_sec', 3.0)

        camera_names = [
            str(value).strip()
            for value in self.get_parameter('camera_names').value
            if str(value).strip()
        ]
        serial_numbers = [
            str(value).strip()
            for value in self.get_parameter('serial_numbers').value
            if str(value).strip()
        ]
        if not camera_names or len(camera_names) != len(serial_numbers):
            raise RuntimeError(
                'camera_names and serial_numbers must be non-empty lists with matching lengths.'
            )

        self._orbbec_launch_file = str(
            self.get_parameter('orbbec_launch_file').value
        ).strip()
        self._device_num = str(self.get_parameter('device_num').value).strip() or str(
            len(camera_names)
        )
        self._workspace_root = Path(
            str(self.get_parameter('workspace_root').value).strip() or os.getcwd()
        ).expanduser()
        self._startup_timeout_sec = self._positive_parameter('startup_timeout_sec', 20.0)
        self._health_timeout_sec = self._positive_parameter('health_timeout_sec', 5.0)
        self._check_period_sec = self._positive_parameter('check_period_sec', 1.0)
        self._restart_delay_sec = self._positive_parameter('restart_delay_sec', 3.0)
        self._restart_backoff_max_sec = max(
            self._restart_delay_sec,
            self._positive_parameter('restart_backoff_max_sec', 30.0),
        )
        self._shutdown_timeout_sec = self._positive_parameter('shutdown_timeout_sec', 3.0)
        self._launch_args = self._load_launch_args()

        now = time.monotonic()
        self._states = {
            name: CameraState(
                name=name,
                serial_number=serial,
                next_start_at=now,
            )
            for name, serial in zip(camera_names, serial_numbers)
        }
        self._start_queue = list(camera_names)
        self._starting_camera: str | None = None
        self._lock = threading.RLock()
        self._shutting_down = False
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._subscription_group = ReentrantCallbackGroup()
        self._image_subscriptions = []

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            'status',
            status_qos,
        )
        self._healthy_publisher = self.create_publisher(Bool, 'healthy', status_qos)
        self._restart_service = self.create_service(
            Trigger,
            'restart_all',
            self._restart_all,
            callback_group=self._control_group,
        )

        for camera_name in camera_names:
            self._image_subscriptions.extend([
                self.create_subscription(
                    Image,
                    f'/{camera_name}/color/image_raw',
                    lambda _message, name=camera_name: self._record_image(name, 'color'),
                    qos_profile_sensor_data,
                    callback_group=self._subscription_group,
                ),
                self.create_subscription(
                    Image,
                    f'/{camera_name}/depth/image_raw',
                    lambda _message, name=camera_name: self._record_image(name, 'depth'),
                    qos_profile_sensor_data,
                    callback_group=self._subscription_group,
                ),
            ])

        self._timer = self.create_timer(
            self._check_period_sec,
            self._check_cameras,
            callback_group=self._control_group,
        )
        self.get_logger().info(
            'Camera watchdog supervising: '
            + ', '.join(
                f'{state.name}({state.serial_number})' for state in self._states.values()
            )
        )
        self._check_cameras()

    def _positive_parameter(self, name: str, default: float) -> float:
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            return default
        return value if value > 0.0 else default

    def _load_launch_args(self) -> dict[str, str]:
        raw_value = str(self.get_parameter('launch_args_json').value).strip()
        try:
            payload = json.loads(raw_value or '{}')
        except json.JSONDecodeError as exc:
            raise RuntimeError(f'launch_args_json is invalid: {exc}') from exc
        if not isinstance(payload, dict):
            raise RuntimeError('launch_args_json must contain a JSON object.')
        return {str(key): self._argument_text(value) for key, value in payload.items()}

    @staticmethod
    def _argument_text(value: object) -> str:
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)

    def _record_image(self, camera_name: str, stream: str) -> None:
        now = time.monotonic()
        with self._lock:
            state = self._states.get(camera_name)
            if state is None:
                return
            if stream == 'color':
                state.last_color_at = now
            else:
                state.last_depth_at = now

    def _check_cameras(self) -> None:
        if self._shutting_down:
            return

        now = time.monotonic()
        processes_to_stop: list[tuple[subprocess.Popen, int | None]] = []
        with self._lock:
            for state in self._states.values():
                if state.phase not in ('starting', 'healthy'):
                    continue
                process = state.process
                if process is None:
                    continue
                return_code = process.poll()
                if return_code is not None:
                    stopped = self._mark_failed_locked(
                        state,
                        f'Orbbec launch process exited with code {return_code}',
                        now,
                    )
                    if stopped is not None:
                        processes_to_stop.append(stopped)
                    continue

                if state.phase == 'starting':
                    color_ready = (
                        state.last_color_at is not None
                        and state.launch_started_at is not None
                        and state.last_color_at >= state.launch_started_at
                    )
                    depth_ready = (
                        state.last_depth_at is not None
                        and state.launch_started_at is not None
                        and state.last_depth_at >= state.launch_started_at
                    )
                    if color_ready and depth_ready:
                        state.phase = 'healthy'
                        state.consecutive_failures = 0
                        state.last_error = ''
                        if self._starting_camera == state.name:
                            self._starting_camera = None
                        self.get_logger().info(
                            f'{state.name} is healthy; color and depth streams are fresh.'
                        )
                    elif (
                        state.launch_started_at is not None
                        and now - state.launch_started_at > self._startup_timeout_sec
                    ):
                        stopped = self._mark_failed_locked(
                            state,
                            'Color and depth streams did not become ready before startup timeout',
                            now,
                        )
                        if stopped is not None:
                            processes_to_stop.append(stopped)
                    continue

                color_age = self._stream_age(now, state.last_color_at)
                depth_age = self._stream_age(now, state.last_depth_at)
                if (
                    color_age > self._health_timeout_sec
                    or depth_age > self._health_timeout_sec
                ):
                    stopped = self._mark_failed_locked(
                        state,
                        (
                            'Image stream stale '
                            f'(color={self._age_text(color_age)}, '
                            f'depth={self._age_text(depth_age)})'
                        ),
                        now,
                    )
                    if stopped is not None:
                        processes_to_stop.append(stopped)

        for process, process_group_id in processes_to_stop:
            self._stop_process(process, process_group_id)

        with self._lock:
            self._queue_due_cameras_locked(now)
            self._start_next_camera_locked(now)
            self._publish_status_locked(now)

    def _mark_failed_locked(
        self,
        state: CameraState,
        reason: str,
        now: float,
    ) -> tuple[subprocess.Popen, int | None] | None:
        process = state.process
        process_group_id = state.process_group_id
        state.process = None
        state.process_group_id = None
        state.phase = 'waiting'
        state.launch_started_at = None
        state.last_color_at = None
        state.last_depth_at = None
        state.restart_count += 1
        state.consecutive_failures += 1
        delay = min(
            self._restart_delay_sec * (2 ** min(state.consecutive_failures - 1, 16)),
            self._restart_backoff_max_sec,
        )
        state.next_start_at = now + delay
        state.last_error = reason
        if self._starting_camera == state.name:
            self._starting_camera = None
        self.get_logger().error(
            f'{state.name} unhealthy: {reason}. Restart attempt scheduled in {delay:.1f}s.'
        )
        return (process, process_group_id) if process is not None else None

    def _queue_due_cameras_locked(self, now: float) -> None:
        for state in self._states.values():
            if (
                state.phase == 'waiting'
                and state.process is None
                and state.next_start_at <= now
                and state.name not in self._start_queue
            ):
                self._start_queue.append(state.name)

    def _start_next_camera_locked(self, now: float) -> None:
        if self._starting_camera is not None:
            return
        while self._start_queue:
            camera_name = self._start_queue.pop(0)
            state = self._states[camera_name]
            if state.phase != 'waiting' or state.process is not None:
                continue
            command = [
                'ros2',
                'launch',
                'orbbec_camera',
                self._orbbec_launch_file,
                f'camera_name:={state.name}',
                f'serial_number:={state.serial_number}',
                f'device_num:={self._device_num}',
                *[
                    f'{key}:={value}'
                    for key, value in self._launch_args.items()
                ],
            ]
            try:
                process = subprocess.Popen(
                    command,
                    cwd=str(self._workspace_root),
                    env=os.environ.copy(),
                    start_new_session=hasattr(os, 'setsid'),
                )
            except Exception as exc:  # noqa: BLE001
                self._mark_failed_locked(state, f'Failed to start camera launch: {exc}', now)
                continue

            state.process = process
            state.process_group_id = process.pid if hasattr(os, 'setsid') else None
            state.phase = 'starting'
            state.launch_started_at = now
            state.last_color_at = None
            state.last_depth_at = None
            state.last_error = ''
            self._starting_camera = state.name
            self.get_logger().info(
                f'Started {state.name} camera launch, pid={process.pid}; '
                'waiting for color and depth images.'
            )
            return

    def _restart_all(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        now = time.monotonic()
        with self._lock:
            processes = [
                (state.process, state.process_group_id)
                for state in self._states.values()
                if state.process is not None
            ]
            self._start_queue = []
            self._starting_camera = None
            for state in self._states.values():
                state.process = None
                state.process_group_id = None
                state.phase = 'waiting'
                state.launch_started_at = None
                state.last_color_at = None
                state.last_depth_at = None
                state.next_start_at = now
                state.consecutive_failures = 0
                state.last_error = 'Manual restart requested'
                self._start_queue.append(state.name)

        for process, process_group_id in processes:
            self._stop_process(process, process_group_id)

        with self._lock:
            self._start_next_camera_locked(time.monotonic())
            self._publish_status_locked(time.monotonic())
        response.success = True
        response.message = 'Camera restart requested.'
        return response

    def _publish_status_locked(self, now: float) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        all_healthy = bool(self._states)
        for state in self._states.values():
            status = DiagnosticStatus()
            status.name = f'camera_watchdog/{state.name}'
            status.hardware_id = state.serial_number
            if state.phase == 'healthy':
                status.level = DiagnosticStatus.OK
                status.message = 'Camera streams healthy'
            elif state.phase == 'starting':
                status.level = DiagnosticStatus.WARN
                status.message = 'Camera driver starting'
                all_healthy = False
            else:
                status.level = (
                    DiagnosticStatus.ERROR if state.restart_count else DiagnosticStatus.WARN
                )
                status.message = state.last_error or 'Waiting to start camera driver'
                all_healthy = False
            status.values = [
                KeyValue(key='state', value=state.phase),
                KeyValue(key='serial_number', value=state.serial_number),
                KeyValue(
                    key='color_age_sec',
                    value=self._age_text(self._stream_age(now, state.last_color_at)),
                ),
                KeyValue(
                    key='depth_age_sec',
                    value=self._age_text(self._stream_age(now, state.last_depth_at)),
                ),
                KeyValue(key='restart_count', value=str(state.restart_count)),
                KeyValue(
                    key='pid',
                    value=str(state.process.pid) if state.process is not None else '',
                ),
                KeyValue(key='last_error', value=state.last_error),
            ]
            message.status.append(status)

        self._status_publisher.publish(message)
        self._healthy_publisher.publish(Bool(data=all_healthy))

    @staticmethod
    def _stream_age(now: float, timestamp: float | None) -> float:
        return float('inf') if timestamp is None else max(0.0, now - timestamp)

    @staticmethod
    def _age_text(age: float) -> str:
        return 'never' if age == float('inf') else f'{age:.2f}'

    @staticmethod
    def _process_group_alive(process_group_id: int | None) -> bool:
        if process_group_id is None or not hasattr(os, 'killpg'):
            return False
        try:
            os.killpg(process_group_id, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _process_and_group_stopped(
        self,
        process: subprocess.Popen,
        process_group_id: int | None,
    ) -> bool:
        return (
            process.poll() is not None
            and not self._process_group_alive(process_group_id)
        )

    def _stop_process(
        self,
        process: subprocess.Popen,
        process_group_id: int | None,
    ) -> None:
        if process_group_id is None and process.poll() is None and hasattr(os, 'getpgid'):
            try:
                process_group_id = os.getpgid(process.pid)
            except (ProcessLookupError, PermissionError):
                process_group_id = None
        if self._process_and_group_stopped(process, process_group_id):
            return

        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                if (
                    process_group_id is not None
                    and self._process_group_alive(process_group_id)
                    and hasattr(os, 'killpg')
                ):
                    os.killpg(process_group_id, sig)
                elif process.poll() is None:
                    process.send_signal(sig)
            except ProcessLookupError:
                pass

            deadline = time.monotonic() + self._shutdown_timeout_sec
            while time.monotonic() < deadline:
                if self._process_and_group_stopped(process, process_group_id):
                    return
                time.sleep(0.05)

    def shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        with self._lock:
            processes = [
                (state.process, state.process_group_id)
                for state in self._states.values()
                if state.process is not None
            ]
            for state in self._states.values():
                state.process = None
                state.process_group_id = None
                state.phase = 'stopped'
        for process, process_group_id in processes:
            self._stop_process(process, process_group_id)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraWatchdog()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
