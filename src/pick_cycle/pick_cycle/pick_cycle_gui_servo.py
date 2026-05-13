import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import scrolledtext

import rclpy
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import TrayInterceptStart
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


ITEM_ARM_SERVICE_DEFAULT = 'item_pick_servo/track'
ITEM_ARM_STATUS_SERVICE_DEFAULT = 'item_pick_servo/track_status'
ITEM_SEEK_SERVICE_DEFAULT = 'item_detect/seek'
ITEM_SEEK_STATUS_SERVICE_DEFAULT = 'item_detect/seek_status'
TRAY_ARM_SERVICE_DEFAULT = 'tray_intercept_servo/start_sequence'
TRAY_ARM_STATUS_SERVICE_DEFAULT = 'tray_intercept_servo/track_status'
TRAY_SEEK_SERVICE_DEFAULT = 'tray_detect/seek'
TRAY_SEEK_STATUS_SERVICE_DEFAULT = 'tray_detect/seek_status'
ROBOT_TCP_TOPIC_DEFAULT = '/dobot_msgs_v4/msg/ToolVectorActual'

ROBOT_LINEAR_MOVE_EPS_MM = 1.0
ROBOT_ROT_MOVE_EPS_DEG = 1.0
ROBOT_STABILITY_SEC_DEFAULT = 0.5
TIMING_SLIDER_MIN_SEC = 0.1
TIMING_SLIDER_MAX_SEC = 1.0
TIMING_SLIDER_STEP_SEC = 0.1
ROBOT_TCP_STALE_SEC = 1.0
ROBOT_MONITOR_TIMEOUT_SEC = 30.0
MOVEMENT_TRACKER_START_LINEAR_MM = ROBOT_LINEAR_MOVE_EPS_MM
MOVEMENT_TRACKER_START_ROT_DEG = ROBOT_ROT_MOVE_EPS_DEG
MOVEMENT_TRACKER_LABEL_TTL_SEC = 90.0
MOVEMENT_TRACKER_LOG_PREFIX = 'Movement tracker'
SEEK_STATUS_POLL_SEC = 0.1
SEEK_STATUS_RESPONSE_TIMEOUT_SEC = 0.2
ARM_CLICK_RESPONSE_TIMEOUT_SEC = 5.5
ARM_STATUS_RESPONSE_TIMEOUT_SEC = 1.0
SERVICE_READY_TIMEOUT_SEC = 5.5
SERVICE_READY_POLL_SEC = 0.1
LOOP_PAUSE_SEC_DEFAULT = 1.0
TRAY_START_WAIT_TIMEOUT_SEC_DEFAULT = 60.0
TRAY_INTERCEPT_SPEED_MM_S_DEFAULT = 650.0
TRAY_STANDOFF_Z_MM_DEFAULT = 100.0
TRAY_FOLLOW_DISTANCE_MM_DEFAULT = 200.0
TRAY_POST_FOLLOW_Z_UP_MM_DEFAULT = 300.0
TRAY_INTERCEPT_X_OFFSET_MIN = -50.0
TRAY_INTERCEPT_X_OFFSET_MAX = 400.0
TRAY_INTERCEPT_Y_OFFSET_MIN = -50.0
TRAY_INTERCEPT_Y_OFFSET_MAX = 300.0
TRAY_EE_ANGLE_MIN_DEG = -90.0
TRAY_EE_ANGLE_MAX_DEG = 90.0
MOVEMENT_DELTA_DEBUG_DIRNAME = 'pick_cycle_movement_deltas'

ROBOT_STATUS_STOP = 'stop'
ROBOT_STATUS_PICKING = 'picking'
ROBOT_STATUS_PLACING = 'placing'
ROBOT_STATUS_PAUSE = 'pause'
ROBOT_STATUS_LABELS = {
    ROBOT_STATUS_STOP: 'Stop',
    ROBOT_STATUS_PICKING: 'Picking',
    ROBOT_STATUS_PLACING: 'Placing',
    ROBOT_STATUS_PAUSE: 'On Pause',
}


def _looks_like_workspace_root(path: Path) -> bool:
    return (
        (path / 'src').exists()
        and (
            (path / 'README.md').exists()
            or (path / 'docker-compose.yml').exists()
            or (path / 'src' / 'dobot_msgs_v4').exists()
        )
    )


def workspace_root() -> Path:
    for start in (Path(__file__).resolve(), Path.cwd()):
        path = start if start.is_dir() else start.parent
        for candidate in (path, *path.parents):
            if _looks_like_workspace_root(candidate):
                return candidate
    return Path.cwd().resolve()


def _safe_filename_fragment(text: str, max_length: int = 72) -> str:
    chars: list[str] = []
    previous_separator = False
    for ch in text:
        if ch.isalnum():
            chars.append(ch)
            previous_separator = False
        elif not previous_separator:
            chars.append('_')
            previous_separator = True
    fragment = ''.join(chars).strip('_')
    return (fragment[:max_length].strip('_') or 'movement_delta')


@dataclass(frozen=True)
class TriggerResult:
    success: bool
    message: str


@dataclass(frozen=True)
class SeekStatusResult:
    available: bool
    active: bool
    message: str


@dataclass(frozen=True)
class CycleConfig:
    loop_pause_sec: float
    loop_enabled: bool
    stability_sec: float
    tray_intercept_x_offset_mm: float
    tray_intercept_y_offset_mm: float
    tray_ee_angle_deg: float


class BackgroundMovementTracker:
    """Passive TCP-feedback-based travel timer.

    The tracker never waits for, blocks, or commands the robot.  It only observes
    ToolVectorActual feedback and emits a timing line after a physical movement
    has stopped for the configured stability window.
    """

    def __init__(self, stability_sec: float = ROBOT_STABILITY_SEC_DEFAULT) -> None:
        self._lock = threading.Lock()
        self._events: queue.Queue[str] = queue.Queue()
        self._stability_sec = max(0.0, float(stability_sec))
        self._last_pose: tuple[float, float, float, float, float, float] | None = None
        self._stable_anchor_pose: tuple[float, float, float, float, float, float] | None = None
        self._stable_since: float | None = None
        self._moving = False
        self._move_start_time = 0.0
        self._move_start_pose: tuple[float, float, float, float, float, float] | None = None
        self._active_labels: list[str] = []
        self._pending_labels: list[tuple[float, str]] = []

    def set_stability_sec(self, stability_sec: float) -> None:
        with self._lock:
            self._stability_sec = max(0.0, float(stability_sec))

    def mark_command(self, label: str) -> None:
        now = time.monotonic()
        clean_label = str(label).strip()
        if not clean_label:
            return
        with self._lock:
            if self._moving:
                self._active_labels.append(clean_label)
            else:
                self._pending_labels.append((now, clean_label))
                self._drop_stale_pending_locked(now)

    def cancel_command(self, label: str) -> None:
        clean_label = str(label).strip()
        if not clean_label:
            return
        with self._lock:
            self._pending_labels = [entry for entry in self._pending_labels if entry[1] != clean_label]
            if self._moving:
                self._active_labels = [entry for entry in self._active_labels if entry != clean_label]

    def update(self, pose: tuple[float, float, float, float, float, float], now: float) -> None:
        with self._lock:
            if self._last_pose is None:
                self._last_pose = pose
                self._stable_anchor_pose = pose
                self._stable_since = now
                return

            if not self._moving:
                anchor_pose = self._stable_anchor_pose or self._last_pose
                linear_delta, rot_delta = self._pose_delta(pose, anchor_pose)
                if linear_delta > MOVEMENT_TRACKER_START_LINEAR_MM or rot_delta > MOVEMENT_TRACKER_START_ROT_DEG:
                    self._moving = True
                    self._move_start_time = now
                    self._move_start_pose = anchor_pose
                    self._active_labels = self._consume_pending_labels_locked(now)
                    self._stable_anchor_pose = pose
                    self._stable_since = now
                else:
                    self._stable_anchor_pose = pose
                    self._stable_since = now
                self._last_pose = pose
                return

            anchor_pose = self._stable_anchor_pose or self._last_pose
            linear_delta, rot_delta = self._pose_delta(pose, anchor_pose)
            if linear_delta > ROBOT_LINEAR_MOVE_EPS_MM or rot_delta > ROBOT_ROT_MOVE_EPS_DEG:
                self._stable_anchor_pose = pose
                self._stable_since = now
            elif self._stable_since is not None and (now - self._stable_since) >= self._stability_sec:
                self._finish_movement_locked(pose, now)

            self._last_pose = pose

    def drain_events(self) -> list[str]:
        events: list[str] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        return events

    def _consume_pending_labels_locked(self, now: float) -> list[str]:
        self._drop_stale_pending_locked(now)
        labels = [label for _, label in self._pending_labels]
        self._pending_labels.clear()
        return labels

    def _drop_stale_pending_locked(self, now: float) -> None:
        cutoff = now - MOVEMENT_TRACKER_LABEL_TTL_SEC
        self._pending_labels = [(label_time, label) for label_time, label in self._pending_labels if label_time >= cutoff]

    def _finish_movement_locked(
        self,
        end_pose: tuple[float, float, float, float, float, float],
        end_time: float,
    ) -> None:
        start_pose = self._move_start_pose or self._last_pose or end_pose
        duration_sec = max(0.0, end_time - self._move_start_time)
        linear_delta, rot_delta = self._pose_delta(end_pose, start_pose)
        label = self._format_labels(self._active_labels)
        self._events.put(
            f'{MOVEMENT_TRACKER_LOG_PREFIX}: {label} travel took {duration_sec:.2f}s '
            f'(TCP delta {linear_delta:.1f}mm, {rot_delta:.1f}deg)'
        )
        self._moving = False
        self._move_start_time = 0.0
        self._move_start_pose = None
        self._active_labels = []
        self._stable_anchor_pose = end_pose
        self._stable_since = end_time

    @staticmethod
    def _format_labels(labels: list[str]) -> str:
        if not labels:
            return 'unlabeled robot movement'
        compact_labels: list[str] = []
        for label in labels:
            if not compact_labels or compact_labels[-1] != label:
                compact_labels.append(label)
        if len(compact_labels) <= 4:
            return ' -> '.join(compact_labels)
        return f'{compact_labels[0]} -> ... -> {compact_labels[-1]}'

    @staticmethod
    def _pose_delta(
        lhs: tuple[float, float, float, float, float, float],
        rhs: tuple[float, float, float, float, float, float],
    ) -> tuple[float, float]:
        linear_delta = (
            (lhs[0] - rhs[0]) ** 2 +
            (lhs[1] - rhs[1]) ** 2 +
            (lhs[2] - rhs[2]) ** 2
        ) ** 0.5
        rot_delta = max(
            BackgroundMovementTracker._angle_delta_deg(lhs[3], rhs[3]),
            BackgroundMovementTracker._angle_delta_deg(lhs[4], rhs[4]),
            BackgroundMovementTracker._angle_delta_deg(lhs[5], rhs[5]),
        )
        return linear_delta, rot_delta

    @staticmethod
    def _angle_delta_deg(lhs: float, rhs: float) -> float:
        return abs((float(lhs) - float(rhs) + 180.0) % 360.0 - 180.0)


class PickCycleNode(Node):
    def __init__(self) -> None:
        super().__init__('pick_cycle_gui_servo')
        self.item_arm_service = self._declare_name_parameter(
            'item_arm_service',
            ITEM_ARM_SERVICE_DEFAULT,
        )
        self.item_arm_status_service = self._declare_name_parameter(
            'item_arm_status_service',
            ITEM_ARM_STATUS_SERVICE_DEFAULT,
        )
        self.item_seek_service = self._declare_name_parameter(
            'item_seek_service',
            ITEM_SEEK_SERVICE_DEFAULT,
        )
        self.item_seek_status_service = self._declare_name_parameter(
            'item_seek_status_service',
            ITEM_SEEK_STATUS_SERVICE_DEFAULT,
        )
        self.tray_arm_service = self._declare_name_parameter(
            'tray_arm_service',
            TRAY_ARM_SERVICE_DEFAULT,
        )
        self.tray_arm_status_service = self._declare_name_parameter(
            'tray_arm_status_service',
            TRAY_ARM_STATUS_SERVICE_DEFAULT,
        )
        self.tray_seek_service = self._declare_name_parameter(
            'tray_seek_service',
            TRAY_SEEK_SERVICE_DEFAULT,
        )
        self.tray_seek_status_service = self._declare_name_parameter(
            'tray_seek_status_service',
            TRAY_SEEK_STATUS_SERVICE_DEFAULT,
        )
        self.robot_tcp_topic = self._declare_name_parameter(
            'robot_tcp_topic',
            ROBOT_TCP_TOPIC_DEFAULT,
        )
        self._tray_arm_client = self.create_client(TrayInterceptStart, self.tray_arm_service)

        self._trigger_clients: dict[str, tuple[str, object]] = {
            'item_arm': (
                self.item_arm_service,
                self.create_client(Trigger, self.item_arm_service),
            ),
            'item_arm_status': (
                self.item_arm_status_service,
                self.create_client(Trigger, self.item_arm_status_service),
            ),
            'item_seek': (
                self.item_seek_service,
                self.create_client(Trigger, self.item_seek_service),
            ),
            'item_seek_status': (
                self.item_seek_status_service,
                self.create_client(Trigger, self.item_seek_status_service),
            ),
            'tray_arm_status': (
                self.tray_arm_status_service,
                self.create_client(Trigger, self.tray_arm_status_service),
            ),
            'tray_seek': (
                self.tray_seek_service,
                self.create_client(Trigger, self.tray_seek_service),
            ),
            'tray_seek_status': (
                self.tray_seek_status_service,
                self.create_client(Trigger, self.tray_seek_status_service),
            ),
        }
        self._startup_service_result: TriggerResult | None = None
        self._startup_service_lock = threading.Lock()
        self._tcp_condition = threading.Condition()
        self._tcp_seq = 0
        self._latest_tcp: tuple[float, float, float, float, float, float] | None = None
        self._last_tcp_receive_time = 0.0
        self._movement_tracker = BackgroundMovementTracker()

        self.create_subscription(ToolVectorActual, self.robot_tcp_topic, self._tcp_callback, 10)

    def _declare_name_parameter(self, parameter_name: str, default_value: str) -> str:
        value = str(self.declare_parameter(parameter_name, default_value).value).strip()
        return value or default_value

    def _required_service_clients(self) -> list[tuple[str, object]]:
        return [*self._trigger_clients.values(), (self.tray_arm_service, self._tray_arm_client)]

    def service_names(self) -> list[tuple[str, str]]:
        return [
            ('Item Pick Arm', self.item_arm_service),
            ('Item Pick Arm Status', self.item_arm_status_service),
            ('Item Seek', self.item_seek_service),
            ('Item Seek Status', self.item_seek_status_service),
            ('Tray Intercept Arm Start', self.tray_arm_service),
            ('Tray Intercept Arm Status', self.tray_arm_status_service),
            ('Tray Seek', self.tray_seek_service),
            ('Tray Seek Status', self.tray_seek_status_service),
        ]

    def topic_names(self) -> list[tuple[str, str]]:
        return [('Robot Feedback', self.robot_tcp_topic)]

    def set_movement_tracker_stability_sec(self, stability_sec: float) -> None:
        self._movement_tracker.set_stability_sec(stability_sec)

    def mark_movement_command(self, label: str) -> None:
        self._movement_tracker.mark_command(label)

    def cancel_movement_command(self, label: str) -> None:
        self._movement_tracker.cancel_command(label)

    def drain_movement_events(self) -> list[str]:
        return self._movement_tracker.drain_events()

    def verify_trigger_services(
        self,
        stop_event: threading.Event | None = None,
        timeout_sec: float = SERVICE_READY_TIMEOUT_SEC,
    ) -> TriggerResult:
        with self._startup_service_lock:
            cached_result = self._startup_service_result
        if cached_result is not None:
            return cached_result

        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        missing_names: list[str] = []

        while rclpy.ok():
            if stop_event is not None and stop_event.is_set():
                result = TriggerResult(False, 'Stopped while checking required services')
                self._cache_startup_service_result(result)
                return result

            missing_names = [
                service_name
                for service_name, client in self._required_service_clients()
                if not client.service_is_ready()
            ]
            if not missing_names:
                result = TriggerResult(True, f'All {len(self._required_service_clients())} services ready')
                self._cache_startup_service_result(result)
                return result

            now = time.monotonic()
            if now >= deadline:
                break
            time.sleep(min(SERVICE_READY_POLL_SEC, max(0.0, deadline - now)))

        if not missing_names:
            missing_names = [
                service_name
                for service_name, client in self._required_service_clients()
                if not client.service_is_ready()
            ]
        missing = ', '.join(missing_names)
        result = TriggerResult(
            False,
            f'Required services unavailable after {timeout_sec:.1f}s: {missing}',
        )
        self._cache_startup_service_result(result)
        return result

    def startup_service_result(self) -> TriggerResult | None:
        with self._startup_service_lock:
            return self._startup_service_result

    def check_trigger_services_now(self) -> TriggerResult:
        missing_names = [
            service_name
            for service_name, client in self._required_service_clients()
            if not client.service_is_ready()
        ]
        if missing_names:
            return TriggerResult(False, 'Required services unavailable: ' + ', '.join(missing_names))
        return TriggerResult(True, f'All {len(self._required_service_clients())} services ready')

    def _cache_startup_service_result(self, result: TriggerResult) -> None:
        with self._startup_service_lock:
            if self._startup_service_result is None:
                self._startup_service_result = result

    def _require_startup_services(self) -> TriggerResult:
        result = self.startup_service_result()
        if result is None:
            return TriggerResult(False, 'Startup service check has not completed')
        if not result.success:
            return TriggerResult(False, f'Startup service check failed: {result.message}')
        return result

    def robot_pose_snapshot(self) -> tuple[float, float, float, float, float, float] | None:
        with self._tcp_condition:
            return self._latest_tcp

    def wait_for_robot_stable(self, stop_event: threading.Event, stability_sec: float) -> TriggerResult:
        stability_sec = max(0.0, float(stability_sec))
        deadline = time.monotonic() + ROBOT_MONITOR_TIMEOUT_SEC
        stable_anchor_pose: tuple[float, float, float, float, float, float] | None = None
        stable_since: float | None = None
        stable_elapsed = 0.0
        last_seq = -1
        last_linear_delta = 0.0
        last_rot_delta = 0.0
        with self._tcp_condition:
            while rclpy.ok():
                now = time.monotonic()
                if self._latest_tcp is not None and (now - self._last_tcp_receive_time) <= ROBOT_TCP_STALE_SEC:
                    if stable_anchor_pose is None:
                        stable_anchor_pose = self._latest_tcp
                        stable_since = self._last_tcp_receive_time
                        last_seq = self._tcp_seq
                    elif self._tcp_seq != last_seq:
                        linear_delta, rot_delta = self._pose_delta(self._latest_tcp, stable_anchor_pose)
                        last_linear_delta = linear_delta
                        last_rot_delta = rot_delta
                        if linear_delta > ROBOT_LINEAR_MOVE_EPS_MM or rot_delta > ROBOT_ROT_MOVE_EPS_DEG:
                            stable_anchor_pose = self._latest_tcp
                            stable_since = self._last_tcp_receive_time
                            last_linear_delta = 0.0
                            last_rot_delta = 0.0
                        last_seq = self._tcp_seq

                    if stable_since is not None:
                        stable_elapsed = max(0.0, self._last_tcp_receive_time - stable_since)
                    if stable_since is not None and stable_elapsed >= stability_sec:
                        return TriggerResult(
                            True,
                            'Robot TCP stable for '
                            f'{stable_elapsed:.2f}s on {self.robot_tcp_topic} '
                            f'(window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)',
                        )
                if stop_event.is_set():
                    return TriggerResult(False, 'Stopped while monitoring robot stability')
                if now >= deadline:
                    if self._latest_tcp is None:
                        return TriggerResult(False, f'No TCP feedback received on {self.robot_tcp_topic}')
                    tcp_age = now - self._last_tcp_receive_time
                    if tcp_age > ROBOT_TCP_STALE_SEC:
                        return TriggerResult(
                            False,
                            f'TCP feedback stale on {self.robot_tcp_topic}: last update {tcp_age:.2f}s ago',
                        )
                    return TriggerResult(
                        False,
                        'Robot did not become stable within '
                        f'{ROBOT_MONITOR_TIMEOUT_SEC:.1f}s '
                        f'(stable time {stable_elapsed:.2f}/{stability_sec:.2f}s, '
                        f'window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)',
                    )
                self._tcp_condition.wait(timeout=0.1)
        return TriggerResult(False, f'ROS shutdown while monitoring {self.robot_tcp_topic}')

    def click_trigger(self, client_key: str, wait_response_sec: float | None = None) -> TriggerResult:
        service_name, client = self._trigger_clients[client_key]
        startup_result = self._require_startup_services()
        if not startup_result.success:
            return startup_result
        if not client.service_is_ready():
            return TriggerResult(False, f'Service unavailable: {service_name}')

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            return TriggerResult(False, f'Failed to send trigger to {service_name}: {exc}')
        if wait_response_sec is None:
            return TriggerResult(True, f'Sent trigger to {service_name}')

        deadline = time.monotonic() + max(0.0, float(wait_response_sec))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return TriggerResult(False, f'Timed out waiting for {service_name} response')

        try:
            response = future.result()
        except Exception as exc:
            return TriggerResult(False, f'{service_name} response failed: {exc}')

        if response is None:
            return TriggerResult(False, f'{service_name} returned no response')
        return TriggerResult(bool(response.success), f'{service_name}: {response.message}')

    def start_tray_intercept(
        self,
        config: CycleConfig,
        wait_response_sec: float = ARM_CLICK_RESPONSE_TIMEOUT_SEC,
    ) -> TriggerResult:
        service_name = self.tray_arm_service
        client = self._tray_arm_client
        startup_result = self._require_startup_services()
        if not startup_result.success:
            return startup_result
        if not client.service_is_ready():
            return TriggerResult(False, f'Service unavailable: {service_name}')

        request = TrayInterceptStart.Request()
        request.tray_vector_wait_timeout_sec = TRAY_START_WAIT_TIMEOUT_SEC_DEFAULT
        request.ee_intercept_speed_mm_s = TRAY_INTERCEPT_SPEED_MM_S_DEFAULT
        request.tray_intercept_x_offset_mm = float(config.tray_intercept_x_offset_mm)
        request.tray_intercept_y_offset_mm = float(config.tray_intercept_y_offset_mm)
        request.ee_final_pose_angle_deg = float(config.tray_ee_angle_deg)
        request.tray_standoff_z_mm = TRAY_STANDOFF_Z_MM_DEFAULT
        request.follow_distance_mm = TRAY_FOLLOW_DISTANCE_MM_DEFAULT
        request.post_follow_z_up_mm = TRAY_POST_FOLLOW_Z_UP_MM_DEFAULT
        request.troubleshoot_tf_only = False

        try:
            future = client.call_async(request)
        except Exception as exc:
            return TriggerResult(False, f'Failed to start tray intercept via {service_name}: {exc}')

        deadline = time.monotonic() + max(0.0, float(wait_response_sec))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return TriggerResult(False, f'Timed out waiting for {service_name} response')

        try:
            response = future.result()
        except Exception as exc:
            return TriggerResult(False, f'{service_name} response failed: {exc}')

        if response is None:
            return TriggerResult(False, f'{service_name} returned no response')
        applied = (
            f'x={float(response.applied_tray_intercept_x_offset_mm):.0f}mm, '
            f'y={float(response.applied_tray_intercept_y_offset_mm):.0f}mm, '
            f'rz={float(response.applied_ee_final_pose_angle_deg):.0f}deg'
        )
        return TriggerResult(
            bool(response.started),
            f'{service_name}: {response.message} ({applied})',
        )

    def read_seek_status(
        self,
        client_key: str,
        timeout_sec: float = SEEK_STATUS_RESPONSE_TIMEOUT_SEC,
    ) -> SeekStatusResult:
        service_name, client = self._trigger_clients[client_key]
        startup_result = self._require_startup_services()
        if not startup_result.success:
            return SeekStatusResult(False, False, startup_result.message)
        if not client.service_is_ready():
            return SeekStatusResult(False, False, f'Service unavailable: {service_name}')

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            return SeekStatusResult(False, False, f'Failed to send status request to {service_name}: {exc}')
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return SeekStatusResult(False, False, f'Timed out waiting for {service_name} response')

        try:
            response = future.result()
        except Exception as exc:
            return SeekStatusResult(False, False, f'{service_name} response failed: {exc}')

        if response is None:
            return SeekStatusResult(False, False, f'{service_name} returned no response')
        return SeekStatusResult(True, bool(response.success), f'{service_name}: {response.message}')

    def read_trigger_status(self, client_key: str, timeout_sec: float = 1.0) -> SeekStatusResult:
        service_name, client = self._trigger_clients[client_key]
        startup_result = self._require_startup_services()
        if not startup_result.success:
            return SeekStatusResult(False, False, startup_result.message)
        if not client.service_is_ready():
            return SeekStatusResult(False, False, f'Service unavailable: {service_name}')

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            return SeekStatusResult(False, False, f'Failed to send status request to {service_name}: {exc}')
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return SeekStatusResult(False, False, f'Timed out waiting for {service_name} response')

        try:
            response = future.result()
        except Exception as exc:
            return SeekStatusResult(False, False, f'{service_name} response failed: {exc}')

        if response is None:
            return SeekStatusResult(False, False, f'{service_name} returned no response')
        return SeekStatusResult(True, bool(response.success), f'{service_name}: {response.message}')

    def _tcp_callback(self, msg: ToolVectorActual) -> None:
        tcp = (
            float(msg.x),
            float(msg.y),
            float(msg.z),
            float(msg.rx),
            float(msg.ry),
            float(msg.rz),
        )
        now = time.monotonic()
        with self._tcp_condition:
            self._tcp_seq += 1
            self._latest_tcp = tcp
            self._last_tcp_receive_time = now
            self._tcp_condition.notify_all()
        self._movement_tracker.update(tcp, now)

    @staticmethod
    def _pose_delta(
        lhs: tuple[float, float, float, float, float, float],
        rhs: tuple[float, float, float, float, float, float],
    ) -> tuple[float, float]:
        linear_delta = (
            (lhs[0] - rhs[0]) ** 2 +
            (lhs[1] - rhs[1]) ** 2 +
            (lhs[2] - rhs[2]) ** 2
        ) ** 0.5
        rot_delta = max(
            PickCycleNode._angle_delta_deg(lhs[3], rhs[3]),
            PickCycleNode._angle_delta_deg(lhs[4], rhs[4]),
            PickCycleNode._angle_delta_deg(lhs[5], rhs[5]),
        )
        return linear_delta, rot_delta

    @staticmethod
    def _angle_delta_deg(lhs: float, rhs: float) -> float:
        return abs((float(lhs) - float(rhs) + 180.0) % 360.0 - 180.0)


class PickCycleGui:
    def __init__(self, node: PickCycleNode, startup_result: TriggerResult) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title('Pick Cycle Mini GUI')
        self.root.geometry('760x590')
        self.root.minsize(720, 540)
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._worker_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._cycle_count = 0
        self._robot_status = ROBOT_STATUS_STOP
        self._startup_services_ok = bool(startup_result.success)
        self._movement_delta_debug_dir = workspace_root() / 'debug files' / MOVEMENT_DELTA_DEBUG_DIRNAME
        self._movement_delta_cycle_files: dict[int, Path] = {}
        self._movement_delta_cycle_started_at: dict[int, str] = {}
        self._movement_delta_cycle_events: dict[int, list[str]] = {}

        self.status_var = tk.StringVar(
            value='Idle' if self._startup_services_ok else 'Startup service check failed'
        )
        self.robot_status_var = tk.StringVar(value=ROBOT_STATUS_LABELS[self._robot_status])
        self.loop_var = tk.BooleanVar(value=False)
        self.loop_pause_var = tk.DoubleVar(value=LOOP_PAUSE_SEC_DEFAULT)
        self.stability_sec_var = tk.DoubleVar(value=ROBOT_STABILITY_SEC_DEFAULT)
        self.tray_intercept_x_var = tk.DoubleVar(value=0.0)
        self.tray_intercept_y_var = tk.DoubleVar(value=0.0)
        self.tray_ee_angle_var = tk.DoubleVar(value=0.0)

        outer = tk.Frame(self.root, padx=12, pady=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(4, weight=1)

        controls = tk.LabelFrame(outer, text='Cycle Controls', padx=10, pady=10)
        controls.grid(row=0, column=0, sticky='ew')
        controls.columnconfigure(1, weight=1)
        controls.columnconfigure(3, weight=1)

        self.start_button = tk.Button(
            controls,
            text='Start Cycle',
            command=self._start_clicked,
            width=18,
            state=tk.NORMAL if self._startup_services_ok else tk.DISABLED,
        )
        self.start_button.grid(row=0, column=0, sticky='ew', padx=(0, 8))
        self.stop_button = tk.Button(controls, text='Stop', command=self._stop_clicked, width=12, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1, sticky='w')
        self.loop_check = tk.Checkbutton(controls, text='Loop after successful cycle', variable=self.loop_var)
        self.loop_check.grid(row=0, column=2, columnspan=2, sticky='w', padx=(14, 0))

        tk.Label(controls, text='Stability time (s)').grid(row=1, column=0, sticky='w', pady=(10, 0))
        self.stability_sec_scale = tk.Scale(
            controls,
            from_=TIMING_SLIDER_MIN_SEC,
            to=TIMING_SLIDER_MAX_SEC,
            resolution=TIMING_SLIDER_STEP_SEC,
            orient=tk.HORIZONTAL,
            variable=self.stability_sec_var,
            length=170,
        )
        self.stability_sec_scale.grid(row=1, column=1, sticky='ew', pady=(4, 0))

        tk.Label(controls, text='Loop pause (s)').grid(row=2, column=0, sticky='w', pady=(8, 0))
        self.loop_pause_spinbox = tk.Spinbox(
            controls,
            from_=0.0,
            to=60.0,
            increment=0.5,
            textvariable=self.loop_pause_var,
            width=8,
            format='%.1f',
        )
        self.loop_pause_spinbox.grid(row=2, column=1, sticky='w', pady=(8, 0))

        tray_settings = tk.LabelFrame(controls, text='Tray Arm Settings', padx=8, pady=6)
        tray_settings.grid(row=3, column=0, columnspan=4, sticky='ew', pady=(10, 0))
        for column in range(3):
            tray_settings.columnconfigure(column, weight=1)

        tk.Label(tray_settings, text='X offset (mm)').grid(row=0, column=0, sticky='w')
        self.tray_intercept_x_spinbox = tk.Spinbox(
            tray_settings,
            from_=TRAY_INTERCEPT_X_OFFSET_MIN,
            to=TRAY_INTERCEPT_X_OFFSET_MAX,
            increment=5.0,
            textvariable=self.tray_intercept_x_var,
            width=8,
            format='%.1f',
        )
        self.tray_intercept_x_spinbox.grid(row=1, column=0, sticky='w', padx=(0, 10))

        tk.Label(tray_settings, text='Y offset (mm)').grid(row=0, column=1, sticky='w')
        self.tray_intercept_y_spinbox = tk.Spinbox(
            tray_settings,
            from_=TRAY_INTERCEPT_Y_OFFSET_MIN,
            to=TRAY_INTERCEPT_Y_OFFSET_MAX,
            increment=5.0,
            textvariable=self.tray_intercept_y_var,
            width=8,
            format='%.1f',
        )
        self.tray_intercept_y_spinbox.grid(row=1, column=1, sticky='w', padx=(0, 10))

        tk.Label(tray_settings, text='RZ angle (deg)').grid(row=0, column=2, sticky='w')
        self.tray_ee_angle_spinbox = tk.Spinbox(
            tray_settings,
            from_=TRAY_EE_ANGLE_MIN_DEG,
            to=TRAY_EE_ANGLE_MAX_DEG,
            increment=1.0,
            textvariable=self.tray_ee_angle_var,
            width=8,
            format='%.1f',
        )
        self.tray_ee_angle_spinbox.grid(row=1, column=2, sticky='w')

        status_frame = tk.LabelFrame(outer, text='Status', padx=10, pady=8)
        status_frame.grid(row=1, column=0, sticky='ew', pady=(10, 0))
        status_frame.columnconfigure(1, weight=1)
        tk.Label(status_frame, text='Robot Status').grid(row=0, column=0, sticky='w')
        tk.Label(status_frame, textvariable=self.robot_status_var, anchor='w').grid(
            row=0,
            column=1,
            sticky='ew',
            padx=(12, 0),
        )
        tk.Label(status_frame, text='Cycle Status').grid(row=1, column=0, sticky='w', pady=(4, 0))
        tk.Label(status_frame, textvariable=self.status_var, anchor='w').grid(
            row=1,
            column=1,
            sticky='ew',
            padx=(12, 0),
            pady=(4, 0),
        )

        services_frame = tk.LabelFrame(outer, text='Services', padx=10, pady=8)
        services_frame.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        services_frame.columnconfigure(1, weight=1)
        for row, (label, service_name) in enumerate(self.node.service_names()):
            tk.Label(services_frame, text=label).grid(row=row, column=0, sticky='w')
            tk.Label(services_frame, text=service_name, anchor='w').grid(row=row, column=1, sticky='ew', padx=(12, 0))

        topics_frame = tk.LabelFrame(outer, text='Watched Topics', padx=10, pady=8)
        topics_frame.grid(row=3, column=0, sticky='ew', pady=(10, 0))
        topics_frame.columnconfigure(1, weight=1)
        for row, (label, topic_name) in enumerate(self.node.topic_names()):
            tk.Label(topics_frame, text=label).grid(row=row, column=0, sticky='w')
            tk.Label(topics_frame, text=topic_name, anchor='w').grid(row=row, column=1, sticky='ew', padx=(12, 0))

        log_frame = tk.LabelFrame(outer, text='Cycle Log', padx=10, pady=8)
        log_frame.grid(row=4, column=0, sticky='nsew', pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, state=tk.DISABLED)
        self.log_text.grid(row=0, column=0, sticky='nsew')

        self._log(
            'Startup service check: '
            f'{"OK" if startup_result.success else "FAIL"} - {startup_result.message}'
        )
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(100, self._drain_queue)

    def _start_clicked(self) -> None:
        if self._running:
            return
        config = self._read_config()
        self.node.set_movement_tracker_stability_sec(config.stability_sec)
        self._stop_event.clear()
        self._running = True
        self._set_running_controls(True)
        self._worker_thread = threading.Thread(target=self._run_worker, args=(config,), daemon=True)
        self._worker_thread.start()

    def _stop_clicked(self) -> None:
        self._stop_event.set()
        self._set_robot_status(ROBOT_STATUS_STOP)
        self._set_status('Stopping after current monitor step...')

    @staticmethod
    def _read_clamped_var(var: tk.DoubleVar, minimum: float, maximum: float) -> float:
        value = max(float(minimum), min(float(maximum), float(var.get())))
        var.set(value)
        return value

    def _read_config(self) -> CycleConfig:
        return CycleConfig(
            loop_pause_sec=max(0.0, float(self.loop_pause_var.get())),
            loop_enabled=bool(self.loop_var.get()),
            stability_sec=self._read_timing_slider(self.stability_sec_var),
            tray_intercept_x_offset_mm=self._read_clamped_var(
                self.tray_intercept_x_var,
                TRAY_INTERCEPT_X_OFFSET_MIN,
                TRAY_INTERCEPT_X_OFFSET_MAX,
            ),
            tray_intercept_y_offset_mm=self._read_clamped_var(
                self.tray_intercept_y_var,
                TRAY_INTERCEPT_Y_OFFSET_MIN,
                TRAY_INTERCEPT_Y_OFFSET_MAX,
            ),
            tray_ee_angle_deg=self._read_clamped_var(
                self.tray_ee_angle_var,
                TRAY_EE_ANGLE_MIN_DEG,
                TRAY_EE_ANGLE_MAX_DEG,
            ),
        )

    def _run_worker(self, config: CycleConfig) -> None:
        try:
            startup_result = self.node.startup_service_result()
            if startup_result is None or not startup_result.success:
                message = (
                    startup_result.message
                    if startup_result is not None
                    else 'Startup service check has not completed'
                )
                self._log(f'Cycle refused: startup service check did not pass - {message}')
                self._queue.put(('finished', None))
                return

            while rclpy.ok() and not self._stop_event.is_set():
                self._cycle_count += 1
                cycle_index = self._cycle_count
                self._log(f'=== Cycle {cycle_index} start ===')
                self._set_status(f'Cycle {cycle_index}: checking required ROS services')
                service_check = self.node.check_trigger_services_now()
                self._log(
                    f'[{cycle_index}] Service health check: '
                    f'{"OK" if service_check.success else "FAIL"} - {service_check.message}'
                )
                if not service_check.success:
                    self._log(f'=== Cycle {cycle_index} stopped/failed ===')
                    break
                success = self._run_one_cycle(config, cycle_index)
                if not success:
                    self._log(f'=== Cycle {cycle_index} stopped/failed ===')
                    break
                self._log(f'=== Cycle {cycle_index} done ===')
                self._set_robot_status(ROBOT_STATUS_STOP)
                if not config.loop_enabled:
                    break
                if not self._sleep_with_stop(config.loop_pause_sec, 'Loop pause'):
                    break
            self._queue.put(('finished', None))
        except Exception as exc:
            self._log(f'Cycle worker crashed: {exc}')
            self._queue.put(('finished', None))

    def _run_one_cycle(self, config: CycleConfig, cycle_index: int) -> bool:
        self._set_robot_status(ROBOT_STATUS_PICKING)
        if not self._seek_step(
            cycle_index,
            'Arm item pick',
            'item_arm',
            'item_arm_status',
            'Seek item detect',
            'item_seek',
            'item_seek_status',
            config,
        ):
            return False

        self._set_robot_status(ROBOT_STATUS_PLACING)
        return self._seek_step(
            cycle_index,
            'Arm tray intercept',
            'tray_arm',
            'tray_arm_status',
            'Seek tray detect',
            'tray_seek',
            'tray_seek_status',
            config,
        )

    def _click_step(
        self,
        cycle_index: int,
        label: str,
        client_key: str,
        wait_response_sec: float | None = None,
    ) -> bool:
        self._set_status(f'Cycle {cycle_index}: {label}')
        self._log(f'[{cycle_index}] {label}...')
        movement_label = f'Cycle {cycle_index}: {label}'
        self.node.mark_movement_command(movement_label)
        result = self.node.click_trigger(client_key, wait_response_sec)
        if not result.success:
            self.node.cancel_movement_command(movement_label)
        success_label = 'OK' if wait_response_sec is not None else 'SENT'
        self._log(f'[{cycle_index}] {label}: {success_label if result.success else "FAIL"} - {result.message}')
        return result.success and not self._stop_event.is_set()

    def _seek_step(
        self,
        cycle_index: int,
        arm_label: str,
        arm_client_key: str,
        arm_status_client_key: str,
        seek_label: str,
        seek_client_key: str,
        seek_status_client_key: str,
        config: CycleConfig,
    ) -> bool:
        if not self._arm_and_verify(cycle_index, arm_label, arm_client_key, arm_status_client_key, config):
            return False
        self._log(f'[{cycle_index}] {arm_label}: armed; confirming robot stability before seek...')

        self._set_status(f'Cycle {cycle_index}: confirming robot stability before {seek_label}')
        pre_seek_result = self.node.wait_for_robot_stable(self._stop_event, config.stability_sec)
        self._log(
            f'[{cycle_index}] {seek_label}: '
            f'{"STABLE" if pre_seek_result.success else "FAIL"} before seek - {pre_seek_result.message}'
        )
        if not pre_seek_result.success or self._stop_event.is_set():
            return False

        if not self._click_step(cycle_index, seek_label, seek_client_key):
            return False
        return self._wait_for_seek_on_then_off(cycle_index, seek_label, seek_status_client_key)

    def _arm_and_verify(
        self,
        cycle_index: int,
        arm_label: str,
        arm_client_key: str,
        arm_status_client_key: str,
        config: CycleConfig,
    ) -> bool:
        if arm_client_key == 'tray_arm':
            self._set_status(f'Cycle {cycle_index}: {arm_label}')
            self._log(
                f'[{cycle_index}] {arm_label}: '
                f'x={config.tray_intercept_x_offset_mm:.0f}mm, '
                f'y={config.tray_intercept_y_offset_mm:.0f}mm, '
                f'rz={config.tray_ee_angle_deg:.0f}deg...'
            )
            movement_label = f'Cycle {cycle_index}: {arm_label}'
            self.node.mark_movement_command(movement_label)
            result = self.node.start_tray_intercept(config, ARM_CLICK_RESPONSE_TIMEOUT_SEC)
            if not result.success:
                self.node.cancel_movement_command(movement_label)
            self._log(f'[{cycle_index}] {arm_label}: {"OK" if result.success else "FAIL"} - {result.message}')
            if not result.success or self._stop_event.is_set():
                return False
        else:
            if not self._click_step(
                cycle_index,
                arm_label,
                arm_client_key,
                wait_response_sec=ARM_CLICK_RESPONSE_TIMEOUT_SEC,
            ):
                return False

        self._set_status(f'Cycle {cycle_index}: verifying {arm_label} armed status')
        status = self.node.read_trigger_status(
            arm_status_client_key,
            timeout_sec=ARM_STATUS_RESPONSE_TIMEOUT_SEC,
        )
        if not status.available:
            self._log(f'[{cycle_index}] {arm_label}: FAIL - {status.message}')
            return False
        if not status.active:
            self._log(f'[{cycle_index}] {arm_label}: FAIL - not armed - {status.message}')
            return False

        self._log(f'[{cycle_index}] {arm_label}: ARMED - {status.message}')
        return not self._stop_event.is_set()

    @staticmethod
    def _status_request_timed_out(status: SeekStatusResult) -> bool:
        return status.message.startswith('Timed out waiting for ')

    def _wait_for_seek_on_then_off(self, cycle_index: int, seek_label: str, seek_status_client_key: str) -> bool:
        self._set_status(f'Cycle {cycle_index}: waiting for {seek_label} to toggle ON')
        self._log(
            f'[{cycle_index}] {seek_label}: command sent; waiting for detect Seek button to turn ON, '
            'then OFF (no GUI seek timeout)...'
        )
        last_log_time = 0.0
        seen_on = False

        while rclpy.ok():
            if self._stop_event.is_set():
                self._log(f'[{cycle_index}] {seek_label}: stopped while waiting for seek status.')
                return False

            status = self.node.read_seek_status(seek_status_client_key)
            if not status.available:
                if self._status_request_timed_out(status):
                    now = time.monotonic()
                    if now - last_log_time >= 1.0:
                        self._log(f'[{cycle_index}] {seek_label}: status delayed - {status.message}')
                        last_log_time = now
                    time.sleep(SEEK_STATUS_POLL_SEC)
                    continue
                self._log(f'[{cycle_index}] {seek_label}: FAIL - {status.message}')
                return False

            if not seen_on:
                if status.active:
                    seen_on = True
                    last_log_time = 0.0
                    self._set_status(f'Cycle {cycle_index}: waiting for {seek_label} to toggle OFF')
                    self._log(f'[{cycle_index}] {seek_label}: ON - {status.message}')
                    time.sleep(SEEK_STATUS_POLL_SEC)
                    continue

                now = time.monotonic()
                if now - last_log_time >= 1.0:
                    self._log(f'[{cycle_index}] {seek_label}: waiting for ON - {status.message}')
                    last_log_time = now
                time.sleep(SEEK_STATUS_POLL_SEC)
                continue

            if not status.active:
                self._log(f'[{cycle_index}] {seek_label}: OFF - {status.message}')
                return True

            now = time.monotonic()
            if now - last_log_time >= 1.0:
                self._log(f'[{cycle_index}] {seek_label}: still ON - {status.message}')
                last_log_time = now

            time.sleep(SEEK_STATUS_POLL_SEC)

        return False

    def _read_timing_slider(self, var: tk.DoubleVar) -> float:
        value = float(var.get())
        value = round(value / TIMING_SLIDER_STEP_SEC) * TIMING_SLIDER_STEP_SEC
        value = min(TIMING_SLIDER_MAX_SEC, max(TIMING_SLIDER_MIN_SEC, value))
        var.set(value)
        return value

    def _sleep_with_stop(self, duration_sec: float, label: str) -> bool:
        if duration_sec <= 0.0:
            return not self._stop_event.is_set()
        self._set_status(f'{label}: {duration_sec:.1f}s')
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            time.sleep(0.05)
        return True

    def _set_running_controls(self, running: bool) -> None:
        start_state = tk.DISABLED if running or not self._startup_services_ok else tk.NORMAL
        self.start_button.configure(state=start_state)
        self.stop_button.configure(state=tk.NORMAL if running else tk.DISABLED)
        self.loop_check.configure(state=tk.DISABLED if running else tk.NORMAL)
        self.stability_sec_scale.configure(state=tk.DISABLED if running else tk.NORMAL)
        setting_state = tk.DISABLED if running else tk.NORMAL
        self.loop_pause_spinbox.configure(state=setting_state)
        self.tray_intercept_x_spinbox.configure(state=setting_state)
        self.tray_intercept_y_spinbox.configure(state=setting_state)
        self.tray_ee_angle_spinbox.configure(state=setting_state)

    def _queue_call(self, action: str, payload: object) -> None:
        self._queue.put((action, payload))

    def _set_status(self, text: str) -> None:
        self._queue_call('status', text)

    def _set_robot_status(self, status: str) -> None:
        self._robot_status = status
        self._queue_call('robot_status', status)

    def _log(self, text: str) -> None:
        timestamp = time.strftime('%H:%M:%S')
        self._queue_call('log', f'{timestamp}  {text}')
        self.node.get_logger().info(text)

    def _drain_queue(self) -> None:
        try:
            while True:
                action, payload = self._queue.get_nowait()
                if action == 'status':
                    self.status_var.set(str(payload))
                elif action == 'robot_status':
                    status = str(payload)
                    self.robot_status_var.set(ROBOT_STATUS_LABELS.get(status, status))
                elif action == 'log':
                    self._append_log(str(payload))
                elif action == 'finished':
                    self._running = False
                    self._set_running_controls(False)
                    self._set_robot_status(ROBOT_STATUS_STOP)
                    self.status_var.set('Idle' if not self._stop_event.is_set() else 'Stopped')
        except queue.Empty:
            pass

        for movement_event in self.node.drain_movement_events():
            timestamp = time.strftime('%H:%M:%S')
            self._append_log(f'{timestamp}  {movement_event}')
            self.node.get_logger().info(movement_event)
            self._write_cycle_movement_delta_debug_file(movement_event)

        self.root.after(100, self._drain_queue)

    def _write_cycle_movement_delta_debug_file(self, movement_event: str) -> None:
        cycle_index = self._cycle_index_from_movement_event(movement_event)
        wall_timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        if cycle_index not in self._movement_delta_cycle_files:
            filename_timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename_label = _safe_filename_fragment(f'cycle_{cycle_index}')
            filename = f'{filename_timestamp}_{filename_label}_movement_deltas.txt'
            self._movement_delta_cycle_files[cycle_index] = self._movement_delta_debug_dir / filename
            self._movement_delta_cycle_started_at[cycle_index] = wall_timestamp
            self._movement_delta_cycle_events[cycle_index] = []

        self._movement_delta_cycle_events[cycle_index].append(f'{wall_timestamp}  {movement_event}')
        path = self._movement_delta_cycle_files[cycle_index]
        try:
            self._movement_delta_debug_dir.mkdir(parents=True, exist_ok=True)
            path.write_text(
                '\n'.join([
                    'pick_cycle cycle movement delta debug',
                    f'cycle: {cycle_index}',
                    f'cycle_file_started_at: {self._movement_delta_cycle_started_at[cycle_index]}',
                    f'last_updated_at: {wall_timestamp}',
                    f'node: {self.node.get_name()}',
                    '',
                    'events:',
                    *self._movement_delta_cycle_events[cycle_index],
                    '',
                ]),
                encoding='utf-8',
            )
        except Exception as exc:
            self.node.get_logger().warn(f'Failed to write cycle movement delta debug file "{path}": {exc}')

    @staticmethod
    def _cycle_index_from_movement_event(movement_event: str) -> int:
        match = re.search(r'\bCycle\s+(\d+)\b', movement_event)
        if match:
            return int(match.group(1))
        return 0

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + '\n')
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _on_close(self) -> None:
        self._stop_event.set()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickCycleNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    startup_result = node.verify_trigger_services(stop_event)
    gui = PickCycleGui(node, startup_result)
    try:
        gui.run()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
