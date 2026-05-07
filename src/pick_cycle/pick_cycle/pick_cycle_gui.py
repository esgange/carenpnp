import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import scrolledtext

import rclpy
from dobot_msgs_v4.msg import ToolVectorActual
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


ITEM_GO_TO_TEACH_SERVICE_DEFAULT = 'item_detect/go_to_teach'
ITEM_ARM_SERVICE_DEFAULT = 'item_pick/track'
ITEM_ARM_STATUS_SERVICE_DEFAULT = 'item_pick/track_status'
ITEM_SEEK_SERVICE_DEFAULT = 'item_detect/seek'
ITEM_SEEK_STATUS_SERVICE_DEFAULT = 'item_detect/seek_status'
TRAY_GO_TO_TEACH_SERVICE_DEFAULT = 'tray_detect/go_to_teach'
TRAY_ARM_SERVICE_DEFAULT = 'tray_intercept/track'
TRAY_ARM_STATUS_SERVICE_DEFAULT = 'tray_intercept/track_status'
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
SEEK_STATUS_POLL_SEC = 0.1
SEEK_STATUS_RESPONSE_TIMEOUT_SEC = 0.2
ARM_CLICK_RESPONSE_TIMEOUT_SEC = 5.5
ARM_STATUS_RESPONSE_TIMEOUT_SEC = 1.0
SERVICE_READY_TIMEOUT_SEC = 5.5
SERVICE_READY_POLL_SEC = 0.1
LOOP_PAUSE_SEC_DEFAULT = 1.0


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


class PickCycleNode(Node):
    def __init__(self) -> None:
        super().__init__('pick_cycle_gui')
        self.item_go_to_teach_service = self._declare_name_parameter(
            'item_go_to_teach_service',
            ITEM_GO_TO_TEACH_SERVICE_DEFAULT,
        )
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
        self.tray_go_to_teach_service = self._declare_name_parameter(
            'tray_go_to_teach_service',
            TRAY_GO_TO_TEACH_SERVICE_DEFAULT,
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

        self._trigger_clients: dict[str, tuple[str, object]] = {
            'item_go_to_teach': (
                self.item_go_to_teach_service,
                self.create_client(Trigger, self.item_go_to_teach_service),
            ),
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
            'tray_go_to_teach': (
                self.tray_go_to_teach_service,
                self.create_client(Trigger, self.tray_go_to_teach_service),
            ),
            'tray_arm': (
                self.tray_arm_service,
                self.create_client(Trigger, self.tray_arm_service),
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

        self.create_subscription(ToolVectorActual, self.robot_tcp_topic, self._tcp_callback, 10)

    def _declare_name_parameter(self, parameter_name: str, default_value: str) -> str:
        value = str(self.declare_parameter(parameter_name, default_value).value).strip()
        return value or default_value

    def service_names(self) -> list[tuple[str, str]]:
        return [
            ('Item Go Teach', self.item_go_to_teach_service),
            ('Item Pick Arm', self.item_arm_service),
            ('Item Pick Arm Status', self.item_arm_status_service),
            ('Item Seek', self.item_seek_service),
            ('Item Seek Status', self.item_seek_status_service),
            ('Tray Go Teach', self.tray_go_to_teach_service),
            ('Tray Intercept Arm', self.tray_arm_service),
            ('Tray Intercept Arm Status', self.tray_arm_status_service),
            ('Tray Seek', self.tray_seek_service),
            ('Tray Seek Status', self.tray_seek_status_service),
        ]

    def topic_names(self) -> list[tuple[str, str]]:
        return [('Robot Feedback', self.robot_tcp_topic)]

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
                for service_name, client in self._trigger_clients.values()
                if not client.service_is_ready()
            ]
            if not missing_names:
                result = TriggerResult(True, f'All {len(self._trigger_clients)} trigger services ready')
                self._cache_startup_service_result(result)
                return result

            now = time.monotonic()
            if now >= deadline:
                break
            time.sleep(min(SERVICE_READY_POLL_SEC, max(0.0, deadline - now)))

        if not missing_names:
            missing_names = [
                service_name
                for service_name, client in self._trigger_clients.values()
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
            for service_name, client in self._trigger_clients.values()
            if not client.service_is_ready()
        ]
        if missing_names:
            return TriggerResult(False, 'Required services unavailable: ' + ', '.join(missing_names))
        return TriggerResult(True, f'All {len(self._trigger_clients)} trigger services ready')

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
        self.root.geometry('760x520')
        self.root.minsize(720, 480)
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._worker_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._cycle_count = 0
        self._startup_services_ok = bool(startup_result.success)

        self.status_var = tk.StringVar(
            value='Idle' if self._startup_services_ok else 'Startup service check failed'
        )
        self.loop_var = tk.BooleanVar(value=False)
        self.loop_pause_var = tk.DoubleVar(value=LOOP_PAUSE_SEC_DEFAULT)
        self.stability_sec_var = tk.DoubleVar(value=ROBOT_STABILITY_SEC_DEFAULT)

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
        tk.Spinbox(
            controls,
            from_=0.0,
            to=60.0,
            increment=0.5,
            textvariable=self.loop_pause_var,
            width=8,
            format='%.1f',
        ).grid(row=2, column=1, sticky='w', pady=(8, 0))

        status_frame = tk.LabelFrame(outer, text='Status', padx=10, pady=8)
        status_frame.grid(row=1, column=0, sticky='ew', pady=(10, 0))
        status_frame.columnconfigure(0, weight=1)
        tk.Label(status_frame, textvariable=self.status_var, anchor='w').grid(row=0, column=0, sticky='ew')

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
        self._stop_event.clear()
        self._running = True
        self._set_running_controls(True)
        self._worker_thread = threading.Thread(target=self._run_worker, args=(config,), daemon=True)
        self._worker_thread.start()

    def _stop_clicked(self) -> None:
        self._stop_event.set()
        self._set_status('Stopping after current monitor step...')

    def _read_config(self) -> CycleConfig:
        return CycleConfig(
            loop_pause_sec=max(0.0, float(self.loop_pause_var.get())),
            loop_enabled=bool(self.loop_var.get()),
            stability_sec=self._read_timing_slider(self.stability_sec_var),
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
                if not config.loop_enabled:
                    break
                if not self._sleep_with_stop(config.loop_pause_sec, 'Loop pause'):
                    break
            self._queue.put(('finished', None))
        except Exception as exc:
            self._log(f'Cycle worker crashed: {exc}')
            self._queue.put(('finished', None))

    def _run_one_cycle(self, config: CycleConfig, cycle_index: int) -> bool:
        return (
            self._go_to_teach_step(cycle_index, 'Go to item detect teach', 'item_go_to_teach')
            and self._seek_step(
                cycle_index,
                'Arm item pick',
                'item_arm',
                'item_arm_status',
                'Seek item detect',
                'item_seek',
                'item_seek_status',
                config,
            )
            and self._go_to_teach_step(cycle_index, 'Go to tray detect teach', 'tray_go_to_teach')
            and self._seek_step(
                cycle_index,
                'Arm tray intercept',
                'tray_arm',
                'tray_arm_status',
                'Seek tray detect',
                'tray_seek',
                'tray_seek_status',
                config,
            )
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
        result = self.node.click_trigger(client_key, wait_response_sec)
        success_label = 'OK' if wait_response_sec is not None else 'SENT'
        self._log(f'[{cycle_index}] {label}: {success_label if result.success else "FAIL"} - {result.message}')
        return result.success and not self._stop_event.is_set()

    def _go_to_teach_step(
        self,
        cycle_index: int,
        label: str,
        client_key: str,
    ) -> bool:
        if not self._click_step(cycle_index, label, client_key):
            return False

        self._log(f'[{cycle_index}] {label}: dispatched; next step will arm.')
        return not self._stop_event.is_set()

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
        if not self._arm_and_verify(cycle_index, arm_label, arm_client_key, arm_status_client_key):
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
    ) -> bool:
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

    def _queue_call(self, action: str, payload: object) -> None:
        self._queue.put((action, payload))

    def _set_status(self, text: str) -> None:
        self._queue_call('status', text)

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
                elif action == 'log':
                    self._append_log(str(payload))
                elif action == 'finished':
                    self._running = False
                    self._set_running_controls(False)
                    self.status_var.set('Idle' if not self._stop_event.is_set() else 'Stopped')
        except queue.Empty:
            pass
        self.root.after(100, self._drain_queue)

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
