import json
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import messagebox, ttk

import rclpy
from ament_index_python.packages import get_package_share_path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dobot_msgs_v4.msg import RobotStatus, ToolVectorActual
from dobot_msgs_v4.srv import (
    AccJ,
    AccL,
    CP,
    ClearError,
    DisableRobot,
    EnableRobot,
    GetErrorID,
    MovJ,
    MovL,
    RobotMode,
    SetPayload,
    SetTool,
    SpeedFactor,
    Stop,
    StopMoveJog,
    StartDrag,
    StopDrag,
    Tool,
    VelJ,
    VelL,
)


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / 'src').exists() and
            (
                (path / 'README.md').exists()
                or (path / 'src' / 'dobot_msgs_v4').exists()
            )
        )

    def find_from(start: Path) -> Path | None:
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
        return None

    for name in ('DOBOT_PICKN_PLACE_ROOT', 'DOBOT_WORKSPACE_ROOT'):
        value = os.environ.get(name)
        if value:
            return find_from(Path(value)) or Path(value).expanduser().resolve()

    candidates = [Path.cwd(), Path(__file__).resolve()]
    for name in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        for token in os.environ.get(name, '').split(os.pathsep):
            if not token:
                continue
            prefix = Path(token)
            candidates.append(prefix)
            if 'install' in prefix.parts:
                candidates.append(Path(*prefix.parts[:prefix.parts.index('install')]))

    for candidate in candidates:
        found = find_from(candidate)
        if found is not None:
            return found
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
TCP_FIELDS = [
    ('x', 'X', 'mm', (-1000.0, 1000.0)),
    ('y', 'Y', 'mm', (-1000.0, 1000.0)),
    ('z', 'Z', 'mm', (-200.0, 1500.0)),
    ('rx', 'Rx', 'deg', (-360.0, 360.0)),
    ('ry', 'Ry', 'deg', (-360.0, 360.0)),
    ('rz', 'Rz', 'deg', (-360.0, 360.0)),
]
SPEED_FIELDS = [
    ('cp', 'CP', '%', 1, 100),
    ('speed_factor', 'Speed Factor', '%', 1, 100),
    ('speed_j', 'SpeedJ', '%', 1, 100),
    ('acc_j', 'AccJ', '%', 1, 100),
    ('speed_l', 'SpeedL', '%', 1, 100),
    ('acc_l', 'AccL', '%', 1, 100),
]
DEFAULT_SPEED_VALUES = {
    'cp': 100,
    'speed_factor': 50,
    'speed_j': 50,
    'acc_j': 50,
    'speed_l': 50,
    'acc_l': 50,
}
ZERO_TOOL_TCP_VALUES = [0.0] * 6
SERVICE_ROOT = '/dobot_bringup_ros2/srv'
STALE_DATA_SEC = 1.5
QUEUED_MOTION_SERVICES = {
    f'{SERVICE_ROOT}/MovJ',
    f'{SERVICE_ROOT}/MovL',
}
SCRIPT_DIR_NAME = 'motion_calibrate'
SCRIPT_FILE_SUFFIX = '.json'
SCRIPT_NAME_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9 _-]{0,63}$')
SCRIPT_POINT_PATTERN = re.compile(
    r'^\s*(?:POINT(?:\s+\d+(?:/\d+)?)?\s+)?(MovJ|MovL)\s*:\s*(.+?)\s*$',
    re.IGNORECASE,
)
SCRIPT_GOAL_JOINT_TOLERANCE_DEG = 5.0
SCRIPT_GOAL_TCP_POSITION_TOLERANCE_MM = 20.0
SCRIPT_GOAL_TCP_ROTATION_TOLERANCE_DEG = 5.0
# CP blending may not land exactly on each waypoint; accept near-goal hits.
SCRIPT_GOAL_PROGRESS_RATIO_THRESHOLD = 0.90
SCRIPT_GOAL_PROGRESS_RATIO_MIN_PERCENT = 60.0
SCRIPT_GOAL_PROGRESS_RATIO_MAX_PERCENT = 99.0
SCRIPT_GOAL_PROGRESS_AXIS_EPSILON = 1e-3
CLEAR_ERROR_ESTOP_CHECK_DELAY_SEC = 2.0
SCRIPT_MOTION_ARGUMENT_KEYS = {
    'MovJ': ('speed_j', 'acc_j'),
    'MovL': ('speed_l', 'acc_l'),
}
SCRIPT_MOTION_ARGUMENT_PROTOCOL_KEYS = {
    'speed_j': 'v',
    'acc_j': 'a',
    'speed_l': 'v',
    'acc_l': 'a',
    'v': 'v',
    'a': 'a',
    'speed': 'speed',
}
SCRIPT_SPEED_PROFILE_KEYS = ('cp', 'speed_factor')
DEFAULT_TCP_LIMITS = {field_name: limits for field_name, _, _, limits in TCP_FIELDS}
CR10A_TCP_LIMITS = {
    **DEFAULT_TCP_LIMITS,
    'x': (-1300.0, 1300.0),
    'y': (-1300.0, 1300.0),
}


@dataclass(frozen=True)
class TcpWorkspaceProfile:
    robot_type: str
    label: str
    limits: dict[str, tuple[float, float]]
    xy_inner_radius_mm: float | None = None
    xy_outer_radius_mm: float | None = None


TCP_WORKSPACE_PROFILES = {
    'cr10': TcpWorkspaceProfile(
        robot_type='cr10',
        label='CR10A radial workspace',
        limits=CR10A_TCP_LIMITS,
        xy_inner_radius_mm=193.0,
        xy_outer_radius_mm=1300.0,
    ),
    'cr10a': TcpWorkspaceProfile(
        robot_type='cr10a',
        label='CR10A radial workspace',
        limits=CR10A_TCP_LIMITS,
        xy_inner_radius_mm=193.0,
        xy_outer_radius_mm=1300.0,
    ),
}


def _load_robot_type_from_bringup_config() -> str:
    candidate_paths: list[Path] = []
    candidate_paths.append(workspace_path('config', 'robot_bringup', 'param.json'))
    try:
        candidate_paths.append(Path(get_package_share_path('cr_robot_ros2')) / 'config' / 'param.json')
    except Exception:
        pass

    seen_paths: set[Path] = set()
    for config_path in candidate_paths:
        if config_path in seen_paths:
            continue
        seen_paths.add(config_path)
        if not config_path.exists():
            continue
        try:
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config = json.load(config_file)
        except Exception:
            continue

        node_info = config.get('node_info', [])
        if not isinstance(node_info, list) or not node_info:
            continue

        current_robot = int(config.get('current_robot', 1))
        index = max(0, min(current_robot - 1, len(node_info) - 1))
        robot_type = str(node_info[index].get('robot_type', 'unknown')).strip().lower()
        if robot_type:
            return robot_type

    return 'unknown'


def _workspace_profile_for_robot(robot_type: str) -> TcpWorkspaceProfile:
    return TCP_WORKSPACE_PROFILES.get(
        robot_type,
        TcpWorkspaceProfile(
            robot_type=robot_type,
            label='Default TCP workspace',
            limits=DEFAULT_TCP_LIMITS,
        ),
    )


@dataclass
class MotionSnapshot:
    connected: bool | None = None
    enabled: bool | None = None
    drag_enabled: bool | None = None
    use_tool_enabled: bool = False
    controller_mode_code: int | None = None
    controller_text: str = 'Waiting for controller state...'
    error_text: str = 'No error info yet'
    error_active: bool = False
    action_text: str = 'Ready'
    busy_action: str | None = None
    ee_load_kg: float | None = None
    tool_tcp_values: list[float] = field(default_factory=lambda: [0.0] * 6)
    speed_values: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_SPEED_VALUES))
    joints_deg: dict[str, float] = field(
        default_factory=lambda: {name: 0.0 for name in JOINT_NAMES}
    )
    tcp_values: dict[str, float] = field(
        default_factory=lambda: {field_name: 0.0 for field_name, _, _, _ in TCP_FIELDS}
    )
    joint_stamp: float | None = None
    tcp_stamp: float | None = None
    status_stamp: float | None = None


@dataclass
class QueuedServiceCall:
    client: object
    request: object
    action_name: str
    service_name: str
    action_call: str
    on_success: object = None
    on_complete: object = None


@dataclass
class ScriptMotionPoint:
    motion_type: str
    values: list[float]
    use_tool: bool = False
    motion_args: list[str] = field(default_factory=list)


@dataclass
class PendingScriptGoal:
    script_name: str
    point_index: int
    total_points: int
    motion_type: str
    values: list[float]
    start_values: list[float] | None = None


@dataclass
class ScriptGoalHit:
    script_name: str
    point_index: int
    total_points: int
    motion_type: str
    values: list[float]
    actual_values: list[float] | None
    ros_sec: int
    ros_nanosec: int


class MotionDebugNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_debug_gui')
        self._lock = threading.Lock()
        self._snapshot = MotionSnapshot()
        self._queued_service_call: QueuedServiceCall | None = None
        self._startup_speed_factor_target = DEFAULT_SPEED_VALUES['speed_factor']
        self._startup_speed_factor_attempted = False
        self._startup_cp_target = DEFAULT_SPEED_VALUES['cp']
        self._startup_cp_attempted = False
        self._startup_tool_disable_attempted = False
        self._startup_tool_tcp_reset_attempted = False
        self._robot_mode_future = None
        self._error_id_future = None
        self._raw_tcp_values = {field_name: 0.0 for field_name, _, _, _ in TCP_FIELDS}
        self._last_motion_point: ScriptMotionPoint | None = None
        self._script_run_active = False
        self._script_run_name: str | None = None
        self._script_command_log: list[str] = []
        self._script_command_log_version = 0
        self._script_dispatched_point_log_by_action: dict[str, str] = {}
        self._pending_script_goals: list[PendingScriptGoal] = []
        self._script_log_start_monotonic: float | None = None
        self._script_hit_zero_ros_nanos: int | None = None
        self._script_goal_progress_ratio_threshold = SCRIPT_GOAL_PROGRESS_RATIO_THRESHOLD
        self._script_stop_requested = False
        self._clear_error_estop_check_deadline: float | None = None
        self._log_lock = threading.Lock()
        self._log_started_at_monotonic = time.monotonic()
        self._log_file_path = self._create_launch_log_file()

        self.create_subscription(JointState, '/joint_states_robot', self._joint_callback, 10)
        self.create_subscription(ToolVectorActual, 'dobot_msgs_v4/msg/ToolVectorActual', self._tcp_callback, 10)
        self.create_subscription(RobotStatus, 'dobot_msgs_v4/msg/RobotStatus', self._status_callback, 10)

        self._clear_error_client = self.create_client(ClearError, f'{SERVICE_ROOT}/ClearError')
        self._enable_robot_client = self.create_client(EnableRobot, f'{SERVICE_ROOT}/EnableRobot')
        self._disable_robot_client = self.create_client(DisableRobot, f'{SERVICE_ROOT}/DisableRobot')
        self._stop_client = self.create_client(Stop, f'{SERVICE_ROOT}/Stop')
        self._stop_move_jog_client = self.create_client(StopMoveJog, f'{SERVICE_ROOT}/StopMoveJog')
        self._start_drag_client = self.create_client(StartDrag, f'{SERVICE_ROOT}/StartDrag')
        self._stop_drag_client = self.create_client(StopDrag, f'{SERVICE_ROOT}/StopDrag')
        self._tool_client = self.create_client(Tool, f'{SERVICE_ROOT}/Tool')
        self._robot_mode_client = self.create_client(RobotMode, f'{SERVICE_ROOT}/RobotMode')
        self._get_error_id_client = self.create_client(GetErrorID, f'{SERVICE_ROOT}/GetErrorID')
        self._set_payload_client = self.create_client(SetPayload, f'{SERVICE_ROOT}/SetPayload')
        self._set_tool_client = self.create_client(SetTool, f'{SERVICE_ROOT}/SetTool')
        self._cp_client = self.create_client(CP, f'{SERVICE_ROOT}/CP')
        self._speed_factor_client = self.create_client(SpeedFactor, f'{SERVICE_ROOT}/SpeedFactor')
        self._vel_j_client = self.create_client(VelJ, f'{SERVICE_ROOT}/VelJ')
        self._acc_j_client = self.create_client(AccJ, f'{SERVICE_ROOT}/AccJ')
        self._vel_l_client = self.create_client(VelL, f'{SERVICE_ROOT}/VelL')
        self._acc_l_client = self.create_client(AccL, f'{SERVICE_ROOT}/AccL')
        self._mov_j_client = self.create_client(MovJ, f'{SERVICE_ROOT}/MovJ')
        self._mov_l_client = self.create_client(MovL, f'{SERVICE_ROOT}/MovL')

        self.create_timer(1.0, self._poll_controller_state)
        self._log_event('startup', f'motion_debug launch log created at {self._log_file_path}')
        self.get_logger().info(f'motion_debug diagnostics log: {self._log_file_path}')

    def snapshot(self) -> MotionSnapshot:
        with self._lock:
            return MotionSnapshot(
                connected=self._snapshot.connected,
                enabled=self._snapshot.enabled,
                drag_enabled=self._snapshot.drag_enabled,
                use_tool_enabled=self._snapshot.use_tool_enabled,
                controller_mode_code=self._snapshot.controller_mode_code,
                controller_text=self._snapshot.controller_text,
                error_text=self._snapshot.error_text,
                error_active=self._snapshot.error_active,
                action_text=self._snapshot.action_text,
                busy_action=self._snapshot.busy_action,
                ee_load_kg=self._snapshot.ee_load_kg,
                tool_tcp_values=list(self._snapshot.tool_tcp_values),
                speed_values=dict(self._snapshot.speed_values),
                joints_deg=dict(self._snapshot.joints_deg),
                tcp_values=dict(self._snapshot.tcp_values),
                joint_stamp=self._snapshot.joint_stamp,
                tcp_stamp=self._snapshot.tcp_stamp,
                status_stamp=self._snapshot.status_stamp,
            )

    def _create_launch_log_file(self) -> Path:
        log_dir = workspace_path('Log', 'motion_debug')
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = log_dir / f'log_{timestamp}.txt'
        log_path.touch(exist_ok=True)
        return log_path

    def _log_event(self, category: str, message: str) -> None:
        timestamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
        elapsed_sec = time.monotonic() - self._log_started_at_monotonic
        with self._log_lock:
            with open(self._log_file_path, 'a', encoding='utf-8') as log_file:
                log_file.write(f'[{timestamp}] [+{elapsed_sec:0.3f}s] {category.upper()}: {message}\n')

    def clear_error(self) -> None:
        self._send_simple_service(
            self._clear_error_client,
            ClearError.Request(),
            'Clear Error',
            f'{SERVICE_ROOT}/ClearError',
            on_complete=self._schedule_clear_error_estop_check,
        )

    def _schedule_clear_error_estop_check(self) -> None:
        with self._lock:
            self._clear_error_estop_check_deadline = time.monotonic() + CLEAR_ERROR_ESTOP_CHECK_DELAY_SEC

    def consume_clear_error_estop_prompt(self) -> bool:
        now = time.monotonic()
        should_prompt = False
        with self._lock:
            if self._clear_error_estop_check_deadline is None:
                return False
            if now < self._clear_error_estop_check_deadline:
                return False

            should_prompt = bool(
                self._snapshot.connected
                and (self._snapshot.controller_mode_code == 9 or self._snapshot.error_active)
            )
            self._clear_error_estop_check_deadline = None

        if should_prompt:
            self._log_event(
                'ui',
                'Clear Error completed but error state is still active; prompting operator to check emergency stop.',
            )
        return should_prompt

    def toggle_enable(self) -> None:
        snapshot = self.snapshot()
        if snapshot.controller_mode_code == 10:
            self._send_simple_service(
                self._stop_client,
                Stop.Request(),
                'Exit Pause',
                f'{SERVICE_ROOT}/Stop',
                on_success=self._enable_after_pause,
            )
            return

        if snapshot.controller_mode_code == 11:
            self._send_simple_service(
                self._stop_move_jog_client,
                StopMoveJog.Request(),
                'Recover Jog',
                f'{SERVICE_ROOT}/StopMoveJog',
                on_complete=self._clear_error_after_stop_jog,
            )
            return

        if self._robot_should_disable(snapshot):
            self._send_simple_service(
                self._stop_move_jog_client,
                StopMoveJog.Request(),
                'Prepare Disable',
                f'{SERVICE_ROOT}/StopMoveJog',
                on_complete=self._disable_robot_after_stop_jog,
            )
        else:
            self._enable_robot_with_defaults()

    def toggle_drag(self) -> None:
        snapshot = self.snapshot()
        if snapshot.drag_enabled:
            self._send_simple_service(
                self._stop_drag_client,
                StopDrag.Request(),
                'Disable Drag',
                f'{SERVICE_ROOT}/StopDrag',
                on_success=self._mark_drag_disabled,
            )
        else:
            if snapshot.ee_load_kg is None:
                self._set_action_text('For Drag Set EE Load.')
                return
            self._send_simple_service(
                self._start_drag_client,
                StartDrag.Request(),
                'Enable Drag',
                f'{SERVICE_ROOT}/StartDrag',
                on_success=self._mark_drag_enabled,
            )

    def toggle_use_tool(self) -> None:
        snapshot = self.snapshot()
        use_tool_enabled = not snapshot.use_tool_enabled
        request = Tool.Request()
        request.index = 1 if use_tool_enabled else 0
        self._send_simple_service(
            self._tool_client,
            request,
            f'Activate Tool {request.index}',
            f'{SERVICE_ROOT}/Tool',
            on_success=lambda enabled=use_tool_enabled: self._mark_use_tool_enabled(enabled),
        )

    def set_speed_profile_value(self, key: str, value: int) -> None:
        clamped = max(1, min(100, int(round(value))))
        client, request, label, service_name = self._build_speed_request(key, clamped)
        if client is None or request is None or label is None or service_name is None:
            self._set_action_text(f'Unsupported speed setting: {key}')
            return

        if not client.service_is_ready():
            self._set_action_text(f'Set {label}: service is not ready.')
            return

        with self._lock:
            self._snapshot.speed_values[key] = clamped

        self._send_simple_service(
            client,
            request,
            f'Set {label} to {clamped}%',
            service_name,
        )

    def get_script_speed_profile_snapshot(self) -> dict[str, int]:
        with self._lock:
            return {
                key: max(1, min(100, int(round(self._snapshot.speed_values.get(key, DEFAULT_SPEED_VALUES[key])))))
                for key in SCRIPT_SPEED_PROFILE_KEYS
            }

    def apply_script_speed_profile_cache(self, speed_profile: dict[str, object] | None) -> dict[str, int]:
        normalized = self._normalize_script_speed_profile(speed_profile)
        with self._lock:
            for key, value in normalized.items():
                self._snapshot.speed_values[key] = value
        return normalized

    def _normalize_script_speed_profile(self, speed_profile: dict[str, object] | None) -> dict[str, int]:
        with self._lock:
            current_values = dict(self._snapshot.speed_values)
        normalized: dict[str, int] = {}
        for key in SCRIPT_SPEED_PROFILE_KEYS:
            fallback = int(current_values.get(key, DEFAULT_SPEED_VALUES[key]))
            raw_value = fallback
            if isinstance(speed_profile, dict):
                raw_value = speed_profile.get(key, fallback)
            try:
                value = int(round(float(raw_value)))
            except (TypeError, ValueError):
                value = fallback
            normalized[key] = max(1, min(100, value))
        return normalized

    def set_script_goal_check_tolerance_percent(self, tolerance_percent: float) -> None:
        try:
            normalized_percent = float(tolerance_percent)
        except (TypeError, ValueError):
            return

        clamped_percent = max(
            SCRIPT_GOAL_PROGRESS_RATIO_MIN_PERCENT,
            min(SCRIPT_GOAL_PROGRESS_RATIO_MAX_PERCENT, normalized_percent),
        )
        with self._lock:
            self._script_goal_progress_ratio_threshold = clamped_percent / 100.0

    def get_script_goal_check_tolerance_percent(self) -> float:
        with self._lock:
            return self._script_goal_progress_ratio_threshold * 100.0

    def set_ee_load(self, load_kg: float) -> None:
        request = SetPayload.Request()
        request.load = load_kg
        request.x = 0.0
        request.y = 0.0
        request.z = 0.0
        self._send_simple_service(
            self._set_payload_client,
            request,
            f'Set EE Load to {load_kg:.2f} kg',
            f'{SERVICE_ROOT}/SetPayload',
            on_success=lambda: self._mark_ee_load(load_kg),
        )

    def set_tool_tcp(self, values: list[float]) -> None:
        self._send_tool_tcp(values, 'Set Tool')

    def _send_tool_tcp(self, values: list[float], action_name: str) -> None:
        request = SetTool.Request()
        request.index = 1
        request.value = '{' + ','.join(f'{value:.3f}' for value in values) + '}'
        self._send_simple_service(
            self._set_tool_client,
            request,
            action_name,
            f'{SERVICE_ROOT}/SetTool',
            on_success=lambda: self._mark_tool_tcp(values),
        )

    def move_joint_target(self, targets: dict[str, float]) -> None:
        snapshot = self.snapshot()
        use_tool_enabled = snapshot.use_tool_enabled
        motion_args = self._script_motion_args_for_speed_values('MovJ', snapshot.speed_values)
        values = [targets[name] for name in JOINT_NAMES]
        request = self._build_movj_request(values, use_tool=use_tool_enabled, motion_args=motion_args)
        self._send_simple_service(
            self._mov_j_client,
            request,
            'Send Joint Target',
            f'{SERVICE_ROOT}/MovJ',
            on_dispatch=lambda point_values=list(values), use_tool=use_tool_enabled, point_motion_args=list(motion_args): self._record_last_motion_point(
                'MovJ',
                point_values,
                use_tool,
                point_motion_args,
            ),
        )

    def move_tcp_target(self, targets: dict[str, float]) -> None:
        snapshot = self.snapshot()
        use_tool_enabled = snapshot.use_tool_enabled
        motion_args = self._script_motion_args_for_speed_values('MovL', snapshot.speed_values)
        values = [targets[field_name] for field_name, _, _, _ in TCP_FIELDS]
        request = self._build_movl_request(values, use_tool=use_tool_enabled, motion_args=motion_args)
        self._send_simple_service(
            self._mov_l_client,
            request,
            'Send TCP Target',
            f'{SERVICE_ROOT}/MovL',
            on_dispatch=lambda point_values=list(values), use_tool=use_tool_enabled, point_motion_args=list(motion_args): self._record_last_motion_point(
                'MovL',
                point_values,
                use_tool,
                point_motion_args,
            ),
        )

    def get_last_motion_point(self) -> dict[str, object] | None:
        with self._lock:
            if self._last_motion_point is None:
                return None
            return {
                'motion_type': self._last_motion_point.motion_type,
                'values': list(self._last_motion_point.values),
                'use_tool': self._last_motion_point.use_tool,
                'motion_args': list(self._last_motion_point.motion_args),
            }

    def capture_current_joint_position_as_last_movj_point(self) -> dict[str, object] | None:
        with self._lock:
            if self._snapshot.joint_stamp is None:
                return None
            values = [float(self._snapshot.joints_deg[name]) for name in JOINT_NAMES]
            use_tool = self._snapshot.use_tool_enabled
            motion_args = self._script_motion_args_for_speed_values('MovJ', self._snapshot.speed_values)
            self._last_motion_point = ScriptMotionPoint(
                motion_type='MovJ',
                values=list(values),
                use_tool=use_tool,
                motion_args=list(motion_args),
            )
            return {
                'motion_type': 'MovJ',
                'values': list(values),
                'use_tool': use_tool,
                'motion_args': list(motion_args),
            }

    def is_script_running(self) -> bool:
        with self._lock:
            return self._is_script_busy_locked()

    def _is_script_busy_locked(self) -> bool:
        return self._script_run_active or self._script_stop_requested or bool(self._pending_script_goals)

    def get_script_command_log(self) -> tuple[int, list[str]]:
        with self._lock:
            return self._script_command_log_version, list(self._script_command_log)

    def clear_script_command_log(self) -> None:
        self._clear_script_command_log()

    def run_motion_script(
        self,
        script_name: str,
        points: list[dict[str, object]],
        script_speed_profile: dict[str, object] | None = None,
    ) -> bool:
        normalized_name = script_name.strip() or 'script'
        normalized_points = self._normalize_script_points(points)
        if not normalized_points:
            self._set_action_text(f'Script "{normalized_name}" has no valid MovJ/MovL points.')
            return False
        normalized_speed_profile = self._normalize_script_speed_profile(script_speed_profile)

        with self._lock:
            if self._is_script_busy_locked():
                active_name = self._script_run_name or 'another script'
                self._snapshot.action_text = f'Script "{active_name}" is already running.'
                return False
            self._script_run_active = True
            self._script_run_name = normalized_name
            self._pending_script_goals = []
            self._script_hit_zero_ros_nanos = None
            self._script_stop_requested = False

        self._clear_script_command_log()
        with self._lock:
            self._script_log_start_monotonic = time.monotonic()
        self._append_script_command_log(
            f'START script "{normalized_name}" with {len(normalized_points)} point(s)',
            elapsed_ms=0,
        )
        self._append_script_command_log(
            f'PROFILE cp={normalized_speed_profile["cp"]}% speed_factor={normalized_speed_profile["speed_factor"]}%'
        )
        self._log_event('script', f'start "{normalized_name}" with {len(normalized_points)} point(s)')
        self._set_action_text(
            f'Running script "{normalized_name}" ({len(normalized_points)} point(s), '
            f'CP={normalized_speed_profile["cp"]}%, SF={normalized_speed_profile["speed_factor"]}%)...'
        )
        self._run_script_with_speed_profile(normalized_name, normalized_points, normalized_speed_profile)
        return True

    def _run_script_with_speed_profile(
        self,
        script_name: str,
        points: list[ScriptMotionPoint],
        speed_profile: dict[str, int],
    ) -> None:
        speed_steps = [('cp', speed_profile['cp']), ('speed_factor', speed_profile['speed_factor'])]
        self._apply_next_script_speed_setting(script_name, points, speed_steps, 0)

    def _apply_next_script_speed_setting(
        self,
        script_name: str,
        points: list[ScriptMotionPoint],
        speed_steps: list[tuple[str, int]],
        step_index: int,
    ) -> None:
        with self._lock:
            if not self._script_run_active:
                return
            if self._script_run_name != script_name:
                return
            stop_requested = self._script_stop_requested
            current_speed_values = dict(self._snapshot.speed_values)
        if stop_requested:
            self._finish_script_run(f'Script "{script_name}" stopped by user.', clear_pending_goals=True)
            return

        if step_index >= len(speed_steps):
            self._run_script_point(script_name, points, 0)
            return

        speed_key, target_value = speed_steps[step_index]
        target_value = max(1, min(100, int(round(target_value))))
        current_value = int(current_speed_values.get(speed_key, DEFAULT_SPEED_VALUES[speed_key]))
        client, request, label, service_name = self._build_speed_request(speed_key, target_value)
        if client is None or request is None or label is None or service_name is None:
            self._finish_script_run(
                f'Script "{script_name}" failed: unsupported script speed key "{speed_key}".',
                clear_pending_goals=True,
            )
            return

        if current_value == target_value:
            self._append_script_command_log(f'SET {label}={target_value}% (already set)')
            self._apply_next_script_speed_setting(script_name, points, speed_steps, step_index + 1)
            return

        self._append_script_command_log(f'SET {label}={target_value}%')
        dispatch_succeeded = {'ok': False}

        def _on_success() -> None:
            dispatch_succeeded['ok'] = True
            with self._lock:
                self._snapshot.speed_values[speed_key] = target_value
            self._apply_next_script_speed_setting(script_name, points, speed_steps, step_index + 1)

        def _on_complete() -> None:
            if dispatch_succeeded['ok']:
                return
            self._finish_script_run(
                f'Script "{script_name}" failed while applying {label}={target_value}%.',
                clear_pending_goals=True,
            )

        sent = self._send_simple_service(
            client,
            request,
            f'Script "{script_name}" Set {label} to {target_value}%',
            service_name,
            on_success=_on_success,
            on_complete=_on_complete,
            allow_local_queue=False,
        )
        if not sent:
            self._finish_script_run(
                f'Script "{script_name}" failed: {label} service is not ready.',
                clear_pending_goals=True,
            )

    def stop_motion_script(self) -> bool:
        manual_stop_only = False
        with self._lock:
            if not self._is_script_busy_locked():
                self._snapshot.action_text = 'No script is currently running. Sending Stop...'
                manual_stop_only = True
            else:
                if self._script_stop_requested:
                    self._snapshot.action_text = 'Script stop already requested.'
                    return True
                active_name = self._script_run_name or 'script'
                self._script_stop_requested = True
                self._pending_script_goals = []
                self._snapshot.action_text = f'Stopping script "{active_name}"...'

        if manual_stop_only:
            return self.stop_robot_motion('Manual Stop')

        self._append_script_command_log('STOP requested by user')
        self._log_event('script', f'stop requested "{active_name}"')
        sent = self.stop_robot_motion(
            'Stop Script Motion',
            on_complete=lambda run_name=active_name: self._complete_script_stop_if_pending(run_name),
        )
        if not sent:
            self._finish_script_run(
                f'Script "{active_name}" stop failed: Stop service is not ready.',
                clear_pending_goals=True,
            )
            return False
        return True

    def stop_robot_motion(self, action_name: str = 'Stop Robot', on_complete=None) -> bool:
        sent = self._send_simple_service(
            self._stop_client,
            Stop.Request(),
            action_name,
            f'{SERVICE_ROOT}/Stop',
            on_complete=on_complete,
            allow_local_queue=False,
        )
        if not sent:
            self._set_action_text(f'{action_name}: Stop service is not ready.')
        return sent

    def _complete_script_stop_if_pending(self, script_name: str) -> None:
        with self._lock:
            if (self._script_run_name or '') != script_name:
                return
            if not self._script_stop_requested:
                return
        self._finish_script_run(f'Script "{script_name}" stopped by user.', clear_pending_goals=True)

    def _build_movj_request(
        self,
        values: list[float],
        use_tool: bool = False,
        motion_args: list[str] | None = None,
    ) -> MovJ.Request:
        request = MovJ.Request()
        request.mode = True
        request.a = values[0]
        request.b = values[1]
        request.c = values[2]
        request.d = values[3]
        request.e = values[4]
        request.f = values[5]
        request.param_value = self._build_motion_param_value_list(use_tool, motion_args)
        return request

    def _build_movl_request(
        self,
        values: list[float],
        use_tool: bool = False,
        motion_args: list[str] | None = None,
    ) -> MovL.Request:
        request = MovL.Request()
        request.mode = False
        request.a = values[0]
        request.b = values[1]
        request.c = values[2]
        request.d = values[3]
        request.e = values[4]
        request.f = values[5]
        request.param_value = self._build_motion_param_value_list(use_tool, motion_args)
        return request

    def _build_motion_param_value_list(self, use_tool: bool, motion_args: list[str] | None = None) -> list[str]:
        merged_args: list[str] = []
        merged_keys: set[str] = set()
        if use_tool:
            merged_args.append('tool=1')
            merged_keys.add('tool')
        if motion_args:
            for arg in motion_args:
                normalized = self._normalize_motion_arg_token(arg)
                if normalized is None:
                    continue
                key, raw_value = normalized.split('=', 1)
                if key == 'tool':
                    continue
                if key in merged_keys:
                    continue
                merged_keys.add(key)
                protocol_key = SCRIPT_MOTION_ARGUMENT_PROTOCOL_KEYS.get(key, key)
                merged_args.append(f'{protocol_key}={raw_value}')
        if not merged_args:
            return []
        if len(merged_args) == 1:
            return merged_args
        return [','.join(merged_args)]

    def _script_motion_args_for_speed_values(self, motion_type: str, speed_values: dict[str, int]) -> list[str]:
        keys = SCRIPT_MOTION_ARGUMENT_KEYS.get(motion_type, ())
        motion_args: list[str] = []
        for key in keys:
            raw_value = speed_values.get(key)
            try:
                normalized_value = max(1, min(100, int(round(float(raw_value)))))
            except (TypeError, ValueError):
                continue
            motion_args.append(f'{key}={normalized_value}')
        return motion_args

    def _normalize_motion_arg_token(self, value) -> str | None:
        if isinstance(value, (int, float)):
            return None
        if value is None:
            return None
        token = str(value).strip().replace(' ', '')
        if not token or '=' not in token:
            return None
        key, raw_value = token.split('=', 1)
        key = key.strip().lower()
        raw_value = raw_value.strip()
        if not key or not raw_value:
            return None
        canonical_key = self._canonical_script_motion_key(key)
        if canonical_key is None:
            return None
        return f'{canonical_key}={raw_value}'

    def _canonical_script_motion_key(self, key: str) -> str | None:
        normalized = key.strip().lower().replace('-', '_')
        normalized_compact = normalized.replace('_', '')
        if normalized_compact == 'tool':
            return 'tool'
        if normalized_compact == 'v':
            return 'v'
        if normalized_compact == 'a':
            return 'a'
        if normalized_compact == 'speed':
            return 'speed'
        if normalized_compact == 'speedj':
            return 'speed_j'
        if normalized_compact == 'accj':
            return 'acc_j'
        if normalized_compact == 'speedl':
            return 'speed_l'
        if normalized_compact == 'accl':
            return 'acc_l'
        return None

    def _canonical_script_motion_key_for_type(self, motion_type: str, key: str) -> str | None:
        canonical_key = self._canonical_script_motion_key(key)
        if canonical_key is None:
            return None
        if canonical_key in {'v', 'speed'}:
            return 'speed_j' if motion_type == 'MovJ' else 'speed_l'
        if canonical_key == 'a':
            return 'acc_j' if motion_type == 'MovJ' else 'acc_l'
        return canonical_key

    def _normalize_script_motion_args_for_type(self, motion_type: str, motion_args) -> list[str]:
        allowed_keys = set(SCRIPT_MOTION_ARGUMENT_KEYS.get(motion_type, ()))
        if not allowed_keys:
            return []

        normalized_map: dict[str, int] = {}

        raw_items: list[tuple[str, object]] = []
        if isinstance(motion_args, dict):
            raw_items.extend((str(key), value) for key, value in motion_args.items())
        elif isinstance(motion_args, (list, tuple)):
            for item in motion_args:
                token = self._normalize_motion_arg_token(item)
                if token is None:
                    continue
                key, raw_value = token.split('=', 1)
                raw_items.append((key, raw_value))
        elif isinstance(motion_args, str):
            for item in motion_args.split(','):
                token = self._normalize_motion_arg_token(item)
                if token is None:
                    continue
                key, raw_value = token.split('=', 1)
                raw_items.append((key, raw_value))

        for key, value in raw_items:
            normalized_key = self._canonical_script_motion_key_for_type(motion_type, str(key))
            if normalized_key is None:
                continue
            if normalized_key not in allowed_keys:
                continue
            try:
                normalized_value = max(1, min(100, int(round(float(value)))))
            except (TypeError, ValueError):
                continue
            normalized_map[normalized_key] = normalized_value

        ordered_args: list[str] = []
        for key in SCRIPT_MOTION_ARGUMENT_KEYS.get(motion_type, ()):
            if key in normalized_map:
                ordered_args.append(f'{key}={normalized_map[key]}')
        return ordered_args

    def _normalize_script_points(self, points: list[dict[str, object]]) -> list[ScriptMotionPoint]:
        normalized: list[ScriptMotionPoint] = []
        for point in points:
            if not isinstance(point, dict):
                continue

            motion_type = self._normalize_motion_type(str(point.get('motion_type', '')))
            values = point.get('values')
            if motion_type is None or not isinstance(values, list) or len(values) != 6:
                continue

            try:
                numeric_values = [float(value) for value in values]
            except (TypeError, ValueError):
                continue
            use_tool = self._normalize_use_tool_flag(point.get('use_tool', False))
            motion_args = self._normalize_script_motion_args_for_type(motion_type, point.get('motion_args'))
            normalized.append(
                ScriptMotionPoint(
                    motion_type=motion_type,
                    values=numeric_values,
                    use_tool=use_tool,
                    motion_args=motion_args,
                )
            )
        return normalized

    def _normalize_motion_type(self, motion_type: str) -> str | None:
        normalized = motion_type.strip().lower()
        if normalized == 'movj':
            return 'MovJ'
        if normalized in {'movl', 'moll'}:
            return 'MovL'
        return None

    def _normalize_use_tool_flag(self, value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {'1', 'true', 'yes', 'on'}:
                return True
        return False

    def _is_script_action_call(self, action_call: str) -> bool:
        return action_call.startswith('Script "')

    def _clear_script_command_log(self) -> None:
        with self._lock:
            self._script_command_log = []
            self._script_command_log_version += 1
            self._script_dispatched_point_log_by_action = {}
            self._script_log_start_monotonic = None
            self._script_hit_zero_ros_nanos = None
            self._script_stop_requested = False

    def _append_script_command_log(self, message: str, elapsed_ms: int | None = None) -> None:
        if elapsed_ms is None:
            with self._lock:
                script_start = self._script_log_start_monotonic
            if script_start is not None:
                elapsed_ms = max(0, int(round((time.monotonic() - script_start) * 1000.0)))

        if elapsed_ms is None:
            timestamp = datetime.now().strftime('%H:%M:%S')
            entry = f'[{timestamp}] {message}'
        else:
            entry = f'[{elapsed_ms}ms] {message}'

        with self._lock:
            self._script_command_log.append(entry)
            if len(self._script_command_log) > 500:
                self._script_command_log = self._script_command_log[-500:]
            self._script_command_log_version += 1

    def _format_script_point_values(self, values: list[float]) -> str:
        return ','.join(f'{value:.3f}' for value in values)

    def _run_script_point(self, script_name: str, points: list[ScriptMotionPoint], step_index: int) -> None:
        with self._lock:
            if not self._script_run_active:
                return
            if self._script_run_name != script_name:
                return
            stop_requested = self._script_stop_requested

        if stop_requested:
            self._finish_script_run(f'Script "{script_name}" stopped by user.', clear_pending_goals=True)
            return

        if step_index >= len(points):
            self._finish_script_run(
                f'Script "{script_name}" finished: {len(points)} point(s).',
                clear_pending_goals=False,
            )
            return

        point = points[step_index]
        self._run_script_dispatch_point_motion(script_name, points, step_index, point)

    def _run_script_dispatch_point_motion(
        self,
        script_name: str,
        points: list[ScriptMotionPoint],
        step_index: int,
        point: ScriptMotionPoint,
    ) -> None:
        values_text = self._format_script_point_values(point.values)
        param_tokens = self._build_motion_param_value_list(point.use_tool, point.motion_args)
        param_suffix = ''
        if param_tokens:
            param_suffix = ', ' + ', '.join(param_tokens)
        point_log_message = f'POINT {step_index + 1}/{len(points)} {point.motion_type}: {values_text}{param_suffix}'
        action_name = f'Script "{script_name}" Point {step_index + 1}/{len(points)} {point.motion_type}'
        self._append_script_command_log(point_log_message)

        if point.motion_type == 'MovJ':
            client = self._mov_j_client
            request = self._build_movj_request(point.values, use_tool=point.use_tool, motion_args=point.motion_args)
            service_name = f'{SERVICE_ROOT}/MovJ'
        else:
            client = self._mov_l_client
            request = self._build_movl_request(point.values, use_tool=point.use_tool, motion_args=point.motion_args)
            service_name = f'{SERVICE_ROOT}/MovL'

        dispatched = self._send_simple_service(
            client,
            request,
            action_name,
            service_name,
            on_dispatch=lambda run_name=script_name, point_number=step_index + 1, total_points=len(points), motion_type=point.motion_type, values=list(point.values), use_tool=point.use_tool, motion_args=list(point.motion_args), dispatched_action_name=action_name, dispatched_point_log_message=point_log_message: self._record_dispatched_script_goal(
                run_name,
                point_number,
                total_points,
                motion_type,
                values,
                use_tool,
                motion_args,
                dispatched_action_name,
                dispatched_point_log_message,
            ),
            on_complete=lambda run_name=script_name, run_points=points, next_step=step_index + 1: self._run_script_point(
                run_name,
                run_points,
                next_step,
            ),
            allow_local_queue=False,
        )
        if not dispatched:
            self._finish_script_run(
                f'Script "{script_name}" failed at point {step_index + 1}: service is not ready.',
                clear_pending_goals=True,
            )

    def _finish_script_run(self, message: str, clear_pending_goals: bool = True) -> None:
        with self._lock:
            if not self._is_script_busy_locked() and self._script_run_name is None:
                return
            self._script_run_active = False
            self._script_dispatched_point_log_by_action = {}
            self._script_stop_requested = False
            if clear_pending_goals:
                self._pending_script_goals = []
                self._script_run_name = None
            elif not self._pending_script_goals:
                self._script_run_name = None
        self._append_script_command_log(f'END {message}')
        self._set_action_text(message)
        self._log_event('script', message)

    def _record_last_motion_point(
        self,
        motion_type: str,
        values: list[float],
        use_tool: bool,
        motion_args: list[str] | None = None,
    ) -> None:
        normalized_motion_args = self._normalize_script_motion_args_for_type(motion_type, motion_args)
        with self._lock:
            self._last_motion_point = ScriptMotionPoint(
                motion_type=motion_type,
                values=list(values),
                use_tool=bool(use_tool),
                motion_args=normalized_motion_args,
            )

    def _record_dispatched_script_goal(
        self,
        script_name: str,
        point_index: int,
        total_points: int,
        motion_type: str,
        values: list[float],
        use_tool: bool,
        motion_args: list[str] | None,
        action_name: str,
        point_log_message: str,
    ) -> None:
        self._record_last_motion_point(motion_type, values, use_tool, motion_args)
        with self._lock:
            start_values = self._current_motion_values_for_goal_locked(motion_type)
            self._pending_script_goals.append(
                PendingScriptGoal(
                    script_name=script_name,
                    point_index=point_index,
                    total_points=total_points,
                    motion_type=motion_type,
                    values=list(values),
                    start_values=start_values,
                )
            )
            self._script_dispatched_point_log_by_action[action_name] = point_log_message

    def _merge_script_action_result_log(self, action_call: str, script_result_text: str) -> bool:
        action_name = action_call.split(' -> ', 1)[0]
        with self._lock:
            point_log_message = self._script_dispatched_point_log_by_action.pop(action_name, None)
            if point_log_message is None:
                return False

            for index in range(len(self._script_command_log) - 1, -1, -1):
                entry = self._script_command_log[index]
                prefix, separator, message = entry.partition('] ')
                if separator and message == point_log_message:
                    if script_result_text == 'SENT':
                        merged_message = f'SENT {point_log_message}'
                    else:
                        merged_message = f'{point_log_message} | {script_result_text}'
                    self._script_command_log[index] = f'{prefix}] {merged_message}'
                    self._script_command_log_version += 1
                    return True
        return False

    def _collect_script_goal_hits_locked(self, motion_type: str, ros_sec: int, ros_nanosec: int) -> list[ScriptGoalHit]:
        if not self._pending_script_goals:
            return []

        # Preserve service-call order: only the oldest pending point can hit next.
        current_goal = self._pending_script_goals[0]
        if current_goal.motion_type != motion_type:
            return []
        if not self._script_goal_reached_locked(current_goal):
            return []

        actual_values = self._current_motion_values_for_goal_locked(current_goal.motion_type)
        if actual_values is not None and len(actual_values) == len(current_goal.values):
            captured_values: list[float] | None = list(actual_values)
        else:
            captured_values = None

        goal = self._pending_script_goals.pop(0)
        if not self._pending_script_goals and not self._script_run_active and not self._script_stop_requested:
            self._script_run_name = None
        return [
            ScriptGoalHit(
                script_name=goal.script_name,
                point_index=goal.point_index,
                total_points=goal.total_points,
                motion_type=goal.motion_type,
                values=list(goal.values),
                actual_values=captured_values,
                ros_sec=ros_sec,
                ros_nanosec=ros_nanosec,
            )
        ]

    def _script_goal_reached_locked(self, goal: PendingScriptGoal) -> bool:
        if goal.motion_type == 'MovJ':
            if self._joint_goal_reached_locked(goal.values):
                return True
        else:
            if self._tcp_goal_reached_locked(goal.values):
                return True

        completion_ratio = self._goal_progress_ratio_locked(goal)
        if completion_ratio is not None and completion_ratio >= self._script_goal_progress_ratio_threshold:
            return True
        return False

    def _current_motion_values_for_goal_locked(self, motion_type: str) -> list[float] | None:
        if motion_type == 'MovJ':
            if self._snapshot.joint_stamp is None:
                return None
            return [float(self._snapshot.joints_deg[name]) for name in JOINT_NAMES]

        if self._snapshot.tcp_stamp is None:
            return None
        return [float(self._snapshot.tcp_values[field_name]) for field_name, _, _, _ in TCP_FIELDS]

    def _goal_progress_ratio_locked(self, goal: PendingScriptGoal) -> float | None:
        if goal.start_values is None or len(goal.start_values) != len(goal.values):
            return None

        current_values = self._current_motion_values_for_goal_locked(goal.motion_type)
        if current_values is None or len(current_values) != len(goal.values):
            return None

        axis_progress_values: list[float] = []
        for index, target_value in enumerate(goal.values):
            start_value = goal.start_values[index]
            total_delta = abs(target_value - start_value)
            if total_delta <= SCRIPT_GOAL_PROGRESS_AXIS_EPSILON:
                continue

            remaining_delta = abs(target_value - current_values[index])
            axis_progress = 1.0 - (remaining_delta / total_delta)
            axis_progress_values.append(max(0.0, min(1.0, axis_progress)))

        if not axis_progress_values:
            return None

        # Use worst-axis completion so all moved axes are near the commanded goal.
        return min(axis_progress_values)

    def _joint_goal_reached_locked(self, target_values: list[float]) -> bool:
        for index, joint_name in enumerate(JOINT_NAMES):
            current_value = self._snapshot.joints_deg.get(joint_name)
            if current_value is None:
                return False
            if abs(current_value - target_values[index]) > SCRIPT_GOAL_JOINT_TOLERANCE_DEG:
                return False
        return True

    def _tcp_position_tolerance_mm_locked(self) -> float:
        # Map tolerance slider percent to linear position error:
        # 60% -> 40mm, 90% -> 10mm, 99% -> 1mm.
        threshold_percent = self._script_goal_progress_ratio_threshold * 100.0
        return max(1.0, 100.0 - threshold_percent)

    def _tcp_goal_reached_locked(self, target_values: list[float]) -> bool:
        position_tolerance_mm = self._tcp_position_tolerance_mm_locked()
        for index, (field_name, _, _, _) in enumerate(TCP_FIELDS):
            current_value = self._snapshot.tcp_values.get(field_name)
            if current_value is None:
                return False
            tolerance = (
                SCRIPT_GOAL_TCP_ROTATION_TOLERANCE_DEG
                if field_name in {'rx', 'ry', 'rz'}
                else position_tolerance_mm
            )
            if abs(current_value - target_values[index]) > tolerance:
                return False
        return True

    def _log_script_goal_hit_events(self, hit_events: list[ScriptGoalHit]) -> None:
        for hit_event in hit_events:
            ros_stamp = self._format_ros_stamp(hit_event.ros_sec, hit_event.ros_nanosec)
            hit_ros_nanos = (int(hit_event.ros_sec) * 1_000_000_000) + int(hit_event.ros_nanosec)
            with self._lock:
                if self._script_hit_zero_ros_nanos is None:
                    self._script_hit_zero_ros_nanos = hit_ros_nanos
                base_hit_nanos = self._script_hit_zero_ros_nanos

            relative_nanos = max(0, hit_ros_nanos - base_hit_nanos)
            relative_ms = int((relative_nanos + 500_000) // 1_000_000)
            target_text = self._format_script_point_values(hit_event.values)
            if hit_event.actual_values is not None:
                actual_text = self._format_script_point_values(hit_event.actual_values)
            else:
                actual_text = 'n/a'
            self._append_script_command_log(
                (
                    f'HIT POINT {hit_event.point_index}/{hit_event.total_points} {hit_event.motion_type} '
                    f'TARGET {target_text} RAW {actual_text}'
                ),
                elapsed_ms=relative_ms,
            )
            self._log_event(
                'script',
                (
                    f'goal hit "{hit_event.script_name}" point {hit_event.point_index}/{hit_event.total_points} '
                    f'{hit_event.motion_type} at {relative_ms}ms '
                    f'(ROS {ros_stamp}, raw: {actual_text}, target: {target_text})'
                ),
            )

    def _format_ros_stamp(self, sec: int, nanosec: int) -> str:
        return f'{{sec: {sec}, nanosec: {nanosec}}}'

    def _build_speed_request(self, key: str, value: int):
        if key == 'cp':
            request = CP.Request()
            request.r = value
            return self._cp_client, request, 'CP', f'{SERVICE_ROOT}/CP'
        if key == 'speed_factor':
            request = SpeedFactor.Request()
            request.ratio = value
            return self._speed_factor_client, request, 'Speed Factor', f'{SERVICE_ROOT}/SpeedFactor'
        if key == 'speed_j':
            request = VelJ.Request()
            request.r = value
            return self._vel_j_client, request, 'SpeedJ', f'{SERVICE_ROOT}/VelJ'
        if key == 'acc_j':
            request = AccJ.Request()
            request.r = value
            return self._acc_j_client, request, 'AccJ', f'{SERVICE_ROOT}/AccJ'
        if key == 'speed_l':
            request = VelL.Request()
            request.r = value
            return self._vel_l_client, request, 'SpeedL', f'{SERVICE_ROOT}/VelL'
        if key == 'acc_l':
            request = AccL.Request()
            request.r = value
            return self._acc_l_client, request, 'AccL', f'{SERVICE_ROOT}/AccL'
        return None, None, None, None

    def _joint_callback(self, msg: JointState) -> None:
        positions: dict[str, float] = {}
        for name, position in zip(msg.name, msg.position):
            positions[name] = math.degrees(position)

        now = time.time()
        ros_stamp_msg = msg.header.stamp
        if ros_stamp_msg.sec == 0 and ros_stamp_msg.nanosec == 0:
            ros_stamp_msg = self.get_clock().now().to_msg()
        hit_events: list[ScriptGoalHit] = []
        with self._lock:
            for name in JOINT_NAMES:
                if name in positions:
                    self._snapshot.joints_deg[name] = positions[name]
            self._snapshot.joint_stamp = now
            hit_events = self._collect_script_goal_hits_locked('MovJ', int(ros_stamp_msg.sec), int(ros_stamp_msg.nanosec))
        self._log_script_goal_hit_events(hit_events)

    def _tcp_callback(self, msg: ToolVectorActual) -> None:
        now = time.time()
        ros_stamp_msg = self.get_clock().now().to_msg()
        hit_events: list[ScriptGoalHit] = []
        with self._lock:
            self._raw_tcp_values['x'] = msg.x
            self._raw_tcp_values['y'] = msg.y
            self._raw_tcp_values['z'] = msg.z
            self._raw_tcp_values['rx'] = msg.rx
            self._raw_tcp_values['ry'] = msg.ry
            self._raw_tcp_values['rz'] = msg.rz
            # ToolVectorActual is the controller's live TCP. Keep the GUI aligned
            # with that feedback instead of reapplying the configured tool offset.
            self._snapshot.tcp_values = dict(self._raw_tcp_values)
            self._snapshot.tcp_stamp = now
            hit_events = self._collect_script_goal_hits_locked('MovL', int(ros_stamp_msg.sec), int(ros_stamp_msg.nanosec))
        self._log_script_goal_hit_events(hit_events)

    def _status_callback(self, msg: RobotStatus) -> None:
        now = time.time()
        with self._lock:
            self._snapshot.connected = msg.is_connected
            self._snapshot.enabled = msg.is_enable
            self._snapshot.status_stamp = now
            if msg.is_connected and msg.is_enable:
                self._snapshot.controller_mode_code = 5
                self._snapshot.drag_enabled = False
                self._snapshot.controller_text = 'Enabled'
                self._snapshot.error_text = 'No active alarms'
                self._snapshot.error_active = False
        self._dispatch_queued_service_if_ready()

    def _poll_controller_state(self) -> None:
        snapshot = self.snapshot()
        if not snapshot.connected:
            with self._lock:
                self._snapshot.controller_mode_code = None
                self._snapshot.drag_enabled = False
                self._snapshot.controller_text = 'Waiting for dashboard connection...'
                self._snapshot.error_text = 'No error info yet'
                self._snapshot.error_active = False
            return

        if snapshot.enabled is True:
            with self._lock:
                self._snapshot.controller_mode_code = 5
                self._snapshot.drag_enabled = False
                self._snapshot.controller_text = 'Enabled'
                self._snapshot.error_text = 'No active alarms'
                self._snapshot.error_active = False
            self._apply_startup_speed_factor_if_needed()
            self._apply_startup_tool_disable_if_needed()
            self._apply_startup_tool_tcp_reset_if_needed()
            self._apply_startup_cp_if_needed()
            return

        if self._robot_mode_future is None or self._robot_mode_future.done():
            if self._robot_mode_client.service_is_ready():
                self._log_event('service', f'dispatch RobotMode -> {SERVICE_ROOT}/RobotMode {{}}')
                self._robot_mode_future = self._robot_mode_client.call_async(RobotMode.Request())
                self._robot_mode_future.add_done_callback(self._handle_robot_mode)

        if snapshot.controller_mode_code == 9:
            if self._error_id_future is None or self._error_id_future.done():
                if self._get_error_id_client.service_is_ready():
                    self._log_event('service', f'dispatch GetErrorID -> {SERVICE_ROOT}/GetErrorID {{}}')
                    self._error_id_future = self._get_error_id_client.call_async(GetErrorID.Request())
                    self._error_id_future.add_done_callback(self._handle_error_id)
        else:
            with self._lock:
                self._snapshot.error_text = 'No active alarms'
                self._snapshot.error_active = False

        self._apply_startup_speed_factor_if_needed()
        self._apply_startup_tool_disable_if_needed()
        self._apply_startup_tool_tcp_reset_if_needed()
        self._apply_startup_cp_if_needed()

    def _handle_robot_mode(self, future) -> None:
        try:
            response = future.result()
            raw_mode_text = self._clean_text(response.robot_return)
            mode_code = self._extract_mode_code(raw_mode_text)
            mode_text = self._format_controller_text(raw_mode_text, response.res)
        except Exception as exc:  # pragma: no cover - defensive UI path
            mode_code = None
            mode_text = f'RobotMode failed: {exc}'
            self._log_event('service', f'RobotMode -> {SERVICE_ROOT}/RobotMode {{}}: {mode_text}')
        else:
            self._log_event(
                'service',
                f'RobotMode -> {SERVICE_ROOT}/RobotMode {{}}: ok | robot_return={raw_mode_text!r}, res={response.res}',
            )

        with self._lock:
            self._snapshot.controller_mode_code = mode_code
            self._snapshot.drag_enabled = mode_code == 6
            self._snapshot.controller_text = mode_text
        self._dispatch_queued_service_if_ready()

    def _handle_error_id(self, future) -> None:
        try:
            response = future.result()
            error_text = self._format_error_text(
                self._clean_text(response.robot_return),
                response.res,
            )
        except Exception as exc:  # pragma: no cover - defensive UI path
            error_text = f'GetErrorID failed: {exc}'
            self._log_event('service', f'GetErrorID -> {SERVICE_ROOT}/GetErrorID {{}}: {error_text}')
        else:
            self._log_event(
                'service',
                f'GetErrorID -> {SERVICE_ROOT}/GetErrorID {{}}: ok | robot_return={self._clean_text(response.robot_return)!r}, res={response.res}',
            )

        with self._lock:
            self._snapshot.error_text = error_text
            self._snapshot.error_active = self._is_error_active(error_text)

    def _send_simple_service(
        self,
        client,
        request,
        action_name: str,
        service_name: str,
        on_success=None,
        on_complete=None,
        on_dispatch=None,
        allow_local_queue: bool = True,
    ) -> bool:
        action_call = self._describe_service_call(action_name, service_name, request)
        is_script_action = self._is_script_action_call(action_call)

        if not client.service_is_ready():
            self._log_event('service', f'{action_call}: service is not ready')
            if is_script_action:
                self._append_script_command_log('NOT READY')
            self._set_action_text(f'{action_call}: service is not ready.')
            return False

        if on_dispatch is not None:
            on_dispatch()

        if allow_local_queue and self._should_queue_service(service_name):
            self._queue_service_call(client, request, action_name, service_name, action_call, on_success, on_complete)
            return True

        self._dispatch_service_call(client, request, action_call, on_success, on_complete)
        return True

    def _dispatch_service_call(self, client, request, action_call: str, on_success=None, on_complete=None) -> None:
        self._log_event('service', f'dispatch {action_call}')
        self._set_busy_action(action_call)
        future = client.call_async(request)
        future.add_done_callback(
            lambda done_future, action_call_text=action_call, success_callback=on_success, complete_callback=on_complete: self._finish_service_call(
                action_call_text,
                done_future,
                success_callback,
                complete_callback,
            )
        )

    def _queue_service_call(self, client, request, action_name: str, service_name: str, action_call: str, on_success=None, on_complete=None) -> None:
        with self._lock:
            replaced = self._queued_service_call is not None
            self._queued_service_call = QueuedServiceCall(
                client=client,
                request=request,
                action_name=action_name,
                service_name=service_name,
                action_call=action_call,
                on_success=on_success,
                on_complete=on_complete,
            )
            prefix = 'Replaced queued action with' if replaced else 'Queued next action'
            self._snapshot.action_text = f'{prefix}: {action_call}'
        self._log_event('service', f'queued {action_call}')
        if self._is_script_action_call(action_call):
            self._append_script_command_log('QUEUED')

    def _dispatch_queued_service_if_ready(self) -> None:
        with self._lock:
            if self._snapshot.busy_action is not None or self._queued_service_call is None:
                return
            queued_call = self._queued_service_call
            if self._should_hold_queued_service_locked(queued_call.service_name):
                return
            self._queued_service_call = None

        if not queued_call.client.service_is_ready():
            self._log_event('service', f'{queued_call.action_call}: queued service is not ready')
            self._set_action_text(f'{queued_call.action_call}: queued service is not ready.')
            return

        self._dispatch_service_call(
            queued_call.client,
            queued_call.request,
            queued_call.action_call,
            queued_call.on_success,
            queued_call.on_complete,
        )

    def _finish_service_call(self, action_call: str, future, on_success=None, on_complete=None) -> None:
        should_run_success = False
        should_run_complete = on_complete is not None
        script_result_text = 'ERROR'
        try:
            response = future.result()
            res = getattr(response, 'res', 0)
            robot_return = self._clean_text(getattr(response, 'robot_return', ''))
            if res == -1:
                message = f'{action_call}: failed'
                script_result_text = 'FAILED'
            else:
                message = f'{action_call}: ok'
                should_run_success = on_success is not None
                script_result_text = 'SENT'
            if robot_return:
                message = f'{message} | {robot_return}'
                if script_result_text != 'SENT':
                    script_result_text = f'{script_result_text} | {robot_return}'
        except Exception as exc:  # pragma: no cover - defensive UI path
            message = f'{action_call}: {exc}'
            script_result_text = f'ERROR | {exc}'

        with self._lock:
            self._snapshot.busy_action = None
            self._snapshot.action_text = message
        self._log_event('service', message)
        if self._is_script_action_call(action_call):
            if not self._merge_script_action_result_log(action_call, script_result_text):
                self._append_script_command_log(script_result_text)

        if should_run_success and on_success is not None:
            on_success()
        if should_run_complete and on_complete is not None:
            on_complete()
        self._dispatch_queued_service_if_ready()

    def _mark_drag_enabled(self) -> None:
        with self._lock:
            self._snapshot.drag_enabled = True

    def _mark_drag_disabled(self) -> None:
        with self._lock:
            self._snapshot.drag_enabled = False

    def _mark_use_tool_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._snapshot.use_tool_enabled = enabled

    def _mark_startup_tool_disable_applied(self) -> None:
        self._startup_tool_disable_attempted = True
        self._mark_use_tool_enabled(False)

    def _enable_after_pause(self) -> None:
        self._enable_robot_with_defaults()

    def _disable_robot_after_stop_jog(self) -> None:
        self._send_simple_service(
            self._disable_robot_client,
            DisableRobot.Request(),
            'Disable Robot',
            f'{SERVICE_ROOT}/DisableRobot',
        )

    def _clear_error_after_stop_jog(self) -> None:
        self._send_simple_service(
            self._clear_error_client,
            ClearError.Request(),
            'Recover Jog Clear Error',
            f'{SERVICE_ROOT}/ClearError',
            on_complete=self._clear_error_after_stop_jog_complete,
        )

    def _clear_error_after_stop_jog_complete(self) -> None:
        self._schedule_clear_error_estop_check()
        self._enable_after_stop_jog()

    def _enable_after_stop_jog(self) -> None:
        self._enable_robot_with_defaults()

    def _enable_robot_with_defaults(self) -> None:
        self._send_simple_service(
            self._enable_robot_client,
            EnableRobot.Request(),
            'Enable Robot',
            f'{SERVICE_ROOT}/EnableRobot',
        )

    def _mark_ee_load(self, load_kg: float) -> None:
        with self._lock:
            self._snapshot.ee_load_kg = load_kg

    def _mark_tool_tcp(self, values: list[float]) -> None:
        with self._lock:
            self._snapshot.tool_tcp_values = list(values)
            self._snapshot.tcp_values = dict(self._raw_tcp_values)

    def _apply_startup_speed_factor_if_needed(self) -> None:
        if self._startup_speed_factor_attempted:
            return

        snapshot = self.snapshot()
        controller_ready = snapshot.enabled is True or snapshot.controller_mode_code in {5, 7, 11}
        if not snapshot.connected or not controller_ready:
            return

        if not self._speed_factor_client.service_is_ready():
            return

        self._startup_speed_factor_attempted = True
        request = SpeedFactor.Request()
        request.ratio = self._startup_speed_factor_target
        self._send_simple_service(
            self._speed_factor_client,
            request,
            f'Set Speed Factor to {self._startup_speed_factor_target}%',
            f'{SERVICE_ROOT}/SpeedFactor',
        )

    def _apply_startup_cp_if_needed(self) -> None:
        if self._startup_cp_attempted:
            return

        snapshot = self.snapshot()
        controller_ready = snapshot.enabled is True or snapshot.controller_mode_code in {5, 7, 11}
        if not snapshot.connected or not controller_ready:
            return

        if snapshot.busy_action is not None:
            return

        if not self._cp_client.service_is_ready():
            return

        self._startup_cp_attempted = True
        request = CP.Request()
        request.r = self._startup_cp_target
        self._send_simple_service(
            self._cp_client,
            request,
            f'Set CP to {self._startup_cp_target}%',
            f'{SERVICE_ROOT}/CP',
            allow_local_queue=False,
        )

    def _apply_startup_tool_disable_if_needed(self) -> None:
        if self._startup_tool_disable_attempted:
            return

        snapshot = self.snapshot()
        controller_ready = snapshot.enabled is True or snapshot.controller_mode_code in {5, 7, 11}
        if not snapshot.connected or not controller_ready:
            return

        if snapshot.busy_action is not None:
            return

        if not self._tool_client.service_is_ready():
            return

        request = Tool.Request()
        request.index = 0
        self._send_simple_service(
            self._tool_client,
            request,
            'Activate Tool 0',
            f'{SERVICE_ROOT}/Tool',
            on_success=self._mark_startup_tool_disable_applied,
            allow_local_queue=False,
        )

    def _apply_startup_tool_tcp_reset_if_needed(self) -> None:
        if self._startup_tool_tcp_reset_attempted:
            return

        if not self._startup_tool_disable_attempted:
            return

        snapshot = self.snapshot()
        controller_ready = snapshot.enabled is True or snapshot.controller_mode_code in {5, 7, 11}
        if not snapshot.connected or not controller_ready:
            return

        if not self._set_tool_client.service_is_ready():
            return

        self._startup_tool_tcp_reset_attempted = True
        self._send_tool_tcp(ZERO_TOOL_TCP_VALUES, 'Set Tool 1 to 0,0,0,0,0,0')

    def _set_action_text(self, text: str) -> None:
        with self._lock:
            self._snapshot.action_text = text

    def _set_busy_action(self, action_name: str) -> None:
        with self._lock:
            self._snapshot.busy_action = action_name
            self._snapshot.action_text = f'{action_name}...'

    def _should_queue_service(self, service_name: str) -> bool:
        snapshot = self.snapshot()
        if snapshot.busy_action is not None:
            return True
        if service_name in QUEUED_MOTION_SERVICES and snapshot.controller_mode_code in {7, 11}:
            return True
        return False

    def _should_hold_queued_service_locked(self, service_name: str) -> bool:
        if service_name not in QUEUED_MOTION_SERVICES:
            return False
        return self._snapshot.controller_mode_code in {7, 11}

    def _describe_service_call(self, action_name: str, service_name: str, request) -> str:
        return f'{action_name} -> {service_name} {self._describe_request(request)}'

    def _describe_request(self, request) -> str:
        field_getter = getattr(request, 'get_fields_and_field_types', None)
        if field_getter is None:
            return '{}'

        fields = list(field_getter().keys())
        parts = []
        for field_name in fields:
            value = getattr(request, field_name)
            if field_name == 'param_value' and not value:
                continue
            parts.append(f'{field_name}: {self._format_request_value(value)}')

        if not parts:
            return '{}'

        return '{' + ', '.join(parts) + '}'

    def _format_request_value(self, value) -> str:
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if isinstance(value, float):
            return f'{value:.2f}'
        if isinstance(value, str):
            return repr(value)
        if isinstance(value, (list, tuple)):
            return '[' + ', '.join(self._format_request_value(item) for item in value) + ']'
        return str(value)

    def _is_error_active(self, error_text: str) -> bool:
        normalized = error_text.strip().lower()
        if not normalized:
            return False
        if normalized in {'{}', '[]', '0', '{0}', 'no active error', 'no active alarms'}:
            return False
        if 'control mode is not tcp' in normalized:
            return False
        return True

    def _clean_text(self, value: str) -> str:
        return value.strip().strip('\t').strip()

    def _extract_mode_code(self, raw_text: str) -> int | None:
        value = raw_text.strip()
        if value.startswith('{') and value.endswith('}') and ',' not in value:
            value = value[1:-1].strip()
        if value.isdigit():
            return int(value)
        return None

    def _robot_should_disable(self, snapshot: MotionSnapshot) -> bool:
        return snapshot.controller_mode_code in {5, 6, 7} or bool(snapshot.enabled)

    def _format_controller_text(self, raw_text: str, res: int) -> str:
        if not raw_text:
            return f'Unknown (res={res})'

        if 'control mode is not tcp' in raw_text.lower():
            return raw_text

        value = raw_text.strip()
        if value.startswith('{') and value.endswith('}') and ',' not in value:
            value = value[1:-1].strip()

        mode_map = {
            '1': 'Initializing',
            '2': 'Brake Open',
            '4': 'Disabled',
            '5': 'Enabled',
            '6': 'Drag Mode',
            '7': 'Running',
            '8': 'Recording',
            '9': 'Error',
            '10': 'Paused',
            '11': 'Jogging',
        }
        if value in mode_map:
            return f'{mode_map[value]} ({value})'

        return raw_text

    def _format_error_text(self, raw_text: str, res: int) -> str:
        if not raw_text:
            return 'No active alarms' if res == 0 else f'Unknown (res={res})'

        compact = ''.join(raw_text.split())
        if compact in {
            '{[]}',
            '{[[]]}',
            '{[],[],[],[],[],[],[]}',
            '{[],[],[],[],[],[],[],[]}',
            '{[ ],[],[],[],[],[],[]}',
            '{[],[ ],[],[],[],[],[]}',
            '{[],[],[ ],[],[],[],[]}',
            '{[],[],[],[ ],[],[],[]}',
            '{[],[],[],[],[ ],[],[]}',
            '{[],[],[],[],[],[ ],[]}',
            '{[],[],[],[],[],[],[ ]}',
        }:
            return 'No active alarms'

        if compact and compact[0] == '{' and compact[-1] == '}':
            return f'Active alarms {raw_text}'

        return raw_text


class SliderRow:
    def __init__(
        self,
        parent: tk.Misc,
        label: str,
        unit: str,
        from_: float,
        to: float,
        resolution: float = 0.1,
        control_margin_ratio: float = 0.0,
        allow_range_expand: bool = True,
        on_release=None,
    ) -> None:
        self.frame = ttk.Frame(parent)
        self.frame.columnconfigure(1, weight=1)
        self.text_var = tk.StringVar(value=f'0.0 {unit}')
        self.unit = unit
        self.base_min_value = from_
        self.base_max_value = to
        self.min_value = from_
        self.max_value = to
        self._enabled = False
        self._dragging = False
        self._control_margin_ratio = max(0.0, min(control_margin_ratio, 0.49))
        self._allow_range_expand = allow_range_expand
        self._on_release = on_release

        ttk.Label(self.frame, text=label, width=12).grid(row=0, column=0, padx=(0, 12), sticky='w')
        self.scale = tk.Scale(
            self.frame,
            from_=self.min_value,
            to=self.max_value,
            orient=tk.HORIZONTAL,
            resolution=resolution,
            showvalue=False,
            sliderlength=18,
            length=620,
            state=tk.DISABLED,
            highlightthickness=0,
            bd=0,
            command=self._on_move if on_release is not None else None,
        )
        self.scale.grid(row=0, column=1, sticky='ew')
        if on_release is not None:
            self.scale.bind('<ButtonPress-1>', self._start_drag)
            self.scale.bind('<ButtonRelease-1>', self._finish_drag)
        ttk.Label(self.frame, textvariable=self.text_var, width=14, anchor='e').grid(
            row=0, column=2, padx=(12, 0), sticky='e'
        )

    def grid(self, **kwargs) -> None:
        self.frame.grid(**kwargs)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if not enabled:
            # If controls get disabled mid-drag (busy/script/override toggle),
            # release events may be missed; force drag state to recover live updates.
            self._dragging = False
        self.scale.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def update_value(self, value: float) -> None:
        if self._dragging and self._enabled:
            return
        self._dragging = False
        self._expand_range_if_needed(value)
        self.scale.configure(state=tk.NORMAL)
        self.scale.set(value)
        self.scale.configure(state=tk.NORMAL if self._enabled else tk.DISABLED)
        self.text_var.set(f'{value:7.2f} {self.unit}')

    def _expand_range_if_needed(self, value: float) -> None:
        if not self._allow_range_expand:
            return
        if self.min_value <= value <= self.max_value:
            return

        span = max(abs(value) * 1.2, 1.0)
        self.min_value = min(self.min_value, -span)
        self.max_value = max(self.max_value, span)
        self.scale.configure(from_=self.min_value, to=self.max_value)

    def _start_drag(self, _event) -> None:
        if not self._enabled:
            self._dragging = False
            return
        self._dragging = True

    def _on_move(self, value: str) -> None:
        self.text_var.set(f'{float(value):7.2f} {self.unit}')

    def _finish_drag(self, _event) -> None:
        value = float(self.scale.get())
        clamped_value = self._clamp_control_value(value)
        if clamped_value != value:
            self.scale.set(clamped_value)
            value = clamped_value
        self._dragging = False
        self.text_var.set(f'{value:7.2f} {self.unit}')
        if self._on_release is not None:
            self._on_release(value)

    def _clamp_control_value(self, value: float) -> float:
        if self._control_margin_ratio <= 0.0:
            return value

        span = self.base_max_value - self.base_min_value
        margin = span * self._control_margin_ratio
        safe_min = self.base_min_value + margin
        safe_max = self.base_max_value - margin
        return max(safe_min, min(safe_max, value))

    def clamp_value(self, value: float) -> float:
        return self._clamp_control_value(value)


class MotionDebugApp:
    def __init__(self, node: MotionDebugNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title('DOBOT Motion Debug')
        self.root.geometry('1280x1120')
        self.root.minsize(900, 940)
        self.root.configure(bg='#f3f6fb')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._closed = False
        self._joint_override_enabled = False
        self._tcp_override_enabled = False
        self._joint_override_targets = {name: 0.0 for name in JOINT_NAMES}
        self._tcp_override_targets = {field_name: 0.0 for field_name, _, _, _ in TCP_FIELDS}
        self._last_tool_tcp_values = [0.0] * 6
        self._last_use_tool_enabled = False
        self._tcp_tool_sync_deadline: float | None = None
        self._robot_type = _load_robot_type_from_bringup_config()
        self._tcp_workspace = _workspace_profile_for_robot(self._robot_type)
        self.joint_target_input_var = tk.StringVar(value='0,0,0,0,0,0')
        self.tcp_target_input_var = tk.StringVar(value='0,0,0,0,0,0')
        self.script_name_input_var = tk.StringVar(value='')
        self.script_open_var = tk.StringVar(value='')
        self.script_status_var = tk.StringVar(value='Script: none loaded')
        self._scripts_dir = workspace_path('config', SCRIPT_DIR_NAME)
        self._current_script_name: str | None = None
        self._current_script_points: list[dict[str, object]] = []
        self._current_script_speed_profile: dict[str, int] = self.node.get_script_speed_profile_snapshot()
        self._script_names: list[str] = []
        self._last_script_log_version = -1
        self._script_goal_tolerance_percent = self.node.get_script_goal_check_tolerance_percent()
        self.script_goal_tolerance_label_var = tk.StringVar(
            value=f'{self._script_goal_tolerance_percent:.1f}%'
        )

        outer = ttk.Frame(self.root, padding=16)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=0, minsize=360)
        outer.rowconfigure(14, weight=1)

        self.script_frame = ttk.LabelFrame(outer, text='Scripts')
        self.script_frame.grid(row=0, column=1, rowspan=15, sticky='nsew', padx=(16, 0))
        self.script_frame.columnconfigure(0, weight=1)
        self.script_frame.columnconfigure(1, weight=0)

        ttk.Label(self.script_frame, text='Name').grid(row=0, column=0, sticky='w')
        self.script_name_entry = ttk.Entry(self.script_frame, textvariable=self.script_name_input_var, width=21)
        self.script_name_entry.grid(row=1, column=0, sticky='ew', padx=(0, 6), pady=(2, 6))
        self.script_create_button = ttk.Button(
            self.script_frame,
            text='Create',
            command=self._create_script_from_entry,
        )
        self.script_create_button.grid(row=1, column=1, sticky='ew', pady=(2, 6))

        ttk.Label(self.script_frame, text='Open').grid(row=2, column=0, sticky='w')
        self.script_open_combo = ttk.Combobox(
            self.script_frame,
            textvariable=self.script_open_var,
            state='readonly',
            width=21,
        )
        self.script_open_combo.grid(row=3, column=0, sticky='ew', padx=(0, 6), pady=(2, 6))
        self.script_open_button = ttk.Button(
            self.script_frame,
            text='Open',
            command=self._open_selected_script,
        )
        self.script_open_button.grid(row=3, column=1, sticky='ew', pady=(2, 6))
        self.script_open_editor_button = ttk.Button(
            self.script_frame,
            text='Open in Editor',
            command=self._open_script_in_editor,
        )
        self.script_open_editor_button.grid(row=4, column=1, sticky='ew', pady=(0, 6))

        ttk.Label(
            self.script_frame,
            textvariable=self.script_status_var,
            justify=tk.LEFT,
            wraplength=300,
        ).grid(row=4, column=0, sticky='w', pady=(0, 6))

        self.script_add_actions_frame = ttk.Frame(self.script_frame)
        self.script_add_actions_frame.grid(row=7, column=0, columnspan=2, sticky='ew', pady=(0, 6))
        self.script_add_actions_frame.columnconfigure(0, weight=1, uniform='script_add_buttons')
        self.script_add_actions_frame.columnconfigure(1, weight=1, uniform='script_add_buttons')

        self.script_add_point_button = ttk.Button(
            self.script_add_actions_frame,
            text='Add Last Movement',
            command=self._add_last_motion_point_to_script,
            width=18,
        )
        self.script_add_point_button.grid(row=0, column=0, sticky='ew', padx=(0, 3))

        self.script_add_position_button = ttk.Button(
            self.script_add_actions_frame,
            text='Add Joint Position',
            command=self._add_current_joint_position_to_script,
            width=18,
        )
        self.script_add_position_button.grid(row=0, column=1, sticky='ew', padx=(3, 0))

        self.script_delete_button = ttk.Button(
            self.script_frame,
            text='Delete',
            command=self._delete_selected_script,
        )
        self.script_delete_button.grid(row=6, column=1, sticky='ew', pady=(0, 6))

        self.script_run_button = tk.Button(
            self.script_frame,
            text='Run Script',
            command=self._run_loaded_script,
            bg='#eef1f6',
            activebackground='#e3e7ee',
            fg='#1f2937',
            activeforeground='#1f2937',
            disabledforeground='#7a8088',
            relief=tk.RAISED,
            bd=1,
        )
        self.script_run_button.grid(row=8, column=0, columnspan=2, sticky='ew')

        self.script_goal_tolerance_frame = ttk.Frame(self.script_frame)
        self.script_goal_tolerance_frame.grid(row=9, column=0, columnspan=2, sticky='ew', pady=(6, 0))
        self.script_goal_tolerance_frame.columnconfigure(0, weight=1)
        ttk.Label(self.script_goal_tolerance_frame, text='Goal Check Tolerance').grid(
            row=0, column=0, sticky='w'
        )
        ttk.Label(
            self.script_goal_tolerance_frame,
            textvariable=self.script_goal_tolerance_label_var,
            width=8,
            anchor='e',
        ).grid(row=0, column=1, sticky='e')
        self.script_goal_tolerance_scale = tk.Scale(
            self.script_goal_tolerance_frame,
            from_=SCRIPT_GOAL_PROGRESS_RATIO_MIN_PERCENT,
            to=SCRIPT_GOAL_PROGRESS_RATIO_MAX_PERCENT,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            showvalue=False,
            sliderlength=18,
            length=240,
            highlightthickness=0,
            bd=0,
            command=self._on_script_goal_tolerance_change,
        )
        self.script_goal_tolerance_scale.set(self._script_goal_tolerance_percent)
        self.script_goal_tolerance_scale.grid(row=1, column=0, columnspan=2, sticky='ew')

        ttk.Label(self.script_frame, text='Script Datalog').grid(
            row=11,
            column=0,
            columnspan=2,
            sticky='w',
            pady=(8, 4),
        )
        self.script_log_frame = ttk.Frame(self.script_frame)
        self.script_log_frame.grid(row=12, column=0, columnspan=2, sticky='nsew')
        self.script_log_frame.columnconfigure(0, weight=1)
        self.script_log_frame.rowconfigure(0, weight=1)
        self.script_frame.rowconfigure(12, weight=1)

        self.script_log_text = tk.Text(
            self.script_log_frame,
            height=16,
            width=46,
            wrap=tk.WORD,
            state=tk.DISABLED,
            font=('TkFixedFont', 9),
        )
        self.script_log_text.grid(row=0, column=0, sticky='nsew')
        self.script_log_scrollbar = ttk.Scrollbar(
            self.script_log_frame,
            orient=tk.VERTICAL,
            command=self.script_log_text.yview,
        )
        self.script_log_scrollbar.grid(row=0, column=1, sticky='ns')
        self.script_log_text.configure(yscrollcommand=self.script_log_scrollbar.set)
        self.script_log_text.bind('<Button-1>', lambda _event: self.script_log_text.focus_set())
        self.script_log_text.bind('<Control-c>', self._copy_script_datalog)
        self.script_log_text.bind('<<Copy>>', self._copy_script_datalog)
        self.script_log_actions_frame = ttk.Frame(self.script_frame)
        self.script_log_actions_frame.grid(row=13, column=0, columnspan=2, sticky='e', pady=(6, 0))
        self.script_copy_log_button = ttk.Button(
            self.script_log_actions_frame,
            text='Copy Datalog',
            command=self._copy_script_datalog,
            width=12,
        )
        self.script_copy_log_button.grid(row=0, column=0, padx=(0, 6))
        self.script_clear_log_button = ttk.Button(
            self.script_log_actions_frame,
            text='Clear Datalog',
            command=self._clear_script_datalog,
            width=12,
        )
        self.script_clear_log_button.grid(row=0, column=1)

        self._ensure_script_dir()
        self._refresh_script_options()
        self._refresh_script_datalog()

        ttk.Label(outer, text='Robot Status', font=('TkDefaultFont', 14, 'bold')).grid(
            row=0, column=0, sticky='w'
        )
        self.status_frame = ttk.Frame(outer, padding=(0, 10, 0, 10))
        self.status_frame.grid(row=1, column=0, sticky='ew')
        self.status_frame.columnconfigure(0, weight=1)
        self.status_frame.columnconfigure(1, weight=1)
        self.status_frame.columnconfigure(2, weight=1)

        self.connected_value = tk.StringVar(value='Waiting for status...')
        self.robot_status_value = tk.StringVar(value='Waiting for status...')
        self.health_value = tk.StringVar(value='No data yet')

        self.connected_badge = self._make_badge(self.status_frame, 'Connected', self.connected_value, 0)
        self.robot_status_badge = self._make_badge(self.status_frame, 'Robot Status', self.robot_status_value, 1)
        self.health_badge = self._make_badge(self.status_frame, 'Stream Health', self.health_value, 2)

        self.controller_var = tk.StringVar(value='Controller: waiting...')
        self.error_var = tk.StringVar(value='Error: waiting...')
        self.action_var = tk.StringVar(value='Last Action: Ready')

        ttk.Label(outer, textvariable=self.controller_var).grid(row=2, column=0, sticky='w', pady=(0, 2))

        self.debug_frame = ttk.Frame(outer)
        self.debug_frame.grid(row=3, column=0, sticky='ew', pady=(4, 8))
        self.debug_frame.columnconfigure(0, weight=1)
        self.debug_frame.columnconfigure(1, weight=1)
        self.debug_frame.columnconfigure(2, weight=1)

        self.clear_error_button = ttk.Button(self.debug_frame, text='Clear Error', command=self.node.clear_error)
        self.clear_error_button.grid(row=0, column=0, sticky='ew', padx=(0, 8))

        self.enable_button = ttk.Button(self.debug_frame, text='Enable Robot', command=self.node.toggle_enable)
        self.enable_button.grid(row=0, column=1, sticky='ew', padx=4)

        self.drag_button = ttk.Button(self.debug_frame, text='Enable Drag', command=self.node.toggle_drag)
        self.drag_button.grid(row=0, column=2, sticky='ew', padx=(8, 0))

        ttk.Label(self.debug_frame, textvariable=self.error_var).grid(
            row=1, column=0, columnspan=3, sticky='w', pady=(4, 0)
        )

        self.ee_load_var = tk.StringVar(value='EE Load: Not set')
        self.tool_tcp_var = tk.StringVar(value='Offset TCP: 0,0,0,0,0,0')
        self.ee_load_input_var = tk.StringVar(value='0.0')
        self.tool_tcp_input_var = tk.StringVar(value='0,0,0,0,0,0')
        self.ee_load_frame = ttk.Frame(outer)
        self.ee_load_frame.grid(row=4, column=0, sticky='ew', pady=(0, 4))
        self.ee_load_frame.columnconfigure(4, weight=1)
        ttk.Label(self.ee_load_frame, text='Load (kg)').grid(row=0, column=0, sticky='w', padx=(0, 8))
        self.ee_load_entry = ttk.Entry(self.ee_load_frame, textvariable=self.ee_load_input_var, width=10)
        self.ee_load_entry.grid(row=0, column=1, sticky='w')
        self.ee_load_button = ttk.Button(self.ee_load_frame, text='Set EE Load', command=self._set_ee_load_from_entry)
        self.ee_load_button.grid(row=0, column=2, sticky='w', padx=(8, 20))
        ttk.Label(
            self.ee_load_frame,
            text='Offset TCP: x, y, z, rx, ry, rz (mm, deg)',
        ).grid(row=0, column=3, sticky='w', padx=(0, 8))
        self.tool_tcp_entry = ttk.Entry(self.ee_load_frame, textvariable=self.tool_tcp_input_var, width=26)
        self.tool_tcp_entry.grid(row=0, column=4, sticky='ew')
        self.tool_tcp_button = ttk.Button(self.ee_load_frame, text='Set Tool', command=self._set_tool_tcp_from_entry)
        self.tool_tcp_button.grid(row=0, column=5, sticky='w', padx=(8, 0))
        ttk.Label(self.ee_load_frame, textvariable=self.ee_load_var).grid(row=1, column=0, sticky='w', pady=(4, 0))
        ttk.Label(self.ee_load_frame, textvariable=self.tool_tcp_var).grid(
            row=1, column=3, columnspan=3, sticky='w', pady=(4, 0)
        )
        ttk.Label(
            outer,
            textvariable=self.action_var,
            font=('TkDefaultFont', 10, 'bold'),
            wraplength=920,
            justify=tk.LEFT,
        ).grid(
            row=5, column=0, sticky='w', pady=(6, 0)
        )

        ttk.Separator(outer, orient=tk.HORIZONTAL).grid(row=6, column=0, sticky='ew', pady=10)

        ttk.Label(outer, text='Speed Status', font=('TkDefaultFont', 14, 'bold')).grid(
            row=7, column=0, sticky='w', pady=(4, 6)
        )
        speed_frame = ttk.Frame(outer)
        speed_frame.grid(row=8, column=0, sticky='ew')
        speed_frame.columnconfigure(0, weight=1)
        self.speed_rows = {}
        for row_index, (key, label, unit, low, high) in enumerate(SPEED_FIELDS):
            row = SliderRow(
                speed_frame,
                label,
                unit,
                low,
                high,
                resolution=1.0,
                on_release=lambda value, speed_key=key: self.node.set_speed_profile_value(speed_key, int(round(value))),
            )
            row.grid(row=row_index, column=0, sticky='ew', pady=4)
            self.speed_rows[key] = row

        ttk.Separator(outer, orient=tk.HORIZONTAL).grid(row=9, column=0, sticky='ew', pady=12)

        self.joint_header = ttk.Frame(outer)
        self.joint_header.grid(row=10, column=0, sticky='ew')
        self.joint_header.columnconfigure(2, minsize=350)
        ttk.Label(self.joint_header, text='Joint Status', font=('TkDefaultFont', 14, 'bold')).grid(
            row=0, column=0, sticky='w'
        )
        self.joint_override_button = tk.Button(
            self.joint_header,
            text='Override Off',
            command=self._toggle_joint_override,
            padx=12,
            pady=4,
            relief=tk.RAISED,
            bd=1,
            highlightthickness=0,
        )
        self.joint_override_button.grid(row=0, column=1, sticky='w', padx=(12, 0))
        self.joint_target_entry_frame = ttk.Frame(self.joint_header, width=350, height=30)
        self.joint_target_entry_frame.grid(row=0, column=2, sticky='ew', padx=(12, 8))
        self.joint_target_entry_frame.grid_propagate(False)
        self.joint_target_entry = ttk.Entry(
            self.joint_target_entry_frame,
            textvariable=self.joint_target_input_var,
        )
        self.joint_target_entry.pack(fill=tk.BOTH, expand=True)
        self.joint_target_go_button = ttk.Button(
            self.joint_header,
            text='Go',
            command=self._send_joint_entry_target,
        )
        self.joint_target_go_button.grid(row=0, column=3, sticky='w')

        joint_frame = ttk.Frame(outer)
        joint_frame.grid(row=11, column=0, sticky='ew', pady=(6, 0))
        joint_frame.columnconfigure(0, weight=1)
        self.joint_rows = {}
        for row_index, name in enumerate(JOINT_NAMES):
            row = SliderRow(
                joint_frame,
                name.upper(),
                'deg',
                -360.0,
                360.0,
                resolution=0.1,
                control_margin_ratio=0.01,
                on_release=lambda value, joint_name=name: self._send_joint_override(joint_name, value),
            )
            row.grid(row=row_index, column=0, sticky='ew', pady=4)
            self.joint_rows[name] = row

        ttk.Separator(outer, orient=tk.HORIZONTAL).grid(row=12, column=0, sticky='ew', pady=12)

        self.tcp_header = ttk.Frame(outer)
        self.tcp_header.grid(row=13, column=0, sticky='ew')
        self.tcp_header.columnconfigure(2, minsize=350)
        ttk.Label(self.tcp_header, text='TCP Status', font=('TkDefaultFont', 14, 'bold')).grid(
            row=0, column=0, sticky='w'
        )
        self.tcp_override_button = tk.Button(
            self.tcp_header,
            text='Override Off',
            command=self._toggle_tcp_override,
            padx=12,
            pady=4,
            relief=tk.RAISED,
            bd=1,
            highlightthickness=0,
        )
        self.tcp_override_button.grid(row=0, column=1, sticky='w', padx=(12, 0))
        self.tcp_target_entry_frame = ttk.Frame(self.tcp_header, width=350, height=30)
        self.tcp_target_entry_frame.grid(row=0, column=2, sticky='ew', padx=(12, 8))
        self.tcp_target_entry_frame.grid_propagate(False)
        self.tcp_target_entry = ttk.Entry(
            self.tcp_target_entry_frame,
            textvariable=self.tcp_target_input_var,
        )
        self.tcp_target_entry.pack(fill=tk.BOTH, expand=True)
        self.tcp_target_go_button = ttk.Button(
            self.tcp_header,
            text='Go',
            command=self._send_tcp_entry_target,
        )
        self.tcp_target_go_button.grid(row=0, column=3, sticky='w')
        self.use_tool_button = tk.Button(
            self.tcp_header,
            text='Use Tool',
            command=self._toggle_use_tool,
            padx=12,
            pady=4,
            relief=tk.RAISED,
            bd=1,
            highlightthickness=0,
        )
        self.use_tool_button.grid(row=0, column=4, sticky='w', padx=(8, 0))

        tcp_frame = ttk.Frame(outer)
        tcp_frame.grid(row=14, column=0, sticky='ew', pady=(6, 0))
        tcp_frame.columnconfigure(0, weight=1)
        self.tcp_rows = {}
        for row_index, (field_name, label, unit, limits) in enumerate(TCP_FIELDS):
            field_limits = self._tcp_workspace.limits.get(field_name, limits)
            row = SliderRow(
                tcp_frame,
                label,
                unit,
                field_limits[0],
                field_limits[1],
                resolution=0.1,
                allow_range_expand=field_name not in {'x', 'y'},
                on_release=lambda value, tcp_field=field_name: self._send_tcp_override(tcp_field, value),
            )
            row.grid(row=row_index, column=0, sticky='ew', pady=4)
            self.tcp_rows[field_name] = row

    def run(self) -> None:
        self._refresh()
        self.root.mainloop()

    def _make_badge(self, parent: tk.Misc, title: str, value_var: tk.StringVar, column: int) -> tk.Label:
        frame = ttk.Frame(parent, relief=tk.FLAT, padding=8)
        frame.grid(row=0, column=column, sticky='ew', padx=(0 if column == 0 else 8, 0))
        ttk.Label(frame, text=title, font=('TkDefaultFont', 10, 'bold')).pack(anchor='w')
        label = tk.Label(
            frame,
            textvariable=value_var,
            anchor='w',
            padx=10,
            pady=8,
            bg='#d5dce7',
            fg='#0d1b2a',
        )
        label.pack(fill=tk.X, expand=True, pady=(6, 0))
        return label

    def _toggle_joint_override(self) -> None:
        if not self._override_available(self.node.snapshot()):
            self._joint_override_enabled = False
            self.node._set_action_text('Joint override is only available when Robot Status is Enabled.')
            return
        self._joint_override_enabled = not self._joint_override_enabled
        if self._joint_override_enabled:
            self._tcp_override_enabled = False
            snapshot = self.node.snapshot()
            self._joint_override_targets = dict(snapshot.joints_deg)
            self._sync_joint_entry_to_targets()

    def _toggle_tcp_override(self) -> None:
        snapshot = self.node.snapshot()
        if not self._override_available(snapshot):
            self._tcp_override_enabled = False
            self.node._set_action_text('TCP override is only available when Robot Status is Enabled.')
            return
        self._tcp_override_enabled = not self._tcp_override_enabled
        if self._tcp_override_enabled:
            self._joint_override_enabled = False
            self._tcp_override_targets = self._clamp_tcp_targets(dict(snapshot.tcp_values))
            self._sync_tcp_entry_to_targets()
        elif snapshot.use_tool_enabled:
            # Turning TCP override off should always leave tool mode disabled.
            self._tcp_tool_sync_deadline = time.time() + 1.0
            self.node.toggle_use_tool()

    def _toggle_use_tool(self) -> None:
        snapshot = self.node.snapshot()
        if not self._tcp_override_enabled or not self._override_available(snapshot) or snapshot.busy_action:
            self.node._set_action_text('Use Tool is only available when TCP override is enabled.')
            return
        self._tcp_tool_sync_deadline = time.time() + 1.0
        self.node.toggle_use_tool()

    def _send_joint_override(self, joint_name: str, value: float) -> None:
        if not self._joint_override_enabled:
            self.node._set_action_text('Joint override is off. Enable Override to send joint commands.')
            return
        self._joint_override_targets[joint_name] = value
        self._sync_joint_entry_to_targets()
        self.node.move_joint_target(dict(self._joint_override_targets))

    def _send_tcp_override(self, field_name: str, value: float) -> None:
        if not self._tcp_override_enabled:
            self.node._set_action_text('TCP override is off. Enable Override to send Cartesian commands.')
            return
        self._tcp_override_targets[field_name] = value
        self._tcp_override_targets = self._clamp_tcp_targets(self._tcp_override_targets)
        for tcp_field_name, row in self.tcp_rows.items():
            row.update_value(self._tcp_override_targets[tcp_field_name])
        self._sync_tcp_entry_to_targets()
        self.node.move_tcp_target(dict(self._tcp_override_targets))

    def _send_joint_entry_target(self) -> None:
        if not self._joint_override_enabled:
            self.node._set_action_text('Joint override is off. Enable Override to send joint commands.')
            return

        values = self._parse_target_values(
            self.joint_target_input_var.get(),
            'Joint target must be 6 comma-separated degree values like 0,0,0,0,0,0.',
        )
        if values is None:
            return

        self._joint_override_targets = {
            joint_name: self.joint_rows[joint_name].clamp_value(value)
            for joint_name, value in zip(JOINT_NAMES, values)
        }
        for joint_name, row in self.joint_rows.items():
            row.update_value(self._joint_override_targets[joint_name])
        self._sync_joint_entry_to_targets()
        self.node.move_joint_target(dict(self._joint_override_targets))

    def _send_tcp_entry_target(self) -> None:
        if not self._tcp_override_enabled:
            self.node._set_action_text('TCP override is off. Enable Override to send Cartesian commands.')
            return

        values = self._parse_target_values(
            self.tcp_target_input_var.get(),
            'TCP target must be 6 comma-separated values like x,y,z,rx,ry,rz.',
        )
        if values is None:
            return

        self._tcp_override_targets = self._clamp_tcp_targets(
            {field_name: value for (field_name, _, _, _), value in zip(TCP_FIELDS, values)}
        )
        for tcp_field_name, row in self.tcp_rows.items():
            row.update_value(self._tcp_override_targets[tcp_field_name])
        self._sync_tcp_entry_to_targets()
        self.node.move_tcp_target(dict(self._tcp_override_targets))

    def _parse_target_values(self, raw_text: str, error_text: str) -> list[float] | None:
        parts = [part.strip() for part in raw_text.split(',')]
        if len(parts) != 6 or any(not part for part in parts):
            self.node._set_action_text(error_text)
            return None

        try:
            return [float(part) for part in parts]
        except ValueError:
            self.node._set_action_text(error_text)
            return None

    def _sync_joint_entry_to_targets(self) -> None:
        self.joint_target_input_var.set(
            ','.join(self._format_target_value(self._joint_override_targets[joint_name]) for joint_name in JOINT_NAMES)
        )

    def _sync_tcp_entry_to_targets(self) -> None:
        self.tcp_target_input_var.set(
            ','.join(
                self._format_target_value(self._tcp_override_targets[field_name])
                for field_name, _, _, _ in TCP_FIELDS
            )
        )

    def _format_target_value(self, value: float) -> str:
        formatted = f'{value:.2f}'.rstrip('0').rstrip('.')
        if formatted == '-0':
            return '0'
        return formatted or '0'

    def _set_ee_load_from_entry(self) -> None:
        try:
            load_kg = float(self.ee_load_input_var.get().strip())
        except ValueError:
            self.node._set_action_text('EE Load must be a valid number in kilograms.')
            return

        if load_kg < 0.0:
            self.node._set_action_text('EE Load must be zero or greater.')
            return

        self.node.set_ee_load(load_kg)

    def _set_tool_tcp_from_entry(self) -> None:
        values = self._parse_target_values(
            self.tool_tcp_input_var.get(),
            'TCP must be 6 comma-separated values like x,y,z,rx,ry,rz.',
        )
        if values is None:
            return

        self.tool_tcp_input_var.set(','.join(self._format_target_value(value) for value in values))
        self.node.set_tool_tcp(values)

    def _ensure_script_dir(self) -> None:
        self._scripts_dir.mkdir(parents=True, exist_ok=True)

    def _refresh_script_options(self, preferred_name: str | None = None) -> None:
        self._script_names = self._list_script_names()
        self.script_open_combo.configure(values=self._script_names)
        selected_name = preferred_name or self._current_script_name or self.script_open_var.get().strip()
        if selected_name in self._script_names:
            self.script_open_var.set(selected_name)
        elif self._script_names:
            self.script_open_var.set(self._script_names[0])
        else:
            self.script_open_var.set('')
        self._update_script_status_text()

    def _refresh_script_datalog(self, force: bool = False) -> None:
        log_version, log_lines = self.node.get_script_command_log()
        if not force and log_version == self._last_script_log_version:
            return

        self._last_script_log_version = log_version
        self.script_log_text.configure(state=tk.NORMAL)
        self.script_log_text.delete('1.0', tk.END)
        if log_lines:
            self.script_log_text.insert(tk.END, '\n'.join(log_lines))
        else:
            self.script_log_text.insert(tk.END, 'No script commands yet.')
        self.script_log_text.configure(state=tk.DISABLED)
        self.script_log_text.see(tk.END)

    def _copy_script_datalog(self, _event=None) -> str:
        try:
            copied_text = self.script_log_text.get('sel.first', 'sel.last')
        except tk.TclError:
            copied_text = self.script_log_text.get('1.0', tk.END).rstrip('\n')

        copied_text = copied_text.rstrip('\n')
        if not copied_text:
            self.node._set_action_text('Script datalog is empty.')
            return 'break'

        self.root.clipboard_clear()
        self.root.clipboard_append(copied_text)
        self.node._set_action_text('Script datalog copied.')
        return 'break'

    def _clear_script_datalog(self) -> None:
        self.node.clear_script_command_log()
        self._refresh_script_datalog(force=True)
        self.node._set_action_text('Script datalog cleared.')

    def _list_script_names(self) -> list[str]:
        if not self._scripts_dir.exists():
            return []
        names: list[str] = []
        for script_path in sorted(self._scripts_dir.glob(f'*{SCRIPT_FILE_SUFFIX}')):
            if script_path.is_file():
                names.append(script_path.stem)
        return names

    def _script_path(self, script_name: str) -> Path:
        return self._scripts_dir / f'{script_name}{SCRIPT_FILE_SUFFIX}'

    def _validate_script_name(self, raw_name: str) -> str | None:
        normalized_name = ' '.join(raw_name.strip().split())
        if not normalized_name:
            self.node._set_action_text('Script name is required.')
            return None
        if SCRIPT_NAME_PATTERN.fullmatch(normalized_name) is None:
            self.node._set_action_text('Script name must start with letter/number and use letters, numbers, space, _, -.')
            return None
        return normalized_name

    def _normalize_script_point(self, point: object) -> dict[str, object] | None:
        if isinstance(point, str):
            return self._parse_script_point_command(point)

        if not isinstance(point, dict):
            return None
        command = point.get('command')
        if isinstance(command, str):
            parsed_point = self._parse_script_point_command(
                command,
                use_tool_hint=point.get('use_tool'),
                motion_args_hint=point.get('motion_args'),
            )
            if parsed_point is not None:
                return parsed_point

        motion_type_raw = str(point.get('motion_type', '')).strip().lower()
        if motion_type_raw == 'movj':
            motion_type = 'MovJ'
        elif motion_type_raw in {'movl', 'moll'}:
            motion_type = 'MovL'
        else:
            motion_type = self._motion_type_from_mode_value(point.get('mode'))
            if motion_type is None:
                return None

        field_names = self._script_point_field_names(motion_type)
        values = point.get('values')
        if not isinstance(values, list) or len(values) != 6:
            named_values = point.get('named_values')
            if isinstance(named_values, dict):
                values = [named_values.get(name) for name in field_names]
            else:
                values = [point.get(name) for name in field_names]

        if len(values) != 6:
            return None
        try:
            numeric_values = [float(value) for value in values]
        except (TypeError, ValueError):
            return None
        use_tool = point.get('use_tool', False)
        if isinstance(use_tool, str):
            use_tool_normalized = use_tool.strip().lower()
            use_tool = use_tool_normalized in {'1', 'true', 'yes', 'on'}
        else:
            use_tool = bool(use_tool)
        motion_args = self.node._normalize_script_motion_args_for_type(
            motion_type,
            point.get('motion_args'),
        )
        return {
            'motion_type': motion_type,
            'values': numeric_values,
            'use_tool': use_tool,
            'motion_args': motion_args,
        }

    def _motion_type_from_mode_value(self, mode_value) -> str | None:
        if isinstance(mode_value, bool):
            return 'MovJ' if mode_value else 'MovL'
        if isinstance(mode_value, (int, float)):
            return 'MovJ' if mode_value != 0 else 'MovL'
        if isinstance(mode_value, str):
            normalized = mode_value.strip().lower()
            if normalized in {'true', '1', 'joint', 'movj'}:
                return 'MovJ'
            if normalized in {'false', '0', 'cartesian', 'movl'}:
                return 'MovL'
        return None

    def _mode_value_from_motion_type(self, motion_type: str) -> bool:
        return motion_type == 'MovJ'

    def _script_point_field_names(self, motion_type: str) -> list[str]:
        if motion_type == 'MovJ':
            return JOINT_NAMES
        return [field_name for field_name, _, _, _ in TCP_FIELDS]

    def _parse_script_point_command(
        self,
        command_text: str,
        use_tool_hint=None,
        motion_args_hint=None,
    ) -> dict[str, object] | None:
        text = command_text.strip()
        if not text:
            return None

        match = SCRIPT_POINT_PATTERN.fullmatch(text)
        if match is None:
            return None

        motion_type = self.node._normalize_motion_type(match.group(1))
        if motion_type is None:
            return None

        raw_segments = [segment.strip() for segment in match.group(2).split(',')]
        if len(raw_segments) < 6:
            return None

        raw_values = raw_segments[:6]
        trailing_segments = raw_segments[6:]

        try:
            numeric_values = [float(value) for value in raw_values]
        except (TypeError, ValueError):
            return None

        use_tool_from_command = False
        use_tool_command_seen = False
        command_motion_tokens: list[str] = []
        for trailing in trailing_segments:
            token = self.node._normalize_motion_arg_token(trailing)
            if token is None:
                continue
            key, raw_value = token.split('=', 1)
            if key == 'tool':
                if raw_value in {'1', 'true', 'on', 'yes'}:
                    use_tool_from_command = True
                    use_tool_command_seen = True
                elif raw_value in {'0', 'false', 'off', 'no'}:
                    use_tool_from_command = False
                    use_tool_command_seen = True
                continue
            command_motion_tokens.append(token)

        if use_tool_hint is None:
            use_tool = use_tool_from_command if use_tool_command_seen else False
        else:
            use_tool = self.node._normalize_use_tool_flag(use_tool_hint)

        motion_args_from_command = self.node._normalize_script_motion_args_for_type(
            motion_type,
            command_motion_tokens,
        )
        if motion_args_hint is None:
            motion_args = motion_args_from_command
        else:
            motion_args = self.node._normalize_script_motion_args_for_type(
                motion_type,
                motion_args_hint,
            )

        return {
            'motion_type': motion_type,
            'values': numeric_values,
            'use_tool': use_tool,
            'motion_args': motion_args,
        }

    def _format_script_point_preview(self, point: dict[str, object]) -> str:
        values = point.get('values', [])
        if not isinstance(values, list):
            values = []
        values_text = ','.join(f'{float(value):.3f}' for value in values)
        motion_type = self.node._normalize_motion_type(str(point.get('motion_type', 'MovJ'))) or 'MovJ'
        use_tool = self.node._normalize_use_tool_flag(point.get('use_tool', False))
        motion_args = self.node._normalize_script_motion_args_for_type(
            motion_type,
            point.get('motion_args'),
        )
        param_tokens = self.node._build_motion_param_value_list(use_tool, motion_args)
        param_suffix = ''
        if param_tokens:
            param_suffix = ', ' + ', '.join(param_tokens)
        return f'{motion_type}: {values_text}{param_suffix}'

    def _save_loaded_script(self) -> bool:
        if self._current_script_name is None:
            self.node._set_action_text('No script loaded.')
            return False

        normalized_points: list[dict[str, object]] = []
        for point in self._current_script_points:
            normalized_point = self._normalize_script_point(point)
            if normalized_point is not None:
                normalized_points.append(normalized_point)

        serialized_points: list[dict[str, object]] = []
        for point in normalized_points:
            motion_type = str(point.get('motion_type', 'MovJ'))
            values = point.get('values', [])
            if not isinstance(values, list):
                values = []
            numeric_values = [round(float(value), 3) for value in values]
            use_tool = bool(point.get('use_tool', False))
            motion_args = self.node._normalize_script_motion_args_for_type(
                motion_type,
                point.get('motion_args'),
            )
            preview_line = self._format_script_point_preview(
                {
                    'motion_type': motion_type,
                    'values': numeric_values,
                    'use_tool': use_tool,
                    'motion_args': motion_args,
                },
            )
            serialized_points.append(
                {
                    'mode': self._mode_value_from_motion_type(motion_type),
                    'command': preview_line,
                    'use_tool': use_tool,
                }
            )

        script_speed_profile = self.node.get_script_speed_profile_snapshot()
        self._current_script_speed_profile = dict(script_speed_profile)
        payload = {
            'name': self._current_script_name,
            'format_version': 3,
            'updated_at': datetime.now().astimezone().isoformat(timespec='seconds'),
            'speed_profile': {
                'cp': script_speed_profile['cp'],
                'speed_factor': script_speed_profile['speed_factor'],
            },
            'points': serialized_points,
        }
        script_path = self._script_path(self._current_script_name)
        try:
            with open(script_path, 'w', encoding='utf-8') as script_file:
                json.dump(payload, script_file, indent=2)
        except OSError as exc:
            self.node._set_action_text(f'Failed to save script "{self._current_script_name}": {exc}')
            return False

        self._current_script_points = normalized_points
        self._refresh_script_options(preferred_name=self._current_script_name)
        return True

    def _update_script_status_text(self) -> None:
        if self._current_script_name is None:
            self.script_status_var.set('Script: none loaded')
            return
        cp_value = self._current_script_speed_profile.get('cp', DEFAULT_SPEED_VALUES['cp'])
        sf_value = self._current_script_speed_profile.get('speed_factor', DEFAULT_SPEED_VALUES['speed_factor'])
        self.script_status_var.set(
            f'Script: {self._current_script_name} ({len(self._current_script_points)} point(s)) | '
            f'CP {cp_value}% SF {sf_value}%'
        )

    def _create_script_from_entry(self) -> None:
        script_name = self._validate_script_name(self.script_name_input_var.get())
        if script_name is None:
            return

        script_path = self._script_path(script_name)
        if script_path.exists():
            self.node._set_action_text(f'Script "{script_name}" already exists. Open it from the dropdown.')
            self._refresh_script_options(preferred_name=script_name)
            return

        self._current_script_name = script_name
        self._current_script_points = []
        self._current_script_speed_profile = self.node.get_script_speed_profile_snapshot()
        self.script_name_input_var.set(script_name)
        if not self._save_loaded_script():
            return
        self.node._set_action_text(f'Created script "{script_name}".')

    def _open_selected_script(self) -> None:
        selected_name = self.script_open_var.get().strip()
        if not selected_name:
            self.node._set_action_text('Select a script from the dropdown to open.')
            return
        self._load_script_by_name(selected_name)

    def _open_script_in_editor(self) -> None:
        selected_name = self.script_open_var.get().strip() or self._current_script_name
        if not selected_name:
            self.node._set_action_text('Select a script from the dropdown to open in editor.')
            return

        script_path = self._script_path(selected_name)
        if not script_path.exists():
            self.node._set_action_text(f'Script "{selected_name}" does not exist.')
            self._refresh_script_options()
            return

        commands: list[list[str]] = []
        script_path_text = str(script_path)
        if sys.platform.startswith('win'):
            commands.append(['notepad.exe', script_path_text])
        elif sys.platform == 'darwin':
            commands.append(['open', '-e', script_path_text])
            commands.append(['open', script_path_text])
        else:
            for editor_name in ('gedit', 'xed', 'kate', 'mousepad', 'pluma', 'code'):
                editor_path = shutil.which(editor_name)
                if editor_path:
                    commands.append([editor_path, script_path_text])
            xdg_open_path = shutil.which('xdg-open')
            if xdg_open_path:
                commands.append([xdg_open_path, script_path_text])

        for command in commands:
            try:
                subprocess.Popen(command)
                self.node._set_action_text(f'Opened script "{selected_name}" in editor.')
                return
            except OSError:
                continue

        self.node._set_action_text('Could not launch editor. Install gedit/xed or enable xdg-open.')

    def _load_script_by_name(self, script_name: str) -> None:
        script_path = self._script_path(script_name)
        if not script_path.exists():
            self.node._set_action_text(f'Script "{script_name}" does not exist.')
            self._refresh_script_options()
            return

        try:
            with open(script_path, 'r', encoding='utf-8') as script_file:
                payload = json.load(script_file)
        except (OSError, json.JSONDecodeError) as exc:
            self.node._set_action_text(f'Failed to open script "{script_name}": {exc}')
            return

        raw_points = payload.get('points', [])
        if not isinstance(raw_points, list):
            self.node._set_action_text(f'Script "{script_name}" has invalid points data.')
            return

        normalized_points: list[dict[str, object]] = []
        for point in raw_points:
            normalized_point = self._normalize_script_point(point)
            if normalized_point is not None:
                normalized_points.append(normalized_point)

        loaded_speed_profile = payload.get('speed_profile')
        speed_profile_from_script = isinstance(loaded_speed_profile, dict)
        if speed_profile_from_script:
            applied_speed_profile = self.node.apply_script_speed_profile_cache(loaded_speed_profile)
        else:
            applied_speed_profile = self.node.get_script_speed_profile_snapshot()

        self._current_script_name = script_name
        self._current_script_points = normalized_points
        self._current_script_speed_profile = dict(applied_speed_profile)
        self.script_name_input_var.set(script_name)
        metadata_upgraded = False
        if speed_profile_from_script:
            self._refresh_script_options(preferred_name=script_name)
        else:
            metadata_upgraded = self._save_loaded_script()
            if not metadata_upgraded:
                self._refresh_script_options(preferred_name=script_name)
        if speed_profile_from_script:
            self.node._set_action_text(
                f'Opened script "{script_name}" with {len(normalized_points)} movement point(s), '
                f'CP={applied_speed_profile["cp"]}% SF={applied_speed_profile["speed_factor"]}%.'
            )
        else:
            upgrade_suffix = ' Script metadata upgraded with current CP/SF.' if metadata_upgraded else ''
            self.node._set_action_text(
                f'Opened script "{script_name}" with {len(normalized_points)} movement point(s). '
                f'No script speed profile found; using current UI CP/SF.{upgrade_suffix}'
            )

    def _delete_selected_script(self) -> None:
        selected_name = self.script_open_var.get().strip() or self._current_script_name
        if not selected_name:
            self.node._set_action_text('Select a script to delete.')
            return

        script_path = self._script_path(selected_name)
        if not script_path.exists():
            self.node._set_action_text(f'Script "{selected_name}" does not exist.')
            self._refresh_script_options()
            return

        try:
            script_path.unlink()
        except OSError as exc:
            self.node._set_action_text(f'Failed to delete script "{selected_name}": {exc}')
            return

        if self._current_script_name == selected_name:
            self._current_script_name = None
            self._current_script_points = []
            self._current_script_speed_profile = self.node.get_script_speed_profile_snapshot()
        self._refresh_script_options()
        self.node._set_action_text(f'Deleted script "{selected_name}".')

    def _add_last_motion_point_to_script(self) -> None:
        if self._current_script_name is None:
            self.node._set_action_text('Create or open a script first.')
            return

        last_point = self.node.get_last_motion_point()
        if last_point is None:
            self.node._set_action_text('No MovJ/MovL call recorded yet. Send a movement first.')
            return

        normalized_point = self._normalize_script_point(last_point)
        if normalized_point is None:
            self.node._set_action_text('Last movement point is invalid and cannot be saved.')
            return

        self._current_script_points.append(normalized_point)
        if not self._save_loaded_script():
            return
        tool_mode_label = 'with Tool' if normalized_point.get('use_tool') else 'without Tool'
        self.node._set_action_text(
            f'Added {normalized_point["motion_type"]} point to "{self._current_script_name}" '
            f'({len(self._current_script_points)} total, {tool_mode_label}).'
        )

    def _add_current_joint_position_to_script(self) -> None:
        if self._current_script_name is None:
            self.node._set_action_text('Create or open a script first.')
            return

        current_point = self.node.capture_current_joint_position_as_last_movj_point()
        if current_point is None:
            self.node._set_action_text('No joint state received yet. Wait for robot joint feedback first.')
            return

        normalized_point = self._normalize_script_point(current_point)
        if normalized_point is None:
            self.node._set_action_text('Current joint position is invalid and cannot be saved.')
            return

        self._current_script_points.append(normalized_point)
        if not self._save_loaded_script():
            return
        tool_mode_label = 'with Tool' if normalized_point.get('use_tool') else 'without Tool'
        self.node._set_action_text(
            f'Added current joint position as {normalized_point["motion_type"]} to "{self._current_script_name}" '
            f'({len(self._current_script_points)} total, {tool_mode_label}).'
        )

    def _run_loaded_script(self) -> None:
        if self.node.is_script_running():
            self.node.stop_motion_script()
            return

        if self._current_script_name is None:
            selected_name = self.script_open_var.get().strip()
            if not selected_name:
                self.node._set_action_text('Create or open a script first.')
                return
            self._load_script_by_name(selected_name)
            if self._current_script_name is None:
                return

        if not self._current_script_points:
            self.node._set_action_text(f'Script "{self._current_script_name}" has no movement points.')
            return

        self.node.run_motion_script(
            self._current_script_name,
            list(self._current_script_points),
            dict(self._current_script_speed_profile),
        )

    def _stop_loaded_script(self) -> None:
        self.node.stop_motion_script()

    def _on_script_goal_tolerance_change(self, value: str) -> None:
        try:
            requested_percent = float(value)
        except (TypeError, ValueError):
            return

        self.node.set_script_goal_check_tolerance_percent(requested_percent)
        applied_percent = self.node.get_script_goal_check_tolerance_percent()
        self.script_goal_tolerance_label_var.set(f'{applied_percent:.1f}%')

    def _refresh(self) -> None:
        snapshot = self.node.snapshot()
        now = time.time()
        self._show_clear_error_estop_prompt_if_needed()
        override_available = self._override_available(snapshot)
        tool_tcp_changed = snapshot.tool_tcp_values != self._last_tool_tcp_values
        use_tool_changed = snapshot.use_tool_enabled != self._last_use_tool_enabled
        if tool_tcp_changed:
            self._last_tool_tcp_values = list(snapshot.tool_tcp_values)
        if use_tool_changed:
            self._last_use_tool_enabled = snapshot.use_tool_enabled
            self._tcp_tool_sync_deadline = now + 1.0

        if not override_available:
            self._joint_override_enabled = False
            self._tcp_override_enabled = False

        if not self._joint_override_enabled:
            self._joint_override_targets = dict(snapshot.joints_deg)
        tcp_tool_sync_active = (
            self._tcp_tool_sync_deadline is not None and now <= self._tcp_tool_sync_deadline
        )
        if not self._tcp_override_enabled or tool_tcp_changed or use_tool_changed or tcp_tool_sync_active:
            self._tcp_override_targets = dict(snapshot.tcp_values)
        elif self._tcp_tool_sync_deadline is not None and now > self._tcp_tool_sync_deadline:
            self._tcp_tool_sync_deadline = None

        self._update_status_badge(
            self.connected_badge,
            self.connected_value,
            snapshot.connected,
            'Connected',
            'Disconnected',
            'Unknown',
        )
        self._update_robot_status_badge(snapshot, now)

        health_text, health_color = self._stream_health(snapshot, now)
        self.health_value.set(health_text)
        self.health_badge.configure(bg=health_color)

        self.controller_var.set(f'Controller: {snapshot.controller_text}')
        self.error_var.set(f'Error: {snapshot.error_text}')
        self.action_var.set(f'Last Action: {snapshot.action_text}')
        if snapshot.ee_load_kg is None:
            self.ee_load_var.set('EE Load: Not set')
        else:
            self.ee_load_var.set(f'EE Load: {snapshot.ee_load_kg:.2f} kg')
        self.tool_tcp_var.set(
            'Offset TCP: ' + ','.join(self._format_target_value(value) for value in snapshot.tool_tcp_values)
        )
        if tool_tcp_changed:
            self.tool_tcp_input_var.set(
                ','.join(self._format_target_value(value) for value in snapshot.tool_tcp_values)
            )
        if (
            tool_tcp_changed
            or use_tool_changed
            or tcp_tool_sync_active
            or not self._tcp_override_enabled
        ):
            self._sync_tcp_entry_to_targets()

        self._update_buttons(snapshot)
        self._refresh_script_datalog()

        speed_controls_enabled = snapshot.connected and not snapshot.busy_action
        for key, row in self.speed_rows.items():
            row.update_value(snapshot.speed_values[key])
            row.set_enabled(speed_controls_enabled)

        joint_override_ready = override_available and not snapshot.busy_action and self._joint_override_enabled
        for joint_name, row in self.joint_rows.items():
            row.update_value(self._joint_override_targets[joint_name] if self._joint_override_enabled else snapshot.joints_deg[joint_name])
            row.set_enabled(joint_override_ready)

        tcp_override_ready = override_available and not snapshot.busy_action and self._tcp_override_enabled
        for field_name, row in self.tcp_rows.items():
            row.update_value(self._tcp_override_targets[field_name] if self._tcp_override_enabled else snapshot.tcp_values[field_name])
            row.set_enabled(tcp_override_ready)

        if not self._closed:
            self.root.after(100, self._refresh)

    def _show_clear_error_estop_prompt_if_needed(self) -> None:
        if not self.node.consume_clear_error_estop_prompt():
            return
        messagebox.showwarning(
            'Clear Error Check',
            'Error is still active after Clear Error.\nCheck whether the emergency stop is pressed.',
            parent=self.root,
        )

    def _update_buttons(self, snapshot: MotionSnapshot) -> None:
        script_running = self.node.is_script_running()
        buttons_enabled = snapshot.connected and not snapshot.busy_action and not script_running
        robot_should_disable = self.node._robot_should_disable(snapshot)
        override_available = self._override_available(snapshot)
        self.clear_error_button.configure(state=tk.NORMAL if buttons_enabled and snapshot.error_active else tk.DISABLED)
        if snapshot.controller_mode_code == 10:
            robot_button_text = 'Exit Pause'
        else:
            robot_button_text = 'Disable Robot' if robot_should_disable else 'Enable Robot'
        self.enable_button.configure(
            text=robot_button_text,
            state=tk.NORMAL if buttons_enabled else tk.DISABLED,
        )

        if snapshot.drag_enabled:
            drag_text = 'Disable Drag'
        elif snapshot.ee_load_kg is None:
            drag_text = 'For Drag Set EE Load'
        else:
            drag_text = 'Enable Drag'
        drag_button_enabled = buttons_enabled and (
            snapshot.drag_enabled or snapshot.enabled or snapshot.controller_mode_code in {5, 6, 7}
        )
        self.drag_button.configure(
            text=drag_text,
            state=tk.NORMAL if drag_button_enabled else tk.DISABLED,
        )

        ee_load_controls_state = tk.NORMAL if buttons_enabled else tk.DISABLED
        self.ee_load_entry.configure(state=ee_load_controls_state)
        self.ee_load_button.configure(state=ee_load_controls_state)
        self.tool_tcp_entry.configure(state=ee_load_controls_state)
        self.tool_tcp_button.configure(state=ee_load_controls_state)

        override_button_state = (
            tk.NORMAL if override_available and not snapshot.busy_action and not script_running else tk.DISABLED
        )
        joint_target_controls_state = (
            tk.NORMAL
            if override_available and not snapshot.busy_action and not script_running and self._joint_override_enabled
            else tk.DISABLED
        )
        tcp_target_controls_state = (
            tk.NORMAL
            if override_available and not snapshot.busy_action and not script_running and self._tcp_override_enabled
            else tk.DISABLED
        )
        self._update_override_button_style(
            self.joint_override_button,
            self._joint_override_enabled,
            override_button_state,
        )
        self._update_override_button_style(
            self.tcp_override_button,
            self._tcp_override_enabled,
            override_button_state,
        )
        tool_button_state = tcp_target_controls_state
        self._update_toggle_button_style(
            self.use_tool_button,
            'Use Tool',
            snapshot.use_tool_enabled,
            tool_button_state,
            active_background='#f6e27a',
            active_background_pressed='#ecd34d',
            active_foreground='#5a4700',
        )
        self.joint_target_entry.configure(state=joint_target_controls_state)
        self.joint_target_go_button.configure(state=joint_target_controls_state)
        self.tcp_target_entry.configure(state=tcp_target_controls_state)
        self.tcp_target_go_button.configure(state=tcp_target_controls_state)

        script_controls_idle = snapshot.busy_action is None and not script_running
        has_script_loaded = self._current_script_name is not None
        has_script_points = bool(self._current_script_points)
        has_last_motion_point = self.node.get_last_motion_point() is not None
        has_joint_feedback = snapshot.joint_stamp is not None
        has_any_saved_scripts = bool(self._script_names)
        selected_script_name = self.script_open_var.get().strip()

        edit_state = tk.NORMAL if script_controls_idle else tk.DISABLED
        self.script_name_entry.configure(state=edit_state)
        self.script_create_button.configure(state=edit_state)
        self.script_open_combo.configure(
            state='readonly' if script_controls_idle and has_any_saved_scripts else tk.DISABLED
        )
        self.script_open_button.configure(
            state=tk.NORMAL if script_controls_idle and has_any_saved_scripts else tk.DISABLED
        )
        self.script_open_editor_button.configure(
            state=tk.NORMAL if script_controls_idle and (has_script_loaded or bool(selected_script_name)) else tk.DISABLED
        )
        self.script_delete_button.configure(
            state=tk.NORMAL if script_controls_idle and (has_script_loaded or bool(selected_script_name)) else tk.DISABLED
        )
        self.script_add_point_button.configure(
            state=tk.NORMAL if script_controls_idle and has_script_loaded and has_last_motion_point else tk.DISABLED
        )
        self.script_add_position_button.configure(
            state=tk.NORMAL if script_controls_idle and has_script_loaded and has_joint_feedback else tk.DISABLED
        )
        if script_running:
            self.script_run_button.configure(
                text='Stop Script (Click Again)',
                command=self._stop_loaded_script,
                state=tk.NORMAL if snapshot.connected else tk.DISABLED,
                bg='#dc2626',
                activebackground='#b91c1c',
                fg='white',
                activeforeground='white',
                disabledforeground='#f3b3b3',
                relief=tk.SUNKEN,
            )
        else:
            run_state = tk.NORMAL if buttons_enabled and has_script_loaded and has_script_points else tk.DISABLED
            if run_state == tk.DISABLED:
                run_bg = '#e5e7eb'
                run_active_bg = '#e5e7eb'
                run_fg = '#7a8088'
                run_active_fg = '#7a8088'
            else:
                run_bg = '#eef1f6'
                run_active_bg = '#e3e7ee'
                run_fg = '#1f2937'
                run_active_fg = '#1f2937'
            self.script_run_button.configure(
                text='Run Script',
                command=self._run_loaded_script,
                state=run_state,
                bg=run_bg,
                activebackground=run_active_bg,
                fg=run_fg,
                activeforeground=run_active_fg,
                disabledforeground='#7a8088',
                relief=tk.RAISED,
            )
        self.script_goal_tolerance_scale.configure(state=tk.NORMAL if script_controls_idle else tk.DISABLED)

    def _update_override_button_style(self, button: tk.Button, active: bool, state: str) -> None:
        self._update_toggle_button_style(
            button,
            'Override On' if active else 'Override Off',
            active,
            state,
        )

    def _update_toggle_button_style(
        self,
        button: tk.Button,
        label: str,
        active: bool,
        state: str,
        active_background: str = '#efb1b1',
        active_background_pressed: str = '#e99999',
        active_foreground: str = '#5f1111',
    ) -> None:
        if active:
            background = active_background
            pressed_background = active_background_pressed
            foreground = active_foreground
            relief = tk.SUNKEN
        elif state == tk.DISABLED:
            background = '#e5e7eb'
            pressed_background = '#e5e7eb'
            foreground = '#7a8088'
            relief = tk.RAISED
        else:
            background = '#eef1f6'
            pressed_background = '#e3e7ee'
            foreground = '#1f2937'
            relief = tk.RAISED

        button.configure(
            text=label,
            state=state,
            bg=background,
            activebackground=pressed_background,
            fg=foreground,
            activeforeground=foreground,
            disabledforeground=foreground,
            relief=relief,
        )

    def _clamp_tcp_targets(self, targets: dict[str, float]) -> dict[str, float]:
        clamped_targets = dict(targets)
        for field_name, limits in self._tcp_workspace.limits.items():
            value = clamped_targets.get(field_name)
            if value is None:
                continue
            low, high = limits
            clamped_targets[field_name] = max(low, min(high, value))

        inner_radius = self._tcp_workspace.xy_inner_radius_mm
        outer_radius = self._tcp_workspace.xy_outer_radius_mm
        if inner_radius is None and outer_radius is None:
            return clamped_targets

        x_value = clamped_targets.get('x', 0.0)
        y_value = clamped_targets.get('y', 0.0)
        radius = math.hypot(x_value, y_value)

        if outer_radius is not None and radius > outer_radius and radius > 0.0:
            scale = outer_radius / radius
            x_value *= scale
            y_value *= scale
            radius = outer_radius

        if inner_radius is not None and radius < inner_radius:
            if radius <= 1e-6:
                x_value = inner_radius
                y_value = 0.0
            else:
                scale = inner_radius / radius
                x_value *= scale
                y_value *= scale

        clamped_targets['x'] = x_value
        clamped_targets['y'] = y_value
        return clamped_targets

    def _update_status_badge(
        self,
        label: tk.Label,
        value_var: tk.StringVar,
        status: bool | None,
        true_text: str,
        false_text: str,
        unknown_text: str,
    ) -> None:
        if status is True:
            value_var.set(true_text)
            label.configure(bg='#b7e4c7')
        elif status is False:
            value_var.set(false_text)
            label.configure(bg='#f5c2c7')
        else:
            value_var.set(unknown_text)
            label.configure(bg='#d5dce7')

    def _update_robot_status_badge(self, snapshot: MotionSnapshot, now: float) -> None:
        mode_map = {
            1: ('Initializing', '#d5dce7'),
            2: ('Brake Open', '#ffe69c'),
            4: ('Disabled', '#f5c2c7'),
            5: ('Enabled', '#b7e4c7'),
            6: ('Drag Mode', '#ffe69c'),
            7: ('Running', '#cfe2ff'),
            8: ('Recording', '#ffe69c'),
            9: ('Error', '#f5c2c7'),
            10: ('Paused', '#cfe2ff'),
            11: ('Jogging', '#ffe69c'),
        }
        if snapshot.controller_mode_code in mode_map:
            text, color = mode_map[snapshot.controller_mode_code]
        elif snapshot.enabled is True:
            text, color = 'Enabled', '#b7e4c7'
        elif snapshot.enabled is False:
            text, color = 'Disabled', '#f5c2c7'
        else:
            text, color = 'Unknown', '#d5dce7'

        if snapshot.controller_mode_code == 6:
            color = self._flash_color(now, '#ffe69c', '#ffc078')
        elif snapshot.controller_mode_code == 9:
            color = self._flash_color(now, '#f5c2c7', '#ff8787')

        self.robot_status_value.set(text)
        self.robot_status_badge.configure(bg=color)

    def _override_available(self, snapshot: MotionSnapshot) -> bool:
        return snapshot.connected is True and (
            snapshot.controller_mode_code in {5, 7, 11} or snapshot.enabled is True
        )

    def _flash_color(self, now: float, primary: str, secondary: str) -> str:
        return primary if int(now * 4) % 2 == 0 else secondary

    def _stream_health(self, snapshot: MotionSnapshot, now: float) -> tuple[str, str]:
        freshest = [stamp for stamp in (snapshot.joint_stamp, snapshot.tcp_stamp, snapshot.status_stamp) if stamp]
        if not freshest:
            return 'Waiting for data', '#ffe69c'

        stalest_age = max(now - stamp for stamp in freshest)
        if stalest_age > STALE_DATA_SEC:
            return f'Stale ({stalest_age:.1f}s)', '#f5c2c7'

        return 'Live', '#b7e4c7'

    def _age_text(self, stamp: float | None, now: float) -> str:
        if stamp is None:
            return 'waiting'
        return f'{max(now - stamp, 0.0):.1f}s'

    def _on_close(self) -> None:
        self._closed = True
        self.root.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionDebugNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    app = MotionDebugApp(node)

    try:
        app.run()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
