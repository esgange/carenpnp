import json
import ipaddress
import os
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext
from typing import Callable

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import LoadOnlineProgram, TrayInterceptStart
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

try:
    import yaml
except Exception:  # pragma: no cover - runtime fallback for minimal installs
    yaml = None


ITEM_GO_TO_TEACH_SERVICE_DEFAULT = 'item_detect/go_to_teach'
ITEM_ARM_SERVICE_DEFAULT = 'item_pick/track'
ITEM_ARM_STATUS_SERVICE_DEFAULT = 'item_pick/track_status'
ITEM_AUTO_REPICK_SERVICE_DEFAULT = 'item_pick/set_auto_repick'
ITEM_SEEK_SERVICE_DEFAULT = 'item_detect/seek'
ITEM_REPICK_SERVICE_DEFAULT = 'item_detect/repick'
ITEM_SEEK_STATUS_SERVICE_DEFAULT = 'item_detect/seek_status'
TRAY_GO_TO_TEACH_SERVICE_DEFAULT = 'tray_detect/go_to_teach'
TRAY_ARM_SERVICE_DEFAULT = 'tray_intercept/start_sequence'
TRAY_ARM_STATUS_SERVICE_DEFAULT = 'tray_intercept/track_status'
TRAY_SEEK_SERVICE_DEFAULT = 'tray_detect/seek'
TRAY_SEEK_STATUS_SERVICE_DEFAULT = 'tray_detect/seek_status'
ONLINE_START_SERVICE_DEFAULT = 'robot_cell_orchestrator/start_online'
ONLINE_LOAD_PROGRAM_SERVICE_DEFAULT = 'robot_cell_orchestrator/load_online_program'
ONLINE_VALIDATE_SERVICE_DEFAULT = 'robot_cell_orchestrator/validate_online_program'
ONLINE_PLACE_SERVICE_DEFAULT = 'robot_cell_orchestrator/place_online'
PHASE_EVENT_TOPIC_DEFAULT = 'robot_cell_orchestrator/events'
ROBOT_TCP_TOPIC_DEFAULT = '/dobot_msgs_v4/msg/ToolVectorActual'
BIN_DETECT_OVERLAY_TOPIC_DEFAULT = 'bin_overlay'
TRAY_DETECT_OVERLAY_TOPIC_DEFAULT = 'tray_overlay'

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
READINESS_SCAN_MS = 1000
PROCESS_SCAN_MS = 1000
RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS = 250
PROCESS_STOP_TIMEOUT_SEC = 3.0
PROCESS_TERMINAL_PID_WAIT_SEC = 2.0
RUNNING_BUTTON_BG = '#d9ead3'
CAMERA_VIEW_WIDTH = 720
CAMERA_VIEW_HEIGHT = 405
CAMERA_WINDOW_MIN_WIDTH = 560
CAMERA_WINDOW_MIN_HEIGHT = 640
CAMERA_WINDOW_DEFAULT_WIDTH = CAMERA_VIEW_WIDTH + 70
CAMERA_WINDOW_DEFAULT_HEIGHT = (CAMERA_VIEW_HEIGHT * 2) + 150
CAMERA_VIEW_REFRESH_MS = 120
CAMERA_VIEW_STALE_SEC = 2.0

MODE_OFFLINE = 'offline'
MODE_ONLINE = 'online'
ONLINE_PHASE_STOPPED = 'stopped'
ONLINE_PHASE_STARTING = 'starting'
ONLINE_PHASE_WAITING_FOR_PICK = 'waiting_for_pick'
ONLINE_PHASE_PICKING = 'picking'
ONLINE_PHASE_WAITING_FOR_PLACE = 'waiting_for_place'
ONLINE_PHASE_PLACING = 'placing'
TEACH_KIND_BIN = 'bin_teach'
TEACH_KIND_ITEM = 'item_detect'
TEACH_KIND_TRAY = 'tray_detect'
TEACH_KIND_TOOL = 'tool_teach'
RUNTIME_REQUIRED_KINDS = (TEACH_KIND_BIN, TEACH_KIND_ITEM, TEACH_KIND_TRAY)
RUNTIME_OPTIONAL_KINDS = (TEACH_KIND_TOOL,)

ROBOT_STATUS_STOP = 'stop'
ROBOT_STATUS_PICKING = 'picking'
ROBOT_STATUS_PLACING = 'placing'
ROBOT_STATUS_PAUSE = 'pause'
ROBOT_STATUS_LABELS = {
    ROBOT_STATUS_STOP: 'Idle',
    ROBOT_STATUS_PICKING: 'Picking',
    ROBOT_STATUS_PLACING: 'Placing',
    ROBOT_STATUS_PAUSE: 'Idle',
}

CALIBRATION_PATTERNS = {
    'Eye-on-hand': 'axab_calibration_eyeonhand_*.yaml',
    'Eye-to-hand': 'axab_calibration_eyetohand_*.yaml',
    'Platform': 'platform_calibration_*.yaml',
}
CALIBRATION_EYE_ON_HAND = 'Eye-on-hand'
CALIBRATION_EYE_TO_HAND = 'Eye-to-hand'
CALIBRATION_PLATFORM = 'Platform'
ROBOT_IP_CONFIG_KEY = 'ROBOT_IP_ADDRESS'
ROBOT_LAN2_DEFAULT_IP = '192.168.200.1'


def _looks_like_workspace_root(path: Path) -> bool:
    return (
        (path / 'src').exists()
        and (
            (path / 'README.md').exists()
            or (path / 'src' / 'dobot_msgs_v4').exists()
        )
    )


def workspace_root() -> Path:
    for env_name in ('DOBOT_PICKN_PLACE_ROOT', 'DOBOT_WORKSPACE_ROOT'):
        value = os.environ.get(env_name)
        if value:
            return Path(value).expanduser().resolve()

    for start in (Path(__file__).resolve(), Path.cwd()):
        path = start if start.is_dir() else start.parent
        for candidate in (path, *path.parents):
            if _looks_like_workspace_root(candidate):
                return candidate
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


def shell_join(args: list[str]) -> str:
    return ' '.join(shlex.quote(str(arg)) for arg in args)


def ros_sourced_shell_command(cmd: list[str], *, source_python_venv: bool = True) -> str:
    root = shlex.quote(str(workspace_root()))
    venv_source = (
        'if [ -f "$ROOT/third_party/.venv/bin/activate" ]; then '
        'source "$ROOT/third_party/.venv/bin/activate"; '
        'fi; '
        if source_python_venv
        else ''
    )
    return (
        'set -e; '
        f'ROOT="${{DOBOT_PICKN_PLACE_ROOT:-{root}}}"; '
        'cd "$ROOT"; '
        'if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi; '
        'if [ -f "$ROOT/install/setup.bash" ]; then source "$ROOT/install/setup.bash"; fi; '
        f'{venv_source}'
        f'exec {shell_join(cmd)}'
    )


def ros_child_environment() -> dict[str, str]:
    env = os.environ.copy()
    env['DOBOT_PICKN_PLACE_ROOT'] = str(workspace_root())
    env['ROS_LOCALHOST_ONLY'] = '1'
    return env


def _normalized_rgb_image(image: object) -> np.ndarray:
    rgb = np.asarray(image)
    if rgb.ndim == 2:
        rgb = np.repeat(rgb[:, :, np.newaxis], 3, axis=2)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise ValueError(f'Unsupported image shape for camera view: {rgb.shape}')
    rgb = rgb[:, :, :3]
    height, width = rgb.shape[:2]
    if width <= 0 or height <= 0:
        raise ValueError('Empty image for camera view')
    return np.array(rgb, dtype=np.uint8, copy=True)


def _rgb_image_to_view_ppm(image: object, target_width: int, target_height: int) -> bytes:
    target_width = max(1, int(target_width))
    target_height = max(1, int(target_height))
    rgb = _normalized_rgb_image(image)
    height, width = rgb.shape[:2]

    scale = min(target_width / float(width), target_height / float(height))
    scaled_width = min(target_width, max(1, int(round(width * scale))))
    scaled_height = min(target_height, max(1, int(round(height * scale))))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(rgb, (scaled_width, scaled_height), interpolation=interpolation)
    resized = np.asarray(resized, dtype=np.uint8)

    canvas = np.full((target_height, target_width, 3), 24, dtype=np.uint8)
    x0 = max(0, (target_width - scaled_width) // 2)
    y0 = max(0, (target_height - scaled_height) // 2)
    canvas[y0:y0 + scaled_height, x0:x0 + scaled_width] = resized
    header = f'P6 {target_width} {target_height} 255\n'.encode('ascii')
    return header + canvas.tobytes()


def visible_terminal_command(title: str, shell_command: str) -> list[str] | None:
    if shutil.which('gnome-terminal'):
        return [
            'gnome-terminal',
            '--wait',
            '--title',
            title,
            '--',
            'bash',
            '-lc',
            shell_command,
        ]
    if shutil.which('xfce4-terminal'):
        return [
            'xfce4-terminal',
            '--disable-server',
            '--title',
            title,
            '--command',
            f'bash -lc {shlex.quote(shell_command)}',
        ]
    if shutil.which('xterm'):
        return ['xterm', '-T', title, '-e', 'bash', '-lc', shell_command]
    if shutil.which('konsole'):
        return [
            'konsole',
            '--nofork',
            '--workdir',
            str(workspace_root()),
            '-p',
            f'tabtitle={title}',
            '-e',
            'bash',
            '-lc',
            shell_command,
        ]
    if shutil.which('mate-terminal'):
        return [
            'mate-terminal',
            '--disable-factory',
            '--title',
            title,
            '--',
            'bash',
            '-lc',
            shell_command,
        ]
    return None


def safe_process_label(text: str) -> str:
    chars = [ch.lower() if ch.isalnum() else '_' for ch in text]
    label = ''.join(chars).strip('_')
    while '__' in label:
        label = label.replace('__', '_')
    return label[:64] or 'process'


def _strip_optional_quotes(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def key_value_config(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        with path.open('r', encoding='utf-8') as infile:
            for raw_line in infile:
                line = raw_line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('export '):
                    line = line[len('export '):].strip()
                if '=' not in line:
                    continue
                key, value = line.split('=', 1)
                key = key.strip()
                if key:
                    values[key] = _strip_optional_quotes(value.strip())
    except Exception:
        return {}
    return values


def update_key_value_config(path: Path, key: str, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
    except FileNotFoundError:
        lines = []

    updated = False
    output: list[str] = []
    for raw_line in lines:
        newline = '\n' if raw_line.endswith('\n') else ''
        content = raw_line[:-1] if newline else raw_line
        stripped = content.strip()
        if not stripped or stripped.startswith('#') or '=' not in stripped:
            output.append(raw_line)
            continue

        leading = content[:len(content) - len(content.lstrip())]
        body = content.lstrip()
        export_prefix = ''
        if body.startswith('export '):
            export_prefix = 'export '
            body = body[len('export '):].strip()
        raw_key, _raw_value = body.split('=', 1)
        if raw_key.strip() == key:
            output.append(f'{leading}{export_prefix}{key}={value}{newline}')
            updated = True
        else:
            output.append(raw_line)

    if not updated:
        if output and not output[-1].endswith('\n'):
            output[-1] += '\n'
        if output and output[-1].strip():
            output.append('\n')
        output.append(f'{key}={value}\n')

    path.write_text(''.join(output), encoding='utf-8')


def normalize_ipv4_address(value: str) -> str | None:
    candidate = str(value or '').strip()
    if not candidate:
        return None
    try:
        address = ipaddress.ip_address(candidate)
    except ValueError:
        return None
    if address.version != 4:
        return None
    return str(address)


def _load_yaml_mapping(path: Path) -> dict | None:
    if yaml is None:
        return None
    try:
        with path.open('r', encoding='utf-8') as infile:
            payload = yaml.safe_load(infile)
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _fallback_top_level_keys(path: Path) -> set[str]:
    keys: set[str] = set()
    try:
        with path.open('r', encoding='utf-8') as infile:
            for raw_line in infile:
                if not raw_line.strip() or raw_line.lstrip().startswith('#'):
                    continue
                if raw_line.startswith((' ', '\t', '-')):
                    continue
                if ':' not in raw_line:
                    continue
                keys.add(raw_line.split(':', 1)[0].strip())
    except Exception:
        return set()
    return keys


def yaml_top_level_keys(path: Path) -> set[str]:
    payload = _load_yaml_mapping(path)
    if payload is not None:
        return {str(key) for key in payload.keys()}
    return _fallback_top_level_keys(path)


def classify_teach_yaml(path: Path) -> str:
    keys = yaml_top_level_keys(path)
    if 'bin_teach' in keys:
        return TEACH_KIND_BIN
    if 'item_detect' in keys:
        return TEACH_KIND_ITEM
    if 'tray_detect' in keys:
        return TEACH_KIND_TRAY
    if 'tool_teach_version' in keys:
        return TEACH_KIND_TOOL
    return 'unknown'


def classify_calibration_yaml(path: Path) -> str:
    payload = _load_yaml_mapping(path)
    if not isinstance(payload, dict) or not isinstance(payload.get('transform'), dict):
        return 'unknown'

    parameters = payload.get('parameters')
    if isinstance(parameters, dict):
        calibration_type = str(parameters.get('calibration_type', '')).strip().lower()
        if calibration_type in ('eye_in_hand', 'eye_on_hand', 'eyeonhand'):
            return CALIBRATION_EYE_ON_HAND
        if calibration_type in ('eye_on_base', 'eye_to_hand', 'eyetohand'):
            return CALIBRATION_EYE_TO_HAND

    metadata = payload.get('metadata')
    if isinstance(metadata, dict):
        calibration_type = str(metadata.get('calibration_type', '')).strip().lower()
        if calibration_type == 'platform_reference':
            return CALIBRATION_PLATFORM
    return 'unknown'


def item_profile_has_embedded_tool_teach(path: Path | None) -> bool:
    if path is None or not path.exists():
        return False
    payload = _load_yaml_mapping(path)
    return isinstance(payload, dict) and isinstance(payload.get('tool_teach'), dict)


def yaml_files_in(path: Path) -> list[Path]:
    if not path.exists() or not path.is_dir():
        return []
    return sorted(
        [candidate for candidate in path.iterdir() if candidate.is_file() and candidate.suffix in ('.yaml', '.yml')],
        key=lambda candidate: candidate.name.lower(),
    )


def file_label(path: Path | None) -> str:
    return path.name if path else 'Missing'


def _coerce_float(raw_value: object, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return default
    return max(float(minimum), min(float(maximum), value))


def _coerce_bool(raw_value: object, default: bool = False) -> bool:
    if isinstance(raw_value, bool):
        return raw_value
    if isinstance(raw_value, (int, float)):
        return bool(raw_value)
    if isinstance(raw_value, str):
        text = raw_value.strip().lower()
        if text in ('1', 'true', 'yes', 'on'):
            return True
        if text in ('0', 'false', 'no', 'off'):
            return False
    return default


def _normalize_window_geometry(raw_value: object, default: str = '1180x850') -> str:
    text = str(raw_value or '').strip().split('+', 1)[0]
    if 'x' not in text:
        text = default
    try:
        width_text, height_text = text.lower().split('x', 1)
        width = max(1060, int(float(width_text)))
        height = max(780, int(float(height_text)))
    except (TypeError, ValueError):
        return default
    return f'{width}x{height}'


@dataclass(frozen=True)
class TriggerResult:
    success: bool
    message: str


@dataclass(frozen=True)
class OnlineProgramLoad:
    qqc_id: str
    bin_teach_file: str
    item_teach_file: str
    tray_teach_file: str
    tray_x_mm: float
    tray_y_mm: float
    tray_rz_deg: float


@dataclass(frozen=True)
class OnlineProgramLoadResult:
    success: bool
    message: str
    runtime_files: tuple[str, ...] = ()


@dataclass(frozen=True)
class SeekStatusResult:
    available: bool
    active: bool
    message: str


@dataclass(frozen=True)
class CameraViewFrame:
    topic: str
    rgb_data: np.ndarray
    received_monotonic: float


@dataclass(frozen=True)
class RobotCellOrchestratorRuntimeSettings:
    loop_enabled: bool
    auto_repick_enabled: bool
    step_mode_enabled: bool
    tray_seek_stability_sec: float
    tray_intercept_x_offset_mm: float
    tray_intercept_y_offset_mm: float
    tray_ee_angle_deg: float
    eye_on_hand_calibration_file: str = ''
    eye_to_hand_calibration_file: str = ''
    platform_calibration_file: str = ''
    window_geometry: str = '1180x850'


@dataclass
class CalibrationScan:
    files: dict[str, Path | None]

    @property
    def ok(self) -> bool:
        return all(path is not None for path in self.files.values())

    @property
    def message(self) -> str:
        if self.ok:
            return 'OK: ' + ', '.join(f'{name}={path.name}' for name, path in self.files.items() if path is not None)
        missing = [name for name, path in self.files.items() if path is None]
        return 'Missing: ' + ', '.join(missing)


@dataclass
class RuntimeScan:
    root: Path
    by_kind: dict[str, list[Path]] = field(default_factory=dict)
    unknown_yaml: list[Path] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        item_paths = self.by_kind.get(TEACH_KIND_ITEM, [])
        item_has_tool = len(item_paths) == 1 and item_profile_has_embedded_tool_teach(item_paths[0])
        legacy_tool_count = len(self.by_kind.get(TEACH_KIND_TOOL, []))
        return (
            self.root.exists()
            and self.root.is_dir()
            and not self.unknown_yaml
            and all(len(self.by_kind.get(kind, [])) == 1 for kind in RUNTIME_REQUIRED_KINDS)
            and (item_has_tool or legacy_tool_count == 1)
        )

    @property
    def message(self) -> str:
        if not self.root.exists() or not self.root.is_dir():
            return f'Missing folder: {self.root}'
        problems: list[str] = []
        for kind in RUNTIME_REQUIRED_KINDS:
            count = len(self.by_kind.get(kind, []))
            if count == 0:
                problems.append(f'missing {kind}')
            elif count > 1:
                problems.append(f'{count} {kind} files')
        item_paths = self.by_kind.get(TEACH_KIND_ITEM, [])
        item_has_tool = len(item_paths) == 1 and item_profile_has_embedded_tool_teach(item_paths[0])
        legacy_tool_count = len(self.by_kind.get(TEACH_KIND_TOOL, []))
        if not item_has_tool and legacy_tool_count == 0:
            problems.append('missing tool_teach')
        elif not item_has_tool and legacy_tool_count > 1:
            problems.append(f'{legacy_tool_count} {TEACH_KIND_TOOL} files')
        if self.unknown_yaml:
            problems.append('unknown YAML: ' + ', '.join(path.name for path in self.unknown_yaml))
        if problems:
            return '; '.join(problems)
        names = [self.by_kind[kind][0].name for kind in RUNTIME_REQUIRED_KINDS]
        names.append('embedded tool_teach' if item_has_tool else self.by_kind[TEACH_KIND_TOOL][0].name)
        return 'OK: ' + ', '.join(names)


@dataclass
class OfflineTeachScan:
    bin_path: Path | None
    item_path: Path | None
    tray_path: Path | None
    tool_path: Path | None

    @property
    def ok(self) -> bool:
        base_ok = all(path is not None and path.exists() for path in (self.bin_path, self.item_path, self.tray_path))
        return base_ok and (item_profile_has_embedded_tool_teach(self.item_path) or (self.tool_path is not None and self.tool_path.exists()))

    @property
    def message(self) -> str:
        if self.ok:
            tool_label = 'embedded' if item_profile_has_embedded_tool_teach(self.item_path) else file_label(self.tool_path)
            return 'OK: bin=%s, item=%s, tray=%s, tool=%s' % (
                file_label(self.bin_path),
                file_label(self.item_path),
                file_label(self.tray_path),
                tool_label,
            )
        missing: list[str] = []
        if self.bin_path is None or not self.bin_path.exists():
            missing.append('bin')
        if self.item_path is None or not self.item_path.exists():
            missing.append('item')
        if self.tray_path is None or not self.tray_path.exists():
            missing.append('tray')
        if not item_profile_has_embedded_tool_teach(self.item_path) and (self.tool_path is None or not self.tool_path.exists()):
            missing.append('item tool')
        return 'Missing: ' + ', '.join(missing)


@dataclass(frozen=True)
class LaunchSpec:
    label: str
    package: str
    launch_file: str
    args_builder: Callable[[str, bool], list[str]]
    calibration_key: str | tuple[str, ...] | None = None
    headless_launch_file: str | None = None
    headless_label: str | None = None
    source_python_venv: bool = True

    def launch_file_for(self, headless: bool) -> str:
        if headless and self.headless_launch_file:
            return self.headless_launch_file
        return self.launch_file

    def label_for(self, headless: bool) -> str:
        if headless and self.headless_label:
            return self.headless_label
        return self.label


class RobotCellOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_cell_orchestrator_gui')
        self.item_go_to_teach_service = self._declare_name_parameter(
            'item_go_to_teach_service',
            ITEM_GO_TO_TEACH_SERVICE_DEFAULT,
        )
        self.item_arm_service = self._declare_name_parameter('item_arm_service', ITEM_ARM_SERVICE_DEFAULT)
        self.item_arm_status_service = self._declare_name_parameter(
            'item_arm_status_service',
            ITEM_ARM_STATUS_SERVICE_DEFAULT,
        )
        self.item_auto_repick_service = self._declare_name_parameter(
            'item_auto_repick_service',
            ITEM_AUTO_REPICK_SERVICE_DEFAULT,
        )
        self.item_seek_service = self._declare_name_parameter('item_seek_service', ITEM_SEEK_SERVICE_DEFAULT)
        self.item_repick_service = self._declare_name_parameter(
            'item_repick_service',
            ITEM_REPICK_SERVICE_DEFAULT,
        )
        self.item_seek_status_service = self._declare_name_parameter(
            'item_seek_status_service',
            ITEM_SEEK_STATUS_SERVICE_DEFAULT,
        )
        self.tray_go_to_teach_service = self._declare_name_parameter(
            'tray_go_to_teach_service',
            TRAY_GO_TO_TEACH_SERVICE_DEFAULT,
        )
        self.tray_arm_service = self._declare_name_parameter('tray_arm_service', TRAY_ARM_SERVICE_DEFAULT)
        self.tray_arm_status_service = self._declare_name_parameter(
            'tray_arm_status_service',
            TRAY_ARM_STATUS_SERVICE_DEFAULT,
        )
        self.tray_seek_service = self._declare_name_parameter('tray_seek_service', TRAY_SEEK_SERVICE_DEFAULT)
        self.tray_seek_status_service = self._declare_name_parameter(
            'tray_seek_status_service',
            TRAY_SEEK_STATUS_SERVICE_DEFAULT,
        )
        self.online_load_program_service = self._declare_name_parameter(
            'online_load_program_service',
            ONLINE_LOAD_PROGRAM_SERVICE_DEFAULT,
        )
        self.online_start_service = self._declare_name_parameter(
            'online_start_service',
            ONLINE_START_SERVICE_DEFAULT,
        )
        self.online_validate_service = self._declare_name_parameter(
            'online_validate_service',
            ONLINE_VALIDATE_SERVICE_DEFAULT,
        )
        self.online_place_service = self._declare_name_parameter(
            'online_place_service',
            ONLINE_PLACE_SERVICE_DEFAULT,
        )
        self.phase_event_topic = self._declare_name_parameter(
            'phase_event_topic',
            PHASE_EVENT_TOPIC_DEFAULT,
        )
        self.robot_tcp_topic = self._declare_name_parameter('robot_tcp_topic', ROBOT_TCP_TOPIC_DEFAULT)
        self.bin_detect_overlay_topic = self._declare_name_parameter(
            'bin_detect_overlay_topic',
            BIN_DETECT_OVERLAY_TOPIC_DEFAULT,
        )
        self.tray_detect_overlay_topic = self._declare_name_parameter(
            'tray_detect_overlay_topic',
            TRAY_DETECT_OVERLAY_TOPIC_DEFAULT,
        )
        self._tray_arm_client = self.create_client(TrayInterceptStart, self.tray_arm_service)
        self._item_auto_repick_client = self.create_client(SetBool, self.item_auto_repick_service)

        self._trigger_clients: dict[str, tuple[str, object]] = {
            'item_go_to_teach': (self.item_go_to_teach_service, self.create_client(Trigger, self.item_go_to_teach_service)),
            'item_arm': (self.item_arm_service, self.create_client(Trigger, self.item_arm_service)),
            'item_arm_status': (self.item_arm_status_service, self.create_client(Trigger, self.item_arm_status_service)),
            'item_seek': (self.item_seek_service, self.create_client(Trigger, self.item_seek_service)),
            'item_repick': (self.item_repick_service, self.create_client(Trigger, self.item_repick_service)),
            'item_seek_status': (self.item_seek_status_service, self.create_client(Trigger, self.item_seek_status_service)),
            'tray_go_to_teach': (self.tray_go_to_teach_service, self.create_client(Trigger, self.tray_go_to_teach_service)),
            'tray_arm_status': (self.tray_arm_status_service, self.create_client(Trigger, self.tray_arm_status_service)),
            'tray_seek': (self.tray_seek_service, self.create_client(Trigger, self.tray_seek_service)),
            'tray_seek_status': (self.tray_seek_status_service, self.create_client(Trigger, self.tray_seek_status_service)),
        }
        self._startup_service_result: TriggerResult | None = None
        self._startup_service_lock = threading.Lock()
        self._online_load_program_handler: Callable[[OnlineProgramLoad], OnlineProgramLoadResult] | None = None
        self._online_load_program_handler_lock = threading.Lock()
        self._online_start_handler: Callable[[], TriggerResult] | None = None
        self._online_start_handler_lock = threading.Lock()
        self._online_validate_handler: Callable[[], TriggerResult] | None = None
        self._online_validate_handler_lock = threading.Lock()
        self._online_place_handler: Callable[[], TriggerResult] | None = None
        self._online_place_handler_lock = threading.Lock()
        self._online_load_program_srv = self.create_service(
            LoadOnlineProgram,
            self.online_load_program_service,
            self._handle_online_load_program,
        )
        self._online_start_srv = self.create_service(Trigger, self.online_start_service, self._handle_online_start)
        self._online_validate_srv = self.create_service(
            Trigger,
            self.online_validate_service,
            self._handle_online_validate,
        )
        self._online_place_srv = self.create_service(Trigger, self.online_place_service, self._handle_online_place)
        self._phase_event_pub = self.create_publisher(String, self.phase_event_topic, 10)
        self._phase_event_seq = 0
        self._phase_event_lock = threading.Lock()
        self._tcp_condition = threading.Condition()
        self._tcp_seq = 0
        self._latest_tcp: tuple[float, float, float, float, float, float] | None = None
        self._last_tcp_receive_time = 0.0
        self._camera_bridge = CvBridge()
        self._camera_frame_lock = threading.Lock()
        self._camera_frames: dict[str, CameraViewFrame] = {}
        self._camera_error_log_time: dict[str, float] = {}

        self.create_subscription(ToolVectorActual, self.robot_tcp_topic, self._tcp_callback, 10)
        self.create_subscription(
            Image,
            self.bin_detect_overlay_topic,
            lambda msg: self._camera_overlay_callback('bin', self.bin_detect_overlay_topic, msg),
            5,
        )
        self.create_subscription(
            Image,
            self.tray_detect_overlay_topic,
            lambda msg: self._camera_overlay_callback('tray', self.tray_detect_overlay_topic, msg),
            5,
        )

    def _declare_name_parameter(self, parameter_name: str, default_value: str) -> str:
        value = str(self.declare_parameter(parameter_name, default_value).value).strip()
        return value or default_value

    def set_online_start_handler(self, handler: Callable[[], TriggerResult]) -> None:
        with self._online_start_handler_lock:
            self._online_start_handler = handler

    def set_online_load_program_handler(
        self,
        handler: Callable[[OnlineProgramLoad], OnlineProgramLoadResult],
    ) -> None:
        with self._online_load_program_handler_lock:
            self._online_load_program_handler = handler

    def set_online_validate_handler(self, handler: Callable[[], TriggerResult]) -> None:
        with self._online_validate_handler_lock:
            self._online_validate_handler = handler

    def set_online_place_handler(self, handler: Callable[[], TriggerResult]) -> None:
        with self._online_place_handler_lock:
            self._online_place_handler = handler

    def _handle_online_start(self, request, response):
        del request
        with self._online_start_handler_lock:
            handler = self._online_start_handler
        if handler is None:
            response.success = False
            response.message = 'Robot Cell Orchestrator GUI is not ready'
            return response
        result = handler()
        response.success = bool(result.success)
        response.message = result.message
        return response

    def _handle_online_load_program(self, request, response):
        with self._online_load_program_handler_lock:
            handler = self._online_load_program_handler
        if handler is None:
            response.success = False
            response.message = 'Robot Cell Orchestrator GUI is not ready'
            response.runtime_files = []
            return response
        result = handler(OnlineProgramLoad(
            qqc_id=str(request.qqc_id).strip(),
            bin_teach_file=str(request.bin_teach_file).strip(),
            item_teach_file=str(request.item_teach_file).strip(),
            tray_teach_file=str(request.tray_teach_file).strip(),
            tray_x_mm=float(request.tray_x_mm),
            tray_y_mm=float(request.tray_y_mm),
            tray_rz_deg=float(request.tray_rz_deg),
        ))
        response.success = bool(result.success)
        response.message = result.message
        response.runtime_files = list(result.runtime_files)
        return response

    def _handle_online_validate(self, request, response):
        del request
        with self._online_validate_handler_lock:
            handler = self._online_validate_handler
        if handler is None:
            response.success = False
            response.message = 'Robot Cell Orchestrator GUI is not ready'
            return response
        result = handler()
        response.success = bool(result.success)
        response.message = result.message
        return response

    def _handle_online_place(self, request, response):
        del request
        with self._online_place_handler_lock:
            handler = self._online_place_handler
        if handler is None:
            response.success = False
            response.message = 'Robot Cell Orchestrator GUI is not ready'
            return response
        result = handler()
        response.success = bool(result.success)
        response.message = result.message
        return response

    def publish_phase_event(self, event: str, *, cycle_index: int | None = None, message: str = '') -> int:
        with self._phase_event_lock:
            self._phase_event_seq += 1
            phase_id = self._phase_event_seq
        payload = {
            'event': event,
            'phase_id': phase_id,
            'cycle_index': cycle_index,
            'timestamp': time.time(),
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._phase_event_pub.publish(msg)
        return phase_id

    def _required_service_clients(self) -> list[tuple[str, object]]:
        return [
            *self._trigger_clients.values(),
            (self.item_auto_repick_service, self._item_auto_repick_client),
            (self.tray_arm_service, self._tray_arm_client),
        ]

    def service_names(self) -> list[tuple[str, str]]:
        return [
            ('Item Go Teach', self.item_go_to_teach_service),
            ('Item Pick Arm', self.item_arm_service),
            ('Item Pick Arm Status', self.item_arm_status_service),
            ('Item Pick Auto Repick', self.item_auto_repick_service),
            ('Item Seek', self.item_seek_service),
            ('Item Repick', self.item_repick_service),
            ('Item Seek Status', self.item_seek_status_service),
            ('Tray Go Teach', self.tray_go_to_teach_service),
            ('Tray Intercept Arm Start', self.tray_arm_service),
            ('Tray Intercept Arm Status', self.tray_arm_status_service),
            ('Tray Seek', self.tray_seek_service),
            ('Tray Seek Status', self.tray_seek_status_service),
            ('Online Load Program', self.online_load_program_service),
            ('Online Start', self.online_start_service),
            ('Online Validate', self.online_validate_service),
            ('Online Place', self.online_place_service),
        ]

    def topic_names(self) -> list[tuple[str, str]]:
        return [
            ('Robot Feedback', self.robot_tcp_topic),
            ('Bin Detect Overlay', self.bin_detect_overlay_topic),
            ('Tray Detect Overlay', self.tray_detect_overlay_topic),
        ]

    def camera_view_topics(self) -> list[tuple[str, str, str]]:
        return [
            ('bin', 'Bin Detect', self.bin_detect_overlay_topic),
            ('tray', 'Tray Detect', self.tray_detect_overlay_topic),
        ]

    def latest_camera_frame(self, key: str) -> CameraViewFrame | None:
        with self._camera_frame_lock:
            return self._camera_frames.get(key)

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
        result = self.check_trigger_services_now()
        while rclpy.ok() and not result.success and time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                result = TriggerResult(False, 'Stopped while checking required services')
                break
            time.sleep(SERVICE_READY_POLL_SEC)
            result = self.check_trigger_services_now()

        if not result.success:
            result = TriggerResult(False, f'Required services unavailable after {timeout_sec:.1f}s: {result.message}')
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

    def service_readiness_map(self) -> dict[str, bool]:
        readiness = {
            service_name: bool(client.service_is_ready())
            for service_name, client in self._required_service_clients()
        }
        readiness[self.online_load_program_service] = True
        readiness[self.online_start_service] = True
        readiness[self.online_validate_service] = True
        readiness[self.online_place_service] = True
        return readiness

    def _cache_startup_service_result(self, result: TriggerResult) -> None:
        with self._startup_service_lock:
            if self._startup_service_result is None:
                self._startup_service_result = result

    def _camera_overlay_callback(self, key: str, topic: str, msg: Image) -> None:
        try:
            rgb = self._camera_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            rgb_data = _normalized_rgb_image(rgb)
        except Exception as exc:
            now = time.monotonic()
            last_log_time = self._camera_error_log_time.get(key, 0.0)
            if now - last_log_time > 5.0:
                self.get_logger().warning(f'Failed to convert {topic} camera overlay: {exc}')
                self._camera_error_log_time[key] = now
            return

        frame = CameraViewFrame(
            topic=topic,
            rgb_data=rgb_data,
            received_monotonic=time.monotonic(),
        )
        with self._camera_frame_lock:
            self._camera_frames[key] = frame

    def _require_startup_services(self) -> TriggerResult:
        return self.check_trigger_services_now()

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
        config: RobotCellOrchestratorRuntimeSettings,
        wait_response_sec: float = ARM_CLICK_RESPONSE_TIMEOUT_SEC,
    ) -> TriggerResult:
        service_name = self.tray_arm_service
        client = self._tray_arm_client
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
        return TriggerResult(bool(response.started), f'{service_name}: {response.message} ({applied})')

    def set_item_auto_repick(
        self,
        enabled: bool,
        wait_response_sec: float = ARM_CLICK_RESPONSE_TIMEOUT_SEC,
    ) -> TriggerResult:
        service_name = self.item_auto_repick_service
        client = self._item_auto_repick_client
        if not client.service_is_ready():
            return TriggerResult(False, f'Service unavailable: {service_name}')

        request = SetBool.Request()
        request.data = bool(enabled)
        try:
            future = client.call_async(request)
        except Exception as exc:
            return TriggerResult(False, f'Failed to set Auto Repick via {service_name}: {exc}')

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
            RobotCellOrchestratorNode._angle_delta_deg(lhs[3], rhs[3]),
            RobotCellOrchestratorNode._angle_delta_deg(lhs[4], rhs[4]),
            RobotCellOrchestratorNode._angle_delta_deg(lhs[5], rhs[5]),
        )
        return linear_delta, rot_delta

    @staticmethod
    def _angle_delta_deg(lhs: float, rhs: float) -> float:
        return abs((float(lhs) - float(rhs) + 180.0) % 360.0 - 180.0)


class RobotCellOrchestratorGui:
    def __init__(self, node: RobotCellOrchestratorNode, startup_result: TriggerResult) -> None:
        self.node = node
        self.node.set_online_load_program_handler(self._online_load_program_service_request)
        self.node.set_online_start_handler(self._online_start_service_request)
        self.node.set_online_validate_handler(self._online_validate_service_request)
        self.node.set_online_place_handler(self._online_place_service_request)
        self._runtime_settings = self._load_robot_cell_orchestrator_runtime_settings()
        self.root = tk.Tk()
        self.root.title('Robot Cell Orchestrator Control')
        self.root.geometry(self._runtime_settings.window_geometry)
        self.root.minsize(1060, 780)
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._worker_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._cycle_count = 0
        self._robot_status = ROBOT_STATUS_STOP
        self._mode = MODE_OFFLINE
        self._online_command_condition = threading.Condition()
        self._online_pick_requested = False
        self._online_place_requested = False
        self._online_phase = ONLINE_PHASE_STOPPED
        self._offline_step_condition = threading.Condition()
        self._offline_step_requested = False
        self._offline_step_waiting = False
        self._offline_step_button_label = 'Next Step'
        self.eye_on_hand_calibration_var = tk.StringVar(
            value=self._runtime_settings.eye_on_hand_calibration_file
        )
        self.eye_to_hand_calibration_var = tk.StringVar(
            value=self._runtime_settings.eye_to_hand_calibration_file
        )
        self.platform_calibration_var = tk.StringVar(
            value=self._runtime_settings.platform_calibration_file
        )
        self._last_calibration_scan = self._scan_calibrations()
        self._last_runtime_scan = self._scan_runtime()
        self._last_offline_scan: OfflineTeachScan | None = None
        self._last_service_result = self.node.check_trigger_services_now()
        self._launch_processes: dict[str, subprocess.Popen] = {}
        self._launch_process_groups: dict[str, int | None] = {}
        self._launch_process_labels: dict[str, str] = {}
        self._launch_buttons: dict[str, tk.Button] = {}
        self._node_launcher_headless_button: tk.Checkbutton | None = None
        self._cell_bridge_process: subprocess.Popen | None = None
        self._cell_bridge_process_group: int | None = None
        self._cell_bridge_mode: str | None = None
        self._cell_bridge_button: tk.Button | None = None
        self._robot_ip_entry: tk.Entry | None = None
        self._robot_ip_save_button: tk.Button | None = None
        self._robot_ip_reload_button: tk.Button | None = None
        self._calibration_buttons: dict[str, tk.Button] = {}
        self._calibration_clear_buttons: dict[str, tk.Button] = {}
        self._service_labels: dict[str, tk.Label] = {}
        self._teach_options: dict[str, list[Path]] = {}
        self._teach_buttons: dict[str, tk.Button] = {}
        self._teach_delete_buttons: dict[str, tk.Button] = {}
        self._runtime_settings_save_after_id: str | None = None
        self._suspend_runtime_settings_events = False
        self._last_saved_window_geometry = self._runtime_settings.window_geometry
        self._camera_window: tk.Toplevel | None = None
        self._camera_viewer_frame: tk.Frame | None = None
        self._view_cameras_button: tk.Button | None = None
        self._right_scroll_canvas: tk.Canvas | None = None
        self._camera_viewer_visible = False
        self._camera_canvases: dict[str, tk.Canvas] = {}
        self._camera_image_refs: dict[str, tk.PhotoImage] = {}
        self._camera_rendered_views: dict[str, tuple[float, int, int]] = {}

        self.status_var = tk.StringVar(value='Idle')
        self.robot_status_var = tk.StringVar(value=ROBOT_STATUS_LABELS[self._robot_status])
        self.mode_var = tk.StringVar(value='Offline')
        self.robot_ip_var = tk.StringVar(value=self._station_robot_ip_or_default())
        self.calibration_status_var = tk.StringVar(value='Calibration: scanning...')
        self.teach_status_var = tk.StringVar(value='Teach files: scanning...')
        self.runtime_status_var = tk.StringVar(value='Runtime: scanning...')
        self.service_status_var = tk.StringVar(value='Services: scanning...')
        self.node_launcher_headless_var = tk.BooleanVar(value=False)
        self.loop_var = tk.BooleanVar(value=self._runtime_settings.loop_enabled)
        self.auto_repick_var = tk.BooleanVar(value=self._runtime_settings.auto_repick_enabled)
        self.step_mode_var = tk.BooleanVar(value=self._runtime_settings.step_mode_enabled)
        self.tray_seek_stability_sec_var = tk.DoubleVar(value=self._runtime_settings.tray_seek_stability_sec)
        self.tray_intercept_x_var = tk.DoubleVar(value=self._runtime_settings.tray_intercept_x_offset_mm)
        self.tray_intercept_y_var = tk.DoubleVar(value=self._runtime_settings.tray_intercept_y_offset_mm)
        self.tray_ee_angle_var = tk.DoubleVar(value=self._runtime_settings.tray_ee_angle_deg)
        self.bin_teach_var = tk.StringVar()
        self.item_teach_var = tk.StringVar()
        self.tray_teach_var = tk.StringVar()

        self._build_launch_specs()
        self._build_ui(startup_result)
        self._configure_runtime_setting_traces()
        self._refresh_teach_buttons()
        self._refresh_status_views()
        self._log(
            'Startup service check: '
            f'{"OK" if startup_result.success else "WAITING"} - {startup_result.message}'
        )
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.bind('<Configure>', self._on_window_configure, add='+')
        self.root.after(100, self._drain_queue)
        self.root.after(CAMERA_VIEW_REFRESH_MS, self._periodic_camera_view_refresh)
        self.root.after(READINESS_SCAN_MS, self._periodic_readiness_scan)
        self.root.after(PROCESS_SCAN_MS, self._periodic_process_scan)

    def _make_scrollable_side_panel(self, parent: tk.Widget) -> tuple[tk.Frame, tk.Frame]:
        container = tk.Frame(parent)
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)

        canvas = tk.Canvas(container, highlightthickness=0, borderwidth=0)
        scrollbar = tk.Scrollbar(container, orient=tk.VERTICAL, command=canvas.yview)
        content = tk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=content, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar.grid(row=0, column=1, sticky='ns')
        self._right_scroll_canvas = canvas

        def refresh_scroll_region(_event: tk.Event | None = None) -> None:
            canvas.configure(scrollregion=canvas.bbox('all'))

        def match_content_width(event: tk.Event) -> None:
            canvas.itemconfigure(window_id, width=max(1, int(event.width)))

        def scroll_units(event: tk.Event) -> int:
            if getattr(event, 'num', None) == 4:
                return -3
            if getattr(event, 'num', None) == 5:
                return 3
            delta = getattr(event, 'delta', 0)
            return int(-1 * (delta / 120)) if delta else 0

        def on_mousewheel(event: tk.Event) -> None:
            units = scroll_units(event)
            if units:
                canvas.yview_scroll(units, 'units')

        def bind_wheel(_event: tk.Event) -> None:
            canvas.bind_all('<MouseWheel>', on_mousewheel)
            canvas.bind_all('<Button-4>', on_mousewheel)
            canvas.bind_all('<Button-5>', on_mousewheel)

        def unbind_wheel(_event: tk.Event) -> None:
            canvas.unbind_all('<MouseWheel>')
            canvas.unbind_all('<Button-4>')
            canvas.unbind_all('<Button-5>')

        content.bind('<Configure>', refresh_scroll_region)
        canvas.bind('<Configure>', match_content_width)
        container.bind('<Enter>', bind_wheel)
        container.bind('<Leave>', unbind_wheel)
        canvas.bind('<Enter>', bind_wheel)
        canvas.bind('<Leave>', unbind_wheel)
        content.bind('<Enter>', bind_wheel)
        content.bind('<Leave>', unbind_wheel)
        return container, content

    @property
    def runtime_dir(self) -> Path:
        return workspace_path('runtime')

    @property
    def bin_teach_dir(self) -> Path:
        return workspace_path('teach', 'bin_teach')

    @property
    def item_teach_dir(self) -> Path:
        return workspace_path('teach', 'item_teach')

    @property
    def tray_teach_dir(self) -> Path:
        return workspace_path('teach', 'tray_teach')

    @property
    def item_detect_runtime_settings_file(self) -> Path:
        return workspace_path('config', 'item_perception', 'item_detect_runtime_settings.yaml')

    @property
    def tray_detect_runtime_settings_file(self) -> Path:
        return workspace_path('config', 'tray_perception', 'tray_detect_runtime_settings.yaml')

    @property
    def item_selected_profile_export_file(self) -> Path:
        return workspace_path('config', 'item_perception', 'item_detect_selected_profile.txt')

    @property
    def item_pick_runtime_settings_file(self) -> Path:
        return workspace_path('config', 'item_perception', 'item_pick_runtime_settings.json')

    @property
    def tray_intercept_runtime_settings_file(self) -> Path:
        return workspace_path('config', 'tray_perception', 'tray_intercept_runtime_settings.json')

    @property
    def robot_cell_orchestrator_runtime_settings_file(self) -> Path:
        return workspace_path('config', 'robot_cell_orchestrator', 'robot_cell_orchestrator_runtime_settings.yaml')

    @property
    def station_config_file(self) -> Path:
        return workspace_path('station_config')

    def _station_robot_ip(self) -> str:
        return key_value_config(self.station_config_file).get(ROBOT_IP_CONFIG_KEY, '').strip()

    def _station_robot_ip_or_default(self) -> str:
        return self._station_robot_ip() or ROBOT_LAN2_DEFAULT_IP

    def _reload_robot_ip_clicked(self) -> None:
        self.robot_ip_var.set(self._station_robot_ip_or_default())
        self._log(f'Reloaded robot IP from station_config: {self.robot_ip_var.get().strip()}')

    def _save_robot_ip_clicked(self) -> None:
        if self._running:
            messagebox.showwarning(
                'Cycle Running',
                'Stop the current cycle before changing the robot IP address.',
                parent=self.root,
            )
            return

        raw_ip = self.robot_ip_var.get().strip()
        normalized_ip = normalize_ipv4_address(raw_ip)
        if normalized_ip is None:
            messagebox.showerror(
                'Invalid Robot IP',
                f'Enter a valid IPv4 address for {ROBOT_IP_CONFIG_KEY}.',
                parent=self.root,
            )
            self._log(f'Invalid robot IP rejected: {raw_ip!r}')
            return

        previous_ip = self._station_robot_ip()
        try:
            update_key_value_config(self.station_config_file, ROBOT_IP_CONFIG_KEY, normalized_ip)
        except Exception as exc:
            messagebox.showerror(
                'Robot IP Save Failed',
                f'Could not update {self.station_config_file}: {exc}',
                parent=self.root,
            )
            self._log(f'Failed to save robot IP {normalized_ip}: {exc}')
            return

        self.robot_ip_var.set(normalized_ip)
        self._log(f'Saved robot IP: {previous_ip or "unset"} -> {normalized_ip}')
        if self._launch_processes.get('robot_bringup') is not None:
            messagebox.showinfo(
                'Restart Robot Bringup',
                'Robot IP saved. Stop and relaunch Robot Bringup to connect to the new controller IP.',
                parent=self.root,
            )

    def _refresh_robot_ip_controls(self) -> None:
        setting_state = tk.DISABLED if self._running else tk.NORMAL
        if self._robot_ip_entry is not None:
            self._robot_ip_entry.configure(state=setting_state)
        if self._robot_ip_save_button is not None:
            self._robot_ip_save_button.configure(state=setting_state)
        if self._robot_ip_reload_button is not None:
            self._robot_ip_reload_button.configure(state=setting_state)

    @property
    def cell_bridge_source_dir(self) -> Path:
        return workspace_path('cell_external_bridge', 'src')

    @property
    def cell_bridge_datalog_dir(self) -> Path:
        return workspace_path('debug files', 'cell_external_bridge')

    def _load_robot_cell_orchestrator_runtime_settings(self) -> RobotCellOrchestratorRuntimeSettings:
        defaults = RobotCellOrchestratorRuntimeSettings(
            loop_enabled=False,
            auto_repick_enabled=True,
            step_mode_enabled=False,
            tray_seek_stability_sec=ROBOT_STABILITY_SEC_DEFAULT,
            tray_intercept_x_offset_mm=0.0,
            tray_intercept_y_offset_mm=0.0,
            tray_ee_angle_deg=0.0,
            eye_on_hand_calibration_file='',
            eye_to_hand_calibration_file='',
            platform_calibration_file='',
        )
        payload = _load_yaml_mapping(workspace_path('config', 'robot_cell_orchestrator', 'robot_cell_orchestrator_runtime_settings.yaml')) or {}
        return RobotCellOrchestratorRuntimeSettings(
            loop_enabled=defaults.loop_enabled,
            auto_repick_enabled=_coerce_bool(
                payload.get('auto_repick_enabled'),
                defaults.auto_repick_enabled,
            ),
            step_mode_enabled=_coerce_bool(payload.get('step_mode_enabled'), defaults.step_mode_enabled),
            tray_seek_stability_sec=_coerce_float(
                payload.get('tray_seek_stability_sec'),
                defaults.tray_seek_stability_sec,
                TIMING_SLIDER_MIN_SEC,
                TIMING_SLIDER_MAX_SEC,
            ),
            tray_intercept_x_offset_mm=_coerce_float(
                payload.get('tray_intercept_x_offset_mm'),
                defaults.tray_intercept_x_offset_mm,
                TRAY_INTERCEPT_X_OFFSET_MIN,
                TRAY_INTERCEPT_X_OFFSET_MAX,
            ),
            tray_intercept_y_offset_mm=_coerce_float(
                payload.get('tray_intercept_y_offset_mm'),
                defaults.tray_intercept_y_offset_mm,
                TRAY_INTERCEPT_Y_OFFSET_MIN,
                TRAY_INTERCEPT_Y_OFFSET_MAX,
            ),
            tray_ee_angle_deg=_coerce_float(
                payload.get('tray_ee_angle_deg'),
                defaults.tray_ee_angle_deg,
                TRAY_EE_ANGLE_MIN_DEG,
                TRAY_EE_ANGLE_MAX_DEG,
            ),
            eye_on_hand_calibration_file=str(
                payload.get('eye_on_hand_calibration_file', defaults.eye_on_hand_calibration_file) or ''
            ).strip(),
            eye_to_hand_calibration_file=str(
                payload.get('eye_to_hand_calibration_file', defaults.eye_to_hand_calibration_file) or ''
            ).strip(),
            platform_calibration_file=str(
                payload.get('platform_calibration_file', defaults.platform_calibration_file) or ''
            ).strip(),
            window_geometry=_normalize_window_geometry(payload.get('window_geometry'), defaults.window_geometry),
        )

    def _current_window_geometry(self) -> str:
        try:
            self.root.update_idletasks()
            return _normalize_window_geometry(self.root.winfo_geometry(), self._runtime_settings.window_geometry)
        except Exception:
            return self._runtime_settings.window_geometry

    def _save_robot_cell_orchestrator_runtime_settings(self) -> None:
        self._runtime_settings_save_after_id = None
        was_suspended = self._suspend_runtime_settings_events
        self._suspend_runtime_settings_events = True
        try:
            settings = self._read_runtime_settings()
        finally:
            self._suspend_runtime_settings_events = was_suspended
        payload = {
            'schema_version': 1,
            'auto_repick_enabled': settings.auto_repick_enabled,
            'step_mode_enabled': settings.step_mode_enabled,
            'tray_seek_stability_sec': settings.tray_seek_stability_sec,
            'tray_intercept_x_offset_mm': settings.tray_intercept_x_offset_mm,
            'tray_intercept_y_offset_mm': settings.tray_intercept_y_offset_mm,
            'tray_ee_angle_deg': settings.tray_ee_angle_deg,
            'eye_on_hand_calibration_file': settings.eye_on_hand_calibration_file,
            'eye_to_hand_calibration_file': settings.eye_to_hand_calibration_file,
            'platform_calibration_file': settings.platform_calibration_file,
            'window_geometry': settings.window_geometry,
        }
        path = self.robot_cell_orchestrator_runtime_settings_file
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            if yaml is not None:
                with path.open('w', encoding='utf-8') as outfile:
                    yaml.safe_dump(payload, outfile, sort_keys=False)
            else:
                with path.open('w', encoding='utf-8') as outfile:
                    for key, value in payload.items():
                        outfile.write(f'{key}: {value}\n')
            self._runtime_settings = settings
            self._last_saved_window_geometry = settings.window_geometry
        except Exception as exc:
            self._log(f'Failed to save Robot Cell Orchestrator runtime settings: {exc}')

    def _cancel_scheduled_robot_cell_orchestrator_runtime_settings_save(self) -> None:
        if self._runtime_settings_save_after_id is None:
            return
        try:
            self.root.after_cancel(self._runtime_settings_save_after_id)
        except tk.TclError:
            pass
        self._runtime_settings_save_after_id = None

    def _flush_robot_cell_orchestrator_runtime_settings(self) -> None:
        self._cancel_scheduled_robot_cell_orchestrator_runtime_settings_save()
        self._save_robot_cell_orchestrator_runtime_settings()

    def _schedule_robot_cell_orchestrator_runtime_settings_save(self) -> None:
        if self._runtime_settings_save_after_id is not None:
            try:
                self.root.after_cancel(self._runtime_settings_save_after_id)
            except tk.TclError:
                pass
            self._runtime_settings_save_after_id = None
        self._runtime_settings_save_after_id = self.root.after(
            RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS,
            self._save_robot_cell_orchestrator_runtime_settings,
        )

    def _on_runtime_setting_changed(self, *_args: object) -> None:
        if self._suspend_runtime_settings_events:
            return
        self._schedule_robot_cell_orchestrator_runtime_settings_save()

    def _auto_repick_changed(self) -> None:
        if self._suspend_runtime_settings_events:
            return
        self._schedule_robot_cell_orchestrator_runtime_settings_save()
        enabled = bool(self.auto_repick_var.get())

        def worker() -> None:
            result = self.node.set_item_auto_repick(enabled)
            state = 'ON' if enabled else 'OFF'
            if result.success:
                self._log(f'Auto Repick {state}: {result.message}')
            else:
                self._log(f'Auto Repick {state}: FAIL - {result.message}')
                self._set_status(f'Auto Repick update failed: {result.message}')

        threading.Thread(target=worker, daemon=True).start()

    def _on_window_configure(self, event: tk.Event) -> None:
        if self._suspend_runtime_settings_events:
            return
        if event.widget is not self.root:
            return
        geometry = _normalize_window_geometry(
            f'{event.width}x{event.height}',
            self._last_saved_window_geometry,
        )
        if geometry == self._last_saved_window_geometry:
            return
        self._last_saved_window_geometry = geometry
        self._schedule_robot_cell_orchestrator_runtime_settings_save()

    def _configure_runtime_setting_traces(self) -> None:
        tracked_vars = [
            self.step_mode_var,
            self.tray_seek_stability_sec_var,
            self.tray_intercept_x_var,
            self.tray_intercept_y_var,
            self.tray_ee_angle_var,
        ]
        for var in tracked_vars:
            var.trace_add('write', self._on_runtime_setting_changed)

    def _selected_detect_profile_launch_args(self, kind: str, mode: str) -> list[str]:
        profile_path: Path | None = None
        if mode == MODE_OFFLINE:
            profile_path = self._selected_path(kind)
        elif mode == MODE_ONLINE:
            runtime_scan = self._last_runtime_scan if self._last_runtime_scan.ok else self._scan_runtime()
            paths = runtime_scan.by_kind.get(kind, [])
            if len(paths) == 1:
                profile_path = paths[0]

        if profile_path is None or not profile_path.exists():
            return []
        return [f'selected_profile_path:={profile_path}']

    def _build_launch_specs(self) -> None:
        def selected_calibration_arg(kind: str, argument_name: str = 'calibration_file') -> list[str]:
            path = self._selected_calibration_path(kind)
            return [f'{argument_name}:={path}'] if path is not None else []

        def offline_bin(mode: str, headless: bool) -> list[str]:
            args = [
                f'bin_teach_dir:={self.bin_teach_dir}',
                f'output_dir:={self.bin_teach_dir}',
                'auto_discover_platform_calibration:=false',
            ]
            args.extend(selected_calibration_arg(CALIBRATION_EYE_TO_HAND))
            args.extend(selected_calibration_arg(CALIBRATION_PLATFORM, 'platform_calibration_file'))
            return args

        def offline_item_teach(mode: str, headless: bool) -> list[str]:
            args = [
                f'profiles_dir:={self.item_teach_dir}',
                f'bin_teach_dir:={self.bin_teach_dir}',
                'auto_discover_calibration:=false',
            ]
            args.extend(selected_calibration_arg(CALIBRATION_EYE_TO_HAND))
            return args

        offline_tray_teach = lambda mode, headless: [f'profiles_dir:={self.tray_teach_dir}']

        def item_detect(mode: str, headless: bool) -> list[str]:
            args = [f'profiles_dir:={self.runtime_dir if mode == MODE_ONLINE else self.item_teach_dir}']
            args.extend(self._selected_detect_profile_launch_args(TEACH_KIND_ITEM, mode))
            args.extend(selected_calibration_arg(CALIBRATION_EYE_TO_HAND))
            if headless:
                args.append('headless:=true')
            return args

        def tray_detect(mode: str, headless: bool) -> list[str]:
            args = [f'profiles_dir:={self.runtime_dir if mode == MODE_ONLINE else self.tray_teach_dir}']
            args.extend(self._selected_detect_profile_launch_args(TEACH_KIND_TRAY, mode))
            args.extend(selected_calibration_arg(CALIBRATION_EYE_ON_HAND))
            if headless:
                args.append('headless:=true')
            return args

        def platform_calibration(mode: str, headless: bool) -> list[str]:
            return selected_calibration_arg(CALIBRATION_EYE_TO_HAND)

        def item_pick(mode: str, headless: bool) -> list[str]:
            if not headless:
                return []
            return [
                'headless:=true',
                'load_runtime_settings:=true',
                f'runtime_settings_file:={self.item_pick_runtime_settings_file}',
                f'item_profile_state_file:={self.item_selected_profile_export_file}',
            ]

        def tray_intercept(mode: str, headless: bool) -> list[str]:
            if not headless:
                return []
            return [
                'headless:=true',
                'load_runtime_settings:=true',
                f'runtime_settings_file:={self.tray_intercept_runtime_settings_file}',
            ]

        self._launch_specs: dict[str, LaunchSpec] = {
            'robot_bringup': LaunchSpec(
                'Robot Bringup',
                'cr_robot_ros2',
                'dobot_bringup_ros2.launch.py',
                lambda mode, headless: [f'station_config:={self.station_config_file}'],
            ),
            'rviz': LaunchSpec('RViz', 'dobot_rviz', 'dobot_rviz.launch.py', lambda mode, headless: []),
            'movement_debug': LaunchSpec('Movement Debug', 'motion_debug', 'motion_debug.launch.py', lambda mode, headless: []),
            'camera_launcher': LaunchSpec(
                'Camera Launcher',
                'orbbec_camera_launcher',
                'camera_launcher.launch.py',
                lambda mode, headless: [],
                headless_launch_file='camera_headless.launch.py',
                headless_label='Camera Launcher Headless',
            ),
            'camera_calibrate': LaunchSpec(
                'Camera Calibrate',
                'camera_calibration',
                'camera_calibration.launch.py',
                lambda mode, headless: ['calibration_mode:=eye_on_hand'],
                None,
                source_python_venv=False,
            ),
            'platform_calibration': LaunchSpec(
                'Platform Calibration',
                'platform_calibration',
                'platform_calibration.launch.py',
                platform_calibration,
                CALIBRATION_EYE_TO_HAND,
            ),
            'bin_teach': LaunchSpec(
                'Bin Teach',
                'item_perception',
                'bin_teach.launch.py',
                offline_bin,
                (CALIBRATION_EYE_TO_HAND, CALIBRATION_PLATFORM),
            ),
            'tray_teach': LaunchSpec('Tray Teach', 'tray_perception', 'tray_teach.launch.py', offline_tray_teach, None),
            'item_teach': LaunchSpec(
                'Item Teach',
                'item_perception',
                'item_teach.launch.py',
                offline_item_teach,
                CALIBRATION_EYE_TO_HAND,
            ),
            'item_pick': LaunchSpec('Item Pick', 'item_pick', 'item_pick.launch.py', item_pick, headless_label='Item Pick Headless'),
            'item_detect': LaunchSpec(
                'Item Detect',
                'item_perception',
                'item_detect.launch.py',
                item_detect,
                CALIBRATION_EYE_TO_HAND,
                headless_label='Item Detect Headless',
            ),
            'tray_intercept': LaunchSpec(
                'Tray Intercept',
                'tray_intercept',
                'tray_intercept.launch.py',
                tray_intercept,
                headless_label='Tray Intercept Headless',
            ),
            'tray_detect': LaunchSpec(
                'Tray Detect',
                'tray_perception',
                'tray_detect.launch.py',
                tray_detect,
                CALIBRATION_EYE_ON_HAND,
                headless_label='Tray Detect Headless',
            ),
        }

    def _build_ui(self, startup_result: TriggerResult) -> None:
        outer = tk.Frame(self.root, padx=12, pady=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=0)
        outer.rowconfigure(4, weight=1)

        left = tk.Frame(outer)
        left.grid(row=0, column=0, rowspan=5, sticky='nsew', padx=(0, 12))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(3, weight=1)

        controls = tk.LabelFrame(left, text='Cycle Controls', padx=10, pady=10)
        controls.grid(row=0, column=0, sticky='ew')
        for column in range(4):
            controls.columnconfigure(column, weight=1)

        self.offline_button = tk.Button(
            controls,
            text='Offline',
            command=lambda: self._set_mode(MODE_OFFLINE),
            width=12,
        )
        self.offline_button.grid(row=0, column=0, sticky='ew', padx=(0, 6))
        self.online_button = tk.Button(
            controls,
            text='Online',
            command=lambda: self._set_mode(MODE_ONLINE),
            width=12,
        )
        self.online_button.grid(row=0, column=1, sticky='ew', padx=(0, 12))
        self.start_button = tk.Button(
            controls,
            text='Start Cycle',
            command=self._start_clicked,
            width=12,
        )
        self.start_button.grid(row=0, column=2, sticky='ew', padx=(0, 8))
        self.stop_button = tk.Button(controls, text='Stop', command=self._stop_clicked, width=12, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=3, sticky='ew')

        self.loop_check = tk.Checkbutton(controls, text='Loop after successful cycle', variable=self.loop_var)
        self.loop_check.grid(row=1, column=0, columnspan=2, sticky='w', pady=(10, 0))
        self.auto_repick_check = tk.Checkbutton(
            controls,
            text='Auto Repick',
            variable=self.auto_repick_var,
            command=self._auto_repick_changed,
        )
        self.auto_repick_check.grid(row=2, column=0, sticky='w', pady=(8, 0))
        self.step_mode_check = tk.Checkbutton(
            controls,
            text='Step Mode',
            variable=self.step_mode_var,
            command=self._refresh_step_controls,
        )
        self.step_mode_check.grid(row=2, column=1, sticky='w', pady=(8, 0))
        self.step_button = tk.Button(
            controls,
            text='Next Step',
            command=self._step_clicked,
            width=12,
            state=tk.DISABLED,
        )
        self.step_button.grid(row=2, column=2, sticky='ew', padx=(0, 8), pady=(8, 0))
        cycle_status_frame = tk.Frame(controls)
        cycle_status_frame.grid(row=2, column=3, sticky='e', pady=(8, 0))
        tk.Label(cycle_status_frame, text='Status').grid(row=0, column=0, sticky='e', padx=(0, 6))
        self.cycle_status_label = tk.Label(
            cycle_status_frame,
            textvariable=self.robot_status_var,
            width=16,
            anchor='center',
            relief=tk.SUNKEN,
        )
        self.cycle_status_label.grid(row=0, column=1, sticky='e')

        tk.Label(controls, text='Tray seek stability (s)').grid(row=1, column=2, sticky='w', pady=(10, 0))
        self.tray_seek_stability_sec_scale = tk.Scale(
            controls,
            from_=TIMING_SLIDER_MIN_SEC,
            to=TIMING_SLIDER_MAX_SEC,
            resolution=TIMING_SLIDER_STEP_SEC,
            orient=tk.HORIZONTAL,
            variable=self.tray_seek_stability_sec_var,
            length=170,
        )
        self.tray_seek_stability_sec_scale.grid(row=1, column=3, sticky='ew', pady=(4, 0))

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

        services_frame = tk.LabelFrame(left, text='Services', padx=10, pady=8)
        services_frame.grid(row=1, column=0, sticky='ew', pady=(10, 0))
        services_frame.columnconfigure(1, weight=1)
        for row, (label, service_name) in enumerate(self.node.service_names()):
            tk.Label(services_frame, text=label).grid(row=row, column=0, sticky='w')
            value_label = tk.Label(services_frame, text=service_name, anchor='w')
            value_label.grid(row=row, column=1, sticky='ew', padx=(12, 0))
            if service_name != self.node.online_start_service:
                self._service_labels[service_name] = value_label

        topics_frame = tk.LabelFrame(left, text='Watched Topics', padx=10, pady=8)
        topics_frame.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        topics_frame.columnconfigure(1, weight=1)
        for row, (label, topic_name) in enumerate(self.node.topic_names()):
            tk.Label(topics_frame, text=label).grid(row=row, column=0, sticky='w')
            tk.Label(topics_frame, text=topic_name, anchor='w').grid(row=row, column=1, sticky='ew', padx=(12, 0))

        log_frame = tk.LabelFrame(left, text='Cycle Log', padx=10, pady=8)
        log_frame.grid(row=3, column=0, sticky='nsew', pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, state=tk.DISABLED)
        self.log_text.grid(row=0, column=0, sticky='nsew')

        right_container, right = self._make_scrollable_side_panel(outer)
        right_container.grid(row=0, column=1, rowspan=5, sticky='nsew')
        right.columnconfigure(0, weight=1)

        station_frame = tk.LabelFrame(right, text='Robot Connection', padx=10, pady=8)
        station_frame.grid(row=0, column=0, sticky='ew')
        station_frame.columnconfigure(1, weight=1)
        tk.Label(station_frame, text='Robot IP').grid(row=0, column=0, sticky='w')
        self._robot_ip_entry = tk.Entry(station_frame, textvariable=self.robot_ip_var, width=16)
        self._robot_ip_entry.grid(row=0, column=1, sticky='ew', padx=(8, 6))
        self._robot_ip_entry.bind('<Return>', lambda _event: self._save_robot_ip_clicked())
        self._robot_ip_save_button = tk.Button(
            station_frame,
            text='Save',
            command=self._save_robot_ip_clicked,
            width=7,
        )
        self._robot_ip_save_button.grid(row=0, column=2, sticky='ew', padx=(0, 4))
        self._robot_ip_reload_button = tk.Button(
            station_frame,
            text='Reload',
            command=self._reload_robot_ip_clicked,
            width=7,
        )
        self._robot_ip_reload_button.grid(row=0, column=3, sticky='ew')
        tk.Label(
            station_frame,
            text=f'LAN2 default: {ROBOT_LAN2_DEFAULT_IP}',
            anchor='w',
            fg='#4a4a4a',
        ).grid(row=1, column=0, columnspan=4, sticky='ew', pady=(4, 0))

        dropdown_frame = tk.LabelFrame(right, text='Offline Teach Selection', padx=10, pady=8)
        dropdown_frame.grid(row=1, column=0, sticky='ew', pady=(10, 0))
        dropdown_frame.columnconfigure(1, weight=1)
        dropdown_frame.columnconfigure(2, weight=0)
        self._make_teach_picker_button(dropdown_frame, 'Bin', TEACH_KIND_BIN, self.bin_teach_var, self.bin_teach_dir, 0)
        self._make_teach_picker_button(dropdown_frame, 'Item', TEACH_KIND_ITEM, self.item_teach_var, self.item_teach_dir, 1)
        self._make_teach_picker_button(dropdown_frame, 'Tray', TEACH_KIND_TRAY, self.tray_teach_var, self.tray_teach_dir, 2)

        calibration_frame = tk.LabelFrame(right, text='Calibration Files', padx=10, pady=8)
        calibration_frame.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        calibration_frame.columnconfigure(1, weight=1)
        calibration_frame.columnconfigure(2, weight=0)
        self._make_calibration_picker_button(
            calibration_frame,
            CALIBRATION_EYE_ON_HAND,
            self.eye_on_hand_calibration_var,
            0,
        )
        self._make_calibration_picker_button(
            calibration_frame,
            CALIBRATION_EYE_TO_HAND,
            self.eye_to_hand_calibration_var,
            1,
        )
        self._make_calibration_picker_button(
            calibration_frame,
            CALIBRATION_PLATFORM,
            self.platform_calibration_var,
            2,
        )

        bridge_frame = tk.LabelFrame(right, text='External Bridge', padx=10, pady=8)
        bridge_frame.grid(row=3, column=0, sticky='ew', pady=(10, 0))
        bridge_frame.columnconfigure(0, weight=1)
        self._cell_bridge_button = tk.Button(
            bridge_frame,
            text='Cell External Bridge',
            command=self._cell_bridge_clicked,
            width=24,
        )
        self._cell_bridge_button.grid(row=0, column=0, sticky='ew')

        launch_frame = tk.LabelFrame(right, text='Node Launcher', padx=10, pady=8)
        launch_frame.grid(row=4, column=0, sticky='ew', pady=(10, 0))
        launch_frame.columnconfigure(0, weight=1)
        self._node_launcher_headless_button = tk.Checkbutton(
            launch_frame,
            text='Headless: OFF',
            variable=self.node_launcher_headless_var,
            command=self._node_launcher_headless_toggled,
            indicatoron=False,
            width=24,
        )
        self._node_launcher_headless_button.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        for row, key in enumerate(self._launch_specs):
            button = tk.Button(
                launch_frame,
                text=self._launch_specs[key].label,
                command=lambda launch_key=key: self._launch_clicked(launch_key),
                width=24,
            )
            button.grid(row=row + 1, column=0, sticky='ew', pady=(0 if row == 0 else 4, 0))
            self._launch_buttons[key] = button

        camera_button_frame = tk.LabelFrame(right, text='Camera Views', padx=10, pady=8)
        camera_button_frame.grid(row=5, column=0, sticky='ew', pady=(10, 0))
        camera_button_frame.columnconfigure(0, weight=1)
        self._view_cameras_button = tk.Button(
            camera_button_frame,
            text='Open Camera Window',
            command=self._view_cameras_clicked,
            width=24,
        )
        self._view_cameras_button.grid(row=0, column=0, sticky='ew')

        self._set_mode(MODE_OFFLINE)

    def _build_camera_viewer(self, parent: tk.Widget) -> tk.Frame:
        viewer = tk.Frame(parent, padx=10, pady=10)
        viewer.columnconfigure(0, weight=1)
        viewer.rowconfigure(0, weight=1)
        viewer.rowconfigure(1, weight=1)
        self._camera_viewer_frame = viewer

        empty_text = {
            'bin': 'No Bin Detect',
            'tray': 'No Tray Detect',
        }
        for row, (key, label, topic) in enumerate(self.node.camera_view_topics()):
            panel = tk.LabelFrame(viewer, text=f'{label}: /{topic.lstrip("/")}', padx=6, pady=6)
            panel.grid(row=row, column=0, sticky='nsew', pady=(0 if row == 0 else 10, 0))
            panel.columnconfigure(0, weight=1)
            panel.rowconfigure(0, weight=1)
            canvas = tk.Canvas(
                panel,
                width=CAMERA_VIEW_WIDTH,
                height=CAMERA_VIEW_HEIGHT,
                bg='#181818',
                highlightthickness=1,
                highlightbackground='#6a6a6a',
            )
            canvas.grid(row=0, column=0, sticky='nsew')
            self._camera_canvases[key] = canvas
            self._draw_camera_placeholder(key, empty_text.get(key, 'No Detect'))
        return viewer

    def _view_cameras_clicked(self) -> None:
        if self._camera_window is not None and self._camera_window.winfo_exists():
            self._camera_window.deiconify()
            self._camera_window.lift()
            self._camera_window.focus_set()
            return

        window = tk.Toplevel(self.root)
        window.title('Camera Detect Views')
        window.geometry(f'{CAMERA_WINDOW_DEFAULT_WIDTH}x{CAMERA_WINDOW_DEFAULT_HEIGHT}')
        window.minsize(CAMERA_WINDOW_MIN_WIDTH, CAMERA_WINDOW_MIN_HEIGHT)
        window.protocol('WM_DELETE_WINDOW', self._close_camera_window)
        viewer = self._build_camera_viewer(window)
        viewer.pack(fill=tk.BOTH, expand=True)
        self._camera_window = window
        self._camera_viewer_visible = True
        if self._view_cameras_button is not None:
            self._view_cameras_button.configure(
                text='Show Camera Window',
                state=tk.NORMAL,
                bg=RUNNING_BUTTON_BG,
                activebackground=RUNNING_BUTTON_BG,
            )
        self._refresh_camera_canvases()

    def _close_camera_window(self) -> None:
        if self._camera_window is not None and self._camera_window.winfo_exists():
            self._camera_window.destroy()
        self._camera_window = None
        self._camera_viewer_frame = None
        self._camera_viewer_visible = False
        self._camera_canvases.clear()
        self._camera_image_refs.clear()
        self._camera_rendered_views.clear()
        if self._view_cameras_button is not None:
            self._view_cameras_button.configure(
                text='Open Camera Window',
                state=tk.NORMAL,
                bg=self.root.cget('bg'),
                activebackground=self.root.cget('bg'),
            )

    def _periodic_camera_view_refresh(self) -> None:
        if self._camera_viewer_visible:
            self._refresh_camera_canvases()
        self.root.after(CAMERA_VIEW_REFRESH_MS, self._periodic_camera_view_refresh)

    def _refresh_camera_canvases(self) -> None:
        for key in self._camera_canvases:
            self._refresh_camera_canvas(key)

    def _refresh_camera_canvas(self, key: str) -> None:
        canvas = self._camera_canvases.get(key)
        if canvas is None:
            return
        width, height = self._camera_canvas_dimensions(canvas)
        frame = self.node.latest_camera_frame(key)
        placeholder_text = 'No Bin Detect' if key == 'bin' else 'No Tray Detect'
        if frame is None or time.monotonic() - frame.received_monotonic > CAMERA_VIEW_STALE_SEC:
            placeholder_view = (-1.0, width, height)
            if self._camera_rendered_views.get(key) != placeholder_view:
                self._draw_camera_placeholder(key, placeholder_text)
                self._camera_rendered_views[key] = placeholder_view
            return
        rendered_view = (frame.received_monotonic, width, height)
        if self._camera_rendered_views.get(key) == rendered_view:
            return

        try:
            photo = tk.PhotoImage(data=_rgb_image_to_view_ppm(frame.rgb_data, width, height), format='PPM')
        except tk.TclError as exc:
            self._log(f'Failed to render {frame.topic}: {exc}')
            self._draw_camera_placeholder(key, placeholder_text)
            self._camera_rendered_views[key] = (-1.0, width, height)
            return

        canvas.delete('all')
        canvas.create_image(0, 0, image=photo, anchor='nw')
        self._camera_image_refs[key] = photo
        self._camera_rendered_views[key] = rendered_view

    def _camera_canvas_dimensions(self, canvas: tk.Canvas) -> tuple[int, int]:
        width = int(canvas.winfo_width())
        height = int(canvas.winfo_height())
        if width <= 1:
            width = int(canvas.winfo_reqwidth())
        if height <= 1:
            height = int(canvas.winfo_reqheight())
        return max(1, width), max(1, height)

    def _draw_camera_placeholder(self, key: str, text: str) -> None:
        canvas = self._camera_canvases.get(key)
        if canvas is None:
            return
        width, height = self._camera_canvas_dimensions(canvas)
        canvas.delete('all')
        canvas.create_rectangle(0, 0, width, height, fill='#181818', outline='')
        canvas.create_text(
            width // 2,
            height // 2,
            text=text,
            fill='#f0f0f0',
            font=('TkDefaultFont', 16, 'bold'),
        )
        self._camera_image_refs.pop(key, None)

    def _make_teach_picker_button(
        self,
        parent: tk.Widget,
        label: str,
        kind: str,
        variable: tk.StringVar,
        directory: Path,
        row: int,
    ) -> tk.Button:
        tk.Label(parent, text=label).grid(row=row, column=0, sticky='w', pady=(0 if row == 0 else 6, 0))
        button = tk.Button(
            parent,
            text='Select file',
            command=lambda: self._select_teach_file(kind, variable, directory, label),
            anchor='w',
            width=26,
        )
        button.grid(row=row, column=1, sticky='ew', padx=(8, 0), pady=(0 if row == 0 else 6, 0))
        delete_button = tk.Button(
            parent,
            text='Delete',
            command=lambda: self._delete_teach_file(kind, variable, directory, label),
            width=8,
        )
        delete_button.grid(row=row, column=2, sticky='ew', padx=(6, 0), pady=(0 if row == 0 else 6, 0))
        variable.trace_add('write', lambda *_args: self._refresh_teach_button_texts())
        self._teach_buttons[kind] = button
        self._teach_delete_buttons[kind] = delete_button
        return button

    def _make_calibration_picker_button(
        self,
        parent: tk.Widget,
        kind: str,
        variable: tk.StringVar,
        row: int,
    ) -> tk.Button:
        pady = (0 if row == 0 else 6, 0)
        tk.Label(parent, text=kind).grid(row=row, column=0, sticky='w', pady=pady)
        button = tk.Button(
            parent,
            text='Select file',
            command=lambda: self._select_calibration_file(kind, variable),
            anchor='w',
            width=26,
        )
        button.grid(row=row, column=1, sticky='ew', padx=(8, 0), pady=pady)
        clear_button = tk.Button(
            parent,
            text='Clear',
            command=lambda: self._clear_calibration_file(kind, variable),
            width=8,
        )
        clear_button.grid(row=row, column=2, sticky='ew', padx=(6, 0), pady=pady)
        variable.trace_add('write', lambda *_args: self._calibration_selection_changed())
        self._calibration_buttons[kind] = button
        self._calibration_clear_buttons[kind] = clear_button
        return button

    def _calibration_variable(self, kind: str) -> tk.StringVar:
        return {
            CALIBRATION_EYE_ON_HAND: self.eye_on_hand_calibration_var,
            CALIBRATION_EYE_TO_HAND: self.eye_to_hand_calibration_var,
            CALIBRATION_PLATFORM: self.platform_calibration_var,
        }[kind]

    def _resolve_calibration_path(self, kind: str, raw_value: str) -> Path | None:
        selected = str(raw_value or '').strip()
        if not selected:
            return None
        path = Path(selected).expanduser()
        if not path.is_absolute():
            path = workspace_root() / path
        if not path.exists() or not path.is_file() or path.stat().st_size <= 0:
            return None
        if classify_calibration_yaml(path) != kind:
            return None
        return path.resolve()

    def _selected_calibration_path(self, kind: str) -> Path | None:
        variable = self._calibration_variable(kind)
        return self._resolve_calibration_path(kind, variable.get())

    def _select_calibration_file(self, kind: str, variable: tk.StringVar) -> None:
        current = self._resolve_calibration_path(kind, variable.get())
        initial_dir = current.parent if current is not None else workspace_path('calibration')
        initial_dir.mkdir(parents=True, exist_ok=True)
        pattern = CALIBRATION_PATTERNS.get(kind, '*.yaml')
        expected_label = f'{kind} calibration ({pattern})'
        selected = filedialog.askopenfilename(
            parent=self.root,
            title=f'Select {kind} calibration file',
            initialdir=str(initial_dir),
            filetypes=((expected_label, pattern),),
        )
        if not selected:
            return

        path = Path(selected).expanduser()
        actual_kind = classify_calibration_yaml(path)
        if actual_kind != kind:
            self._log(
                f'Calibration file {path.name} is {actual_kind}, expected {kind}. '
                'Select a calibration YAML with matching calibration_type metadata.'
            )
            messagebox.showerror(
                'Wrong Calibration Type',
                f'{path.name} is {actual_kind}; expected {kind}.',
                parent=self.root,
            )
            return

        variable.set(str(path.resolve()))
        self._log(f'Selected {kind} calibration: {path.name}')

    def _clear_calibration_file(self, kind: str, variable: tk.StringVar) -> None:
        variable.set('')
        self._log(f'Cleared {kind} calibration selection')

    def _calibration_selection_changed(self) -> None:
        self._last_calibration_scan = self._scan_calibrations()
        self._refresh_calibration_button_texts()
        self._schedule_robot_cell_orchestrator_runtime_settings_save()
        if hasattr(self, 'calibration_status_var'):
            self._refresh_status_views()

    def _refresh_calibration_button_texts(self) -> None:
        for kind, button in self._calibration_buttons.items():
            selected = self._selected_calibration_path(kind)
            button.configure(
                text=selected.name if selected is not None else 'Select file',
                fg='#1b7f3a' if selected is not None else '#b00020',
                state=tk.DISABLED if self._running else tk.NORMAL,
            )
            clear_button = self._calibration_clear_buttons.get(kind)
            if clear_button is not None:
                clear_button.configure(
                    state=tk.NORMAL if selected is not None and not self._running else tk.DISABLED
                )

    def _set_mode(self, mode: str) -> None:
        if mode not in (MODE_OFFLINE, MODE_ONLINE):
            return
        previous_mode = self._mode
        if mode != previous_mode and self._is_cell_bridge_running():
            self._stop_cell_bridge()
            self._log('Stopped Cell External Bridge while changing mode')
        self._mode = mode
        self.mode_var.set('Offline' if mode == MODE_OFFLINE else 'Online')
        self.offline_button.configure(relief=tk.SUNKEN if mode == MODE_OFFLINE else tk.RAISED)
        self.online_button.configure(relief=tk.SUNKEN if mode == MODE_ONLINE else tk.RAISED)
        self._refresh_status_views()
        self._log(f'Mode set to {self.mode_var.get()}')

    def _scan_calibrations(self) -> CalibrationScan:
        return CalibrationScan({
            kind: self._selected_calibration_path(kind)
            for kind in CALIBRATION_PATTERNS
        })

    def _scan_runtime(self) -> RuntimeScan:
        scan = RuntimeScan(root=self.runtime_dir)
        for path in yaml_files_in(scan.root):
            kind = classify_teach_yaml(path)
            if kind in RUNTIME_REQUIRED_KINDS or kind in RUNTIME_OPTIONAL_KINDS:
                scan.by_kind.setdefault(kind, []).append(path)
            else:
                scan.unknown_yaml.append(path)
        return scan

    def _teach_paths_by_kind(self, directory: Path, kind: str) -> list[Path]:
        return [path for path in yaml_files_in(directory) if classify_teach_yaml(path) == kind]

    def _refresh_teach_buttons(self) -> None:
        self._teach_options = {
            TEACH_KIND_BIN: self._teach_paths_by_kind(self.bin_teach_dir, TEACH_KIND_BIN),
            TEACH_KIND_ITEM: self._teach_paths_by_kind(self.item_teach_dir, TEACH_KIND_ITEM),
            TEACH_KIND_TRAY: self._teach_paths_by_kind(self.tray_teach_dir, TEACH_KIND_TRAY),
        }
        self._ensure_teach_selection(TEACH_KIND_BIN, self.bin_teach_var)
        self._ensure_teach_selection(TEACH_KIND_ITEM, self.item_teach_var)
        self._ensure_teach_selection(TEACH_KIND_TRAY, self.tray_teach_var)
        self._refresh_teach_button_texts()
        self._refresh_status_views()

    def _ensure_teach_selection(self, kind: str, variable: tk.StringVar) -> None:
        current = self._resolve_teach_variable_path(kind, variable.get())
        if current is not None:
            return
        paths = self._teach_options.get(kind, [])
        variable.set(str(paths[0]) if paths else '')

    def _refresh_teach_button_texts(self) -> None:
        for kind, button in self._teach_buttons.items():
            variable = {
                TEACH_KIND_BIN: self.bin_teach_var,
                TEACH_KIND_ITEM: self.item_teach_var,
                TEACH_KIND_TRAY: self.tray_teach_var,
            }[kind]
            selected = self._resolve_teach_variable_path(kind, variable.get())
            button.configure(text=selected.name if selected is not None else 'Select file')
            delete_button = self._teach_delete_buttons.get(kind)
            if delete_button is not None:
                delete_button.configure(state=tk.NORMAL if selected is not None and not self._running else tk.DISABLED)

    def _selected_path(self, kind: str) -> Path | None:
        variable = {
            TEACH_KIND_BIN: self.bin_teach_var,
            TEACH_KIND_ITEM: self.item_teach_var,
            TEACH_KIND_TRAY: self.tray_teach_var,
        }[kind]
        return self._resolve_teach_variable_path(kind, variable.get())

    def _resolve_teach_variable_path(self, kind: str, raw_value: str) -> Path | None:
        selected_name = str(raw_value).strip()
        if selected_name:
            path = Path(selected_name).expanduser()
            if not path.is_absolute():
                path = {
                    TEACH_KIND_BIN: self.bin_teach_dir,
                    TEACH_KIND_ITEM: self.item_teach_dir,
                    TEACH_KIND_TRAY: self.tray_teach_dir,
                }[kind] / path
            if path.exists() and path.is_file() and classify_teach_yaml(path) == kind:
                return path
        for path in self._teach_options.get(kind, []):
            if path.name == selected_name:
                return path
        return None

    def _select_teach_file(self, kind: str, variable: tk.StringVar, directory: Path, label: str) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        selected = filedialog.askopenfilename(
            parent=self.root,
            title=f'Select {label} teach file',
            initialdir=str(directory),
            filetypes=(('YAML files', '*.yaml *.yml'), ('All files', '*')),
        )
        if not selected:
            return
        path = Path(selected).expanduser()
        if not path.exists() or not path.is_file():
            self._log(f'{label} teach file not found: {path}')
            return
        actual_kind = classify_teach_yaml(path)
        if actual_kind != kind:
            self._log(f'{label} teach file {path.name} is {actual_kind}, expected {kind}')
            return
        variable.set(str(path))
        if path not in self._teach_options.get(kind, []):
            self._teach_options.setdefault(kind, []).append(path)
        self._refresh_teach_button_texts()
        self._refresh_status_views()
        self._log(f'Selected {label} teach file: {path.name}')

    def _delete_teach_file(self, kind: str, variable: tk.StringVar, directory: Path, label: str) -> None:
        if self._running:
            messagebox.showwarning(
                f'Delete {label} Teach',
                'Stop the current cycle before deleting teach files.',
                parent=self.root,
            )
            return

        path = self._resolve_teach_variable_path(kind, variable.get())
        if path is None:
            self._log(f'No {label.lower()} teach file selected to delete')
            return

        try:
            resolved_path = path.resolve()
            teach_dir = directory.resolve()
        except Exception:
            self._log(f'Could not resolve {label.lower()} teach file for delete: {path}')
            return
        if resolved_path.parent != teach_dir:
            self._log(f'Refusing to delete {label.lower()} teach file outside {teach_dir}: {resolved_path}')
            return

        confirmed = messagebox.askyesno(
            f'Delete {label} Teach',
            f'Delete {path.name}?\n\nThis removes the file from:\n{teach_dir}',
            parent=self.root,
        )
        if not confirmed:
            return

        try:
            resolved_path.unlink()
        except FileNotFoundError:
            self._log(f'{label} teach file already deleted: {path.name}')
        except Exception as exc:
            self._log(f'Failed to delete {label.lower()} teach file {path.name}: {exc}')
            return

        variable.set('')
        self._refresh_teach_buttons()
        self._log(f'Deleted {label} teach file: {path.name}')

    def _find_tool_for_item(self, item_path: Path | None) -> Path | None:
        if item_path is None:
            return None
        if item_profile_has_embedded_tool_teach(item_path):
            return item_path
        tool_paths = self._teach_paths_by_kind(item_path.parent, TEACH_KIND_TOOL)
        if not tool_paths:
            return None
        if len(tool_paths) == 1:
            return tool_paths[0]
        for tool_path in tool_paths:
            payload = _load_yaml_mapping(tool_path)
            target = ''
            if isinstance(payload, dict):
                target = str(payload.get('item_detect_profile_path', '')).strip()
            if target and (Path(target).expanduser().name == item_path.name or Path(target).expanduser() == item_path):
                return tool_path
        return None

    @staticmethod
    def _is_safe_yaml_filename(raw_name: str) -> bool:
        if not raw_name:
            return False
        normalized = raw_name.replace('\\', '/')
        name = Path(normalized).name
        return (
            normalized == name
            and name not in ('', '.', '..')
            and Path(name).suffix.lower() in ('.yaml', '.yml')
        )

    def _resolve_requested_teach_file(
        self,
        directory: Path,
        raw_name: str,
        expected_kind: str,
        label: str,
    ) -> tuple[Path | None, str | None]:
        if not self._is_safe_yaml_filename(raw_name):
            return None, f'{label} filename must be a local .yaml/.yml basename: {raw_name!r}'
        path = directory / raw_name
        if not path.exists() or not path.is_file():
            return None, f'{label} teach file not found: {path}'
        kind = classify_teach_yaml(path)
        if kind != expected_kind:
            return None, f'{label} teach file {path.name} is {kind}, expected {expected_kind}'
        return path, None

    @staticmethod
    def _range_error(label: str, value: float, minimum: float, maximum: float) -> str | None:
        if value < minimum or value > maximum:
            return f'{label} {value:.1f} outside allowed range {minimum:.1f}..{maximum:.1f}'
        return None

    def _write_selected_profile_state_file(self, profile_path: Path) -> None:
        try:
            self.item_selected_profile_export_file.parent.mkdir(parents=True, exist_ok=True)
            self.item_selected_profile_export_file.write_text(str(profile_path) + '\n', encoding='utf-8')
        except Exception as exc:
            self._log(f'Failed to write item selected profile state file: {exc}')

    def _write_tray_intercept_runtime_placement(self, load: OnlineProgramLoad) -> None:
        path = self.tray_intercept_runtime_settings_file
        payload: dict[str, object] = {}
        try:
            if path.exists():
                with path.open('r', encoding='utf-8') as infile:
                    loaded = json.load(infile)
                    if isinstance(loaded, dict):
                        payload = loaded
        except Exception as exc:
            self._log(f'Failed to read tray intercept runtime settings before update: {exc}')
        payload['tray_intercept_x_offset_mm'] = float(load.tray_x_mm)
        payload['tray_intercept_y_offset_mm'] = float(load.tray_y_mm)
        payload['ee_final_pose_angle_deg'] = float(load.tray_rz_deg)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('w', encoding='utf-8') as outfile:
                json.dump(payload, outfile, indent=2)
                outfile.write('\n')
        except Exception as exc:
            self._log(f'Failed to write tray intercept runtime settings: {exc}')

    def _load_online_program(self, load: OnlineProgramLoad) -> OnlineProgramLoadResult:
        if not load.qqc_id:
            return OnlineProgramLoadResult(False, 'missing qqc_id')
        if self._running and self._online_phase != ONLINE_PHASE_WAITING_FOR_PICK:
            return OnlineProgramLoadResult(False, f'Robot Cell Orchestrator is {self._online_phase}; runtime cannot change now')
        for label, value, minimum, maximum in (
            ('tray_x_mm', load.tray_x_mm, TRAY_INTERCEPT_X_OFFSET_MIN, TRAY_INTERCEPT_X_OFFSET_MAX),
            ('tray_y_mm', load.tray_y_mm, TRAY_INTERCEPT_Y_OFFSET_MIN, TRAY_INTERCEPT_Y_OFFSET_MAX),
            ('tray_rz_deg', load.tray_rz_deg, TRAY_EE_ANGLE_MIN_DEG, TRAY_EE_ANGLE_MAX_DEG),
        ):
            error = self._range_error(label, value, minimum, maximum)
            if error:
                return OnlineProgramLoadResult(False, error)

        bin_path, error = self._resolve_requested_teach_file(
            self.bin_teach_dir,
            load.bin_teach_file,
            TEACH_KIND_BIN,
            'bin',
        )
        if error:
            return OnlineProgramLoadResult(False, error)
        item_path, error = self._resolve_requested_teach_file(
            self.item_teach_dir,
            load.item_teach_file,
            TEACH_KIND_ITEM,
            'item',
        )
        if error:
            return OnlineProgramLoadResult(False, error)
        tray_path, error = self._resolve_requested_teach_file(
            self.tray_teach_dir,
            load.tray_teach_file,
            TEACH_KIND_TRAY,
            'tray',
        )
        if error:
            return OnlineProgramLoadResult(False, error)
        tool_path = self._find_tool_for_item(item_path)
        if tool_path is None or not tool_path.exists():
            return OnlineProgramLoadResult(False, f'item tool teach not found for {item_path.name}')

        sources = [bin_path, item_path, tray_path]
        if tool_path != item_path:
            sources.append(tool_path)
        try:
            self.runtime_dir.mkdir(parents=True, exist_ok=True)
            for existing in yaml_files_in(self.runtime_dir):
                existing.unlink()
            runtime_files: list[str] = []
            for source in sources:
                destination = self.runtime_dir / source.name
                shutil.copy2(source, destination)
                runtime_files.append(destination.name)
        except Exception as exc:
            return OnlineProgramLoadResult(False, f'failed to prepare runtime folder: {exc}')

        runtime_item_path = self.runtime_dir / item_path.name
        self._write_selected_profile_state_file(runtime_item_path)
        self._write_tray_intercept_runtime_placement(load)

        self.tray_intercept_x_var.set(float(load.tray_x_mm))
        self.tray_intercept_y_var.set(float(load.tray_y_mm))
        self.tray_ee_angle_var.set(float(load.tray_rz_deg))
        self._flush_robot_cell_orchestrator_runtime_settings()
        self._last_calibration_scan = self._scan_calibrations()
        self._last_runtime_scan = self._scan_runtime()
        self._last_service_result = self.node.check_trigger_services_now()
        self._refresh_status_views()

        if not self._last_runtime_scan.ok:
            return OnlineProgramLoadResult(
                False,
                'runtime validation failed after load: ' + self._last_runtime_scan.message,
                tuple(runtime_files),
            )
        if not self._last_calibration_scan.ok:
            return OnlineProgramLoadResult(
                False,
                'calibration not ready: ' + self._last_calibration_scan.message,
                tuple(runtime_files),
            )

        self._log(
            f'Online program {load.qqc_id} loaded to runtime: '
            f'bin={bin_path.name}, item={item_path.name}, tray={tray_path.name}, '
            f'tool={"embedded" if tool_path == item_path else tool_path.name}, tray x={load.tray_x_mm:.1f}, '
            f'y={load.tray_y_mm:.1f}, rz={load.tray_rz_deg:.1f}'
        )
        return OnlineProgramLoadResult(True, f'Loaded online program {load.qqc_id}', tuple(runtime_files))

    def _scan_offline_teach(self) -> OfflineTeachScan:
        item_path = self._selected_path(TEACH_KIND_ITEM)
        return OfflineTeachScan(
            bin_path=self._selected_path(TEACH_KIND_BIN),
            item_path=item_path,
            tray_path=self._selected_path(TEACH_KIND_TRAY),
            tool_path=self._find_tool_for_item(item_path),
        )

    def _readiness_for_mode(self, mode: str, refresh_ui: bool = True) -> TriggerResult:
        self._last_calibration_scan = self._scan_calibrations()
        self._last_runtime_scan = self._scan_runtime()
        self._last_offline_scan = self._scan_offline_teach()
        self._last_service_result = self.node.check_trigger_services_now()
        if refresh_ui:
            self._refresh_status_views()

        if not self._last_calibration_scan.ok:
            return TriggerResult(False, 'Calibration not ready: ' + self._last_calibration_scan.message)
        if mode == MODE_ONLINE:
            if not self._last_runtime_scan.ok:
                return TriggerResult(False, 'Runtime not ready: ' + self._last_runtime_scan.message)
        else:
            if not self._last_offline_scan.ok:
                return TriggerResult(False, 'Offline teach not ready: ' + self._last_offline_scan.message)
        if not self._last_service_result.success:
            return self._last_service_result
        return TriggerResult(True, 'Ready')

    def _refresh_status_views(self) -> None:
        self._last_offline_scan = self._scan_offline_teach()
        self.calibration_status_var.set(self._last_calibration_scan.message)
        self.runtime_status_var.set(self._last_runtime_scan.message)
        self.teach_status_var.set(self._last_offline_scan.message)
        self.service_status_var.set(self._last_service_result.message)

        self.start_button.configure(state=tk.DISABLED if self._running else tk.NORMAL)
        self.stop_button.configure(state=tk.NORMAL if self._running else tk.DISABLED)
        self._refresh_step_controls()
        if self._node_launcher_headless_button is not None:
            self._node_launcher_headless_button.configure(
                text='Headless: ON' if self._node_launcher_headless_enabled() else 'Headless: OFF',
                bg=RUNNING_BUTTON_BG if self._node_launcher_headless_enabled() else self.root.cget('bg'),
                activebackground=RUNNING_BUTTON_BG if self._node_launcher_headless_enabled() else self.root.cget('bg'),
            )

        self._refresh_calibration_button_texts()
        self._refresh_robot_ip_controls()
        for service_name, ready in self.node.service_readiness_map().items():
            label = self._service_labels.get(service_name)
            if label is not None:
                label.configure(fg='#1b7f3a' if ready else '#b00020')

        for key, button in self._launch_buttons.items():
            spec = self._launch_specs[key]
            process = self._launch_processes.get(key)
            running = process is not None and process.poll() is None
            missing_calibration = self._launch_missing_calibration(spec)
            launch_label = self._launch_process_labels.get(
                key,
                spec.label_for(self._node_launcher_headless_enabled()),
            )
            text = ('Stop ' if running else '') + launch_label
            if missing_calibration:
                button.configure(text=text, bg='#b00020', activebackground='#b00020')
            else:
                button.configure(
                    text=text,
                    bg=RUNNING_BUTTON_BG if running else self.root.cget('bg'),
                    activebackground=RUNNING_BUTTON_BG if running else self.root.cget('bg'),
                )
        self._refresh_cell_bridge_button()

    def _launch_missing_calibration(self, spec: LaunchSpec) -> bool:
        keys = spec.calibration_key
        if keys is None:
            return False
        if isinstance(keys, str):
            keys = (keys,)
        return any(self._last_calibration_scan.files.get(key) is None for key in keys)

    def _periodic_readiness_scan(self) -> None:
        self._last_calibration_scan = self._scan_calibrations()
        self._last_runtime_scan = self._scan_runtime()
        self._last_service_result = self.node.check_trigger_services_now()
        self._refresh_status_views()
        self.root.after(READINESS_SCAN_MS, self._periodic_readiness_scan)

    def _periodic_process_scan(self) -> None:
        finished = []
        for key, process in self._launch_processes.items():
            if process.poll() is not None:
                finished.append(key)
        for key in finished:
            process = self._launch_processes.pop(key)
            pgid = self._launch_process_groups.pop(key, None)
            label = self._launch_process_labels.pop(key, self._launch_specs[key].label)
            return_code = process.returncode
            self._log(f'{label} exited with code {return_code}')
            if self._process_group_alive(pgid):
                self._log(f'{label} launch exited but child processes are still running; cleaning up')
                self._stop_process(process, label, pgid)
        if self._cell_bridge_process is not None and self._cell_bridge_process.poll() is not None:
            process = self._cell_bridge_process
            pgid = self._cell_bridge_process_group
            return_code = self._cell_bridge_process.returncode
            self._cell_bridge_process = None
            self._cell_bridge_process_group = None
            mode_label = self._cell_bridge_mode_label(self._cell_bridge_mode)
            self._cell_bridge_mode = None
            self._log(f'Cell External Bridge {mode_label} exited with code {return_code}')
            if self._process_group_alive(pgid):
                self._log(f'Cell External Bridge {mode_label} exited but child processes are still running; cleaning up')
                self._stop_process(process, f'Cell External Bridge {mode_label}', pgid)
        self._refresh_status_views()
        self.root.after(PROCESS_SCAN_MS, self._periodic_process_scan)

    def _node_launcher_headless_enabled(self) -> bool:
        return bool(self.node_launcher_headless_var.get())

    def _node_launcher_headless_toggled(self) -> None:
        state = 'ON' if self._node_launcher_headless_enabled() else 'OFF'
        self._log(f'Node Launcher headless mode {state}')
        self._refresh_status_views()

    def _launch_clicked(self, key: str) -> None:
        process = self._launch_processes.get(key)
        if process is not None and process.poll() is None:
            self._stop_launch(key)
            return
        self._start_launch(key, self._mode, headless=self._node_launcher_headless_enabled())

    def _managed_terminal_shell_command(self, shell_command: str, pid_file: Path) -> str:
        return (
            'set -e; '
            f'echo $$ > {shlex.quote(str(pid_file))}; '
            f'{shell_command}'
        )

    def _runtime_process_pid_file(self, label: str) -> Path:
        pid_dir = workspace_path('runtime', 'robot_cell_orchestrator_process_pids')
        pid_dir.mkdir(parents=True, exist_ok=True)
        filename = f'{safe_process_label(label)}_{time.monotonic_ns()}.pid'
        return pid_dir / filename

    def _wait_for_managed_process_group(
        self,
        pid_file: Path,
        terminal_process: subprocess.Popen,
    ) -> int | None:
        deadline = time.monotonic() + PROCESS_TERMINAL_PID_WAIT_SEC
        while time.monotonic() < deadline:
            try:
                if pid_file.exists():
                    text = pid_file.read_text(encoding='utf-8').strip()
                    if text:
                        pid = int(text)
                        return os.getpgid(pid) if hasattr(os, 'getpgid') else None
            except (OSError, ValueError, ProcessLookupError):
                return None
            if terminal_process.poll() is not None:
                break
            time.sleep(0.05)
        return None

    def _start_visible_terminal_process(
        self,
        label: str,
        shell_command: str,
        env: dict[str, str],
    ) -> tuple[subprocess.Popen, int | None] | None:
        terminal_title = f'Robot Cell Orchestrator - {label}'
        pid_file = self._runtime_process_pid_file(label)
        managed_command = self._managed_terminal_shell_command(shell_command, pid_file)
        terminal_cmd = visible_terminal_command(terminal_title, managed_command)
        if terminal_cmd is None:
            self._log(
                f'No supported terminal emulator found; refusing to launch hidden process for {label}. '
                'Install gnome-terminal, xterm, xfce4-terminal, konsole, or mate-terminal.'
            )
            return None
        try:
            process = subprocess.Popen(
                terminal_cmd,
                cwd=str(workspace_root()),
                env=env,
                start_new_session=hasattr(os, 'setsid'),
            )
            pgid = self._wait_for_managed_process_group(pid_file, process)
            return process, pgid or self._capture_process_group(process)
        except Exception as exc:
            self._log(f'Failed to open terminal for {label}: {exc}')
            return None
        finally:
            try:
                pid_file.unlink()
            except FileNotFoundError:
                pass
            except Exception:
                pass

    def _start_launch(
        self,
        key: str,
        mode: str,
        *,
        headless: bool = False,
    ) -> bool:
        process = self._launch_processes.get(key)
        if process is not None and process.poll() is None:
            self._stop_launch(key)
            return True
        spec = self._launch_specs[key]
        self._last_calibration_scan = self._scan_calibrations()
        if self._launch_missing_calibration(spec):
            keys = spec.calibration_key
            required = (keys,) if isinstance(keys, str) else tuple(keys or ())
            missing = [
                calibration_kind
                for calibration_kind in required
                if self._last_calibration_scan.files.get(calibration_kind) is None
            ]
            message = f'Select calibration file(s) before launching {spec.label}: {", ".join(missing)}'
            self._log(message)
            messagebox.showwarning('Calibration Required', message, parent=self.root)
            self._refresh_status_views()
            return False
        launch_label = spec.label_for(headless)
        launch_file = spec.launch_file_for(headless)
        args = spec.args_builder(mode, headless) or []
        cmd = ['ros2', 'launch', spec.package, launch_file, *args]
        launch_cmd = ros_sourced_shell_command(cmd, source_python_venv=spec.source_python_venv)
        env = ros_child_environment()
        started = self._start_visible_terminal_process(launch_label, launch_cmd, env)
        if started is None:
            return False
        process, pgid = started
        self._launch_processes[key] = process
        self._launch_process_groups[key] = pgid
        self._launch_process_labels[key] = launch_label
        self._log(f'Launched {launch_label} in terminal: {" ".join(cmd)}')
        self._refresh_status_views()
        return True

    def _stop_launch(self, key: str) -> None:
        process = self._launch_processes.pop(key, None)
        pgid = self._launch_process_groups.pop(key, None)
        label = self._launch_process_labels.pop(key, self._launch_specs[key].label)
        if process is None:
            return
        self._stop_process(process, label, pgid)
        self._refresh_status_views()

    def _capture_process_group(self, process: subprocess.Popen) -> int | None:
        if not hasattr(os, 'getpgid'):
            return None
        try:
            return os.getpgid(process.pid)
        except ProcessLookupError:
            return None
        except Exception:
            return None

    def _process_group_alive(self, pgid: int | None) -> bool:
        if pgid is None or not hasattr(os, 'killpg'):
            return False
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        except Exception:
            return False

    def _send_process_signal(
        self,
        process: subprocess.Popen,
        pgid: int | None,
        sig: signal.Signals,
    ) -> bool:
        if pgid is not None and hasattr(os, 'killpg'):
            try:
                os.killpg(pgid, sig)
                return True
            except ProcessLookupError:
                pass
        if process.poll() is None:
            try:
                process.send_signal(sig)
                return True
            except ProcessLookupError:
                pass
        return False

    def _wait_process_or_group_stopped(
        self,
        process: subprocess.Popen,
        pgid: int | None,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            parent_running = process.poll() is None
            group_running = self._process_group_alive(pgid)
            if not parent_running and not group_running:
                return True
            time.sleep(0.05)
        return process.poll() is not None and not self._process_group_alive(pgid)

    def _stop_process(
        self,
        process: subprocess.Popen,
        label: str,
        pgid: int | None = None,
    ) -> None:
        try:
            if process.poll() is None or self._process_group_alive(pgid):
                self._log(f'Stopping {label}')
                self._send_process_signal(process, pgid, signal.SIGINT)
                if self._wait_process_or_group_stopped(process, pgid, PROCESS_STOP_TIMEOUT_SEC):
                    return
                self._send_process_signal(process, pgid, signal.SIGTERM)
                if self._wait_process_or_group_stopped(process, pgid, PROCESS_STOP_TIMEOUT_SEC):
                    return
                self._send_process_signal(process, pgid, signal.SIGKILL)
                self._log(f'Force stopped {label}')
                if not self._wait_process_or_group_stopped(process, pgid, 1.0):
                    self._log(f'{label} did not exit after SIGKILL')
        except ProcessLookupError:
            pass
        except Exception as exc:
            self._log(f'Failed to stop {label}: {exc}')

    def _cell_bridge_clicked(self) -> None:
        if self._is_cell_bridge_running():
            self._stop_cell_bridge()
            return
        self._start_cell_bridge()

    def _is_cell_bridge_running(self) -> bool:
        return self._cell_bridge_process is not None and self._cell_bridge_process.poll() is None

    def _start_cell_bridge(self) -> bool:
        mode = self._mode
        env = os.environ.copy()
        env['ROS_LOCALHOST_ONLY'] = '1'
        for key, value in key_value_config(self.station_config_file).items():
            env.setdefault(key, value)
        python_path = str(self.cell_bridge_source_dir)
        if env.get('PYTHONPATH'):
            python_path = python_path + os.pathsep + env['PYTHONPATH']
        env['PYTHONPATH'] = python_path
        env['DOBOT_PICKN_PLACE_ROOT'] = str(workspace_root())
        env.setdefault('STATION_CONFIG_PATH', str(self.station_config_file))
        if mode == MODE_OFFLINE:
            env['CELL_BRIDGE_OFFLINE_DEBUG'] = '1'
            env['CELL_BRIDGE_DATALOG_DIR'] = str(self.cell_bridge_datalog_dir)
            env['CELL_BRIDGE_WARMUP_TIMEOUT_S'] = '0'
        else:
            env['CELL_BRIDGE_OFFLINE_DEBUG'] = '0'

        cmd = [sys.executable, '-m', 'cell_external_bridge.robot_arm_controller']
        mode_label = self._cell_bridge_mode_label(mode)
        shell_command = (
            f'cd {shlex.quote(str(workspace_root()))}; '
            f'exec {shell_join(cmd)}'
        )
        started = self._start_visible_terminal_process(
            f'Cell External Bridge {mode_label}',
            shell_command,
            env,
        )
        if started is None:
            return False
        process, pgid = started
        self._cell_bridge_process = process
        self._cell_bridge_process_group = pgid
        self._cell_bridge_mode = mode
        if mode == MODE_OFFLINE:
            self._log(
                'Launched Cell External Bridge offline debug in terminal: '
                f'datalog={self.cell_bridge_datalog_dir}'
            )
        else:
            self._log('Launched Cell External Bridge online in terminal')
        self._refresh_status_views()
        return True

    def _stop_cell_bridge(self) -> None:
        process = self._cell_bridge_process
        if process is None:
            return
        mode_label = self._cell_bridge_mode_label(self._cell_bridge_mode)
        pgid = self._cell_bridge_process_group
        self._cell_bridge_process = None
        self._cell_bridge_process_group = None
        self._cell_bridge_mode = None
        self._stop_process(process, f'Cell External Bridge {mode_label}', pgid)
        self._refresh_status_views()

    def _refresh_cell_bridge_button(self) -> None:
        if self._cell_bridge_button is None:
            return
        running = self._is_cell_bridge_running()
        if running:
            self._cell_bridge_button.configure(
                text=f'Stop Cell External Bridge',
                bg=RUNNING_BUTTON_BG,
                activebackground=RUNNING_BUTTON_BG,
            )
            return
        self._cell_bridge_button.configure(
            text='Cell External Bridge',
            bg=self.root.cget('bg'),
            activebackground=self.root.cget('bg'),
        )

    @staticmethod
    def _cell_bridge_mode_label(mode: str | None) -> str:
        if mode == MODE_OFFLINE:
            return 'Offline Debug'
        if mode == MODE_ONLINE:
            return 'Online'
        return 'Unknown'

    def _start_clicked(self) -> None:
        result = self._start_cycle_from_ui(self._mode, 'manual button')
        if not result.success:
            self._log(f'Cycle refused: {result.message}')

    def _online_start_service_request(self) -> TriggerResult:
        response_queue: queue.Queue[TriggerResult] = queue.Queue(maxsize=1)
        self._queue.put(('online_start_request', response_queue))
        try:
            return response_queue.get(timeout=15.0)
        except queue.Empty:
            return TriggerResult(False, 'Timed out waiting for Robot Cell Orchestrator GUI to process online start')

    def _online_load_program_service_request(self, load: OnlineProgramLoad) -> OnlineProgramLoadResult:
        response_queue: queue.Queue[OnlineProgramLoadResult] = queue.Queue(maxsize=1)
        self._queue.put(('online_load_program_request', (load, response_queue)))
        try:
            return response_queue.get(timeout=15.0)
        except queue.Empty:
            return OnlineProgramLoadResult(False, 'Timed out waiting for Robot Cell Orchestrator GUI to load online program')

    def _online_validate_service_request(self) -> TriggerResult:
        response_queue: queue.Queue[TriggerResult] = queue.Queue(maxsize=1)
        self._queue.put(('online_validate_request', response_queue))
        try:
            return response_queue.get(timeout=15.0)
        except queue.Empty:
            return TriggerResult(False, 'Timed out waiting for Robot Cell Orchestrator GUI to validate online program')

    def _online_place_service_request(self) -> TriggerResult:
        response_queue: queue.Queue[TriggerResult] = queue.Queue(maxsize=1)
        self._queue.put(('online_place_request', response_queue))
        try:
            return response_queue.get(timeout=15.0)
        except queue.Empty:
            return TriggerResult(False, 'Timed out waiting for Robot Cell Orchestrator GUI to process online place')

    def _set_online_phase(self, phase: str) -> None:
        with self._online_command_condition:
            self._online_phase = phase
            self._online_command_condition.notify_all()

    def _reset_online_commands(self, phase: str = ONLINE_PHASE_STOPPED) -> None:
        with self._online_command_condition:
            self._online_pick_requested = False
            self._online_place_requested = False
            self._online_phase = phase
            self._online_command_condition.notify_all()

    def _request_online_pick(self) -> TriggerResult:
        with self._online_command_condition:
            if self._mode != MODE_ONLINE:
                return TriggerResult(False, 'Robot Cell Orchestrator is offline/maintenance override')
            if not self._running:
                return TriggerResult(False, 'Robot Cell Orchestrator online worker is not running')
            if self._online_phase not in (ONLINE_PHASE_STARTING, ONLINE_PHASE_WAITING_FOR_PICK):
                return TriggerResult(False, f'Robot Cell Orchestrator is {self._online_phase}; waiting for cmd.place or current motion')
            self._online_pick_requested = True
            self._online_command_condition.notify_all()
        return TriggerResult(True, 'Online pick accepted')

    def _request_online_place(self) -> TriggerResult:
        with self._online_command_condition:
            if self._mode != MODE_ONLINE:
                return TriggerResult(False, 'Robot Cell Orchestrator is offline/maintenance override')
            if not self._running:
                return TriggerResult(False, 'Robot Cell Orchestrator online worker is not running')
            if self._online_phase == ONLINE_PHASE_PLACING:
                return TriggerResult(True, 'Online place already in progress')
            if self._online_phase != ONLINE_PHASE_WAITING_FOR_PLACE:
                return TriggerResult(False, f'Robot Cell Orchestrator is {self._online_phase}; no picked item is waiting for place')
            self._online_place_requested = True
            self._online_command_condition.notify_all()
        return TriggerResult(True, 'Online place accepted')

    def _wait_for_online_pick_request(self) -> bool:
        self._set_online_phase(ONLINE_PHASE_WAITING_FOR_PICK)
        self._set_robot_status(ROBOT_STATUS_STOP)
        self._set_status('Online: waiting for cmd.pick at conveyor side')
        self.node.publish_phase_event('waiting_for_pick', message='Waiting for cmd.pick at conveyor side')
        with self._online_command_condition:
            while rclpy.ok() and not self._stop_event.is_set() and not self._online_pick_requested:
                self._online_command_condition.wait(timeout=0.1)
            if self._stop_event.is_set() or not rclpy.ok():
                return False
            self._online_pick_requested = False
            self._online_place_requested = False
        return True

    def _wait_for_online_place_request(self, cycle_index: int) -> bool:
        with self._online_command_condition:
            while rclpy.ok() and not self._stop_event.is_set() and not self._online_place_requested:
                self._online_command_condition.wait(timeout=0.1)
            if self._stop_event.is_set() or not rclpy.ok():
                return False
            self._online_place_requested = False
        return True

    def _start_cycle_from_ui(self, mode: str, source: str) -> TriggerResult:
        if self._running:
            if mode == MODE_ONLINE and self._mode == MODE_ONLINE and source == self.node.online_start_service:
                return self._request_online_pick()
            return TriggerResult(False, 'Robot Cell Orchestrator is already running')
        readiness = self._readiness_for_mode(mode)
        if not readiness.success:
            self.status_var.set('Start blocked')
            return readiness

        if mode != self._mode:
            self._set_mode(mode)
        config = self._read_runtime_settings()
        self._flush_robot_cell_orchestrator_runtime_settings()
        if mode == MODE_ONLINE:
            config = RobotCellOrchestratorRuntimeSettings(
                loop_enabled=True,
                auto_repick_enabled=config.auto_repick_enabled,
                step_mode_enabled=False,
                tray_seek_stability_sec=config.tray_seek_stability_sec,
                tray_intercept_x_offset_mm=config.tray_intercept_x_offset_mm,
                tray_intercept_y_offset_mm=config.tray_intercept_y_offset_mm,
                tray_ee_angle_deg=config.tray_ee_angle_deg,
                eye_on_hand_calibration_file=config.eye_on_hand_calibration_file,
                eye_to_hand_calibration_file=config.eye_to_hand_calibration_file,
                platform_calibration_file=config.platform_calibration_file,
                window_geometry=config.window_geometry,
            )
            self._reset_online_commands(ONLINE_PHASE_STARTING)
        self._reset_offline_step_state()
        self._stop_event.clear()
        self._running = True
        self._set_running_controls(True)
        self._worker_thread = threading.Thread(target=self._run_worker, args=(config,), daemon=True)
        self._worker_thread.start()
        if mode == MODE_ONLINE:
            with self._online_command_condition:
                self._online_pick_requested = True
                self._online_command_condition.notify_all()
        self._log(f'Robot Cell Orchestrator started from {source} in {mode} mode')
        return TriggerResult(True, f'Robot Cell Orchestrator started in {mode} mode')

    def _stop_clicked(self) -> None:
        self._stop_event.set()
        with self._online_command_condition:
            self._online_command_condition.notify_all()
        with self._offline_step_condition:
            self._offline_step_condition.notify_all()
        self._set_robot_status(ROBOT_STATUS_STOP)
        self._set_status('Stopping after current monitor step...')

    def _step_clicked(self) -> None:
        with self._offline_step_condition:
            if not self._offline_step_waiting:
                return
            self._offline_step_requested = True
            self._offline_step_waiting = False
            self._offline_step_condition.notify_all()
        self._refresh_step_controls()

    @staticmethod
    def _read_clamped_var(var: tk.DoubleVar, minimum: float, maximum: float) -> float:
        value = max(float(minimum), min(float(maximum), float(var.get())))
        var.set(value)
        return value

    def _read_runtime_settings(self) -> RobotCellOrchestratorRuntimeSettings:
        return RobotCellOrchestratorRuntimeSettings(
            loop_enabled=bool(self.loop_var.get()),
            auto_repick_enabled=bool(self.auto_repick_var.get()),
            step_mode_enabled=bool(self.step_mode_var.get()),
            tray_seek_stability_sec=self._read_timing_slider(self.tray_seek_stability_sec_var),
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
            eye_on_hand_calibration_file=self.eye_on_hand_calibration_var.get().strip(),
            eye_to_hand_calibration_file=self.eye_to_hand_calibration_var.get().strip(),
            platform_calibration_file=self.platform_calibration_var.get().strip(),
            window_geometry=self._current_window_geometry(),
        )

    def _run_worker(self, config: RobotCellOrchestratorRuntimeSettings) -> None:
        finished_status: str | None = None
        try:
            if self._mode == MODE_ONLINE:
                self._run_online_worker(config)
            else:
                completed = self._run_offline_worker(config)
                if completed and config.step_mode_enabled and not self._stop_event.is_set():
                    finished_status = 'Offline step mode: ready for next pick'
            if self._mode == MODE_ONLINE:
                self._reset_online_commands(ONLINE_PHASE_STOPPED)
                self.node.publish_phase_event('online_loop_stopped', message='Online loop stopped')
            self._queue.put(('finished', finished_status))
        except Exception as exc:
            self._log(f'Cycle worker crashed: {exc}')
            if self._mode == MODE_ONLINE:
                self._reset_online_commands(ONLINE_PHASE_STOPPED)
                self.node.publish_phase_event('online_loop_stopped', message=f'Online loop crashed: {exc}')
            self._queue.put(('finished', None))

    def _run_offline_worker(self, config: RobotCellOrchestratorRuntimeSettings) -> bool:
        completed = False
        while rclpy.ok() and not self._stop_event.is_set():
            self._cycle_count += 1
            cycle_index = self._cycle_count
            self._log(f'=== Cycle {cycle_index} start ===')
            self._set_status(f'Cycle {cycle_index}: checking readiness')
            readiness = self._readiness_for_mode(self._mode, refresh_ui=False)
            self._log(f'[{cycle_index}] Readiness: {"OK" if readiness.success else "FAIL"} - {readiness.message}')
            if not readiness.success:
                self._log(f'=== Cycle {cycle_index} stopped/failed ===')
                break
            success = self._run_one_cycle(config, cycle_index)
            if not success:
                self._log(f'=== Cycle {cycle_index} stopped/failed ===')
                break
            self._log(f'=== Cycle {cycle_index} done ===')
            self._set_robot_status(ROBOT_STATUS_STOP)
            completed = True
            if config.step_mode_enabled:
                self._set_status('Offline step mode: ready for next pick')
                break
            if not config.loop_enabled:
                break
        return completed

    def _run_online_worker(self, config: RobotCellOrchestratorRuntimeSettings) -> None:
        self.node.publish_phase_event('online_loop_started', message='Online loop started')
        while rclpy.ok() and not self._stop_event.is_set():
            if not self._wait_for_online_pick_request():
                break
            self._cycle_count += 1
            cycle_index = self._cycle_count
            self._log(f'=== Online cycle {cycle_index} pick side start ===')
            self._set_online_phase(ONLINE_PHASE_PICKING)
            self._set_status(f'Online cycle {cycle_index}: checking readiness')
            readiness = self._readiness_for_mode(MODE_ONLINE, refresh_ui=False)
            self._log(f'[{cycle_index}] Readiness: {"OK" if readiness.success else "FAIL"} - {readiness.message}')
            if not readiness.success:
                self._log(f'=== Online cycle {cycle_index} stopped/failed ===')
                break
            if not self._run_pick_side(config, cycle_index):
                self._log(f'=== Online cycle {cycle_index} pick side failed ===')
                break
            self._set_online_phase(ONLINE_PHASE_WAITING_FOR_PLACE)
            self._set_robot_status(ROBOT_STATUS_PAUSE)
            self._set_status('Online: waiting for cmd.place; robot ready at conveyor side')
            self.node.publish_phase_event(
                'moving_to_tray',
                cycle_index=cycle_index,
                message='Pick side complete; robot ready at conveyor side and waiting for cmd.place',
            )
            self.node.publish_phase_event(
                'waiting_for_place',
                cycle_index=cycle_index,
                message='Pick side complete; waiting for cmd.place at conveyor side',
            )
            self._log(f'=== Online cycle {cycle_index} waiting for cmd.place ===')
            if not self._wait_for_online_place_request(cycle_index):
                break
            self._set_online_phase(ONLINE_PHASE_PLACING)
            self._log(f'=== Online cycle {cycle_index} place side start ===')
            if not self._run_place_side(config, cycle_index):
                self._log(f'=== Online cycle {cycle_index} place side failed ===')
                break
            self._set_online_phase(ONLINE_PHASE_WAITING_FOR_PICK)
            self._set_robot_status(ROBOT_STATUS_STOP)
            self._set_status('Online: ready for next cmd.pick at conveyor side')
            self.node.publish_phase_event(
                'moving_to_bin',
                cycle_index=cycle_index,
                message='Place side complete; ready for next cmd.pick',
            )
            self._log(f'=== Online cycle {cycle_index} place side done ===')
            self._set_robot_status(ROBOT_STATUS_STOP)

    def _run_one_cycle(self, config: RobotCellOrchestratorRuntimeSettings, cycle_index: int) -> bool:
        if not self._run_pick_side(config, cycle_index):
            return False
        if config.step_mode_enabled and self._mode == MODE_OFFLINE:
            self._set_robot_status(ROBOT_STATUS_PAUSE)
            if not self._wait_for_offline_step(cycle_index):
                return False
        self._set_robot_status(ROBOT_STATUS_PLACING)
        return self._run_place_side(config, cycle_index)

    def _reset_offline_step_state(self) -> None:
        with self._offline_step_condition:
            self._offline_step_requested = False
            self._offline_step_waiting = False
            self._offline_step_button_label = 'Next Step'
        self._queue_call('step_state', None)

    def _set_offline_step_waiting(self, waiting: bool, button_label: str = 'Next Step') -> None:
        with self._offline_step_condition:
            self._offline_step_waiting = bool(waiting)
            self._offline_step_button_label = button_label
            if waiting:
                self._offline_step_requested = False
            self._offline_step_condition.notify_all()
        self._queue_call('step_state', None)

    def _wait_for_offline_step(self, cycle_index: int) -> bool:
        self._set_offline_step_waiting(True, 'Run Place')
        self._set_status(f'Cycle {cycle_index}: pick complete; waiting for Step')
        self._log(f'[{cycle_index}] Step Mode: pick complete; click Run Place to continue.')
        try:
            with self._offline_step_condition:
                while rclpy.ok() and not self._stop_event.is_set() and not self._offline_step_requested:
                    self._offline_step_condition.wait(timeout=0.1)
                if self._stop_event.is_set() or not rclpy.ok():
                    return False
                self._offline_step_requested = False
            self._log(f'[{cycle_index}] Step Mode: Run Place accepted.')
            return True
        finally:
            self._set_offline_step_waiting(False)

    def _apply_auto_repick_setting(self, cycle_index: int, enabled: bool) -> bool:
        state = 'ON' if enabled else 'OFF'
        self._set_status(f'Cycle {cycle_index}: setting Auto Repick {state}')
        result = self.node.set_item_auto_repick(enabled)
        self._log(f'[{cycle_index}] Auto Repick {state}: {"OK" if result.success else "FAIL"} - {result.message}')
        return result.success and not self._stop_event.is_set()

    def _run_pick_side(self, config: RobotCellOrchestratorRuntimeSettings, cycle_index: int) -> bool:
        self._set_robot_status(ROBOT_STATUS_PICKING)
        return (
            self._apply_auto_repick_setting(cycle_index, config.auto_repick_enabled)
            and self._go_to_teach_step(cycle_index, 'Go to item detect teach', 'item_go_to_teach')
            and self._seek_step(
                cycle_index,
                'Arm item pick',
                'item_arm',
                'item_arm_status',
                'Seek item detect',
                'item_seek',
                'item_seek_status',
                config,
                None,
            )
        )

    def _run_place_side(self, config: RobotCellOrchestratorRuntimeSettings, cycle_index: int) -> bool:
        self._set_robot_status(ROBOT_STATUS_PLACING)
        return (
            self._go_to_teach_step(cycle_index, 'Go to tray detect teach', 'tray_go_to_teach')
            and self._seek_step(
                cycle_index,
                'Arm tray intercept',
                'tray_arm',
                'tray_arm_status',
                'Seek tray detect',
                'tray_seek',
                'tray_seek_status',
                config,
                config.tray_seek_stability_sec,
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
        if self._mode == MODE_ONLINE and not result.success and 'timed out' in result.message.lower():
            self.node.publish_phase_event(
                'timeout',
                cycle_index=cycle_index,
                message=f'{label}: {result.message}',
            )
        return result.success and not self._stop_event.is_set()

    def _go_to_teach_step(self, cycle_index: int, label: str, client_key: str) -> bool:
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
        config: RobotCellOrchestratorRuntimeSettings,
        stability_sec: float | None,
    ) -> bool:
        if not self._arm_and_verify(cycle_index, arm_label, arm_client_key, arm_status_client_key, config):
            return False
        if stability_sec is not None and stability_sec > 0.0:
            self._log(
                f'[{cycle_index}] {arm_label}: armed; confirming robot stability '
                f'for {stability_sec:.1f}s before seek...'
            )

            self._set_status(f'Cycle {cycle_index}: confirming robot stability before {seek_label}')
            pre_seek_result = self.node.wait_for_robot_stable(self._stop_event, stability_sec)
            self._log(
                f'[{cycle_index}] {seek_label}: '
                f'{"STABLE" if pre_seek_result.success else "FAIL"} before seek - {pre_seek_result.message}'
            )
            if not pre_seek_result.success or self._stop_event.is_set():
                if self._mode == MODE_ONLINE and (
                    'timed out' in pre_seek_result.message.lower()
                    or 'did not become stable' in pre_seek_result.message.lower()
                ):
                    self.node.publish_phase_event(
                        'timeout',
                        cycle_index=cycle_index,
                        message=f'{seek_label}: {pre_seek_result.message}',
                    )
                return False
        else:
            self._log(f'[{cycle_index}] {arm_label}: armed; skipping robot stability wait before {seek_label}.')

        if not self._click_step(cycle_index, seek_label, seek_client_key):
            return False
        return self._wait_for_seek_on_then_off(cycle_index, seek_label, seek_status_client_key)

    def _arm_and_verify(
        self,
        cycle_index: int,
        arm_label: str,
        arm_client_key: str,
        arm_status_client_key: str,
        config: RobotCellOrchestratorRuntimeSettings,
    ) -> bool:
        if arm_client_key == 'tray_arm':
            self._set_status(f'Cycle {cycle_index}: {arm_label}')
            self._log(
                f'[{cycle_index}] {arm_label}: '
                f'x={config.tray_intercept_x_offset_mm:.0f}mm, '
                f'y={config.tray_intercept_y_offset_mm:.0f}mm, '
                f'rz={config.tray_ee_angle_deg:.0f}deg...'
            )
            result = self.node.start_tray_intercept(config, ARM_CLICK_RESPONSE_TIMEOUT_SEC)
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
        status = self.node.read_trigger_status(arm_status_client_key, timeout_sec=ARM_STATUS_RESPONSE_TIMEOUT_SEC)
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
        start_state = tk.DISABLED if running else tk.NORMAL
        self.start_button.configure(state=start_state)
        self.stop_button.configure(state=tk.NORMAL if running else tk.DISABLED)
        self.loop_check.configure(state=tk.DISABLED if running else tk.NORMAL)
        self.auto_repick_check.configure(state=tk.DISABLED if running else tk.NORMAL)
        self.step_mode_check.configure(state=tk.DISABLED if running else tk.NORMAL)
        self.tray_seek_stability_sec_scale.configure(state=tk.DISABLED if running else tk.NORMAL)
        setting_state = tk.DISABLED if running else tk.NORMAL
        self.tray_intercept_x_spinbox.configure(state=setting_state)
        self.tray_intercept_y_spinbox.configure(state=setting_state)
        self.tray_ee_angle_spinbox.configure(state=setting_state)
        self.offline_button.configure(state=setting_state)
        self.online_button.configure(state=setting_state)
        self._refresh_robot_ip_controls()
        self._refresh_step_controls()
        self._refresh_teach_button_texts()
        self._refresh_calibration_button_texts()

    def _refresh_step_controls(self) -> None:
        if not hasattr(self, 'step_button'):
            return
        with self._offline_step_condition:
            waiting = self._offline_step_waiting
            label = self._offline_step_button_label
        enabled = (
            self._running
            and self._mode == MODE_OFFLINE
            and bool(self.step_mode_var.get())
            and waiting
            and not self._stop_event.is_set()
        )
        self.step_button.configure(
            text=label if waiting else 'Next Step',
            state=tk.NORMAL if enabled else tk.DISABLED,
        )

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
                    self._reset_offline_step_state()
                    self._set_running_controls(False)
                    self._set_robot_status(ROBOT_STATUS_STOP)
                    if isinstance(payload, str) and payload:
                        self.status_var.set(payload)
                    else:
                        self.status_var.set('Idle' if not self._stop_event.is_set() else 'Stopped')
                elif action == 'step_state':
                    self._refresh_step_controls()
                elif action == 'online_start_request':
                    response_queue = payload
                    if isinstance(response_queue, queue.Queue):
                        if self._mode != MODE_ONLINE:
                            response_queue.put(
                                TriggerResult(False, 'Robot Cell Orchestrator is offline/maintenance override')
                            )
                        else:
                            response_queue.put(self._start_cycle_from_ui(MODE_ONLINE, self.node.online_start_service))
                elif action == 'online_load_program_request':
                    if isinstance(payload, tuple) and len(payload) == 2:
                        load, response_queue = payload
                        if isinstance(load, OnlineProgramLoad) and isinstance(response_queue, queue.Queue):
                            if self._mode != MODE_ONLINE:
                                response_queue.put(
                                    OnlineProgramLoadResult(False, 'Robot Cell Orchestrator is offline/maintenance override')
                                )
                            else:
                                response_queue.put(self._load_online_program(load))
                elif action == 'online_validate_request':
                    response_queue = payload
                    if isinstance(response_queue, queue.Queue):
                        if self._mode != MODE_ONLINE:
                            response_queue.put(
                                TriggerResult(False, 'Robot Cell Orchestrator is offline/maintenance override')
                            )
                        elif self._running and self._online_phase != ONLINE_PHASE_WAITING_FOR_PICK:
                            response_queue.put(
                                TriggerResult(False, f'Robot Cell Orchestrator is {self._online_phase}; runtime cannot change now')
                            )
                        else:
                            response_queue.put(self._readiness_for_mode(MODE_ONLINE))
                elif action == 'online_place_request':
                    response_queue = payload
                    if isinstance(response_queue, queue.Queue):
                        response_queue.put(self._request_online_place())
        except queue.Empty:
            pass

        self.root.after(100, self._drain_queue)

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + '\n')
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _on_close(self) -> None:
        self._flush_robot_cell_orchestrator_runtime_settings()
        self._stop_event.set()
        with self._online_command_condition:
            self._online_command_condition.notify_all()
        with self._offline_step_condition:
            self._offline_step_condition.notify_all()
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        self._close_camera_window()
        self._stop_cell_bridge()
        for key in list(self._launch_processes.keys()):
            self._stop_launch(key)
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotCellOrchestratorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    startup_result = node.check_trigger_services_now()
    gui = RobotCellOrchestratorGui(node, startup_result)
    try:
        gui.run()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
