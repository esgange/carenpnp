import json
import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import DO, MovL, Stop, TrayInterceptStart
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

try:
    import yaml
except Exception:
    yaml = None


SERVICE_ROOT_DEFAULT = '/dobot_bringup_ros2/srv'
GRIPPER_DO_SERVICE_DEFAULT = f'{SERVICE_ROOT_DEFAULT}/DO'
LINEAR_SPEED_MM_S_MIN = 50.0
LINEAR_SPEED_MM_S_MAX = 350.0
DEFAULT_ACC_PERCENT = 100
TCP_FIELDS = ('x', 'y', 'z', 'rx', 'ry', 'rz')
ITEM_POSE_TOPIC = 'bin_seek_pose'
ROBOT_GOAL_FRAME_DEFAULT = 'base_link'
ROBOT_GRIPPER_FRAME_DEFAULT = 'Link6'
CALIBRATED_CAMERA_FRAME_DEFAULT = 'calibrated_camera_link'
POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT = 'item_goal_tcp'
POST_STOP_MOVL_GOAL_NOMINAL_DEBUG_FRAME_DEFAULT = 'item_movel_goal_nominal_tcp'
POST_STOP_MOVL_GOAL_TOOL_OFFSET_DEBUG_FRAME_DEFAULT = 'item_movel_goal_tool_offset'
POST_STOP_MOVL_GOAL_TOOL_AXIS_X_TIP_FRAME_DEFAULT = 'item_movel_goal_tool_axis_x_tip'
POST_STOP_MOVL_GOAL_TOOL_AXIS_Y_TIP_FRAME_DEFAULT = 'item_movel_goal_tool_axis_y_tip'
POST_STOP_MOVL_GOAL_TOOL_AXIS_Z_TIP_FRAME_DEFAULT = 'item_movel_goal_tool_axis_z_tip'
ITEM_POSE_WATCH_TIMEOUT_SEC = 60.0
ITEM_POSE_WATCH_TIMEOUT_MIN = 1.0
ITEM_POSE_WATCH_TIMEOUT_MAX = 60.0
ITEM_POSE_MOTION_NOISE_FLOOR_MM_S = 5.0
POST_STOP_MOVL_SPEED_MIN = 150.0
POST_STOP_MOVL_SPEED_MAX = 350.0
POST_STOP_X_OFFSET_MIN = -50.0
POST_STOP_X_OFFSET_MAX = 400.0
POST_STOP_Y_OFFSET_MIN = -50.0
POST_STOP_Y_OFFSET_MAX = 300.0
POST_STOP_Z_OFFSET_MIN = 50.0
POST_STOP_Z_OFFSET_MAX = 200.0
APPROACH_Z_UP_MIN = 50.0
APPROACH_Z_UP_MAX = 200.0
APPROACH_Z_UP_DEFAULT = 200.0
FINAL_Z_UP_MIN = 50.0
FINAL_Z_UP_MAX = 300.0
FINAL_Z_UP_DEFAULT = APPROACH_Z_UP_DEFAULT
SETTLING_TIME_MIN_SEC = 0.1
SETTLING_TIME_MAX_SEC = 1.0
SETTLING_TIME_DEFAULT_SEC = 0.1
TOOL_OFFSET_TRANSLATION_MIN_MM = -500.0
TOOL_OFFSET_TRANSLATION_MAX_MM = 500.0
TOOL_OFFSET_ROTATION_MIN_DEG = -180.0
TOOL_OFFSET_ROTATION_MAX_DEG = 180.0
COMMAND_HYSTERESIS_MIN_SEC = 0.1
COMMAND_HYSTERESIS_MAX_SEC = 1.0
COMMAND_HYSTERESIS_DEFAULT_SEC = 0.1
LOCKED_MAX_SPEED_MM_S = POST_STOP_MOVL_SPEED_MAX
TCP_GOAL_REACHED_TOLERANCE_MM = 5.0
MANUAL_RELEASE_PULSE_MS = 300
GOAL_TF_LOOKUP_TIMEOUT_SEC_DEFAULT = 0.2
CAMERA_BIN_SAFE_MARGIN_MM_DEFAULT = 0.0
START_SEQUENCE_SERVICE_DEFAULT = 'item_pick/start_sequence'
TRACK_SERVICE_DEFAULT = 'item_pick/track'
TRACK_STATUS_SERVICE_DEFAULT = 'item_pick/track_status'
ITEM_SEEK_COMPLETE_SERVICE_DEFAULT = 'item_detect/seek_complete'
TOOL_OFFSET_PREVIEW_PARENT_FRAME_DEFAULT = 'Link6'
TOOL_OFFSET_PREVIEW_FRAME_DEFAULT = 'item_pick_tool_offset_preview'
TOOL_OFFSET_PREVIEW_AXIS_X_TIP_FRAME_DEFAULT = 'item_pick_tool_offset_preview_axis_x_tip'
TOOL_OFFSET_PREVIEW_AXIS_Y_TIP_FRAME_DEFAULT = 'item_pick_tool_offset_preview_axis_y_tip'
TOOL_OFFSET_PREVIEW_AXIS_Z_TIP_FRAME_DEFAULT = 'item_pick_tool_offset_preview_axis_z_tip'
RUNTIME_SETTINGS_PATH = Path.home() / '.ros' / 'item_pick_runtime_settings.json'
ITEM_PROFILE_STATE_PATH = Path('/home/erds/DOBOT_pickn_place/config/bins/item_detect_selected_profile.txt')
LEGACY_ITEM_PROFILE_STATE_PATH = Path('/home/erds/DOBOT_pickn_place/config/bins/bin_detect_selected_profile.txt')
TOOL_TEACH_FILE_SUFFIX = '_tool.yaml'
RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS = 250
GOAL_TF_DIAG_AXIS_LENGTH_MM = 60.0
GRIPPER_DO_CLOSE_INDEX = 1
GRIPPER_DO_OPEN_INDEX = 2
GRIPPER_DO_SUCTION_INDEX = 3


def _safe_tool_teach_name(raw_name: object) -> str:
    safe_chars: list[str] = []
    previous_was_separator = False
    for ch in str(raw_name).strip():
        if ch.isalnum():
            safe_chars.append(ch)
            previous_was_separator = False
        elif not previous_was_separator:
            safe_chars.append('_')
            previous_was_separator = True
    safe_name = ''.join(safe_chars).strip('_')
    return safe_name or 'item'


def _fallback_item_teach_name_from_profile_path(profile_path: Path) -> str:
    stem = profile_path.stem.strip()
    for prefix in ('item_', 'bin_'):
        if stem.startswith(prefix):
            stem = stem[len(prefix):]
            break
    if '_bin_' in stem:
        stem = stem.split('_bin_', 1)[0]
    parts = stem.split('_')
    if len(parts) > 1 and parts[-1].isdigit() and len(parts[-1]) == 8:
        stem = '_'.join(parts[:-1])
    if stem in ('item_teach_settings', 'bin_teach_settings', 'teach_settings', 'settings'):
        stem = 'item'
    return stem or 'item'


def item_teach_name_for_profile(profile_key: str | Path) -> str:
    profile_path = Path(profile_key).expanduser()
    if profile_path.exists():
        try:
            payload = read_simple_yaml_mapping(profile_path)
        except Exception:
            payload = None
        if isinstance(payload, dict):
            item_name = str(payload.get('item_name', '')).strip()
            if item_name:
                return _safe_tool_teach_name(item_name)
            legacy_item_name = str(payload.get('item_teach_name', '')).strip()
            if legacy_item_name:
                return _safe_tool_teach_name(legacy_item_name)
            legacy_bin_name = str(payload.get('bin_name', '')).strip()
            if legacy_bin_name:
                return _safe_tool_teach_name(legacy_bin_name)
    return _safe_tool_teach_name(_fallback_item_teach_name_from_profile_path(profile_path))


def tool_teach_path_for_profile(profile_key: str | Path) -> Path:
    profile_path = Path(profile_key).expanduser()
    item_teach_name = item_teach_name_for_profile(profile_path)
    return profile_path.with_name(f'{item_teach_name}{TOOL_TEACH_FILE_SUFFIX}')


def display_name_for_item_teach_profile(profile_key: str | None) -> str:
    if not profile_key:
        return 'No active item teach'
    try:
        return item_teach_name_for_profile(profile_key)
    except Exception:
        try:
            return Path(profile_key).name or str(profile_key)
        except Exception:
            return str(profile_key)


def display_name_for_tool_teach_profile(profile_key: str | None) -> str:
    if not profile_key:
        return 'No active tool teach'
    try:
        return tool_teach_path_for_profile(profile_key).name
    except Exception:
        return display_name_for_item_teach_profile(profile_key)


def _parse_simple_yaml_scalar(raw_value: str) -> object:
    text = str(raw_value).strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        text = text[1:-1]
    lowered = text.lower()
    if lowered == 'true':
        return True
    if lowered == 'false':
        return False
    try:
        if any(ch in text for ch in ('.', 'e', 'E')):
            return float(text)
        return int(text)
    except ValueError:
        return text


def read_simple_yaml_mapping(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    payload: dict[str, object] = {}
    with path.open('r', encoding='utf-8') as infile:
        for line in infile:
            stripped = line.strip()
            if not stripped or stripped.startswith('#') or ':' not in stripped:
                continue
            key, raw_value = stripped.split(':', 1)
            key = key.strip()
            if not key:
                continue
            payload[key] = _parse_simple_yaml_scalar(raw_value)
    return payload


def _yaml_scalar_text(value: object) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return format(value, '.10g')
    return str(value)


def write_simple_yaml_mapping(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as outfile:
        outfile.write('# item_pick tool teach sidecar\n')
        for key, value in payload.items():
            outfile.write(f'{key}: {_yaml_scalar_text(value)}\n')


@dataclass
class ItemPickSnapshot:
    tcp_values: dict[str, float] = field(default_factory=lambda: {name: 0.0 for name in TCP_FIELDS})
    tcp_stamp: float | None = None
    busy: bool = False
    action_text: str = 'Ready'
    item_pose_seq: int = 0
    has_last_item: bool = False


@dataclass(frozen=True)
class ItemPoseTarget:
    position_mm: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    frame_id: str
    stamp_sec: float


@dataclass(frozen=True)
class PredictedGoal:
    x_mm: float
    y_mm: float
    z_mm: float
    rx_deg: float
    ry_deg: float
    rz_deg: float
    source_frame_id: str
    lead_time_sec: float
    item_age_sec: float
    item_speed_base_mmps: float
    nominal_x_mm: float = 0.0
    nominal_y_mm: float = 0.0
    nominal_z_mm: float = 0.0
    nominal_rx_deg: float = 0.0
    nominal_ry_deg: float = 0.0
    nominal_rz_deg: float = 0.0
    orientation_choice: str = 'preferred'
    camera_safety_message: str = ''


@dataclass(frozen=True)
class BinCameraSafetyArea:
    profile_path: Path
    bin_teach_path: Path
    bin_frame_id: str
    base_to_bin_translation_m: tuple[float, float, float]
    base_to_bin_rotation_xyzw: tuple[float, float, float, float]
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    margin_m: float


class ItemPickNode(Node):
    def __init__(self) -> None:
        super().__init__('item_pick')
        self._lock = threading.Lock()
        self._snapshot = ItemPickSnapshot()
        self._item_pose_seq = 0
        self._item_pose_watch_armed = False
        self._item_pose_watch_seq_floor = 0
        self._item_pose_watch_deadline_monotonic = 0.0
        self._item_pose_watch_stop_dispatched = False
        self._item_pose_watch_generation = 0
        self._item_pose_watch_tf_only_mode = True
        self._item_pose_watch_timeout_sec = ITEM_POSE_WATCH_TIMEOUT_SEC
        self._cancel_requested = False
        self._manual_stop_inflight = False
        self._manual_release_inflight = False
        self._goal_tf_diagnose_inflight = False
        self._post_stop_movel_speed_mm_s = LOCKED_MAX_SPEED_MM_S
        self._post_stop_x_offset_mm = 0.0
        self._post_stop_y_offset_mm = 0.0
        self._post_stop_z_offset_mm = 100.0
        self._approach_z_up_mm = APPROACH_Z_UP_DEFAULT
        self._final_z_up_mm = FINAL_Z_UP_DEFAULT
        self._tool_offset_x_mm = 0.0
        self._tool_offset_y_mm = 0.0
        self._tool_offset_z_mm = 0.0
        self._tool_offset_rx_deg = 0.0
        self._tool_offset_ry_deg = 0.0
        self._tool_offset_rz_deg = 0.0
        self._settling_time_sec = max(
            SETTLING_TIME_MIN_SEC,
            min(
                SETTLING_TIME_MAX_SEC,
                float(self.declare_parameter('settling_time_sec', SETTLING_TIME_DEFAULT_SEC).value),
            ),
        )
        self._command_hysteresis_sec = max(
            COMMAND_HYSTERESIS_MIN_SEC,
            min(
                COMMAND_HYSTERESIS_MAX_SEC,
                float(
                    self.declare_parameter(
                        'command_hysteresis_sec',
                        COMMAND_HYSTERESIS_DEFAULT_SEC,
                    ).value
                ),
            ),
        )
        self._publish_goal_debug_tf = bool(
            self.declare_parameter('publish_goal_debug_tf', True).value
        )
        self._robot_goal_frame_id = str(
            self.declare_parameter('robot_goal_frame_id', ROBOT_GOAL_FRAME_DEFAULT).value
        ).strip() or ROBOT_GOAL_FRAME_DEFAULT
        self._robot_gripper_frame_id = str(
            self.declare_parameter('robot_gripper_frame_id', ROBOT_GRIPPER_FRAME_DEFAULT).value
        ).strip() or ROBOT_GRIPPER_FRAME_DEFAULT
        self._camera_safety_frame_id = str(
            self.declare_parameter('camera_safety_frame_id', CALIBRATED_CAMERA_FRAME_DEFAULT).value
        ).strip() or CALIBRATED_CAMERA_FRAME_DEFAULT
        self._prefer_camera_inside_bin = bool(
            self.declare_parameter('prefer_camera_inside_bin', True).value
        )
        self._camera_bin_safe_margin_mm = max(
            0.0,
            float(self.declare_parameter(
                'camera_bin_safe_margin_mm',
                CAMERA_BIN_SAFE_MARGIN_MM_DEFAULT,
            ).value),
        )
        self._post_stop_movel_goal_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_debug_frame_id',
                POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT
        self._post_stop_movel_goal_nominal_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_nominal_debug_frame_id',
                POST_STOP_MOVL_GOAL_NOMINAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_NOMINAL_DEBUG_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_offset_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_offset_debug_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_OFFSET_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_OFFSET_DEBUG_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_axis_x_tip_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_axis_x_tip_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_AXIS_X_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_AXIS_X_TIP_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_axis_y_tip_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_axis_y_tip_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_AXIS_Y_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_AXIS_Y_TIP_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_axis_z_tip_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_axis_z_tip_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_AXIS_Z_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_AXIS_Z_TIP_FRAME_DEFAULT
        self._tool_offset_preview_parent_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_parent_frame_id',
                TOOL_OFFSET_PREVIEW_PARENT_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_PARENT_FRAME_DEFAULT
        self._tool_offset_preview_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_frame_id',
                TOOL_OFFSET_PREVIEW_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_FRAME_DEFAULT
        self._tool_offset_preview_axis_x_tip_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_axis_x_tip_frame_id',
                TOOL_OFFSET_PREVIEW_AXIS_X_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_AXIS_X_TIP_FRAME_DEFAULT
        self._tool_offset_preview_axis_y_tip_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_axis_y_tip_frame_id',
                TOOL_OFFSET_PREVIEW_AXIS_Y_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_AXIS_Y_TIP_FRAME_DEFAULT
        self._tool_offset_preview_axis_z_tip_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_axis_z_tip_frame_id',
                TOOL_OFFSET_PREVIEW_AXIS_Z_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_AXIS_Z_TIP_FRAME_DEFAULT
        self._tool_offset_x_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('tool_offset_x_mm', 0.0).value),
        )
        self._tool_offset_y_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('tool_offset_y_mm', 0.0).value),
        )
        self._tool_offset_z_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('tool_offset_z_mm', 0.0).value),
        )
        self._tool_offset_rx_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('tool_offset_rx_deg', 0.0).value),
        )
        self._tool_offset_ry_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('tool_offset_ry_deg', 0.0).value),
        )
        self._tool_offset_rz_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('tool_offset_rz_deg', 0.0).value),
        )
        self._goal_tf_lookup_timeout_sec = max(
            0.01,
            float(self.declare_parameter(
                'goal_tf_lookup_timeout_sec',
                GOAL_TF_LOOKUP_TIMEOUT_SEC_DEFAULT,
            ).value),
        )
        self._start_sequence_service_name = str(
            self.declare_parameter(
                'start_sequence_service',
                START_SEQUENCE_SERVICE_DEFAULT,
            ).value
        ).strip() or START_SEQUENCE_SERVICE_DEFAULT
        self._track_service_name = str(
            self.declare_parameter(
                'track_service',
                TRACK_SERVICE_DEFAULT,
            ).value
        ).strip() or TRACK_SERVICE_DEFAULT
        self._track_status_service_name = str(
            self.declare_parameter(
                'track_status_service',
                TRACK_STATUS_SERVICE_DEFAULT,
            ).value
        ).strip() or TRACK_STATUS_SERVICE_DEFAULT
        self._item_seek_complete_service_name = str(
            self.declare_parameter(
                'item_seek_complete_service',
                ITEM_SEEK_COMPLETE_SERVICE_DEFAULT,
            ).value
        ).strip() or ITEM_SEEK_COMPLETE_SERVICE_DEFAULT
        self._gripper_do_service_name = str(
            self.declare_parameter(
                'gripper_do_service',
                GRIPPER_DO_SERVICE_DEFAULT,
            ).value
        ).strip() or GRIPPER_DO_SERVICE_DEFAULT

        self._mov_l_client = self.create_client(MovL, f'{SERVICE_ROOT_DEFAULT}/MovL')
        self._stop_client = self.create_client(Stop, f'{SERVICE_ROOT_DEFAULT}/Stop')
        self._do_client = self.create_client(DO, self._gripper_do_service_name)
        self._item_seek_complete_client = self.create_client(
            Trigger,
            self._item_seek_complete_service_name,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._goal_static_tf_by_child: dict[str, TransformStamped] = {}
        self._item_profile_state_path = ITEM_PROFILE_STATE_PATH
        self._legacy_item_profile_state_path = LEGACY_ITEM_PROFILE_STATE_PATH
        self._active_item_profile_key: str | None = None
        self._profile_state_mtime_ns: int | None = None
        self._active_profile_saved_tool_offsets: dict[str, float] | None = None
        self._track_trigger_handler = None
        self._last_item_target: ItemPoseTarget | None = None
        self.create_subscription(ToolVectorActual, 'dobot_msgs_v4/msg/ToolVectorActual', self._tcp_callback, 10)
        self.create_subscription(PoseStamped, ITEM_POSE_TOPIC, self._item_pose_callback, 10)
        self._start_sequence_service = self.create_service(
            TrayInterceptStart,
            self._start_sequence_service_name,
            self._start_sequence_service_callback,
        )
        self._track_service = self.create_service(
            Trigger,
            self._track_service_name,
            self._track_service_callback,
        )
        self._track_status_service = self.create_service(
            Trigger,
            self._track_status_service_name,
            self._track_status_service_callback,
        )
        self.get_logger().info(
            'Item pick mode configured: MovL approach/descent/retract/final with explicit DO and two settling waits.'
        )
        self.get_logger().info(f'Start item pick sequence service: {self._start_sequence_service_name}')
        self.get_logger().info(f'Track virtual-click service: {self._track_service_name}')
        self.get_logger().info(f'Track armed status service: {self._track_status_service_name}')
        self.get_logger().info(f'Item detect seek-complete service: {self._item_seek_complete_service_name}')
        self.get_logger().info(f'Gripper DO service: {self._gripper_do_service_name}')
        self.get_logger().info(
            'Startup defaults: '
            f'wait={self._item_pose_watch_timeout_sec:.0f}s, '
            f'speed={self._post_stop_movel_speed_mm_s:.0f} mm/s, '
            f'offsets(x={self._post_stop_x_offset_mm:.0f},'
            f'y={self._post_stop_y_offset_mm:.0f},'
            f'z={self._post_stop_z_offset_mm:.0f}) mm, '
            f'tool_offset(x={self._tool_offset_x_mm:.1f},'
            f'y={self._tool_offset_y_mm:.1f},'
            f'z={self._tool_offset_z_mm:.1f},'
            f'rx={self._tool_offset_rx_deg:.1f},'
            f'ry={self._tool_offset_ry_deg:.1f},'
            f'rz={self._tool_offset_rz_deg:.1f}), '
            f'approach_z={self._approach_z_up_mm:.0f} mm, '
            f'final_z_up={self._final_z_up_mm:.0f} mm, '
            f'settling={self._settling_time_sec:.1f}s'
        )
        self._sync_profile_tool_offsets_from_state(force=True)

    def _reset_runtime_state_locked(self, reason: str) -> None:
        self._item_pose_watch_generation += 1
        self._item_pose_watch_armed = False
        self._item_pose_watch_stop_dispatched = False
        self._item_pose_watch_seq_floor = self._item_pose_seq
        self._item_pose_watch_deadline_monotonic = 0.0
        self._cancel_requested = False
        self._snapshot.busy = False
        self._snapshot.action_text = reason

    def _reset_runtime_state(self, reason: str) -> None:
        with self._lock:
            self._reset_runtime_state_locked(reason)

    def snapshot(self) -> ItemPickSnapshot:
        with self._lock:
            return ItemPickSnapshot(
                tcp_values=dict(self._snapshot.tcp_values),
                tcp_stamp=self._snapshot.tcp_stamp,
                busy=self._snapshot.busy,
                action_text=self._snapshot.action_text,
                item_pose_seq=self._item_pose_seq,
                has_last_item=self._last_item_target is not None,
            )

    def _tcp_callback(self, msg: ToolVectorActual) -> None:
        with self._lock:
            self._snapshot.tcp_values['x'] = float(msg.x)
            self._snapshot.tcp_values['y'] = float(msg.y)
            self._snapshot.tcp_values['z'] = float(msg.z)
            self._snapshot.tcp_values['rx'] = float(msg.rx)
            self._snapshot.tcp_values['ry'] = float(msg.ry)
            self._snapshot.tcp_values['rz'] = float(msg.rz)
            self._snapshot.tcp_stamp = time.time()

    @staticmethod
    def _rpy_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
        roll = math.radians(float(roll_deg))
        pitch = math.radians(float(pitch_deg))
        yaw = math.radians(float(yaw_deg))

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    @staticmethod
    def _quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        return (-q[0], -q[1], -q[2], q[3])

    @staticmethod
    def _quat_multiply(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        norm = math.sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]))
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)

    @classmethod
    def _quat_angular_distance_deg(
        cls,
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> float:
        l = cls._quat_normalize(left)
        r = cls._quat_normalize(right)
        dot = (l[0] * r[0]) + (l[1] * r[1]) + (l[2] * r[2]) + (l[3] * r[3])
        # q and -q represent the same rotation; use absolute dot for shortest distance.
        dot = max(-1.0, min(1.0, abs(dot)))
        return math.degrees(2.0 * math.acos(dot))

    @classmethod
    def _rotate_vector_by_quaternion(
        cls,
        vector_xyz: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        q = cls._quat_normalize(quaternion_xyzw)
        pure = (float(vector_xyz[0]), float(vector_xyz[1]), float(vector_xyz[2]), 0.0)
        rotated = cls._quat_multiply(cls._quat_multiply(q, pure), cls._quat_conjugate(q))
        return (rotated[0], rotated[1], rotated[2])

    @classmethod
    def _yaw_deg_from_quaternion_xy_axis(
        cls,
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> float:
        x_axis = cls._rotate_vector_by_quaternion((1.0, 0.0, 0.0), quaternion_xyzw)
        return math.degrees(math.atan2(x_axis[1], x_axis[0]))

    @classmethod
    def _sterilize_quaternion_to_world_normal(
        cls,
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        # Keep yaw but remove roll/pitch so tool Z becomes world-normal (up/down).
        yaw_deg = cls._yaw_deg_from_quaternion_xy_axis(quaternion_xyzw)
        q_up = cls._quat_normalize(cls._rpy_deg_to_quaternion(0.0, 0.0, yaw_deg))
        q_down = cls._quat_normalize(cls._rpy_deg_to_quaternion(180.0, 0.0, yaw_deg))
        q_input = cls._quat_normalize(quaternion_xyzw)
        return min(
            (q_up, q_down),
            key=lambda q_candidate: cls._quat_angular_distance_deg(q_input, q_candidate),
        )

    @staticmethod
    def _quaternion_to_rpy_deg(
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = quaternion_xyzw
        sinr_cosp = 2.0 * ((qw * qx) + (qy * qz))
        cosr_cosp = 1.0 - (2.0 * ((qx * qx) + (qy * qy)))
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * ((qw * qy) - (qz * qx))
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * ((qw * qz) + (qx * qy))
        cosy_cosp = 1.0 - (2.0 * ((qy * qy) + (qz * qz)))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def _choose_min_rotation_candidate_index(
        self,
        candidates: tuple[tuple[float, float, float, float], ...],
    ) -> int:
        if len(candidates) <= 1:
            return 0

        snapshot = self.snapshot()
        if snapshot.tcp_stamp is None:
            return 0

        q_current_tcp = self._quat_normalize(
            self._rpy_deg_to_quaternion(
                float(snapshot.tcp_values.get('rx', 0.0)),
                float(snapshot.tcp_values.get('ry', 0.0)),
                float(snapshot.tcp_values.get('rz', 0.0)),
            )
        )
        return min(
            range(len(candidates)),
            key=lambda idx: self._quat_angular_distance_deg(q_current_tcp, candidates[idx]),
        )

    @classmethod
    def _compose_transform_m(
        cls,
        parent_translation_m: tuple[float, float, float],
        parent_rotation_xyzw: tuple[float, float, float, float],
        child_translation_m: tuple[float, float, float],
        child_rotation_xyzw: tuple[float, float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        rotated_child_translation = cls._rotate_vector_by_quaternion(
            child_translation_m,
            parent_rotation_xyzw,
        )
        composed_translation = (
            parent_translation_m[0] + rotated_child_translation[0],
            parent_translation_m[1] + rotated_child_translation[1],
            parent_translation_m[2] + rotated_child_translation[2],
        )
        composed_rotation = cls._quat_normalize(
            cls._quat_multiply(parent_rotation_xyzw, child_rotation_xyzw)
        )
        return composed_translation, composed_rotation

    @classmethod
    def _transform_point_inverse_m(
        cls,
        point_m: tuple[float, float, float],
        parent_to_child_translation_m: tuple[float, float, float],
        parent_to_child_rotation_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        relative = (
            point_m[0] - parent_to_child_translation_m[0],
            point_m[1] - parent_to_child_translation_m[1],
            point_m[2] - parent_to_child_translation_m[2],
        )
        return cls._rotate_vector_by_quaternion(
            relative,
            cls._quat_conjugate(parent_to_child_rotation_xyzw),
        )

    @staticmethod
    def _builtin_time_to_sec(stamp) -> float:
        sec = float(getattr(stamp, 'sec', 0))
        nanosec = float(getattr(stamp, 'nanosec', 0))
        return sec + (nanosec * 1e-9)

    @staticmethod
    def _vector_norm3(vector_xyz: tuple[float, float, float]) -> float:
        return math.sqrt((vector_xyz[0] * vector_xyz[0]) + (vector_xyz[1] * vector_xyz[1]) + (vector_xyz[2] * vector_xyz[2]))

    @staticmethod
    def _vector_dot3(left_xyz: tuple[float, float, float], right_xyz: tuple[float, float, float]) -> float:
        return (left_xyz[0] * right_xyz[0]) + (left_xyz[1] * right_xyz[1]) + (left_xyz[2] * right_xyz[2])

    @staticmethod
    def _normalize_vector3(vector_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        norm = ItemPickNode._vector_norm3(vector_xyz)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0)
        return (vector_xyz[0] / norm, vector_xyz[1] / norm, vector_xyz[2] / norm)

    @staticmethod
    def _clamp_tool_offset_translation_mm(value_mm: float) -> float:
        return max(
            TOOL_OFFSET_TRANSLATION_MIN_MM,
            min(TOOL_OFFSET_TRANSLATION_MAX_MM, float(value_mm)),
        )

    @staticmethod
    def _clamp_tool_offset_rotation_deg(value_deg: float) -> float:
        return max(
            TOOL_OFFSET_ROTATION_MIN_DEG,
            min(TOOL_OFFSET_ROTATION_MAX_DEG, float(value_deg)),
        )

    @staticmethod
    def _normalize_profile_key(path_text: object) -> str:
        raw = str(path_text).strip()
        if not raw:
            return ''
        try:
            return str(Path(raw).expanduser())
        except Exception:
            return raw

    @staticmethod
    def _profile_display_name(profile_key: str | None) -> str:
        return display_name_for_item_teach_profile(profile_key)

    def _read_active_item_profile_key(self) -> str | None:
        profile_state_path = self._item_profile_state_path
        if not profile_state_path.exists() and self._legacy_item_profile_state_path.exists():
            profile_state_path = self._legacy_item_profile_state_path
        if not profile_state_path.exists():
            return None
        try:
            active_profile_text = profile_state_path.read_text(encoding='utf-8').strip()
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to read item profile state file "{profile_state_path}": {exc}'
            )
            return None
        normalized = self._normalize_profile_key(active_profile_text)
        return normalized or None

    @staticmethod
    def _safe_yaml_load(path: Path) -> dict[str, object] | None:
        if yaml is None:
            return None
        try:
            with path.open('r', encoding='utf-8') as infile:
                payload = yaml.safe_load(infile)
        except Exception:
            return None
        return payload if isinstance(payload, dict) else None

    @staticmethod
    def _yaml_map(payload: object, key: str) -> dict[str, object]:
        if not isinstance(payload, dict):
            return {}
        value = payload.get(key)
        return value if isinstance(value, dict) else {}

    @staticmethod
    def _yaml_str(payload: object, key: str, default: str = '') -> str:
        if not isinstance(payload, dict):
            return default
        value = payload.get(key, default)
        return str(value).strip() if value is not None else default

    @staticmethod
    def _yaml_xyz_m(payload: object) -> tuple[float, float, float] | None:
        if not isinstance(payload, dict):
            return None
        try:
            return (
                float(payload['x']),
                float(payload['y']),
                float(payload['z']),
            )
        except Exception:
            return None

    @classmethod
    def _yaml_xyzw_quaternion(cls, payload: object) -> tuple[float, float, float, float] | None:
        if not isinstance(payload, dict):
            return None
        try:
            return cls._quat_normalize((
                float(payload['x']),
                float(payload['y']),
                float(payload['z']),
                float(payload['w']),
            ))
        except Exception:
            return None

    def _load_active_bin_camera_safety_area(self) -> tuple[BinCameraSafetyArea | None, str]:
        if not self._prefer_camera_inside_bin:
            return None, 'camera-bin preference disabled'
        if yaml is None:
            return None, 'PyYAML unavailable; camera-bin preference skipped'

        active_profile_key = self._active_item_profile_key or self._read_active_item_profile_key()
        if not active_profile_key:
            return None, 'no active item profile; camera-bin preference skipped'

        profile_path = Path(active_profile_key).expanduser()
        profile_root = self._safe_yaml_load(profile_path)
        profile_params = self._yaml_map(self._yaml_map(profile_root, 'item_detect'), 'ros__parameters')
        if not profile_params:
            profile_params = self._yaml_map(self._yaml_map(profile_root, 'bin_detect'), 'ros__parameters')
        bin_teach_text = self._yaml_str(profile_params, 'bin_teach_file')
        if not bin_teach_text:
            return None, f'active profile "{profile_path.name}" has no bin_teach_file'

        bin_teach_path = Path(bin_teach_text).expanduser()
        bin_root = self._safe_yaml_load(bin_teach_path)
        bin_data = self._yaml_map(bin_root, 'bin_teach')
        if not bin_data:
            return None, f'could not read bin teach file "{bin_teach_path}"'

        parent_frame = self._yaml_str(bin_data, 'parent_frame')
        bin_frame_id = self._yaml_str(bin_data, 'bin_frame', 'bin_frame')
        transform = self._yaml_map(bin_data, 'transform')
        parent_to_bin_t = self._yaml_xyz_m(self._yaml_map(transform, 'translation'))
        parent_to_bin_q = self._yaml_xyzw_quaternion(self._yaml_map(transform, 'rotation'))
        if parent_to_bin_t is None or parent_to_bin_q is None:
            return None, f'bin teach file "{bin_teach_path.name}" has no usable parent->bin transform'

        base_to_parent_t = (0.0, 0.0, 0.0)
        base_to_parent_q = (0.0, 0.0, 0.0, 1.0)
        if parent_frame and parent_frame != self._robot_goal_frame_id:
            platform_ref = self._yaml_map(bin_data, 'platform_reference')
            platform_calibration_text = self._yaml_str(platform_ref, 'platform_calibration_file')
            if not platform_calibration_text:
                return None, f'bin parent "{parent_frame}" is not "{self._robot_goal_frame_id}" and has no platform calibration file'

            platform_path = Path(platform_calibration_text).expanduser()
            platform_root = self._safe_yaml_load(platform_path)
            platform_tf = self._yaml_map(platform_root, 'calibration_transform')
            metadata = self._yaml_map(platform_root, 'metadata')
            metadata_parent = self._yaml_str(metadata, 'transform_parent_frame')
            metadata_child = self._yaml_str(metadata, 'transform_child_frame')
            if metadata_parent and metadata_parent != self._robot_goal_frame_id:
                return None, (
                    f'platform calibration parent "{metadata_parent}" is not '
                    f'"{self._robot_goal_frame_id}"'
                )
            if metadata_child and metadata_child != parent_frame:
                return None, (
                    f'platform calibration child "{metadata_child}" is not bin parent '
                    f'"{parent_frame}"'
                )
            base_to_parent_t = self._yaml_xyz_m(self._yaml_map(platform_tf, 'translation'))
            base_to_parent_q = self._yaml_xyzw_quaternion(self._yaml_map(platform_tf, 'rotation'))
            if base_to_parent_t is None or base_to_parent_q is None:
                return None, f'could not read platform calibration "{platform_path}"'

        base_to_bin_t, base_to_bin_q = self._compose_transform_m(
            base_to_parent_t,
            base_to_parent_q,
            parent_to_bin_t,
            parent_to_bin_q,
        )

        marker_positions = self._yaml_map(bin_data, 'marker_positions')
        marker_points_bin: list[tuple[float, float, float]] = []
        for marker_pose in marker_positions.values():
            marker_parent = self._yaml_xyz_m(marker_pose)
            if marker_parent is None:
                continue
            marker_points_bin.append(
                self._transform_point_inverse_m(marker_parent, parent_to_bin_t, parent_to_bin_q)
            )
        if len(marker_points_bin) < 3:
            return None, f'bin teach file "{bin_teach_path.name}" has fewer than 3 marker positions'

        x_values = [point[0] for point in marker_points_bin]
        y_values = [point[1] for point in marker_points_bin]
        x_min = min(x_values)
        x_max = max(x_values)
        y_min = min(y_values)
        y_max = max(y_values)
        margin_m = max(0.0, float(self._camera_bin_safe_margin_mm) * 0.001)
        max_margin_m = max(0.0, min(x_max - x_min, y_max - y_min) * 0.5 - 1e-6)
        margin_m = min(margin_m, max_margin_m)
        return BinCameraSafetyArea(
            profile_path=profile_path,
            bin_teach_path=bin_teach_path,
            bin_frame_id=bin_frame_id,
            base_to_bin_translation_m=base_to_bin_t,
            base_to_bin_rotation_xyzw=base_to_bin_q,
            x_min_m=x_min + margin_m,
            x_max_m=x_max - margin_m,
            y_min_m=y_min + margin_m,
            y_max_m=y_max - margin_m,
            margin_m=margin_m,
        ), f'camera-bin preference loaded from "{bin_teach_path.name}"'

    def _read_tool_teach_sidecar_payload(self, active_profile_key: str) -> dict[str, object] | None:
        sidecar_path = tool_teach_path_for_profile(active_profile_key)
        try:
            payload = read_simple_yaml_mapping(sidecar_path)
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to read tool teach sidecar "{sidecar_path}": {exc}'
            )
            return None
        return payload if isinstance(payload, dict) else None

    def _load_saved_tool_offsets_for_profile(self, active_profile_key: str) -> dict[str, float] | None:
        profile_offsets = self._read_tool_teach_sidecar_payload(active_profile_key)
        if not isinstance(profile_offsets, dict):
            return None
        return {
            'item_standoff_z_mm': max(
                POST_STOP_Z_OFFSET_MIN,
                min(
                    POST_STOP_Z_OFFSET_MAX,
                    float(profile_offsets.get(
                        'item_standoff_z_mm',
                        profile_offsets.get('tray_standoff_z_mm', self._post_stop_z_offset_mm),
                    )),
                ),
            ),
            'approach_z_up_mm': max(
                APPROACH_Z_UP_MIN,
                min(
                    APPROACH_Z_UP_MAX,
                    float(profile_offsets.get('approach_z_up_mm', self._approach_z_up_mm)),
                ),
            ),
            'final_z_up_mm': max(
                FINAL_Z_UP_MIN,
                min(
                    FINAL_Z_UP_MAX,
                    float(profile_offsets.get('final_z_up_mm', self._final_z_up_mm)),
                ),
            ),
            'settling_time_sec': max(
                SETTLING_TIME_MIN_SEC,
                min(
                    SETTLING_TIME_MAX_SEC,
                    float(profile_offsets.get('settling_time_sec', self._settling_time_sec)),
                ),
            ),
            'tool_offset_x_mm': self._clamp_tool_offset_translation_mm(
                profile_offsets.get('tool_offset_x_mm', 0.0)
            ),
            'tool_offset_y_mm': self._clamp_tool_offset_translation_mm(
                profile_offsets.get('tool_offset_y_mm', 0.0)
            ),
            'tool_offset_z_mm': self._clamp_tool_offset_translation_mm(
                profile_offsets.get('tool_offset_z_mm', 0.0)
            ),
            'tool_offset_rx_deg': self._clamp_tool_offset_rotation_deg(
                profile_offsets.get('tool_offset_rx_deg', 0.0)
            ),
            'tool_offset_ry_deg': self._clamp_tool_offset_rotation_deg(
                profile_offsets.get('tool_offset_ry_deg', 0.0)
            ),
            'tool_offset_rz_deg': self._clamp_tool_offset_rotation_deg(
                profile_offsets.get('tool_offset_rz_deg', 0.0)
            ),
        }

    def _apply_saved_tool_offsets_locked(self, profile_offsets: dict[str, float]) -> None:
        self._post_stop_z_offset_mm = float(profile_offsets['item_standoff_z_mm'])
        self._approach_z_up_mm = float(profile_offsets['approach_z_up_mm'])
        self._final_z_up_mm = float(profile_offsets['final_z_up_mm'])
        self._settling_time_sec = float(profile_offsets['settling_time_sec'])
        self._tool_offset_x_mm = float(profile_offsets['tool_offset_x_mm'])
        self._tool_offset_y_mm = float(profile_offsets['tool_offset_y_mm'])
        self._tool_offset_z_mm = float(profile_offsets['tool_offset_z_mm'])
        self._tool_offset_rx_deg = float(profile_offsets['tool_offset_rx_deg'])
        self._tool_offset_ry_deg = float(profile_offsets['tool_offset_ry_deg'])
        self._tool_offset_rz_deg = float(profile_offsets['tool_offset_rz_deg'])

    def _sync_profile_tool_offsets_from_state(self, force: bool = False) -> tuple[str | None, dict[str, float] | None]:
        profile_state_path = self._item_profile_state_path
        if not profile_state_path.exists() and self._legacy_item_profile_state_path.exists():
            profile_state_path = self._legacy_item_profile_state_path
        try:
            current_mtime_ns = profile_state_path.stat().st_mtime_ns
        except FileNotFoundError:
            current_mtime_ns = None
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to stat item profile state file "{profile_state_path}": {exc}'
            )
            current_mtime_ns = None

        if not force and current_mtime_ns == self._profile_state_mtime_ns:
            with self._lock:
                saved_offsets = (
                    dict(self._active_profile_saved_tool_offsets)
                    if self._active_profile_saved_tool_offsets is not None
                    else None
                )
                return self._active_item_profile_key, saved_offsets

        self._profile_state_mtime_ns = current_mtime_ns
        active_profile_key = self._read_active_item_profile_key()
        saved_offsets = None
        if active_profile_key is not None:
            saved_offsets = self._load_saved_tool_offsets_for_profile(active_profile_key)

        with self._lock:
            self._active_item_profile_key = active_profile_key
            self._active_profile_saved_tool_offsets = dict(saved_offsets) if saved_offsets is not None else None
            if saved_offsets is not None:
                self._apply_saved_tool_offsets_locked(saved_offsets)
        return active_profile_key, dict(saved_offsets) if saved_offsets is not None else None

    def get_active_profile_tool_offset_state(self, force: bool = False) -> tuple[str | None, dict[str, float] | None]:
        return self._sync_profile_tool_offsets_from_state(force=force)

    @staticmethod
    def _solve_intercept_time_sec(
        relative_position_mm: tuple[float, float, float],
        target_velocity_mmps: tuple[float, float, float],
        interceptor_speed_mmps: float,
    ) -> float | None:
        s = max(1e-6, float(interceptor_speed_mmps))
        r = (
            float(relative_position_mm[0]),
            float(relative_position_mm[1]),
            float(relative_position_mm[2]),
        )
        v = (
            float(target_velocity_mmps[0]),
            float(target_velocity_mmps[1]),
            float(target_velocity_mmps[2]),
        )

        c = ItemPickNode._vector_dot3(r, r)
        if c <= 1e-9:
            return 0.0

        a = ItemPickNode._vector_dot3(v, v) - (s * s)
        b = 2.0 * ItemPickNode._vector_dot3(r, v)
        eps = 1e-9

        if abs(a) <= eps:
            if abs(b) <= eps:
                return None
            t_linear = -c / b
            return t_linear if t_linear >= 0.0 else None

        disc = (b * b) - (4.0 * a * c)
        if disc < 0.0:
            return None

        sqrt_disc = math.sqrt(max(0.0, disc))
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        candidates = [t for t in (t1, t2) if t >= 0.0]
        if not candidates:
            return None
        return min(candidates)

    def _item_pose_camera_to_base(
        self,
        item_x_mm: float,
        item_y_mm: float,
        item_z_mm: float,
        item_rx_deg: float,
        item_ry_deg: float,
        item_rz_deg: float,
        camera_frame_id: str,
    ) -> tuple[
        float,
        float,
        float,
        float,
        float,
        float,
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ] | None:
        source_frame = str(camera_frame_id).strip() or 'camera_color_optical_frame'
        target_frame = self._robot_goal_frame_id
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self._goal_tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self._set_action_text(f'TF lookup failed {target_frame}<-{source_frame}: {exc}')
            return None

        q_base_camera = self._quat_normalize((
            float(tf_msg.transform.rotation.x),
            float(tf_msg.transform.rotation.y),
            float(tf_msg.transform.rotation.z),
            float(tf_msg.transform.rotation.w),
        ))
        t_base_camera_m = (
            float(tf_msg.transform.translation.x),
            float(tf_msg.transform.translation.y),
            float(tf_msg.transform.translation.z),
        )
        p_camera_item_m = (
            float(item_x_mm) * 0.001,
            float(item_y_mm) * 0.001,
            float(item_z_mm) * 0.001,
        )
        p_base_item_offset_m = self._rotate_vector_by_quaternion(p_camera_item_m, q_base_camera)
        p_base_item_m = (
            t_base_camera_m[0] + p_base_item_offset_m[0],
            t_base_camera_m[1] + p_base_item_offset_m[1],
            t_base_camera_m[2] + p_base_item_offset_m[2],
        )

        q_camera_item = self._quat_normalize(
            self._rpy_deg_to_quaternion(item_rx_deg, item_ry_deg, item_rz_deg),
        )
        q_base_item = self._quat_normalize(self._quat_multiply(q_base_camera, q_camera_item))
        item_rpy_base_deg = self._quaternion_to_rpy_deg(q_base_item)

        return (
            p_base_item_m[0] * 1000.0,
            p_base_item_m[1] * 1000.0,
            p_base_item_m[2] * 1000.0,
            item_rpy_base_deg[0],
            item_rpy_base_deg[1],
            item_rpy_base_deg[2],
            q_base_item,
            q_base_camera,
        )

    def _lookup_camera_offset_in_gripper_m(self) -> tuple[float, float, float] | None:
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._robot_gripper_frame_id,
                self._camera_safety_frame_id,
                Time(),
                timeout=Duration(seconds=self._goal_tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Camera-bin preference skipped: TF lookup failed '
                f'{self._robot_gripper_frame_id}<-{self._camera_safety_frame_id}: {exc}'
            )
            return None
        return (
            float(tf_msg.transform.translation.x),
            float(tf_msg.transform.translation.y),
            float(tf_msg.transform.translation.z),
        )

    def _camera_position_for_goal_m(
        self,
        goal_xyz_mm: tuple[float, float, float],
        goal_rotation_xyzw: tuple[float, float, float, float],
        camera_offset_gripper_m: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        camera_offset_base_m = self._rotate_vector_by_quaternion(
            camera_offset_gripper_m,
            goal_rotation_xyzw,
        )
        return (
            (float(goal_xyz_mm[0]) * 0.001) + camera_offset_base_m[0],
            (float(goal_xyz_mm[1]) * 0.001) + camera_offset_base_m[1],
            (float(goal_xyz_mm[2]) * 0.001) + camera_offset_base_m[2],
        )

    def _camera_inside_bin_area(
        self,
        camera_base_m: tuple[float, float, float],
        safety_area: BinCameraSafetyArea,
    ) -> tuple[bool, tuple[float, float, float]]:
        camera_bin_m = self._transform_point_inverse_m(
            camera_base_m,
            safety_area.base_to_bin_translation_m,
            safety_area.base_to_bin_rotation_xyzw,
        )
        inside = (
            safety_area.x_min_m <= camera_bin_m[0] <= safety_area.x_max_m and
            safety_area.y_min_m <= camera_bin_m[1] <= safety_area.y_max_m
        )
        return inside, camera_bin_m

    def _choose_camera_preferred_candidate_index(
        self,
        preferred_index: int,
        q_base_goal_candidates: tuple[tuple[float, float, float, float], ...],
        candidate_goal_xyz_mm: tuple[tuple[float, float, float], ...],
    ) -> tuple[int, str]:
        if not self._prefer_camera_inside_bin or len(q_base_goal_candidates) <= 1:
            return preferred_index, ''

        safety_area, safety_reason = self._load_active_bin_camera_safety_area()
        if safety_area is None:
            return preferred_index, safety_reason

        camera_offset_gripper_m = self._lookup_camera_offset_in_gripper_m()
        if camera_offset_gripper_m is None:
            return preferred_index, 'camera-bin preference skipped: camera TF unavailable'

        checks: list[tuple[bool, tuple[float, float, float]]] = []
        for idx, q_goal in enumerate(q_base_goal_candidates):
            camera_base_m = self._camera_position_for_goal_m(
                candidate_goal_xyz_mm[idx],
                q_goal,
                camera_offset_gripper_m,
            )
            checks.append(self._camera_inside_bin_area(camera_base_m, safety_area))

        preferred_inside, preferred_bin_m = checks[preferred_index]
        if preferred_inside:
            return preferred_index, (
                f'Camera-bin preference: preferred pose keeps {self._camera_safety_frame_id} '
                f'inside {safety_area.bin_frame_id} '
                f'(x={preferred_bin_m[0] * 1000.0:.1f}, y={preferred_bin_m[1] * 1000.0:.1f} mm).'
            )

        for idx, (inside, camera_bin_m) in enumerate(checks):
            if idx == preferred_index or not inside:
                continue
            message = (
                f'Camera-bin preference selected 180deg opposite item-X pose: '
                f'preferred camera would be outside {safety_area.bin_frame_id} '
                f'(x={preferred_bin_m[0] * 1000.0:.1f}, y={preferred_bin_m[1] * 1000.0:.1f} mm), '
                f'flipped camera is inside '
                f'(x={camera_bin_m[0] * 1000.0:.1f}, y={camera_bin_m[1] * 1000.0:.1f} mm).'
            )
            self.get_logger().info(message)
            return idx, message

        message = (
            f'Camera-bin preference warning: both item-X pose options put '
            f'{self._camera_safety_frame_id} outside {safety_area.bin_frame_id}; '
            f'continuing with preferred pick anyway '
            f'(x={preferred_bin_m[0] * 1000.0:.1f}, y={preferred_bin_m[1] * 1000.0:.1f} mm).'
        )
        self.get_logger().warn(message)
        return preferred_index, message

    def _publish_goal_debug_transform(
        self,
        parent_frame: str,
        child_frame: str,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
    ) -> None:
        if not self._publish_goal_debug_tf:
            return

        frame_id = str(parent_frame).strip() or self._robot_goal_frame_id
        child = str(child_frame).strip() or self._post_stop_movel_goal_debug_frame_id

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(x_mm) * 0.001
        tf_msg.transform.translation.y = float(y_mm) * 0.001
        tf_msg.transform.translation.z = float(z_mm) * 0.001
        qx, qy, qz, qw = self._rpy_deg_to_quaternion(
            float(rx_deg),
            float(ry_deg),
            float(rz_deg),
        )
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._goal_static_tf_by_child[child] = tf_msg
        self._goal_tf_static_broadcaster.sendTransform(list(self._goal_static_tf_by_child.values()))

    def _publish_goal_debug_transforms(
        self,
        approach_goal: tuple[float, float, float, float, float, float],
        nominal_approach_goal: tuple[float, float, float, float, float, float],
        tool_offset: tuple[float, float, float, float, float, float],
    ) -> None:
        self._publish_goal_debug_transform(
            self._robot_goal_frame_id,
            self._post_stop_movel_goal_debug_frame_id,
            approach_goal[0],
            approach_goal[1],
            approach_goal[2],
            approach_goal[3],
            approach_goal[4],
            approach_goal[5],
        )
        self._publish_goal_debug_transform(
            self._robot_goal_frame_id,
            self._post_stop_movel_goal_nominal_debug_frame_id,
            nominal_approach_goal[0],
            nominal_approach_goal[1],
            nominal_approach_goal[2],
            nominal_approach_goal[3],
            nominal_approach_goal[4],
            nominal_approach_goal[5],
        )
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_nominal_debug_frame_id,
            self._post_stop_movel_goal_tool_offset_debug_frame_id,
            tool_offset[0],
            tool_offset[1],
            tool_offset[2],
            tool_offset[3],
            tool_offset[4],
            tool_offset[5],
        )

    def _publish_goal_tool_axis_tips(self, axis_length_mm: float = GOAL_TF_DIAG_AXIS_LENGTH_MM) -> None:
        axis_len = max(1.0, float(axis_length_mm))
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_tool_axis_x_tip_frame_id,
            axis_len,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_tool_axis_y_tip_frame_id,
            0.0,
            axis_len,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_tool_axis_z_tip_frame_id,
            0.0,
            0.0,
            axis_len,
            0.0,
            0.0,
            0.0,
        )

    def _publish_primary_goal_debug_transform(
        self,
        goal: tuple[float, float, float, float, float, float],
    ) -> None:
        self._publish_goal_debug_transform(
            self._robot_goal_frame_id,
            self._post_stop_movel_goal_debug_frame_id,
            goal[0],
            goal[1],
            goal[2],
            goal[3],
            goal[4],
            goal[5],
        )

    def _item_pose_callback(self, msg: PoseStamped) -> None:
        header_stamp_sec = self._builtin_time_to_sec(msg.header.stamp)
        orientation = (
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        rpy_deg = self._quaternion_to_rpy_deg(orientation)
        item_target = ItemPoseTarget(
            position_mm=(
                float(msg.pose.position.x) * 1000.0,
                float(msg.pose.position.y) * 1000.0,
                float(msg.pose.position.z) * 1000.0,
            ),
            rpy_deg=rpy_deg,
            frame_id=str(msg.header.frame_id),
            stamp_sec=header_stamp_sec,
        )

        should_send_stop = False
        tf_only_mode = False
        dispatch_target: ItemPoseTarget | None = None
        post_speed_mm_s = 0.0
        x_offset_mm = 0.0
        y_offset_mm = 0.0
        z_offset_mm = 0.0
        approach_z_up_mm = 0.0
        final_z_up_mm = 0.0
        settling_time_sec = 0.0
        tool_offset_x_mm = 0.0
        tool_offset_y_mm = 0.0
        tool_offset_z_mm = 0.0
        tool_offset_rx_deg = 0.0
        tool_offset_ry_deg = 0.0
        tool_offset_rz_deg = 0.0
        watch_timeout_sec = ITEM_POSE_WATCH_TIMEOUT_SEC
        with self._lock:
            self._item_pose_seq += 1
            self._last_item_target = item_target
            if not self._item_pose_watch_armed:
                return
            if self._item_pose_seq <= self._item_pose_watch_seq_floor:
                return
            if self._item_pose_watch_stop_dispatched:
                return
            if time.monotonic() > self._item_pose_watch_deadline_monotonic:
                watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
                self._reset_runtime_state_locked(
                    f'No item pose within {watch_timeout_sec:.0f}s. Node reset.'
                )
                return

            self._item_pose_watch_armed = False
            self._item_pose_watch_stop_dispatched = True
            tf_only_mode = bool(self._item_pose_watch_tf_only_mode)
            watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
            should_send_stop = True
            dispatch_target = item_target
            post_speed_mm_s = float(self._post_stop_movel_speed_mm_s)
            x_offset_mm = float(self._post_stop_x_offset_mm)
            y_offset_mm = float(self._post_stop_y_offset_mm)
            z_offset_mm = float(self._post_stop_z_offset_mm)
            approach_z_up_mm = float(self._approach_z_up_mm)
            final_z_up_mm = float(self._final_z_up_mm)
            settling_time_sec = float(self._settling_time_sec)
            tool_offset_x_mm = float(self._tool_offset_x_mm)
            tool_offset_y_mm = float(self._tool_offset_y_mm)
            tool_offset_z_mm = float(self._tool_offset_z_mm)
            tool_offset_rx_deg = float(self._tool_offset_rx_deg)
            tool_offset_ry_deg = float(self._tool_offset_ry_deg)
            tool_offset_rz_deg = float(self._tool_offset_rz_deg)

        if tf_only_mode:
            if should_send_stop and dispatch_target is not None:
                self._set_action_text(
                    'Item pose update detected. Troubleshoot mode: goal TF preview only...'
                )
                worker = threading.Thread(
                    target=self._preview_goal_only_request,
                    args=(
                        dispatch_target,
                        x_offset_mm,
                        y_offset_mm,
                        z_offset_mm,
                        approach_z_up_mm,
                        final_z_up_mm,
                        settling_time_sec,
                        tool_offset_x_mm,
                        tool_offset_y_mm,
                        tool_offset_z_mm,
                        tool_offset_rx_deg,
                        tool_offset_ry_deg,
                        tool_offset_rz_deg,
                    ),
                    daemon=True,
                )
                worker.start()
            return

        if should_send_stop and dispatch_target is not None:
            self._set_action_text('Item pose update detected. Starting pick sequence...')
            worker = threading.Thread(
                target=self._send_movel_request,
                args=(
                    dispatch_target,
                    post_speed_mm_s,
                    x_offset_mm,
                    y_offset_mm,
                    z_offset_mm,
                    approach_z_up_mm,
                    final_z_up_mm,
                    settling_time_sec,
                    tool_offset_x_mm,
                    tool_offset_y_mm,
                    tool_offset_z_mm,
                    tool_offset_rx_deg,
                    tool_offset_ry_deg,
                    tool_offset_rz_deg,
                ),
                daemon=True,
            )
            worker.start()

    def _compute_base_goal_from_item_target(
        self,
        item_target: ItemPoseTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
        ee_speed_mmps: float,
        predict_target_motion: bool = True,
    ) -> PredictedGoal | None:
        _ = ee_speed_mmps
        _ = predict_target_motion
        target_x, target_y, target_z = item_target.position_mm
        target_rx, target_ry, target_rz = item_target.rpy_deg
        frame_id = item_target.frame_id
        item_base_pose = self._item_pose_camera_to_base(
            target_x,
            target_y,
            target_z,
            target_rx,
            target_ry,
            target_rz,
            frame_id,
        )
        if item_base_pose is None:
            return None

        item_base_x, item_base_y, item_base_z, _, _, _, q_base_item, _ = item_base_pose
        # Sterilize item frame orientation first so generated goals are world-normal.
        item_yaw_deg = self._yaw_deg_from_quaternion_xy_axis(q_base_item)
        q_base_item_sterilized = self._quat_normalize(
            self._rpy_deg_to_quaternion(0.0, 0.0, item_yaw_deg)
        )
        item_local_x_in_base = self._rotate_vector_by_quaternion((1.0, 0.0, 0.0), q_base_item_sterilized)
        item_local_y_in_base = self._rotate_vector_by_quaternion((0.0, 1.0, 0.0), q_base_item_sterilized)
        item_local_z_in_base = self._rotate_vector_by_quaternion((0.0, 0.0, 1.0), q_base_item_sterilized)
        item_age_sec = 0.0
        item_now_x = item_base_x
        item_now_y = item_base_y
        item_now_z = item_base_z

        stand_off_vec_base_mm = (
            (item_local_x_in_base[0] * x_offset_mm)
            + (item_local_y_in_base[0] * y_offset_mm)
            + (item_local_z_in_base[0] * z_offset_mm),
            (item_local_x_in_base[1] * x_offset_mm)
            + (item_local_y_in_base[1] * y_offset_mm)
            + (item_local_z_in_base[1] * z_offset_mm),
            (item_local_x_in_base[2] * x_offset_mm)
            + (item_local_y_in_base[2] * y_offset_mm)
            + (item_local_z_in_base[2] * z_offset_mm),
        )
        desired_now_goal_mm = (
            item_now_x + stand_off_vec_base_mm[0],
            item_now_y + stand_off_vec_base_mm[1],
            item_now_z + stand_off_vec_base_mm[2],
        )

        lead_time_sec = 0.0
        nominal_x_goal = desired_now_goal_mm[0]
        nominal_y_goal = desired_now_goal_mm[1]
        nominal_z_goal = desired_now_goal_mm[2]

        # Build two valid item-aligned EE orientation candidates (+/- direction),
        # then choose the one with minimal rotation from current TCP.
        # No fixed 90-degree Z offset is baked here; use tool_offset_rz_deg for that.
        q_align_options = (
            self._rpy_deg_to_quaternion(180.0, 0.0, 0.0),
            self._rpy_deg_to_quaternion(180.0, 0.0, 180.0),
        )
        q_tool_offset = self._quat_normalize(
            self._rpy_deg_to_quaternion(tool_offset_rx_deg, tool_offset_ry_deg, tool_offset_rz_deg)
        )
        q_base_nominal_candidates = tuple(
            self._quat_normalize(self._quat_multiply(q_base_item_sterilized, q_align))
            for q_align in q_align_options
        )
        q_base_goal_raw_candidates = tuple(
            self._quat_normalize(self._quat_multiply(q_nominal, q_tool_offset))
            for q_nominal in q_base_nominal_candidates
        )
        # Final sterilization: all generated goal orientations keep Z world-normal.
        q_base_goal_candidates = tuple(
            self._sterilize_quaternion_to_world_normal(q_raw)
            for q_raw in q_base_goal_raw_candidates
        )
        preferred_candidate_idx = self._choose_min_rotation_candidate_index(q_base_goal_candidates)
        candidate_tool_offsets_base_mm = tuple(
            self._rotate_vector_by_quaternion(
                (tool_offset_x_mm, tool_offset_y_mm, tool_offset_z_mm),
                q_nominal,
            )
            for q_nominal in q_base_nominal_candidates
        )
        candidate_goal_xyz_mm = tuple(
            (
                nominal_x_goal + tool_offset_base_mm[0],
                nominal_y_goal + tool_offset_base_mm[1],
                nominal_z_goal + tool_offset_base_mm[2],
            )
            for tool_offset_base_mm in candidate_tool_offsets_base_mm
        )
        selected_candidate_idx, camera_safety_message = self._choose_camera_preferred_candidate_index(
            preferred_candidate_idx,
            q_base_goal_candidates,
            candidate_goal_xyz_mm,
        )
        q_base_nominal_goal = q_base_nominal_candidates[selected_candidate_idx]
        q_base_goal = q_base_goal_candidates[selected_candidate_idx]
        target_x_goal, target_y_goal, target_z_goal = candidate_goal_xyz_mm[selected_candidate_idx]

        nominal_rx_deg, nominal_ry_deg, nominal_rz_deg = self._quaternion_to_rpy_deg(q_base_nominal_goal)
        goal_rx_deg, goal_ry_deg, goal_rz_deg = self._quaternion_to_rpy_deg(q_base_goal)
        orientation_choice = 'preferred'
        if selected_candidate_idx != preferred_candidate_idx:
            orientation_choice = 'flipped_180'

        return PredictedGoal(
            x_mm=target_x_goal,
            y_mm=target_y_goal,
            z_mm=target_z_goal,
            rx_deg=goal_rx_deg,
            ry_deg=goal_ry_deg,
            rz_deg=goal_rz_deg,
            source_frame_id=frame_id,
            lead_time_sec=lead_time_sec,
            item_age_sec=item_age_sec,
            item_speed_base_mmps=0.0,
            nominal_x_mm=nominal_x_goal,
            nominal_y_mm=nominal_y_goal,
            nominal_z_mm=nominal_z_goal,
            nominal_rx_deg=nominal_rx_deg,
            nominal_ry_deg=nominal_ry_deg,
            nominal_rz_deg=nominal_rz_deg,
            orientation_choice=orientation_choice,
            camera_safety_message=camera_safety_message,
        )

    def _send_movel_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        _ = (reference_pose, speed_mm_s)
        if forced_v_percent is not None:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
        else:
            v_percent = 100
            mapping_source = 'locked_max'
        a_percent = DEFAULT_ACC_PERCENT
        if forced_a_percent is not None:
            a_percent = max(1, min(100, int(forced_a_percent)))
        movl_request = MovL.Request()
        movl_request.mode = False
        movl_request.a = float(goal[0])
        movl_request.b = float(goal[1])
        movl_request.c = float(goal[2])
        movl_request.d = float(goal[3])
        movl_request.e = float(goal[4])
        movl_request.f = float(goal[5])
        movl_request.param_value = self._build_motion_param_value(v_percent, a_percent)
        movl_label = (
            f'{label_prefix}('
            f'{movl_request.a:.1f},{movl_request.b:.1f},{movl_request.c:.1f},'
            f'{movl_request.d:.2f},{movl_request.e:.2f},{movl_request.f:.2f},'
            f'v={v_percent},a={a_percent})'
        )
        movl_response = self._call_service(self._mov_l_client, movl_request, movl_label)
        if movl_response is None:
            return False, v_percent, mapping_source
        if int(getattr(movl_response, 'res', -1)) < 0:
            return False, v_percent, mapping_source
        return True, v_percent, mapping_source

    def _send_do(self, index: int, status: int, time_ms: int = 0) -> bool:
        if not self._wait_for_service(self._do_client, 'DO'):
            return False

        request = DO.Request()
        request.index = int(index)
        request.status = int(status)
        request.time = int(time_ms)
        label = f'DO(index={request.index},status={request.status},time={request.time})'
        response = self._call_service(
            self._do_client,
            request,
            label,
            timeout_sec=4.0,
        )
        return response is not None and int(getattr(response, 'res', -1)) >= 0

    def _gripper_set_open_hold(self) -> bool:
        self._set_action_text('Gripper open-hold: disable close (DO1 OFF), enable open (DO2 ON)...')
        if not self._send_do(1, 0):
            return False
        return self._send_do(2, 1)

    def _gripper_set_open_suction(self) -> bool:
        self._set_action_text(
            'Gripper pickup state: disable close (DO1 OFF), enable open (DO2 ON), enable suction (DO3 ON)...'
        )
        if not self._send_do(GRIPPER_DO_CLOSE_INDEX, 0):
            return False
        if not self._send_do(GRIPPER_DO_OPEN_INDEX, 1):
            return False
        return self._send_do(GRIPPER_DO_SUCTION_INDEX, 1)

    def _gripper_set_close_hold(self) -> bool:
        self._set_action_text(
            'Gripper close-hold: disable open (DO2 OFF), enable close (DO1 ON), enable suction (DO3 ON)...'
        )
        if not self._send_do(GRIPPER_DO_OPEN_INDEX, 0):
            return False
        if not self._send_do(GRIPPER_DO_CLOSE_INDEX, 1):
            return False
        return self._send_do(GRIPPER_DO_SUCTION_INDEX, 1)

    def _wait_for_tcp_xyz_goal(
        self,
        goal_xyz_mm: tuple[float, float, float],
        tolerance_mm: float = TCP_GOAL_REACHED_TOLERANCE_MM,
        timeout_sec: float | None = None,
        update_action_text: bool = True,
    ) -> bool:
        tolerance = max(0.1, float(tolerance_mm))
        deadline = None
        if timeout_sec is not None:
            deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while rclpy.ok():
            if self._is_cancel_requested():
                if update_action_text:
                    self._set_action_text('Sequence cancelled while waiting for pick position.')
                return False
            if deadline is not None and time.monotonic() >= deadline:
                break
            snapshot = self.snapshot()
            if snapshot.tcp_stamp is None:
                time.sleep(0.02)
                continue
            dx = float(snapshot.tcp_values.get('x', 0.0)) - float(goal_xyz_mm[0])
            dy = float(snapshot.tcp_values.get('y', 0.0)) - float(goal_xyz_mm[1])
            dz = float(snapshot.tcp_values.get('z', 0.0)) - float(goal_xyz_mm[2])
            distance_mm = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            if distance_mm <= tolerance:
                return True
            time.sleep(0.02)
        if not rclpy.ok():
            if update_action_text:
                self._set_action_text('ROS shutdown while waiting for pick pose reach.')
            return False
        if update_action_text:
            self._set_action_text(
                f'Timeout waiting for pick pose reach (tol={tolerance:.1f} mm).'
            )
        return False

    def _wait_settling_time(self, settling_time_sec: float, label: str) -> bool:
        wait_sec = max(0.0, float(settling_time_sec))
        if wait_sec <= 1e-6:
            return True
        self._set_action_text(f'{label}: settling for {wait_sec:.1f}s...')
        deadline = time.monotonic() + wait_sec
        while rclpy.ok():
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled during settling wait.')
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(0.02, remaining))
        self._set_action_text('ROS shutdown during settling wait.')
        return False

    def _preview_goal_only_request(
        self,
        item_target: ItemPoseTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        approach_z_up_mm: float,
        final_z_up_mm: float,
        settling_time_sec: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> None:
        try:
            self._set_action_text('Computing item goal preview in base frame...')
            base_goal = self._compute_base_goal_from_item_target(
                item_target,
                x_offset_mm,
                y_offset_mm,
                z_offset_mm,
                tool_offset_x_mm,
                tool_offset_y_mm,
                tool_offset_z_mm,
                tool_offset_rx_deg,
                tool_offset_ry_deg,
                tool_offset_rz_deg,
                self._post_stop_movel_speed_mm_s,
                predict_target_motion=False,
            )
            if base_goal is None:
                return

            approach_goal = (
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm + float(approach_z_up_mm),
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_primary_goal_debug_transform(approach_goal)
            self._set_action_text(
                f'Previewed approach goal from {base_goal.source_frame_id}: '
                f'pick pose + Z stand-off ({z_offset_mm:.1f} mm), with Approach Z '
                f'({approach_z_up_mm:.1f} mm), final Z-up ({final_z_up_mm:.1f} mm), '
                f'settling {settling_time_sec:.1f}s, '
                f'tool offset=({tool_offset_x_mm:.1f},{tool_offset_y_mm:.1f},{tool_offset_z_mm:.1f},'
                f'{tool_offset_rx_deg:.1f},{tool_offset_ry_deg:.1f},{tool_offset_rz_deg:.1f}). '
                f'TF-only frame="{self._post_stop_movel_goal_debug_frame_id}".'
            )
        except Exception as exc:
            self.get_logger().error(f'Preview goal computation failed: {exc}')
            self._set_action_text(f'Preview goal computation failed: {exc}')
        finally:
            self._set_busy(False)

    def _arm_item_pose_watch_locked(self) -> int:
        self._item_pose_watch_generation += 1
        self._item_pose_watch_armed = True
        self._item_pose_watch_seq_floor = self._item_pose_seq
        self._item_pose_watch_deadline_monotonic = (
            time.monotonic() + max(
                ITEM_POSE_WATCH_TIMEOUT_MIN,
                min(ITEM_POSE_WATCH_TIMEOUT_MAX, float(self._item_pose_watch_timeout_sec)),
            )
        )
        self._item_pose_watch_stop_dispatched = False
        return self._item_pose_watch_generation

    def _item_pose_watchdog_worker(self, generation: int) -> None:
        while rclpy.ok():
            if self.count_publishers(ITEM_POSE_TOPIC) <= 0:
                self._reset_runtime_state(
                    f'No item pose publisher on "{ITEM_POSE_TOPIC}". Node reset.'
                )
                return
            with self._lock:
                if self._item_pose_watch_generation != generation:
                    return
                if not self._item_pose_watch_armed:
                    return
                remaining_sec = self._item_pose_watch_deadline_monotonic - time.monotonic()
                if remaining_sec <= 0.0:
                    watch_timeout_sec = max(
                        ITEM_POSE_WATCH_TIMEOUT_MIN,
                        min(ITEM_POSE_WATCH_TIMEOUT_MAX, float(self._item_pose_watch_timeout_sec)),
                    )
                    self._reset_runtime_state_locked(
                        f'No item pose within {watch_timeout_sec:.0f}s. Node reset.'
                    )
                    return
            time.sleep(min(0.1, max(0.02, remaining_sec)))

    def _send_movel_request(
        self,
        item_target: ItemPoseTarget,
        post_speed_mm_s: float,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        approach_z_up_mm: float,
        final_z_up_mm: float,
        settling_time_sec: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> None:
        seek_complete_notified = False
        busy_released_after_final_zup_queue = False
        try:
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled before dispatch.')
                return
            self._set_action_text('Computing item goal in base frame...')

            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled before goal computation.')
                return
            if not self._wait_for_service(self._mov_l_client, 'MovL'):
                return

            base_goal = self._compute_base_goal_from_item_target(
                item_target,
                x_offset_mm,
                y_offset_mm,
                z_offset_mm,
                tool_offset_x_mm,
                tool_offset_y_mm,
                tool_offset_z_mm,
                tool_offset_rx_deg,
                tool_offset_ry_deg,
                tool_offset_rz_deg,
                post_speed_mm_s,
            )
            if base_goal is None:
                return
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled during goal computation.')
                return

            snapshot = self.snapshot()
            current_pose = (
                float(snapshot.tcp_values.get('x', 0.0)),
                float(snapshot.tcp_values.get('y', 0.0)),
                float(snapshot.tcp_values.get('z', 0.0)),
                float(snapshot.tcp_values.get('rx', 0.0)),
                float(snapshot.tcp_values.get('ry', 0.0)),
                float(snapshot.tcp_values.get('rz', 0.0)),
            )
            pick_goal = (
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            approach_goal = (
                pick_goal[0],
                pick_goal[1],
                pick_goal[2] + float(approach_z_up_mm),
                pick_goal[3],
                pick_goal[4],
                pick_goal[5],
            )
            final_z_goal = (
                approach_goal[0],
                approach_goal[1],
                approach_goal[2] + float(final_z_up_mm),
                approach_goal[3],
                approach_goal[4],
                approach_goal[5],
            )
            self._publish_primary_goal_debug_transform(approach_goal)

            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled before queued motion dispatch.')
                return
            self._set_action_text('Queueing approach and descent...')
            approach_ok, approach_v, approach_map = self._send_movel_goal(
                approach_goal,
                current_pose,
                post_speed_mm_s,
                f'MovL approach from {base_goal.source_frame_id}',
                forced_v_percent=100,
                forced_a_percent=100,
            )
            if not approach_ok:
                return

            if not self._gripper_set_open_suction():
                return

            pick_ok, pick_v, pick_map = self._send_movel_goal(
                pick_goal,
                approach_goal,
                post_speed_mm_s,
                'MovL descent with open+suction state',
                forced_v_percent=100,
                forced_a_percent=100,
            )
            if not pick_ok:
                return

            self._set_action_text('Queued approach/descent. Monitoring pickup depth...')
            if not self._wait_for_tcp_xyz_goal(
                (pick_goal[0], pick_goal[1], pick_goal[2]),
            ):
                return
            if not self._wait_settling_time(settling_time_sec, 'Pickup depth reached'):
                return

            if not self._gripper_set_close_hold():
                return

            retract_ok, retract_v, retract_map = self._send_movel_goal(
                approach_goal,
                pick_goal,
                POST_STOP_MOVL_SPEED_MAX,
                'MovL retract with close+suction state',
                forced_v_percent=100,
                forced_a_percent=100,
            )
            if not retract_ok:
                return

            self._set_action_text('Queued retract. Monitoring approach height...')
            if not self._wait_for_tcp_xyz_goal(
                (approach_goal[0], approach_goal[1], approach_goal[2]),
            ):
                return
            if not self._wait_settling_time(settling_time_sec, 'Approach height reached'):
                return

            final_ok, final_v, final_map = self._send_movel_goal(
                final_z_goal,
                approach_goal,
                POST_STOP_MOVL_SPEED_MAX,
                'MovL final Z-up',
                forced_v_percent=100,
                forced_a_percent=100,
            )
            if not final_ok:
                return

            seek_complete_notified = self._notify_item_detect_seek_complete()
            busy_released_after_final_zup_queue = True
            self._set_busy(False)
            self._set_action_text('Queued pick sequence through final Z-up. Ready for next arm.')
            if not self._wait_for_tcp_xyz_goal(
                (final_z_goal[0], final_z_goal[1], final_z_goal[2]),
                update_action_text=False,
            ):
                return

            self.get_logger().info(
                'Completed pick sequence (approach + open/suction DO + descent + '
                'pickup settle + close/suction DO + retract + approach settle + final Z-up): '
                f'pick stand-off offsets (X {x_offset_mm:.0f}, Y {y_offset_mm:.0f}, Z {z_offset_mm:.0f} mm). '
                f'tool offset=({tool_offset_x_mm:.1f},{tool_offset_y_mm:.1f},{tool_offset_z_mm:.1f},'
                f'{tool_offset_rx_deg:.1f},{tool_offset_ry_deg:.1f},{tool_offset_rz_deg:.1f}). '
                f'approach_z={approach_z_up_mm:.0f} mm, final_z_up={final_z_up_mm:.0f} mm, '
                f'settling={settling_time_sec:.1f}s at pickup and approach. '
                f'(v: approach={approach_v}/{approach_map}, down={pick_v}/{pick_map}, '
                f'retract={retract_v}/{retract_map}, final-up={final_v}/{final_map}).'
            )
        except Exception as exc:
            self.get_logger().error(f'MovL predicted-goal flow failed: {exc}')
            self._set_action_text(f'MovL predicted-goal flow failed: {exc}')
        finally:
            if not seek_complete_notified:
                self._notify_item_detect_seek_complete()
            if not busy_released_after_final_zup_queue:
                self._set_busy(False)

    def _notify_item_detect_seek_complete(self) -> bool:
        if not self._item_seek_complete_client.service_is_ready():
            if not self._item_seek_complete_client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warn(
                    f'Item detect seek-complete service not ready: {self._item_seek_complete_service_name}'
                )
                return False

        future = self._item_seek_complete_client.call_async(Trigger.Request())
        started = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - started) >= 1.0:
                self.get_logger().warn(
                    f'Timed out notifying item detect seek completion: {self._item_seek_complete_service_name}'
                )
                return False
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            self.get_logger().warn(f'Item detect seek-complete call failed: {exception}')
            return False

        response = future.result()
        if response is not None and not bool(response.success):
            self.get_logger().warn(f'Item detect seek-complete rejected: {response.message}')
            return False
        return response is not None

    def _set_action_text(self, text: str) -> None:
        with self._lock:
            self._snapshot.action_text = text

    def _set_busy(self, busy: bool) -> None:
        with self._lock:
            self._snapshot.busy = busy

    def _start_sequence_service_callback(
        self,
        request: TrayInterceptStart.Request,
        response: TrayInterceptStart.Response,
    ) -> TrayInterceptStart.Response:
        started = self.run_item_sequence(
            float(request.tray_vector_wait_timeout_sec),
            float(request.ee_intercept_speed_mm_s),
            float(request.tray_intercept_x_offset_mm),
            float(request.tray_intercept_y_offset_mm),
            float(request.tray_standoff_z_mm),
            float(request.follow_distance_mm),
            float(request.post_follow_z_up_mm),
            self.get_settling_time_sec(),
            bool(request.troubleshoot_tf_only),
        )
        with self._lock:
            response.started = bool(started)
            response.message = str(self._snapshot.action_text)
            response.applied_tray_vector_wait_timeout_sec = float(self._item_pose_watch_timeout_sec)
            response.applied_ee_intercept_speed_mm_s = float(self._post_stop_movel_speed_mm_s)
            response.applied_tray_intercept_x_offset_mm = float(self._post_stop_x_offset_mm)
            response.applied_tray_intercept_y_offset_mm = float(self._post_stop_y_offset_mm)
            response.applied_tray_standoff_z_mm = float(self._post_stop_z_offset_mm)
            response.applied_follow_distance_mm = 0.0
            response.applied_post_follow_z_up_mm = float(self._approach_z_up_mm)
            response.applied_troubleshoot_tf_only = bool(self._item_pose_watch_tf_only_mode)
        return response

    def set_track_trigger_handler(self, handler) -> None:
        self._track_trigger_handler = handler

    def _track_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        handler = self._track_trigger_handler
        if handler is not None:
            try:
                started, message = handler()
            except Exception as exc:
                started = False
                message = f'Track virtual-click failed: {exc}'
                self._set_action_text(message)
        else:
            started = self.run_track_from_current_settings()
            with self._lock:
                message = str(self._snapshot.action_text)

        response.success = bool(started)
        response.message = str(message)
        return response

    def _track_status_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self._lock:
            armed = bool(self._item_pose_watch_armed)
            busy = bool(self._snapshot.busy)
            action_text = str(self._snapshot.action_text)

        response.success = armed
        if armed:
            response.message = f'Track armed: waiting for "{ITEM_POSE_TOPIC}". {action_text}'
        elif busy:
            response.message = f'Track busy but not armed. {action_text}'
        else:
            response.message = f'Track not armed. {action_text}'
        return response

    def run_track_from_current_settings(self) -> bool:
        with self._lock:
            item_pose_watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
            post_stop_z_offset_mm = float(self._post_stop_z_offset_mm)
            approach_z_up_mm = float(self._approach_z_up_mm)
            settling_time_sec = float(self._settling_time_sec)
            tf_only_mode = bool(self._item_pose_watch_tf_only_mode)
            tool_offset_x_mm = float(self._tool_offset_x_mm)
            tool_offset_y_mm = float(self._tool_offset_y_mm)
            tool_offset_z_mm = float(self._tool_offset_z_mm)
            tool_offset_rx_deg = float(self._tool_offset_rx_deg)
            tool_offset_ry_deg = float(self._tool_offset_ry_deg)
            tool_offset_rz_deg = float(self._tool_offset_rz_deg)

        return self.run_item_sequence(
            item_pose_watch_timeout_sec,
            LOCKED_MAX_SPEED_MM_S,
            0.0,
            0.0,
            post_stop_z_offset_mm,
            0.0,
            approach_z_up_mm,
            settling_time_sec,
            tf_only_mode,
            tool_offset_x_mm,
            tool_offset_y_mm,
            tool_offset_z_mm,
            tool_offset_rx_deg,
            tool_offset_ry_deg,
            tool_offset_rz_deg,
        )

    def _is_cancel_requested(self) -> bool:
        with self._lock:
            return bool(self._cancel_requested)

    def is_manual_release_inflight(self) -> bool:
        with self._lock:
            return bool(self._manual_release_inflight)

    def is_manual_stop_inflight(self) -> bool:
        with self._lock:
            return bool(self._manual_stop_inflight)

    def is_goal_tf_diagnose_inflight(self) -> bool:
        with self._lock:
            return bool(self._goal_tf_diagnose_inflight)

    def request_publish_tool_offset_preview(
        self,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> bool:
        if not self._publish_goal_debug_tf:
            self._set_action_text('TF publishing is disabled (publish_goal_debug_tf=false).')
            return False

        try:
            tx_mm = self._clamp_tool_offset_translation_mm(tool_offset_x_mm)
            ty_mm = self._clamp_tool_offset_translation_mm(tool_offset_y_mm)
            tz_mm = self._clamp_tool_offset_translation_mm(tool_offset_z_mm)
            rx_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rx_deg)
            ry_deg = self._clamp_tool_offset_rotation_deg(tool_offset_ry_deg)
            rz_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rz_deg)

            with self._lock:
                self._tool_offset_x_mm = tx_mm
                self._tool_offset_y_mm = ty_mm
                self._tool_offset_z_mm = tz_mm
                self._tool_offset_rx_deg = rx_deg
                self._tool_offset_ry_deg = ry_deg
                self._tool_offset_rz_deg = rz_deg

            self._publish_goal_debug_transform(
                self._tool_offset_preview_parent_frame_id,
                self._tool_offset_preview_frame_id,
                tx_mm,
                ty_mm,
                tz_mm,
                rx_deg,
                ry_deg,
                rz_deg,
            )
            self._publish_goal_debug_transform(
                self._tool_offset_preview_frame_id,
                self._tool_offset_preview_axis_x_tip_frame_id,
                GOAL_TF_DIAG_AXIS_LENGTH_MM,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._publish_goal_debug_transform(
                self._tool_offset_preview_frame_id,
                self._tool_offset_preview_axis_y_tip_frame_id,
                0.0,
                GOAL_TF_DIAG_AXIS_LENGTH_MM,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._publish_goal_debug_transform(
                self._tool_offset_preview_frame_id,
                self._tool_offset_preview_axis_z_tip_frame_id,
                0.0,
                0.0,
                GOAL_TF_DIAG_AXIS_LENGTH_MM,
                0.0,
                0.0,
                0.0,
            )
            self._set_action_text(
                'Published tool-offset TF preview in RViz: '
                f'"{self._tool_offset_preview_parent_frame_id}" -> "{self._tool_offset_preview_frame_id}".'
            )
            return True
        except Exception as exc:
            self._set_action_text(f'Failed to publish tool-offset TF preview: {exc}')
            return False

    def request_goal_tf_diagnose(
        self,
        post_stop_z_offset_mm: float,
        approach_z_up_mm: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> bool:
        with self._lock:
            if self._goal_tf_diagnose_inflight:
                self._snapshot.action_text = 'TF diagnose already in progress.'
                return False
            item_target = self._last_item_target
            if item_target is None:
                self._snapshot.action_text = 'No item pose received yet. Publish item pose, then retry TF diagnose.'
                return False
            self._goal_tf_diagnose_inflight = True
            self._snapshot.action_text = (
                f'TF diagnose started: computing goal and publishing "{self._post_stop_movel_goal_debug_frame_id}"...'
            )

        worker = threading.Thread(
            target=self._goal_tf_diagnose_worker,
            args=(
                item_target,
                post_stop_z_offset_mm,
                approach_z_up_mm,
                tool_offset_x_mm,
                tool_offset_y_mm,
                tool_offset_z_mm,
                tool_offset_rx_deg,
                tool_offset_ry_deg,
                tool_offset_rz_deg,
            ),
            daemon=True,
        )
        worker.start()
        return True

    def _goal_tf_diagnose_worker(
        self,
        item_target: ItemPoseTarget,
        post_stop_z_offset_mm: float,
        approach_z_up_mm: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> None:
        try:
            z_offset_mm = max(
                POST_STOP_Z_OFFSET_MIN,
                min(POST_STOP_Z_OFFSET_MAX, float(post_stop_z_offset_mm)),
            )
            z_up_mm = max(
                APPROACH_Z_UP_MIN,
                min(APPROACH_Z_UP_MAX, float(approach_z_up_mm)),
            )
            tx_mm = self._clamp_tool_offset_translation_mm(tool_offset_x_mm)
            ty_mm = self._clamp_tool_offset_translation_mm(tool_offset_y_mm)
            tz_mm = self._clamp_tool_offset_translation_mm(tool_offset_z_mm)
            rx_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rx_deg)
            ry_deg = self._clamp_tool_offset_rotation_deg(tool_offset_ry_deg)
            rz_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rz_deg)

            self._set_action_text('TF diagnose: computing goal from latest item pose...')
            base_goal = self._compute_base_goal_from_item_target(
                item_target,
                0.0,
                0.0,
                z_offset_mm,
                tx_mm,
                ty_mm,
                tz_mm,
                rx_deg,
                ry_deg,
                rz_deg,
                self._post_stop_movel_speed_mm_s,
                predict_target_motion=False,
            )
            if base_goal is None:
                return

            approach_goal = (
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm + z_up_mm,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_primary_goal_debug_transform(approach_goal)
            self._set_action_text(
                'TF diagnose published. RViz TF frame: '
                f'"{self._post_stop_movel_goal_debug_frame_id}".'
            )
        finally:
            with self._lock:
                self._goal_tf_diagnose_inflight = False

    def request_release_pulse(self) -> bool:
        with self._lock:
            if self._manual_release_inflight:
                self._snapshot.action_text = 'Release pulse already in progress.'
                return False
            if self._snapshot.busy:
                self._snapshot.action_text = 'Cannot run release pulse while item pick sequence is active.'
                return False
            self._manual_release_inflight = True
            self._snapshot.action_text = (
                f'Release pulse started: DO1 OFF + DO3 OFF (vent) + DO2 pulse {int(MANUAL_RELEASE_PULSE_MS)} ms.'
            )

        worker = threading.Thread(target=self._manual_release_pulse_worker, daemon=True)
        worker.start()
        return True

    def _manual_release_pulse_worker(self) -> None:
        try:
            if not self._send_do(1, 0):
                return
            if not self._send_do(3, 0):
                return
            if not self._send_do(2, 1):
                return
            pulse_sec = float(MANUAL_RELEASE_PULSE_MS) * 0.001
            wait_started = time.monotonic()
            while (time.monotonic() - wait_started) < pulse_sec:
                time.sleep(0.01)
            if not self._send_do(2, 0):
                return
            self._set_action_text('Release pulse complete: neutral state (DO1 OFF, DO2 OFF, DO3 OFF/vent).')
        finally:
            with self._lock:
                self._manual_release_inflight = False

    def request_manual_stop(self) -> bool:
        with self._lock:
            if self._manual_stop_inflight:
                self._snapshot.action_text = 'Manual Stop already in progress.'
                return False
            self._manual_stop_inflight = True
            self._cancel_requested = True
            self._item_pose_watch_generation += 1
            self._item_pose_watch_armed = False
            self._item_pose_watch_stop_dispatched = False
            self._item_pose_watch_deadline_monotonic = 0.0
            self._snapshot.busy = False
            self._snapshot.action_text = 'Manual Stop requested. Sending robot Stop...'

        worker = threading.Thread(target=self._manual_stop_worker, daemon=True)
        worker.start()
        return True

    def _manual_stop_worker(self) -> None:
        try:
            if not self._wait_for_service(self._stop_client, 'Stop'):
                return
            stop_response = self._call_service(self._stop_client, Stop.Request(), 'Stop() [manual]')
            if stop_response is None:
                return
            if int(getattr(stop_response, 'res', -1)) < 0:
                return
            self._set_action_text('Manual Stop sent. Sequence halted.')
        finally:
            self._notify_item_detect_seek_complete()
            with self._lock:
                self._snapshot.busy = False
                self._manual_stop_inflight = False

    def _build_motion_param_value(self, v_percent: int, a_percent: int, include_tool: bool = True) -> list[str]:
        args = [f'v={int(v_percent)}', f'a={int(a_percent)}']
        if include_tool:
            args.append('tool=1')
        return [','.join(args)]

    def get_command_hysteresis_sec(self) -> float:
        with self._lock:
            return float(self._command_hysteresis_sec)

    def get_settling_time_sec(self) -> float:
        with self._lock:
            return float(self._settling_time_sec)

    def set_command_hysteresis_sec(self, command_hysteresis_sec: float) -> float:
        with self._lock:
            self._command_hysteresis_sec = max(
                COMMAND_HYSTERESIS_MIN_SEC,
                min(COMMAND_HYSTERESIS_MAX_SEC, float(command_hysteresis_sec)),
            )
            return float(self._command_hysteresis_sec)

    def run_item_sequence(
        self,
        item_pose_watch_timeout_sec: float,
        post_stop_movel_speed_mm_s: float,
        post_stop_x_offset_mm: float,
        post_stop_y_offset_mm: float,
        post_stop_z_offset_mm: float,
        follow_distance_mm: float,
        approach_z_up_mm: float,
        settling_time_sec: float,
        tf_only_mode: bool,
        tool_offset_x_mm: float | None = None,
        tool_offset_y_mm: float | None = None,
        tool_offset_z_mm: float | None = None,
        tool_offset_rx_deg: float | None = None,
        tool_offset_ry_deg: float | None = None,
        tool_offset_rz_deg: float | None = None,
    ) -> bool:
        _ = (post_stop_movel_speed_mm_s, follow_distance_mm)
        active_profile_key, saved_offsets = self._sync_profile_tool_offsets_from_state(force=False)
        if active_profile_key is None:
            self._set_action_text(
                'No active item teach selected in item_detect. Load an item teach profile first.'
            )
            return False
        if saved_offsets is None:
            self._set_action_text(
                'No saved tool teach for '
                f'"{self._profile_display_name(active_profile_key)}". Save it in item pick first.'
            )
            return False
        saved_post_stop_z_offset_mm = float(saved_offsets['item_standoff_z_mm'])
        saved_approach_z_up_mm = float(saved_offsets['approach_z_up_mm'])
        saved_final_z_up_mm = float(saved_offsets['final_z_up_mm'])
        saved_settling_time_sec = float(saved_offsets['settling_time_sec'])
        saved_tool_offset_x_mm = float(saved_offsets['tool_offset_x_mm'])
        saved_tool_offset_y_mm = float(saved_offsets['tool_offset_y_mm'])
        saved_tool_offset_z_mm = float(saved_offsets['tool_offset_z_mm'])
        saved_tool_offset_rx_deg = float(saved_offsets['tool_offset_rx_deg'])
        saved_tool_offset_ry_deg = float(saved_offsets['tool_offset_ry_deg'])
        saved_tool_offset_rz_deg = float(saved_offsets['tool_offset_rz_deg'])
        if self.count_publishers(ITEM_POSE_TOPIC) <= 0:
            self._reset_runtime_state(
                f'No item pose publisher on "{ITEM_POSE_TOPIC}". Node reset.'
            )
            return False

        with self._lock:
            if self._snapshot.busy:
                self._snapshot.action_text = 'Busy running previous item pick sequence.'
                return False
            self._snapshot.busy = True
            self._cancel_requested = False
            self._item_pose_watch_timeout_sec = max(
                ITEM_POSE_WATCH_TIMEOUT_MIN,
                min(ITEM_POSE_WATCH_TIMEOUT_MAX, float(item_pose_watch_timeout_sec)),
            )
            self._post_stop_movel_speed_mm_s = LOCKED_MAX_SPEED_MM_S
            # Always pick on the detected item location (no XY operator offset).
            self._post_stop_x_offset_mm = 0.0
            self._post_stop_y_offset_mm = 0.0
            self._post_stop_z_offset_mm = saved_post_stop_z_offset_mm
            self._approach_z_up_mm = saved_approach_z_up_mm
            self._final_z_up_mm = saved_final_z_up_mm
            self._settling_time_sec = saved_settling_time_sec
            self._tool_offset_x_mm = saved_tool_offset_x_mm
            self._tool_offset_y_mm = saved_tool_offset_y_mm
            self._tool_offset_z_mm = saved_tool_offset_z_mm
            self._tool_offset_rx_deg = saved_tool_offset_rx_deg
            self._tool_offset_ry_deg = saved_tool_offset_ry_deg
            self._tool_offset_rz_deg = saved_tool_offset_rz_deg
            self._item_pose_watch_tf_only_mode = bool(tf_only_mode)
            generation = self._arm_item_pose_watch_locked()
            watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
            mode_name = 'tf_only' if tf_only_mode else 'normal'
            self.get_logger().info(
                'Run settings: '
                f'mode={mode_name} '
                f'wait={watch_timeout_sec:.0f}s '
                f'speed={self._post_stop_movel_speed_mm_s:.0f} mm/s '
                f'offsets(x=0,y=0,z={self._post_stop_z_offset_mm:.0f}) mm '
                f'tool_offset(x={self._tool_offset_x_mm:.1f},y={self._tool_offset_y_mm:.1f},'
                f'z={self._tool_offset_z_mm:.1f},rx={self._tool_offset_rx_deg:.1f},'
                f'ry={self._tool_offset_ry_deg:.1f},rz={self._tool_offset_rz_deg:.1f}) '
                f'approach_z={self._approach_z_up_mm:.0f} mm '
                f'final_z_up={self._final_z_up_mm:.0f} mm '
                f'settling={self._settling_time_sec:.1f}s'
            )
            if tf_only_mode:
                self._snapshot.action_text = (
                    f'Troubleshoot mode armed... waiting for "{ITEM_POSE_TOPIC}" '
                    f'for {watch_timeout_sec:.0f}s (TF preview only).'
                )
            else:
                self._snapshot.action_text = (
                    f'Item pick sequence armed... waiting for fresh "{ITEM_POSE_TOPIC}" '
                    f'for {watch_timeout_sec:.0f}s.'
                )

        watchdog = threading.Thread(
            target=self._item_pose_watchdog_worker,
            args=(generation,),
            daemon=True,
        )
        watchdog.start()
        return True

    def _wait_for_service(self, client, label: str, timeout_sec: float = 10.0) -> bool:
        started = time.time()
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=0.3):
                return True
            if (time.time() - started) >= timeout_sec:
                break
        self._set_action_text(f'{label} service not ready.')
        return False

    def _call_service(self, client, request, label: str, timeout_sec: float = 8.0):
        self._set_action_text(f'SEND {label}')
        future = client.call_async(request)
        started = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - started) >= timeout_sec:
                self._set_action_text(f'Timeout: {label}')
                return None
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            self._set_action_text(f'Exception: {label}: {exception}')
            return None

        response = future.result()
        if response is None:
            self._set_action_text(f'No response: {label}')
            return None

        res = int(getattr(response, 'res', -1))
        robot_return = str(getattr(response, 'robot_return', '')).strip()
        if res < 0:
            if robot_return:
                self._set_action_text(f'FAIL {label}: res={res}, return={robot_return}')
            else:
                self._set_action_text(f'FAIL {label}: res={res}')
            return response

        if robot_return:
            self._set_action_text(f'OK {label}: {robot_return}')
        else:
            self._set_action_text(f'OK {label}')
        return response

class ItemPickGui:
    def __init__(self, node: ItemPickNode) -> None:
        self.node = node
        self._gui_thread_id = threading.get_ident()
        self.root = tk.Tk()
        self.root.title('Item Pick Operator Console')
        fixed_width = 960
        fixed_height = 520
        self.root.geometry(f'{fixed_width}x{fixed_height}')
        self.root.minsize(fixed_width, fixed_height)
        self.root.maxsize(fixed_width, fixed_height)
        self.root.resizable(False, False)
        self._closed = False
        self._runtime_settings_path = RUNTIME_SETTINGS_PATH
        self._runtime_settings_save_after_id: str | None = None
        self._suspend_runtime_settings_events = False
        self._active_item_profile_key: str | None = None
        self._active_profile_has_saved_tool_teach = False
        self._saved_tool_teach_values: tuple[
            float, float, float, float, float, float, float, float, float, float
        ] | None = None

        outer = tk.Frame(self.root, padx=12, pady=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1, uniform='maincols')
        outer.columnconfigure(1, weight=1, uniform='maincols')
        outer.rowconfigure(0, weight=1)

        modes_frame = tk.LabelFrame(outer, text='Operating Modes', padx=10, pady=8)
        modes_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        modes_frame.columnconfigure(0, weight=1)
        slider_length = 250

        self.run_button = tk.Button(
            modes_frame,
            text='Arm Track Item',
            command=self._run_clicked,
            width=20,
        )
        self.run_button.grid(row=0, column=0, sticky='ew')

        self.stop_button = tk.Button(
            modes_frame,
            text='Stop',
            command=self._stop_clicked,
            width=20,
        )
        self.stop_button.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        self._stop_default_bg = self.stop_button.cget('bg')
        self._stop_default_fg = self.stop_button.cget('fg')
        self._stop_default_active_bg = self.stop_button.cget('activebackground')
        self._stop_default_active_fg = self.stop_button.cget('activeforeground')
        self._set_stop_button_enabled(False)

        self.release_button = tk.Button(
            modes_frame,
            text=f'Release {int(MANUAL_RELEASE_PULSE_MS)}ms',
            command=self._release_clicked,
            width=20,
            bg='#1565c0',
            fg='white',
            activebackground='#0d47a1',
            activeforeground='white',
        )
        self.release_button.grid(row=2, column=0, sticky='ew', pady=(8, 0))

        self.tf_only_var = tk.BooleanVar(value=True)
        self.tf_only_button = tk.Button(
            modes_frame,
            command=self._toggle_tf_only_clicked,
            width=24,
        )
        self.tf_only_button.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        self._tf_only_default_bg = self.tf_only_button.cget('bg')
        self._tf_only_default_fg = self.tf_only_button.cget('fg')
        self._tf_only_default_active_bg = self.tf_only_button.cget('activebackground')
        self._tf_only_default_active_fg = self.tf_only_button.cget('activeforeground')
        self._sync_tf_only_button(is_busy=False)

        tk.Label(modes_frame, text='Item pose wait timeout (sec)').grid(row=4, column=0, sticky='w', pady=(10, 0))
        self.item_pose_watch_timeout_var = tk.DoubleVar(value=ITEM_POSE_WATCH_TIMEOUT_SEC)
        self.item_pose_watch_timeout_scale = tk.Scale(
            modes_frame,
            from_=ITEM_POSE_WATCH_TIMEOUT_MIN,
            to=ITEM_POSE_WATCH_TIMEOUT_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.item_pose_watch_timeout_var,
            showvalue=True,
        )
        self.item_pose_watch_timeout_scale.grid(row=5, column=0, sticky='ew')

        mode_hint = (
            'Press Arm Track Item, then wait for item pose. '
            'Normal mode sends approach/descent, settles, retracts, settles, then final Z-up.'
        )
        tk.Label(
            modes_frame,
            text=mode_hint,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=6, column=0, sticky='w', pady=(8, 0))
        self.action_var = tk.StringVar(value='Ready')
        tk.Label(
            modes_frame,
            textvariable=self.action_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=7, column=0, sticky='ew', pady=(8, 0))

        ee_settings_frame = tk.LabelFrame(outer, text='EE Position Settings', padx=10, pady=8)
        ee_settings_frame.grid(row=0, column=1, sticky='nsew')
        ee_settings_frame.columnconfigure(0, weight=1)

        tk.Label(ee_settings_frame, text='Item stand-off (+item Z, mm)').grid(row=0, column=0, sticky='w')
        self.post_stop_z_offset_var = tk.DoubleVar(value=100.0)
        self.post_stop_z_offset_scale = tk.Scale(
            ee_settings_frame,
            from_=POST_STOP_Z_OFFSET_MIN,
            to=POST_STOP_Z_OFFSET_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.post_stop_z_offset_var,
            showvalue=True,
        )
        self.post_stop_z_offset_scale.grid(row=1, column=0, sticky='ew')

        tk.Label(ee_settings_frame, text='Approach Z (mm)').grid(row=2, column=0, sticky='w', pady=(10, 0))
        self.approach_z_up_var = tk.DoubleVar(value=APPROACH_Z_UP_DEFAULT)
        self.approach_z_up_scale = tk.Scale(
            ee_settings_frame,
            from_=APPROACH_Z_UP_MIN,
            to=APPROACH_Z_UP_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.approach_z_up_var,
            showvalue=True,
        )
        self.approach_z_up_scale.grid(row=3, column=0, sticky='ew')

        tk.Label(ee_settings_frame, text='Final Z-up (mm)').grid(row=4, column=0, sticky='w', pady=(10, 0))
        self.final_z_up_var = tk.DoubleVar(value=FINAL_Z_UP_DEFAULT)
        self.final_z_up_scale = tk.Scale(
            ee_settings_frame,
            from_=FINAL_Z_UP_MIN,
            to=FINAL_Z_UP_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.final_z_up_var,
            showvalue=True,
        )
        self.final_z_up_scale.grid(row=5, column=0, sticky='ew')

        tk.Label(ee_settings_frame, text='Item Pick Settling Time (sec)').grid(row=6, column=0, sticky='w', pady=(10, 0))
        self.settling_time_var = tk.DoubleVar(value=SETTLING_TIME_DEFAULT_SEC)
        self.settling_time_scale = tk.Scale(
            ee_settings_frame,
            from_=SETTLING_TIME_MIN_SEC,
            to=SETTLING_TIME_MAX_SEC,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=slider_length,
            variable=self.settling_time_var,
            showvalue=True,
        )
        self.settling_time_scale.grid(row=7, column=0, sticky='ew')

        tk.Label(
            ee_settings_frame,
            text='Tool offset (x/y/z mm, rx/ry/rz deg) | Use button to preview TF wrt Link6 in RViz',
        ).grid(row=8, column=0, sticky='w', pady=(10, 0))
        tool_offset_frame = tk.Frame(ee_settings_frame)
        tool_offset_frame.grid(row=9, column=0, sticky='ew')
        for col in range(3):
            tool_offset_frame.columnconfigure(col, weight=1, uniform='tool_offset_col')
        self.active_profile_var = tk.StringVar(value='Active item teach: waiting for item_detect...')
        tk.Label(
            tool_offset_frame,
            textvariable=self.active_profile_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=0, column=0, columnspan=3, sticky='ew', pady=(0, 6))

        self.tool_offset_x_var = tk.DoubleVar(value=0.0)
        self.tool_offset_y_var = tk.DoubleVar(value=0.0)
        self.tool_offset_z_var = tk.DoubleVar(value=0.0)
        self.tool_offset_rx_var = tk.DoubleVar(value=0.0)
        self.tool_offset_ry_var = tk.DoubleVar(value=0.0)
        self.tool_offset_rz_var = tk.DoubleVar(value=0.0)

        tk.Label(tool_offset_frame, text='X').grid(row=1, column=0, sticky='w')
        tk.Label(tool_offset_frame, text='Y').grid(row=1, column=1, sticky='w')
        tk.Label(tool_offset_frame, text='Z').grid(row=1, column=2, sticky='w')
        tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_TRANSLATION_MIN_MM,
            to=TOOL_OFFSET_TRANSLATION_MAX_MM,
            increment=1.0,
            textvariable=self.tool_offset_x_var,
            width=9,
            format='%.1f',
        ).grid(row=2, column=0, sticky='ew', padx=(0, 4))
        tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_TRANSLATION_MIN_MM,
            to=TOOL_OFFSET_TRANSLATION_MAX_MM,
            increment=1.0,
            textvariable=self.tool_offset_y_var,
            width=9,
            format='%.1f',
        ).grid(row=2, column=1, sticky='ew', padx=(0, 4))
        tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_TRANSLATION_MIN_MM,
            to=TOOL_OFFSET_TRANSLATION_MAX_MM,
            increment=1.0,
            textvariable=self.tool_offset_z_var,
            width=9,
            format='%.1f',
        ).grid(row=2, column=2, sticky='ew')

        tk.Label(tool_offset_frame, text='Rx').grid(row=3, column=0, sticky='w', pady=(8, 0))
        tk.Label(tool_offset_frame, text='Ry').grid(row=3, column=1, sticky='w', pady=(8, 0))
        tk.Label(tool_offset_frame, text='Rz').grid(row=3, column=2, sticky='w', pady=(8, 0))
        tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_ROTATION_MIN_DEG,
            to=TOOL_OFFSET_ROTATION_MAX_DEG,
            increment=1.0,
            textvariable=self.tool_offset_rx_var,
            width=9,
            format='%.1f',
        ).grid(row=4, column=0, sticky='ew', padx=(0, 4))
        tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_ROTATION_MIN_DEG,
            to=TOOL_OFFSET_ROTATION_MAX_DEG,
            increment=1.0,
            textvariable=self.tool_offset_ry_var,
            width=9,
            format='%.1f',
        ).grid(row=4, column=1, sticky='ew', padx=(0, 4))
        tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_ROTATION_MIN_DEG,
            to=TOOL_OFFSET_ROTATION_MAX_DEG,
            increment=1.0,
            textvariable=self.tool_offset_rz_var,
            width=9,
            format='%.1f',
        ).grid(row=4, column=2, sticky='ew')

        tool_offset_button_frame = tk.Frame(tool_offset_frame)
        tool_offset_button_frame.grid(row=5, column=0, columnspan=3, sticky='ew', pady=(10, 0))
        tool_offset_button_frame.columnconfigure(0, weight=1)
        tool_offset_button_frame.columnconfigure(1, weight=1)
        self.save_tool_offset_button = tk.Button(
            tool_offset_button_frame,
            text='Save Tool Teach',
            command=self._save_tool_offset_profile_clicked,
            width=24,
            bg='#6a1b9a',
            fg='white',
            activebackground='#4a148c',
            activeforeground='white',
        )
        self.save_tool_offset_button.grid(row=0, column=0, sticky='ew', padx=(0, 4))
        self.show_tool_tf_button = tk.Button(
            tool_offset_button_frame,
            text='Show Tool TF (Link6 Ref)',
            command=self._show_tool_tf_preview_clicked,
            width=24,
            bg='#2e7d32',
            fg='white',
            activebackground='#1b5e20',
            activeforeground='white',
        )
        self.show_tool_tf_button.grid(row=0, column=1, sticky='ew', padx=(4, 0))
        self.tool_offset_profile_status_var = tk.StringVar(
            value='Tool teach must be saved for the active item teach before arming.'
        )
        tk.Label(
            tool_offset_frame,
            textvariable=self.tool_offset_profile_status_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=6, column=0, columnspan=3, sticky='ew', pady=(8, 0))

        self._register_runtime_setting_traces()
        self._load_runtime_settings()
        self._sync_profile_tool_offsets_from_state(force=True)
        self._sync_tf_only_button(is_busy=False)
        self.node.set_track_trigger_handler(self._track_clicked_from_service)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._refresh()

    def _track_clicked_from_service(self) -> tuple[bool, str]:
        if self._closed:
            return False, 'Item pick GUI is closed.'

        if threading.get_ident() == self._gui_thread_id:
            started = bool(self._run_clicked())
            snapshot = self.node.snapshot()
            return started, str(snapshot.action_text)

        done = threading.Event()
        result: dict[str, object] = {
            'started': False,
            'message': 'Track virtual-click did not run.',
        }

        def run_on_gui() -> None:
            try:
                started = bool(self._run_clicked())
                snapshot = self.node.snapshot()
                result['started'] = started
                result['message'] = str(snapshot.action_text)
            except Exception as exc:
                message = f'Track virtual-click failed: {exc}'
                self.node._set_action_text(message)
                self.action_var.set(message)
                result['started'] = False
                result['message'] = message
            finally:
                done.set()

        try:
            self.root.after(0, run_on_gui)
        except Exception as exc:
            return False, f'Track virtual-click could not reach GUI thread: {exc}'

        if not done.wait(timeout=5.0):
            return False, 'Track virtual-click timed out waiting for GUI thread.'

        return bool(result['started']), str(result['message'])

    def _run_clicked(self) -> bool:
        self._sync_profile_tool_offsets_from_state()
        if not self._can_arm_sequence():
            reason = self._arm_block_reason()
            self.node._set_action_text(reason)
            self.action_var.set(reason)
            return False
        item_pose_watch_timeout_value = float(self.item_pose_watch_timeout_var.get())
        post_stop_z_offset_value = float(self.post_stop_z_offset_var.get())
        approach_z_up_value = float(self.approach_z_up_var.get())
        settling_time_value = float(self.settling_time_var.get())
        tool_offset_x_value = float(self.tool_offset_x_var.get())
        tool_offset_y_value = float(self.tool_offset_y_var.get())
        tool_offset_z_value = float(self.tool_offset_z_var.get())
        tool_offset_rx_value = float(self.tool_offset_rx_var.get())
        tool_offset_ry_value = float(self.tool_offset_ry_var.get())
        tool_offset_rz_value = float(self.tool_offset_rz_var.get())
        tf_only_mode = bool(self.tf_only_var.get())

        started = self.node.run_item_sequence(
            item_pose_watch_timeout_value,
            LOCKED_MAX_SPEED_MM_S,
            0.0,
            0.0,
            post_stop_z_offset_value,
            0.0,
            approach_z_up_value,
            settling_time_value,
            tf_only_mode,
            tool_offset_x_value,
            tool_offset_y_value,
            tool_offset_z_value,
            tool_offset_rx_value,
            tool_offset_ry_value,
            tool_offset_rz_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)
            return False
        else:
            self._set_stop_button_enabled(True)
            return True

    def _stop_clicked(self) -> None:
        started = self.node.request_manual_stop()
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _release_clicked(self) -> None:
        started = self.node.request_release_pulse()
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _toggle_tf_only_clicked(self) -> None:
        current = bool(self.tf_only_var.get())
        self.tf_only_var.set(not current)
        self._sync_tf_only_button(is_busy=False)

    def _show_tool_tf_preview_clicked(self) -> None:
        tool_offset_x_value = float(self.tool_offset_x_var.get())
        tool_offset_y_value = float(self.tool_offset_y_var.get())
        tool_offset_z_value = float(self.tool_offset_z_var.get())
        tool_offset_rx_value = float(self.tool_offset_rx_var.get())
        tool_offset_ry_value = float(self.tool_offset_ry_var.get())
        tool_offset_rz_value = float(self.tool_offset_rz_var.get())

        started = self.node.request_publish_tool_offset_preview(
            tool_offset_x_value,
            tool_offset_y_value,
            tool_offset_z_value,
            tool_offset_rx_value,
            tool_offset_ry_value,
            tool_offset_rz_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _save_tool_offset_profile_clicked(self) -> None:
        saved = self._save_profile_tool_offsets_for_active_item()
        if saved:
            tool_teach_name = display_name_for_tool_teach_profile(self._active_item_profile_key)
            message = f'Saved tool teach "{tool_teach_name}".'
            self.node._set_action_text(message)
            self.action_var.set(message)

    def _goal_tf_diagnose_clicked(self) -> None:
        post_stop_z_offset_value = float(self.post_stop_z_offset_var.get())
        approach_z_up_value = float(self.approach_z_up_var.get())
        tool_offset_x_value = float(self.tool_offset_x_var.get())
        tool_offset_y_value = float(self.tool_offset_y_var.get())
        tool_offset_z_value = float(self.tool_offset_z_var.get())
        tool_offset_rx_value = float(self.tool_offset_rx_var.get())
        tool_offset_ry_value = float(self.tool_offset_ry_var.get())
        tool_offset_rz_value = float(self.tool_offset_rz_var.get())

        started = self.node.request_goal_tf_diagnose(
            post_stop_z_offset_value,
            approach_z_up_value,
            tool_offset_x_value,
            tool_offset_y_value,
            tool_offset_z_value,
            tool_offset_rx_value,
            tool_offset_ry_value,
            tool_offset_rz_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))

    @staticmethod
    def _profile_display_name(profile_key: str | None) -> str:
        return display_name_for_item_teach_profile(profile_key)

    @staticmethod
    def _tool_offset_signature_from_profile(
        profile_offsets: dict[str, float] | None,
    ) -> tuple[float, float, float, float, float, float, float, float, float, float] | None:
        if profile_offsets is None:
            return None
        return (
            round(float(profile_offsets.get('item_standoff_z_mm', 0.0)), 4),
            round(float(profile_offsets.get('approach_z_up_mm', 0.0)), 4),
            round(float(profile_offsets.get('final_z_up_mm', FINAL_Z_UP_DEFAULT)), 4),
            round(float(profile_offsets.get('settling_time_sec', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_x_mm', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_y_mm', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_z_mm', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_rx_deg', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_ry_deg', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_rz_deg', 0.0)), 4),
        )

    def _current_tool_offset_signature(self) -> tuple[float, float, float, float, float, float, float, float, float, float]:
        return (
            round(float(self.post_stop_z_offset_var.get()), 4),
            round(float(self.approach_z_up_var.get()), 4),
            round(float(self.final_z_up_var.get()), 4),
            round(float(self.settling_time_var.get()), 4),
            round(float(self.tool_offset_x_var.get()), 4),
            round(float(self.tool_offset_y_var.get()), 4),
            round(float(self.tool_offset_z_var.get()), 4),
            round(float(self.tool_offset_rx_var.get()), 4),
            round(float(self.tool_offset_ry_var.get()), 4),
            round(float(self.tool_offset_rz_var.get()), 4),
        )

    def _has_unsaved_tool_offset_changes(self) -> bool:
        if self._saved_tool_teach_values is None:
            return True
        return self._current_tool_offset_signature() != self._saved_tool_teach_values

    def _arm_block_reason(self) -> str:
        if self._active_item_profile_key is None:
            return 'No active item teach selected in item_detect. Load an item teach profile first.'
        if not self._active_profile_has_saved_tool_teach or self._saved_tool_teach_values is None:
            return (
                'No saved tool teach for '
                f'"{self._profile_display_name(self._active_item_profile_key)}". Save it before arming.'
            )
        if self._has_unsaved_tool_offset_changes():
            return (
                'Tool teach changes for '
                f'"{self._profile_display_name(self._active_item_profile_key)}" are not saved. '
                'Save them before arming.'
            )
        return ''

    def _can_arm_sequence(self) -> bool:
        return not bool(self._arm_block_reason())

    def _update_profile_tool_offset_status(self) -> None:
        profile_name = self._profile_display_name(self._active_item_profile_key)
        self.active_profile_var.set(f'Active item teach: {profile_name}')
        if self._active_item_profile_key is None:
            self.tool_offset_profile_status_var.set(
                'Waiting for item_detect active teach selection.'
            )
            return
        if not self._active_profile_has_saved_tool_teach or self._saved_tool_teach_values is None:
            self.tool_offset_profile_status_var.set(
                'No saved tool teach for this item teach. Current UI values are unsaved.'
            )
            return
        if self._has_unsaved_tool_offset_changes():
            self.tool_offset_profile_status_var.set(
                'Saved tool teach loaded, but current UI values have unsaved changes.'
            )
            return
        self.tool_offset_profile_status_var.set(
            'Saved tool teach loaded and ready for arming.'
        )

    def _register_runtime_setting_traces(self) -> None:
        tracked_vars = [
            self.tf_only_var,
            self.item_pose_watch_timeout_var,
            self.post_stop_z_offset_var,
            self.approach_z_up_var,
            self.final_z_up_var,
            self.settling_time_var,
            self.tool_offset_x_var,
            self.tool_offset_y_var,
            self.tool_offset_z_var,
            self.tool_offset_rx_var,
            self.tool_offset_ry_var,
            self.tool_offset_rz_var,
        ]
        for var in tracked_vars:
            var.trace_add('write', self._on_runtime_setting_changed)

    def _sync_profile_tool_offsets_from_state(self, force: bool = False) -> None:
        active_profile_key, profile_offsets = self.node.get_active_profile_tool_offset_state(force=force)
        profile_signature = self._tool_offset_signature_from_profile(profile_offsets)
        profile_changed = active_profile_key != self._active_item_profile_key
        saved_changed = profile_signature != self._saved_tool_teach_values
        self._active_item_profile_key = active_profile_key
        self._active_profile_has_saved_tool_teach = profile_offsets is not None
        if profile_offsets is not None and (force or profile_changed or saved_changed):
            self._suspend_runtime_settings_events = True
            try:
                self.post_stop_z_offset_var.set(self._clamp(
                    profile_offsets.get('item_standoff_z_mm', self.post_stop_z_offset_var.get()),
                    POST_STOP_Z_OFFSET_MIN,
                    POST_STOP_Z_OFFSET_MAX,
                ))
                self.approach_z_up_var.set(self._clamp(
                    profile_offsets.get('approach_z_up_mm', self.approach_z_up_var.get()),
                    APPROACH_Z_UP_MIN,
                    APPROACH_Z_UP_MAX,
                ))
                self.final_z_up_var.set(self._clamp(
                    profile_offsets.get('final_z_up_mm', self.final_z_up_var.get()),
                    FINAL_Z_UP_MIN,
                    FINAL_Z_UP_MAX,
                ))
                self.settling_time_var.set(self._clamp(
                    profile_offsets.get('settling_time_sec', self.settling_time_var.get()),
                    SETTLING_TIME_MIN_SEC,
                    SETTLING_TIME_MAX_SEC,
                ))
                self.tool_offset_x_var.set(self._clamp(
                    profile_offsets.get('tool_offset_x_mm', self.tool_offset_x_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.tool_offset_y_var.set(self._clamp(
                    profile_offsets.get('tool_offset_y_mm', self.tool_offset_y_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.tool_offset_z_var.set(self._clamp(
                    profile_offsets.get('tool_offset_z_mm', self.tool_offset_z_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.tool_offset_rx_var.set(self._clamp(
                    profile_offsets.get('tool_offset_rx_deg', self.tool_offset_rx_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.tool_offset_ry_var.set(self._clamp(
                    profile_offsets.get('tool_offset_ry_deg', self.tool_offset_ry_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.tool_offset_rz_var.set(self._clamp(
                    profile_offsets.get('tool_offset_rz_deg', self.tool_offset_rz_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
            finally:
                self._suspend_runtime_settings_events = False
        self._saved_tool_teach_values = profile_signature
        self._update_profile_tool_offset_status()

    def _save_profile_tool_offsets_for_active_item(self) -> bool:
        active_profile_key = self._active_item_profile_key
        if active_profile_key is None:
            message = 'No active item teach selected in item_detect. Load an item teach profile first.'
            self.node._set_action_text(message)
            self.action_var.set(message)
            return False

        tool_teach_path = tool_teach_path_for_profile(active_profile_key)
        payload: dict[str, object] = {
            'tool_teach_version': 2,
            'item_teach_name': item_teach_name_for_profile(active_profile_key),
            'item_detect_profile_path': active_profile_key,
            'item_standoff_z_mm': float(self.post_stop_z_offset_var.get()),
            'approach_z_up_mm': float(self.approach_z_up_var.get()),
            'final_z_up_mm': float(self.final_z_up_var.get()),
            'settling_time_sec': float(self.settling_time_var.get()),
            'tool_offset_x_mm': float(self.tool_offset_x_var.get()),
            'tool_offset_y_mm': float(self.tool_offset_y_var.get()),
            'tool_offset_z_mm': float(self.tool_offset_z_var.get()),
            'tool_offset_rx_deg': float(self.tool_offset_rx_var.get()),
            'tool_offset_ry_deg': float(self.tool_offset_ry_var.get()),
            'tool_offset_rz_deg': float(self.tool_offset_rz_var.get()),
        }

        try:
            write_simple_yaml_mapping(tool_teach_path, payload)
        except Exception as exc:
            self.node.get_logger().warn(
                f'Failed to save tool teach sidecar "{tool_teach_path}": {exc}'
            )
            message = f'Failed to save tool teach for "{self._profile_display_name(active_profile_key)}".'
            self.node._set_action_text(message)
            self.action_var.set(message)
            return False
        self.node.get_active_profile_tool_offset_state(force=True)
        self._sync_profile_tool_offsets_from_state(force=True)
        self._save_runtime_settings()
        return True

    def _collect_runtime_settings(self) -> dict:
        return {
            'schema_version': 7,
            'tf_only_mode': bool(self.tf_only_var.get()),
            'item_pose_wait_timeout_sec': float(self.item_pose_watch_timeout_var.get()),
            'tray_intercept_x_offset_mm': 0.0,
            'tray_intercept_y_offset_mm': 0.0,
            'item_standoff_z_mm': float(self.post_stop_z_offset_var.get()),
            'approach_z_up_mm': float(self.approach_z_up_var.get()),
            'final_z_up_mm': float(self.final_z_up_var.get()),
            'settling_time_sec': float(self.settling_time_var.get()),
            'tool_offset_x_mm': float(self.tool_offset_x_var.get()),
            'tool_offset_y_mm': float(self.tool_offset_y_var.get()),
            'tool_offset_z_mm': float(self.tool_offset_z_var.get()),
            'tool_offset_rx_deg': float(self.tool_offset_rx_var.get()),
            'tool_offset_ry_deg': float(self.tool_offset_ry_var.get()),
            'tool_offset_rz_deg': float(self.tool_offset_rz_var.get()),
        }

    def _schedule_runtime_settings_save(self) -> None:
        if self._runtime_settings_save_after_id is not None:
            self.root.after_cancel(self._runtime_settings_save_after_id)
            self._runtime_settings_save_after_id = None
        self._runtime_settings_save_after_id = self.root.after(
            RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS,
            self._save_runtime_settings,
        )

    def _on_runtime_setting_changed(self, *_args) -> None:
        if self._suspend_runtime_settings_events:
            return
        self._sync_tf_only_button(is_busy=False)
        self._update_profile_tool_offset_status()
        self._schedule_runtime_settings_save()

    def _load_runtime_settings(self) -> None:
        if not self._runtime_settings_path.exists():
            return
        try:
            with self._runtime_settings_path.open('r', encoding='utf-8') as infile:
                payload = json.load(infile)
        except Exception as exc:
            self.node.get_logger().warn(
                f'Failed to read item pick runtime settings at "{self._runtime_settings_path}": {exc}'
            )
            return

        if not isinstance(payload, dict):
            return

        self._suspend_runtime_settings_events = True
        try:
            self.tf_only_var.set(bool(payload.get('tf_only_mode', True)))
            self.item_pose_watch_timeout_var.set(self._clamp(
                payload.get(
                    'item_pose_wait_timeout_sec',
                    payload.get('tray_vector_wait_timeout_sec', ITEM_POSE_WATCH_TIMEOUT_SEC),
                ),
                ITEM_POSE_WATCH_TIMEOUT_MIN,
                ITEM_POSE_WATCH_TIMEOUT_MAX,
            ))
            self.post_stop_z_offset_var.set(self._clamp(
                payload.get('item_standoff_z_mm', payload.get('tray_standoff_z_mm', 100.0)),
                POST_STOP_Z_OFFSET_MIN,
                POST_STOP_Z_OFFSET_MAX,
            ))
            self.approach_z_up_var.set(self._clamp(
                payload.get('approach_z_up_mm', payload.get('post_pick_z_up_mm', APPROACH_Z_UP_DEFAULT)),
                APPROACH_Z_UP_MIN,
                APPROACH_Z_UP_MAX,
            ))
            self.final_z_up_var.set(self._clamp(
                payload.get('final_z_up_mm', FINAL_Z_UP_DEFAULT),
                FINAL_Z_UP_MIN,
                FINAL_Z_UP_MAX,
            ))
            self.settling_time_var.set(self._clamp(
                payload.get('settling_time_sec', SETTLING_TIME_DEFAULT_SEC),
                SETTLING_TIME_MIN_SEC,
                SETTLING_TIME_MAX_SEC,
            ))
            self.tool_offset_x_var.set(self._clamp(
                payload.get('tool_offset_x_mm', 0.0),
                TOOL_OFFSET_TRANSLATION_MIN_MM,
                TOOL_OFFSET_TRANSLATION_MAX_MM,
            ))
            self.tool_offset_y_var.set(self._clamp(
                payload.get('tool_offset_y_mm', 0.0),
                TOOL_OFFSET_TRANSLATION_MIN_MM,
                TOOL_OFFSET_TRANSLATION_MAX_MM,
            ))
            self.tool_offset_z_var.set(self._clamp(
                payload.get('tool_offset_z_mm', 0.0),
                TOOL_OFFSET_TRANSLATION_MIN_MM,
                TOOL_OFFSET_TRANSLATION_MAX_MM,
            ))
            self.tool_offset_rx_var.set(self._clamp(
                payload.get('tool_offset_rx_deg', 0.0),
                TOOL_OFFSET_ROTATION_MIN_DEG,
                TOOL_OFFSET_ROTATION_MAX_DEG,
            ))
            self.tool_offset_ry_var.set(self._clamp(
                payload.get('tool_offset_ry_deg', 0.0),
                TOOL_OFFSET_ROTATION_MIN_DEG,
                TOOL_OFFSET_ROTATION_MAX_DEG,
            ))
            self.tool_offset_rz_var.set(self._clamp(
                payload.get('tool_offset_rz_deg', 0.0),
                TOOL_OFFSET_ROTATION_MIN_DEG,
                TOOL_OFFSET_ROTATION_MAX_DEG,
            ))
        finally:
            self._suspend_runtime_settings_events = False

    def _save_runtime_settings(self) -> None:
        self._runtime_settings_save_after_id = None
        payload = self._collect_runtime_settings()
        try:
            self._runtime_settings_path.parent.mkdir(parents=True, exist_ok=True)
            with self._runtime_settings_path.open('w', encoding='utf-8') as outfile:
                json.dump(payload, outfile, indent=2)
        except Exception as exc:
            self.node.get_logger().warn(
                f'Failed to save item pick runtime settings at "{self._runtime_settings_path}": {exc}'
            )

    def _refresh(self) -> None:
        self._sync_profile_tool_offsets_from_state()
        snapshot = self.node.snapshot()
        stop_inflight = self.node.is_manual_stop_inflight()
        self._sync_tf_only_button(is_busy=bool(snapshot.busy) or stop_inflight)
        release_inflight = self.node.is_manual_release_inflight()
        can_arm = self._can_arm_sequence()
        can_save_tool_offset = (
            self._active_item_profile_key is not None
            and not snapshot.busy
            and not stop_inflight
        )

        self.action_var.set(snapshot.action_text)
        self.run_button.configure(
            state=tk.NORMAL if (not snapshot.busy and not stop_inflight and can_arm) else tk.DISABLED
        )
        self._set_stop_button_enabled(bool(snapshot.busy) and not stop_inflight)
        self.release_button.configure(
            state=tk.DISABLED if (snapshot.busy or stop_inflight or release_inflight) else tk.NORMAL
        )
        self.show_tool_tf_button.configure(
            state=tk.DISABLED if (snapshot.busy or stop_inflight) else tk.NORMAL
        )
        self.save_tool_offset_button.configure(
            state=tk.NORMAL if can_save_tool_offset else tk.DISABLED
        )

        if not self._closed:
            self.root.after(100, self._refresh)

    def _set_stop_button_enabled(self, enabled: bool) -> None:
        if enabled:
            self.stop_button.configure(
                state=tk.NORMAL,
                bg='#d32f2f',
                fg='white',
                activebackground='#b71c1c',
                activeforeground='white',
            )
            return
        self.stop_button.configure(
            state=tk.DISABLED,
            bg=self._stop_default_bg,
            fg=self._stop_default_fg,
            activebackground=self._stop_default_active_bg,
            activeforeground=self._stop_default_active_fg,
        )

    def _sync_tf_only_button(self, is_busy: bool) -> None:
        tf_only_enabled = bool(self.tf_only_var.get())
        if tf_only_enabled:
            label = 'Troubleshoot TF-only: ON'
            bg = '#ef6c00'
            fg = 'white'
            active_bg = '#e65100'
            active_fg = 'white'
        else:
            label = 'Troubleshoot TF-only: OFF'
            bg = self._tf_only_default_bg
            fg = self._tf_only_default_fg
            active_bg = self._tf_only_default_active_bg
            active_fg = self._tf_only_default_active_fg

        self.tf_only_button.configure(
            text=label,
            state=tk.DISABLED if is_busy else tk.NORMAL,
            bg=bg,
            fg=fg,
            activebackground=active_bg,
            activeforeground=active_fg,
        )

    def _on_close(self) -> None:
        if self._runtime_settings_save_after_id is not None:
            self.root.after_cancel(self._runtime_settings_save_after_id)
            self._runtime_settings_save_after_id = None
        self._save_runtime_settings()
        self._closed = True
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemPickNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    gui = ItemPickGui(node)
    try:
        gui.run()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
