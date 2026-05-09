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

from dobot_msgs_v4.msg import ToolVectorActual, TrayVector
from dobot_msgs_v4.srv import CP, GetTrayDimensions, MovL, MovLIO, SpeedFactor, Stop, TrayInterceptStart
from geometry_msgs.msg import PolygonStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


SERVICE_ROOT_DEFAULT = '/dobot_bringup_ros2/srv'
LINEAR_SPEED_MM_S_MIN = 50.0
LINEAR_SPEED_MM_S_MAX = 650.0
DEFAULT_ACC_PERCENT = 100
TCP_FIELDS = ('x', 'y', 'z', 'rx', 'ry', 'rz')
TRANSLATION_AXES = ('x', 'y', 'z')
CALIBRATION_FILE_PATH = Path.home() / '.ros' / 'relmovl_speed_calibration.json'
CALIBRATION_DIR_PATH = Path.home() / 'DOBOT_pickn_place' / 'calibration'
TRAY_VECTOR_TOPIC = 'tray_vector'
TRAY_AXIS_OVERLAY_TOPIC = 'tray_axis_overlay'
ROBOT_GOAL_FRAME_DEFAULT = 'base_link'
POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT = 'tray_movel_goal_tcp'
FOLLOW_MOVL_GOAL_DEBUG_FRAME_DEFAULT = 'tray_follow_goal_tcp'
POST_FOLLOW_ZUP_GOAL_DEBUG_FRAME_DEFAULT = 'tray_post_follow_zup_goal_tcp'
TRAY_VECTOR_WATCH_TIMEOUT_SEC = 60.0
TRAY_VECTOR_WATCH_TIMEOUT_MIN = 1.0
TRAY_VECTOR_WATCH_TIMEOUT_MAX = 60.0
TRAY_VECTOR_MOTION_NOISE_FLOOR_MM_S = 5.0
POST_STOP_MOVL_SPEED_MAX = 650.0
FIXED_EE_INTERCEPT_SPEED_MM_S = 650.0
EE_FINAL_POSE_ANGLE_MIN_DEG = -90.0
EE_FINAL_POSE_ANGLE_MAX_DEG = 90.0
EE_FINAL_POSE_ANGLE_DEFAULT_DEG = 0.0
POST_STOP_X_OFFSET_MIN = -50.0
POST_STOP_X_OFFSET_MAX = 400.0
POST_STOP_Y_OFFSET_MIN = -50.0
POST_STOP_Y_OFFSET_MAX = 300.0
POST_STOP_Z_OFFSET_MIN = 50.0
POST_STOP_Z_OFFSET_MAX = 200.0
COMMAND_HYSTERESIS_MIN_SEC = 0.1
COMMAND_HYSTERESIS_MAX_SEC = 1.0
COMMAND_HYSTERESIS_DEFAULT_SEC = 0.1
FOLLOW_DISTANCE_MIN = 0.0
FOLLOW_DISTANCE_MAX = 400.0
POST_FOLLOW_Z_UP_MIN = 0.0
POST_FOLLOW_Z_UP_MAX = 300.0
GOAL_TF_LOOKUP_TIMEOUT_SEC_DEFAULT = 0.2
TRAY_PREDICTION_MAX_LEAD_SEC_DEFAULT = 3.0
START_SEQUENCE_SERVICE_DEFAULT = 'tray_intercept/start_sequence'
TRACK_SERVICE_DEFAULT = 'tray_intercept/track'
TRACK_STATUS_SERVICE_DEFAULT = 'tray_intercept/track_status'
TRAY_DIMENSIONS_SERVICE_DEFAULT = 'tray_detect/get_tray_dimensions'
TRAY_SEEK_COMPLETE_SERVICE_DEFAULT = 'tray_detect/seek_complete'
TRAY_DIMENSIONS_SERVICE_WAIT_SEC = 2.0
TRAY_DIMENSIONS_SERVICE_CALL_SEC = 3.0
TRAY_DIMENSIONS_AUTO_REFRESH_SEC = 2.0
TRAY_DIMENSIONS_AUTO_RETRY_SEC = 1.0
TRAY_PREVIEW_LENGTH_MM = 400.0
TRAY_PREVIEW_WIDTH_MM = 300.0
TRAY_PREVIEW_LENGTH_MIN_MM = 100.0
TRAY_PREVIEW_LENGTH_MAX_MM = 500.0
TRAY_PREVIEW_WIDTH_MIN_MM = 100.0
TRAY_PREVIEW_WIDTH_MAX_MM = 500.0
TRAY_PREVIEW_BORDER_THICKNESS_MM = 50.0
TRAY_PREVIEW_BORDER_MIN_MM = 0.0
TRAY_PREVIEW_BORDER_MAX_MM = 150.0
INTERCEPT_DOT_DIAMETER_DEFAULT_MM = 10.0
INTERCEPT_DOT_DIAMETER_MIN_MM = 2.0
INTERCEPT_DOT_DIAMETER_MAX_MM = 60.0
RUNTIME_SETTINGS_PATH = Path.home() / '.ros' / 'tray_intercept_runtime_settings.json'
RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS = 250
MOTION_PROFILE_ENFORCE_WAIT_SEC = 12.0
MOTION_PROFILE_ENFORCE_CALL_SEC = 8.0
MOVLIO_RELEASE_MODE_PERCENT = 0
MOVLIO_RELEASE_START_DISTANCE_PERCENT = 1
GRIPPER_DO_CLOSE_INDEX = 1
GRIPPER_DO_OPEN_INDEX = 2
GRIPPER_DO_SUCTION_INDEX = 3


@dataclass
class MiniSnapshot:
    tcp_values: dict[str, float] = field(default_factory=lambda: {name: 0.0 for name in TCP_FIELDS})
    tcp_stamp: float | None = None
    busy: bool = False
    armed: bool = False
    action_text: str = 'Ready'
    tray_seq: int = 0
    has_last_tray: bool = False
    tray_preview_axes_valid: bool = False
    tray_preview_x_axis: tuple[float, float] = (1.0, 0.0)
    tray_preview_y_axis: tuple[float, float] = (0.0, 1.0)


@dataclass(frozen=True)
class TrayVectorTarget:
    position_mm: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    frame_id: str
    stamp_sec: float
    decay_sec: float
    speed_mmps: float
    direction_unit: tuple[float, float, float]


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
    tray_age_sec: float
    tray_speed_base_mmps: float
    follow_direction_base_unit: tuple[float, float, float]
    tray_axis_name: str = 'tray_y'
    tray_axis_rz_deg: float = 0.0
    ee_angle_signed_deg: float = 0.0
    ee_angle_direction_label: str = 'tray_axis'


class RelMovLMiniNode(Node):
    def __init__(self) -> None:
        super().__init__('tray_intercept')
        self._lock = threading.Lock()
        self._snapshot = MiniSnapshot()
        self._speed_calibration_path: Path | None = None
        self._calibration_startup_cp: int | None = None
        self._calibration_startup_speed_factor: int | None = None
        self._startup_motion_profile_applied = False
        self._axis_speed_lookup = self._load_speed_calibration()
        self._tray_vector_seq = 0
        self._tray_watch_armed = False
        self._tray_watch_seq_floor = 0
        self._tray_watch_deadline_monotonic = 0.0
        self._tray_watch_stop_dispatched = False
        self._tray_watch_generation = 0
        self._tray_watch_tf_only_mode = True
        self._tray_vector_watch_timeout_sec = TRAY_VECTOR_WATCH_TIMEOUT_SEC
        self._preview_inflight = False
        self._cancel_requested = False
        self._manual_stop_inflight = False
        self._post_stop_movel_speed_mm_s = FIXED_EE_INTERCEPT_SPEED_MM_S
        self._ee_final_pose_angle_deg = EE_FINAL_POSE_ANGLE_DEFAULT_DEG
        self._post_stop_x_offset_mm = 0.0
        self._post_stop_y_offset_mm = 0.0
        self._post_stop_z_offset_mm = 100.0
        self._tray_preview_length_mm = TRAY_PREVIEW_LENGTH_MM
        self._tray_preview_width_mm = TRAY_PREVIEW_WIDTH_MM
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
        self._follow_distance_mm = 200.0
        self._post_follow_z_up_mm = 300.0
        self._release_grip_enabled = False
        self._publish_goal_debug_tf = bool(
            self.declare_parameter('publish_goal_debug_tf', True).value
        )
        self._robot_goal_frame_id = str(
            self.declare_parameter('robot_goal_frame_id', ROBOT_GOAL_FRAME_DEFAULT).value
        ).strip() or ROBOT_GOAL_FRAME_DEFAULT
        self._post_stop_movel_goal_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_debug_frame_id',
                POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT
        self._follow_movel_goal_debug_frame_id = str(
            self.declare_parameter(
                'follow_movel_goal_debug_frame_id',
                FOLLOW_MOVL_GOAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or FOLLOW_MOVL_GOAL_DEBUG_FRAME_DEFAULT
        self._post_follow_zup_goal_debug_frame_id = str(
            self.declare_parameter(
                'post_follow_zup_goal_debug_frame_id',
                POST_FOLLOW_ZUP_GOAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_FOLLOW_ZUP_GOAL_DEBUG_FRAME_DEFAULT
        self._goal_tf_lookup_timeout_sec = max(
            0.01,
            float(self.declare_parameter(
                'goal_tf_lookup_timeout_sec',
                GOAL_TF_LOOKUP_TIMEOUT_SEC_DEFAULT,
            ).value),
        )
        self._tray_prediction_max_lead_sec = max(
            0.0,
            float(self.declare_parameter(
                'tray_prediction_max_lead_sec',
                TRAY_PREDICTION_MAX_LEAD_SEC_DEFAULT,
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
        self._tray_dimensions_service_name = str(
            self.declare_parameter(
                'tray_dimensions_service',
                TRAY_DIMENSIONS_SERVICE_DEFAULT,
            ).value
        ).strip() or TRAY_DIMENSIONS_SERVICE_DEFAULT
        self._tray_axis_overlay_topic = str(
            self.declare_parameter(
                'tray_axis_overlay_topic',
                TRAY_AXIS_OVERLAY_TOPIC,
            ).value
        ).strip() or TRAY_AXIS_OVERLAY_TOPIC
        self._tray_seek_complete_service_name = str(
            self.declare_parameter(
                'tray_seek_complete_service',
                TRAY_SEEK_COMPLETE_SERVICE_DEFAULT,
            ).value
        ).strip() or TRAY_SEEK_COMPLETE_SERVICE_DEFAULT

        self._mov_l_client = self.create_client(MovL, f'{SERVICE_ROOT_DEFAULT}/MovL')
        self._mov_lio_client = self.create_client(MovLIO, f'{SERVICE_ROOT_DEFAULT}/MovLIO')
        self._stop_client = self.create_client(Stop, f'{SERVICE_ROOT_DEFAULT}/Stop')
        self._cp_client = self.create_client(CP, f'{SERVICE_ROOT_DEFAULT}/CP')
        self._speed_factor_client = self.create_client(SpeedFactor, f'{SERVICE_ROOT_DEFAULT}/SpeedFactor')
        self._tray_dimensions_client = self.create_client(
            GetTrayDimensions,
            self._tray_dimensions_service_name,
        )
        self._tray_seek_complete_client = self.create_client(
            Trigger,
            self._tray_seek_complete_service_name,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._goal_static_tf_by_child: dict[str, TransformStamped] = {}
        self._track_trigger_handler = None
        self._last_tray_target: TrayVectorTarget | None = None
        self._last_tray_preview_axes_valid = False
        self._last_tray_preview_x_axis = (1.0, 0.0)
        self._last_tray_preview_y_axis = (0.0, 1.0)
        self.create_subscription(ToolVectorActual, 'dobot_msgs_v4/msg/ToolVectorActual', self._tcp_callback, 10)
        self.create_subscription(TrayVector, TRAY_VECTOR_TOPIC, self._tray_vector_callback, 10)
        self.create_subscription(PolygonStamped, self._tray_axis_overlay_topic, self._tray_axis_overlay_callback, 10)
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
        self.get_logger().info('Tray mode configured: queued MovL motion, MovLIO when release IO is enabled.')
        self.get_logger().info(f'Start tray sequence service: {self._start_sequence_service_name}')
        self.get_logger().info(f'Track virtual-click service: {self._track_service_name}')
        self.get_logger().info(f'Track armed status service: {self._track_status_service_name}')
        self.get_logger().info(
            'Startup defaults: '
            f'wait={self._tray_vector_watch_timeout_sec:.0f}s, '
            f'fixed_speed={self._post_stop_movel_speed_mm_s:.0f} mm/s, '
            f'ee_angle_offset={self._ee_final_pose_angle_deg:.0f} deg, '
            f'offsets(x={self._post_stop_x_offset_mm:.0f},'
            f'y={self._post_stop_y_offset_mm:.0f},'
            f'z={self._post_stop_z_offset_mm:.0f}) mm, '
            f'hysteresis={self._command_hysteresis_sec:.2f}s, '
            f'follow={self._follow_distance_mm:.0f} mm, '
            f'post_z_up={self._post_follow_z_up_mm:.0f} mm, '
            f'release_grip={"on" if self._release_grip_enabled else "off"}'
        )
        if self._calibration_startup_cp is not None and self._calibration_startup_speed_factor is not None:
            source = str(self._speed_calibration_path) if self._speed_calibration_path is not None else 'speed calibration file'
            self.get_logger().info(
                'Movement profile from calibration: '
                f'CP={self._calibration_startup_cp}, '
                f'SpeedFactor={self._calibration_startup_speed_factor} '
                f'(source "{source}").'
            )
        else:
            self.get_logger().warn(
                'Movement calibration CP/SF metadata not found. '
                'Tray intercept will keep current robot CP/SpeedFactor state.'
            )

    def _reset_runtime_state_locked(self, reason: str) -> None:
        self._tray_watch_generation += 1
        self._tray_watch_armed = False
        self._tray_watch_stop_dispatched = False
        self._tray_watch_seq_floor = self._tray_vector_seq
        self._tray_watch_deadline_monotonic = 0.0
        self._cancel_requested = False
        self._snapshot.busy = False
        self._snapshot.action_text = reason

    def _reset_runtime_state(self, reason: str) -> None:
        with self._lock:
            self._reset_runtime_state_locked(reason)

    def snapshot(self) -> MiniSnapshot:
        with self._lock:
            return MiniSnapshot(
                tcp_values=dict(self._snapshot.tcp_values),
                tcp_stamp=self._snapshot.tcp_stamp,
                busy=self._snapshot.busy,
                armed=self._tray_watch_armed,
                action_text=self._snapshot.action_text,
                tray_seq=self._tray_vector_seq,
                has_last_tray=self._last_tray_target is not None,
                tray_preview_axes_valid=self._last_tray_preview_axes_valid,
                tray_preview_x_axis=self._last_tray_preview_x_axis,
                tray_preview_y_axis=self._last_tray_preview_y_axis,
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

    def _tray_axis_overlay_callback(self, msg: PolygonStamped) -> None:
        points = getattr(getattr(msg, 'polygon', None), 'points', [])
        if len(points) < 3:
            return

        x_axis = self._image_axis_to_preview_axis(points[1].x, points[1].y)
        y_axis = self._image_axis_to_preview_axis(points[2].x, points[2].y)
        if x_axis is None or y_axis is None:
            return

        det = (x_axis[0] * y_axis[1]) - (x_axis[1] * y_axis[0])
        if abs(det) <= 1e-6:
            return

        with self._lock:
            self._last_tray_preview_axes_valid = True
            self._last_tray_preview_x_axis = x_axis
            self._last_tray_preview_y_axis = y_axis

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
    def _rotate_vector_by_quaternion(
        cls,
        vector_xyz: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        q = cls._quat_normalize(quaternion_xyzw)
        pure = (float(vector_xyz[0]), float(vector_xyz[1]), float(vector_xyz[2]), 0.0)
        rotated = cls._quat_multiply(cls._quat_multiply(q, pure), cls._quat_conjugate(q))
        return (rotated[0], rotated[1], rotated[2])

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

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        normalized = (float(angle_deg) + 180.0) % 360.0 - 180.0
        if normalized <= -180.0:
            return normalized + 360.0
        return normalized

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
        norm = RelMovLMiniNode._vector_norm3(vector_xyz)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0)
        return (vector_xyz[0] / norm, vector_xyz[1] / norm, vector_xyz[2] / norm)

    @staticmethod
    def _project_unit_to_base_xy(vector_xyz: tuple[float, float, float]) -> tuple[float, float, float] | None:
        x = float(vector_xyz[0])
        y = float(vector_xyz[1])
        norm = math.sqrt((x * x) + (y * y))
        if norm <= 1e-9:
            return None
        return (x / norm, y / norm, 0.0)

    @staticmethod
    def _image_axis_to_preview_axis(x_image: float, y_image: float) -> tuple[float, float] | None:
        x = float(x_image)
        y = -float(y_image)
        norm = math.sqrt((x * x) + (y * y))
        if norm <= 1e-9:
            return None
        return (x / norm, y / norm)

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

        c = RelMovLMiniNode._vector_dot3(r, r)
        if c <= 1e-9:
            return 0.0

        a = RelMovLMiniNode._vector_dot3(v, v) - (s * s)
        b = 2.0 * RelMovLMiniNode._vector_dot3(r, v)
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

    def _tray_pose_camera_to_base(
        self,
        tray_x_mm: float,
        tray_y_mm: float,
        tray_z_mm: float,
        tray_rx_deg: float,
        tray_ry_deg: float,
        tray_rz_deg: float,
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
        p_camera_tray_m = (
            float(tray_x_mm) * 0.001,
            float(tray_y_mm) * 0.001,
            float(tray_z_mm) * 0.001,
        )
        p_base_tray_offset_m = self._rotate_vector_by_quaternion(p_camera_tray_m, q_base_camera)
        p_base_tray_m = (
            t_base_camera_m[0] + p_base_tray_offset_m[0],
            t_base_camera_m[1] + p_base_tray_offset_m[1],
            t_base_camera_m[2] + p_base_tray_offset_m[2],
        )

        q_camera_tray = self._quat_normalize(
            self._rpy_deg_to_quaternion(tray_rx_deg, tray_ry_deg, tray_rz_deg),
        )
        q_base_tray = self._quat_normalize(self._quat_multiply(q_base_camera, q_camera_tray))
        tray_rpy_base_deg = self._quaternion_to_rpy_deg(q_base_tray)

        return (
            p_base_tray_m[0] * 1000.0,
            p_base_tray_m[1] * 1000.0,
            p_base_tray_m[2] * 1000.0,
            tray_rpy_base_deg[0],
            tray_rpy_base_deg[1],
            tray_rpy_base_deg[2],
            q_base_tray,
            q_base_camera,
        )

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

    def _tray_vector_callback(self, msg: TrayVector) -> None:
        header_stamp_sec = self._builtin_time_to_sec(msg.header.stamp)
        last_stamp_sec = self._builtin_time_to_sec(msg.last_stamp)
        tray_stamp_sec = last_stamp_sec if last_stamp_sec > 1e-9 else header_stamp_sec
        raw_speed_mmps = max(0.0, float(msg.speed_mmps))
        if raw_speed_mmps < TRAY_VECTOR_MOTION_NOISE_FLOOR_MM_S:
            filtered_speed_mmps = 0.0
            filtered_direction_unit = (0.0, 0.0, 0.0)
        else:
            filtered_speed_mmps = raw_speed_mmps
            filtered_direction_unit = (
                float(msg.direction_unit.x),
                float(msg.direction_unit.y),
                float(msg.direction_unit.z),
            )
        tray_target = TrayVectorTarget(
            position_mm=(
                float(msg.position_mm.x),
                float(msg.position_mm.y),
                float(msg.position_mm.z),
            ),
            rpy_deg=(
                float(msg.rpy_deg.x),
                float(msg.rpy_deg.y),
                float(msg.rpy_deg.z),
            ),
            frame_id=str(msg.header.frame_id),
            stamp_sec=tray_stamp_sec,
            decay_sec=max(0.0, float(getattr(msg, 'decay_sec', 0.0))),
            speed_mmps=filtered_speed_mmps,
            direction_unit=filtered_direction_unit,
        )

        should_send_stop = False
        tf_only_mode = False
        dispatch_target: TrayVectorTarget | None = None
        x_offset_mm = 0.0
        y_offset_mm = 0.0
        z_offset_mm = 0.0
        follow_distance_mm = 0.0
        post_follow_z_up_mm = 0.0
        ee_final_pose_angle_deg = EE_FINAL_POSE_ANGLE_DEFAULT_DEG
        release_grip_enabled = False
        watch_timeout_sec = TRAY_VECTOR_WATCH_TIMEOUT_SEC
        with self._lock:
            self._tray_vector_seq += 1
            self._last_tray_target = tray_target
            if not self._tray_watch_armed:
                return
            if self._tray_vector_seq <= self._tray_watch_seq_floor:
                return
            if self._tray_watch_stop_dispatched:
                return
            if time.monotonic() > self._tray_watch_deadline_monotonic:
                watch_timeout_sec = float(self._tray_vector_watch_timeout_sec)
                self._reset_runtime_state_locked(
                    f'No tray vector within {watch_timeout_sec:.0f}s. Node reset.'
                )
                return

            self._tray_watch_armed = False
            self._tray_watch_stop_dispatched = True
            tf_only_mode = bool(self._tray_watch_tf_only_mode)
            watch_timeout_sec = float(self._tray_vector_watch_timeout_sec)
            should_send_stop = True
            dispatch_target = tray_target
            x_offset_mm = float(self._post_stop_x_offset_mm)
            y_offset_mm = float(self._post_stop_y_offset_mm)
            z_offset_mm = float(self._post_stop_z_offset_mm)
            follow_distance_mm = float(self._follow_distance_mm)
            post_follow_z_up_mm = float(self._post_follow_z_up_mm)
            ee_final_pose_angle_deg = float(self._ee_final_pose_angle_deg)
            release_grip_enabled = bool(self._release_grip_enabled)

        if tf_only_mode:
            if should_send_stop and dispatch_target is not None:
                self._set_action_text(
                    'Tray vector update detected. Troubleshoot mode: goal TF preview only...'
                )
                worker = threading.Thread(
                    target=self._preview_goal_only_request,
                    args=(
                        dispatch_target,
                        x_offset_mm,
                        y_offset_mm,
                        z_offset_mm,
                        follow_distance_mm,
                        post_follow_z_up_mm,
                        ee_final_pose_angle_deg,
                    ),
                    daemon=True,
                )
                worker.start()
            return

        if should_send_stop and dispatch_target is not None:
            self._set_action_text('Tray vector update detected. Sending Stop...')
            worker = threading.Thread(
                target=self._send_stop_and_movel_request,
                args=(
                    dispatch_target,
                    x_offset_mm,
                    y_offset_mm,
                    z_offset_mm,
                    follow_distance_mm,
                    post_follow_z_up_mm,
                    ee_final_pose_angle_deg,
                    release_grip_enabled,
                ),
                daemon=True,
            )
            worker.start()

    def _compute_base_goal_from_tray_target(
        self,
        tray_target: TrayVectorTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        ee_speed_mmps: float,
        ee_final_pose_angle_deg: float = EE_FINAL_POSE_ANGLE_DEFAULT_DEG,
        predict_target_motion: bool = True,
    ) -> PredictedGoal | None:
        target_x, target_y, target_z = tray_target.position_mm
        target_rx, target_ry, target_rz = tray_target.rpy_deg
        frame_id = tray_target.frame_id
        tray_base_pose = self._tray_pose_camera_to_base(
            target_x,
            target_y,
            target_z,
            target_rx,
            target_ry,
            target_rz,
            frame_id,
        )
        if tray_base_pose is None:
            return None

        tray_base_x, tray_base_y, tray_base_z, _, _, _, q_base_tray, q_base_camera = tray_base_pose
        raw_tray_local_x_in_base = self._rotate_vector_by_quaternion((1.0, 0.0, 0.0), q_base_tray)
        raw_tray_local_y_in_base = self._rotate_vector_by_quaternion((0.0, 1.0, 0.0), q_base_tray)
        raw_tray_local_z_in_base = self._rotate_vector_by_quaternion((0.0, 0.0, 1.0), q_base_tray)
        tray_local_x_in_base = self._project_unit_to_base_xy(raw_tray_local_x_in_base)
        tray_local_y_in_base = self._project_unit_to_base_xy(raw_tray_local_y_in_base)
        if tray_local_x_in_base is None or tray_local_y_in_base is None:
            self._set_action_text('Tray pose rejected: tray X/Y axes cannot be projected safely into base XY.')
            return None
        if raw_tray_local_z_in_base[2] < -0.1:
            self.get_logger().warn(
                'Detected tray Z points down; using base-XY tray axes and base-up standoff for robot safety.'
            )

        speed_mmps = max(0.0, float(tray_target.speed_mmps))
        direction_cam = self._normalize_vector3(tray_target.direction_unit)
        velocity_cam_mmps = (
            direction_cam[0] * speed_mmps,
            direction_cam[1] * speed_mmps,
            direction_cam[2] * speed_mmps,
        )
        raw_velocity_base_mmps = self._rotate_vector_by_quaternion(velocity_cam_mmps, q_base_camera)
        velocity_base_mmps = (
            raw_velocity_base_mmps[0],
            raw_velocity_base_mmps[1],
            0.0,
        )
        tray_speed_base_mmps = self._vector_norm3(velocity_base_mmps)
        if tray_speed_base_mmps > 1e-6:
            follow_direction_base_unit = self._normalize_vector3(velocity_base_mmps)
        else:
            follow_direction_base_unit = (0.0, 0.0, 0.0)

        if predict_target_motion:
            now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
            raw_age_sec = max(0.0, now_sec - max(0.0, float(tray_target.stamp_sec)))
            tray_age_sec = (
                raw_age_sec
                + max(0.0, float(tray_target.decay_sec))
                + max(0.0, float(self._command_hysteresis_sec))
            )
            tray_now_x = tray_base_x + (velocity_base_mmps[0] * tray_age_sec)
            tray_now_y = tray_base_y + (velocity_base_mmps[1] * tray_age_sec)
            tray_now_z = tray_base_z + (velocity_base_mmps[2] * tray_age_sec)
        else:
            tray_age_sec = 0.0
            tray_now_x = tray_base_x
            tray_now_y = tray_base_y
            tray_now_z = tray_base_z

        snapshot = self.snapshot()
        safe_z_offset_mm = max(POST_STOP_Z_OFFSET_MIN, float(z_offset_mm))
        stand_off_vec_base_mm = (
            (tray_local_x_in_base[0] * x_offset_mm)
            + (tray_local_y_in_base[0] * y_offset_mm),
            (tray_local_x_in_base[1] * x_offset_mm)
            + (tray_local_y_in_base[1] * y_offset_mm),
            safe_z_offset_mm,
        )
        desired_now_goal_mm = (
            tray_now_x + stand_off_vec_base_mm[0],
            tray_now_y + stand_off_vec_base_mm[1],
            tray_now_z + stand_off_vec_base_mm[2],
        )
        tcp_now_mm = (
            float(snapshot.tcp_values.get('x', 0.0)),
            float(snapshot.tcp_values.get('y', 0.0)),
            float(snapshot.tcp_values.get('z', 0.0)),
        )
        relative_goal_mm = (
            desired_now_goal_mm[0] - tcp_now_mm[0],
            desired_now_goal_mm[1] - tcp_now_mm[1],
            desired_now_goal_mm[2] - tcp_now_mm[2],
        )

        if predict_target_motion:
            ee_speed_limited_mmps = max(1.0, min(POST_STOP_MOVL_SPEED_MAX, float(ee_speed_mmps)))
            lead_time_sec = self._solve_intercept_time_sec(
                relative_goal_mm,
                velocity_base_mmps,
                ee_speed_limited_mmps,
            )
            if lead_time_sec is None:
                lead_time_sec = self._vector_norm3(relative_goal_mm) / ee_speed_limited_mmps
            lead_time_sec = max(0.0, min(self._tray_prediction_max_lead_sec, lead_time_sec))
        else:
            lead_time_sec = 0.0

        target_x_goal = desired_now_goal_mm[0] + (velocity_base_mmps[0] * lead_time_sec)
        target_y_goal = desired_now_goal_mm[1] + (velocity_base_mmps[1] * lead_time_sec)
        target_z_goal = desired_now_goal_mm[2] + (velocity_base_mmps[2] * lead_time_sec)

        current_rx_deg = float(snapshot.tcp_values.get('rx', 0.0))
        current_ry_deg = float(snapshot.tcp_values.get('ry', 0.0))
        ee_angle_deg = max(
            EE_FINAL_POSE_ANGLE_MIN_DEG,
            min(EE_FINAL_POSE_ANGLE_MAX_DEG, float(ee_final_pose_angle_deg)),
        )
        signed_ee_angle_deg = 0.0
        ee_angle_direction_label = 'tray_axis'
        if ee_angle_deg < -1e-6:
            signed_ee_angle_deg = ee_angle_deg
            ee_angle_direction_label = 'ccw_offset'
        elif ee_angle_deg > 1e-6:
            signed_ee_angle_deg = ee_angle_deg
            ee_angle_direction_label = 'cw_offset'

        x_vertical_score = abs(tray_local_x_in_base[1])
        y_vertical_score = abs(tray_local_y_in_base[1])
        if x_vertical_score > y_vertical_score:
            vertical_axis_name = 'tray_x'
            vertical_axis_in_base = tray_local_x_in_base
        else:
            vertical_axis_name = 'tray_y'
            vertical_axis_in_base = tray_local_y_in_base
        tray_axis_rz_deg = self._normalize_angle_deg(
            math.degrees(math.atan2(vertical_axis_in_base[1], vertical_axis_in_base[0]))
        )
        # GUI convention is negative=CCW and positive=CW. Mathematical yaw is
        # positive CCW, so subtract the operator offset from the selected
        # vertical tray-axis yaw.
        goal_rx_deg = current_rx_deg
        goal_ry_deg = current_ry_deg
        goal_rz_deg = self._normalize_angle_deg(tray_axis_rz_deg - signed_ee_angle_deg)

        return PredictedGoal(
            x_mm=target_x_goal,
            y_mm=target_y_goal,
            z_mm=target_z_goal,
            rx_deg=goal_rx_deg,
            ry_deg=goal_ry_deg,
            rz_deg=goal_rz_deg,
            source_frame_id=frame_id,
            lead_time_sec=lead_time_sec,
            tray_age_sec=tray_age_sec,
            tray_speed_base_mmps=tray_speed_base_mmps,
            follow_direction_base_unit=follow_direction_base_unit,
            tray_axis_name=vertical_axis_name,
            tray_axis_rz_deg=tray_axis_rz_deg,
            ee_angle_signed_deg=signed_ee_angle_deg,
            ee_angle_direction_label=ee_angle_direction_label,
        )

    def _compute_follow_and_post_z_up_goals(
        self,
        base_goal: PredictedGoal,
        follow_distance_mm: float,
        post_follow_z_up_mm: float,
        post_z_up_speed_mm_s: float,
    ) -> tuple[
        tuple[float, float, float, float, float, float],
        tuple[float, float, float, float, float, float],
        float,
    ]:
        post_z_up_speed_limited = max(LINEAR_SPEED_MM_S_MIN, min(POST_STOP_MOVL_SPEED_MAX, float(post_z_up_speed_mm_s)))
        post_z_up_duration_sec = max(0.0, float(post_follow_z_up_mm)) / post_z_up_speed_limited
        zup_follow_distance_mm = float(base_goal.tray_speed_base_mmps) * post_z_up_duration_sec
        follow_goal = (
            base_goal.x_mm + (base_goal.follow_direction_base_unit[0] * follow_distance_mm),
            base_goal.y_mm + (base_goal.follow_direction_base_unit[1] * follow_distance_mm),
            base_goal.z_mm + (base_goal.follow_direction_base_unit[2] * follow_distance_mm),
            base_goal.rx_deg,
            base_goal.ry_deg,
            base_goal.rz_deg,
        )
        post_follow_goal = (
            follow_goal[0] + (base_goal.follow_direction_base_unit[0] * zup_follow_distance_mm),
            follow_goal[1] + (base_goal.follow_direction_base_unit[1] * zup_follow_distance_mm),
            follow_goal[2] + (base_goal.follow_direction_base_unit[2] * zup_follow_distance_mm) + post_follow_z_up_mm,
            follow_goal[3],
            follow_goal[4],
            follow_goal[5],
        )
        return follow_goal, post_follow_goal, zup_follow_distance_mm

    def _send_movel_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        if forced_v_percent is None:
            if reference_pose is None:
                delta_xyz = (
                    float(goal[0]),
                    float(goal[1]),
                    float(goal[2]),
                )
            else:
                delta_xyz = (
                    float(goal[0]) - float(reference_pose[0]),
                    float(goal[1]) - float(reference_pose[1]),
                    float(goal[2]) - float(reference_pose[2]),
                )
            dominant_axis = self._dominant_linear_axis([
                delta_xyz[0],
                delta_xyz[1],
                delta_xyz[2],
            ])
            v_percent, mapping_source = self._mm_s_to_v_percent(speed_mm_s, dominant_axis)
        else:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
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

    @staticmethod
    def _build_movelio_do_token(
        mode: int,
        distance: int,
        index: int,
        status: int,
    ) -> str:
        return f'{{{int(mode)},{int(distance)},{int(index)},{int(status)}}}'

    def _build_release_follow_mdis(self) -> list[str]:
        return [
            self._build_movelio_do_token(
                MOVLIO_RELEASE_MODE_PERCENT,
                MOVLIO_RELEASE_START_DISTANCE_PERCENT,
                GRIPPER_DO_CLOSE_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_RELEASE_MODE_PERCENT,
                MOVLIO_RELEASE_START_DISTANCE_PERCENT,
                GRIPPER_DO_SUCTION_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_RELEASE_MODE_PERCENT,
                MOVLIO_RELEASE_START_DISTANCE_PERCENT,
                GRIPPER_DO_OPEN_INDEX,
                1,
            ),
        ]

    def _build_release_post_follow_mdis(self) -> list[str]:
        return [
            self._build_movelio_do_token(
                MOVLIO_RELEASE_MODE_PERCENT,
                MOVLIO_RELEASE_START_DISTANCE_PERCENT,
                GRIPPER_DO_CLOSE_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_RELEASE_MODE_PERCENT,
                MOVLIO_RELEASE_START_DISTANCE_PERCENT,
                GRIPPER_DO_OPEN_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_RELEASE_MODE_PERCENT,
                MOVLIO_RELEASE_START_DISTANCE_PERCENT,
                GRIPPER_DO_SUCTION_INDEX,
                0,
            ),
        ]

    def _send_movelio_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        mdis: list[str] | None = None,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        if forced_v_percent is None:
            if reference_pose is None:
                delta_xyz = (
                    float(goal[0]),
                    float(goal[1]),
                    float(goal[2]),
                )
            else:
                delta_xyz = (
                    float(goal[0]) - float(reference_pose[0]),
                    float(goal[1]) - float(reference_pose[1]),
                    float(goal[2]) - float(reference_pose[2]),
                )
            dominant_axis = self._dominant_linear_axis([
                delta_xyz[0],
                delta_xyz[1],
                delta_xyz[2],
            ])
            v_percent, mapping_source = self._mm_s_to_v_percent(speed_mm_s, dominant_axis)
        else:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
        a_percent = DEFAULT_ACC_PERCENT
        if forced_a_percent is not None:
            a_percent = max(1, min(100, int(forced_a_percent)))

        movlio_request = MovLIO.Request()
        movlio_request.mode = False
        movlio_request.a = float(goal[0])
        movlio_request.b = float(goal[1])
        movlio_request.c = float(goal[2])
        movlio_request.d = float(goal[3])
        movlio_request.e = float(goal[4])
        movlio_request.f = float(goal[5])
        movlio_request.mdis = list(mdis) if mdis is not None else []
        movlio_request.param_value = self._build_motion_param_value(v_percent, a_percent)

        mdis_label = ''
        if movlio_request.mdis:
            mdis_label = f',mdis={";".join(movlio_request.mdis)}'
        movlio_label = (
            f'{label_prefix}('
            f'{movlio_request.a:.1f},{movlio_request.b:.1f},{movlio_request.c:.1f},'
            f'{movlio_request.d:.2f},{movlio_request.e:.2f},{movlio_request.f:.2f},'
            f'v={v_percent},a={a_percent}{mdis_label})'
        )
        movlio_response = self._call_service(self._mov_lio_client, movlio_request, movlio_label)
        if movlio_response is None:
            return False, v_percent, mapping_source
        if int(getattr(movlio_response, 'res', -1)) < 0:
            return False, v_percent, mapping_source
        return True, v_percent, mapping_source

    def _wait_for_tcp_xyz_goal(
        self,
        goal_xyz_mm: tuple[float, float, float],
        tolerance_mm: float = 5.0,
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
                    self._set_action_text('Sequence cancelled while monitoring queued tray motion.')
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
                self._set_action_text('ROS shutdown while monitoring queued tray motion.')
            return False
        if update_action_text:
            self._set_action_text(f'Timeout monitoring queued tray final goal (tol={tolerance:.1f} mm).')
        return False

    def _preview_goal_only_request(
        self,
        tray_target: TrayVectorTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        follow_distance_mm: float,
        post_follow_z_up_mm: float,
        ee_final_pose_angle_deg: float,
    ) -> None:
        try:
            intercept_speed_mm_s = FIXED_EE_INTERCEPT_SPEED_MM_S
            post_z_up_speed_mm_s = POST_STOP_MOVL_SPEED_MAX
            self._set_action_text('Computing tray goal preview in base frame...')
            base_goal = self._compute_base_goal_from_tray_target(
                tray_target,
                x_offset_mm,
                y_offset_mm,
                z_offset_mm,
                intercept_speed_mm_s,
                ee_final_pose_angle_deg,
                predict_target_motion=False,
            )
            if base_goal is None:
                return
            follow_goal, post_follow_goal, zup_follow_distance_mm = self._compute_follow_and_post_z_up_goals(
                base_goal,
                follow_distance_mm,
                post_follow_z_up_mm,
                post_z_up_speed_mm_s,
            )

            self._publish_goal_debug_transform(
                self._robot_goal_frame_id,
                self._post_stop_movel_goal_debug_frame_id,
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_goal_debug_transform(
                self._robot_goal_frame_id,
                self._follow_movel_goal_debug_frame_id,
                follow_goal[0],
                follow_goal[1],
                follow_goal[2],
                follow_goal[3],
                follow_goal[4],
                follow_goal[5],
            )
            self._publish_goal_debug_transform(
                self._robot_goal_frame_id,
                self._post_follow_zup_goal_debug_frame_id,
                post_follow_goal[0],
                post_follow_goal[1],
                post_follow_goal[2],
                post_follow_goal[3],
                post_follow_goal[4],
                post_follow_goal[5],
            )
            self._set_action_text(
                f'Previewed 3-stage goal from {base_goal.source_frame_id}: '
                f'age={base_goal.tray_age_sec:.3f}s lead={base_goal.lead_time_sec:.3f}s '
                f'tray_speed={base_goal.tray_speed_base_mmps:.1f} mm/s '
                f'tray_axis={base_goal.tray_axis_name} '
                f'axis_rz={base_goal.tray_axis_rz_deg:.1f}deg '
                f'ee_offset={base_goal.ee_angle_signed_deg:.1f}deg/{base_goal.ee_angle_direction_label} '
                f'goal_rz={base_goal.rz_deg:.1f}deg '
                f'z-up follow={zup_follow_distance_mm:.1f} mm. TF-only.'
            )
        except Exception as exc:
            self.get_logger().error(f'Preview goal computation failed: {exc}')
            self._set_action_text(f'Preview goal computation failed: {exc}')
        finally:
            with self._lock:
                self._preview_inflight = False
            self._set_busy(False)

    def _arm_tray_vector_watch_locked(self) -> int:
        self._tray_watch_generation += 1
        self._tray_watch_armed = True
        self._tray_watch_seq_floor = self._tray_vector_seq
        self._tray_watch_deadline_monotonic = (
            time.monotonic() + max(
                TRAY_VECTOR_WATCH_TIMEOUT_MIN,
                min(TRAY_VECTOR_WATCH_TIMEOUT_MAX, float(self._tray_vector_watch_timeout_sec)),
            )
        )
        self._tray_watch_stop_dispatched = False
        return self._tray_watch_generation

    def _tray_vector_watchdog_worker(self, generation: int) -> None:
        while rclpy.ok():
            if self.count_publishers(TRAY_VECTOR_TOPIC) <= 0:
                self._reset_runtime_state(
                    f'No tray node detected on "{TRAY_VECTOR_TOPIC}". Node reset.'
                )
                return
            with self._lock:
                if self._tray_watch_generation != generation:
                    return
                if not self._tray_watch_armed:
                    return
                remaining_sec = self._tray_watch_deadline_monotonic - time.monotonic()
                if remaining_sec <= 0.0:
                    watch_timeout_sec = max(
                        TRAY_VECTOR_WATCH_TIMEOUT_MIN,
                        min(TRAY_VECTOR_WATCH_TIMEOUT_MAX, float(self._tray_vector_watch_timeout_sec)),
                    )
                    self._reset_runtime_state_locked(
                        f'No tray vector within {watch_timeout_sec:.0f}s. Node reset.'
                    )
                    return
            time.sleep(min(0.1, max(0.02, remaining_sec)))

    def _send_stop_and_movel_request(
        self,
        tray_target: TrayVectorTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        follow_distance_mm: float,
        post_follow_z_up_mm: float,
        ee_final_pose_angle_deg: float,
        release_grip_enabled: bool,
    ) -> None:
        seek_complete_notified = False
        busy_released_after_zup_queue = False
        try:
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled before dispatch.')
                return
            if not self._ensure_startup_motion_profile():
                return
            if not self._wait_for_service(self._stop_client, 'Stop'):
                return

            stop_request = Stop.Request()
            stop_response = self._call_service(self._stop_client, stop_request, 'Stop()')
            if stop_response is None:
                return
            if int(getattr(stop_response, 'res', -1)) < 0:
                return
            self._set_action_text('Computing tray goal in base frame...')

            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled after stop.')
                return
            if not self._wait_for_service(self._mov_l_client, 'MovL'):
                return
            if not self._wait_for_service(self._mov_lio_client, 'MovLIO'):
                return

            intercept_speed_mm_s = FIXED_EE_INTERCEPT_SPEED_MM_S
            base_goal = self._compute_base_goal_from_tray_target(
                tray_target,
                x_offset_mm,
                y_offset_mm,
                z_offset_mm,
                intercept_speed_mm_s,
                ee_final_pose_angle_deg,
            )
            if base_goal is None:
                return
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled during goal computation.')
                return

            post_z_up_speed_mm_s = POST_STOP_MOVL_SPEED_MAX
            follow_goal, post_follow_goal, zup_follow_distance_mm = self._compute_follow_and_post_z_up_goals(
                base_goal,
                follow_distance_mm,
                post_follow_z_up_mm,
                post_z_up_speed_mm_s,
            )
            follow_speed_mm_s = max(
                LINEAR_SPEED_MM_S_MIN,
                min(POST_STOP_MOVL_SPEED_MAX, float(base_goal.tray_speed_base_mmps)),
            )

            snapshot = self.snapshot()
            current_pose = (
                float(snapshot.tcp_values.get('x', 0.0)),
                float(snapshot.tcp_values.get('y', 0.0)),
                float(snapshot.tcp_values.get('z', 0.0)),
                float(snapshot.tcp_values.get('rx', 0.0)),
                float(snapshot.tcp_values.get('ry', 0.0)),
                float(snapshot.tcp_values.get('rz', 0.0)),
            )
            intercept_goal = (
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_goal_debug_transform(
                self._robot_goal_frame_id,
                self._post_stop_movel_goal_debug_frame_id,
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_goal_debug_transform(
                self._robot_goal_frame_id,
                self._follow_movel_goal_debug_frame_id,
                follow_goal[0],
                follow_goal[1],
                follow_goal[2],
                follow_goal[3],
                follow_goal[4],
                follow_goal[5],
            )
            self._publish_goal_debug_transform(
                self._robot_goal_frame_id,
                self._post_follow_zup_goal_debug_frame_id,
                post_follow_goal[0],
                post_follow_goal[1],
                post_follow_goal[2],
                post_follow_goal[3],
                post_follow_goal[4],
                post_follow_goal[5],
            )

            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled before queued tray motion dispatch.')
                return
            self._set_action_text('Queueing tray intercept motion commands...')
            intercept_ok, intercept_v, intercept_map = self._send_movel_goal(
                intercept_goal,
                current_pose,
                intercept_speed_mm_s,
                f'MovL intercept from {base_goal.source_frame_id}',
            )
            if not intercept_ok:
                return

            follow_mdis = self._build_release_follow_mdis() if release_grip_enabled else None
            if release_grip_enabled:
                follow_ok, follow_v, follow_map = self._send_movelio_goal(
                    follow_goal,
                    intercept_goal,
                    follow_speed_mm_s,
                    'MovLIO follow tray direction + release IO',
                    mdis=follow_mdis,
                )
                follow_cmd = 'MovLIO'
            else:
                follow_ok, follow_v, follow_map = self._send_movel_goal(
                    follow_goal,
                    intercept_goal,
                    follow_speed_mm_s,
                    'MovL follow tray direction',
                )
                follow_cmd = 'MovL'
            if not follow_ok:
                return

            zup_mdis = self._build_release_post_follow_mdis() if release_grip_enabled else None
            if release_grip_enabled:
                zup_ok, zup_v, zup_map = self._send_movelio_goal(
                    post_follow_goal,
                    follow_goal,
                    post_z_up_speed_mm_s,
                    'MovLIO post-follow z-up + release IO',
                    mdis=zup_mdis,
                )
                zup_cmd = 'MovLIO'
            else:
                zup_ok, zup_v, zup_map = self._send_movel_goal(
                    post_follow_goal,
                    follow_goal,
                    post_z_up_speed_mm_s,
                    'MovL post-follow z-up + tray follow',
                )
                zup_cmd = 'MovL'
            if not zup_ok:
                return

            seek_complete_notified = self._notify_tray_detect_seek_complete()
            busy_released_after_zup_queue = True
            self._set_busy(False)
            self._set_action_text(
                'Queued tray sequence through final post-follow Z-up. Ready for next arm.'
            )
            if not self._wait_for_tcp_xyz_goal(
                (post_follow_goal[0], post_follow_goal[1], post_follow_goal[2]),
                update_action_text=False,
            ):
                return

            self.get_logger().info(
                f'Completed queued tray sequence (MovL intercept + {follow_cmd} follow + {zup_cmd} post-follow): '
                f'intercept offsets (X {x_offset_mm:.0f}, Y {y_offset_mm:.0f}, Z {z_offset_mm:.0f} mm), '
                f'intercept {intercept_speed_mm_s:.0f} mm/s, follow {follow_distance_mm:.0f} mm '
                f'at tray speed {follow_speed_mm_s:.0f} mm/s, '
                f'post Z-up {post_follow_z_up_mm:.0f} mm with tray-follow {zup_follow_distance_mm:.1f} mm. '
                f'tray_axis={base_goal.tray_axis_name} '
                f'axis_rz={base_goal.tray_axis_rz_deg:.1f}deg '
                f'ee_offset={base_goal.ee_angle_signed_deg:.1f}deg/{base_goal.ee_angle_direction_label} '
                f'goal_rz={base_goal.rz_deg:.1f}deg '
                f'release_grip={"on" if release_grip_enabled else "off"} '
                f'age={base_goal.tray_age_sec:.3f}s lead={base_goal.lead_time_sec:.3f}s '
                f'tray_speed={base_goal.tray_speed_base_mmps:.1f} mm/s '
                f'(v: intercept={intercept_v}/{intercept_map}, '
                f'follow={follow_v}/{follow_map}, z-up={zup_v}/{zup_map}).'
            )
        except Exception as exc:
            self.get_logger().error(f'MovL predicted-goal flow failed: {exc}')
            self._set_action_text(f'MovL predicted-goal flow failed: {exc}')
        finally:
            if not seek_complete_notified:
                self._notify_tray_detect_seek_complete()
            if not busy_released_after_zup_queue:
                self._set_busy(False)

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
        started = self.run_tray_sequence(
            float(request.tray_vector_wait_timeout_sec),
            float(request.ee_intercept_speed_mm_s),
            float(request.tray_intercept_x_offset_mm),
            float(request.tray_intercept_y_offset_mm),
            float(request.tray_standoff_z_mm),
            float(request.follow_distance_mm),
            float(request.post_follow_z_up_mm),
            bool(request.troubleshoot_tf_only),
        )
        with self._lock:
            response.started = bool(started)
            response.message = str(self._snapshot.action_text)
            response.applied_tray_vector_wait_timeout_sec = float(self._tray_vector_watch_timeout_sec)
            response.applied_ee_intercept_speed_mm_s = float(self._post_stop_movel_speed_mm_s)
            response.applied_tray_intercept_x_offset_mm = float(self._post_stop_x_offset_mm)
            response.applied_tray_intercept_y_offset_mm = float(self._post_stop_y_offset_mm)
            response.applied_tray_standoff_z_mm = float(self._post_stop_z_offset_mm)
            response.applied_follow_distance_mm = float(self._follow_distance_mm)
            response.applied_post_follow_z_up_mm = float(self._post_follow_z_up_mm)
            response.applied_troubleshoot_tf_only = bool(self._tray_watch_tf_only_mode)
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
            armed = bool(self._tray_watch_armed)
            busy = bool(self._snapshot.busy)
            action_text = str(self._snapshot.action_text)

        response.success = armed
        if armed:
            response.message = f'Track armed: waiting for "{TRAY_VECTOR_TOPIC}". {action_text}'
        elif busy:
            response.message = f'Track busy but not armed. {action_text}'
        else:
            response.message = f'Track not armed. {action_text}'
        return response

    def run_track_from_current_settings(self) -> bool:
        with self._lock:
            tray_vector_watch_timeout_sec = float(self._tray_vector_watch_timeout_sec)
            post_stop_x_offset_mm = float(self._post_stop_x_offset_mm)
            post_stop_y_offset_mm = float(self._post_stop_y_offset_mm)
            post_stop_z_offset_mm = float(self._post_stop_z_offset_mm)
            follow_distance_mm = float(self._follow_distance_mm)
            post_follow_z_up_mm = float(self._post_follow_z_up_mm)
            tf_only_mode = bool(self._tray_watch_tf_only_mode)
            ee_final_pose_angle_deg = float(self._ee_final_pose_angle_deg)

        return self.run_tray_sequence(
            tray_vector_watch_timeout_sec,
            FIXED_EE_INTERCEPT_SPEED_MM_S,
            post_stop_x_offset_mm,
            post_stop_y_offset_mm,
            post_stop_z_offset_mm,
            follow_distance_mm,
            post_follow_z_up_mm,
            tf_only_mode,
            ee_final_pose_angle_deg,
        )

    def preview_from_last_tray(
        self,
        ee_final_pose_angle_deg: float,
        post_stop_x_offset_mm: float,
        post_stop_y_offset_mm: float,
        post_stop_z_offset_mm: float,
        follow_distance_mm: float,
        post_follow_z_up_mm: float,
    ) -> bool:
        with self._lock:
            if self._snapshot.busy or self._preview_inflight:
                return False
            tray_target = self._last_tray_target
            if tray_target is None:
                self._snapshot.action_text = 'No tray pose received yet for troubleshoot preview.'
                return False

            ee_angle_deg = max(
                EE_FINAL_POSE_ANGLE_MIN_DEG,
                min(EE_FINAL_POSE_ANGLE_MAX_DEG, float(ee_final_pose_angle_deg)),
            )
            x_offset_mm = max(
                POST_STOP_X_OFFSET_MIN,
                min(POST_STOP_X_OFFSET_MAX, float(post_stop_x_offset_mm)),
            )
            y_offset_mm = max(
                POST_STOP_Y_OFFSET_MIN,
                min(POST_STOP_Y_OFFSET_MAX, float(post_stop_y_offset_mm)),
            )
            z_offset_mm = max(
                POST_STOP_Z_OFFSET_MIN,
                min(POST_STOP_Z_OFFSET_MAX, float(post_stop_z_offset_mm)),
            )
            follow_distance = max(
                FOLLOW_DISTANCE_MIN,
                min(FOLLOW_DISTANCE_MAX, float(follow_distance_mm)),
            )
            post_follow_z_up = max(
                POST_FOLLOW_Z_UP_MIN,
                min(POST_FOLLOW_Z_UP_MAX, float(post_follow_z_up_mm)),
            )
            self._preview_inflight = True
            self._post_stop_movel_speed_mm_s = FIXED_EE_INTERCEPT_SPEED_MM_S
            self._ee_final_pose_angle_deg = ee_angle_deg

        worker = threading.Thread(
            target=self._preview_goal_only_request,
            args=(
                tray_target,
                x_offset_mm,
                y_offset_mm,
                z_offset_mm,
                follow_distance,
                post_follow_z_up,
                ee_angle_deg,
            ),
            daemon=True,
        )
        worker.start()
        return True

    def _is_cancel_requested(self) -> bool:
        with self._lock:
            return bool(self._cancel_requested)

    def is_manual_stop_inflight(self) -> bool:
        with self._lock:
            return bool(self._manual_stop_inflight)

    def request_manual_stop(self) -> bool:
        with self._lock:
            if self._manual_stop_inflight:
                self._snapshot.action_text = 'Manual Stop already in progress.'
                return False
            self._manual_stop_inflight = True
            self._cancel_requested = True
            self._tray_watch_generation += 1
            self._tray_watch_armed = False
            self._tray_watch_stop_dispatched = False
            self._tray_watch_deadline_monotonic = 0.0

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
            self._notify_tray_detect_seek_complete()
            with self._lock:
                self._snapshot.busy = False
                self._manual_stop_inflight = False

    def _notify_tray_detect_seek_complete(self) -> bool:
        client = self._tray_seek_complete_client
        if client is None:
            return False
        try:
            if not (client.service_is_ready() or client.wait_for_service(timeout_sec=0.05)):
                return False
            future = client.call_async(Trigger.Request())
            started = time.time()
            while rclpy.ok() and not future.done():
                if (time.time() - started) >= 1.0:
                    self.get_logger().warn(
                        f'Timed out notifying tray detect seek completion: {self._tray_seek_complete_service_name}'
                    )
                    return False
                time.sleep(0.02)
            if future.exception() is not None:
                self.get_logger().warn(
                    f'Failed notifying tray detect seek completion: {future.exception()}'
                )
                return False
            response = future.result()
            if response is not None and not bool(response.success):
                self.get_logger().warn(f'Tray detect seek-complete rejected: {response.message}')
                return False
            return response is not None
        except Exception as exc:
            self.get_logger().warn(f'Failed notifying tray detect seek completion: {exc}')
            return False

    def _ensure_startup_motion_profile(self) -> bool:
        with self._lock:
            if self._startup_motion_profile_applied:
                return True
            cp_value = self._calibration_startup_cp
            sf_value = self._calibration_startup_speed_factor
        if cp_value is None or sf_value is None:
            with self._lock:
                self._startup_motion_profile_applied = True
            self.get_logger().warn(
                'Skipping CP/SpeedFactor enforcement because calibration metadata is missing.'
            )
            return True

        if not self._wait_for_service(
            self._cp_client,
            'CP',
            timeout_sec=MOTION_PROFILE_ENFORCE_WAIT_SEC,
        ):
            self._set_action_text('CP service not ready (startup profile).')
            return False
        if not self._wait_for_service(
            self._speed_factor_client,
            'SpeedFactor',
            timeout_sec=MOTION_PROFILE_ENFORCE_WAIT_SEC,
        ):
            self._set_action_text('SpeedFactor service not ready (startup profile).')
            return False

        cp_request = CP.Request()
        cp_request.r = int(cp_value)
        cp_response = self._call_service(
            self._cp_client,
            cp_request,
            f'CP({cp_request.r}) [startup profile]',
            timeout_sec=MOTION_PROFILE_ENFORCE_CALL_SEC,
        )
        if cp_response is None or int(getattr(cp_response, 'res', -1)) < 0:
            self._set_action_text('Failed to apply startup CP from calibration profile.')
            return False

        sf_request = SpeedFactor.Request()
        sf_request.ratio = int(sf_value)
        sf_response = self._call_service(
            self._speed_factor_client,
            sf_request,
            f'SpeedFactor({sf_request.ratio}) [startup profile]',
            timeout_sec=MOTION_PROFILE_ENFORCE_CALL_SEC,
        )
        if sf_response is None or int(getattr(sf_response, 'res', -1)) < 0:
            self._set_action_text('Failed to apply startup SpeedFactor from calibration profile.')
            return False

        with self._lock:
            self._startup_motion_profile_applied = True
        self.get_logger().info(
            f'Applied startup motion profile from calibration: CP={cp_value}, SpeedFactor={sf_value}.'
        )
        return True

    def _build_motion_param_value(self, v_percent: int, a_percent: int, include_tool: bool = True) -> list[str]:
        args = [f'v={int(v_percent)}', f'a={int(a_percent)}']
        if include_tool:
            args.append('tool=1')
        return [','.join(args)]

    def get_command_hysteresis_sec(self) -> float:
        with self._lock:
            return float(self._command_hysteresis_sec)

    def set_command_hysteresis_sec(self, command_hysteresis_sec: float) -> float:
        with self._lock:
            self._command_hysteresis_sec = max(
                COMMAND_HYSTERESIS_MIN_SEC,
                min(COMMAND_HYSTERESIS_MAX_SEC, float(command_hysteresis_sec)),
            )
            return float(self._command_hysteresis_sec)

    def get_release_grip_enabled(self) -> bool:
        with self._lock:
            return bool(self._release_grip_enabled)

    def set_release_grip_enabled(self, enabled: bool) -> bool:
        with self._lock:
            self._release_grip_enabled = bool(enabled)
            return bool(self._release_grip_enabled)

    def set_preview_tray_dimensions(self, length_mm: float, width_mm: float) -> tuple[float, float]:
        with self._lock:
            self._tray_preview_length_mm = max(
                TRAY_PREVIEW_LENGTH_MIN_MM,
                min(TRAY_PREVIEW_LENGTH_MAX_MM, float(length_mm)),
            )
            self._tray_preview_width_mm = max(
                TRAY_PREVIEW_WIDTH_MIN_MM,
                min(TRAY_PREVIEW_WIDTH_MAX_MM, float(width_mm)),
            )
            return float(self._tray_preview_length_mm), float(self._tray_preview_width_mm)

    def run_tray_sequence(
        self,
        tray_vector_watch_timeout_sec: float,
        post_stop_movel_speed_mm_s: float,
        post_stop_x_offset_mm: float,
        post_stop_y_offset_mm: float,
        post_stop_z_offset_mm: float,
        follow_distance_mm: float,
        post_follow_z_up_mm: float,
        tf_only_mode: bool,
        ee_final_pose_angle_deg: float | None = None,
    ) -> bool:
        if self.count_publishers(TRAY_VECTOR_TOPIC) <= 0:
            self._reset_runtime_state(
                f'No tray node detected on "{TRAY_VECTOR_TOPIC}". Node reset.'
            )
            return False

        with self._lock:
            _ = post_stop_movel_speed_mm_s
            if self._snapshot.busy:
                self._snapshot.action_text = 'Busy running previous tray sequence.'
                return False
            self._snapshot.busy = True
            self._cancel_requested = False
            self._tray_vector_watch_timeout_sec = max(
                TRAY_VECTOR_WATCH_TIMEOUT_MIN,
                min(TRAY_VECTOR_WATCH_TIMEOUT_MAX, float(tray_vector_watch_timeout_sec)),
            )
            self._post_stop_movel_speed_mm_s = FIXED_EE_INTERCEPT_SPEED_MM_S
            if ee_final_pose_angle_deg is not None:
                self._ee_final_pose_angle_deg = max(
                    EE_FINAL_POSE_ANGLE_MIN_DEG,
                    min(EE_FINAL_POSE_ANGLE_MAX_DEG, float(ee_final_pose_angle_deg)),
                )
            self._post_stop_x_offset_mm = max(
                POST_STOP_X_OFFSET_MIN,
                min(POST_STOP_X_OFFSET_MAX, float(post_stop_x_offset_mm)),
            )
            self._post_stop_y_offset_mm = max(
                POST_STOP_Y_OFFSET_MIN,
                min(POST_STOP_Y_OFFSET_MAX, float(post_stop_y_offset_mm)),
            )
            self._post_stop_z_offset_mm = max(
                POST_STOP_Z_OFFSET_MIN,
                min(POST_STOP_Z_OFFSET_MAX, float(post_stop_z_offset_mm)),
            )
            self._follow_distance_mm = max(
                FOLLOW_DISTANCE_MIN,
                min(FOLLOW_DISTANCE_MAX, float(follow_distance_mm)),
            )
            self._post_follow_z_up_mm = max(
                POST_FOLLOW_Z_UP_MIN,
                min(POST_FOLLOW_Z_UP_MAX, float(post_follow_z_up_mm)),
            )
            self._tray_watch_tf_only_mode = bool(tf_only_mode)
            generation = self._arm_tray_vector_watch_locked()
            watch_timeout_sec = float(self._tray_vector_watch_timeout_sec)
            mode_name = 'tf_only' if tf_only_mode else 'normal'
            self.get_logger().info(
                'Run settings: '
                f'mode={mode_name} '
                f'wait={watch_timeout_sec:.0f}s '
                f'fixed_speed={self._post_stop_movel_speed_mm_s:.0f} mm/s '
                f'ee_angle_offset={self._ee_final_pose_angle_deg:.0f} deg '
                f'offsets(x={self._post_stop_x_offset_mm:.0f},'
                f'y={self._post_stop_y_offset_mm:.0f},'
                f'z={self._post_stop_z_offset_mm:.0f}) mm '
                f'hysteresis={self._command_hysteresis_sec:.2f}s '
                f'follow={self._follow_distance_mm:.0f} mm '
                f'post_z_up={self._post_follow_z_up_mm:.0f} mm '
                f'release_grip={"on" if self._release_grip_enabled else "off"}'
            )
            if tf_only_mode:
                self._snapshot.action_text = (
                    f'Troubleshoot mode armed... waiting for "{TRAY_VECTOR_TOPIC}" '
                    f'for {watch_timeout_sec:.0f}s (TF preview only).'
                )
            else:
                self._snapshot.action_text = (
                    f'Tray sequence armed... waiting for "{TRAY_VECTOR_TOPIC}" '
                    f'for {watch_timeout_sec:.0f}s.'
                )

        watchdog = threading.Thread(
            target=self._tray_vector_watchdog_worker,
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

    def fetch_tray_dimensions(self, quiet: bool = False) -> tuple[float, float, bool, str, str] | None:
        label = 'GetTrayDimensions'
        if not self.is_tray_dimensions_service_available():
            if not quiet:
                self._set_action_text(f'{label} service not ready.')
            return None
        if not self._wait_for_service(
            self._tray_dimensions_client,
            label,
            timeout_sec=TRAY_DIMENSIONS_SERVICE_WAIT_SEC,
        ):
            return None

        if not quiet:
            self._set_action_text(f'SEND {label}')
        future = self._tray_dimensions_client.call_async(GetTrayDimensions.Request())
        started = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - started) >= TRAY_DIMENSIONS_SERVICE_CALL_SEC:
                if not quiet:
                    self._set_action_text(f'Timeout: {label}')
                return None
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            if not quiet:
                self._set_action_text(f'Exception: {label}: {exception}')
            return None

        response = future.result()
        if response is None:
            if not quiet:
                self._set_action_text(f'No response: {label}')
            return None

        message = str(getattr(response, 'message', '')).strip()
        if not bool(getattr(response, 'success', False)):
            if not quiet:
                self._set_action_text(message or 'Tray dimensions unavailable.')
            return None

        x_size_mm = float(getattr(response, 'x_size_mm', 0.0))
        y_size_mm = float(getattr(response, 'y_size_mm', 0.0))
        live_detection = bool(getattr(response, 'live_detection', False))
        tray_name = str(getattr(response, 'tray_name', '')).strip()
        source_label = 'live detect' if live_detection else 'taught profile'
        if not quiet:
            if message:
                self._set_action_text(f'OK {label}: {message}')
            else:
                self._set_action_text(f'OK {label}: {source_label}')
        return x_size_mm, y_size_mm, live_detection, tray_name, message

    def is_tray_dimensions_service_available(self) -> bool:
        try:
            return bool(
                self._tray_dimensions_client.service_is_ready()
                or self._tray_dimensions_client.wait_for_service(timeout_sec=0.0)
            )
        except Exception:
            return False

    def _load_speed_calibration(self) -> dict[str, list[tuple[float, int]]]:
        calibration_path = self._resolve_speed_calibration_path()
        if calibration_path is None:
            return {}
        self._speed_calibration_path = calibration_path

        try:
            with open(calibration_path, 'r', encoding='utf-8') as calibration_file:
                root = json.load(calibration_file)
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to read speed calibration "{calibration_path}": {exc}')
            return {}

        self._extract_startup_motion_profile(root)

        axis_models = root.get('axis_models')
        if not isinstance(axis_models, dict):
            return {}

        lookup: dict[str, list[tuple[float, int]]] = {}
        sample_source_by_axis: dict[str, str] = {}
        for axis_name in TRANSLATION_AXES:
            axis_node = axis_models.get(axis_name)
            if not isinstance(axis_node, dict):
                continue
            fit_samples = axis_node.get('fit_samples')
            if isinstance(fit_samples, list) and fit_samples:
                samples = fit_samples
                sample_source_by_axis[axis_name] = 'fit_samples'
            else:
                samples = axis_node.get('samples')
                sample_source_by_axis[axis_name] = 'samples'
            if not isinstance(samples, list):
                continue

            pairs: list[tuple[float, int]] = []
            for sample in samples:
                if not isinstance(sample, dict):
                    continue
                try:
                    v_percent = int(round(float(sample.get('v_percent'))))
                    speed_mm_s = float(sample.get('speed_mm_s'))
                except (TypeError, ValueError):
                    continue
                if speed_mm_s <= 0.0:
                    continue
                v_percent = max(1, min(100, v_percent))
                pairs.append((speed_mm_s, v_percent))

            if not pairs:
                continue

            # Keep only one entry per v% (highest observed speed), then sort by speed for interpolation.
            by_v_percent: dict[int, float] = {}
            for speed_mm_s, v_percent in pairs:
                current = by_v_percent.get(v_percent)
                if current is None or speed_mm_s > current:
                    by_v_percent[v_percent] = speed_mm_s
            normalized_pairs = [(speed, v) for v, speed in by_v_percent.items()]
            normalized_pairs.sort(key=lambda item: item[0])
            lookup[axis_name] = normalized_pairs

        if lookup:
            loaded_axes = ', '.join(sorted(lookup.keys()))
            source_summary = ', '.join(
                f'{axis}:{sample_source_by_axis.get(axis, "samples")}' for axis in sorted(lookup.keys())
            )
            self.get_logger().info(
                f'Loaded RelMovL speed calibration from "{calibration_path}" for axes: {loaded_axes} '
                f'(source {source_summary})')
        return lookup

    @staticmethod
    def _parse_percent_value(raw_value: object) -> int | None:
        try:
            value = int(round(float(raw_value)))
        except (TypeError, ValueError):
            return None
        return max(1, min(100, value))

    def _extract_startup_motion_profile(self, root: dict) -> None:
        cp_candidate: object | None = None
        sf_candidate: object | None = None
        applied = root.get('applied_startup_settings')
        if isinstance(applied, dict):
            cp_candidate = applied.get('cp_percent')
            sf_candidate = applied.get('speed_factor_percent')
        if cp_candidate is None:
            cp_candidate = root.get('startup_cp')
        if sf_candidate is None:
            sf_candidate = root.get('startup_speed_factor')

        self._calibration_startup_cp = self._parse_percent_value(cp_candidate)
        self._calibration_startup_speed_factor = self._parse_percent_value(sf_candidate)

    def _resolve_speed_calibration_path(self) -> Path | None:
        candidates: list[Path] = []
        try:
            if CALIBRATION_DIR_PATH.exists():
                for path in CALIBRATION_DIR_PATH.glob('relmovl_speed_calibration*.json'):
                    if path.is_file() and path.stat().st_size > 0:
                        candidates.append(path)
        except Exception as exc:
            self.get_logger().warn(
                f'Failed scanning calibration directory "{CALIBRATION_DIR_PATH}": {exc}')

        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime)
        if CALIBRATION_FILE_PATH.exists():
            return CALIBRATION_FILE_PATH
        return None

    def _dominant_linear_axis(self, delta_goal: list[float]) -> str:
        dominant_axis = 'x'
        dominant_magnitude = -1.0
        for axis_index, axis_name in enumerate(TRANSLATION_AXES):
            if axis_index >= len(delta_goal):
                break
            magnitude = abs(float(delta_goal[axis_index]))
            if magnitude > dominant_magnitude:
                dominant_magnitude = magnitude
                dominant_axis = axis_name
        return dominant_axis

    def _interpolate_calibrated_v_percent(self, speed_mm_s: float, axis_name: str) -> int | None:
        points = self._axis_speed_lookup.get(axis_name)
        if not points:
            return None
        if len(points) == 1:
            return points[0][1]

        clamped_speed = max(LINEAR_SPEED_MM_S_MIN, min(LINEAR_SPEED_MM_S_MAX, float(speed_mm_s)))
        if clamped_speed <= points[0][0]:
            return points[0][1]
        if clamped_speed >= points[-1][0]:
            return points[-1][1]

        for index in range(1, len(points)):
            left_speed, left_v = points[index - 1]
            right_speed, right_v = points[index]
            if clamped_speed > right_speed:
                continue
            speed_span = right_speed - left_speed
            if speed_span <= 1e-9:
                return left_v
            ratio = (clamped_speed - left_speed) / speed_span
            interpolated = left_v + ratio * (right_v - left_v)
            return max(1, min(100, int(round(interpolated))))
        return None

    def _mm_s_to_v_percent(self, speed_mm_s: float, axis_name: str) -> tuple[int, str]:
        calibrated = self._interpolate_calibrated_v_percent(speed_mm_s, axis_name)
        if calibrated is not None:
            return calibrated, f'cal:{axis_name}'

        clamped = max(LINEAR_SPEED_MM_S_MIN, min(LINEAR_SPEED_MM_S_MAX, float(speed_mm_s)))
        ratio = clamped / LINEAR_SPEED_MM_S_MAX
        v_percent = int(round(ratio * 100.0))
        return max(1, min(100, v_percent)), 'linear'


class RelMovLMiniGui:
    def __init__(self, node: RelMovLMiniNode) -> None:
        self.node = node
        self._gui_thread_id = threading.get_ident()
        self.root = tk.Tk()
        self.root.title('Tray Intercept Operator Console')
        fixed_width = 1320
        fixed_height = 760
        self.root.geometry(f'{fixed_width}x{fixed_height}')
        self.root.minsize(fixed_width, fixed_height)
        self.root.maxsize(fixed_width, fixed_height)
        self.root.resizable(False, False)
        self._closed = False
        self._last_preview_signature: tuple | None = None
        self._preview_canvas_transform: dict[str, object] | None = None
        self._runtime_settings_path = RUNTIME_SETTINGS_PATH
        self._runtime_settings_save_after_id: str | None = None
        self._suspend_runtime_settings_events = False
        self._fetch_tray_size_inflight = False
        self._last_tray_size_fetch_monotonic = 0.0

        outer = tk.Frame(self.root, padx=12, pady=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1, uniform='maincols')
        outer.columnconfigure(1, weight=1, uniform='maincols')
        outer.columnconfigure(2, weight=1, uniform='maincols')
        outer.rowconfigure(1, weight=1)

        modes_frame = tk.LabelFrame(outer, text='Operating Modes', padx=10, pady=8)
        modes_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        modes_frame.columnconfigure(0, weight=1)

        self.run_button = tk.Button(
            modes_frame,
            text='Arm Track Tray',
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

        self.tf_only_var = tk.BooleanVar(value=True)
        self.tf_only_button = tk.Button(
            modes_frame,
            command=self._toggle_tf_only_clicked,
            width=24,
        )
        self.tf_only_button.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        self._tf_only_default_bg = self.tf_only_button.cget('bg')
        self._tf_only_default_fg = self.tf_only_button.cget('fg')
        self._tf_only_default_active_bg = self.tf_only_button.cget('activebackground')
        self._tf_only_default_active_fg = self.tf_only_button.cget('activeforeground')
        self._sync_tf_only_button(is_busy=False)

        tk.Label(modes_frame, text='Tray vector wait timeout (sec)').grid(row=3, column=0, sticky='w', pady=(10, 0))
        self.tray_watch_timeout_var = tk.DoubleVar(value=TRAY_VECTOR_WATCH_TIMEOUT_SEC)
        self.tray_watch_timeout_scale = tk.Scale(
            modes_frame,
            from_=TRAY_VECTOR_WATCH_TIMEOUT_MIN,
            to=TRAY_VECTOR_WATCH_TIMEOUT_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.tray_watch_timeout_var,
            showvalue=True,
        )
        self.tray_watch_timeout_scale.grid(row=4, column=0, sticky='ew')

        mode_hint = (
            'Press Arm Track Tray, then wait for tray vector. '
            'Normal mode queues Stop + MovL intercept, then MovL/MovLIO follow and post-follow.'
        )
        tk.Label(
            modes_frame,
            text=mode_hint,
            anchor='w',
            justify=tk.LEFT,
            wraplength=520,
        ).grid(row=5, column=0, sticky='w', pady=(8, 0))
        self.action_var = tk.StringVar(value='Ready')
        tk.Label(
            modes_frame,
            textvariable=self.action_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=520,
        ).grid(row=6, column=0, sticky='ew', pady=(8, 0))

        ee_settings_frame = tk.LabelFrame(outer, text='EE Position Settings', padx=10, pady=8)
        ee_settings_frame.grid(row=0, column=1, sticky='nsew', padx=(0, 8))
        ee_settings_frame.columnconfigure(0, weight=1)

        tk.Label(ee_settings_frame, text='Tray intercept X offset (+tray X, mm)').grid(row=0, column=0, sticky='w')
        self.post_stop_x_offset_var = tk.DoubleVar(value=0.0)
        self.post_stop_x_offset_scale = tk.Scale(
            ee_settings_frame,
            from_=POST_STOP_X_OFFSET_MIN,
            to=POST_STOP_X_OFFSET_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.post_stop_x_offset_var,
            showvalue=True,
        )
        self.post_stop_x_offset_scale.grid(row=1, column=0, sticky='ew')

        tk.Label(ee_settings_frame, text='Tray intercept Y offset (+tray Y, mm)').grid(row=2, column=0, sticky='w', pady=(10, 0))
        self.post_stop_y_offset_var = tk.DoubleVar(value=0.0)
        self.post_stop_y_offset_scale = tk.Scale(
            ee_settings_frame,
            from_=POST_STOP_Y_OFFSET_MIN,
            to=POST_STOP_Y_OFFSET_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.post_stop_y_offset_var,
            showvalue=True,
        )
        self.post_stop_y_offset_scale.grid(row=3, column=0, sticky='ew')

        tk.Label(ee_settings_frame, text='Tray stand-off (+base Z, mm)').grid(row=4, column=0, sticky='w', pady=(10, 0))
        self.post_stop_z_offset_var = tk.DoubleVar(value=100.0)
        self.post_stop_z_offset_scale = tk.Scale(
            ee_settings_frame,
            from_=POST_STOP_Z_OFFSET_MIN,
            to=POST_STOP_Z_OFFSET_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.post_stop_z_offset_var,
            showvalue=True,
        )
        self.post_stop_z_offset_scale.grid(row=5, column=0, sticky='ew')

        tk.Label(ee_settings_frame, text='Command hysteresis (sec)').grid(row=6, column=0, sticky='w', pady=(10, 0))
        self.command_hysteresis_var = tk.DoubleVar(value=self.node.get_command_hysteresis_sec())
        self.command_hysteresis_scale = tk.Scale(
            ee_settings_frame,
            from_=COMMAND_HYSTERESIS_MIN_SEC,
            to=COMMAND_HYSTERESIS_MAX_SEC,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=280,
            variable=self.command_hysteresis_var,
            showvalue=True,
        )
        self.command_hysteresis_scale.grid(row=7, column=0, sticky='ew')

        settings_frame = tk.LabelFrame(outer, text='Tray Intercept Settings', padx=10, pady=8)
        settings_frame.grid(row=0, column=2, sticky='nsew')
        settings_frame.columnconfigure(0, weight=1)

        tk.Label(
            settings_frame,
            text=f'EE intercept speed fixed: {FIXED_EE_INTERCEPT_SPEED_MM_S:.0f} mm/s',
        ).grid(row=0, column=0, sticky='w')

        tk.Label(settings_frame, text='EE vertical-axis angle offset (deg, -CCW / +CW)').grid(row=1, column=0, sticky='w', pady=(10, 0))
        self.ee_final_pose_angle_var = tk.DoubleVar(value=EE_FINAL_POSE_ANGLE_DEFAULT_DEG)
        self.ee_final_pose_angle_scale = tk.Scale(
            settings_frame,
            from_=EE_FINAL_POSE_ANGLE_MIN_DEG,
            to=EE_FINAL_POSE_ANGLE_MAX_DEG,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.ee_final_pose_angle_var,
            showvalue=True,
        )
        self.ee_final_pose_angle_scale.grid(row=2, column=0, sticky='ew')

        tk.Label(settings_frame, text='Follow tray distance (mm)').grid(row=3, column=0, sticky='w', pady=(10, 0))
        self.follow_distance_var = tk.DoubleVar(value=200.0)
        self.follow_distance_scale = tk.Scale(
            settings_frame,
            from_=FOLLOW_DISTANCE_MIN,
            to=FOLLOW_DISTANCE_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.follow_distance_var,
            showvalue=True,
        )
        self.follow_distance_scale.grid(row=4, column=0, sticky='ew')

        tk.Label(
            settings_frame,
            text='Follow uses tray speed; post Z-up uses arm max',
        ).grid(row=5, column=0, sticky='w', pady=(8, 0))

        tk.Label(settings_frame, text='Post-follow Z-up (mm)').grid(row=6, column=0, sticky='w', pady=(10, 0))
        self.post_follow_z_up_var = tk.DoubleVar(value=300.0)
        self.post_follow_z_up_scale = tk.Scale(
            settings_frame,
            from_=POST_FOLLOW_Z_UP_MIN,
            to=POST_FOLLOW_Z_UP_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.post_follow_z_up_var,
            showvalue=True,
        )
        self.post_follow_z_up_scale.grid(row=7, column=0, sticky='ew')

        self.release_grip_var = tk.BooleanVar(value=self.node.get_release_grip_enabled())
        self.release_grip_button = tk.Button(
            settings_frame,
            command=self._toggle_release_grip_clicked,
            width=24,
        )
        self.release_grip_button.grid(row=8, column=0, sticky='ew', pady=(10, 0))
        self._release_grip_default_bg = self.release_grip_button.cget('bg')
        self._release_grip_default_fg = self.release_grip_button.cget('fg')
        self._release_grip_default_active_bg = self.release_grip_button.cget('activebackground')
        self._release_grip_default_active_fg = self.release_grip_button.cget('activeforeground')
        self._sync_release_grip_button(is_busy=False)

        preview_controls_frame = tk.LabelFrame(outer, text='Tray Intercept Preview Controls', padx=10, pady=8)
        preview_controls_frame.grid(row=1, column=0, sticky='nsew', pady=(10, 0), padx=(0, 8))
        preview_controls_frame.columnconfigure(0, weight=1)

        preview_canvas_frame = tk.LabelFrame(outer, text='Tray Intercept Preview', padx=10, pady=8)
        preview_canvas_frame.grid(row=1, column=1, columnspan=2, sticky='nsew', pady=(10, 0))
        preview_canvas_frame.columnconfigure(0, weight=1)
        preview_canvas_frame.rowconfigure(0, weight=1)

        self.preview_canvas = tk.Canvas(
            preview_canvas_frame,
            bg='#141414',
            highlightthickness=1,
            highlightbackground='#4d4d4d',
            width=900,
            height=230,
        )
        self.preview_canvas.grid(row=0, column=0, sticky='nsew')
        self.preview_canvas.bind('<Configure>', lambda _event: self._draw_intercept_preview())
        self.preview_canvas.bind('<Button-1>', self._on_preview_canvas_clicked)

        preview_controls = tk.Frame(preview_controls_frame)
        preview_controls.grid(row=0, column=0, sticky='nsew')
        preview_controls.columnconfigure(0, weight=1)

        tk.Label(preview_controls, text='Tray Length (mm)').grid(row=0, column=0, sticky='w')
        self.tray_preview_length_var = tk.DoubleVar(value=TRAY_PREVIEW_LENGTH_MM)
        self.tray_preview_length_scale = tk.Scale(
            preview_controls,
            from_=TRAY_PREVIEW_LENGTH_MIN_MM,
            to=TRAY_PREVIEW_LENGTH_MAX_MM,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.tray_preview_length_var,
            showvalue=True,
            command=lambda _value: self._draw_intercept_preview(),
        )
        self.tray_preview_length_scale.grid(row=1, column=0, sticky='ew')

        tk.Label(preview_controls, text='Tray Width (mm)').grid(row=2, column=0, sticky='w', pady=(8, 0))
        self.tray_preview_width_var = tk.DoubleVar(value=TRAY_PREVIEW_WIDTH_MM)
        self.tray_preview_width_scale = tk.Scale(
            preview_controls,
            from_=TRAY_PREVIEW_WIDTH_MIN_MM,
            to=TRAY_PREVIEW_WIDTH_MAX_MM,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.tray_preview_width_var,
            showvalue=True,
            command=lambda _value: self._draw_intercept_preview(),
        )
        self.tray_preview_width_scale.grid(row=3, column=0, sticky='ew')

        tk.Label(preview_controls, text='Tray Border (mm)').grid(row=4, column=0, sticky='w', pady=(8, 0))
        self.tray_preview_border_var = tk.DoubleVar(value=TRAY_PREVIEW_BORDER_THICKNESS_MM)
        self.tray_preview_border_scale = tk.Scale(
            preview_controls,
            from_=TRAY_PREVIEW_BORDER_MIN_MM,
            to=TRAY_PREVIEW_BORDER_MAX_MM,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.tray_preview_border_var,
            showvalue=True,
            command=lambda _value: self._draw_intercept_preview(),
        )
        self.tray_preview_border_scale.grid(row=5, column=0, sticky='ew')

        tk.Label(preview_controls, text='Intercept Dot Diameter (mm)').grid(row=6, column=0, sticky='w', pady=(8, 0))
        self.intercept_dot_diameter_var = tk.DoubleVar(value=INTERCEPT_DOT_DIAMETER_DEFAULT_MM)
        self.intercept_dot_diameter_scale = tk.Scale(
            preview_controls,
            from_=INTERCEPT_DOT_DIAMETER_MIN_MM,
            to=INTERCEPT_DOT_DIAMETER_MAX_MM,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=280,
            variable=self.intercept_dot_diameter_var,
            showvalue=True,
            command=lambda _value: self._draw_intercept_preview(),
        )
        self.intercept_dot_diameter_scale.grid(row=7, column=0, sticky='ew')

        self.dot_info_var = tk.StringVar(value='Red dot Ø 10 mm (default)')
        tk.Label(
            preview_controls,
            textvariable=self.dot_info_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=260,
        ).grid(row=8, column=0, sticky='w', pady=(8, 0))

        self.border_info_var = tk.StringVar(value='Tray border thickness: 50 mm.')
        tk.Label(
            preview_controls,
            textvariable=self.border_info_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=260,
        ).grid(row=9, column=0, sticky='w', pady=(6, 0))

        tk.Label(
            preview_controls,
            text='Click the tray preview to place the intercept X/Y target.',
            anchor='w',
            justify=tk.LEFT,
            wraplength=260,
        ).grid(row=10, column=0, sticky='w', pady=(8, 0))

        self._arm_locked_setting_controls = [
            self.tray_watch_timeout_scale,
            self.post_stop_x_offset_scale,
            self.post_stop_y_offset_scale,
            self.post_stop_z_offset_scale,
            self.command_hysteresis_scale,
            self.ee_final_pose_angle_scale,
            self.follow_distance_scale,
            self.post_follow_z_up_scale,
            self.tray_preview_length_scale,
            self.tray_preview_width_scale,
            self.tray_preview_border_scale,
            self.intercept_dot_diameter_scale,
        ]

        self._register_runtime_setting_traces()
        self._load_runtime_settings()
        self.node.set_command_hysteresis_sec(float(self.command_hysteresis_var.get()))
        self.node.set_release_grip_enabled(bool(self.release_grip_var.get()))
        self.node.set_preview_tray_dimensions(
            float(self.tray_preview_length_var.get()),
            float(self.tray_preview_width_var.get()),
        )
        self._sync_tf_only_button(is_busy=False)
        self._sync_release_grip_button(is_busy=False)
        self.node.set_track_trigger_handler(self._track_clicked_from_service)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._maybe_auto_fetch_tray_size(force=True)
        self._draw_intercept_preview()
        self._refresh()

    def _track_clicked_from_service(self) -> tuple[bool, str]:
        if self._closed:
            return False, 'Tray intercept GUI is closed.'

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
        self._last_preview_signature = None
        self.node.set_command_hysteresis_sec(float(self.command_hysteresis_var.get()))
        self.node.set_preview_tray_dimensions(
            float(self.tray_preview_length_var.get()),
            float(self.tray_preview_width_var.get()),
        )
        tray_watch_timeout_value = float(self.tray_watch_timeout_var.get())
        ee_final_pose_angle_value = float(self.ee_final_pose_angle_var.get())
        post_stop_x_offset_value = float(self.post_stop_x_offset_var.get())
        post_stop_y_offset_value = float(self.post_stop_y_offset_var.get())
        post_stop_z_offset_value = float(self.post_stop_z_offset_var.get())
        follow_distance_value = float(self.follow_distance_var.get())
        post_follow_z_up_value = float(self.post_follow_z_up_var.get())
        release_grip_enabled = bool(self.release_grip_var.get())
        tf_only_mode = bool(self.tf_only_var.get())
        self.node.set_release_grip_enabled(release_grip_enabled)

        started = self.node.run_tray_sequence(
            tray_watch_timeout_value,
            FIXED_EE_INTERCEPT_SPEED_MM_S,
            post_stop_x_offset_value,
            post_stop_y_offset_value,
            post_stop_z_offset_value,
            follow_distance_value,
            post_follow_z_up_value,
            tf_only_mode,
            ee_final_pose_angle_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)
            return False
        else:
            self._set_stop_button_enabled(True)
            self.run_button.configure(state=tk.DISABLED)
            self._sync_tf_only_button(is_busy=True)
            self._sync_release_grip_button(is_busy=True)
            self._set_arm_locked_setting_controls_enabled(False)
            return True

    def _stop_clicked(self) -> None:
        self._last_preview_signature = None
        started = self.node.request_manual_stop()
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _toggle_tf_only_clicked(self) -> None:
        self._last_preview_signature = None
        current = bool(self.tf_only_var.get())
        self.tf_only_var.set(not current)
        self._sync_tf_only_button(is_busy=False)

    def _toggle_release_grip_clicked(self) -> None:
        self._last_preview_signature = None
        current = bool(self.release_grip_var.get())
        self.release_grip_var.set(not current)
        self._sync_release_grip_button(is_busy=False)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))

    def _is_ui_locked(self) -> bool:
        snapshot = self.node.snapshot()
        return bool(snapshot.armed or snapshot.busy or self.node.is_manual_stop_inflight())

    def _set_arm_locked_setting_controls_enabled(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for control in self._arm_locked_setting_controls:
            try:
                control.configure(state=state)
            except tk.TclError:
                pass

    def _register_runtime_setting_traces(self) -> None:
        tracked_vars = [
            self.tf_only_var,
            self.tray_watch_timeout_var,
            self.ee_final_pose_angle_var,
            self.post_stop_x_offset_var,
            self.post_stop_y_offset_var,
            self.post_stop_z_offset_var,
            self.command_hysteresis_var,
            self.follow_distance_var,
            self.post_follow_z_up_var,
            self.release_grip_var,
            self.tray_preview_length_var,
            self.tray_preview_width_var,
            self.tray_preview_border_var,
            self.intercept_dot_diameter_var,
        ]
        for var in tracked_vars:
            var.trace_add('write', self._on_runtime_setting_changed)

    def _collect_runtime_settings(self) -> dict:
        return {
            'schema_version': 2,
            'tf_only_mode': bool(self.tf_only_var.get()),
            'tray_vector_wait_timeout_sec': float(self.tray_watch_timeout_var.get()),
            'ee_intercept_speed_mm_s': float(FIXED_EE_INTERCEPT_SPEED_MM_S),
            'ee_final_pose_angle_deg': float(self.ee_final_pose_angle_var.get()),
            'tray_intercept_x_offset_mm': float(self.post_stop_x_offset_var.get()),
            'tray_intercept_y_offset_mm': float(self.post_stop_y_offset_var.get()),
            'tray_standoff_z_mm': float(self.post_stop_z_offset_var.get()),
            'command_hysteresis_sec': float(self.command_hysteresis_var.get()),
            'follow_distance_mm': float(self.follow_distance_var.get()),
            'post_follow_z_up_mm': float(self.post_follow_z_up_var.get()),
            'release_grip_enabled': bool(self.release_grip_var.get()),
            'preview_tray_length_mm': float(self.tray_preview_length_var.get()),
            'preview_tray_width_mm': float(self.tray_preview_width_var.get()),
            'preview_tray_border_mm': float(self.tray_preview_border_var.get()),
            'preview_intercept_dot_diameter_mm': float(self.intercept_dot_diameter_var.get()),
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
        self._last_preview_signature = None
        self.node.set_command_hysteresis_sec(float(self.command_hysteresis_var.get()))
        self.node.set_release_grip_enabled(bool(self.release_grip_var.get()))
        self.node.set_preview_tray_dimensions(
            float(self.tray_preview_length_var.get()),
            float(self.tray_preview_width_var.get()),
        )
        ui_locked = self._is_ui_locked()
        self._sync_tf_only_button(is_busy=ui_locked)
        self._sync_release_grip_button(is_busy=ui_locked)
        self._set_arm_locked_setting_controls_enabled(not ui_locked)
        self._draw_intercept_preview()
        self._schedule_runtime_settings_save()

    def _load_runtime_settings(self) -> None:
        if not self._runtime_settings_path.exists():
            return
        try:
            with self._runtime_settings_path.open('r', encoding='utf-8') as infile:
                payload = json.load(infile)
        except Exception as exc:
            self.node.get_logger().warn(
                f'Failed to read tray intercept runtime settings at "{self._runtime_settings_path}": {exc}'
            )
            return

        if not isinstance(payload, dict):
            return

        self._suspend_runtime_settings_events = True
        try:
            self.tf_only_var.set(bool(payload.get('tf_only_mode', True)))
            self.tray_watch_timeout_var.set(self._clamp(
                payload.get('tray_vector_wait_timeout_sec', TRAY_VECTOR_WATCH_TIMEOUT_SEC),
                TRAY_VECTOR_WATCH_TIMEOUT_MIN,
                TRAY_VECTOR_WATCH_TIMEOUT_MAX,
            ))
            self.ee_final_pose_angle_var.set(self._clamp(
                payload.get('ee_final_pose_angle_deg', EE_FINAL_POSE_ANGLE_DEFAULT_DEG),
                EE_FINAL_POSE_ANGLE_MIN_DEG,
                EE_FINAL_POSE_ANGLE_MAX_DEG,
            ))
            self.post_stop_x_offset_var.set(self._clamp(
                payload.get('tray_intercept_x_offset_mm', 0.0),
                POST_STOP_X_OFFSET_MIN,
                POST_STOP_X_OFFSET_MAX,
            ))
            self.post_stop_y_offset_var.set(self._clamp(
                payload.get('tray_intercept_y_offset_mm', 0.0),
                POST_STOP_Y_OFFSET_MIN,
                POST_STOP_Y_OFFSET_MAX,
            ))
            self.post_stop_z_offset_var.set(self._clamp(
                payload.get('tray_standoff_z_mm', 100.0),
                POST_STOP_Z_OFFSET_MIN,
                POST_STOP_Z_OFFSET_MAX,
            ))
            self.command_hysteresis_var.set(self._clamp(
                payload.get('command_hysteresis_sec', COMMAND_HYSTERESIS_DEFAULT_SEC),
                COMMAND_HYSTERESIS_MIN_SEC,
                COMMAND_HYSTERESIS_MAX_SEC,
            ))
            self.follow_distance_var.set(self._clamp(
                payload.get('follow_distance_mm', 200.0),
                FOLLOW_DISTANCE_MIN,
                FOLLOW_DISTANCE_MAX,
            ))
            self.post_follow_z_up_var.set(self._clamp(
                payload.get('post_follow_z_up_mm', 300.0),
                POST_FOLLOW_Z_UP_MIN,
                POST_FOLLOW_Z_UP_MAX,
            ))
            self.release_grip_var.set(bool(payload.get('release_grip_enabled', False)))
            self.tray_preview_length_var.set(self._clamp(
                payload.get('preview_tray_length_mm', TRAY_PREVIEW_LENGTH_MM),
                TRAY_PREVIEW_LENGTH_MIN_MM,
                TRAY_PREVIEW_LENGTH_MAX_MM,
            ))
            self.tray_preview_width_var.set(self._clamp(
                payload.get('preview_tray_width_mm', TRAY_PREVIEW_WIDTH_MM),
                TRAY_PREVIEW_WIDTH_MIN_MM,
                TRAY_PREVIEW_WIDTH_MAX_MM,
            ))
            self.tray_preview_border_var.set(self._clamp(
                payload.get('preview_tray_border_mm', TRAY_PREVIEW_BORDER_THICKNESS_MM),
                TRAY_PREVIEW_BORDER_MIN_MM,
                TRAY_PREVIEW_BORDER_MAX_MM,
            ))
            self.intercept_dot_diameter_var.set(self._clamp(
                payload.get('preview_intercept_dot_diameter_mm', INTERCEPT_DOT_DIAMETER_DEFAULT_MM),
                INTERCEPT_DOT_DIAMETER_MIN_MM,
                INTERCEPT_DOT_DIAMETER_MAX_MM,
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
                f'Failed to save tray intercept runtime settings at "{self._runtime_settings_path}": {exc}'
            )

    def _set_intercept_offsets_from_preview(self, x_mm: float, y_mm: float) -> None:
        clamped_x_mm = self._clamp(x_mm, POST_STOP_X_OFFSET_MIN, POST_STOP_X_OFFSET_MAX)
        clamped_y_mm = self._clamp(y_mm, POST_STOP_Y_OFFSET_MIN, POST_STOP_Y_OFFSET_MAX)
        current_x_mm = float(self.post_stop_x_offset_var.get())
        current_y_mm = float(self.post_stop_y_offset_var.get())
        if abs(clamped_x_mm - current_x_mm) < 1e-6 and abs(clamped_y_mm - current_y_mm) < 1e-6:
            return

        self._suspend_runtime_settings_events = True
        try:
            self.post_stop_x_offset_var.set(clamped_x_mm)
            self.post_stop_y_offset_var.set(clamped_y_mm)
        finally:
            self._suspend_runtime_settings_events = False
        self._on_runtime_setting_changed()

    def _maybe_auto_fetch_tray_size(self, force: bool = False) -> None:
        if self._fetch_tray_size_inflight:
            return
        if not self.node.is_tray_dimensions_service_available():
            return
        now = time.monotonic()
        refresh_sec = TRAY_DIMENSIONS_AUTO_REFRESH_SEC
        if not force and (now - self._last_tray_size_fetch_monotonic) < refresh_sec:
            return
        self._last_tray_size_fetch_monotonic = now
        self._fetch_tray_size_inflight = True
        worker = threading.Thread(target=self._fetch_tray_size_worker, daemon=True)
        worker.start()

    def _fetch_tray_size_worker(self) -> None:
        result = self.node.fetch_tray_dimensions(quiet=True)
        if self._closed:
            return
        try:
            self.root.after(0, lambda: self._finish_fetch_tray_size(result))
        except tk.TclError:
            pass

    def _finish_fetch_tray_size(
        self,
        result: tuple[float, float, bool, str, str] | None,
    ) -> None:
        self._fetch_tray_size_inflight = False
        if self._is_ui_locked():
            return
        if result is not None:
            x_size_mm, y_size_mm, _live_detection, _tray_name, _message = result
            clamped_x_size_mm = self._clamp(
                x_size_mm,
                TRAY_PREVIEW_LENGTH_MIN_MM,
                TRAY_PREVIEW_LENGTH_MAX_MM,
            )
            clamped_y_size_mm = self._clamp(
                y_size_mm,
                TRAY_PREVIEW_WIDTH_MIN_MM,
                TRAY_PREVIEW_WIDTH_MAX_MM,
            )
            self._suspend_runtime_settings_events = True
            try:
                self.tray_preview_length_var.set(clamped_x_size_mm)
                self.tray_preview_width_var.set(clamped_y_size_mm)
            finally:
                self._suspend_runtime_settings_events = False
            self.node.set_preview_tray_dimensions(clamped_x_size_mm, clamped_y_size_mm)
            self._last_preview_signature = None
            self._draw_intercept_preview()
            self._schedule_runtime_settings_save()
        else:
            self._last_tray_size_fetch_monotonic = max(
                0.0,
                time.monotonic() - TRAY_DIMENSIONS_AUTO_REFRESH_SEC + TRAY_DIMENSIONS_AUTO_RETRY_SEC,
            )

    def _on_preview_canvas_clicked(self, event: tk.Event) -> None:
        if self._is_ui_locked():
            return
        transform = self._preview_canvas_transform
        if transform is None:
            return

        outer_rect = transform.get('outer_rect_px')
        if not isinstance(outer_rect, tuple) or len(outer_rect) != 4:
            return

        left_px, top_px, right_px, bottom_px = outer_rect
        if event.x < left_px or event.x > right_px or event.y < top_px or event.y > bottom_px:
            return

        scale = float(transform['scale'])
        if scale <= 1e-9:
            return

        plane_min_x_mm = float(transform['plane_min_display_x_mm'])
        plane_max_y_mm = float(transform['plane_max_display_y_mm'])
        x_offset_px = float(transform['x_offset_px'])
        y_offset_px = float(transform['y_offset_px'])
        clicked_display_x_mm = plane_min_x_mm + (float(event.x) - x_offset_px) / scale
        clicked_display_y_mm = plane_max_y_mm - (float(event.y) - y_offset_px) / scale
        x_axis = transform.get('x_axis_xy', (1.0, 0.0))
        y_axis = transform.get('y_axis_xy', (0.0, 1.0))
        det = (
            float(x_axis[0]) * float(y_axis[1])
            - float(x_axis[1]) * float(y_axis[0])
        )
        if abs(det) <= 1e-9:
            return
        clicked_x_mm = (
            (clicked_display_x_mm * float(y_axis[1]))
            - (clicked_display_y_mm * float(y_axis[0]))
        ) / det
        clicked_y_mm = (
            (float(x_axis[0]) * clicked_display_y_mm)
            - (float(x_axis[1]) * clicked_display_x_mm)
        )
        clicked_y_mm /= det
        self._set_intercept_offsets_from_preview(clicked_x_mm, clicked_y_mm)

    def _draw_intercept_preview(self) -> None:
        if not hasattr(self, 'preview_canvas'):
            return

        canvas = self.preview_canvas
        width = int(canvas.winfo_width())
        height = int(canvas.winfo_height())
        if width <= 2 or height <= 2:
            return

        tray_length_mm = max(
            TRAY_PREVIEW_LENGTH_MIN_MM,
            min(TRAY_PREVIEW_LENGTH_MAX_MM, float(self.tray_preview_length_var.get())),
        )
        tray_width_mm = max(
            TRAY_PREVIEW_WIDTH_MIN_MM,
            min(TRAY_PREVIEW_WIDTH_MAX_MM, float(self.tray_preview_width_var.get())),
        )
        border_mm = max(
            TRAY_PREVIEW_BORDER_MIN_MM,
            min(TRAY_PREVIEW_BORDER_MAX_MM, float(self.tray_preview_border_var.get())),
        )
        x_offset_mm = float(self.post_stop_x_offset_var.get())
        y_offset_mm = float(self.post_stop_y_offset_var.get())
        dot_diameter_mm = max(
            INTERCEPT_DOT_DIAMETER_MIN_MM,
            min(INTERCEPT_DOT_DIAMETER_MAX_MM, float(self.intercept_dot_diameter_var.get())),
        )
        dot_radius_mm = 0.5 * dot_diameter_mm
        self.dot_info_var.set(f'Red dot Ø {dot_diameter_mm:.0f} mm (default 10 mm)')
        self.border_info_var.set(f'Tray border thickness: {border_mm:.0f} mm.')

        snapshot = self.node.snapshot()
        x_axis_xy = snapshot.tray_preview_x_axis if snapshot.tray_preview_axes_valid else (1.0, 0.0)
        y_axis_xy = snapshot.tray_preview_y_axis if snapshot.tray_preview_axes_valid else (0.0, 1.0)

        def local_to_display_xy(x_mm: float, y_mm: float) -> tuple[float, float]:
            return (
                (float(x_mm) * x_axis_xy[0]) + (float(y_mm) * y_axis_xy[0]),
                (float(x_mm) * x_axis_xy[1]) + (float(y_mm) * y_axis_xy[1]),
            )

        outer_corners_local = (
            (-border_mm, -border_mm),
            (tray_length_mm + border_mm, -border_mm),
            (tray_length_mm + border_mm, tray_width_mm + border_mm),
            (-border_mm, tray_width_mm + border_mm),
        )
        display_points = [local_to_display_xy(x_mm, y_mm) for x_mm, y_mm in outer_corners_local]
        dot_display_x_mm, dot_display_y_mm = local_to_display_xy(x_offset_mm, y_offset_mm)
        display_points.extend((
            (dot_display_x_mm - dot_radius_mm - 20.0, dot_display_y_mm),
            (dot_display_x_mm + dot_radius_mm + 20.0, dot_display_y_mm),
            (dot_display_x_mm, dot_display_y_mm - dot_radius_mm - 20.0),
            (dot_display_x_mm, dot_display_y_mm + dot_radius_mm + 20.0),
        ))

        plane_min_x_mm = min(point[0] for point in display_points)
        plane_max_x_mm = max(point[0] for point in display_points)
        plane_min_y_mm = min(point[1] for point in display_points)
        plane_max_y_mm = max(point[1] for point in display_points)
        span_x_mm = max(1.0, plane_max_x_mm - plane_min_x_mm)
        span_y_mm = max(1.0, plane_max_y_mm - plane_min_y_mm)
        pad_px = 16.0
        usable_w_px = max(1.0, width - 2.0 * pad_px)
        usable_h_px = max(1.0, height - 2.0 * pad_px)
        scale = min(
            usable_w_px / span_x_mm,
            usable_h_px / span_y_mm,
        )
        used_w_px = span_x_mm * scale
        used_h_px = span_y_mm * scale
        x_offset_px = pad_px + 0.5 * (usable_w_px - used_w_px)
        y_offset_px = pad_px + 0.5 * (usable_h_px - used_h_px)

        def display_to_canvas(x_mm: float, y_mm: float) -> tuple[float, float]:
            x_px = x_offset_px + (x_mm - plane_min_x_mm) * scale
            y_px = y_offset_px + (plane_max_y_mm - y_mm) * scale
            return (x_px, y_px)

        def to_canvas(x_mm: float, y_mm: float) -> tuple[float, float]:
            return display_to_canvas(*local_to_display_xy(x_mm, y_mm))

        def polygon_coords_mm(
            x0_mm: float,
            y0_mm: float,
            x1_mm: float,
            y1_mm: float,
        ) -> tuple[float, ...]:
            points = (
                to_canvas(x0_mm, y0_mm),
                to_canvas(x1_mm, y0_mm),
                to_canvas(x1_mm, y1_mm),
                to_canvas(x0_mm, y1_mm),
            )
            return tuple(coord for point in points for coord in point)

        canvas.delete('all')

        # Tray border (50 mm thick all around): outer fill then inner cutout.
        outer_polygon = polygon_coords_mm(
            -border_mm,
            -border_mm,
            tray_length_mm + border_mm,
            tray_width_mm + border_mm,
        )
        inner_polygon = polygon_coords_mm(0.0, 0.0, tray_length_mm, tray_width_mm)
        outer_rect = (
            x_offset_px,
            y_offset_px,
            x_offset_px + used_w_px,
            y_offset_px + used_h_px,
        )
        self._preview_canvas_transform = {
            'scale': scale,
            'plane_min_display_x_mm': plane_min_x_mm,
            'plane_max_display_y_mm': plane_max_y_mm,
            'x_offset_px': x_offset_px,
            'y_offset_px': y_offset_px,
            'outer_rect_px': outer_rect,
            'tray_length_mm': tray_length_mm,
            'x_axis_xy': x_axis_xy,
            'y_axis_xy': y_axis_xy,
        }
        canvas.create_polygon(*outer_polygon, fill='#2b8e53', outline='#7fe8ad', width=2)
        canvas.create_polygon(*inner_polygon, fill='#141414', outline='#67d593', width=2)

        # Origin marker at tray local (0,0).
        origin_x, origin_y = to_canvas(0.0, 0.0)
        axis_len_mm = 40.0
        x_axis_x, x_axis_y = to_canvas(axis_len_mm, 0.0)
        y_axis_x, y_axis_y = to_canvas(0.0, axis_len_mm)
        canvas.create_line(origin_x, origin_y, x_axis_x, x_axis_y, fill='#ff6666', width=2)
        canvas.create_line(origin_x, origin_y, y_axis_x, y_axis_y, fill='#00ff00', width=2)
        canvas.create_text(x_axis_x, x_axis_y, text='X', anchor='center', fill='#ffb3b3', font=('TkDefaultFont', 9, 'bold'))
        canvas.create_text(y_axis_x, y_axis_y, text='Y', anchor='center', fill='#b9ffb9', font=('TkDefaultFont', 9, 'bold'))
        canvas.create_oval(origin_x - 3, origin_y - 3, origin_x + 3, origin_y + 3, fill='#ffe27a', outline='')
        canvas.create_text(
            origin_x + 8,
            origin_y - 8,
            text='Origin LL',
            anchor='sw',
            fill='#f6f6f6',
            font=('TkDefaultFont', 9, 'bold'),
        )

        # Intercept point from X/Y offsets (red dot with adjustable physical diameter).
        dot_x_px, dot_y_px = to_canvas(x_offset_mm, y_offset_mm)
        dot_radius_px = max(2.0, dot_radius_mm * scale)
        canvas.create_oval(
            dot_x_px - dot_radius_px,
            dot_y_px - dot_radius_px,
            dot_x_px + dot_radius_px,
            dot_y_px + dot_radius_px,
            fill='#ff2d2d',
            outline='#ffd1d1',
            width=2,
        )
        canvas.create_text(
            dot_x_px + dot_radius_px + 8,
            dot_y_px,
            text=f'X={x_offset_mm:.0f} mm, Y={y_offset_mm:.0f} mm',
            anchor='w',
            fill='#ffecec',
            font=('TkDefaultFont', 9, 'bold'),
        )

    def _refresh(self) -> None:
        snapshot = self.node.snapshot()
        stop_inflight = self.node.is_manual_stop_inflight()
        ui_locked = bool(snapshot.armed or snapshot.busy or stop_inflight)
        self._sync_tf_only_button(is_busy=ui_locked)
        self._sync_release_grip_button(is_busy=ui_locked)
        self._set_arm_locked_setting_controls_enabled(not ui_locked)
        if not ui_locked:
            self._maybe_auto_fetch_tray_size()

        if bool(self.tf_only_var.get()) and not ui_locked and snapshot.has_last_tray:
            preview_signature = (
                snapshot.tray_seq,
                round(float(self.ee_final_pose_angle_var.get()), 3),
                round(float(self.post_stop_x_offset_var.get()), 3),
                round(float(self.post_stop_y_offset_var.get()), 3),
                round(float(self.post_stop_z_offset_var.get()), 3),
                round(float(self.follow_distance_var.get()), 3),
                round(float(self.post_follow_z_up_var.get()), 3),
            )
            if preview_signature != self._last_preview_signature:
                started = self.node.preview_from_last_tray(
                    float(self.ee_final_pose_angle_var.get()),
                    float(self.post_stop_x_offset_var.get()),
                    float(self.post_stop_y_offset_var.get()),
                    float(self.post_stop_z_offset_var.get()),
                    float(self.follow_distance_var.get()),
                    float(self.post_follow_z_up_var.get()),
                )
                if started:
                    self._last_preview_signature = preview_signature
        else:
            self._last_preview_signature = None

        self.action_var.set(snapshot.action_text)
        self._draw_intercept_preview()
        self.run_button.configure(state=tk.DISABLED if ui_locked else tk.NORMAL)
        self._set_stop_button_enabled(bool(snapshot.busy) and not stop_inflight)

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

    def _sync_release_grip_button(self, is_busy: bool) -> None:
        release_enabled = bool(self.release_grip_var.get())
        if release_enabled:
            label = 'Release Grip: ON'
            bg = '#d32f2f'
            fg = 'white'
            active_bg = '#b71c1c'
            active_fg = 'white'
        else:
            label = 'Release Grip: OFF'
            bg = self._release_grip_default_bg
            fg = self._release_grip_default_fg
            active_bg = self._release_grip_default_active_bg
            active_fg = self._release_grip_default_active_fg

        self.release_grip_button.configure(
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
    node = RelMovLMiniNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    gui = RelMovLMiniGui(node)
    try:
        gui.run()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
