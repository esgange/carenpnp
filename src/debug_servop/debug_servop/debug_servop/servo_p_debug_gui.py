import math
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext

import rclpy
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import MovJ, ServoP
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


SERVICE_ROOT_DEFAULT = '/dobot_bringup_ros2/srv'
TCP_TOPIC_DEFAULT = '/dobot_msgs_v4/msg/ToolVectorActual'
VISUALIZATION_TOPIC_DEFAULT = '/debug_servop/trajectory_markers'
TF_FRAME_ID_DEFAULT = 'base_link'
TF_CHILD_PREFIX_DEFAULT = 'servop_plan'
TF_POSITION_SCALE_DEFAULT = 0.001  # Dobot TCP x/y/z are normally millimeters; RViz TF is meters.

SERVO_P_AHEADTIME = 50.0
SERVO_P_GAIN = 500.0
DEFAULT_SERVO_P_TIME_SEC = 1.5
DEFAULT_ACCEL_PERCENT = 40
MIN_ACCEL_PERCENT = 10
MAX_ACCEL_PERCENT = 100
MIN_SERVO_INTERVAL_MS = 50
MAX_SERVO_INTERVAL_MS = 200
DEFAULT_SERVO_INTERVAL_MS = 50
MAX_TF_FRAMES = 120
MOVEJ_SETTLE_SEC = 1.0
TF_REPUBLISH_PERIOD_SEC = 0.25


Pose6 = tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class TrajectoryPoint:
    index: int
    time_sec: float
    u: float
    s: float
    pose: Pose6


@dataclass(frozen=True)
class ServoPTrajectoryPlan:
    points: list[TrajectoryPoint]
    dt_sec: float
    total_time_sec: float
    acceleration_percent: int
    ramp_fraction: float
    planner_hz: float
    interval_ms: int
    lock_orientation: bool


def format_pose(pose: Pose6) -> str:
    return ', '.join(f'{value:.3f}' for value in pose)


def parse_pose(text: str) -> Pose6:
    cleaned = text.strip().replace('\n', ' ').replace(',', ' ')
    parts = [part for part in cleaned.split() if part]
    if len(parts) != 6:
        raise ValueError('Expected 6 numbers: x,y,z,rx,ry,rz')
    values = tuple(float(part) for part in parts)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Pose values must be finite numbers')
    return values


def shortest_angle_delta_degrees(start_deg: float, end_deg: float) -> float:
    """Return the shortest signed angular move from start to end in degrees."""
    return (float(end_deg) - float(start_deg) + 180.0) % 360.0 - 180.0


def interpolate_pose(
    start_pose: Pose6,
    end_pose: Pose6,
    ratio: float,
    lock_orientation: bool = True,
) -> Pose6:
    """Interpolate XYZ while preventing accidental EE spin.

    When lock_orientation is True, rx/ry/rz stay equal to Point A for every
    planned waypoint. If orientation interpolation is enabled later, angles use
    the shortest angular difference instead of raw deltas such as 179 -> -179
    becoming a 358 degree rotation.
    """
    clamped_ratio = max(0.0, min(1.0, float(ratio)))

    x = start_pose[0] + (end_pose[0] - start_pose[0]) * clamped_ratio
    y = start_pose[1] + (end_pose[1] - start_pose[1]) * clamped_ratio
    z = start_pose[2] + (end_pose[2] - start_pose[2]) * clamped_ratio

    if lock_orientation:
        rx, ry, rz = start_pose[3], start_pose[4], start_pose[5]
    else:
        rx = start_pose[3] + shortest_angle_delta_degrees(start_pose[3], end_pose[3]) * clamped_ratio
        ry = start_pose[4] + shortest_angle_delta_degrees(start_pose[4], end_pose[4]) * clamped_ratio
        rz = start_pose[5] + shortest_angle_delta_degrees(start_pose[5], end_pose[5]) * clamped_ratio

    return (x, y, z, rx, ry, rz)


def accel_percent_to_ramp_fraction(acceleration_percent: int) -> float:
    """Map a simple acceleration percentage to accel/decel time share.

    Lower percentage means a softer move: longer accel/decel ramps and smaller
    position deltas at the start/end. Higher percentage means a sharper move.
    """
    clamped = max(MIN_ACCEL_PERCENT, min(MAX_ACCEL_PERCENT, int(acceleration_percent)))
    normalized = (clamped - MIN_ACCEL_PERCENT) / (MAX_ACCEL_PERCENT - MIN_ACCEL_PERCENT)
    return 0.45 - (0.35 * normalized)  # 10% -> 45% ramp, 100% -> 10% ramp.


def trapezoid_progress(u: float, ramp_fraction: float) -> float:
    """Normalized position progress for a trapezoidal velocity profile."""
    u = max(0.0, min(1.0, float(u)))
    r = max(0.001, min(0.499, float(ramp_fraction)))
    denom = 2.0 * r * (1.0 - r)

    if u <= r:
        return (u * u) / denom
    if u >= 1.0 - r:
        remaining = 1.0 - u
        return 1.0 - ((remaining * remaining) / denom)
    return (u - (0.5 * r)) / (1.0 - r)


def build_linear_servop_plan(
    point_a: Pose6,
    point_b: Pose6,
    total_time_sec: float,
    acceleration_percent: int,
    interval_ms: int = DEFAULT_SERVO_INTERVAL_MS,
    lock_orientation: bool = True,
) -> ServoPTrajectoryPlan:
    if not math.isfinite(total_time_sec) or total_time_sec <= 0.0:
        raise ValueError('ServoP time must be a positive number')

    accel_percent = max(MIN_ACCEL_PERCENT, min(MAX_ACCEL_PERCENT, int(acceleration_percent)))
    clamped_interval_ms = max(MIN_SERVO_INTERVAL_MS, min(MAX_SERVO_INTERVAL_MS, int(interval_ms)))
    nominal_dt = clamped_interval_ms / 1000.0
    if total_time_sec < nominal_dt:
        raise ValueError(
            f'ServoP total time must be at least the selected interval ({clamped_interval_ms} ms)'
        )

    ramp_fraction = accel_percent_to_ramp_fraction(accel_percent)
    time_points: list[float] = [0.0]
    next_time = nominal_dt
    while next_time < total_time_sec - 1e-9:
        time_points.append(next_time)
        next_time += nominal_dt
    if total_time_sec > time_points[-1] + 1e-9:
        time_points.append(total_time_sec)

    points: list[TrajectoryPoint] = []
    for index, point_time_sec in enumerate(time_points):
        u = point_time_sec / total_time_sec
        s = trapezoid_progress(u, ramp_fraction)
        pose = interpolate_pose(point_a, point_b, s, lock_orientation=lock_orientation)
        points.append(TrajectoryPoint(index=index, time_sec=point_time_sec, u=u, s=s, pose=pose))

    return ServoPTrajectoryPlan(
        points=points,
        dt_sec=nominal_dt,
        total_time_sec=total_time_sec,
        acceleration_percent=accel_percent,
        ramp_fraction=ramp_fraction,
        planner_hz=1.0 / nominal_dt,
        interval_ms=clamped_interval_ms,
        lock_orientation=lock_orientation,
    )


def euler_degrees_to_quaternion(rx_deg: float, ry_deg: float, rz_deg: float) -> tuple[float, float, float, float]:
    roll = math.radians(rx_deg)
    pitch = math.radians(ry_deg)
    yaw = math.radians(rz_deg)

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


class ServoPDebugNode(Node):
    def __init__(self) -> None:
        super().__init__('servo_p_debug_gui')
        self._lock = threading.Lock()
        self._latest_pose: Pose6 | None = None
        self._latest_pose_time = 0.0
        self._active_plan_transforms: list[TransformStamped] = []

        self._tcp_topic = str(
            self.declare_parameter('tcp_topic', TCP_TOPIC_DEFAULT).value
        ).strip() or TCP_TOPIC_DEFAULT
        self._movj_service = str(
            self.declare_parameter('movj_service', f'{SERVICE_ROOT_DEFAULT}/MovJ').value
        ).strip() or f'{SERVICE_ROOT_DEFAULT}/MovJ'
        self._servop_service = str(
            self.declare_parameter('servop_service', f'{SERVICE_ROOT_DEFAULT}/ServoP').value
        ).strip() or f'{SERVICE_ROOT_DEFAULT}/ServoP'
        self._visualization_topic = str(
            self.declare_parameter('visualization_topic', VISUALIZATION_TOPIC_DEFAULT).value
        ).strip() or VISUALIZATION_TOPIC_DEFAULT
        self._tf_frame_id = str(
            self.declare_parameter('tf_frame_id', TF_FRAME_ID_DEFAULT).value
        ).strip() or TF_FRAME_ID_DEFAULT
        self._tf_child_prefix = str(
            self.declare_parameter('tf_child_prefix', TF_CHILD_PREFIX_DEFAULT).value
        ).strip() or TF_CHILD_PREFIX_DEFAULT
        self._tf_position_scale = float(
            self.declare_parameter('tf_position_scale', TF_POSITION_SCALE_DEFAULT).value
        )
        if not math.isfinite(self._tf_position_scale) or self._tf_position_scale <= 0.0:
            self._tf_position_scale = TF_POSITION_SCALE_DEFAULT

        self._movj_client = self.create_client(MovJ, self._movj_service)
        self._servop_client = self.create_client(ServoP, self._servop_service)
        self._marker_pub = self.create_publisher(MarkerArray, self._visualization_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(ToolVectorActual, self._tcp_topic, self._tcp_callback, 10)
        self.create_timer(TF_REPUBLISH_PERIOD_SEC, self._publish_active_tf_frames)

        self.get_logger().info(
            f'ServoP Debug ready. TCP topic={self._tcp_topic}, '
            f'MovJ={self._movj_service}, ServoP={self._servop_service}, '
            f'RViz markers={self._visualization_topic}, TF frame={self._tf_frame_id}'
        )

    def _tcp_callback(self, msg: ToolVectorActual) -> None:
        pose = (
            float(msg.x),
            float(msg.y),
            float(msg.z),
            float(msg.rx),
            float(msg.ry),
            float(msg.rz),
        )
        with self._lock:
            self._latest_pose = pose
            self._latest_pose_time = time.monotonic()

    def latest_pose(self) -> tuple[Pose6 | None, float]:
        with self._lock:
            return self._latest_pose, self._latest_pose_time

    def _wait_for_service(self, client, label: str, status_callback, timeout_sec: float = 5.0) -> bool:
        started = time.monotonic()
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=0.2):
                return True
            if time.monotonic() - started >= timeout_sec:
                status_callback(f'{label} service not ready')
                return False
            status_callback(f'Waiting for {label} service...')
        status_callback(f'ROS shutdown while waiting for {label}')
        return False

    @staticmethod
    def _motion_param_value(v_percent: int = 100, a_percent: int = 100) -> list[str]:
        return [f'v={int(v_percent)},a={int(a_percent)}']

    @staticmethod
    def _servop_param_value(t_sec: float) -> list[str]:
        return [
            f't={float(t_sec):.3f}',
            f'aheadtime={SERVO_P_AHEADTIME:.1f}',
            f'gain={SERVO_P_GAIN:.1f}',
        ]

    @staticmethod
    def _fill_pose_request(request, pose: Pose6) -> None:
        request.a = pose[0]
        request.b = pose[1]
        request.c = pose[2]
        request.d = pose[3]
        request.e = pose[4]
        request.f = pose[5]

    def _call_service(
        self,
        client,
        request,
        label: str,
        status_callback,
        timeout_sec: float = 8.0,
        announce: bool = True,
        success_status: bool = True,
    ):
        if announce:
            status_callback(f'Sending {label}...')
        future = client.call_async(request)
        started = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - started >= timeout_sec:
                status_callback(f'Timeout waiting for {label}')
                return None
            time.sleep(0.002)

        if not rclpy.ok():
            status_callback(f'ROS shutdown during {label}')
            return None

        exception = future.exception()
        if exception is not None:
            status_callback(f'{label} exception: {exception}')
            return None

        response = future.result()
        if response is None:
            status_callback(f'{label} returned no response')
            return None

        res = int(getattr(response, 'res', -1))
        robot_return = str(getattr(response, 'robot_return', '')).strip()
        if res < 0:
            detail = f'res={res}'
            if robot_return:
                detail += f', return={robot_return}'
            status_callback(f'{label} failed: {detail}')
            return response

        if success_status:
            if robot_return:
                status_callback(f'{label} OK: {robot_return}')
            else:
                status_callback(f'{label} OK')
        return response

    def build_plan(
        self,
        point_a: Pose6,
        point_b: Pose6,
        servo_p_time_sec: float,
        acceleration_percent: int,
        interval_ms: int,
    ) -> ServoPTrajectoryPlan:
        return build_linear_servop_plan(
            point_a=point_a,
            point_b=point_b,
            total_time_sec=servo_p_time_sec,
            acceleration_percent=acceleration_percent,
            interval_ms=interval_ms,
            lock_orientation=True,
        )

    def _plan_summary(self, plan: ServoPTrajectoryPlan) -> str:
        orientation_text = 'orientation locked to Point A' if plan.lock_orientation else 'orientation shortest-angle interpolated'
        return (
            f'Linear ServoP plan: {len(plan.points)} points, interval={plan.interval_ms} ms '
            f'(last segment may be shorter), total={plan.total_time_sec:.3f}s, '
            f'accel={plan.acceleration_percent}%, '
            f'accel/decel ramp={plan.ramp_fraction * 100.0:.1f}% each side, '
            f'{orientation_text}'
        )

    def clear_plan_visualization(self) -> None:
        """Clear old RViz plan markers and stop re-publishing old TF waypoint frames."""
        with self._lock:
            self._active_plan_transforms = []

        delete_marker = Marker()
        delete_marker.header.frame_id = self._tf_frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = self._tf_child_prefix
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker)
        self._marker_pub.publish(marker_array)

    def _publish_active_tf_frames(self) -> None:
        with self._lock:
            transforms = list(self._active_plan_transforms)
        if not transforms:
            return

        stamp = self.get_clock().now().to_msg()
        for transform in transforms:
            transform.header.stamp = stamp
        self._tf_broadcaster.sendTransform(transforms)

    def publish_plan_visualization(self, plan: ServoPTrajectoryPlan, status_callback) -> None:
        self.clear_plan_visualization()
        stamp = self.get_clock().now().to_msg()

        decimation = max(1, math.ceil(len(plan.points) / MAX_TF_FRAMES))
        tf_points = [point for point in plan.points if point.index % decimation == 0]
        if tf_points[-1].index != plan.points[-1].index:
            tf_points.append(plan.points[-1])

        transforms: list[TransformStamped] = []
        for frame_index, point in enumerate(tf_points):
            pose = point.pose
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._tf_frame_id
            transform.child_frame_id = f'{self._tf_child_prefix}_{frame_index:03d}'
            transform.transform.translation.x = pose[0] * self._tf_position_scale
            transform.transform.translation.y = pose[1] * self._tf_position_scale
            transform.transform.translation.z = pose[2] * self._tf_position_scale
            qx, qy, qz, qw = euler_degrees_to_quaternion(pose[3], pose[4], pose[5])
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        with self._lock:
            self._active_plan_transforms = transforms
        self._publish_active_tf_frames()
        self._publish_marker_array(plan, self._tf_child_prefix, stamp)
        status_callback(
            f'Published RViz plan: cleared previous plan, then published '
            f'{len(plan.points)} marker points + {len(transforms)} TF frames under {self._tf_child_prefix}_*'
        )

    def _publish_marker_array(self, plan: ServoPTrajectoryPlan, namespace: str, stamp) -> None:
        marker_array = MarkerArray()

        line_marker = Marker()
        line_marker.header.frame_id = self._tf_frame_id
        line_marker.header.stamp = stamp
        line_marker.ns = namespace
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.006
        line_marker.color.r = 0.1
        line_marker.color.g = 0.9
        line_marker.color.b = 0.1
        line_marker.color.a = 1.0

        point_marker = Marker()
        point_marker.header.frame_id = self._tf_frame_id
        point_marker.header.stamp = stamp
        point_marker.ns = namespace
        point_marker.id = 2
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = 0.018
        point_marker.scale.y = 0.018
        point_marker.scale.z = 0.018
        point_marker.color.r = 1.0
        point_marker.color.g = 0.8
        point_marker.color.b = 0.1
        point_marker.color.a = 1.0

        for plan_point in plan.points:
            pose = plan_point.pose
            rviz_point = Point()
            rviz_point.x = pose[0] * self._tf_position_scale
            rviz_point.y = pose[1] * self._tf_position_scale
            rviz_point.z = pose[2] * self._tf_position_scale
            line_marker.points.append(rviz_point)
            point_marker.points.append(rviz_point)

        marker_array.markers.append(line_marker)
        marker_array.markers.append(point_marker)
        self._marker_pub.publish(marker_array)

    def publish_tf_only(
        self,
        point_a: Pose6,
        point_b: Pose6,
        servo_p_time_sec: float,
        acceleration_percent: int,
        interval_ms: int,
        status_callback,
    ) -> ServoPTrajectoryPlan:
        plan = self.build_plan(point_a, point_b, servo_p_time_sec, acceleration_percent, interval_ms)
        status_callback(self._plan_summary(plan))
        status_callback(f'First point: s={plan.points[0].s:.3f}, pose=({format_pose(plan.points[0].pose)})')
        middle = plan.points[len(plan.points) // 2]
        status_callback(f'Middle point: s={middle.s:.3f}, pose=({format_pose(middle.pose)})')
        status_callback(f'Last point: s={plan.points[-1].s:.3f}, pose=({format_pose(plan.points[-1].pose)})')
        self.publish_plan_visualization(plan, status_callback)
        return plan

    def run_test(
        self,
        point_a: Pose6,
        point_b: Pose6,
        servo_p_time_sec: float,
        acceleration_percent: int,
        interval_ms: int,
        status_callback,
    ) -> bool:
        plan = self.publish_tf_only(point_a, point_b, servo_p_time_sec, acceleration_percent, interval_ms, status_callback)

        if not self._wait_for_service(self._movj_client, 'MovJ', status_callback):
            return False
        if not self._wait_for_service(self._servop_client, 'ServoP', status_callback):
            return False

        movj_request = MovJ.Request()
        movj_request.mode = False
        self._fill_pose_request(movj_request, point_a)
        movj_request.param_value = self._motion_param_value()
        movj_response = self._call_service(
            self._movj_client,
            movj_request,
            f'MovJ Point A ({format_pose(point_a)})',
            status_callback,
        )
        if movj_response is None or int(getattr(movj_response, 'res', -1)) < 0:
            return False

        if MOVEJ_SETTLE_SEC > 0.0:
            status_callback(f'Waiting {MOVEJ_SETTLE_SEC:.1f}s before ServoP stream...')
            time.sleep(MOVEJ_SETTLE_SEC)

        send_points = plan.points[1:]
        status_callback(
            f'Streaming {len(send_points)} ServoP waypoints at up to {plan.interval_ms} ms intervals '
            f'(aheadtime={SERVO_P_AHEADTIME:.1f}, gain={SERVO_P_GAIN:.1f})'
        )

        stream_started = time.monotonic()
        log_every = max(1, len(send_points) // 10)
        previous_point = plan.points[0]
        for send_index, point in enumerate(send_points, start=1):
            segment_dt = max(0.001, point.time_sec - previous_point.time_sec)
            target_time = stream_started + previous_point.time_sec
            now = time.monotonic()
            if now < target_time:
                time.sleep(target_time - now)

            servo_p_request = ServoP.Request()
            self._fill_pose_request(servo_p_request, point.pose)
            servo_p_request.param_value = self._servop_param_value(segment_dt)
            servo_p_response = self._call_service(
                self._servop_client,
                servo_p_request,
                f'ServoP waypoint {send_index}/{len(send_points)}',
                status_callback,
                timeout_sec=max(1.0, segment_dt * 5.0),
                announce=False,
                success_status=False,
            )
            if servo_p_response is None or int(getattr(servo_p_response, 'res', -1)) < 0:
                return False

            if send_index == 1 or send_index == len(send_points) or send_index % log_every == 0:
                status_callback(
                    f'ServoP waypoint {send_index}/{len(send_points)}: '
                    f't={segment_dt:.3f}s, s={point.s:.3f}, pose=({format_pose(point.pose)})'
                )
            previous_point = point

        elapsed = time.monotonic() - stream_started
        status_callback(f'Linear ServoP trajectory complete in {elapsed:.3f}s')
        return True


class ServoPDebugGui:
    def __init__(self, node: ServoPDebugNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title('ServoP Linear Planner Debug')
        self.root.geometry('900x750')
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._worker: threading.Thread | None = None

        self.status_var = tk.StringVar(value='Waiting for TCP feedback...')

        outer = tk.Frame(self.root, padx=12, pady=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(7, weight=1)

        pose_row = tk.Frame(outer)
        pose_row.grid(row=0, column=0, sticky='ew')
        pose_row.columnconfigure(1, weight=1)
        tk.Button(pose_row, text='Get Pose', command=self._get_pose_clicked, width=14).grid(row=0, column=0, padx=(0, 8))
        self.pose_output_var = tk.StringVar(value='')
        tk.Entry(pose_row, textvariable=self.pose_output_var).grid(row=0, column=1, sticky='ew')
        tk.Button(pose_row, text='Copy Pose', command=self._copy_pose_clicked, width=12).grid(row=0, column=2, padx=(8, 0))

        tk.Label(outer, text='Point A (MovJ start, x,y,z,rx,ry,rz)').grid(row=1, column=0, sticky='w', pady=(14, 2))
        self.point_a_text = tk.Text(outer, height=2, wrap='none')
        self.point_a_text.grid(row=2, column=0, sticky='ew')

        tk.Label(outer, text='Point B (ServoP final target, x,y,z,rx,ry,rz)').grid(row=3, column=0, sticky='w', pady=(12, 2))
        self.point_b_text = tk.Text(outer, height=2, wrap='none')
        self.point_b_text.grid(row=4, column=0, sticky='ew')

        control_row = tk.Frame(outer)
        control_row.grid(row=5, column=0, sticky='ew', pady=(12, 0))
        control_row.columnconfigure(5, weight=1)
        tk.Label(control_row, text='ServoP total time (sec)').grid(row=0, column=0, sticky='w')
        self.time_var = tk.StringVar(value=f'{DEFAULT_SERVO_P_TIME_SEC:.1f}')
        tk.Entry(control_row, textvariable=self.time_var, width=10).grid(row=0, column=1, padx=(8, 16), sticky='w')
        self.tf_button = tk.Button(control_row, text='TF Only / Visualize', command=self._tf_only_clicked, width=18)
        self.tf_button.grid(row=0, column=2, sticky='w')
        self.test_button = tk.Button(control_row, text='Run ServoP Planner', command=self._test_servop_clicked, width=20)
        self.test_button.grid(row=0, column=3, sticky='w', padx=(8, 0))
        tk.Label(control_row, textvariable=self.status_var, anchor='w').grid(row=0, column=5, sticky='ew', padx=(16, 0))

        tk.Label(control_row, text='Acceleration %').grid(row=1, column=0, sticky='w', pady=(10, 0))
        self.accel_percent_var = tk.IntVar(value=DEFAULT_ACCEL_PERCENT)
        self.accel_percent_label_var = tk.StringVar(value='')
        tk.Scale(
            control_row,
            from_=MIN_ACCEL_PERCENT,
            to=MAX_ACCEL_PERCENT,
            orient=tk.HORIZONTAL,
            variable=self.accel_percent_var,
            command=self._accel_percent_changed,
            showvalue=True,
            length=320,
            resolution=1,
        ).grid(row=1, column=1, columnspan=3, sticky='ew', padx=(8, 16), pady=(6, 0))
        tk.Label(control_row, textvariable=self.accel_percent_label_var, anchor='w').grid(row=1, column=5, sticky='ew', padx=(16, 0), pady=(10, 0))
        self._accel_percent_changed(str(DEFAULT_ACCEL_PERCENT))

        tk.Label(control_row, text='Plan/send interval (ms)').grid(row=2, column=0, sticky='w', pady=(10, 0))
        self.interval_ms_var = tk.IntVar(value=DEFAULT_SERVO_INTERVAL_MS)
        self.interval_ms_label_var = tk.StringVar(value='')
        tk.Scale(
            control_row,
            from_=MIN_SERVO_INTERVAL_MS,
            to=MAX_SERVO_INTERVAL_MS,
            orient=tk.HORIZONTAL,
            variable=self.interval_ms_var,
            command=self._interval_ms_changed,
            showvalue=True,
            length=320,
            resolution=1,
        ).grid(row=2, column=1, columnspan=3, sticky='ew', padx=(8, 16), pady=(6, 0))
        tk.Label(control_row, textvariable=self.interval_ms_label_var, anchor='w').grid(row=2, column=5, sticky='ew', padx=(16, 0), pady=(10, 0))
        self._interval_ms_changed(str(DEFAULT_SERVO_INTERVAL_MS))

        tk.Label(
            outer,
            text=(
                'RViz: add TF display and MarkerArray topic '
                f'{VISUALIZATION_TOPIC_DEFAULT}. TF positions are scaled from mm to m by default.'
            ),
            anchor='w',
        ).grid(row=6, column=0, sticky='ew', pady=(14, 2))

        self.log_text = scrolledtext.ScrolledText(outer, height=12, state=tk.DISABLED)
        self.log_text.grid(row=7, column=0, sticky='nsew')

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._poll_queue()

    def _set_status(self, text: str) -> None:
        self._queue.put(('status', text))

    def _append_log(self, text: str) -> None:
        self._queue.put(('log', text))

    def _accel_percent_changed(self, value: str) -> None:
        percent = int(float(value))
        ramp_fraction = accel_percent_to_ramp_fraction(percent)
        self.accel_percent_label_var.set(
            f'{percent}% accel -> accel/decel ramp {ramp_fraction * 100.0:.1f}% each side; '
            f'lower = smoother'
        )

    def _interval_ms_changed(self, value: str) -> None:
        interval_ms = int(float(value))
        frequency_hz = 1000.0 / interval_ms
        self.interval_ms_label_var.set(
            f'{interval_ms} ms between planned/sent ServoP points (~{frequency_hz:.1f} Hz)'
        )

    def _get_text_pose(self, widget: tk.Text) -> Pose6:
        return parse_pose(widget.get('1.0', 'end').strip())

    def _fill_text_pose(self, widget: tk.Text, pose_text: str) -> None:
        widget.delete('1.0', 'end')
        widget.insert('1.0', pose_text)

    def _get_pose_clicked(self) -> None:
        pose, stamp = self.node.latest_pose()
        if pose is None:
            messagebox.showwarning('ServoP Debug', 'No TCP pose received yet.')
            return
        pose_text = format_pose(pose)
        age = time.monotonic() - stamp
        self.pose_output_var.set(pose_text)
        self.root.clipboard_clear()
        self.root.clipboard_append(pose_text)
        self.status_var.set(f'Pose copied ({age:.1f}s old)')
        self._append_log(f'Get Pose: {pose_text}')

    def _copy_pose_clicked(self) -> None:
        pose_text = self.pose_output_var.get().strip()
        if not pose_text:
            messagebox.showwarning('ServoP Debug', 'No pose output to copy.')
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(pose_text)
        self.status_var.set('Pose copied to clipboard')

    def _read_plan_inputs(self) -> tuple[Pose6, Pose6, float, int, int]:
        point_a = self._get_text_pose(self.point_a_text)
        point_b = self._get_text_pose(self.point_b_text)
        servo_p_time = float(self.time_var.get().strip())
        if not math.isfinite(servo_p_time) or servo_p_time <= 0.0:
            raise ValueError('ServoP time must be a positive number')
        acceleration_percent = int(self.accel_percent_var.get())
        if acceleration_percent < MIN_ACCEL_PERCENT or acceleration_percent > MAX_ACCEL_PERCENT:
            raise ValueError(f'Acceleration percentage must be between {MIN_ACCEL_PERCENT} and {MAX_ACCEL_PERCENT}')
        interval_ms = int(self.interval_ms_var.get())
        if interval_ms < MIN_SERVO_INTERVAL_MS or interval_ms > MAX_SERVO_INTERVAL_MS:
            raise ValueError(f'Plan/send interval must be between {MIN_SERVO_INTERVAL_MS} and {MAX_SERVO_INTERVAL_MS} ms')
        if servo_p_time < interval_ms / 1000.0:
            raise ValueError(f'ServoP total time must be at least the selected interval ({interval_ms} ms)')
        return point_a, point_b, servo_p_time, acceleration_percent, interval_ms

    def _tf_only_clicked(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            messagebox.showinfo('ServoP Debug', 'A ServoP test is already running.')
            return
        self.node.clear_plan_visualization()
        try:
            point_a, point_b, servo_p_time, acceleration_percent, interval_ms = self._read_plan_inputs()
        except ValueError as exc:
            messagebox.showerror('ServoP Debug', str(exc))
            return

        self._append_log(f'TF Only: Point A={format_pose(point_a)}')
        self._append_log(
            f'TF Only: Point B={format_pose(point_b)}, total time={servo_p_time:.3f}s, '
            f'accel={acceleration_percent}%, interval={interval_ms} ms, orientation locked to Point A'
        )
        try:
            self.node.publish_tf_only(point_a, point_b, servo_p_time, acceleration_percent, interval_ms, self._set_status)
        except Exception as exc:
            self.status_var.set(f'TF Only failed: {exc}')
            self._append_log(f'TF Only failed: {exc}')

    def _test_servop_clicked(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            messagebox.showinfo('ServoP Debug', 'A test is already running.')
            return
        self.node.clear_plan_visualization()
        try:
            point_a, point_b, servo_p_time, acceleration_percent, interval_ms = self._read_plan_inputs()
        except ValueError as exc:
            messagebox.showerror('ServoP Debug', str(exc))
            return

        self.test_button.configure(state=tk.DISABLED)
        self.tf_button.configure(state=tk.DISABLED)
        self._append_log(f'Run ServoP Planner: Point A={format_pose(point_a)}')
        self._append_log(
            f'Run ServoP Planner: Point B={format_pose(point_b)}, total time={servo_p_time:.3f}s, '
            f'accel={acceleration_percent}%, interval={interval_ms} ms, orientation locked to Point A'
        )
        self._worker = threading.Thread(
            target=self._run_test_worker,
            args=(point_a, point_b, servo_p_time, acceleration_percent, interval_ms),
            daemon=True,
        )
        self._worker.start()

    def _run_test_worker(
        self,
        point_a: Pose6,
        point_b: Pose6,
        servo_p_time: float,
        acceleration_percent: int,
        interval_ms: int,
    ) -> None:
        success = False
        try:
            success = self.node.run_test(point_a, point_b, servo_p_time, acceleration_percent, interval_ms, self._set_status)
        except Exception as exc:
            self._set_status(f'Test failed: {exc}')
        self._queue.put(('done', success))

    def _poll_queue(self) -> None:
        while True:
            try:
                kind, payload = self._queue.get_nowait()
            except queue.Empty:
                break
            if kind == 'status':
                text = str(payload)
                self.status_var.set(text)
                self._append_log(text)
            elif kind == 'log':
                self.log_text.configure(state=tk.NORMAL)
                self.log_text.insert('end', str(payload) + '\n')
                self.log_text.see('end')
                self.log_text.configure(state=tk.DISABLED)
            elif kind == 'done':
                self.test_button.configure(state=tk.NORMAL)
                self.tf_button.configure(state=tk.NORMAL)
                if payload:
                    self.status_var.set('Linear ServoP trajectory complete')
        self.root.after(100, self._poll_queue)

    def _on_close(self) -> None:
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = ServoPDebugNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    gui = ServoPDebugGui(node)
    try:
        gui.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
