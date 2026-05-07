import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from functools import partial
import math
from pathlib import Path
import re
import time

from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import InverseKin, MovJ, SetTool, Stop, Tool
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


@dataclass(frozen=True)
class GoalPose:
  x: float
  y: float
  z: float
  rx: float
  ry: float
  rz: float
  distance_to_tag_mm: float = 0.0
  tilt_from_seed_deg: float = 0.0


CALIB_MODE_EYE_ON_HAND = "eye_on_hand"
CALIB_MODE_EYE_TO_HAND = "eye_to_hand"


def normalize_calibration_mode(value):
  mode = str(value or "").strip().lower()
  if mode == CALIB_MODE_EYE_TO_HAND:
    return CALIB_MODE_EYE_TO_HAND
  return CALIB_MODE_EYE_ON_HAND


def parse_joint_values_from_robot_return(text):
  raw = str(text or "")
  if not raw:
    return None

  float_pattern = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
  for content in re.findall(r"\{([^{}]+)\}", raw):
    numbers = [float(token) for token in re.findall(float_pattern, content)]
    if len(numbers) == 6:
      return numbers

  numbers = [float(token) for token in re.findall(float_pattern, raw)]
  if len(numbers) == 6:
    return numbers
  return None


def default_output_path():
  calib_dir = Path.home() / "DOBOT_pickn_place" / "calibration"
  try:
    calib_dir.mkdir(parents=True, exist_ok=True)
  except Exception:
    pass
  return str(calib_dir / "axab_calibration.yaml")


def normalize_output_path_setting(path_text):
  path = Path(str(path_text or "").strip()).expanduser()
  if path.name.startswith("axab_calibration_") and path.suffix == ".yaml":
    return default_output_path()
  return str(path) if str(path) else default_output_path()


def rpy_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg):
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


def quaternion_normalize(quat):
  qx, qy, qz, qw = quat
  norm = math.sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw))
  if norm < 1e-12:
    return 0.0, 0.0, 0.0, 1.0
  inv = 1.0 / norm
  return qx * inv, qy * inv, qz * inv, qw * inv


def quaternion_multiply(lhs, rhs):
  lx, ly, lz, lw = lhs
  rx, ry, rz, rw = rhs
  return quaternion_normalize((
    (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
    (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
    (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
    (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
  ))


def quaternion_conjugate(quat):
  x, y, z, w = quaternion_normalize(quat)
  return -x, -y, -z, w


def quaternion_rotate_vector(quat, vec):
  q = quaternion_normalize(quat)
  q_vec = (q[0], q[1], q[2])
  vx, vy, vz = float(vec[0]), float(vec[1]), float(vec[2])
  v = (vx, vy, vz)
  t = vec_cross(q_vec, v)
  t = (2.0 * t[0], 2.0 * t[1], 2.0 * t[2])
  q_cross_t = vec_cross(q_vec, t)
  return (
    v[0] + (q[3] * t[0]) + q_cross_t[0],
    v[1] + (q[3] * t[1]) + q_cross_t[1],
    v[2] + (q[3] * t[2]) + q_cross_t[2],
  )


def quaternion_to_rpy_deg(quat):
  x, y, z, w = quaternion_normalize(quat)

  sinr_cosp = 2.0 * ((w * x) + (y * z))
  cosr_cosp = 1.0 - (2.0 * ((x * x) + (y * y)))
  roll = math.atan2(sinr_cosp, cosr_cosp)

  sinp = 2.0 * ((w * y) - (z * x))
  if abs(sinp) >= 1.0:
    pitch = math.copysign(math.pi / 2.0, sinp)
  else:
    pitch = math.asin(sinp)

  siny_cosp = 2.0 * ((w * z) + (x * y))
  cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
  yaw = math.atan2(siny_cosp, cosy_cosp)
  return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def vec_dot(lhs, rhs):
  return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2])


def vec_cross(lhs, rhs):
  return (
    (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]),
    (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]),
    (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]),
  )


def vec_norm(vec):
  return math.sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]))


def vec_normalize(vec):
  norm = vec_norm(vec)
  if norm < 1e-9:
    return None
  inv = 1.0 / norm
  return vec[0] * inv, vec[1] * inv, vec[2] * inv


def angle_between_vectors_deg(lhs, rhs):
  a = vec_normalize(lhs)
  b = vec_normalize(rhs)
  if a is None or b is None:
    return 0.0
  dot = max(-1.0, min(1.0, vec_dot(a, b)))
  return math.degrees(math.acos(dot))


def quaternion_from_two_vectors(start_vec, end_vec):
  a = vec_normalize(start_vec)
  b = vec_normalize(end_vec)
  if a is None or b is None:
    return 0.0, 0.0, 0.0, 1.0

  dot = max(-1.0, min(1.0, vec_dot(a, b)))
  if dot < -0.999999:
    axis = vec_cross((1.0, 0.0, 0.0), a)
    if vec_norm(axis) < 1e-6:
      axis = vec_cross((0.0, 1.0, 0.0), a)
    axis = vec_normalize(axis)
    if axis is None:
      return 0.0, 0.0, 0.0, 1.0
    return axis[0], axis[1], axis[2], 0.0

  cross = vec_cross(a, b)
  quat = (cross[0], cross[1], cross[2], 1.0 + dot)
  return quaternion_normalize(quat)


def quaternion_from_axis_angle(axis, angle_deg):
  axis_n = vec_normalize(axis)
  if axis_n is None:
    return 0.0, 0.0, 0.0, 1.0
  half_angle = 0.5 * math.radians(float(angle_deg))
  sin_half = math.sin(half_angle)
  return quaternion_normalize((
    axis_n[0] * sin_half,
    axis_n[1] * sin_half,
    axis_n[2] * sin_half,
    math.cos(half_angle),
  ))


def rotate_vector_about_axis(vec, axis, angle_deg):
  v = vec_normalize(vec)
  k = vec_normalize(axis)
  if v is None or k is None:
    return v

  angle_rad = math.radians(float(angle_deg))
  c = math.cos(angle_rad)
  s = math.sin(angle_rad)
  kv = vec_dot(k, v)
  k_cross_v = vec_cross(k, v)

  rotated = (
    (v[0] * c) + (k_cross_v[0] * s) + (k[0] * kv * (1.0 - c)),
    (v[1] * c) + (k_cross_v[1] * s) + (k[1] * kv * (1.0 - c)),
    (v[2] * c) + (k_cross_v[2] * s) + (k[2] * kv * (1.0 - c)),
  )
  return vec_normalize(rotated)


def transform_pose_compose(parent_to_child, child_to_grandchild):
  px, py, pz, pqx, pqy, pqz, pqw = parent_to_child
  cx, cy, cz, cqx, cqy, cqz, cqw = child_to_grandchild
  rotated_child = quaternion_rotate_vector((pqx, pqy, pqz, pqw), (cx, cy, cz))
  return (
    px + rotated_child[0],
    py + rotated_child[1],
    pz + rotated_child[2],
    *quaternion_multiply((pqx, pqy, pqz, pqw), (cqx, cqy, cqz, cqw)),
  )


def transform_pose_inverse(parent_to_child):
  tx, ty, tz, qx, qy, qz, qw = parent_to_child
  inv_q = quaternion_conjugate((qx, qy, qz, qw))
  inv_t = quaternion_rotate_vector(inv_q, (-tx, -ty, -tz))
  return inv_t[0], inv_t[1], inv_t[2], inv_q[0], inv_q[1], inv_q[2], inv_q[3]


def identity_transform_pose_mm():
  return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0


def select_spread_subset(candidates, desired_count, seed_position=None):
  n = max(1, int(desired_count))
  if not candidates:
    return []
  if len(candidates) <= n:
    return list(candidates)

  if seed_position is None:
    seed_position = (candidates[0].x, candidates[0].y, candidates[0].z)
  sx, sy, sz = seed_position
  start_idx = min(
    range(len(candidates)),
    key=lambda idx: (
      (candidates[idx].x - sx) ** 2
      + (candidates[idx].y - sy) ** 2
      + (candidates[idx].z - sz) ** 2
    ),
  )

  selected = [start_idx]
  min_dist_sq = [float("inf")] * len(candidates)
  for idx, pose in enumerate(candidates):
    dx = pose.x - candidates[start_idx].x
    dy = pose.y - candidates[start_idx].y
    dz = pose.z - candidates[start_idx].z
    min_dist_sq[idx] = (dx * dx) + (dy * dy) + (dz * dz)
  min_dist_sq[start_idx] = -1.0

  while len(selected) < n:
    next_idx = max(range(len(candidates)), key=lambda idx: min_dist_sq[idx])
    if min_dist_sq[next_idx] < 0.0:
      break
    selected.append(next_idx)
    min_dist_sq[next_idx] = -1.0

    next_pose = candidates[next_idx]
    for idx, pose in enumerate(candidates):
      if min_dist_sq[idx] < 0.0:
        continue
      dx = pose.x - next_pose.x
      dy = pose.y - next_pose.y
      dz = pose.z - next_pose.z
      dist_sq = (dx * dx) + (dy * dy) + (dz * dz)
      if dist_sq < min_dist_sq[idx]:
        min_dist_sq[idx] = dist_sq

  return [candidates[idx] for idx in selected]


def build_camera_centered_gripper_goal(
  seed_pose,
  tag_pose,
  gripper_to_camera_pose,
  camera_to_tag_dir,
  distance_mm,
  min_base_z_mm,
  wrist_spin_deg=0.0,
  tilt_from_center_deg=0.0,
):
  base_x, base_y, base_z, base_rx, base_ry, base_rz = seed_pose
  tag_x, tag_y, tag_z, _tag_qx, _tag_qy, _tag_qz, _tag_qw = tag_pose
  if distance_mm <= 0.0:
    return None

  look_dir = vec_normalize(camera_to_tag_dir)
  if look_dir is None:
    return None

  seed_gripper_quat = quaternion_normalize(rpy_deg_to_quaternion(base_rx, base_ry, base_rz))
  seed_gripper_pose = (
    float(base_x),
    float(base_y),
    float(base_z),
    seed_gripper_quat[0],
    seed_gripper_quat[1],
    seed_gripper_quat[2],
    seed_gripper_quat[3],
  )
  seed_camera_pose = transform_pose_compose(seed_gripper_pose, gripper_to_camera_pose)
  seed_camera_quat = seed_camera_pose[3], seed_camera_pose[4], seed_camera_pose[5], seed_camera_pose[6]
  seed_camera_z_axis = vec_normalize(quaternion_rotate_vector(seed_camera_quat, (0.0, 0.0, 1.0)))
  if seed_camera_z_axis is None:
    return None

  desired_camera_x = tag_x - (look_dir[0] * distance_mm)
  desired_camera_y = tag_y - (look_dir[1] * distance_mm)
  desired_camera_z = tag_z - (look_dir[2] * distance_mm)

  q_delta = quaternion_from_two_vectors(seed_camera_z_axis, look_dir)
  desired_camera_quat = quaternion_multiply(q_delta, seed_camera_quat)
  if abs(wrist_spin_deg) > 1e-6:
    desired_camera_quat = quaternion_multiply(
      desired_camera_quat,
      quaternion_from_axis_angle((0.0, 0.0, 1.0), wrist_spin_deg),
    )

  desired_camera_pose = (
    desired_camera_x,
    desired_camera_y,
    desired_camera_z,
    desired_camera_quat[0],
    desired_camera_quat[1],
    desired_camera_quat[2],
    desired_camera_quat[3],
  )
  desired_gripper_pose = transform_pose_compose(
    desired_camera_pose,
    transform_pose_inverse(gripper_to_camera_pose),
  )
  if desired_gripper_pose[2] < min_base_z_mm:
    return None

  rx, ry, rz = quaternion_to_rpy_deg(desired_gripper_pose[3:])
  return GoalPose(
    x=desired_gripper_pose[0],
    y=desired_gripper_pose[1],
    z=desired_gripper_pose[2],
    rx=rx,
    ry=ry,
    rz=rz,
    distance_to_tag_mm=distance_mm,
    tilt_from_seed_deg=tilt_from_center_deg,
  )


def goal_pose_key(pose):
  return (
    int(round(float(pose.x))),
    int(round(float(pose.y))),
    int(round(float(pose.z))),
    int(round(float(pose.rx) * 10.0)),
    int(round(float(pose.ry) * 10.0)),
    int(round(float(pose.rz) * 10.0)),
  )


def build_align_goal_pose(seed_pose, tag_pose, gripper_to_camera_pose, distance_mm, min_base_z_mm):
  tag_x, tag_y, tag_z, tag_qx, tag_qy, tag_qz, tag_qw = tag_pose
  if distance_mm <= 0.0:
    return None

  tag_z_axis = vec_normalize(quaternion_rotate_vector((tag_qx, tag_qy, tag_qz, tag_qw), (0.0, 0.0, 1.0)))
  if tag_z_axis is None:
    return None

  # Put the camera on +tag_z side and point camera +Z directly at the target.
  return build_camera_centered_gripper_goal(
    seed_pose,
    (tag_x, tag_y, tag_z, tag_qx, tag_qy, tag_qz, tag_qw),
    gripper_to_camera_pose,
    (-tag_z_axis[0], -tag_z_axis[1], -tag_z_axis[2]),
    distance_mm,
    min_base_z_mm,
  )


def build_goal_pose_sequence(
  seed_pose,
  tag_pose,
  gripper_to_camera_pose,
  sample_count,
  min_distance_mm,
  max_distance_mm,
  max_tilt_deg,
  look_up_bias_deg,
  min_base_z_mm,
  sequence_offset=0,
):
  tag_x, tag_y, tag_z, tag_qx, tag_qy, tag_qz, tag_qw = tag_pose
  n = max(1, int(sample_count))
  if max_distance_mm <= min_distance_mm:
    return [], 0, 0, 0
  if gripper_to_camera_pose is None:
    gripper_to_camera_pose = identity_transform_pose_mm()

  tag_z_axis = vec_normalize(quaternion_rotate_vector((tag_qx, tag_qy, tag_qz, tag_qw), (0.0, 0.0, 1.0)))
  if tag_z_axis is None:
    return [], 0, 0, 0

  # Sample around tag Z-axis (front side): center line from goal->tag is -tag_z.
  center_dir = vec_normalize((-tag_z_axis[0], -tag_z_axis[1], -tag_z_axis[2]))
  if center_dir is None:
    return [], 0, 0, 0

  # Build basis around center direction for cone sampling.
  helper_axis = (0.0, 0.0, 1.0)
  if abs(vec_dot(center_dir, helper_axis)) > 0.95:
    helper_axis = (0.0, 1.0, 0.0)
  basis_x = vec_normalize(vec_cross(helper_axis, center_dir))
  if basis_x is None:
    return [], 0, 0, 0
  basis_y = vec_normalize(vec_cross(center_dir, basis_x))
  if basis_y is None:
    return [], 0, 0, 0

  # Stratified bins over distance + azimuth for more even spread.
  distance_bins = max(1, int(math.ceil(math.sqrt(float(n)))))
  azimuth_bins = max(1, int(math.ceil(float(n) / float(distance_bins))))
  cell_count = max(1, distance_bins * azimuth_bins)
  attempt_count = max(cell_count * 4, n * 8)

  dist_span = max_distance_mm - min_distance_mm
  cos_max_tilt = math.cos(math.radians(max_tilt_deg))
  offset = max(0, int(sequence_offset))
  wrist_spin_range_deg = 35.0

  candidates = []
  skipped_by_distance = 0
  skipped_by_tilt = 0
  skipped_by_height = 0

  for attempt_idx in range(attempt_count):
    if len(candidates) >= n:
      break

    # Cycle through all (distance, azimuth) bins in a permuted order.
    if cell_count > 1:
      cell_linear_idx = (offset + (attempt_idx * (cell_count - 1))) % cell_count
    else:
      cell_linear_idx = 0
    cycle_idx = attempt_idx // cell_count
    distance_bin = cell_linear_idx // azimuth_bins
    azimuth_bin = cell_linear_idx % azimuth_bins

    # Bin-local jitter (changes across refill rounds) keeps bins filled while avoiding duplicates.
    dist_phase = (0.5 + ((offset + cycle_idx) * 0.6180339887498949)) % 1.0
    az_phase = (0.5 + ((offset + cycle_idx) * 0.3819660112501051)) % 1.0

    u_dist = (distance_bin + dist_phase) / float(distance_bins)
    u_dist = max(0.0, min(0.999999, u_dist))
    dist_to_tag = min_distance_mm + (dist_span * u_dist)
    if dist_to_tag < min_distance_mm or dist_to_tag > max_distance_mm:
      skipped_by_distance += 1
      continue

    u_az = (azimuth_bin + az_phase) / float(azimuth_bins)
    u_az = max(0.0, min(0.999999, u_az))
    phi = 2.0 * math.pi * u_az
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    # Equal-area cone sampling for tilt around center direction.
    u_cap = (0.5 + ((attempt_idx + offset) * 0.7548776662466927)) % 1.0
    cos_theta = 1.0 - (u_cap * (1.0 - cos_max_tilt))
    cos_theta = max(-1.0, min(1.0, cos_theta))
    sin_theta = math.sqrt(max(0.0, 1.0 - (cos_theta * cos_theta)))

    goal_to_tag = vec_normalize((
      (center_dir[0] * cos_theta) + (basis_x[0] * sin_theta * cos_phi) + (basis_y[0] * sin_theta * sin_phi),
      (center_dir[1] * cos_theta) + (basis_x[1] * sin_theta * cos_phi) + (basis_y[1] * sin_theta * sin_phi),
      (center_dir[2] * cos_theta) + (basis_x[2] * sin_theta * cos_phi) + (basis_y[2] * sin_theta * sin_phi),
    ))
    if goal_to_tag is None:
      continue

    # Apply a small upward camera bias so the tag sits closer to image center.
    if abs(look_up_bias_deg) > 1e-6:
      look_up_axis = vec_cross(goal_to_tag, (0.0, 0.0, 1.0))
      if vec_norm(look_up_axis) > 1e-9:
        biased = rotate_vector_about_axis(goal_to_tag, look_up_axis, look_up_bias_deg)
        if biased is not None:
          goal_to_tag = biased

    tilt_from_center_deg = angle_between_vectors_deg(center_dir, goal_to_tag)
    if tilt_from_center_deg > (max_tilt_deg + 1e-6):
      skipped_by_tilt += 1
      continue

    # Add deterministic wrist (Link6 local Z) variation while preserving look-at direction.
    spin_phase = (
      0.5
      + (azimuth_bin * 0.6180339887498949)
      + ((distance_bin + cycle_idx + offset) * 0.4142135623730950)
    ) % 1.0
    spin_deg = (2.0 * spin_phase - 1.0) * wrist_spin_range_deg

    goal = build_camera_centered_gripper_goal(
      seed_pose,
      (tag_x, tag_y, tag_z, tag_qx, tag_qy, tag_qz, tag_qw),
      gripper_to_camera_pose,
      goal_to_tag,
      dist_to_tag,
      min_base_z_mm,
      wrist_spin_deg=spin_deg,
      tilt_from_center_deg=tilt_from_center_deg,
    )
    if goal is None:
      skipped_by_height += 1
      continue
    candidates.append(goal)

  if not candidates:
    return [], skipped_by_distance, skipped_by_tilt, skipped_by_height
  return candidates[:n], skipped_by_distance, skipped_by_tilt, skipped_by_height


def wrap_angle_deg(angle_deg):
  wrapped = (float(angle_deg) + 180.0) % 360.0
  return wrapped - 180.0


def build_eye_to_hand_goal_pose_sequence(seed_pose, sample_count, min_base_z_mm, sequence_offset=0):
  base_x, base_y, base_z, base_rx, base_ry, base_rz = seed_pose
  n = max(1, int(sample_count))
  offset = max(0, int(sequence_offset))

  xy_span_mm = 90.0
  z_span_mm = 70.0
  rot_span_deg = 20.0
  attempt_count = max(32, n * 14)

  candidates = []
  skipped_by_height = 0
  seen = set()

  # Include current pose first to guarantee one valid anchor sample.
  seed_goal = GoalPose(
    x=float(base_x),
    y=float(base_y),
    z=float(base_z),
    rx=wrap_angle_deg(base_rx),
    ry=wrap_angle_deg(base_ry),
    rz=wrap_angle_deg(base_rz),
    distance_to_tag_mm=0.0,
    tilt_from_seed_deg=0.0,
  )
  if float(base_z) >= float(min_base_z_mm):
    seed_key = goal_pose_key(seed_goal)
    seen.add(seed_key)
    candidates.append(seed_goal)
  else:
    skipped_by_height += 1

  for attempt_idx in range(attempt_count):
    if len(candidates) >= n:
      break
    phase = float(offset + attempt_idx + 1)
    u1 = (0.5 + (phase * 0.6180339887498949)) % 1.0
    u2 = (0.5 + (phase * 0.4142135623730950)) % 1.0
    u3 = (0.5 + (phase * 0.7548776662466927)) % 1.0
    u4 = (0.5 + (phase * 0.3819660112501051)) % 1.0
    u5 = (0.5 + (phase * 0.5698402909980532)) % 1.0
    u6 = (0.5 + (phase * 0.2775557561562891)) % 1.0

    radius = xy_span_mm * math.sqrt(max(0.0, u1))
    theta = 2.0 * math.pi * u2
    dx = radius * math.cos(theta)
    dy = radius * math.sin(theta)
    dz = (2.0 * u3 - 1.0) * z_span_mm

    px = float(base_x) + dx
    py = float(base_y) + dy
    pz = float(base_z) + dz
    if pz < float(min_base_z_mm):
      skipped_by_height += 1
      continue

    rx = wrap_angle_deg(float(base_rx) + ((2.0 * u4 - 1.0) * rot_span_deg))
    ry = wrap_angle_deg(float(base_ry) + ((2.0 * u5 - 1.0) * rot_span_deg))
    rz = wrap_angle_deg(float(base_rz) + ((2.0 * u6 - 1.0) * rot_span_deg))
    rot_delta = math.sqrt(
      (wrap_angle_deg(rx - float(base_rx)) ** 2)
      + (wrap_angle_deg(ry - float(base_ry)) ** 2)
      + (wrap_angle_deg(rz - float(base_rz)) ** 2)
    )
    dist_mm = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    goal = GoalPose(
      x=px,
      y=py,
      z=pz,
      rx=rx,
      ry=ry,
      rz=rz,
      distance_to_tag_mm=dist_mm,
      tilt_from_seed_deg=rot_delta,
    )
    key = goal_pose_key(goal)
    if key in seen:
      continue
    seen.add(key)
    candidates.append(goal)

  return candidates[:n], skipped_by_height


class CalibGui(QtWidgets.QWidget):
  def __init__(self, ros_if, parent=None):
    super().__init__(parent)
    self.ros_if = ros_if
    self._ui_settings = QtCore.QSettings("DOBOT", "camera_calibration_gui")
    self.calib_process = None
    self.align_in_flight = False
    self.generated_goals = []
    self.auto_running = False
    self.auto_state = "idle"
    self.auto_goal_index = 0
    self.auto_capture_retry = 0
    self.auto_target_pose = None
    self.auto_move_deadline = 0.0
    self.auto_settle_deadline = 0.0
    self._save_yaml_ready = False
    self.setWindowTitle("Camera Calibration")
    self._build_ui()
    self._restore_ui_settings()
    self.ros_if.apply_calibration_mode_tool(self._current_calibration_mode())
    self._connect_ui_setting_signals()
    self._update_mode_dependent_ui()
    self._update_window_title()
    self._persist_all_ui_settings(log_changes=False)
    self._log_ui_settings_snapshot("UI settings active at startup")
    self._setup_monitors()

  def _build_ui(self):
    root_layout = QtWidgets.QHBoxLayout()

    controls_widget = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(controls_widget)

    form = QtWidgets.QFormLayout()
    self.calibration_mode = QtWidgets.QComboBox()
    self.calibration_mode.addItem("Eye-on-Hand", CALIB_MODE_EYE_ON_HAND)
    self.calibration_mode.addItem("Eye-to-Hand", CALIB_MODE_EYE_TO_HAND)
    self.base_frame = QtWidgets.QLineEdit("base_link")
    self.gripper_frame = QtWidgets.QLineEdit("Link6")
    self.camera_frame = QtWidgets.QLineEdit("camera_link")
    self.target_frame = QtWidgets.QLineEdit("tag_frame")
    self.output_path = QtWidgets.QLineEdit(default_output_path())
    self.min_samples = QtWidgets.QSpinBox()
    self.min_samples.setMinimum(3)
    self.min_samples.setMaximum(1000)
    self.min_samples.setValue(21)
    self.min_tag_distance_mm = QtWidgets.QDoubleSpinBox()
    self.min_tag_distance_mm.setRange(50.0, 5000.0)
    self.min_tag_distance_mm.setSingleStep(10.0)
    self.min_tag_distance_mm.setValue(250.0)
    self.min_tag_distance_mm.setSuffix(" mm")
    self.max_tag_distance_mm = QtWidgets.QDoubleSpinBox()
    self.max_tag_distance_mm.setRange(50.0, 5000.0)
    self.max_tag_distance_mm.setSingleStep(10.0)
    self.max_tag_distance_mm.setValue(800.0)
    self.max_tag_distance_mm.setSuffix(" mm")
    self.max_tilt_deg = QtWidgets.QDoubleSpinBox()
    self.max_tilt_deg.setRange(1.0, 180.0)
    self.max_tilt_deg.setSingleStep(1.0)
    self.max_tilt_deg.setValue(30.0)
    self.max_tilt_deg.setSuffix(" deg")
    self.look_up_bias_deg = QtWidgets.QDoubleSpinBox()
    self.look_up_bias_deg.setRange(-30.0, 30.0)
    self.look_up_bias_deg.setSingleStep(0.5)
    self.look_up_bias_deg.setValue(10.0)
    self.look_up_bias_deg.setSuffix(" deg")
    self.min_base_z_mm = QtWidgets.QDoubleSpinBox()
    self.min_base_z_mm.setRange(-1000.0, 5000.0)
    self.min_base_z_mm.setSingleStep(10.0)
    self.min_base_z_mm.setValue(200.0)
    self.min_base_z_mm.setSuffix(" mm")
    self.tag_tool_offset = QtWidgets.QLineEdit("0,0,0,0,0,0")
    self.apply_tool_btn = QtWidgets.QPushButton("Apply Tag Tool")

    form.addRow("Calibration mode", self.calibration_mode)
    form.addRow("Base frame", self.base_frame)
    form.addRow("Gripper frame", self.gripper_frame)
    form.addRow("Camera frame", self.camera_frame)
    form.addRow("Target frame", self.target_frame)
    form.addRow("Output YAML path", self.output_path)
    form.addRow("Minimum samples", self.min_samples)
    form.addRow("Min tag distance", self.min_tag_distance_mm)
    form.addRow("Max tag distance", self.max_tag_distance_mm)
    form.addRow("Max tilt from tag Z", self.max_tilt_deg)
    form.addRow("Look up bias", self.look_up_bias_deg)
    form.addRow("Min base Z height", self.min_base_z_mm)
    form.addRow("Tag tool offset", self.tag_tool_offset)
    layout.addLayout(form)

    button_layout = QtWidgets.QHBoxLayout()
    self.start_btn = QtWidgets.QPushButton("Start")
    self.stop_btn = QtWidgets.QPushButton("Stop")
    self.align_btn = QtWidgets.QPushButton("Align")
    self.start_btn.setEnabled(False)
    self.stop_btn.setEnabled(False)
    button_layout.addWidget(self.start_btn)
    button_layout.addWidget(self.stop_btn)
    button_layout.addWidget(self.align_btn)
    layout.addLayout(button_layout)

    action_layout = QtWidgets.QHBoxLayout()
    self.add_btn = QtWidgets.QPushButton("Generate Pose")
    action_layout.addWidget(self.apply_tool_btn)
    self.save_btn = QtWidgets.QPushButton("Save YAML")
    action_layout.addWidget(self.add_btn)
    action_layout.addWidget(self.save_btn)
    layout.addLayout(action_layout)

    self.status = QtWidgets.QPlainTextEdit()
    self.status.setReadOnly(True)
    layout.addWidget(self.status)

    overlay_group = QtWidgets.QGroupBox("ArUco Overlay (/aruco_overlay)")
    overlay_layout = QtWidgets.QVBoxLayout(overlay_group)
    self.overlay_label = QtWidgets.QLabel("Waiting for /aruco_overlay ...")
    self.overlay_label.setAlignment(QtCore.Qt.AlignCenter)
    self.overlay_label.setMinimumSize(640, 360)
    self.overlay_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
    self.overlay_label.setStyleSheet(
      "QLabel { background-color: #101010; color: #d0d0d0; border: 1px solid #444; }"
    )
    overlay_layout.addWidget(self.overlay_label)

    root_layout.addWidget(controls_widget, 0)
    root_layout.addWidget(overlay_group, 1)

    self.setLayout(root_layout)

    self.start_btn.clicked.connect(self.start_automation)
    self.stop_btn.clicked.connect(self.stop_calibrator)
    self.align_btn.clicked.connect(self.align_to_tag)
    self.add_btn.clicked.connect(self.generate_goal_poses)
    self.apply_tool_btn.clicked.connect(self.apply_tag_tool_offset)
    self.save_btn.clicked.connect(self.save_yaml)

  def log(self, text):
    self.status.appendPlainText(text)

  def _set_save_yaml_ready(self, ready):
    ready = bool(ready)
    if ready == self._save_yaml_ready:
      return
    self._save_yaml_ready = ready
    if ready:
      self.save_btn.setStyleSheet(
        "QPushButton { background-color: #2e7d32; color: #ffffff; font-weight: 600; }"
      )
      self.log("[ui] Save YAML highlighted: calibration finished and calibrated TF is live.")
      return

    self.save_btn.setStyleSheet("")

  def _current_calibration_mode(self):
    return normalize_calibration_mode(self.calibration_mode.currentData())

  def _set_calibration_mode(self, mode):
    normalized = normalize_calibration_mode(mode)
    index = self.calibration_mode.findData(normalized)
    if index < 0:
      index = 0
    self.calibration_mode.setCurrentIndex(index)

  def _update_window_title(self):
    mode = self._current_calibration_mode()
    suffix = "Eye-to-Hand" if mode == CALIB_MODE_EYE_TO_HAND else "Eye-on-Hand"
    self.setWindowTitle(f"Camera Calibration ({suffix})")

  def _update_mode_dependent_ui(self):
    mode = self._current_calibration_mode()
    eye_to_hand = mode == CALIB_MODE_EYE_TO_HAND
    self.align_btn.setEnabled((not self.auto_running) and (not self.align_in_flight) and (not eye_to_hand))
    if eye_to_hand:
      self.align_btn.setToolTip("Align is only used for eye-on-hand mode.")
      self.min_tag_distance_mm.setToolTip("Visibility gate: minimum camera-to-tag distance for sample capture.")
      self.max_tag_distance_mm.setToolTip("Visibility gate: maximum camera-to-tag distance for sample capture.")
      self.max_tilt_deg.setToolTip("Visibility gate: maximum tag tilt for sample capture.")
      self.look_up_bias_deg.setToolTip("Eye-to-hand generator ignores look-up bias.")
    else:
      self.align_btn.setToolTip("Move to the closest IK-valid camera-centered view of the target.")
      self.min_tag_distance_mm.setToolTip("")
      self.max_tag_distance_mm.setToolTip("")
      self.max_tilt_deg.setToolTip("")
      self.look_up_bias_deg.setToolTip("")

  @staticmethod
  def _parse_tool_offset_values(raw_text):
    text = str(raw_text or "").replace("{", "").replace("}", "").strip()
    if not text:
      return None
    tokens = [token.strip() for token in text.split(",")]
    if len(tokens) != 6:
      return None
    values = []
    for token in tokens:
      try:
        values.append(float(token))
      except ValueError:
        return None
    return values

  def _on_calibration_mode_changed(self, *_):
    self.ros_if.apply_calibration_mode_tool(self._current_calibration_mode())
    if self.generated_goals and (not self.auto_running):
      self.generated_goals = []
      self.start_btn.setEnabled(False)
      self.log("Calibration mode changed: cleared generated poses.")
    self._save_setting(
      "calibration_mode",
      "Calibration mode",
      self._current_calibration_mode(),
      display_value=self.calibration_mode.currentText(),
      emit_log=True,
    )
    self._update_window_title()
    self._update_mode_dependent_ui()

  def _settings_key(self, name):
    return f"ui/{name}"

  @staticmethod
  def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

  @staticmethod
  def _format_numeric_widget_value(widget):
    if isinstance(widget, QtWidgets.QSpinBox):
      numeric_value = int(widget.value())
      display_value = f"{numeric_value}"
    else:
      numeric_value = float(widget.value())
      display_value = f"{numeric_value:.2f}"
    suffix = str(widget.suffix() or "").strip()
    if suffix:
      display_value = f"{display_value} {suffix}"
    return numeric_value, display_value

  def _save_setting(self, name, label, value, display_value=None, emit_log=True):
    key = self._settings_key(name)
    previous = self._ui_settings.value(key, None)
    self._ui_settings.setValue(key, value)
    self._ui_settings.sync()

    if (not emit_log) or (previous is not None and str(previous) == str(value)):
      return
    shown = str(display_value) if display_value is not None else str(value)
    self.log(f"[ui] {label} = {shown} (saved)")

  def _read_text_setting(self, name, fallback):
    key = self._settings_key(name)
    if not self._ui_settings.contains(key):
      return str(fallback)
    value = self._ui_settings.value(key, fallback)
    if value is None:
      return str(fallback)
    return str(value)

  def _read_int_setting(self, name, fallback, minimum, maximum):
    key = self._settings_key(name)
    if not self._ui_settings.contains(key):
      return int(fallback)

    raw = self._ui_settings.value(key, fallback)
    try:
      value = int(raw)
    except (TypeError, ValueError):
      try:
        value = int(float(raw))
      except (TypeError, ValueError):
        value = int(fallback)
    return int(self._clamp(value, int(minimum), int(maximum)))

  def _read_float_setting(self, name, fallback, minimum, maximum):
    key = self._settings_key(name)
    if not self._ui_settings.contains(key):
      return float(fallback)

    raw = self._ui_settings.value(key, fallback)
    try:
      value = float(raw)
    except (TypeError, ValueError):
      value = float(fallback)
    return float(self._clamp(value, float(minimum), float(maximum)))

  def _restore_ui_settings(self):
    self._set_calibration_mode(
      self._read_text_setting("calibration_mode", self.ros_if.get_default_calibration_mode())
    )
    self.base_frame.setText(self._read_text_setting("base_frame", self.base_frame.text()))
    self.gripper_frame.setText(self._read_text_setting("gripper_frame", self.gripper_frame.text()))
    self.camera_frame.setText(self._read_text_setting("camera_frame", self.camera_frame.text()))
    self.target_frame.setText(self._read_text_setting("target_frame", self.target_frame.text()))
    self.output_path.setText(
      normalize_output_path_setting(self._read_text_setting("output_path", self.output_path.text()))
    )
    self.tag_tool_offset.setText(
      self._read_text_setting("tag_tool_offset", self.tag_tool_offset.text())
    )

    self.min_samples.setValue(
      self._read_int_setting(
        "min_samples",
        self.min_samples.value(),
        self.min_samples.minimum(),
        self.min_samples.maximum(),
      )
    )
    self.min_tag_distance_mm.setValue(
      self._read_float_setting(
        "min_tag_distance_mm",
        self.min_tag_distance_mm.value(),
        self.min_tag_distance_mm.minimum(),
        self.min_tag_distance_mm.maximum(),
      )
    )
    self.max_tag_distance_mm.setValue(
      self._read_float_setting(
        "max_tag_distance_mm",
        self.max_tag_distance_mm.value(),
        self.max_tag_distance_mm.minimum(),
        self.max_tag_distance_mm.maximum(),
      )
    )
    self.max_tilt_deg.setValue(
      self._read_float_setting(
        "max_tilt_deg",
        self.max_tilt_deg.value(),
        self.max_tilt_deg.minimum(),
        self.max_tilt_deg.maximum(),
      )
    )
    self.look_up_bias_deg.setValue(
      self._read_float_setting(
        "look_up_bias_deg",
        self.look_up_bias_deg.value(),
        self.look_up_bias_deg.minimum(),
        self.look_up_bias_deg.maximum(),
      )
    )
    self.min_base_z_mm.setValue(
      self._read_float_setting(
        "min_base_z_mm",
        self.min_base_z_mm.value(),
        self.min_base_z_mm.minimum(),
        self.min_base_z_mm.maximum(),
      )
    )

  def _connect_ui_setting_signals(self):
    self.calibration_mode.currentIndexChanged.connect(self._on_calibration_mode_changed)
    self.base_frame.editingFinished.connect(
      partial(self._save_line_edit_setting, "base_frame", "Base frame", self.base_frame)
    )
    self.gripper_frame.editingFinished.connect(
      partial(self._save_line_edit_setting, "gripper_frame", "Gripper frame", self.gripper_frame)
    )
    self.camera_frame.editingFinished.connect(
      partial(self._save_line_edit_setting, "camera_frame", "Camera frame", self.camera_frame)
    )
    self.target_frame.editingFinished.connect(
      partial(self._save_line_edit_setting, "target_frame", "Target frame", self.target_frame)
    )
    self.output_path.editingFinished.connect(
      partial(self._save_line_edit_setting, "output_path", "Output YAML path", self.output_path)
    )
    self.tag_tool_offset.editingFinished.connect(
      partial(self._save_line_edit_setting, "tag_tool_offset", "Tag tool offset", self.tag_tool_offset)
    )

    self.min_samples.valueChanged.connect(
      partial(self._save_spin_setting, "min_samples", "Minimum samples", self.min_samples)
    )
    self.min_tag_distance_mm.valueChanged.connect(
      partial(self._save_spin_setting, "min_tag_distance_mm", "Min tag distance", self.min_tag_distance_mm)
    )
    self.max_tag_distance_mm.valueChanged.connect(
      partial(self._save_spin_setting, "max_tag_distance_mm", "Max tag distance", self.max_tag_distance_mm)
    )
    self.max_tilt_deg.valueChanged.connect(
      partial(self._save_spin_setting, "max_tilt_deg", "Max tilt from tag Z", self.max_tilt_deg)
    )
    self.look_up_bias_deg.valueChanged.connect(
      partial(self._save_spin_setting, "look_up_bias_deg", "Look up bias", self.look_up_bias_deg)
    )
    self.min_base_z_mm.valueChanged.connect(
      partial(self._save_spin_setting, "min_base_z_mm", "Min base Z height", self.min_base_z_mm)
    )

  def _save_line_edit_setting(self, name, label, widget):
    value = str(widget.text() or "").strip()
    if value != widget.text():
      widget.setText(value)
    self._save_setting(name, label, value, display_value=value, emit_log=True)

  def _save_spin_setting(self, name, label, widget, *_):
    numeric_value, display_value = self._format_numeric_widget_value(widget)
    self._save_setting(name, label, numeric_value, display_value=display_value, emit_log=True)

  def _persist_all_ui_settings(self, log_changes=False):
    self._save_setting(
      "calibration_mode",
      "Calibration mode",
      self._current_calibration_mode(),
      display_value=self.calibration_mode.currentText(),
      emit_log=log_changes,
    )
    self._save_setting("base_frame", "Base frame", self.base_frame.text().strip(), emit_log=log_changes)
    self._save_setting("gripper_frame", "Gripper frame", self.gripper_frame.text().strip(), emit_log=log_changes)
    self._save_setting("camera_frame", "Camera frame", self.camera_frame.text().strip(), emit_log=log_changes)
    self._save_setting("target_frame", "Target frame", self.target_frame.text().strip(), emit_log=log_changes)
    self._save_setting("output_path", "Output YAML path", self.output_path.text().strip(), emit_log=log_changes)
    self._save_setting("tag_tool_offset", "Tag tool offset", self.tag_tool_offset.text().strip(), emit_log=log_changes)

    min_samples_value, min_samples_display = self._format_numeric_widget_value(self.min_samples)
    self._save_setting(
      "min_samples",
      "Minimum samples",
      min_samples_value,
      display_value=min_samples_display,
      emit_log=log_changes,
    )
    min_tag_distance_value, min_tag_distance_display = self._format_numeric_widget_value(self.min_tag_distance_mm)
    self._save_setting(
      "min_tag_distance_mm",
      "Min tag distance",
      min_tag_distance_value,
      display_value=min_tag_distance_display,
      emit_log=log_changes,
    )
    max_tag_distance_value, max_tag_distance_display = self._format_numeric_widget_value(self.max_tag_distance_mm)
    self._save_setting(
      "max_tag_distance_mm",
      "Max tag distance",
      max_tag_distance_value,
      display_value=max_tag_distance_display,
      emit_log=log_changes,
    )
    max_tilt_value, max_tilt_display = self._format_numeric_widget_value(self.max_tilt_deg)
    self._save_setting(
      "max_tilt_deg",
      "Max tilt from tag Z",
      max_tilt_value,
      display_value=max_tilt_display,
      emit_log=log_changes,
    )
    look_up_bias_value, look_up_bias_display = self._format_numeric_widget_value(self.look_up_bias_deg)
    self._save_setting(
      "look_up_bias_deg",
      "Look up bias",
      look_up_bias_value,
      display_value=look_up_bias_display,
      emit_log=log_changes,
    )
    min_base_z_value, min_base_z_display = self._format_numeric_widget_value(self.min_base_z_mm)
    self._save_setting(
      "min_base_z_mm",
      "Min base Z height",
      min_base_z_value,
      display_value=min_base_z_display,
      emit_log=log_changes,
    )

  def _log_ui_settings_snapshot(self, title):
    self.log(f"[ui] {title}:")
    self.log(f"[ui] Calibration mode = {self.calibration_mode.currentText()} ({self._current_calibration_mode()})")
    self.log(f"[ui] Base frame = {self.base_frame.text().strip()}")
    self.log(f"[ui] Gripper frame = {self.gripper_frame.text().strip()}")
    self.log(f"[ui] Camera frame = {self.camera_frame.text().strip()}")
    self.log(f"[ui] Target frame = {self.target_frame.text().strip()}")
    self.log(f"[ui] Output YAML path = {self.output_path.text().strip()}")
    self.log(f"[ui] Tag tool offset = {self.tag_tool_offset.text().strip()}")
    self.log(f"[ui] Minimum samples = {int(self.min_samples.value())}")
    self.log(f"[ui] Min tag distance = {float(self.min_tag_distance_mm.value()):.2f} mm")
    self.log(f"[ui] Max tag distance = {float(self.max_tag_distance_mm.value()):.2f} mm")
    self.log(f"[ui] Max tilt from tag Z = {float(self.max_tilt_deg.value()):.2f} deg")
    self.log(f"[ui] Look up bias = {float(self.look_up_bias_deg.value()):.2f} deg")
    self.log(f"[ui] Min base Z height = {float(self.min_base_z_mm.value()):.2f} mm")

  def _setup_monitors(self):
    # Poll calibrator subprocess; if it exits (e.g., user stops it externally), close the GUI.
    self.process_watch = QtCore.QTimer(self)
    self.process_watch.setInterval(300)
    self.process_watch.timeout.connect(self._check_calibrator_process)
    self.process_watch.start()

    self.overlay_watch = QtCore.QTimer(self)
    self.overlay_watch.setInterval(100)
    self.overlay_watch.timeout.connect(self._refresh_overlay)
    self.overlay_watch.start()

    self.auto_watch = QtCore.QTimer(self)
    self.auto_watch.setInterval(100)
    self.auto_watch.timeout.connect(self._run_auto_step)
    self.auto_watch.start()

  def _set_controls_during_auto(self, running):
    for widget in [
      self.calibration_mode,
      self.base_frame,
      self.gripper_frame,
      self.camera_frame,
      self.target_frame,
      self.output_path,
      self.tag_tool_offset,
      self.min_samples,
      self.min_tag_distance_mm,
      self.max_tag_distance_mm,
      self.max_tilt_deg,
      self.look_up_bias_deg,
      self.min_base_z_mm,
      self.apply_tool_btn,
      self.add_btn,
      self.save_btn,
    ]:
      widget.setEnabled(not running)

    self.start_btn.setEnabled((not running) and bool(self.generated_goals))
    self.stop_btn.setEnabled(running or (self.calib_process is not None))
    self._update_mode_dependent_ui()

  def start_calibrator(self):
    if self.calib_process is not None:
      return True

    cmd = [
      "ros2",
      "run",
      "camera_calibration",
      "eye_on_hand_calibrator",
      "--ros-args",
      "-p",
      f"calibration_mode:={self._current_calibration_mode()}",
      "-p",
      f"base_frame:={self.base_frame.text()}",
      "-p",
      f"gripper_frame:={self.gripper_frame.text()}",
      "-p",
      f"camera_frame:={self.camera_frame.text()}",
      "-p",
      f"target_frame:={self.target_frame.text()}",
      "-p",
      f"output_path:={self.output_path.text()}",
      "-p",
      f"min_samples:={self.min_samples.value()}",
    ]
    self.log("Starting calibrator...")
    try:
      self.calib_process = subprocess.Popen(cmd)
      self.log("Calibrator started.")
      self.stop_btn.setEnabled(True)
      return True
    except Exception as exc:
      self.log(f"Failed to start calibrator: {exc}")
      self.calib_process = None
      self.stop_btn.setEnabled(False)
      return False

  def start_automation(self):
    self._persist_all_ui_settings(log_changes=False)
    self._set_save_yaml_ready(False)
    mode = self._current_calibration_mode()
    self.ros_if.apply_calibration_mode_tool(mode)
    if mode == CALIB_MODE_EYE_TO_HAND:
      values = self._parse_tool_offset_values(self.tag_tool_offset.text())
      if values is None:
        self.log("Eye-to-hand requires valid tag tool offset: x,y,z,rx,ry,rz")
        return
      success, detail = self.ros_if.apply_tag_tool_offset(values)
      if not success:
        self.log(f"[tag_tool] ERROR: {detail}")
        return
      self.log(f"[tag_tool] OK: {detail}")
    if self.auto_running:
      self.log("Auto calibration is already running.")
      return
    if not self.generated_goals:
      self.log("No valid generated poses yet. Click 'Generate Pose' first.")
      self.start_btn.setEnabled(False)
      return

    if self.calib_process is not None:
      self.log("Restarting calibrator with current GUI settings...")
      self._stop_calibrator(clear_generated_goals=False)

    if not self.start_calibrator():
      return

    if not self.ros_if.ensure_motion_ready(timeout_sec=0.5):
      self.log("Motion service is not ready: /dobot_bringup_ros2/srv/MovJ")
      return

    ordered_goals = self._order_goals_for_motion(self.generated_goals)
    parent_frame = self.base_frame.text().strip() or "base_link"
    if not self.ros_if.publish_goal_pose_transforms(parent_frame, ordered_goals):
      self.log("Failed to republish ordered goal pose TFs.")
      return

    self.generated_goals = ordered_goals
    self.auto_goal_index = 0
    self.auto_capture_retry = 0
    self.auto_target_pose = None
    self.auto_state = "resetting"
    self.auto_running = True
    self._set_controls_during_auto(True)
    self.log("Execution order: nearest distance first, then lowest Z first.")
    self.log(
      f"Auto run started with {len(ordered_goals)} pose(s): move -> capture -> compute."
    )
    dispatched = self._call_trigger_async("reset_samples", self._on_reset_samples_done)
    if not dispatched:
      self._finish_automation(
        False,
        "Failed to reset samples. Ensure calibrator services are available.",
      )

  def _clear_generated_goals(self):
    had_goals = bool(self.generated_goals)
    self.generated_goals = []
    self.start_btn.setEnabled(False)
    if had_goals:
      self.log("Generated poses cleared. Click 'Generate Pose' to enable Start again.")

  def _stop_calibrator(self, clear_generated_goals):
    if clear_generated_goals:
      self._set_save_yaml_ready(False)
      self._clear_generated_goals()

    if self.auto_running:
      self.log("Stopping auto calibration sequence...")
      self.ros_if.stop_motion()
      self._finish_automation(False, "Auto run canceled by user.")

    if self.calib_process is None:
      self.log("Calibrator is not running.")
      self.stop_btn.setEnabled(False)
      return
    self.log("Stopping calibrator...")
    try:
      self.calib_process.terminate()
      try:
        self.calib_process.wait(timeout=3.0)
      except subprocess.TimeoutExpired:
        self.calib_process.kill()
    finally:
      self.calib_process = None
      self.stop_btn.setEnabled(self.auto_running)
      self.log("Calibrator stopped.")

  def stop_calibrator(self, *_):
    self._stop_calibrator(clear_generated_goals=True)

  @staticmethod
  def _order_goals_for_motion(goals):
    return sorted(
      goals,
      key=lambda pose: (
        float(pose.distance_to_tag_mm),
        float(pose.z),
        float(pose.tilt_from_seed_deg),
        float(pose.x),
        float(pose.y),
      ),
    )

  def _lookup_gripper_to_camera_pose(self):
    gripper_frame = self.gripper_frame.text().strip() or "Link6"
    camera_frame = self.camera_frame.text().strip() or "camera_link"
    if gripper_frame == camera_frame:
      return identity_transform_pose_mm()

    pose = self.ros_if.lookup_pose_mm(gripper_frame, camera_frame)
    if pose is None:
      self.log(
        f"Cannot resolve TF '{gripper_frame}' -> '{camera_frame}'. "
        "Need a valid camera mount transform for camera-centered motion."
      )
      return None
    return pose

  def align_to_tag(self):
    self._persist_all_ui_settings(log_changes=False)
    self.ros_if.apply_calibration_mode_tool(self._current_calibration_mode())
    if self._current_calibration_mode() == CALIB_MODE_EYE_TO_HAND:
      self.log("Align is disabled in eye-to-hand mode. Use Generate Pose + Start.")
      return
    if self.auto_running:
      self.log("Cannot align while auto calibration is running.")
      return
    if self.align_in_flight:
      self.log("Align command is already in progress.")
      return

    seed_pose = self.ros_if.get_latest_tcp()
    if seed_pose is None:
      self.log("No live TCP pose yet on dobot_msgs_v4/msg/ToolVectorActual. Cannot align.")
      return

    min_distance_mm = float(self.min_tag_distance_mm.value())
    max_distance_mm = float(self.max_tag_distance_mm.value())
    min_base_z_mm = float(self.min_base_z_mm.value())
    if min_distance_mm <= 0.0:
      self.log("Min tag distance must be > 0 for align.")
      return
    if max_distance_mm < min_distance_mm:
      max_distance_mm = min_distance_mm

    parent_frame = self.base_frame.text().strip() or "base_link"
    target_frame = self.target_frame.text().strip() or "tag_frame"
    tag_pose = self.ros_if.lookup_pose_mm(parent_frame, target_frame)
    if tag_pose is None:
      self.log(
        f"Cannot resolve TF '{parent_frame}' -> '{target_frame}'. "
        "Need a valid tag transform before align."
      )
      return

    gripper_to_camera_pose = self._lookup_gripper_to_camera_pose()
    if gripper_to_camera_pose is None:
      return

    if not self.ros_if.ensure_motion_ready(timeout_sec=0.5):
      self.log("Motion service is not ready: /dobot_bringup_ros2/srv/MovJ")
      return

    self.ros_if.clear_ik_joint_cache()
    if not self.ros_if.ensure_ik_ready(timeout_sec=0.25):
      self.log("[align] IK service is not ready. Joint-mode MovJ requires IK.")
      return

    distance_count = 1 if max_distance_mm <= min_distance_mm else 12
    distance_candidates = []
    for index in range(distance_count):
      if distance_count == 1:
        distance = min_distance_mm
      else:
        distance = min_distance_mm + ((max_distance_mm - min_distance_mm) * index / float(distance_count - 1))
      distance_candidates.append(distance)

    goal = None
    rejected_details = []
    for distance in distance_candidates:
      candidate = build_align_goal_pose(
        seed_pose,
        tag_pose,
        gripper_to_camera_pose,
        distance,
        min_base_z_mm,
      )
      if candidate is None:
        rejected_details.append(f"{distance:.0f} mm: geometry/min-z")
        continue

      ok, detail = self.ros_if.check_pose_reachable_ik(candidate, timeout_sec=0.4)
      if ok is True:
        goal = candidate
        break
      rejected_details.append(f"{distance:.0f} mm: {detail}")

    if goal is None:
      self.log(
        f"[align] No IK-valid camera-centered pose in distance range "
        f"{min_distance_mm:.0f}-{max_distance_mm:.0f} mm."
      )
      for detail in rejected_details[:6]:
        self.log(f"  [align] reject {detail}")
      if len(rejected_details) > 6:
        self.log(f"  [align] ... {len(rejected_details) - 6} more rejected distance(s).")
      return

    self.ros_if.publish_goal_pose_transforms(parent_frame, [goal])
    self.align_in_flight = True
    self.align_btn.setEnabled(False)
    self.log(
      f"[align] Camera-centered move ({goal.distance_to_tag_mm:.0f} mm) -> "
      f"x={goal.x:.1f} y={goal.y:.1f} z={goal.z:.1f} "
      f"rx={goal.rx:.1f} ry={goal.ry:.1f} rz={goal.rz:.1f}"
    )
    ik_joints = self.ros_if.get_cached_ik_joint_solution(goal)
    if ik_joints is None:
      self.align_in_flight = False
      self.align_btn.setEnabled(True)
      self.log("[align] IK joint solution missing; aborting move.")
      return
    self.log("[align] Dispatch mode: MovJ joint.")
    future = self.ros_if.send_movj_goal(joint_values=ik_joints)
    if future is None:
      self.align_in_flight = False
      self.align_btn.setEnabled(True)
      self.log("[align] Failed to dispatch motion command.")
      return
    future.add_done_callback(self._on_align_move_done)

  def _on_align_move_done(self, future):
    self.align_in_flight = False
    self._update_mode_dependent_ui()
    try:
      res = future.result()
      if int(getattr(res, "res", 0)) != 0:
        self.log(
          f"[align] Move command rejected: res={res.res}, reply={res.robot_return}"
        )
        return
      self.log("[align] Move command accepted.")
    except Exception as exc:
      self.log(f"[align] Move command failed: {exc}")

  def _check_calibrator_process(self):
    if self.calib_process is None:
      return
    if self.calib_process.poll() is None:
      return
    # Process exited on its own; mirror UI state and close GUI so launch shuts down cleanly.
    self.log("Calibrator process exited; closing GUI.")
    self.auto_running = False
    self.calib_process = None
    self.stop_btn.setEnabled(False)
    QtWidgets.QApplication.instance().quit()

  def _refresh_overlay(self):
    image = self.ros_if.get_latest_overlay_qimage()
    if image is None:
      self.overlay_label.clear()
      self.overlay_label.setText("no camera topics...\nWaiting for /aruco_overlay ...")
      return

    pixmap = QtGui.QPixmap.fromImage(image)
    if pixmap.isNull():
      return

    target_size = self.overlay_label.size()
    if target_size.width() > 1 and target_size.height() > 1:
      pixmap = pixmap.scaled(
        target_size,
        QtCore.Qt.KeepAspectRatio,
        QtCore.Qt.SmoothTransformation,
      )
    self.overlay_label.setText("")
    self.overlay_label.setPixmap(pixmap)

  @staticmethod
  def _angle_error_deg(current_deg, target_deg):
    wrapped = (float(current_deg) - float(target_deg) + 180.0) % 360.0
    return abs(wrapped - 180.0)

  def _goal_reached(self, current_tcp, target_pose):
    pos_tol_mm = 5.0
    rot_tol_deg = 2.0
    if abs(current_tcp[0] - target_pose.x) > pos_tol_mm:
      return False
    if abs(current_tcp[1] - target_pose.y) > pos_tol_mm:
      return False
    if abs(current_tcp[2] - target_pose.z) > pos_tol_mm:
      return False
    if self._angle_error_deg(current_tcp[3], target_pose.rx) > rot_tol_deg:
      return False
    if self._angle_error_deg(current_tcp[4], target_pose.ry) > rot_tol_deg:
      return False
    if self._angle_error_deg(current_tcp[5], target_pose.rz) > rot_tol_deg:
      return False
    return True

  def _check_eye_to_hand_visibility_gate(self, log_rejections):
    mode = self._current_calibration_mode()
    if mode != CALIB_MODE_EYE_TO_HAND:
      return True, "visibility gate disabled for eye-on-hand mode"

    camera_frame = self.camera_frame.text().strip() or "camera_link"
    target_frame = self.target_frame.text().strip() or "tag_frame"
    pose = self.ros_if.lookup_pose_mm(camera_frame, target_frame, timeout_sec=0.08)
    if pose is None:
      message = f"Cannot resolve TF '{camera_frame}' -> '{target_frame}'."
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    tx, ty, tz, qx, qy, qz, qw = pose
    distance_mm = math.sqrt((tx * tx) + (ty * ty) + (tz * tz))
    min_distance_mm = float(self.min_tag_distance_mm.value())
    max_distance_mm = float(self.max_tag_distance_mm.value())
    if distance_mm < min_distance_mm or distance_mm > max_distance_mm:
      message = (
        f"Tag distance {distance_mm:.1f} mm outside range "
        f"[{min_distance_mm:.1f}, {max_distance_mm:.1f}] mm."
      )
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    tag_z_axis = vec_normalize(quaternion_rotate_vector((qx, qy, qz, qw), (0.0, 0.0, 1.0)))
    if tag_z_axis is None:
      message = "Tag orientation is invalid."
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    tilt_deg = angle_between_vectors_deg(tag_z_axis, (0.0, 0.0, -1.0))
    max_tilt = float(self.max_tilt_deg.value())
    if tilt_deg > max_tilt:
      message = f"Tag tilt {tilt_deg:.1f} deg exceeds max {max_tilt:.1f} deg."
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    return True, f"distance={distance_mm:.1f} mm, tilt={tilt_deg:.1f} deg"

  def _dispatch_next_auto_move(self):
    if self.auto_goal_index >= len(self.generated_goals):
      self.log("All goal poses captured. Computing calibration...")
      self.auto_state = "waiting_compute"
      dispatched = self._call_trigger_async("compute_calibration", self._on_compute_done)
      if not dispatched:
        self._finish_automation(False, "Failed to call compute_calibration.")
      return

    goal = self.generated_goals[self.auto_goal_index]
    ik_joints = self.ros_if.get_cached_ik_joint_solution(goal)
    if ik_joints is None:
      self._finish_automation(
        False,
        f"Missing IK joint solution for goal {self.auto_goal_index + 1}; aborting.",
      )
      return
    future = self.ros_if.send_movj_goal(joint_values=ik_joints)
    if future is None:
      self._finish_automation(False, "Failed to dispatch motion command (MovJ unavailable).")
      return

    goal_no = self.auto_goal_index + 1
    self.log(
      f"[auto] Move {goal_no}/{len(self.generated_goals)} -> "
      f"x={goal.x:.1f} y={goal.y:.1f} z={goal.z:.1f} "
      f"rx={goal.rx:.1f} ry={goal.ry:.1f} rz={goal.rz:.1f} mode=joint"
    )
    future.add_done_callback(lambda f, step=goal_no: self._on_move_command_done(f, step))
    self.auto_target_pose = goal
    self.auto_move_deadline = time.monotonic() + 40.0
    self.auto_state = "waiting_motion"

  def _run_auto_step(self):
    if not self.auto_running:
      return

    if self.auto_state == "dispatch_move":
      self._dispatch_next_auto_move()
      return

    if self.auto_state == "waiting_motion":
      if time.monotonic() > self.auto_move_deadline:
        self._finish_automation(
          False,
          f"Move timeout at goal {self.auto_goal_index + 1}.",
        )
        return

      current_tcp = self.ros_if.get_latest_tcp()
      if current_tcp is None or self.auto_target_pose is None:
        return

      if self._goal_reached(current_tcp, self.auto_target_pose):
        self.auto_settle_deadline = time.monotonic() + 2.0
        self.auto_state = "settling"
      return

    if self.auto_state == "settling":
      if time.monotonic() < self.auto_settle_deadline:
        return
      if self._current_calibration_mode() == CALIB_MODE_EYE_TO_HAND:
        visible, visibility_detail = self._check_eye_to_hand_visibility_gate(log_rejections=True)
        if not visible:
          goal_no = self.auto_goal_index + 1
          self.log(
            f"[auto] Skip goal {goal_no}/{len(self.generated_goals)}: "
            "tag visibility gate failed."
          )
          self.auto_goal_index += 1
          self.auto_capture_retry = 0
          self.auto_state = "dispatch_move"
          return
        self.log(f"[visibility] OK: {visibility_detail}")
      self.auto_capture_retry += 1
      self.auto_state = "waiting_capture"
      dispatched = self._call_trigger_async("add_sample", self._on_add_sample_done)
      if not dispatched:
        self._finish_automation(False, "Failed to call add_sample.")
      return

  def _on_reset_samples_done(self, success, message):
    if not self.auto_running:
      return
    prefix = "OK" if success else "ERROR"
    self.log(f"[reset_samples] {prefix}: {message}")
    if not success:
      self._finish_automation(False, "Cannot continue without resetting samples.")
      return
    self.auto_state = "dispatch_move"

  def _on_move_command_done(self, future, step_index):
    if not self.auto_running:
      return
    try:
      res = future.result()
      if int(getattr(res, "res", 0)) != 0:
        self._finish_automation(
          False,
          f"Move command rejected at step {step_index}: res={res.res}, reply={res.robot_return}",
        )
    except Exception as exc:
      self._finish_automation(False, f"Move command failed at step {step_index}: {exc}")

  def _on_add_sample_done(self, success, message):
    if not self.auto_running:
      return
    if success:
      self.log(
        f"[add_sample] OK ({self.auto_goal_index + 1}/{len(self.generated_goals)}): {message}"
      )
      self.auto_goal_index += 1
      self.auto_capture_retry = 0
      self.auto_state = "dispatch_move"
      return

    self.log(f"[add_sample] ERROR: {message}")
    if self.auto_capture_retry < 3:
      self.log(f"Retrying sample capture at goal {self.auto_goal_index + 1}...")
      self.auto_settle_deadline = time.monotonic() + 2.0
      self.auto_state = "settling"
      return
    self._finish_automation(
      False,
      f"Sample capture failed repeatedly at goal {self.auto_goal_index + 1}.",
    )

  def _on_compute_done(self, success, message):
    if not self.auto_running:
      return
    prefix = "OK" if success else "ERROR"
    self.log(f"[compute_calibration] {prefix}: {message}")
    message_lc = str(message or "").lower()
    tf_broadcasted = ("broadcasted static tf" in message_lc) and ("calibrated_camera_link" in message_lc)
    if success:
      if tf_broadcasted:
        self._set_save_yaml_ready(True)
      else:
        self._set_save_yaml_ready(False)
        self.log("[ui] Save YAML not highlighted: compute succeeded but TF broadcast was not confirmed.")
      self._finish_automation(True, "Auto calibration run completed.")
    else:
      self._set_save_yaml_ready(False)
      self._finish_automation(False, "Auto run ended with compute failure.")

  def _finish_automation(self, success, final_message):
    if final_message:
      self.log(final_message)
    self.auto_running = False
    self.auto_state = "idle"
    self.auto_target_pose = None
    self.auto_move_deadline = 0.0
    self.auto_settle_deadline = 0.0
    self.auto_capture_retry = 0
    self._set_controls_during_auto(False)
    self.stop_btn.setEnabled(self.calib_process is not None)

  def generate_goal_poses(self):
    self._persist_all_ui_settings(log_changes=False)
    self._set_save_yaml_ready(False)
    self.generated_goals = []
    self.start_btn.setEnabled(False)
    goals = self._build_and_publish_goal_poses(log_preview=True)
    if goals:
      self.generated_goals = goals
      self.start_btn.setEnabled(not self.auto_running)

  def _build_and_publish_goal_poses(self, log_preview):
    seed_pose = self.ros_if.get_latest_tcp()
    if seed_pose is None:
      self.log("No live TCP pose yet on dobot_msgs_v4/msg/ToolVectorActual. Cannot generate goals.")
      return None
    self.ros_if.apply_calibration_mode_tool(self._current_calibration_mode())
    self.ros_if.clear_ik_joint_cache()

    sample_count = int(self.min_samples.value())
    if sample_count <= 0:
      self.log("Minimum samples must be > 0.")
      return None

    min_distance_mm = float(self.min_tag_distance_mm.value())
    max_distance_mm = float(self.max_tag_distance_mm.value())
    max_tilt_deg = float(self.max_tilt_deg.value())
    look_up_bias_deg = float(self.look_up_bias_deg.value())
    min_base_z_mm = float(self.min_base_z_mm.value())
    if min_distance_mm >= max_distance_mm:
      self.log("Invalid distance limits: min distance must be smaller than max distance.")
      return None

    mode = self._current_calibration_mode()
    eye_to_hand = mode == CALIB_MODE_EYE_TO_HAND
    if eye_to_hand:
      values = self._parse_tool_offset_values(self.tag_tool_offset.text())
      if values is None:
        self.log("Eye-to-hand requires valid tag tool offset: x,y,z,rx,ry,rz")
        return None
      success, detail = self.ros_if.apply_tag_tool_offset(values)
      if not success:
        self.log(f"[tag_tool] ERROR: {detail}")
        return None
      self.log(f"[tag_tool] OK: {detail}")
    parent_frame = self.base_frame.text().strip() or "base_link"
    if parent_frame != "base_link":
      self.log(
        f"Note: Min base Z height is checked in '{parent_frame}'. "
        "Set Base frame to 'base_link' for strict base_link constraint."
      )
    target_frame = self.target_frame.text().strip() or "tag_frame"
    tag_pose = None
    gripper_to_camera_pose = identity_transform_pose_mm()
    if not eye_to_hand:
      tag_pose = self.ros_if.lookup_pose_mm(parent_frame, target_frame)
      if tag_pose is None:
        self.log(
          f"Cannot resolve TF '{parent_frame}' -> '{target_frame}'. "
          "Need a valid tag transform before generating look-at poses."
        )
        return None
      gripper_to_camera_pose = self._lookup_gripper_to_camera_pose()
      if gripper_to_camera_pose is None:
        return None
    else:
      visible, visibility_message = self._check_eye_to_hand_visibility_gate(log_rejections=False)
      if not visible:
        self.log(
          "Eye-to-hand needs a visible tag before pose generation. "
          f"Current visibility check failed: {visibility_message}"
        )
        return None

    ik_ready = self.ros_if.ensure_ik_ready(timeout_sec=0.25)
    if not ik_ready:
      self.log("IK service is not ready. Joint-mode MovJ requires IK for every goal.")
      return None
    ik_rejected = 0
    ik_checked = 0
    ik_rounds_used = 0
    skipped_by_distance = 0
    skipped_by_tilt = 0
    skipped_by_height = 0

    goals = []
    if eye_to_hand:
      self.log(
        f"IK pre-check enabled (eye-to-hand): generating {sample_count} local sweep poses, "
        "then refilling only failed/missing poses."
      )
    else:
      self.log(
        f"IK pre-check enabled: generating {sample_count} poses first, "
        "then refilling only failed/missing poses."
      )
    seen_goal_keys = set()
    max_refill_rounds = 8

    for round_idx in range(max_refill_rounds):
      missing = sample_count - len(goals)
      if missing <= 0:
        break

      ik_rounds_used = round_idx + 1
      if eye_to_hand:
        batch, skip_h = build_eye_to_hand_goal_pose_sequence(
          seed_pose,
          missing,
          min_base_z_mm,
          sequence_offset=round_idx * 409,
        )
        skipped_by_height += skip_h
      else:
        batch, skip_d, skip_t, skip_h = build_goal_pose_sequence(
          seed_pose,
          tag_pose,
          gripper_to_camera_pose,
          missing,
          min_distance_mm,
          max_distance_mm,
          max_tilt_deg,
          look_up_bias_deg,
          min_base_z_mm,
          sequence_offset=round_idx * 409,
        )
        skipped_by_distance += skip_d
        skipped_by_tilt += skip_t
        skipped_by_height += skip_h

      if not batch:
        if round_idx == 0:
          if eye_to_hand:
            self.log("Failed to generate eye-to-hand sweep poses from current TCP.")
          else:
            self.log("Failed to generate poses: invalid seed/tag geometry.")
          return None
        self.log(f"No additional candidates available in refill round {ik_rounds_used}.")
        break

      self.log(
        f"IK batch {ik_rounds_used}: generated {len(batch)} candidate pose(s) "
        f"for {missing} missing pose(s)."
      )
      accepted_this_round = 0
      for index, pose in enumerate(batch, start=1):
        key = goal_pose_key(pose)
        if key in seen_goal_keys:
          continue
        seen_goal_keys.add(key)

        ik_checked += 1
        ok, detail = self.ros_if.check_pose_reachable_ik(pose, timeout_sec=0.4)
        if ok is True:
          goals.append(pose)
          accepted_this_round += 1
          if len(goals) >= sample_count:
            break
          continue

        ik_rejected += 1
        self.log(f"  [ik] reject r{ik_rounds_used} pose {index}: {detail}")

      if len(goals) >= sample_count:
        break
      if accepted_this_round == 0:
        self.log(f"Round {ik_rounds_used} added no IK-valid new poses.")

    if not goals:
      self.log("All generated goals were rejected by IK pre-check.")
      return None

    published = self.ros_if.publish_goal_pose_transforms(parent_frame, goals)
    if not published:
      self.log("Failed to publish goal pose TFs.")
      return None

    if log_preview:
      if eye_to_hand:
        self.log(
          f"Generated {len(goals)} eye-to-hand goal pose frame(s) under parent '{parent_frame}'. "
          "Visibility is checked at capture time before each sample."
        )
      else:
        self.log(
          f"Generated {len(goals)} camera-centered goal pose frame(s) under parent '{parent_frame}'. "
          f"The camera frame is aimed at '{target_frame}' for each pose."
        )
      if len(goals) < sample_count:
        if eye_to_hand:
          self.log(
            f"Requested {sample_count}, but only {len(goals)} satisfied safety limits "
            f"(local sweep + min z {min_base_z_mm:.0f} mm)."
          )
        else:
          self.log(
            f"Requested {sample_count}, but only {len(goals)} satisfied safety limits "
            f"(min/max distance {min_distance_mm:.0f}/{max_distance_mm:.0f} mm, "
            f"max tilt {max_tilt_deg:.1f} deg, look up {look_up_bias_deg:.1f} deg, "
            f"min z {min_base_z_mm:.0f} mm)."
          )
      if ik_rejected:
        self.log(f"IK pre-check rejected {ik_rejected} pose(s) out of {ik_checked}.")
      if ik_rounds_used:
        self.log(f"IK refill rounds used: {ik_rounds_used}.")
      if eye_to_hand:
        if skipped_by_height:
          self.log(f"Filtered out candidates: low_z={skipped_by_height}.")
      elif skipped_by_distance or skipped_by_tilt or skipped_by_height:
        self.log(
          f"Filtered out candidates: distance={skipped_by_distance}, "
          f"tilt={skipped_by_tilt}, low_z={skipped_by_height}."
        )
      preview_count = min(5, len(goals))
      for index in range(preview_count):
        p = goals[index]
        self.log(
          f"  goal {index + 1}: "
          f"x={p.x:.1f} y={p.y:.1f} z={p.z:.1f} "
          f"rx={p.rx:.1f} ry={p.ry:.1f} rz={p.rz:.1f} "
          f"dist={p.distance_to_tag_mm:.1f} tilt={p.tilt_from_seed_deg:.1f}"
        )
      if len(goals) > preview_count:
        self.log(f"  ... {len(goals) - preview_count} more goal poses.")
    return goals

  def _call_trigger_async(self, service_name, done_callback):
    client = self.ros_if.get_client(service_name)
    if client is None:
      self.log(f"Service /{service_name} unavailable.")
      return False

    future = client.call_async(Trigger.Request())

    def wrapped_callback(result_future):
      success = False
      message = ""
      try:
        result = result_future.result()
        success = bool(result.success)
        message = result.message
      except Exception as exc:
        success = False
        message = f"call failed: {exc}"
      done_callback(success, message)

    future.add_done_callback(wrapped_callback)
    return True

  def _call_trigger(self, service_name):
    def done_callback(future):
      try:
        res = future.result()
        prefix = "OK" if res.success else "ERROR"
        self.log(f"[{service_name}] {prefix}: {res.message}")
      except Exception as exc:
        self.log(f"[{service_name}] call failed: {exc}")

    client = self.ros_if.get_client(service_name)
    if client is None:
      self.log(f"Service /{service_name} unavailable. Ensure calibrator is running.")
      return

    future = client.call_async(Trigger.Request())
    future.add_done_callback(done_callback)

  def apply_tag_tool_offset(self):
    self._persist_all_ui_settings(log_changes=False)
    values = self._parse_tool_offset_values(self.tag_tool_offset.text())
    if values is None:
      self.log("Invalid tag tool offset. Expected six comma-separated values: x,y,z,rx,ry,rz")
      return
    success, detail = self.ros_if.apply_tag_tool_offset(values)
    prefix = "OK" if success else "ERROR"
    self.log(f"[tag_tool] {prefix}: {detail}")

  def save_yaml(self):
    self._persist_all_ui_settings(log_changes=False)
    self._call_trigger("save_calibration")

  def closeEvent(self, event):
    self._persist_all_ui_settings(log_changes=False)
    super().closeEvent(event)


class RosInterface(Node):
  def __init__(self):
    super().__init__("camera_calibration_gui")
    self._client_map = {}
    self._latest_tcp = None
    self._latest_overlay_qimage = None
    self._latest_overlay_received_monotonic = None
    self._unsupported_overlay_encodings = set()
    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
    self._goal_tf_broadcaster = StaticTransformBroadcaster(self)
    self._motion_service_root = str(
      self.declare_parameter("motion_service_root", "/dobot_bringup_ros2/srv").value
    ).rstrip("/")
    self._default_calibration_mode = normalize_calibration_mode(
      self.declare_parameter("calibration_mode", CALIB_MODE_EYE_ON_HAND).value
    )
    self._auto_move_speed_l = int(self.declare_parameter("auto_move_speed_l", 35).value)
    self._auto_move_acc_l = int(self.declare_parameter("auto_move_acc_l", 35).value)
    self._ik_user = str(self.declare_parameter("ik_user", "0").value)
    self._ik_tool = str(self.declare_parameter("ik_tool", "0").value)
    self._active_ik_tool = str(self._ik_tool)
    self._ik_use_joint_near = str(self.declare_parameter("ik_use_joint_near", "0").value)
    self._ik_joint_near = str(
      self.declare_parameter("ik_joint_near", "{0,0,0,0,0,0}").value
    )
    self._ik_joint_solution_cache = {}
    self._movj_client = self.create_client(MovJ, f"{self._motion_service_root}/MovJ")
    self._stop_motion_client = self.create_client(Stop, f"{self._motion_service_root}/Stop")
    self._ik_client = self.create_client(InverseKin, f"{self._motion_service_root}/InverseKin")
    self._tool_client = self.create_client(Tool, f"{self._motion_service_root}/Tool")
    self._set_tool_client = self.create_client(SetTool, f"{self._motion_service_root}/SetTool")
    self.apply_calibration_mode_tool(self._default_calibration_mode)

    self.create_subscription(
      ToolVectorActual,
      "dobot_msgs_v4/msg/ToolVectorActual",
      self._tcp_callback,
      10,
    )
    self.create_subscription(
      Image,
      "/aruco_overlay",
      self._overlay_callback,
      5,
    )

  def get_client(self, service_name):
    if service_name in self._client_map:
      return self._client_map[service_name]
    client = self.create_client(Trigger, service_name, qos_profile=qos_profile_services_default)
    if not client.wait_for_service(timeout_sec=1.5):
      self.get_logger().warn(f"Service {service_name} not available")
      return None
    self._client_map[service_name] = client
    return client

  def get_default_calibration_mode(self):
    return self._default_calibration_mode

  def ensure_motion_ready(self, timeout_sec=0.5):
    return self._movj_client.wait_for_service(timeout_sec=timeout_sec)

  def ensure_ik_ready(self, timeout_sec=0.5):
    return self._ik_client.wait_for_service(timeout_sec=timeout_sec)

  def clear_ik_joint_cache(self):
    self._ik_joint_solution_cache.clear()

  def set_active_ik_tool(self, tool_value):
    self._active_ik_tool = str(tool_value)

  def apply_calibration_mode_tool(self, calibration_mode):
    mode = normalize_calibration_mode(calibration_mode)
    if mode == CALIB_MODE_EYE_TO_HAND:
      self._active_ik_tool = "1"
      return
    self._active_ik_tool = str(self._ik_tool)

  @staticmethod
  def _is_nonzero_tool_value(tool_value):
    text = str(tool_value or "").strip().lower()
    return text not in ("", "0", "false", "off", "no")

  def apply_tag_tool_offset(self, values):
    if values is None or len(values) != 6:
      return False, "Expected six tool offset values."
    if not self._set_tool_client.wait_for_service(timeout_sec=0.5):
      return False, f"Service unavailable: {self._motion_service_root}/SetTool"
    if not self._tool_client.wait_for_service(timeout_sec=0.5):
      return False, f"Service unavailable: {self._motion_service_root}/Tool"

    set_req = SetTool.Request()
    set_req.index = 1
    set_req.value = "{" + ",".join(f"{float(v):.3f}" for v in values) + "}"
    set_future = self._set_tool_client.call_async(set_req)
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and (not set_future.done()) and time.monotonic() < deadline:
      rclpy.spin_once(self, timeout_sec=0.01)
    if not set_future.done():
      return False, "SetTool timeout."
    try:
      set_res = set_future.result()
    except Exception as exc:
      return False, f"SetTool call failed: {exc}"
    if int(getattr(set_res, "res", -1)) != 0:
      return False, f"SetTool rejected (res={set_res.res})."

    tool_req = Tool.Request()
    tool_req.index = 1
    tool_future = self._tool_client.call_async(tool_req)
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and (not tool_future.done()) and time.monotonic() < deadline:
      rclpy.spin_once(self, timeout_sec=0.01)
    if not tool_future.done():
      return False, "Tool activation timeout."
    try:
      tool_res = tool_future.result()
    except Exception as exc:
      return False, f"Tool activation failed: {exc}"
    if int(getattr(tool_res, "res", -1)) != 0:
      return False, f"Tool activation rejected (res={tool_res.res})."

    self._active_ik_tool = "1"
    return True, "Tool 1 offset applied and activated."

  def get_cached_ik_joint_solution(self, goal_pose):
    return self._ik_joint_solution_cache.get(goal_pose_key(goal_pose))

  def check_pose_reachable_ik(self, goal_pose, timeout_sec=0.5):
    if not self.ensure_ik_ready(timeout_sec=0.2):
      return None, "IK service unavailable"

    req = InverseKin.Request()
    req.x = float(goal_pose.x)
    req.y = float(goal_pose.y)
    req.z = float(goal_pose.z)
    req.rx = float(goal_pose.rx)
    req.ry = float(goal_pose.ry)
    req.rz = float(goal_pose.rz)
    req.user = self._ik_user
    req.tool = self._active_ik_tool
    req.use_joint_near = self._ik_use_joint_near
    req.joint_near = self._ik_joint_near

    future = self._ik_client.call_async(req)
    deadline = time.monotonic() + max(0.05, float(timeout_sec))
    while rclpy.ok() and (not future.done()) and time.monotonic() < deadline:
      rclpy.spin_once(self, timeout_sec=0.01)

    if not future.done():
      return False, "IK timeout"

    try:
      response = future.result()
    except Exception as exc:
      return False, f"IK call failed: {exc}"

    if int(getattr(response, "res", -1)) == 0:
      joint_values = parse_joint_values_from_robot_return(getattr(response, "robot_return", ""))
      if joint_values is None:
        return False, "IK returned no parseable joint solution"
      self._ik_joint_solution_cache[goal_pose_key(goal_pose)] = joint_values
      return True, "IK OK"
    return False, f"res={response.res}, reply={response.robot_return}"

  def send_movj_goal(self, joint_values):
    if not self.ensure_motion_ready(timeout_sec=0.5):
      self.get_logger().warn(
        f"Motion service not available: {self._motion_service_root}/MovJ"
      )
      return None
    if joint_values is None or len(joint_values) < 6:
      self.get_logger().warn("Joint-mode MovJ requires 6 joint values.")
      return None

    speed_l = max(1, min(100, int(self._auto_move_speed_l)))
    acc_l = max(1, min(100, int(self._auto_move_acc_l)))
    req = MovJ.Request()
    req.mode = True
    req.a = float(joint_values[0])
    req.b = float(joint_values[1])
    req.c = float(joint_values[2])
    req.d = float(joint_values[3])
    req.e = float(joint_values[4])
    req.f = float(joint_values[5])
    motion_args = [f"v={speed_l},a={acc_l}"]
    if self._is_nonzero_tool_value(self._active_ik_tool):
      motion_args.append("tool=1")
    req.param_value = motion_args
    return self._movj_client.call_async(req)

  def stop_motion(self):
    if not self._stop_motion_client.wait_for_service(timeout_sec=0.2):
      return False
    self._stop_motion_client.call_async(Stop.Request())
    return True

  def _tcp_callback(self, msg):
    self._latest_tcp = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]

  def get_latest_tcp(self):
    if self._latest_tcp is None:
      return None
    return list(self._latest_tcp)

  def _overlay_callback(self, msg):
    image = self._image_msg_to_qimage(msg)
    if image is None:
      return
    self._latest_overlay_qimage = image
    self._latest_overlay_received_monotonic = time.monotonic()

  def get_latest_overlay_qimage(self):
    if self._latest_overlay_received_monotonic is None:
      return None
    if time.monotonic() - self._latest_overlay_received_monotonic > 3.0:
      return None
    return self._latest_overlay_qimage

  def _image_msg_to_qimage(self, msg):
    if msg.width <= 0 or msg.height <= 0:
      return None

    expected_size = int(msg.step) * int(msg.height)
    if len(msg.data) < expected_size:
      self.get_logger().warn("Received invalid /aruco_overlay image: data buffer too small.")
      return None

    encoding = (msg.encoding or "").lower()
    data = bytes(msg.data)
    if encoding in ("rgb8", "8uc3"):
      image = QtGui.QImage(data, msg.width, msg.height, msg.step, QtGui.QImage.Format_RGB888)
      return image.copy()
    if encoding == "bgr8":
      image = QtGui.QImage(data, msg.width, msg.height, msg.step, QtGui.QImage.Format_RGB888)
      return image.rgbSwapped().copy()
    if encoding in ("mono8", "8uc1"):
      image = QtGui.QImage(data, msg.width, msg.height, msg.step, QtGui.QImage.Format_Grayscale8)
      return image.copy()

    if encoding not in self._unsupported_overlay_encodings:
      self._unsupported_overlay_encodings.add(encoding)
      self.get_logger().warn(
        f"Unsupported /aruco_overlay encoding '{msg.encoding}'. Expected bgr8/rgb8/mono8."
      )
    return None

  def lookup_translation_mm(self, parent_frame, child_frame, timeout_sec=0.15):
    timeout_ns = int(max(0.01, float(timeout_sec)) * 1e9)
    try:
      tf_msg = self._tf_buffer.lookup_transform(
        parent_frame,
        child_frame,
        Time(),
        timeout=Duration(nanoseconds=timeout_ns),
      )
      return (
        float(tf_msg.transform.translation.x) * 1000.0,
        float(tf_msg.transform.translation.y) * 1000.0,
        float(tf_msg.transform.translation.z) * 1000.0,
      )
    except TransformException as exc:
      self.get_logger().warn(
        f"TF lookup failed ({parent_frame} -> {child_frame}): {exc}"
      )
      return None

  def lookup_pose_mm(self, parent_frame, child_frame, timeout_sec=0.15):
    timeout_ns = int(max(0.01, float(timeout_sec)) * 1e9)
    try:
      tf_msg = self._tf_buffer.lookup_transform(
        parent_frame,
        child_frame,
        Time(),
        timeout=Duration(nanoseconds=timeout_ns),
      )
      return (
        float(tf_msg.transform.translation.x) * 1000.0,
        float(tf_msg.transform.translation.y) * 1000.0,
        float(tf_msg.transform.translation.z) * 1000.0,
        float(tf_msg.transform.rotation.x),
        float(tf_msg.transform.rotation.y),
        float(tf_msg.transform.rotation.z),
        float(tf_msg.transform.rotation.w),
      )
    except TransformException as exc:
      self.get_logger().warn(
        f"TF lookup failed ({parent_frame} -> {child_frame}): {exc}"
      )
      return None

  def publish_goal_pose_transforms(self, parent_frame, goals):
    if not parent_frame:
      self.get_logger().warn("Empty parent_frame for goal poses.")
      return False
    if not goals:
      self.get_logger().warn("No goal poses to publish.")
      return False

    now_msg = self.get_clock().now().to_msg()
    transforms = []
    for index, pose in enumerate(goals, start=1):
      tf_msg = TransformStamped()
      tf_msg.header.stamp = now_msg
      tf_msg.header.frame_id = parent_frame
      tf_msg.child_frame_id = f"calib_goal_{index:02d}"
      tf_msg.transform.translation.x = float(pose.x) / 1000.0
      tf_msg.transform.translation.y = float(pose.y) / 1000.0
      tf_msg.transform.translation.z = float(pose.z) / 1000.0
      qx, qy, qz, qw = rpy_deg_to_quaternion(pose.rx, pose.ry, pose.rz)
      tf_msg.transform.rotation.x = qx
      tf_msg.transform.rotation.y = qy
      tf_msg.transform.rotation.z = qz
      tf_msg.transform.rotation.w = qw
      transforms.append(tf_msg)

    self._goal_tf_broadcaster.sendTransform(transforms)
    return True


def main(args=None):
  rclpy.init(args=args)
  ros_if = RosInterface()

  app = QtWidgets.QApplication(sys.argv)
  widget = CalibGui(ros_if)
  widget.resize(1240, 700)
  widget.show()

  # Periodically spin rclpy to service clients
  timer = QtCore.QTimer()
  def spin_once():
    if not rclpy.ok():
      QtWidgets.QApplication.instance().quit()
      return
    rclpy.spin_once(ros_if, timeout_sec=0.01)

  timer.timeout.connect(spin_once)
  timer.start(10)

  ret = app.exec_()

  if widget.calib_process is not None:
    widget.stop_calibrator()

  rclpy.shutdown()
  sys.exit(ret)


if __name__ == "__main__":
  main()
