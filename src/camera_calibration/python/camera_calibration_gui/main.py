import subprocess
import sys
import os
import signal
from datetime import datetime
import math
from pathlib import Path
import re
import time

from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_services_default
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


class OverlayImageLabel(QtWidgets.QLabel):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._status_overlay_text = ""
    self._gate_overlay_text = ""

  def set_status_overlay_text(self, text):
    normalized = str(text or "").strip()
    if normalized == self._status_overlay_text:
      return
    self._status_overlay_text = normalized
    self.update()

  def set_gate_overlay_text(self, text):
    normalized = str(text or "").strip()
    if normalized == self._gate_overlay_text:
      return
    self._gate_overlay_text = normalized
    self.update()

  def paintEvent(self, event):
    super().paintEvent(event)
    if not self._status_overlay_text and not self._gate_overlay_text:
      return

    painter = QtGui.QPainter(self)
    painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

    font = QtGui.QFont("Monospace")
    font.setStyleHint(QtGui.QFont.TypeWriter)
    font.setPointSize(10)
    painter.setFont(font)

    if self._status_overlay_text:
      self._draw_overlay_box(painter, self._status_overlay_text, align_right=False)
    if self._gate_overlay_text:
      self._draw_overlay_box(painter, self._gate_overlay_text, align_right=True)

  def _draw_overlay_box(self, painter, text, align_right):
    margin = 12
    pad_x = 10
    pad_y = 8
    flags = QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap
    available_width = max(120, self.width() - (margin * 2))
    max_width = min(available_width, max(260, int(self.width() * 0.44)))
    max_height = max(80, self.height() - (margin * 2))
    measure_rect = QtCore.QRect(0, 0, max_width - (pad_x * 2), max_height - (pad_y * 2))
    bounds = painter.boundingRect(measure_rect, flags, text)
    box_width = min(max_width, bounds.width() + (pad_x * 2))
    box_height = min(max_height, bounds.height() + (pad_y * 2))
    x = self.width() - margin - box_width if align_right else margin
    box = QtCore.QRect(x, margin, box_width, box_height)

    painter.setPen(QtCore.Qt.NoPen)
    painter.setBrush(QtGui.QColor(0, 0, 0, 180))
    painter.drawRoundedRect(box, 6, 6)
    painter.setPen(QtGui.QColor(245, 245, 245))
    painter.drawText(box.adjusted(pad_x, pad_y, -pad_x, -pad_y), flags, text)


CALIB_MODE_EYE_ON_HAND = "eye_on_hand"
CALIB_MODE_EYE_TO_HAND = "eye_to_hand"
TAG_FRAME = "tag_frame"
TAG_FRAME_MAX_AGE_SEC = 0.5
TAG_STABILITY_WINDOW_SEC = 1.0
TAG_STABILITY_TIME_EPS_SEC = 0.08
TAG_STABILITY_TRANSLATION_TOL_MM = 1.0
TAG_STABILITY_ROTATION_TOL_DEG = 1.0
DEFAULT_ARUCO_IDS = [1, 2, 3, 4]
ARUCO_ID_MIN = 0
ARUCO_ID_MAX = 49  # DICT_5X5_50


def workspace_root() -> Path:
  def looks_like_root(path: Path) -> bool:
    return (
      (path / "src").exists() and
      ((path / "README.md").exists() or
       (path / "src" / "dobot_msgs_v4").exists())
    )

  def find_from(start: Path) -> Path | None:
    path = start.expanduser().resolve()
    if path.is_file():
      path = path.parent
    for candidate in (path, *path.parents):
      if looks_like_root(candidate):
        return candidate
    return None

  for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
    value = os.environ.get(name)
    if value:
      return find_from(Path(value)) or Path(value).expanduser().resolve()

  candidates = [Path.cwd(), Path(__file__).resolve()]
  for name in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
    for token in os.environ.get(name, "").split(os.pathsep):
      if not token:
        continue
      prefix = Path(token)
      candidates.append(prefix)
      if "install" in prefix.parts:
        candidates.append(Path(*prefix.parts[:prefix.parts.index("install")]))

  for candidate in candidates:
    found = find_from(candidate)
    if found is not None:
      return found
  return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
  return workspace_root().joinpath(*parts)


def _unquote_config_value(value):
  text = str(value or "").strip()
  if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
    text = text[1:-1]
  return text.strip()


def read_station_config_value(*keys):
  station_config = workspace_path("station_config")
  try:
    with station_config.open("r", encoding="utf-8") as stream:
      values = {}
      for raw_line in stream:
        line = raw_line.strip()
        if not line or line.startswith("#"):
          continue
        if line.startswith("export "):
          line = line[len("export "):].strip()
        if "=" not in line:
          continue
        key, value = line.split("=", 1)
        values[key.strip()] = _unquote_config_value(value)
  except OSError:
    return ""

  for key in keys:
    value = values.get(key)
    if value:
      return value
  return ""


def resolve_robot_ip_address(value=None):
  requested = str(value or "").strip()
  if requested:
    return requested
  env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
  if env_ip:
    return env_ip
  return read_station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def sanitize_filename_token(value):
  token = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip())
  token = re.sub(r"_+", "_", token)
  return token.strip("_")


def normalize_calibration_mode(value):
  mode = str(value or "").strip().lower().replace("-", "_")
  if mode in (CALIB_MODE_EYE_TO_HAND, "eyetohand"):
    return CALIB_MODE_EYE_TO_HAND
  return CALIB_MODE_EYE_ON_HAND


def default_camera_prefix_for_mode(mode):
  return "bin_camera" if normalize_calibration_mode(mode) == CALIB_MODE_EYE_TO_HAND else "robot_camera"


def default_camera_frame_for_mode(mode, camera_prefix=None):
  prefix = normalize_camera_prefix(camera_prefix, mode)
  return f"{prefix}_color_optical_frame"


def default_calibrated_camera_frame_for_mode(mode):
  if normalize_calibration_mode(mode) == CALIB_MODE_EYE_TO_HAND:
    return "bin_calibrated_camera_link"
  return "arm_calibrated_camera_link"


def normalize_camera_prefix(value, mode=CALIB_MODE_EYE_ON_HAND):
  prefix = str(value or "").strip().strip("/")
  if prefix.lower() == "auto":
    prefix = ""
  return prefix or default_camera_prefix_for_mode(mode)


def normalize_camera_frame(value, mode=CALIB_MODE_EYE_ON_HAND, camera_prefix=None):
  frame = str(value or "").strip().strip("/")
  if frame.lower() == "auto":
    frame = ""
  return frame or default_camera_frame_for_mode(mode, camera_prefix)


def camera_topics_for_prefix(prefix):
  normalized = normalize_camera_prefix(prefix)
  return (
    f"/{normalized}/color/image_raw",
    f"/{normalized}/depth/image_raw",
    f"/{normalized}/color/camera_info",
  )


def format_aruco_ids(ids):
  return ",".join(str(int(marker_id)) for marker_id in ids)


def validate_aruco_ids(ids):
  if len(ids) != 4:
    return False, "ArUco IDs must contain exactly 4 IDs in TL,TR,BL,BR order."
  for marker_id in ids:
    if marker_id < ARUCO_ID_MIN or marker_id > ARUCO_ID_MAX:
      return False, "ArUco IDs must be in range 0..49 for DICT_5X5_50."
  if len(set(ids)) != len(ids):
    return False, "ArUco IDs must be unique for depth board pose fitting."
  return True, ""


def parse_aruco_ids_text(text):
  tokens = [token for token in re.split(r"[\s,;]+", str(text or "").strip()) if token]
  try:
    ids = [int(token) for token in tokens]
  except ValueError:
    return None, "ArUco IDs must be integers, for example 1,2,3,4."
  valid, reason = validate_aruco_ids(ids)
  if not valid:
    return None, reason
  return ids, ""


def coerce_aruco_ids(value):
  if isinstance(value, str):
    return parse_aruco_ids_text(value)
  try:
    ids = [int(marker_id) for marker_id in value]
  except (TypeError, ValueError):
    return None, "ArUco IDs must be an integer array."
  valid, reason = validate_aruco_ids(ids)
  if not valid:
    return None, reason
  return ids, ""


def calibration_mode_filename_token(mode):
  return "eyetohand" if normalize_calibration_mode(mode) == CALIB_MODE_EYE_TO_HAND else "eyeonhand"


def calibration_filename_for_mode(mode, robot_ip_address=None):
  filename = f"axab_calibration_{calibration_mode_filename_token(mode)}_{datetime.now().strftime('%d%m%Y')}"
  ip_token = sanitize_filename_token(resolve_robot_ip_address(robot_ip_address))
  if ip_token:
    filename += f"_{ip_token}"
  return f"{filename}.yaml"


def default_output_path(mode=CALIB_MODE_EYE_ON_HAND, robot_ip_address=None):
  calib_dir = workspace_path("calibration")
  try:
    calib_dir.mkdir(parents=True, exist_ok=True)
  except Exception:
    pass
  return str(calib_dir / calibration_filename_for_mode(mode, robot_ip_address))


def quaternion_normalize(quat):
  qx, qy, qz, qw = quat
  norm = math.sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw))
  if norm < 1e-12:
    return 0.0, 0.0, 0.0, 1.0
  inv = 1.0 / norm
  return qx * inv, qy * inv, qz * inv, qw * inv


def quaternion_angular_distance_deg(lhs, rhs):
  a = quaternion_normalize(lhs)
  b = quaternion_normalize(rhs)
  dot = abs((a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]) + (a[3] * b[3]))
  dot = max(-1.0, min(1.0, dot))
  return math.degrees(2.0 * math.acos(dot))


class CalibGui(QtWidgets.QWidget):
  def __init__(self, ros_if, parent=None):
    super().__init__(parent)
    self.ros_if = ros_if
    self._ui_settings = QtCore.QSettings("DOBOT", "camera_calibration_gui")
    self.calib_process = None
    self.manual_sample_count = 0
    self.manual_capture_in_flight = False
    self.manual_compute_in_flight = False
    self.reset_samples_in_flight = False
    self.preview_compute_in_flight = False
    self._calibrator_ready = False
    self._active_calibrator_config = None
    self._calibrator_namespace_base = f"/camera_calibration_gui_{os.getpid()}"
    self._calibrator_generation = 0
    self._calibrator_namespace = self._calibrator_namespace_for_generation()
    self._save_yaml_ready = False
    self._tag_stability_history = []
    self._tag_stability_key = None
    self._latest_tag_gate_ready = False
    self._latest_tag_gate_detail = "Waiting for stable tag_frame."
    self.setWindowTitle("Camera Calibration")
    self._build_ui()
    self._restore_ui_settings()
    self.ros_if.apply_calibration_mode_tool(self._current_calibration_mode())
    self._set_camera_prefix_for_mode(
      self._current_calibration_mode(),
      log_change=True,
      force_apply=True,
    )
    self._set_camera_frame_for_mode(
      self._current_calibration_mode(),
      log_change=True,
      force_apply=True,
    )
    self._connect_ui_setting_signals()
    self._update_mode_dependent_ui()
    self._update_window_title()
    self._persist_all_ui_settings(log_changes=False)
    self._apply_aruco_ids_to_perception(log_result=True)
    self._log_ui_settings_snapshot("UI settings active at startup")
    self._setup_monitors()
    self.start_calibrator()

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
    self.camera_prefix = QtWidgets.QLineEdit(default_camera_prefix_for_mode(CALIB_MODE_EYE_ON_HAND))
    self.camera_frame = QtWidgets.QLineEdit(default_camera_frame_for_mode(CALIB_MODE_EYE_ON_HAND))
    self.aruco_ids = QtWidgets.QLineEdit(format_aruco_ids(DEFAULT_ARUCO_IDS))
    self.min_samples = QtWidgets.QSpinBox()
    self.min_samples.setMinimum(3)
    self.min_samples.setMaximum(1000)
    self.min_samples.setValue(21)
    form.addRow("Calibration mode", self.calibration_mode)
    form.addRow("Base frame", self.base_frame)
    form.addRow("Gripper frame", self.gripper_frame)
    form.addRow("Camera topic prefix", self.camera_prefix)
    form.addRow("Camera frame", self.camera_frame)
    form.addRow("ArUco IDs", self.aruco_ids)
    form.addRow("Minimum samples", self.min_samples)
    layout.addLayout(form)

    action_layout = QtWidgets.QHBoxLayout()
    self.add_btn = QtWidgets.QPushButton("Get Sample")
    self.undo_last_btn = QtWidgets.QPushButton("Undo Last")
    self.reset_samples_btn = QtWidgets.QPushButton("Reset Samples")
    self.save_btn = QtWidgets.QPushButton("Save YAML")
    self.undo_last_btn.setEnabled(False)
    self.reset_samples_btn.setEnabled(False)
    action_layout.addWidget(self.add_btn)
    action_layout.addWidget(self.undo_last_btn)
    action_layout.addWidget(self.reset_samples_btn)
    action_layout.addWidget(self.save_btn)
    layout.addLayout(action_layout)

    self.status = QtWidgets.QPlainTextEdit()
    self.status.setReadOnly(True)
    layout.addWidget(self.status)

    overlay_group = QtWidgets.QGroupBox("ArUco Overlay (/aruco_overlay)")
    overlay_layout = QtWidgets.QVBoxLayout(overlay_group)
    self.overlay_label = OverlayImageLabel("Waiting for /aruco_overlay ...")
    self.overlay_label.setAlignment(QtCore.Qt.AlignCenter)
    self.overlay_label.setMinimumSize(640, 360)
    self.overlay_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
    self.overlay_label.setStyleSheet(
      "QLabel { background-color: #101010; color: #d0d0d0; border: 1px solid #444; }"
    )
    self.overlay_label.set_status_overlay_text(
      "Live calibrated TF preview\nWaiting for 3 samples."
    )
    self.overlay_label.set_gate_overlay_text(
      "Tag gate\nBLOCKED\nWaiting for tag_frame."
    )
    overlay_layout.addWidget(self.overlay_label)

    root_layout.addWidget(controls_widget, 0)
    root_layout.addWidget(overlay_group, 1)

    self.setLayout(root_layout)

    self.add_btn.clicked.connect(self.handle_add_button_clicked)
    self.undo_last_btn.clicked.connect(self.undo_last_sample)
    self.reset_samples_btn.clicked.connect(self.reset_samples)
    self.save_btn.clicked.connect(self.save_yaml)

  def log(self, text):
    self.status.appendPlainText(text)

  @staticmethod
  def _extract_transform_block(message):
    text = str(message or "").strip()
    index = text.find("transform:")
    if index >= 0:
      return text[index:].strip()
    return text

  def _set_live_transform_text(self, text):
    self.overlay_label.set_status_overlay_text(str(text or "").strip())

  def _set_tag_gate_overlay_text(self, ready, detail):
    mode = self._current_calibration_mode()
    camera_frame = self.camera_frame.text().strip() or default_camera_frame_for_mode(
      mode, self.camera_prefix.text())
    state = "READY" if ready else "BLOCKED"
    self.overlay_label.set_gate_overlay_text(
      f"Tag frame gate\n"
      f"{state}\n"
      f"{camera_frame} -> {TAG_FRAME}\n"
      f"{detail}"
    )

  def _log_sample_count(self):
    self.log(f"Samples: {self.manual_sample_count}/{int(self.min_samples.value())}")

  def _set_sample_count(self, count):
    try:
      self.manual_sample_count = max(0, int(count))
    except (TypeError, ValueError):
      self.manual_sample_count = max(0, int(self.manual_sample_count))
    self._update_undo_button_state()

  def _update_undo_button_state(self):
    if not hasattr(self, "undo_last_btn"):
      return
    can_undo = (
      self.manual_sample_count > 0 and
      not self.manual_capture_in_flight and
      not self.manual_compute_in_flight and
      not self.reset_samples_in_flight
    )
    self.undo_last_btn.setEnabled(can_undo)
    if hasattr(self, "reset_samples_btn"):
      self.reset_samples_btn.setEnabled(can_undo)

  def _set_save_yaml_ready(self, ready):
    ready = bool(ready)
    if ready == self._save_yaml_ready:
      return
    self._save_yaml_ready = ready
    if ready:
      self.save_btn.setStyleSheet(
        "QPushButton { background-color: #2e7d32; color: #ffffff; font-weight: 600; }"
      )
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
    self.add_btn.setText("Get Sample")
    add_enabled = (
      self._latest_tag_gate_ready and
      self._calibrator_ready and
      not self.manual_capture_in_flight and
      not self.manual_compute_in_flight and
      not self.reset_samples_in_flight
    )
    self.add_btn.setEnabled(add_enabled)
    if not self._calibrator_ready:
      self.add_btn.setToolTip("Waiting for calibration services to start.")
    elif self._latest_tag_gate_ready:
      self.add_btn.setToolTip("Capture the current hand-guided robot pose as a calibration sample.")
    else:
      self.add_btn.setToolTip(f"Waiting for stable {TAG_FRAME}: {self._latest_tag_gate_detail}")

  def _on_calibration_mode_changed(self, *_):
    mode = self._current_calibration_mode()
    self._reset_tag_stability()
    self.ros_if.apply_calibration_mode_tool(mode)
    self._set_camera_prefix_for_mode(mode, log_change=True, force_apply=True)
    self._set_camera_frame_for_mode(mode, log_change=True, force_apply=True)
    self._save_setting(
      "calibration_mode",
      "Calibration mode",
      mode,
      display_value=self.calibration_mode.currentText(),
      emit_log=True,
    )
    self._update_window_title()
    self._update_mode_dependent_ui()
    self._restart_calibrator_for_current_settings("[ui] Calibration mode changed")

  def _set_camera_prefix_for_mode(self, mode, log_change, force_apply=False):
    prefix = default_camera_prefix_for_mode(mode)
    changed = self.camera_prefix.text().strip().strip("/") != prefix
    if changed:
      self.camera_prefix.setText(prefix)
    if not changed and not force_apply:
      return
    if log_change:
      if changed:
        self.log(f"[ui] Camera topic prefix default for {mode}: {prefix}")
      self._log_camera_prefix_launch_note(prefix)
      success, detail = self.ros_if.set_aruco_camera_prefix(prefix)
      prefix_text = "OK" if success else "ERROR"
      self.log(f"[camera_prefix] {prefix_text}: {detail}")

  def _set_camera_frame_for_mode(self, mode, log_change, force_apply=False):
    frame = default_camera_frame_for_mode(mode, self.camera_prefix.text())
    changed = self.camera_frame.text().strip().strip("/") != frame
    if changed:
      self.camera_frame.setText(frame)
    if not changed and not force_apply:
      return
    if log_change:
      if changed:
        self.log(f"[ui] Raw camera frame default for {mode}: {frame}")
      success, detail = self.ros_if.set_calibration_camera_frame(frame)
      prefix_text = "OK" if success else "ERROR"
      self.log(f"[camera_frame] {prefix_text}: {detail}")

  def _log_camera_prefix_launch_note(self, prefix):
    active_prefix = self.ros_if.get_default_camera_prefix()
    normalized = normalize_camera_prefix(prefix, self._current_calibration_mode())
    color_topic, depth_topic, camera_info_topic = camera_topics_for_prefix(normalized)
    self.log(
      f"[ui] Prefix {normalized} maps to {color_topic}, {depth_topic}, {camera_info_topic}."
    )
    if normalized == active_prefix:
      self.log("[ui] Prefix matches the active ArUco stream.")

  def _on_camera_prefix_changed(self):
    self._reset_tag_stability()
    prefix = normalize_camera_prefix(self.camera_prefix.text(), self._current_calibration_mode())
    if prefix != self.camera_prefix.text().strip():
      self.camera_prefix.setText(prefix)
    self._save_setting(
      "camera_prefix",
      "Camera topic prefix",
      prefix,
      display_value=prefix,
      emit_log=True,
    )
    self._log_camera_prefix_launch_note(prefix)
    success, detail = self.ros_if.set_aruco_camera_prefix(prefix)
    prefix_text = "OK" if success else "ERROR"
    self.log(f"[camera_prefix] {prefix_text}: {detail}")
    self._set_camera_frame_for_mode(self._current_calibration_mode(), log_change=True)
    self._restart_calibrator_for_current_settings("[ui] Camera topic prefix changed")

  def _on_camera_frame_changed(self):
    self._reset_tag_stability()
    frame = normalize_camera_frame(
      self.camera_frame.text(),
      self._current_calibration_mode(),
      self.camera_prefix.text(),
    )
    if frame != self.camera_frame.text().strip():
      self.camera_frame.setText(frame)
    self._save_setting("camera_frame", "Camera frame", frame, display_value=frame, emit_log=True)
    success, detail = self.ros_if.set_calibration_camera_frame(frame)
    prefix_text = "OK" if success else "ERROR"
    self.log(f"[camera_frame] {prefix_text}: {detail}")
    self._restart_calibrator_for_current_settings("[ui] Camera frame changed")


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

  def _restore_ui_settings(self):
    self._set_calibration_mode(self.ros_if.get_default_calibration_mode())
    mode = self._current_calibration_mode()
    self.base_frame.setText(self._read_text_setting("base_frame", self.base_frame.text()))
    self.gripper_frame.setText(self._read_text_setting("gripper_frame", self.gripper_frame.text()))
    self.camera_prefix.setText(default_camera_prefix_for_mode(mode))
    self.camera_frame.setText(default_camera_frame_for_mode(mode, self.camera_prefix.text()))
    self.aruco_ids.setText(
      self._read_text_setting(
        "aruco_ids",
        format_aruco_ids(self.ros_if.get_default_aruco_ids()),
      )
    )

    self.min_samples.setValue(
      self._read_int_setting(
        "min_samples",
        self.min_samples.value(),
        self.min_samples.minimum(),
        self.min_samples.maximum(),
      )
    )

  def _connect_ui_setting_signals(self):
    self.calibration_mode.currentIndexChanged.connect(self._on_calibration_mode_changed)
    self.base_frame.editingFinished.connect(self._on_base_frame_changed)
    self.gripper_frame.editingFinished.connect(self._on_gripper_frame_changed)
    self.camera_frame.editingFinished.connect(self._on_camera_frame_changed)
    self.camera_prefix.editingFinished.connect(self._on_camera_prefix_changed)
    self.aruco_ids.editingFinished.connect(self._on_aruco_ids_changed)

    self.min_samples.valueChanged.connect(self._on_min_samples_changed)

  def _on_base_frame_changed(self):
    self._save_line_edit_setting("base_frame", "Base frame", self.base_frame)
    self._restart_calibrator_for_current_settings("[ui] Base frame changed")

  def _on_gripper_frame_changed(self):
    self._save_line_edit_setting("gripper_frame", "Gripper frame", self.gripper_frame)
    self._restart_calibrator_for_current_settings("[ui] Gripper frame changed")

  def _on_min_samples_changed(self, *_):
    self._save_spin_setting("min_samples", "Minimum samples", self.min_samples)
    self._restart_calibrator_for_current_settings("[ui] Minimum samples changed")

  def _save_line_edit_setting(self, name, label, widget):
    value = str(widget.text() or "").strip()
    if value != widget.text():
      widget.setText(value)
    self._save_setting(name, label, value, display_value=value, emit_log=True)

  def _save_spin_setting(self, name, label, widget, *_):
    numeric_value, display_value = self._format_numeric_widget_value(widget)
    self._save_setting(name, label, numeric_value, display_value=display_value, emit_log=True)

  def _normalize_aruco_ids_field(self, log_errors=True):
    ids, reason = parse_aruco_ids_text(self.aruco_ids.text())
    if ids is None:
      if log_errors:
        self.log(f"[aruco_ids] ERROR: {reason}")
      return None
    normalized = format_aruco_ids(ids)
    if normalized != self.aruco_ids.text().strip():
      self.aruco_ids.setText(normalized)
    return ids

  def _save_aruco_ids_setting(self, log_changes):
    ids = self._normalize_aruco_ids_field(log_errors=log_changes)
    if ids is None:
      return None
    normalized = format_aruco_ids(ids)
    self._save_setting(
      "aruco_ids",
      "ArUco IDs",
      normalized,
      display_value=normalized,
      emit_log=log_changes,
    )
    return ids

  def _apply_aruco_ids_to_perception(self, log_result=True):
    ids = self._save_aruco_ids_setting(log_changes=False)
    if ids is None:
      self.log("[aruco_ids] ERROR: tag_frame will not update until the ID list is valid.")
      return False
    success, detail = self.ros_if.set_calibration_aruco_ids(ids)
    if log_result or not success:
      prefix = "OK" if success else "ERROR"
      self.log(f"[aruco_ids] {prefix}: {detail}")
    return success

  def _on_aruco_ids_changed(self):
    self._reset_tag_stability()
    self._apply_aruco_ids_to_perception(log_result=True)

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
    self._save_setting("camera_prefix", "Camera topic prefix", self.camera_prefix.text().strip(), emit_log=log_changes)
    self._save_setting("camera_frame", "Camera frame", self.camera_frame.text().strip(), emit_log=log_changes)
    self._save_aruco_ids_setting(log_changes)

    min_samples_value, min_samples_display = self._format_numeric_widget_value(self.min_samples)
    self._save_setting(
      "min_samples",
      "Minimum samples",
      min_samples_value,
      display_value=min_samples_display,
      emit_log=log_changes,
    )

  def _log_ui_settings_snapshot(self, title):
    self.log(f"[ui] {title}:")
    self.log(f"[ui] Calibration mode = {self.calibration_mode.currentText()} ({self._current_calibration_mode()})")
    self.log(f"[ui] Base frame = {self.base_frame.text().strip()}")
    self.log(f"[ui] Gripper frame = {self.gripper_frame.text().strip()}")
    self.log(f"[ui] Camera topic prefix = {self.camera_prefix.text().strip()}")
    self.log(f"[ui] Camera frame = {self.camera_frame.text().strip()}")
    self.log(f"[ui] ArUco IDs = {self.aruco_ids.text().strip()} -> {TAG_FRAME}")
    color_topic, depth_topic, camera_info_topic = self.ros_if.get_camera_topics()
    self.log(f"[ui] Active color topic = {color_topic}")
    self.log(f"[ui] Active depth topic = {depth_topic}")
    self.log(f"[ui] Active camera info topic = {camera_info_topic}")
    self.log(f"[ui] Private calibrator namespace base = {self._calibrator_namespace_base}")
    self.log(
      "[ui] Output YAML path = "
      f"{default_output_path(self._current_calibration_mode(), self.ros_if.get_robot_ip_address())} (auto)"
    )
    self.log(f"[ui] Minimum samples = {int(self.min_samples.value())}")

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

    self.tag_gate_watch = QtCore.QTimer(self)
    self.tag_gate_watch.setInterval(250)
    self.tag_gate_watch.timeout.connect(self._refresh_tag_gate_controls)
    self.tag_gate_watch.start()

    self.calibrator_ready_watch = QtCore.QTimer(self)
    self.calibrator_ready_watch.setInterval(250)
    self.calibrator_ready_watch.timeout.connect(self._refresh_calibrator_ready_controls)
    self.calibrator_ready_watch.start()

  def _reset_tag_stability(self):
    self._tag_stability_history = []
    self._tag_stability_key = None
    self._latest_tag_gate_ready = False
    self._latest_tag_gate_detail = "Waiting for stable tag_frame."
    self._set_tag_gate_overlay_text(False, self._latest_tag_gate_detail)

  def _refresh_tag_gate_controls(self):
    ready, detail = self._check_tag_visibility_gate(log_rejections=False)
    self._latest_tag_gate_ready = ready
    self._latest_tag_gate_detail = detail
    self._set_tag_gate_overlay_text(ready, detail)
    self._update_mode_dependent_ui()

  def _refresh_calibrator_ready_controls(self):
    ready = (
      self.calib_process is not None and
      self.ros_if.is_trigger_service_ready(
        self._calibrator_service_name("add_sample"),
        timeout_sec=0.0,
      )
    )
    if ready == self._calibrator_ready:
      return
    self._calibrator_ready = ready
    if ready:
      self.log("Calibrator services ready.")
    self._update_mode_dependent_ui()

  def _update_tag_stability(self, key, pose):
    if key != self._tag_stability_key:
      self._tag_stability_key = key
      self._tag_stability_history = []

    now = time.monotonic()
    current = tuple(float(value) for value in pose[:7])
    self._tag_stability_history.append((now, current))

    retention_sec = max(TAG_STABILITY_WINDOW_SEC * 2.0, TAG_STABILITY_WINDOW_SEC + 0.5)
    self._tag_stability_history = [
      sample for sample in self._tag_stability_history
      if now - sample[0] <= retention_sec
    ]

    if not self._tag_stability_history:
      return False, "Tag stability: waiting for marker samples."

    newest_time, newest_pose = self._tag_stability_history[-1]
    window_start = newest_time - TAG_STABILITY_WINDOW_SEC - TAG_STABILITY_TIME_EPS_SEC
    window_samples = [
      sample for sample in self._tag_stability_history
      if sample[0] >= window_start
    ]
    span_sec = newest_time - window_samples[0][0]
    if span_sec + TAG_STABILITY_TIME_EPS_SEC < TAG_STABILITY_WINDOW_SEC:
      return (
        False,
        f"Tag stability: collecting {min(span_sec, TAG_STABILITY_WINDOW_SEC):.2f}/{TAG_STABILITY_WINDOW_SEC:.2f}s.",
      )

    nx, ny, nz = newest_pose[:3]
    newest_quat = newest_pose[3:7]
    max_translation_mm = 0.0
    max_rotation_deg = 0.0
    for _sample_time, sample_pose in window_samples:
      sx, sy, sz = sample_pose[:3]
      axis_delta_mm = max(abs(sx - nx), abs(sy - ny), abs(sz - nz))
      max_translation_mm = max(max_translation_mm, axis_delta_mm)
      max_rotation_deg = max(
        max_rotation_deg,
        quaternion_angular_distance_deg(sample_pose[3:7], newest_quat),
      )

    stable = (
      max_translation_mm <= TAG_STABILITY_TRANSLATION_TOL_MM and
      max_rotation_deg <= TAG_STABILITY_ROTATION_TOL_DEG
    )
    message = (
      f"Tag stability: {max_translation_mm:.2f}mm / {max_rotation_deg:.2f}deg "
      f"over {span_sec:.2f}s "
      f"(limits {TAG_STABILITY_TRANSLATION_TOL_MM:.1f}mm / "
      f"{TAG_STABILITY_ROTATION_TOL_DEG:.1f}deg)."
    )
    return stable, message

  def _current_calibrator_config(self):
    mode = self._current_calibration_mode()
    return (
      mode,
      self.base_frame.text().strip(),
      self.gripper_frame.text().strip(),
      self.camera_frame.text().strip(),
      default_calibrated_camera_frame_for_mode(mode),
      int(self.min_samples.value()),
      self.ros_if.get_robot_ip_address(),
    )

  def _calibrator_namespace_for_generation(self):
    return f"{self._calibrator_namespace_base}_{self._calibrator_generation}"

  @staticmethod
  def _format_calibrator_config(config):
    if config is None:
      return "none"
    mode, base_frame, gripper_frame, camera_frame, calibrated_frame, min_samples, robot_ip = config
    robot_ip_text = robot_ip or "auto"
    return (
      f"mode={mode}, base={base_frame}, gripper={gripper_frame}, "
      f"camera={camera_frame}, calibrated={calibrated_frame}, "
      f"min_samples={min_samples}, robot_ip={robot_ip_text}"
    )

  def _ensure_active_calibrator_matches_ui(self):
    expected_config = self._current_calibrator_config()
    if self.calib_process is not None and self._active_calibrator_config == expected_config:
      return True
    self.log(
      "[sample] Active solver config mismatch; restarting. "
      f"active=({self._format_calibrator_config(self._active_calibrator_config)}) "
      f"expected=({self._format_calibrator_config(expected_config)})"
    )
    self._restart_calibrator_for_current_settings("[sample] Solver config mismatch")
    self.log("Wait for calibrator services to become ready, then click Get Sample again.")
    return False

  def _restart_calibrator_for_current_settings(self, reason):
    if self.manual_capture_in_flight or self.manual_compute_in_flight or self.reset_samples_in_flight:
      self.log(f"{reason}: wait for the current calibration request to finish before restarting.")
      return

    next_config = self._current_calibrator_config()
    if self.calib_process is not None and self._active_calibrator_config == next_config:
      self._update_mode_dependent_ui()
      return

    self._calibrator_ready = False
    self.preview_compute_in_flight = False
    self._set_sample_count(0)
    self._set_save_yaml_ready(False)
    self._update_mode_dependent_ui()

    if self.calib_process is not None:
      self.log(f"{reason}: restarting calibrator.")
      self._stop_calibrator(log_not_running=False)
    else:
      self.log(f"{reason}: starting calibrator.")
    self.start_calibrator()

  def start_calibrator(self):
    if self.calib_process is not None:
      return True

    self._calibrator_generation += 1
    self._calibrator_namespace = self._calibrator_namespace_for_generation()
    self._calibrator_ready = False
    self._active_calibrator_config = None
    self._update_mode_dependent_ui()
    mode = self._current_calibration_mode()
    calibrated_camera_frame = default_calibrated_camera_frame_for_mode(mode)
    camera_frame = self.camera_frame.text().strip()
    robot_ip_address = self.ros_if.get_robot_ip_address()
    output_path = default_output_path(mode, robot_ip_address)
    self.log(f"Output YAML path: {output_path}")
    self.log(
      "Active solver target: "
      f"mode={mode}, camera_frame={camera_frame}, "
      f"calibrated_frame={calibrated_camera_frame}, "
      f"service={self._calibrator_service_name('add_sample')}"
    )
    parent_frame = self.base_frame.text().strip() if mode == CALIB_MODE_EYE_TO_HAND else self.gripper_frame.text().strip()
    self._set_live_transform_text(
      f"Live calibrated TF preview\n{parent_frame} -> {calibrated_camera_frame}\nWaiting for 3 samples."
    )

    cmd = [
      "ros2",
      "run",
      "camera_calibration",
      "eye_on_hand_calibrator",
      "--ros-args",
      "-r",
      f"__ns:={self._calibrator_namespace}",
      "-p",
      f"calibration_mode:={mode}",
      "-p",
      f"base_frame:={self.base_frame.text()}",
      "-p",
      f"gripper_frame:={self.gripper_frame.text()}",
      "-p",
      f"camera_frame:={camera_frame}",
      "-p",
      f"calibrated_camera_frame:={calibrated_camera_frame}",
      "-p",
      f"target_frame:={TAG_FRAME}",
      "-p",
      f"tracking_base_frame:={camera_frame}",
      "-p",
      f"tracking_marker_frame:={TAG_FRAME}",
      "-p",
      f"min_samples:={self.min_samples.value()}",
      "-p",
      "max_target_age_sec:=0.5",
    ]
    if robot_ip_address:
      cmd.extend([
        "-p",
        f"robot_ip_address:={robot_ip_address}",
      ])
    self.log("Starting calibrator...")
    self.log(f"Calibrator services: {self._calibrator_namespace}/<service>")
    try:
      self.calib_process = subprocess.Popen(cmd, start_new_session=True)
      self._active_calibrator_config = self._current_calibrator_config()
      self.log("Calibrator started.")
      self._update_mode_dependent_ui()
      return True
    except Exception as exc:
      self.log(f"Failed to start calibrator: {exc}")
      self.calib_process = None
      self._active_calibrator_config = None
      self._update_mode_dependent_ui()
      return False

  def _stop_calibrator(self, log_not_running=True):
    self._calibrator_ready = False
    self._active_calibrator_config = None
    self._set_save_yaml_ready(False)
    self.manual_capture_in_flight = False
    self.manual_compute_in_flight = False
    self.reset_samples_in_flight = False
    self.preview_compute_in_flight = False
    if self.calib_process is None:
      if log_not_running:
        self.log("Calibrator is not running.")
      self._update_mode_dependent_ui()
      return
    self.log("Stopping calibrator...")
    try:
      try:
        os.killpg(os.getpgid(self.calib_process.pid), signal.SIGTERM)
      except ProcessLookupError:
        pass
      try:
        self.calib_process.wait(timeout=3.0)
      except subprocess.TimeoutExpired:
        try:
          os.killpg(os.getpgid(self.calib_process.pid), signal.SIGKILL)
        except ProcessLookupError:
          pass
        self.calib_process.wait(timeout=1.0)
    finally:
      self.calib_process = None
      self._set_sample_count(0)
      self.log("Calibrator stopped.")
      self._update_mode_dependent_ui()

  def stop_calibrator(self, *_):
    self._stop_calibrator()

  def _check_calibrator_process(self):
    if self.calib_process is None:
      return
    if self.calib_process.poll() is None:
      return
    # Process exited on its own; mirror UI state and close GUI so launch shuts down cleanly.
    self.log("Calibrator process exited; closing GUI.")
    self.manual_capture_in_flight = False
    self.manual_compute_in_flight = False
    self.reset_samples_in_flight = False
    self.preview_compute_in_flight = False
    self._calibrator_ready = False
    self._active_calibrator_config = None
    self.calib_process = None
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

  def _check_tag_visibility_gate(self, log_rejections):
    mode = self._current_calibration_mode()
    camera_frame = self.camera_frame.text().strip() or default_camera_frame_for_mode(
      mode, self.camera_prefix.text())
    target_frame = TAG_FRAME
    pose = self.ros_if.lookup_pose_mm_with_age(
      camera_frame,
      target_frame,
      timeout_sec=0.08 if log_rejections else 0.01,
      warn_on_failure=log_rejections,
    )
    if pose is None:
      message = f"No TF sample yet for {camera_frame} -> {target_frame}."
      self._tag_stability_history = []
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    _tx, _ty, _tz, _qx, _qy, _qz, _qw, stamp_age_sec = pose
    tf_age_sec = self.ros_if.latest_dynamic_tf_age(camera_frame, target_frame)
    age_sec = tf_age_sec if tf_age_sec is not None else stamp_age_sec
    if not math.isfinite(age_sec) or age_sec > TAG_FRAME_MAX_AGE_SEC:
      age_text = "unknown" if not math.isfinite(age_sec) else f"{max(0.0, age_sec):.3f}s"
      message = (
        f"Tag TF is stale (age {age_text}, max {TAG_FRAME_MAX_AGE_SEC:.2f}s). "
        "All 4 calibration markers must be visible."
      )
      self._tag_stability_history = []
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    stable, stability_detail = self._update_tag_stability((camera_frame, target_frame), pose)
    if not stable:
      message = (
        f"tf_age={max(0.0, age_sec):.3f}s; {stability_detail} "
        "All 4 calibration markers must stay visible before sampling."
      )
      if log_rejections:
        self.log(f"[visibility] {message}")
      return False, message

    detail = f"all 4 markers visible, tf_age={max(0.0, age_sec):.3f}s; {stability_detail}"
    return True, detail

  @staticmethod
  def _parse_sample_total(message):
    match = re.search(r"Total:\s*(\d+)", str(message or ""))
    if match is None:
      return None
    try:
      return int(match.group(1))
    except ValueError:
      return None

  def _request_preview_calibration(self, sample_count):
    if sample_count < 3:
      return
    if self.preview_compute_in_flight:
      return

    self.preview_compute_in_flight = True
    dispatched = self._call_trigger_async(
      "preview_calibration",
      self._on_preview_calibration_done,
    )
    if not dispatched:
      self.preview_compute_in_flight = False
      self.log("Failed to call preview_calibration.")

  def _on_preview_calibration_done(self, success, message):
    self.preview_compute_in_flight = False
    if success:
      self._set_live_transform_text(self._extract_transform_block(message))
    else:
      self.log(f"[preview_calibration] ERROR: {message}")
      self._set_live_transform_text(f"Live calibrated TF preview\n{message}")
    self._update_mode_dependent_ui()
    self._update_undo_button_state()

  def handle_add_button_clicked(self):
    self.get_manual_sample()

  def get_manual_sample(self):
    self._persist_all_ui_settings(log_changes=False)
    self.ros_if.apply_calibration_mode_tool(self._current_calibration_mode())
    if not self._apply_aruco_ids_to_perception(log_result=False):
      return
    if self.manual_capture_in_flight or self.manual_compute_in_flight:
      self.log("Sample capture/compute is already in progress.")
      return
    if self.reset_samples_in_flight:
      self.log("Reset samples is already in progress.")
      return

    if self.calib_process is None:
      if self.start_calibrator():
        self.log("Calibrator was not running; wait for services to become ready, then click Get Sample.")
      return
    if not self._ensure_active_calibrator_matches_ui():
      return
    if not self._calibrator_ready:
      self.log("Calibration services are still starting; wait for Get Sample to enable.")
      self._update_mode_dependent_ui()
      return

    visible, _detail = self._check_tag_visibility_gate(log_rejections=True)
    if not visible:
      self.log("Sample not added: visibility gate failed.")
      return

    self._set_save_yaml_ready(False)
    self.manual_capture_in_flight = True
    self.add_btn.setEnabled(False)
    self.reset_samples_btn.setEnabled(False)
    dispatched = self._call_trigger_async("add_sample", self._on_manual_add_sample_done)
    if not dispatched:
      self.manual_capture_in_flight = False
      self.log("Failed to call add_sample for manual capture.")
      self._update_mode_dependent_ui()
      self._update_undo_button_state()

  def _on_manual_add_sample_done(self, success, message):
    self.manual_capture_in_flight = False
    if not success:
      self.log(f"[add_sample] ERROR: {message}")
      self._update_mode_dependent_ui()
      self._update_undo_button_state()
      return

    parsed_total = self._parse_sample_total(message)
    if parsed_total is None:
      self.log(f"[add_sample] ERROR: response did not include solver sample count: {message}")
      self._update_mode_dependent_ui()
      self._update_undo_button_state()
      return

    self._set_sample_count(parsed_total)

    required = int(self.min_samples.value())
    self._log_sample_count()
    preview_from_add_sample = "transform:" in str(message or "")
    if preview_from_add_sample:
      self._set_live_transform_text(self._extract_transform_block(message))

    if self.manual_sample_count < required:
      if not preview_from_add_sample:
        self._request_preview_calibration(self.manual_sample_count)
      self._update_mode_dependent_ui()
      self._update_undo_button_state()
      return

    self.manual_compute_in_flight = True
    self.add_btn.setEnabled(False)
    self.undo_last_btn.setEnabled(False)
    self.reset_samples_btn.setEnabled(False)
    dispatched = self._call_trigger_async("compute_calibration", self._on_manual_compute_done)
    if not dispatched:
      self.manual_compute_in_flight = False
      self._update_mode_dependent_ui()
      self.log("Failed to call compute_calibration after manual sample.")

  def _on_manual_compute_done(self, success, message):
    self.manual_compute_in_flight = False
    message_lc = str(message or "").lower()
    expected_frame = default_calibrated_camera_frame_for_mode(self._current_calibration_mode())
    tf_broadcasted = ("broadcasted static tf" in message_lc) and (expected_frame in message_lc)
    if success and tf_broadcasted:
      self._set_live_transform_text(self._extract_transform_block(message))
      self._set_save_yaml_ready(True)
    elif success:
      self._set_live_transform_text(self._extract_transform_block(message))
      self._set_save_yaml_ready(False)
    else:
      self._set_save_yaml_ready(False)
      self.log(f"[compute_calibration] ERROR: {message}")
    self._update_mode_dependent_ui()
    self._update_undo_button_state()

  def undo_last_sample(self):
    if self.manual_capture_in_flight or self.manual_compute_in_flight or self.reset_samples_in_flight:
      self.log("Cannot undo while sample capture/compute/reset is in progress.")
      return
    if self.calib_process is None:
      self.log("Calibrator is not running. No active samples to undo.")
      self._set_sample_count(0)
      return
    if self.manual_sample_count <= 0:
      self.log("No samples to undo.")
      return

    self._set_save_yaml_ready(False)
    self.undo_last_btn.setEnabled(False)
    self.reset_samples_btn.setEnabled(False)
    dispatched = self._call_trigger_async("remove_last_sample", self._on_undo_last_sample_done)
    if not dispatched:
      self._update_undo_button_state()
      self.log("Failed to call remove_last_sample.")

  def _on_undo_last_sample_done(self, success, message):
    prefix = "OK" if success else "ERROR"
    self.log(f"[remove_last_sample] {prefix}: {message}")
    if not success:
      self._update_undo_button_state()
      return

    parsed_total = self._parse_sample_total(message)
    if parsed_total is not None:
      self._set_sample_count(parsed_total)
    else:
      self._set_sample_count(self.manual_sample_count - 1)

    self._set_save_yaml_ready(False)
    self._log_sample_count()
    parent_frame = (
      self.base_frame.text().strip()
      if self._current_calibration_mode() == CALIB_MODE_EYE_TO_HAND
      else self.gripper_frame.text().strip()
    )
    calibrated_frame = default_calibrated_camera_frame_for_mode(self._current_calibration_mode())
    if self.manual_sample_count < 3:
      self._set_live_transform_text(
        f"Live calibrated TF preview\n{parent_frame} -> {calibrated_frame}\nWaiting for 3 samples."
      )
      self._update_mode_dependent_ui()
      self._update_undo_button_state()
      return

    if self.manual_sample_count >= int(self.min_samples.value()):
      self.manual_compute_in_flight = True
      self.add_btn.setEnabled(False)
      self.undo_last_btn.setEnabled(False)
      self.reset_samples_btn.setEnabled(False)
      dispatched = self._call_trigger_async("compute_calibration", self._on_manual_compute_done)
      if not dispatched:
        self.manual_compute_in_flight = False
        self.log("Failed to recompute calibration after undo.")
        self._update_mode_dependent_ui()
        self._update_undo_button_state()
      return

    self._request_preview_calibration(self.manual_sample_count)
    self._update_mode_dependent_ui()
    self._update_undo_button_state()

  def _confirm_reset_samples(self):
    dialog = QtWidgets.QMessageBox(self)
    dialog.setIcon(QtWidgets.QMessageBox.Warning)
    dialog.setWindowTitle("Reset Samples?")
    dialog.setText("Delete all calibration samples?")
    dialog.setInformativeText(
      "This cannot be undone. The current calibration sample set will be cleared."
    )
    reset_button = dialog.addButton("Reset Samples", QtWidgets.QMessageBox.DestructiveRole)
    cancel_button = dialog.addButton("Cancel", QtWidgets.QMessageBox.RejectRole)
    dialog.setDefaultButton(cancel_button)
    dialog.exec_()
    return dialog.clickedButton() == reset_button

  def reset_samples(self):
    if self.manual_capture_in_flight or self.manual_compute_in_flight or self.reset_samples_in_flight:
      self.log("Cannot reset while sample capture/compute/reset is in progress.")
      return
    if self.calib_process is None:
      self.log("Calibrator is not running. No active samples to reset.")
      self._set_sample_count(0)
      self._set_save_yaml_ready(False)
      return
    if self.manual_sample_count <= 0:
      self.log("No samples to reset.")
      return
    if not self._confirm_reset_samples():
      self.log("Reset samples canceled.")
      return

    self._set_save_yaml_ready(False)
    self.reset_samples_in_flight = True
    self.add_btn.setEnabled(False)
    self.undo_last_btn.setEnabled(False)
    self.reset_samples_btn.setEnabled(False)
    dispatched = self._call_trigger_async("reset_samples", self._on_reset_samples_done)
    if not dispatched:
      self.reset_samples_in_flight = False
      self.log("Failed to call reset_samples.")
      self._update_mode_dependent_ui()
      self._update_undo_button_state()

  def _on_reset_samples_done(self, success, message):
    self.reset_samples_in_flight = False
    prefix = "OK" if success else "ERROR"
    self.log(f"[reset_samples] {prefix}: {message}")
    if success:
      self._set_sample_count(0)
      parent_frame = (
        self.base_frame.text().strip()
        if self._current_calibration_mode() == CALIB_MODE_EYE_TO_HAND
        else self.gripper_frame.text().strip()
      )
      calibrated_frame = default_calibrated_camera_frame_for_mode(self._current_calibration_mode())
      self._set_live_transform_text(
        f"Live calibrated TF preview\n{parent_frame} -> {calibrated_frame}\nWaiting for 3 samples."
      )
      self._log_sample_count()
    self._update_mode_dependent_ui()
    self._update_undo_button_state()

  def _call_trigger_async(self, service_name, done_callback):
    service_path = self._calibrator_service_name(service_name)
    client = self.ros_if.get_client(service_path)
    if client is None:
      self.log(f"Service {service_path} unavailable.")
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

    service_path = self._calibrator_service_name(service_name)
    client = self.ros_if.get_client(service_path)
    if client is None:
      self.log(f"Service {service_path} unavailable. Ensure calibrator is running.")
      return

    future = client.call_async(Trigger.Request())
    future.add_done_callback(done_callback)

  def _calibrator_service_name(self, service_name):
    name = str(service_name or "").strip().strip("/")
    return f"{self._calibrator_namespace}/{name}"

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
    self._latest_overlay_qimage = None
    self._latest_overlay_received_monotonic = None
    self._latest_dynamic_tf_received_monotonic = {}
    self._unsupported_overlay_encodings = set()
    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
    self._default_calibration_mode = normalize_calibration_mode(
      self.declare_parameter("calibration_mode", CALIB_MODE_EYE_ON_HAND).value
    )
    self._robot_ip_address = resolve_robot_ip_address(
      self.declare_parameter("robot_ip_address", "").value
    )
    self._default_camera_prefix = normalize_camera_prefix(
      self.declare_parameter(
        "camera_prefix",
        default_camera_prefix_for_mode(self._default_calibration_mode),
      ).value,
      self._default_calibration_mode,
    )
    self._default_camera_frame = normalize_camera_frame(
      self.declare_parameter(
        "camera_frame",
        default_camera_frame_for_mode(self._default_calibration_mode, self._default_camera_prefix),
      ).value,
      self._default_calibration_mode,
      self._default_camera_prefix,
    )
    self._default_aruco_ids = DEFAULT_ARUCO_IDS.copy()
    aruco_ids_value = self.declare_parameter("aruco_ids", DEFAULT_ARUCO_IDS).value
    aruco_ids, aruco_ids_reason = coerce_aruco_ids(aruco_ids_value)
    if aruco_ids is None:
      self.get_logger().warn(
        f"Invalid aruco_ids parameter: {aruco_ids_reason} Using {format_aruco_ids(DEFAULT_ARUCO_IDS)}."
      )
    else:
      self._default_aruco_ids = aruco_ids
    self._color_topic = str(
      self.declare_parameter(
        "color_topic",
        f"/{self._default_camera_prefix}/color/image_raw",
      ).value
    )
    self._depth_topic = str(
      self.declare_parameter(
        "depth_topic",
        f"/{self._default_camera_prefix}/depth/image_raw",
      ).value
    )
    self._camera_info_topic = str(
      self.declare_parameter(
        "camera_info_topic",
        f"/{self._default_camera_prefix}/color/camera_info",
      ).value
    )
    self._aruco_set_parameters_client = self.create_client(
      SetParameters,
      "/aruco_perception/set_parameters",
    )
    self._calibration_perception_set_parameters_client = self.create_client(
      SetParameters,
      "/calibration_perception/set_parameters",
    )
    self.apply_calibration_mode_tool(self._default_calibration_mode)

    self.create_subscription(
      Image,
      "/aruco_overlay",
      self._overlay_callback,
      5,
    )
    self.create_subscription(
      TFMessage,
      "/tf",
      self._tf_callback,
      50,
    )

  def get_client(self, service_name):
    if service_name in self._client_map:
      client = self._client_map[service_name]
    else:
      client = self.create_client(Trigger, service_name, qos_profile=qos_profile_services_default)
      self._client_map[service_name] = client

    if not client.wait_for_service(timeout_sec=1.5):
      self.get_logger().warn(f"Service {service_name} not available")
      return None
    return client

  def is_trigger_service_ready(self, service_name, timeout_sec=0.0):
    if service_name in self._client_map:
      client = self._client_map[service_name]
    else:
      client = self.create_client(Trigger, service_name, qos_profile=qos_profile_services_default)
      self._client_map[service_name] = client
    return bool(client.wait_for_service(timeout_sec=max(0.0, float(timeout_sec))))

  def get_default_calibration_mode(self):
    return self._default_calibration_mode

  def get_default_camera_prefix(self):
    return self._default_camera_prefix

  def get_default_camera_frame(self):
    return self._default_camera_frame

  def get_default_aruco_ids(self):
    return list(self._default_aruco_ids)

  def get_robot_ip_address(self):
    return self._robot_ip_address

  def get_camera_topics(self):
    return self._color_topic, self._depth_topic, self._camera_info_topic

  @staticmethod
  def _tf_key(parent_frame, child_frame):
    return (
      str(parent_frame or "").strip().strip("/"),
      str(child_frame or "").strip().strip("/"),
    )

  def _tf_callback(self, msg):
    received = time.monotonic()
    for transform in msg.transforms:
      key = self._tf_key(transform.header.frame_id, transform.child_frame_id)
      self._latest_dynamic_tf_received_monotonic[key] = received

  def latest_dynamic_tf_age(self, parent_frame, child_frame):
    received = self._latest_dynamic_tf_received_monotonic.get(
      self._tf_key(parent_frame, child_frame)
    )
    if received is None:
      return None
    return time.monotonic() - received

  @staticmethod
  def _to_parameter_msg(parameter_spec):
    if isinstance(parameter_spec, Parameter):
      return parameter_spec.to_parameter_msg()
    if len(parameter_spec) == 2:
      name, value = parameter_spec
      parameter_type = Parameter.Type.STRING
    else:
      name, parameter_type, value = parameter_spec
    return Parameter(name, parameter_type, value).to_parameter_msg()

  def _call_set_parameters(self, client, parameters, unavailable_message, timeout_sec=1.5):
    if not client.wait_for_service(timeout_sec=timeout_sec):
      return False, unavailable_message

    req = SetParameters.Request()
    req.parameters = [self._to_parameter_msg(parameter_spec) for parameter_spec in parameters]
    future = client.call_async(req)
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    while rclpy.ok() and (not future.done()) and time.monotonic() < deadline:
      rclpy.spin_once(self, timeout_sec=0.01)
    if not future.done():
      return False, "Timed out updating parameters."

    try:
      res = future.result()
    except Exception as exc:
      return False, f"SetParameters call failed: {exc}"

    failures = [result.reason for result in res.results if not result.successful]
    if failures:
      return False, "; ".join(reason or "parameter rejected" for reason in failures)
    return True, "parameters updated"

  def set_aruco_camera_prefix(self, prefix, timeout_sec=1.5):
    normalized = normalize_camera_prefix(prefix, self._default_calibration_mode)
    color_topic, depth_topic, camera_info_topic = camera_topics_for_prefix(normalized)
    success, detail = self._call_set_parameters(
      self._aruco_set_parameters_client,
      [
        ("color_topic", color_topic),
        ("depth_topic", depth_topic),
        ("camera_info_topic", camera_info_topic),
      ],
      "Service unavailable: /aruco_perception/set_parameters. "
      f"Launch will use camera_prefix:={normalized} on restart.",
      timeout_sec=timeout_sec,
    )
    if not success:
      return False, detail

    self._default_camera_prefix = normalized
    self._color_topic = color_topic
    self._depth_topic = depth_topic
    self._camera_info_topic = camera_info_topic
    return (
      True,
      f"ArUco stream updated to {normalized}: "
      f"{color_topic}, {depth_topic}, {camera_info_topic}",
    )

  def set_calibration_camera_frame(self, camera_frame, timeout_sec=1.5):
    frame = normalize_camera_frame(camera_frame, self._default_calibration_mode, self._default_camera_prefix)
    aruco_success, aruco_detail = self._call_set_parameters(
      self._aruco_set_parameters_client,
      [("camera_frame", frame)],
      "Service unavailable: /aruco_perception/set_parameters.",
      timeout_sec=timeout_sec,
    )
    perception_success, perception_detail = self._call_set_parameters(
      self._calibration_perception_set_parameters_client,
      [("parent_frame", frame)],
      "Service unavailable: /calibration_perception/set_parameters.",
      timeout_sec=timeout_sec,
    )
    if aruco_success and perception_success:
      self._default_camera_frame = frame
      return True, f"ArUco/tag frame parent updated to {frame}."
    return (
      False,
      f"ArUco: {aruco_detail}; calibration_perception: {perception_detail}",
    )

  def set_calibration_aruco_ids(self, ids, timeout_sec=1.5):
    normalized = [int(marker_id) for marker_id in ids]
    success, detail = self._call_set_parameters(
      self._calibration_perception_set_parameters_client,
      [("marker_ids", Parameter.Type.INTEGER_ARRAY, normalized)],
      "Service unavailable: /calibration_perception/set_parameters.",
      timeout_sec=timeout_sec,
    )
    if success:
      self._default_aruco_ids = normalized
      return True, f"{TAG_FRAME} now fits depth board pose from ArUco IDs {format_aruco_ids(normalized)}."
    return False, detail

  def apply_calibration_mode_tool(self, calibration_mode):
    self._default_calibration_mode = normalize_calibration_mode(calibration_mode)

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

  def lookup_pose_mm(self, parent_frame, child_frame, timeout_sec=0.15, warn_on_failure=True):
    pose = self.lookup_pose_mm_with_age(
      parent_frame,
      child_frame,
      timeout_sec,
      warn_on_failure=warn_on_failure,
    )
    if pose is None:
      return None
    return pose[:7]

  def lookup_pose_mm_with_age(
    self,
    parent_frame,
    child_frame,
    timeout_sec=0.15,
    warn_on_failure=True,
  ):
    timeout_ns = int(max(0.01, float(timeout_sec)) * 1e9)
    try:
      tf_msg = self._tf_buffer.lookup_transform(
        parent_frame,
        child_frame,
        Time(),
        timeout=Duration(nanoseconds=timeout_ns),
      )
      stamp = Time.from_msg(tf_msg.header.stamp)
      age_sec = float("inf")
      if stamp.nanoseconds != 0:
        age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
      return (
        float(tf_msg.transform.translation.x) * 1000.0,
        float(tf_msg.transform.translation.y) * 1000.0,
        float(tf_msg.transform.translation.z) * 1000.0,
        float(tf_msg.transform.rotation.x),
        float(tf_msg.transform.rotation.y),
        float(tf_msg.transform.rotation.z),
        float(tf_msg.transform.rotation.w),
        float(age_sec),
      )
    except TransformException as exc:
      if warn_on_failure:
        self.get_logger().warn(
          f"TF lookup failed ({parent_frame} -> {child_frame}): {exc}"
        )
      return None

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
