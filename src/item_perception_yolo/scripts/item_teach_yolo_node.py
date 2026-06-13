#!/usr/bin/env python3
import datetime as _dt
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import torch
import yaml
from dobot_msgs_v4.srv import GetAngle
from orbbec_camera_msgs.srv import SetInt32
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists() and
            (
                (path / "README.md").exists()
                or (path / "src" / "dobot_msgs_v4").exists()
            )
        )

    def find_from(start: Path) -> Optional[Path]:
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


WINDOW_NAME = "item_teach_yolo_view"
BIN_CAMERA_COLOR_TOPIC = "/bin_camera/color/image_raw"
BIN_CAMERA_DEPTH_TOPIC = "/bin_camera/depth/image_raw"
BIN_CAMERA_INFO_TOPIC = "/bin_camera/color/camera_info"
BIN_CAMERA_CONTROL_SERVICE_ROOT = "/bin_camera"
LEFT_PANEL_WIDTH = 440
VIDEO_TOP_BAR_HEIGHT = 92
PREVIEW_CANVAS_WIDTH = 1080
PREVIEW_CANVAS_HEIGHT = 680
PANEL_PAD = 20
BUTTON_HEIGHT = 38
DROPDOWN_ROW_HEIGHT = 32
MAX_DROPDOWN_ROWS = 7
MAX_SESSION_DROPDOWN_ROWS = 8
EXPOSURE_PERCENT_MIN = 0
EXPOSURE_PERCENT_MAX = 100
DEFAULT_EXPOSURE_MIN_US = 1
DEFAULT_EXPOSURE_MAX_US = 32000
TEACH_COLOR_EXPOSURE_MAX_US = 100
DEFAULT_RECORD_FPS = 5.0
VIDEO_FILE_SUFFIXES = {".avi", ".mp4", ".mov", ".mkv", ".mpeg", ".mpg", ".m4v"}
IMAGE_FILE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
RAW_CAPTURE_SUFFIX = "_raw"


@dataclass
class BinEntry:
    label: str
    path: Path
    bin_name: str
    teach_date: str
    roi_points: List[Tuple[float, float]]
    depth_plane: Dict[str, float]


@dataclass
class Button:
    name: str
    rect: Tuple[int, int, int, int]
    enabled: bool = True
    role: str = "default"


@dataclass
class SavedSessionEntry:
    label: str
    path: Path
    item_name: str
    sample_count: int
    background_sample_count: int
    modified_time: float


@dataclass
class VideoRecordingEntry:
    label: str
    path: Path
    frame_count: int
    annotated_count: int
    skipped_count: int
    modified_time: float


def resolve_path(path_text: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_text))).resolve()


def safe_name(text: str, fallback: str = "") -> str:
    value = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip().lower()).strip("_")
    return value or fallback


def timestamp_for_path() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def compact_date_for_path() -> str:
    return _dt.datetime.now().strftime("%d%m%Y")


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def normalize_service_root(root: object, fallback: str = "/dobot_bringup_ros2/srv") -> str:
    value = str(root or "").strip()
    while value.endswith("/") and len(value) > 1:
        value = value[:-1]
    return value or fallback


def clamp_exposure_percent(percent: int) -> int:
    return max(EXPOSURE_PERCENT_MIN, min(EXPOSURE_PERCENT_MAX, int(percent)))


def clamp_exposure_usec(value: int) -> int:
    return max(1, int(value))


def exposure_percent_to_usec(percent: int, min_us: int, max_us: int) -> int:
    clamped_percent = clamp_exposure_percent(percent)
    if clamped_percent <= 0:
        return 0
    clamped_min = clamp_exposure_usec(min_us)
    clamped_max = max(clamped_min, clamp_exposure_usec(max_us))
    value = clamped_min + round((clamped_max - clamped_min) * (clamped_percent / 100.0))
    return max(clamped_min, min(clamped_max, int(value)))


def clamp_exposure_usec_or_auto(value: int, min_us: int, max_us: int) -> int:
    if int(value) <= 0:
        return 0
    clamped_min = clamp_exposure_usec(min_us)
    clamped_max = max(clamped_min, clamp_exposure_usec(max_us))
    return max(clamped_min, min(clamped_max, int(value)))


def exposure_usec_to_percent(exposure_us: int, min_us: int, max_us: int) -> int:
    if int(exposure_us) <= 0:
        return 0
    clamped_min = clamp_exposure_usec(min_us)
    clamped_max = max(clamped_min, clamp_exposure_usec(max_us))
    clamped_exposure = max(clamped_min, min(clamped_max, int(exposure_us)))
    if clamped_max == clamped_min:
        return 100
    return clamp_exposure_percent(
        round(((clamped_exposure - clamped_min) / float(clamped_max - clamped_min)) * 100.0))


def parse_flat_points(values) -> List[Tuple[float, float]]:
    if not isinstance(values, list) or len(values) < 8:
        return []
    points: List[Tuple[float, float]] = []
    for i in range(0, min(len(values), 8), 2):
        points.append((float(values[i]), float(values[i + 1])))
    return points if len(points) == 4 else []


def fit_text(text: str, max_chars: int) -> str:
    if len(text) <= max_chars:
        return text
    return text[: max(0, max_chars - 3)] + "..."


def polygon_area(points: np.ndarray) -> float:
    if points.shape[0] < 3:
        return 0.0
    return float(abs(cv2.contourArea(points.reshape(-1, 1, 2).astype(np.float32))))


class ItemTeachYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("item_teach_yolo")
        self.bridge = CvBridge()

        self.color_topic = self.declare_parameter("color_topic", BIN_CAMERA_COLOR_TOPIC).value
        self.motion_service_root = normalize_service_root(
            self.declare_parameter("motion_service_root", "/dobot_bringup_ros2/srv").value)
        self.get_angle_service_name = f"{self.motion_service_root}/GetAngle"
        self.item_name = str(self.declare_parameter("item_name", "").value).strip()
        self.bin_teach_dir = resolve_path(
            self.declare_parameter(
                "bin_teach_dir",
                str(workspace_path("teach", "bin_teach")),
            ).value)
        self.runtime_root = resolve_path(
            self.declare_parameter(
                "runtime_root",
                str(workspace_path("config", "item_perception_yolo", "item_teach_yolo_runtime")),
            ).value)
        self.saved_sessions_root = resolve_path(
            self.declare_parameter(
                "saved_sessions_root",
                str(workspace_path("config", "item_perception_yolo", "item_teach_yolo_saved_sessions")),
            ).value)
        self.runtime_settings_path = resolve_path(
            self.declare_parameter(
                "runtime_settings_path",
                str(workspace_path("config", "item_perception_yolo", "item_teach_yolo_runtime_settings.yaml")),
            ).value)
        self.clear_runtime_on_start = as_bool(
            self.declare_parameter("clear_runtime_on_start", False).value)
        self.profile_dir = resolve_path(
            self.declare_parameter(
                "profile_dir",
                str(workspace_path("teach", "item_teach_yolo")),
            ).value)
        self.model_root = resolve_path(
            self.declare_parameter(
                "model_root",
                str(workspace_path("teach", "item_teach_yolo")),
            ).value)
        self.depth_topic = self.declare_parameter("depth_topic", BIN_CAMERA_DEPTH_TOPIC).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", BIN_CAMERA_INFO_TOPIC).value
        self.overlay_topic = self.declare_parameter("overlay_topic", "bin_overlay").value
        self.camera_control_service_root = self.normalize_camera_control_service_root(
            self.declare_parameter("camera_control_service_root", BIN_CAMERA_CONTROL_SERVICE_ROOT).value)
        color_exposure_percent = clamp_exposure_percent(
            int(self.declare_parameter("color_exposure_percent", 0).value))
        depth_exposure_percent = clamp_exposure_percent(
            int(self.declare_parameter("depth_exposure_percent", 0).value))
        self.color_exposure_min_us = clamp_exposure_usec(
            int(self.declare_parameter("color_exposure_min_us", DEFAULT_EXPOSURE_MIN_US).value))
        self.color_exposure_max_us = max(
            self.color_exposure_min_us,
            clamp_exposure_usec(
                int(self.declare_parameter("color_exposure_max_us", DEFAULT_EXPOSURE_MAX_US).value)))
        self.color_exposure_min_us = max(1, min(self.color_exposure_min_us, TEACH_COLOR_EXPOSURE_MAX_US))
        self.color_exposure_max_us = TEACH_COLOR_EXPOSURE_MAX_US
        self.depth_exposure_min_us = clamp_exposure_usec(
            int(self.declare_parameter("depth_exposure_min_us", DEFAULT_EXPOSURE_MIN_US).value))
        self.depth_exposure_max_us = max(
            self.depth_exposure_min_us,
            clamp_exposure_usec(
                int(self.declare_parameter("depth_exposure_max_us", DEFAULT_EXPOSURE_MAX_US).value)))
        self.color_exposure_us = clamp_exposure_usec_or_auto(
            int(self.declare_parameter(
                "color_exposure_us",
                exposure_percent_to_usec(
                    color_exposure_percent,
                    self.color_exposure_min_us,
                    self.color_exposure_max_us)).value),
            self.color_exposure_min_us,
            self.color_exposure_max_us)
        self.depth_exposure_us = clamp_exposure_usec_or_auto(
            int(self.declare_parameter(
                "depth_exposure_us",
                exposure_percent_to_usec(
                    depth_exposure_percent,
                    self.depth_exposure_min_us,
                    self.depth_exposure_max_us)).value),
            self.depth_exposure_min_us,
            self.depth_exposure_max_us)
        self.depth_exposure_us = 0
        self.sam2_checkpoint = resolve_path(
            self.declare_parameter(
                "sam2_checkpoint",
                str(workspace_path("third_party", "sam2", "checkpoints", "sam2.1_hiera_tiny.pt"))).value)
        self.sam2_config = self.declare_parameter(
            "sam2_config", "configs/sam2.1/sam2.1_hiera_t.yaml").value
        self.yolo_base_model = self.declare_parameter(
            "yolo_base_model",
            str(workspace_path("third_party", "yolo", "checkpoints", "yolo11n-seg.pt"))).value
        self.train_epochs = int(self.declare_parameter("train_epochs", 80).value)
        self.train_imgsz = int(self.declare_parameter("train_imgsz", 640).value)
        self.train_device = str(self.declare_parameter("train_device", "0").value).strip() or "0"
        self.train_use_gpu_if_available = as_bool(
            self.declare_parameter("train_use_gpu_if_available", True).value)
        self.display_scale = float(self.declare_parameter("display_scale", 1.0).value)
        self.overlay_enabled = as_bool(self.declare_parameter("overlay_enabled", True).value)
        self.live_view_enabled = as_bool(self.declare_parameter("live_view_enabled", True).value)
        self.record_fps = max(
            0.5,
            float(self.declare_parameter("record_fps", DEFAULT_RECORD_FPS).value),
        )

        self.lock = threading.Lock()
        self.latest_bgr: Optional[np.ndarray] = None
        self.latest_header_stamp = ""
        self.latest_joint_positions_deg = [0.0] * 6
        self.has_joint_positions = False
        self.teach_joints_source = ""
        self.get_angle_request_in_flight = False
        self.pending_teach_joint_snapshot_reason = ""
        self.pending_teach_joint_snapshot_save = False
        self.pending_teach_joint_snapshot_update_profile = False
        self.get_angle_warning_logged = False

        self.bin_entries: List[BinEntry] = []
        self.active_bin_index = -1
        self.active_bin: Optional[BinEntry] = None
        self.bin_dropdown_open = False
        self.saved_session_entries: List[SavedSessionEntry] = []
        self.load_session_dropdown_open = False
        self.saved_session_option_rects: List[Tuple[int, int, int, int, int]] = []
        self.saved_session_delete_rects: List[Tuple[int, int, int, int, int]] = []
        self.roi_crop_rect: Optional[Tuple[int, int, int, int]] = None
        self.roi_crop_mask: Optional[np.ndarray] = None

        self.frozen_frame_bgr: Optional[np.ndarray] = None
        self.frozen_crop_bgr: Optional[np.ndarray] = None
        self.frozen_crop_rgb: Optional[np.ndarray] = None
        self.current_mask: Optional[np.ndarray] = None
        self.sam2_image_key: Optional[Tuple[int, int, int]] = None
        self.positive_points: List[Tuple[int, int]] = []
        self.negative_points: List[Tuple[int, int]] = []
        self.prediction_dirty = False
        self.predicting = False
        self.status = "Load a Bin Teach file to begin"
        self.item_name_edit_active = False
        self.item_name_edit_buffer = self.item_name

        self.sample_count = 0
        self.background_sample_count = 0
        self.sample_history: List[Dict[str, str]] = []
        self.training_thread: Optional[threading.Thread] = None
        self.training_status = "idle"
        self.training_epoch_current = 0
        self.training_epoch_total = max(1, self.train_epochs)
        self.training_progress = 0.0
        self.trained_model_path = ""
        self.trained_onnx_path = ""
        self.train_device_used = "cpu"
        self.final_model_dir = ""
        self.latest_profile_path = ""
        self.video_recordings: List[VideoRecordingEntry] = []
        self.roi_image_capture_count = 0
        self.recording_active = False
        self.recording_dir: Optional[Path] = None
        self.recording_frames_dir: Optional[Path] = None
        self.recording_metadata: Dict = {}
        self.recording_writer: Optional[cv2.VideoWriter] = None
        self.recording_video_path: Optional[Path] = None
        self.recording_frame_count = 0
        self.recording_last_capture_time = 0.0
        self.last_space_capture_time = 0.0
        self.recording_frame_size = (0, 0)
        self.review_mode = False
        self.review_recording_index = -1
        self.review_recording_meta: Dict = {}
        self.review_frame_index = 0

        self.buttons: Dict[str, Button] = {}
        self.item_name_input_rect = (0, 0, 0, 0)
        self.preview_rect = (LEFT_PANEL_WIDTH, VIDEO_TOP_BAR_HEIGHT, PREVIEW_CANVAS_WIDTH, PREVIEW_CANVAS_HEIGHT)
        self.preview_scale = 1.0
        self.preview_image_size = (0, 0)
        self.preview_source_size = (0, 0)
        self.rendered_window_size = (0, 0)
        self.exposure_slider_rect = (0, 0, 0, 0)
        self.exposure_slider_active = False
        self.camera_exposure_dirty = True
        self.last_camera_exposure_attempt_time = self.get_clock().now()
        self.last_applied_color_exposure_us = -1
        self.last_applied_depth_exposure_us = -1

        self.load_runtime_settings()

        if self.clear_runtime_on_start:
            self.clear_runtime_root()
        self.session_dir = self.create_session_dir()
        self.configure_session_storage()
        self.write_dataset_yaml()

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(
            f"Loading SAM2 checkpoint={self.sam2_checkpoint} device={self.device}")
        sam2_model = build_sam2(self.sam2_config, str(self.sam2_checkpoint), device=self.device)
        self.predictor = SAM2ImagePredictor(sam2_model)

        self.refresh_bin_files()
        self.refresh_saved_sessions()
        self.refresh_video_recordings()
        self.save_session()

        self.get_angle_client = self.create_client(GetAngle, self.get_angle_service_name)
        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.teach_joint_snapshot_timer = self.create_timer(
            0.5, self.try_pending_teach_joint_snapshot)
        self.request_teach_joint_snapshot("node start")
        self.create_camera_exposure_clients()
        self.camera_exposure_timer = self.create_timer(
            0.2, self.apply_pending_camera_exposure_settings)

        window_flags = cv2.WINDOW_NORMAL | getattr(cv2, "WINDOW_GUI_NORMAL", 0)
        cv2.namedWindow(WINDOW_NAME, window_flags)
        cv2.resizeWindow(
            WINDOW_NAME,
            LEFT_PANEL_WIDTH + PREVIEW_CANVAS_WIDTH,
            VIDEO_TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT,
        )
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        self.timer = self.create_timer(1.0 / 20.0, self.update_view)
        self.get_logger().info(f"item_teach_yolo session: {self.session_dir}")

    def normalize_camera_control_service_root(self, value: str) -> str:
        root = str(value or BIN_CAMERA_CONTROL_SERVICE_ROOT).strip() or BIN_CAMERA_CONTROL_SERVICE_ROOT
        while len(root) > 1 and root.endswith("/"):
            root = root[:-1]
        if not root.startswith("/"):
            root = "/" + root
        return root

    def camera_control_service_name(self, leaf: str) -> str:
        return f"{self.camera_control_service_root}/{leaf}"

    def create_camera_exposure_clients(self) -> None:
        self.color_auto_exposure_client = self.create_client(
            SetBool, self.camera_control_service_name("set_color_auto_exposure"))
        self.color_exposure_client = self.create_client(
            SetInt32, self.camera_control_service_name("set_color_exposure"))
        self.depth_auto_exposure_client = self.create_client(
            SetBool, self.camera_control_service_name("set_depth_auto_exposure"))
        self.depth_exposure_client = self.create_client(
            SetInt32, self.camera_control_service_name("set_depth_exposure"))
        self.mark_camera_exposure_dirty()

    def exposure_mode_text(self, exposure_us: int) -> str:
        return "auto" if exposure_us <= 0 else f"{exposure_us} us"

    def mark_camera_exposure_dirty(self) -> None:
        self.camera_exposure_dirty = True

    def apply_camera_exposure_setting(self, label: str, exposure_us: int,
                                      auto_client, exposure_client,
                                      last_attr: str) -> bool:
        if getattr(self, last_attr) == exposure_us:
            return True
        if auto_client is None or not auto_client.service_is_ready():
            return False
        if exposure_us > 0 and (exposure_client is None or not exposure_client.service_is_ready()):
            return False

        auto_request = SetBool.Request()
        auto_request.data = exposure_us <= 0
        future = auto_client.call_async(auto_request)

        def on_auto_done(done_future, request_label=label):
            try:
                response = done_future.result()
                if response is None or not response.success:
                    message = response.message if response is not None else "no response"
                    self.get_logger().warn(f"{request_label} auto exposure request failed: {message}")
            except Exception as exc:
                self.get_logger().warn(f"{request_label} auto exposure request error: {exc}")

        future.add_done_callback(on_auto_done)

        if exposure_us > 0:
            exposure_request = SetInt32.Request()
            exposure_request.data = exposure_us
            exposure_future = exposure_client.call_async(exposure_request)

            def on_exposure_done(done_future, request_label=label, request_exposure=exposure_us):
                try:
                    response = done_future.result()
                    if response is None or not response.success:
                        message = response.message if response is not None else "no response"
                        self.get_logger().warn(
                            f"{request_label} exposure {request_exposure} us request failed: {message}")
                except Exception as exc:
                    self.get_logger().warn(f"{request_label} exposure request error: {exc}")

            exposure_future.add_done_callback(on_exposure_done)

        setattr(self, last_attr, exposure_us)
        self.get_logger().info(f"{label} exposure set to {self.exposure_mode_text(exposure_us)}")
        return True

    def apply_pending_camera_exposure_settings(self) -> None:
        if not self.camera_exposure_dirty:
            return
        now = self.get_clock().now()
        if (now - self.last_camera_exposure_attempt_time).nanoseconds < 500_000_000:
            return
        self.last_camera_exposure_attempt_time = now

        color_ok = self.apply_camera_exposure_setting(
            "RGB",
            self.color_exposure_us,
            self.color_auto_exposure_client,
            self.color_exposure_client,
            "last_applied_color_exposure_us")
        self.depth_exposure_us = 0
        depth_ok = self.apply_camera_exposure_setting(
            "Depth",
            self.depth_exposure_us,
            self.depth_auto_exposure_client,
            self.depth_exposure_client,
            "last_applied_depth_exposure_us")
        self.camera_exposure_dirty = not (color_ok and depth_ok)

    def load_runtime_settings(self) -> None:
        if not self.runtime_settings_path.exists():
            return
        try:
            root = yaml.safe_load(self.runtime_settings_path.read_text(encoding="utf-8")) or {}
            params = root.get("item_teach_yolo_runtime", {}).get("ros__parameters", {})
            if not isinstance(params, dict):
                return
            self.color_exposure_min_us = max(
                1,
                min(
                    clamp_exposure_usec(
                        int(params.get("color_exposure_min_us", self.color_exposure_min_us))),
                    TEACH_COLOR_EXPOSURE_MAX_US))
            self.color_exposure_max_us = TEACH_COLOR_EXPOSURE_MAX_US
            self.depth_exposure_min_us = clamp_exposure_usec(
                int(params.get("depth_exposure_min_us", self.depth_exposure_min_us)))
            self.depth_exposure_max_us = max(
                self.depth_exposure_min_us,
                clamp_exposure_usec(
                    int(params.get("depth_exposure_max_us", self.depth_exposure_max_us))))
            if "color_exposure_us" in params:
                self.color_exposure_us = clamp_exposure_usec_or_auto(
                    int(params["color_exposure_us"]),
                    self.color_exposure_min_us,
                    self.color_exposure_max_us)
            elif "color_exposure_percent" in params:
                self.color_exposure_us = exposure_percent_to_usec(
                    int(params["color_exposure_percent"]),
                    self.color_exposure_min_us,
                    self.color_exposure_max_us)
            if "train_use_gpu_if_available" in params:
                self.train_use_gpu_if_available = as_bool(params["train_use_gpu_if_available"])
            self.depth_exposure_us = 0
            self.mark_camera_exposure_dirty()
        except Exception as exc:
            self.get_logger().warn(f"YOLO teach runtime settings load failed: {exc}")

    def save_runtime_settings(self) -> None:
        try:
            self.runtime_settings_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "item_teach_yolo_runtime": {
                    "ros__parameters": {
                        "camera_control_service_root": self.camera_control_service_root,
                        "color_exposure_us": self.color_exposure_us,
                        "depth_exposure_us": 0,
                        "color_exposure_percent": exposure_usec_to_percent(
                            self.color_exposure_us,
                            self.color_exposure_min_us,
                            self.color_exposure_max_us),
                        "depth_exposure_percent": 0,
                        "color_exposure_min_us": self.color_exposure_min_us,
                        "color_exposure_max_us": self.color_exposure_max_us,
                        "depth_exposure_min_us": self.depth_exposure_min_us,
                        "depth_exposure_max_us": self.depth_exposure_max_us,
                        "train_use_gpu_if_available": self.train_use_gpu_if_available,
                    }
                }
            }
            tmp_path = self.runtime_settings_path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            tmp_path.replace(self.runtime_settings_path)
        except Exception as exc:
            self.get_logger().warn(f"YOLO teach runtime settings save failed: {exc}")

    def cuda_training_available(self) -> bool:
        try:
            return bool(torch.cuda.is_available() and torch.cuda.device_count() > 0)
        except Exception:
            return False

    def cuda_training_device_name(self) -> str:
        if not self.cuda_training_available():
            return ""
        try:
            return str(torch.cuda.get_device_name(0))
        except Exception:
            return "CUDA device"

    def configured_gpu_train_device(self) -> str:
        token = str(self.train_device).strip()
        if not token or token.lower() in ("auto", "gpu", "cuda", "cuda:0", "cpu"):
            return "0"
        return token

    def effective_train_device(self) -> str:
        if self.train_use_gpu_if_available and self.cuda_training_available():
            return self.configured_gpu_train_device()
        return "cpu"

    def training_device_label(self, device: Optional[str] = None) -> str:
        selected = str(device if device is not None else self.effective_train_device()).strip()
        if selected.lower() == "cpu":
            return "CPU"
        device_name = self.cuda_training_device_name()
        return f"GPU {selected}: {device_name}" if device_name else f"GPU {selected}"

    def gpu_training_button_label(self) -> str:
        if not self.train_use_gpu_if_available:
            return "GPU Training: OFF (CPU)"
        device_name = self.cuda_training_device_name()
        if device_name:
            return f"GPU Training: ON ({fit_text(device_name, 22)})"
        return "GPU Training: ON (CUDA unavailable)"

    def toggle_gpu_training(self) -> None:
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot change GPU setting while training"
            return
        self.train_use_gpu_if_available = not self.train_use_gpu_if_available
        if self.train_use_gpu_if_available:
            device_name = self.cuda_training_device_name()
            self.status = (
                f"GPU training enabled: {device_name}"
                if device_name else
                "GPU training enabled, but CUDA is unavailable; training will use CPU"
            )
        else:
            self.status = "GPU training disabled; training will use CPU"
        self.save_runtime_settings()
        self.save_session()

    def create_session_dir(self) -> Path:
        self.runtime_root.mkdir(parents=True, exist_ok=True)
        name = f"{safe_name(self.item_name, fallback='unnamed_item')}_{timestamp_for_path()}"
        path = self.runtime_root / name
        suffix = 1
        while path.exists():
            path = self.runtime_root / f"{name}_{suffix}"
            suffix += 1
        path.mkdir(parents=True, exist_ok=True)
        return path

    def runtime_root_is_safe_to_clear(self, path: Path) -> bool:
        workspace = workspace_root().resolve()
        config_root = (workspace / "config").resolve()
        if path == workspace or path == config_root or path == Path(path.anchor):
            return False
        try:
            path.relative_to(config_root)
            return True
        except ValueError:
            pass
        return path.name in ("runtime", "item_teach_yolo_runtime") and len(path.parts) >= 3

    def clear_runtime_root(self) -> None:
        try:
            self.runtime_root.mkdir(parents=True, exist_ok=True)
            resolved_root = self.runtime_root.resolve()
            if not self.runtime_root_is_safe_to_clear(resolved_root):
                self.get_logger().warn(
                    f"Skipping runtime cleanup for unsafe runtime_root={resolved_root}")
                return
            for child in resolved_root.iterdir():
                if child.is_dir():
                    shutil.rmtree(child)
                else:
                    child.unlink()
        except Exception as exc:
            self.get_logger().warn(f"Could not clear YOLO teach runtime folder {self.runtime_root}: {exc}")

    def configure_session_storage(self) -> None:
        self.dataset_dir = self.session_dir / "dataset"
        self.images_dir = self.dataset_dir / "images" / "train"
        self.labels_dir = self.dataset_dir / "labels" / "train"
        self.masks_dir = self.session_dir / "masks"
        self.previews_dir = self.session_dir / "previews"
        self.prompts_dir = self.session_dir / "prompts"
        self.models_dir = self.session_dir / "models"
        self.videos_dir = self.session_dir / "videos"
        for path in [
            self.images_dir,
            self.labels_dir,
            self.masks_dir,
            self.previews_dir,
            self.prompts_dir,
            self.models_dir,
            self.videos_dir,
        ]:
            path.mkdir(parents=True, exist_ok=True)

        self.dataset_yaml_path = self.session_dir / "dataset.yaml"
        self.session_yaml_path = self.session_dir / "session.yaml"

    def remove_runtime_dir(self, path: Path) -> None:
        try:
            resolved_path = path.resolve()
            resolved_root = self.runtime_root.resolve()
            if resolved_path == resolved_root:
                return
            resolved_path.relative_to(resolved_root)
            if resolved_path.exists():
                shutil.rmtree(resolved_path)
        except Exception as exc:
            self.get_logger().warn(f"Could not clear old runtime folder {path}: {exc}")

    def reset_runtime_for_item_name(self, new_name: str) -> None:
        self.clear_prompts(save=False)
        self.item_name = new_name
        self.item_name_edit_buffer = new_name
        self.sample_count = 0
        self.background_sample_count = 0
        self.sample_history.clear()
        self.training_status = "idle"
        self.training_epoch_current = 0
        self.training_epoch_total = max(1, self.train_epochs)
        self.training_progress = 0.0
        self.trained_model_path = ""
        self.trained_onnx_path = ""
        self.final_model_dir = ""
        self.latest_profile_path = ""
        self.reset_video_review_state(clear_prompts=False)
        self.session_dir = self.create_session_dir()
        self.configure_session_storage()
        self.refresh_video_recordings()
        self.write_dataset_yaml()
        self.status = f"Item name changed to {self.item_name}; new session created"
        self.save_session()
        self.request_teach_joint_snapshot("new item teach")

    def write_dataset_yaml(self) -> None:
        class_name = self.item_name.strip() if self.has_item_name() else "unnamed_item"
        data = {
            "path": str(self.dataset_dir),
            "train": "images/train",
            "val": "images/train",
            "names": {0: class_name},
        }
        self.dataset_yaml_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    def save_session(self) -> None:
        active_bin = self.active_bin
        review_recording_path = (
            self.video_recordings[self.review_recording_index].path
            if 0 <= self.review_recording_index < len(self.video_recordings) else None
        )
        review_frame = self.current_review_frame_record()
        review_frame_status = str(review_frame.get("status", "")) if review_frame else ""
        review_frame_sample_stems = self.review_frame_sample_stems(review_frame)
        review_recording_entry = (
            self.video_recordings[self.review_recording_index]
            if 0 <= self.review_recording_index < len(self.video_recordings) else None
        )
        data = {
            "item_teach_yolo_session": {
                "item_name": self.item_name,
                "session_dir": str(self.session_dir),
                "dataset_yaml": str(self.dataset_yaml_path),
                "sample_count": self.sample_count,
                "background_sample_count": self.background_sample_count,
                "total_training_image_count": self.total_training_image_count(),
                "background_ratio_percent": self.background_ratio_percent(),
                "sample_history": list(self.sample_history),
                "training_status": self.training_status,
                "training_epoch_current": self.training_epoch_current,
                "training_epoch_total": self.training_epoch_total,
                "training_progress": round(float(self.training_progress), 4),
                "trained_model_path": self.trained_model_path,
                "trained_onnx_path": self.trained_onnx_path,
                "train_device": self.train_device,
                "train_use_gpu_if_available": self.train_use_gpu_if_available,
                "train_device_used": self.train_device_used,
                "final_model_dir": self.final_model_dir,
                "latest_profile_path": self.latest_profile_path,
                "record_fps": self.record_fps,
                "video_recording_count": len(self.video_recordings),
                "recording_active": self.recording_active,
                "review_mode": self.review_mode,
                "review_recording": str(review_recording_path) if review_recording_path else "",
                "review_recording_name": review_recording_path.name if review_recording_path else "",
                "review_frame_index": self.review_frame_index,
                "review_frame_status": review_frame_status,
                "review_frame_sample_stems": review_frame_sample_stems,
                "review_recording_frame_count": review_recording_entry.frame_count if review_recording_entry else 0,
                "review_recording_annotated_count": (
                    review_recording_entry.annotated_count if review_recording_entry else 0
                ),
                "review_recording_skipped_count": review_recording_entry.skipped_count if review_recording_entry else 0,
                "active_bin_name": active_bin.bin_name if active_bin else "",
                "active_bin_file": str(active_bin.path) if active_bin else "",
                "roi_crop_rect": list(self.roi_crop_rect) if self.roi_crop_rect else [],
                "camera_control_service_root": self.camera_control_service_root,
                "color_exposure_us": self.color_exposure_us,
                "depth_exposure_us": 0,
                "color_exposure_percent": exposure_usec_to_percent(
                    self.color_exposure_us,
                    self.color_exposure_min_us,
                    self.color_exposure_max_us),
                "depth_exposure_percent": 0,
                "color_exposure_min_us": self.color_exposure_min_us,
                "color_exposure_max_us": self.color_exposure_max_us,
                "depth_exposure_min_us": self.depth_exposure_min_us,
                "depth_exposure_max_us": self.depth_exposure_max_us,
                "motion_service_root": self.motion_service_root,
                "get_angle_service": self.get_angle_service_name,
                "teach_joints_deg": self.latest_joint_positions_deg if self.has_joint_positions else [],
                "teach_joints_source": self.teach_joints_source,
                "positive_prompt_count": len(self.positive_points),
                "negative_prompt_count": len(self.negative_points),
            }
        }
        tmp_path = self.session_yaml_path.with_suffix(".tmp")
        tmp_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        tmp_path.replace(self.session_yaml_path)

    def request_teach_joint_snapshot(
        self,
        reason: str,
        save_on_success: bool = True,
        update_profile_on_success: bool = True,
    ) -> None:
        self.pending_teach_joint_snapshot_reason = reason or "teach"
        self.pending_teach_joint_snapshot_save = (
            self.pending_teach_joint_snapshot_save or save_on_success)
        self.pending_teach_joint_snapshot_update_profile = (
            self.pending_teach_joint_snapshot_update_profile or update_profile_on_success)
        self.try_pending_teach_joint_snapshot()

    def try_pending_teach_joint_snapshot(self) -> None:
        if not self.pending_teach_joint_snapshot_reason or self.get_angle_request_in_flight:
            return
        if not hasattr(self, "get_angle_client") or self.get_angle_client is None:
            return
        if not self.get_angle_client.service_is_ready():
            if not self.get_angle_warning_logged:
                self.get_logger().warn(
                    f"Robot joint snapshot service not available: {self.get_angle_service_name}")
                self.get_angle_warning_logged = True
            return

        reason = self.pending_teach_joint_snapshot_reason
        save_on_success = self.pending_teach_joint_snapshot_save
        update_profile_on_success = self.pending_teach_joint_snapshot_update_profile
        self.pending_teach_joint_snapshot_reason = ""
        self.pending_teach_joint_snapshot_save = False
        self.pending_teach_joint_snapshot_update_profile = False
        self.get_angle_request_in_flight = True
        future = self.get_angle_client.call_async(GetAngle.Request())
        future.add_done_callback(
            lambda done_future: self.handle_teach_joint_snapshot_response(
                done_future,
                reason,
                save_on_success,
                update_profile_on_success,
            )
        )

    def requeue_teach_joint_snapshot(
        self,
        reason: str,
        save_on_success: bool,
        update_profile_on_success: bool,
    ) -> None:
        self.pending_teach_joint_snapshot_reason = reason or "teach"
        self.pending_teach_joint_snapshot_save = (
            self.pending_teach_joint_snapshot_save or save_on_success)
        self.pending_teach_joint_snapshot_update_profile = (
            self.pending_teach_joint_snapshot_update_profile or update_profile_on_success)

    def handle_teach_joint_snapshot_response(
        self,
        future,
        reason: str,
        save_on_success: bool,
        update_profile_on_success: bool,
    ) -> None:
        self.get_angle_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            if not self.get_angle_warning_logged:
                self.get_logger().warn(f"GetAngle call failed while saving teach joints: {exc}")
                self.get_angle_warning_logged = True
            self.requeue_teach_joint_snapshot(reason, save_on_success, update_profile_on_success)
            return

        if response is None or int(getattr(response, "res", -1)) < 0:
            if not self.get_angle_warning_logged:
                self.get_logger().warn(
                    "GetAngle failed while saving teach joints: "
                    f"res={getattr(response, 'res', None)}, "
                    f"return={getattr(response, 'robot_return', '')}")
                self.get_angle_warning_logged = True
            self.requeue_teach_joint_snapshot(reason, save_on_success, update_profile_on_success)
            return

        joints_deg = self.parse_six_values_from_robot_return(
            getattr(response, "robot_return", ""))
        if joints_deg is None:
            if not self.get_angle_warning_logged:
                self.get_logger().warn(
                    "Could not parse GetAngle reply while saving teach joints: "
                    f"{getattr(response, 'robot_return', '')}")
                self.get_angle_warning_logged = True
            self.requeue_teach_joint_snapshot(reason, save_on_success, update_profile_on_success)
            return

        self.latest_joint_positions_deg = [float(value) for value in joints_deg]
        self.has_joint_positions = True
        self.teach_joints_source = "GetAngle"
        self.get_angle_warning_logged = False
        self.get_logger().info(
            f"Saved YOLO teach joints from {self.get_angle_service_name} ({reason}): "
            f"[{', '.join(f'{value:.3f}' for value in self.latest_joint_positions_deg)}]")

        if save_on_success:
            self.save_session()
        if update_profile_on_success and self.final_model_dir:
            try:
                self.write_profile()
            except Exception as exc:
                self.get_logger().warn(f"Could not update YOLO teach profile joints: {exc}")
        if reason != "node start":
            self.status = f"Teach position saved from robot: {reason}"

    @staticmethod
    def parse_six_values_from_robot_return(
        robot_return: object,
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        text = str(robot_return or "")
        if not text:
            return None
        float_pattern = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
        for content in re.findall(r"\{([^{}]+)\}", text):
            values = [float(token) for token in re.findall(float_pattern, content)]
            if len(values) == 6:
                return tuple(values)  # type: ignore[return-value]
        values = [float(token) for token in re.findall(float_pattern, text)]
        if len(values) == 6:
            return tuple(values)  # type: ignore[return-value]
        return None

    def reset_video_review_state(self, clear_prompts: bool = True) -> None:
        self.review_mode = False
        self.review_recording_index = -1
        self.review_recording_meta = {}
        self.review_frame_index = 0
        if clear_prompts:
            self.clear_prompts(save=False)

    def recording_metadata_path(self, recording_path: Path) -> Path:
        if recording_path.is_dir():
            return recording_path / "recording.yaml"
        if recording_path.suffix.lower() in (".yaml", ".yml"):
            return recording_path
        return recording_path.with_suffix(".yaml")

    def recording_root_for_relative_files(self, recording_path: Path) -> Path:
        return recording_path if recording_path.is_dir() else recording_path.parent

    def default_recording_video_path(self, recording_path: Path) -> Path:
        if recording_path.is_dir():
            return recording_path / "roi_recording.avi"
        if recording_path.suffix.lower() in VIDEO_FILE_SUFFIXES:
            return recording_path
        return recording_path.with_suffix(".avi")

    def default_recording_image_path(self, recording_path: Path) -> Path:
        if recording_path.suffix.lower() in IMAGE_FILE_SUFFIXES:
            return recording_path
        return recording_path.with_suffix(".png")

    def raw_capture_image_path(self, recording_path: Path) -> Path:
        base_path = recording_path.with_suffix("")
        return base_path.with_name(f"{base_path.name}{RAW_CAPTURE_SUFFIX}").with_suffix(".png")

    def is_raw_capture_image(self, path: Path) -> bool:
        return path.suffix.lower() in IMAGE_FILE_SUFFIXES and path.with_suffix("").name.endswith(RAW_CAPTURE_SUFFIX)

    def capture_metadata_path_for_image(self, image_path: Path) -> Path:
        direct_metadata_path = image_path.with_suffix(".yaml")
        if direct_metadata_path.exists() or not self.is_raw_capture_image(image_path):
            return direct_metadata_path
        stem = image_path.with_suffix("").name
        base_stem = stem[:-len(RAW_CAPTURE_SUFFIX)]
        return image_path.with_name(base_stem).with_suffix(".yaml")

    def review_frame_uses_roi_rect(self, frame_view: str) -> bool:
        return frame_view in {"full_roi_masked", "full_raw", "full_camera_raw"}

    def recording_is_roi_image_capture(self, recording_path: Path, recording: Dict) -> bool:
        if not recording_path.name.startswith("roi_"):
            return False
        if not str(recording.get("image_file", "")).strip():
            return False
        frame_source = str(recording.get("frame_source", "")).strip()
        if frame_source in {"selected_image", "image_folder"}:
            return False
        if str(recording.get("raw_image_file", "")).strip():
            return True
        return frame_source in {
            "raw_camera_full_roi_masked",
            "raw_camera_full_roi_pair",
            "raw_camera_roi_crop_pair",
        }

    def recording_image_path_from_meta(self, recording_path: Path, recording: Dict) -> Optional[Path]:
        image_text = str(recording.get("image_file", "")).strip()
        if not image_text:
            default_path = self.default_recording_image_path(recording_path)
            return default_path if default_path.exists() else None
        image_path = Path(image_text)
        if not image_path.is_absolute():
            image_path = self.recording_root_for_relative_files(recording_path) / image_path
        return image_path

    def recording_video_path_from_meta(self, recording_path: Path, recording: Dict) -> Optional[Path]:
        video_text = str(recording.get("video_file", "")).strip()
        if not video_text:
            default_path = self.default_recording_video_path(recording_path)
            return default_path if default_path.exists() else None
        if video_text == "roi_recording.avi" and not recording_path.is_dir():
            default_path = self.default_recording_video_path(recording_path)
            if default_path.exists():
                return default_path
        video_path = Path(video_text)
        if not video_path.is_absolute():
            video_path = self.recording_root_for_relative_files(recording_path) / video_path
        return video_path

    def recorded_video_frame_count(self, recording_path: Path, recording: Dict) -> int:
        video_path = self.recording_video_path_from_meta(recording_path, recording)
        if video_path is None or not video_path.exists():
            return 0
        cap = cv2.VideoCapture(str(video_path))
        try:
            if not cap.isOpened():
                return 0
            return max(0, int(round(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)))
        finally:
            cap.release()

    def normalize_recording_metadata(self, recording_path: Path, root: Dict) -> Dict:
        recording = root.setdefault("recording", {})
        frames = recording.get("frames", [])
        if not isinstance(frames, list):
            frames = []

        image_path = self.recording_image_path_from_meta(recording_path, recording)
        try:
            frame_count = int(recording.get("frame_count", len(frames)))
        except (TypeError, ValueError):
            frame_count = len(frames)
        if not frames and image_path is not None and image_path.exists():
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            height, width = image.shape[:2] if image is not None else (0, 0)
            frames = [{
                "index": 1,
                "file": image_path.name,
                "frame_view": "full_roi_masked",
                "width": int(width),
                "height": int(height),
                "status": "pending",
                "roi_crop_rect": list(recording.get("roi_crop_rect", [])),
            }]
            frame_count = 1
        if frame_count <= 0:
            frame_count = len(frames) or self.recorded_video_frame_count(recording_path, recording)

        if not frames and frame_count > 0:
            frames = [
                {
                    "index": index,
                    "video_frame": index,
                    "status": "pending",
                }
                for index in range(1, frame_count + 1)
            ]

        recording["frames"] = frames
        recording["frame_count"] = max(frame_count, len(frames))
        if "name" not in recording or not str(recording.get("name", "")).strip():
            recording["name"] = recording_path.name
        if "video_file" not in recording or not str(recording.get("video_file", "")).strip():
            default_path = self.default_recording_video_path(recording_path)
            if default_path.exists():
                recording["video_file"] = (
                    default_path.name
                    if default_path.parent == self.recording_root_for_relative_files(recording_path)
                    else str(default_path)
                )
        if "image_file" not in recording or not str(recording.get("image_file", "")).strip():
            default_path = self.default_recording_image_path(recording_path)
            if default_path.exists():
                recording["image_file"] = (
                    default_path.name
                    if default_path.parent == self.recording_root_for_relative_files(recording_path)
                    else str(default_path)
                )
        return root

    def write_recording_metadata(self) -> None:
        if self.recording_dir is None or not self.recording_metadata:
            return
        try:
            path = self.recording_metadata_path(self.recording_dir)
            tmp_path = path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(self.recording_metadata, sort_keys=False), encoding="utf-8")
            tmp_path.replace(path)
        except Exception as exc:
            self.get_logger().warn(f"Could not save ROI recording metadata: {exc}")

    def read_recording_metadata(self, recording_path: Path) -> Dict:
        metadata_path = self.recording_metadata_path(recording_path)
        if metadata_path.exists():
            root = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
            if isinstance(root, dict) and isinstance(root.get("recording"), dict):
                return self.normalize_recording_metadata(recording_path, root)

        frames_dir = recording_path / "frames" if recording_path.is_dir() else None
        frames = []
        if frames_dir is not None and frames_dir.exists():
            for index, frame_path in enumerate(sorted(frames_dir.glob("frame_*.png")), start=1):
                frames.append({
                    "index": index,
                    "file": str(frame_path.relative_to(self.recording_root_for_relative_files(recording_path))),
                    "status": "pending",
                })
        default_video_path = self.default_recording_video_path(recording_path)
        default_image_path = self.default_recording_image_path(recording_path)
        if default_image_path.exists():
            recording = {
                "name": recording_path.with_suffix("").name,
                "created_at": "",
                "ended_at": "",
                "bin_name": "",
                "bin_file": "",
                "image_file": default_image_path.name,
                "frame_source": "raw_camera_full_roi_masked",
                "mask_mode": "outside_roi_black",
                "frame_count": 1,
                "frames": [],
            }
            return self.normalize_recording_metadata(recording_path, {"recording": recording})

        recording = {
            "name": recording_path.name,
            "created_at": "",
            "ended_at": "",
            "bin_name": "",
            "bin_file": "",
            "fps": self.record_fps,
            "video_file": default_video_path.name,
            "frame_count": len(frames),
            "frames": frames,
        }
        if not frames:
            frame_count = self.recorded_video_frame_count(recording_path, recording)
            recording["frame_count"] = frame_count
        return self.normalize_recording_metadata(recording_path, {"recording": recording})

    def recording_refs_in_videos_dir(self) -> List[Path]:
        refs: Dict[str, Path] = {}
        try:
            self.videos_dir.mkdir(parents=True, exist_ok=True)
            for path in self.videos_dir.iterdir():
                if path.is_dir():
                    refs[str(path.resolve())] = path
                    continue
                suffix = path.suffix.lower()
                if suffix in (".yaml", ".yml"):
                    stem = path.with_suffix("")
                    refs[str(stem.resolve())] = stem
                elif suffix in VIDEO_FILE_SUFFIXES:
                    metadata_path = path.with_suffix(".yaml")
                    ref = path.with_suffix("") if metadata_path.exists() else path
                    refs[str(ref.resolve())] = ref
                elif suffix in IMAGE_FILE_SUFFIXES:
                    if self.is_raw_capture_image(path):
                        continue
                    refs[str(path.with_suffix("").resolve())] = path.with_suffix("")
        except Exception as exc:
            self.get_logger().warn(f"Could not list ROI recordings: {exc}")
        return list(refs.values())

    def refresh_video_recordings(self) -> None:
        entries: List[VideoRecordingEntry] = []
        capture_count = 0
        try:
            for path in self.recording_refs_in_videos_dir():
                try:
                    root = self.read_recording_metadata(path)
                    recording = root.get("recording", {})
                    frames = recording.get("frames", [])
                    if not isinstance(frames, list):
                        frames = []
                    frame_count = int(recording.get("frame_count", len(frames)))
                    annotated_count = sum(
                        1 for frame in frames
                        if isinstance(frame, dict) and frame.get("status") == "annotated"
                    )
                    skipped_count = sum(
                        1 for frame in frames
                        if isinstance(frame, dict) and frame.get("status") == "skipped"
                    )
                    metadata_path = self.recording_metadata_path(path)
                    video_path = self.recording_video_path_from_meta(path, recording)
                    image_path = self.recording_image_path_from_meta(path, recording)
                    modified_time = (
                        metadata_path.stat().st_mtime if metadata_path.exists() else path.stat().st_mtime
                        if path.exists() else video_path.stat().st_mtime
                        if video_path is not None and video_path.exists() else image_path.stat().st_mtime
                        if image_path is not None and image_path.exists() else time.time()
                    )
                    label = (
                        f"{path.name} | {frame_count} frames | "
                        f"{annotated_count} saved {skipped_count} skipped"
                    )
                    if frame_count > 0:
                        if self.recording_is_roi_image_capture(path, recording):
                            capture_count += 1
                        entries.append(VideoRecordingEntry(
                            label=label,
                            path=path,
                            frame_count=frame_count,
                            annotated_count=annotated_count,
                            skipped_count=skipped_count,
                            modified_time=modified_time,
                        ))
                except Exception as exc:
                    self.get_logger().warn(f"Could not read ROI recording {path}: {exc}")
        except Exception as exc:
            self.get_logger().warn(f"Could not list ROI recordings: {exc}")

        current_path: Optional[Path] = None
        if 0 <= self.review_recording_index < len(self.video_recordings):
            current_path = self.video_recordings[self.review_recording_index].path
        self.video_recordings = sorted(entries, key=lambda entry: entry.modified_time, reverse=True)
        self.roi_image_capture_count = capture_count
        if current_path is not None:
            self.review_recording_index = next(
                (
                    index for index, entry in enumerate(self.video_recordings)
                    if entry.path == current_path
                ),
                -1,
            )

    def recording_index_for_session_ref(self, recording_text: str, recording_name: str = "") -> int:
        reference_path: Optional[Path] = None
        if recording_text:
            try:
                reference_path = resolve_path(recording_text)
            except Exception:
                reference_path = Path(recording_text)
        recording_name = recording_name.strip()
        for index, entry in enumerate(self.video_recordings):
            if reference_path is not None:
                try:
                    if entry.path.resolve() == reference_path.resolve():
                        return index
                except Exception:
                    pass
                if entry.path.name == reference_path.name:
                    return index
            if recording_name and entry.path.name == recording_name:
                return index
        return -1

    def review_resume_recording_index(self) -> int:
        in_progress_pending: List[Tuple[float, int]] = []
        multi_frame_pending: List[Tuple[float, int]] = []
        pending_only: List[Tuple[float, int]] = []
        completed_with_progress: List[Tuple[float, int]] = []
        multi_frame_completed: List[Tuple[float, int]] = []
        completed_without_progress: List[Tuple[float, int]] = []
        for index, entry in enumerate(self.video_recordings):
            try:
                root = self.read_recording_metadata(entry.path)
                recording = root.get("recording", {})
                frames = recording.get("frames", []) if isinstance(recording, dict) else []
                if not isinstance(frames, list) or not frames:
                    continue
                has_pending = any(
                    isinstance(frame, dict) and frame.get("status", "pending") == "pending"
                    for frame in frames
                )
                has_progress = entry.annotated_count > 0 or entry.skipped_count > 0
                is_multi_frame = entry.frame_count > 1
                if has_pending:
                    if has_progress:
                        in_progress_pending.append((entry.modified_time, index))
                    elif is_multi_frame:
                        multi_frame_pending.append((entry.modified_time, index))
                    else:
                        pending_only.append((entry.modified_time, index))
                elif has_progress:
                    completed_with_progress.append((entry.modified_time, index))
                elif is_multi_frame:
                    multi_frame_completed.append((entry.modified_time, index))
                else:
                    completed_without_progress.append((entry.modified_time, index))
            except Exception as exc:
                self.get_logger().warn(f"Could not inspect ROI review state {entry.path}: {exc}")
        if in_progress_pending:
            return max(in_progress_pending, key=lambda item: item[0])[1]
        if completed_with_progress:
            return max(completed_with_progress, key=lambda item: item[0])[1]
        if multi_frame_pending:
            return max(multi_frame_pending, key=lambda item: item[0])[1]
        if multi_frame_completed:
            return max(multi_frame_completed, key=lambda item: item[0])[1]
        if pending_only:
            return min(pending_only, key=lambda item: item[0])[1]
        if completed_without_progress:
            return max(completed_without_progress, key=lambda item: item[0])[1]
        return 0 if self.video_recordings else -1

    def restore_video_review_from_session(self, params: Dict) -> bool:
        review_requested = as_bool(params.get("review_mode", False))
        recording_text = str(params.get("review_recording", "")).strip()
        recording_name = str(params.get("review_recording_name", "")).strip()
        if not review_requested and not recording_text and not recording_name:
            return False
        index = self.recording_index_for_session_ref(recording_text, recording_name)
        if index < 0:
            return False
        try:
            frame_index = max(0, int(params.get("review_frame_index", 0)))
        except (TypeError, ValueError):
            frame_index = 0
        self.enter_video_review(index, frame_index=frame_index)
        return self.review_mode

    def resume_existing_video_review(self) -> bool:
        self.refresh_video_recordings()
        if not self.video_recordings:
            return False
        index = self.review_resume_recording_index()
        if index < 0:
            return False
        entry = self.video_recordings[index]
        if entry.frame_count <= 1 and entry.annotated_count <= 0 and entry.skipped_count <= 0:
            return False
        self.enter_video_review(index)
        return self.review_mode

    def select_review_image_path(self) -> Optional[Path]:
        initial_dir = self.videos_dir if self.videos_dir.exists() else self.session_dir
        try:
            import tkinter as tk
            from tkinter import filedialog

            root = tk.Tk()
            root.withdraw()
            try:
                root.attributes("-topmost", True)
            except tk.TclError:
                pass
            selected = filedialog.askopenfilename(
                title="Select ROI image to review",
                initialdir=str(initial_dir),
                filetypes=[
                    ("Image files", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff"),
                    ("Legacy video files", "*.avi *.mp4 *.mov *.mkv *.mpeg *.mpg *.m4v"),
                    ("All files", "*.*"),
                ],
            )
            root.destroy()
            return Path(selected).expanduser().resolve() if selected else None
        except Exception as exc:
            self.get_logger().warn(f"Tk image file picker unavailable: {exc}")

        for command in ("zenity", "kdialog"):
            if shutil.which(command) is None:
                continue
            try:
                if command == "zenity":
                    result = subprocess.run(
                        [
                            "zenity",
                            "--file-selection",
                            "--title=Select ROI image to review",
                            f"--filename={str(initial_dir)}/",
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                else:
                    result = subprocess.run(
                        [
                            "kdialog",
                            "--getopenfilename",
                            str(initial_dir),
                            "*.png *.jpg *.jpeg *.bmp *.tif *.tiff|Image files",
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                selected = result.stdout.strip()
                if result.returncode == 0 and selected:
                    return Path(selected).expanduser().resolve()
                return None
            except Exception as exc:
                self.get_logger().warn(f"{command} image file picker failed: {exc}")
        self.status = "Could not open image file picker"
        return None

    def select_review_image_folder(self) -> Optional[Path]:
        initial_dir = self.videos_dir if self.videos_dir.exists() else self.session_dir
        try:
            import tkinter as tk
            from tkinter import filedialog

            root = tk.Tk()
            root.withdraw()
            try:
                root.attributes("-topmost", True)
            except tk.TclError:
                pass
            selected = filedialog.askdirectory(
                title="Select ROI image folder to review",
                initialdir=str(initial_dir),
                mustexist=True,
            )
            root.destroy()
            return Path(selected).expanduser().resolve() if selected else None
        except Exception as exc:
            self.get_logger().warn(f"Tk image folder picker unavailable: {exc}")

        for command in ("zenity", "kdialog"):
            if shutil.which(command) is None:
                continue
            try:
                if command == "zenity":
                    result = subprocess.run(
                        [
                            "zenity",
                            "--file-selection",
                            "--directory",
                            "--title=Select ROI image folder to review",
                            f"--filename={str(initial_dir)}/",
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                else:
                    result = subprocess.run(
                        [
                            "kdialog",
                            "--getexistingdirectory",
                            str(initial_dir),
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                selected = result.stdout.strip()
                if result.returncode == 0 and selected:
                    return Path(selected).expanduser().resolve()
                return None
            except Exception as exc:
                self.get_logger().warn(f"{command} image folder picker failed: {exc}")
        self.status = "Could not open image folder picker"
        return None

    def image_frame_record_from_file(self, image_path: Path, index: int) -> Optional[Dict]:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            return None
        h, w = image.shape[:2]
        is_raw_capture = self.is_raw_capture_image(image_path)
        frame_record: Dict = {
            "index": index,
            "file": str(image_path),
            "frame_view": "full_raw" if is_raw_capture else "full_roi_masked",
            "width": int(w),
            "height": int(h),
            "status": "pending",
            "roi_crop_rect": [],
        }
        metadata_path = self.capture_metadata_path_for_image(image_path)
        if metadata_path.exists():
            try:
                root = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
                recording = root.get("recording", {}) if isinstance(root, dict) else {}
                frames = recording.get("frames", []) if isinstance(recording, dict) else []
                if isinstance(frames, list) and frames and isinstance(frames[0], dict):
                    source_frame = frames[0]
                    frame_rect = source_frame.get("roi_crop_rect", [])
                    source_rect = source_frame.get("source_roi_crop_rect", recording.get("source_roi_crop_rect", []))
                    roi_rect = source_rect if is_raw_capture and isinstance(source_rect, list) else frame_rect
                    if isinstance(roi_rect, list):
                        frame_record["roi_crop_rect"] = list(roi_rect[:4])
                    frame_view = str(source_frame.get("frame_view", "")).strip()
                    if frame_view:
                        frame_record["frame_view"] = frame_view
                    if is_raw_capture:
                        frame_record["frame_view"] = "full_raw"
                    raw_file = str(source_frame.get("raw_file", "")).strip()
                    if raw_file:
                        raw_path = Path(raw_file)
                        if not raw_path.is_absolute():
                            raw_path = image_path.parent / raw_path
                        frame_record["raw_file"] = str(raw_path)
                    if isinstance(source_rect, list):
                        frame_record["source_roi_crop_rect"] = list(source_rect[:4])
                    for key in (
                        "status",
                        "updated_at",
                        "sample_stem",
                        "sample_stems",
                        "sample_count",
                    ):
                        if key in source_frame:
                            frame_record[key] = source_frame[key]
            except Exception as exc:
                self.get_logger().warn(f"Could not read ROI image metadata {metadata_path}: {exc}")
        return frame_record

    def import_review_folder(self, folder_path: Path) -> Optional[Path]:
        if not folder_path.exists() or not folder_path.is_dir():
            self.status = "Selected ROI image folder is missing"
            return None
        image_paths = [
            path for path in sorted(folder_path.iterdir(), key=lambda p: (p.name.lower(), p.name))
            if (
                path.is_file() and
                path.suffix.lower() in IMAGE_FILE_SUFFIXES and
                not self.is_raw_capture_image(path)
            )
        ]
        if not image_paths:
            self.status = "Selected folder has no ROI images"
            return None

        frames = []
        for image_path in image_paths:
            frame_record = self.image_frame_record_from_file(image_path, len(frames) + 1)
            if frame_record is not None:
                frames.append(frame_record)
        if not frames:
            self.status = "Selected folder has no readable ROI images"
            return None

        self.videos_dir.mkdir(parents=True, exist_ok=True)
        folder_token = safe_name(folder_path.name, fallback="images")
        review_path = self.videos_dir / f"review_{folder_token}_{timestamp_for_path()}"
        suffix = 1
        while review_path.with_suffix(".yaml").exists() or review_path.exists():
            review_path = self.videos_dir / f"review_{folder_token}_{timestamp_for_path()}_{suffix}"
            suffix += 1

        now_text = _dt.datetime.now().isoformat(timespec="seconds")
        root = {
            "recording": {
                "name": review_path.name,
                "created_at": now_text,
                "ended_at": now_text,
                "source_folder": str(folder_path),
                "frame_source": "image_folder",
                "mask_mode": "images_as_frames",
                "frame_count": len(frames),
                "frames": frames,
            }
        }
        try:
            metadata_path = self.recording_metadata_path(review_path)
            tmp_path = metadata_path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(root, sort_keys=False), encoding="utf-8")
            tmp_path.replace(metadata_path)
        except Exception as exc:
            self.status = f"Could not save review folder metadata: {exc}"
            return None
        return review_path

    def existing_recording_dir_for_media(self, media_path: Path) -> Optional[Path]:
        try:
            target = media_path.resolve()
            self.videos_dir.mkdir(parents=True, exist_ok=True)
            for candidate in self.recording_refs_in_videos_dir():
                try:
                    root = self.read_recording_metadata(candidate)
                    recording = root.get("recording", {})
                    resolved_video = self.recording_video_path_from_meta(candidate, recording)
                    resolved_image = self.recording_image_path_from_meta(candidate, recording)
                    if resolved_video is not None and resolved_video.resolve() == target:
                        return candidate
                    if resolved_image is not None and resolved_image.resolve() == target:
                        return candidate
                except Exception:
                    continue
        except Exception as exc:
            self.get_logger().warn(f"Could not match selected ROI media: {exc}")
        return None

    def import_review_media(self, media_path: Path) -> Optional[Path]:
        if not media_path.exists() or not media_path.is_file():
            self.status = "Selected ROI image file is missing"
            return None
        suffix = media_path.suffix.lower()
        is_image = suffix in IMAGE_FILE_SUFFIXES
        is_video = suffix in VIDEO_FILE_SUFFIXES
        if not is_image and not is_video:
            self.status = "Selected file is not a supported image"
            return None
        self.videos_dir.mkdir(parents=True, exist_ok=True)
        item_token = safe_name(self.item_name, fallback="item")
        media_token = safe_name(media_path.stem, fallback="image")
        name = f"selected_{item_token}_{media_token}_{timestamp_for_path()}"
        recording_path = self.videos_dir / name
        suffix = 1
        while (
            recording_path.exists()
            or recording_path.with_suffix(".yaml").exists()
            or recording_path.with_suffix(media_path.suffix or ".png").exists()
        ):
            recording_path = self.videos_dir / f"{name}_{suffix}"
            suffix += 1

        copied_media = False
        if media_path.parent.resolve() == self.videos_dir.resolve():
            review_media_path = media_path
            recording_path = media_path.with_suffix("")
        else:
            review_media_path = recording_path.with_suffix(media_path.suffix or ".png")
            shutil.copy2(media_path, review_media_path)
            copied_media = True

        if is_image:
            image = cv2.imread(str(review_media_path), cv2.IMREAD_COLOR)
            if image is None:
                if copied_media:
                    review_media_path.unlink(missing_ok=True)
                self.status = "Selected image could not be read"
                return None
            h, w = image.shape[:2]
            root = self.normalize_recording_metadata(
                recording_path,
                {
                    "recording": {
                        "name": recording_path.name,
                        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
                        "ended_at": "",
                        "bin_name": self.active_bin.bin_name if self.active_bin else "",
                        "bin_file": str(self.active_bin.path) if self.active_bin else "",
                        "image_file": review_media_path.name,
                        "source_image_file": str(media_path),
                        "frame_source": "selected_image",
                        "mask_mode": "image_as_frame",
                        "image_width": int(w),
                        "image_height": int(h),
                        "frame_count": 1,
                        "frames": [{
                            "index": 1,
                            "file": review_media_path.name,
                            "frame_view": "full_roi_masked",
                            "width": int(w),
                            "height": int(h),
                            "status": "pending",
                            "roi_crop_rect": [],
                        }],
                    }
                },
            )
        else:
            root = self.normalize_recording_metadata(
                recording_path,
                {
                    "recording": {
                        "name": recording_path.name,
                        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
                        "ended_at": "",
                        "bin_name": self.active_bin.bin_name if self.active_bin else "",
                        "bin_file": str(self.active_bin.path) if self.active_bin else "",
                        "fps": self.record_fps,
                        "video_file": review_media_path.name,
                        "source_video_file": str(media_path),
                        "frame_count": 0,
                        "frames": [],
                    }
                },
            )

        frame_count = int(root.get("recording", {}).get("frame_count", 0))
        if frame_count <= 0:
            if copied_media:
                review_media_path.unlink(missing_ok=True)
            self.status = "Selected file has no readable frames"
            return None
        try:
            path = self.recording_metadata_path(recording_path)
            tmp_path = path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(root, sort_keys=False), encoding="utf-8")
            tmp_path.replace(path)
        except Exception as exc:
            self.recording_metadata_path(recording_path).unlink(missing_ok=True)
            if copied_media:
                review_media_path.unlink(missing_ok=True)
            self.status = f"Could not save selected image metadata: {exc}"
            return None
        return recording_path

    def enter_video_review_from_file(self, media_path: Path) -> None:
        if self.recording_active:
            self.status = "Stop recording before reviewing"
            return
        media_path = media_path.expanduser().resolve()
        recording_dir = self.existing_recording_dir_for_media(media_path)
        if recording_dir is None:
            recording_dir = self.import_review_media(media_path)
        if recording_dir is None:
            return
        self.refresh_video_recordings()
        review_index = next(
            (
                index for index, entry in enumerate(self.video_recordings)
                if entry.path.resolve() == recording_dir.resolve()
            ),
            -1,
        )
        if review_index < 0:
            self.status = "Selected image could not be added to review list"
            return
        self.enter_video_review(review_index)

    def choose_video_review(self) -> None:
        if self.recording_active:
            self.status = "Stop recording before reviewing"
            return
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot review images while training"
            return
        if self.resume_existing_video_review():
            return
        if self.videos_dir.exists():
            has_session_images = any(
                path.is_file()
                and path.suffix.lower() in IMAGE_FILE_SUFFIXES
                and not self.is_raw_capture_image(path)
                for path in self.videos_dir.iterdir()
            )
            if has_session_images:
                review_path = self.import_review_folder(self.videos_dir)
                if review_path is not None:
                    self.refresh_video_recordings()
                    review_index = next(
                        (
                            index for index, entry in enumerate(self.video_recordings)
                            if entry.path.resolve() == review_path.resolve()
                        ),
                        -1,
                    )
                    if review_index >= 0:
                        self.enter_video_review(review_index)
                        return
        selected_folder = self.select_review_image_folder()
        if selected_folder is None:
            self.status = "Image folder review selection canceled"
            return
        review_path = self.import_review_folder(selected_folder)
        if review_path is None:
            return
        self.refresh_video_recordings()
        review_index = next(
            (
                index for index, entry in enumerate(self.video_recordings)
                if entry.path.resolve() == review_path.resolve()
            ),
            -1,
        )
        if review_index < 0:
            self.status = "Selected folder could not be added to review list"
            return
        self.enter_video_review(review_index)

    def unique_recording_dir(self) -> Path:
        self.videos_dir.mkdir(parents=True, exist_ok=True)
        item_token = safe_name(self.item_name, fallback="item")
        bin_token = safe_name(self.active_bin.bin_name, fallback="bin") if self.active_bin else "bin"
        name = f"roi_{item_token}_{bin_token}_{timestamp_for_path()}"
        path = self.videos_dir / name
        suffix = 1
        while (
            path.exists()
            or path.with_suffix(".png").exists()
            or self.raw_capture_image_path(path).exists()
            or path.with_suffix(".avi").exists()
            or path.with_suffix(".yaml").exists()
        ):
            path = self.videos_dir / f"{name}_{suffix}"
            suffix += 1
        return path

    def capture_roi_image(self) -> None:
        if self.review_mode:
            self.status = "Exit image review before capturing ROI"
            return
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot capture while training"
            return
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.active_bin is None:
            self.status = "Load a bin before capturing ROI"
            return
        frame = self.latest_frame_copy()
        if frame is None:
            self.status = "Wait for color image before capturing"
            return
        roi = self.roi_views_from_frame(frame)
        if roi is None:
            self.status = "Load a bin before capturing ROI"
            return

        _, rect, _, full_roi_frame, _ = roi
        capture_path = self.unique_recording_dir()
        image_path = self.default_recording_image_path(capture_path)
        now_text = _dt.datetime.now().isoformat(timespec="seconds")
        metadata = {
            "recording": {
                "name": capture_path.name,
                "created_at": now_text,
                "ended_at": now_text,
                "bin_name": self.active_bin.bin_name if self.active_bin else "",
                "bin_file": str(self.active_bin.path) if self.active_bin else "",
                "image_file": image_path.name,
                "frame_source": "raw_camera_full_roi_masked",
                "mask_mode": "outside_roi_black",
                "image_width": int(full_roi_frame.shape[1]),
                "image_height": int(full_roi_frame.shape[0]),
                "roi_crop_rect": list(rect),
                "source_roi_crop_rect": list(rect),
                "frame_count": 1,
                "frames": [{
                    "index": 1,
                    "file": image_path.name,
                    "captured_at": _dt.datetime.now().isoformat(timespec="milliseconds"),
                    "frame_view": "full_roi_masked",
                    "width": int(full_roi_frame.shape[1]),
                    "height": int(full_roi_frame.shape[0]),
                    "status": "pending",
                    "roi_crop_rect": list(rect),
                    "source_roi_crop_rect": list(rect),
                }],
            }
        }
        try:
            if not cv2.imwrite(str(image_path), full_roi_frame):
                raise RuntimeError(f"cv2.imwrite returned false for {image_path}")
            metadata_path = self.recording_metadata_path(capture_path)
            tmp_path = metadata_path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
            tmp_path.replace(metadata_path)
            self.refresh_video_recordings()
            self.status = (
                f"Captured ROI image {full_roi_frame.shape[1]}x{full_roi_frame.shape[0]}: "
                f"{image_path.name}"
            )
            self.save_session()
        except Exception as exc:
            image_path.unlink(missing_ok=True)
            self.recording_metadata_path(capture_path).unlink(missing_ok=True)
            self.status = f"ROI capture failed: {exc}"
            self.get_logger().warn(self.status)

    def start_video_recording(self) -> None:
        if self.recording_active:
            self.stop_video_recording()
            return
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot record while training"
            return
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.active_bin is None:
            self.status = "Load a bin before recording ROI video"
            return
        if self.latest_frame_copy() is None:
            self.status = "Wait for color image before recording"
            return

        self.reset_video_review_state(clear_prompts=True)
        self.recording_dir = self.unique_recording_dir()
        self.recording_frames_dir = None
        self.recording_video_path = self.default_recording_video_path(self.recording_dir)
        self.recording_writer = None
        self.recording_frame_count = 0
        self.recording_last_capture_time = 0.0
        self.recording_frame_size = (0, 0)
        now_text = _dt.datetime.now().isoformat(timespec="seconds")
        self.recording_metadata = {
            "recording": {
                "name": self.recording_dir.name,
                "created_at": now_text,
                "ended_at": "",
                "bin_name": self.active_bin.bin_name if self.active_bin else "",
                "bin_file": str(self.active_bin.path) if self.active_bin else "",
                "fps": self.record_fps,
                "video_file": str(self.recording_video_path.name),
                "frame_source": "raw_camera_full_roi_masked",
                "mask_mode": "outside_roi_black",
                "video_width": 0,
                "video_height": 0,
                "frame_count": 0,
                "frames": [],
            }
        }
        self.recording_active = True
        self.write_recording_metadata()
        self.status = f"Recording raw-size ROI video at {self.record_fps:.1f} FPS"
        self.save_session()

    def stop_video_recording(self, enter_review: bool = True) -> None:
        if not self.recording_active:
            return
        try:
            if self.recording_writer is not None:
                self.recording_writer.release()
        except Exception as exc:
            self.get_logger().warn(f"Could not close ROI recording video writer: {exc}")
        self.recording_writer = None
        self.recording_active = False
        if self.recording_metadata:
            recording = self.recording_metadata.setdefault("recording", {})
            recording["ended_at"] = _dt.datetime.now().isoformat(timespec="seconds")
            recording["frame_count"] = self.recording_frame_count
            self.write_recording_metadata()
        finished_dir = self.recording_dir
        frame_count = self.recording_frame_count
        frame_size = self.recording_frame_size
        self.recording_dir = None
        self.recording_frames_dir = None
        self.recording_metadata = {}
        self.recording_frame_count = 0
        self.recording_last_capture_time = 0.0
        self.recording_frame_size = (0, 0)
        self.refresh_video_recordings()
        if enter_review and finished_dir is not None and frame_count > 0:
            review_index = next(
                (
                    index for index, entry in enumerate(self.video_recordings)
                    if entry.path == finished_dir
                ),
                0,
            )
            self.enter_video_review(review_index)
        else:
            self.status = (
                f"Recorded raw-size ROI video {frame_size[0]}x{frame_size[1]} with {frame_count} frames"
                if frame_count > 0 else
                "ROI recording stopped with no frames"
            )
        self.save_session()

    def ensure_recording_writer(self, frame: np.ndarray) -> None:
        if self.recording_writer is not None or self.recording_video_path is None:
            return
        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return
        try:
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            writer = cv2.VideoWriter(
                str(self.recording_video_path),
                fourcc,
                float(max(0.5, self.record_fps)),
                (w, h),
            )
            if writer.isOpened():
                self.recording_writer = writer
                self.recording_frame_size = (w, h)
            else:
                writer.release()
                self.recording_writer = None
        except Exception as exc:
            self.recording_writer = None
            self.get_logger().warn(f"Could not open ROI recording video writer: {exc}")

    def record_roi_frame(self, frame_view: np.ndarray, rect: Tuple[int, int, int, int]) -> None:
        if not self.recording_active or self.recording_dir is None:
            return
        now = time.monotonic()
        interval = 1.0 / float(max(0.5, self.record_fps))
        if self.recording_frame_count > 0 and now - self.recording_last_capture_time < interval:
            return
        try:
            self.ensure_recording_writer(frame_view)
            if self.recording_writer is None:
                self.status = "ROI recording failed: could not open video writer"
                return
            if self.recording_frame_size != (frame_view.shape[1], frame_view.shape[0]):
                self.status = "ROI recording skipped frame: video size changed"
                return
            self.recording_writer.write(frame_view)
            self.recording_frame_count += 1
            frame_index = self.recording_frame_count
            recording = self.recording_metadata.setdefault("recording", {})
            recording["frame_source"] = "raw_camera_full_roi_masked"
            recording["mask_mode"] = "outside_roi_black"
            recording["video_width"] = int(frame_view.shape[1])
            recording["video_height"] = int(frame_view.shape[0])
            frames = recording.setdefault("frames", [])
            frames.append({
                "index": frame_index,
                "video_frame": frame_index,
                "captured_at": _dt.datetime.now().isoformat(timespec="milliseconds"),
                "frame_view": "full_roi_masked",
                "width": int(frame_view.shape[1]),
                "height": int(frame_view.shape[0]),
                "status": "pending",
                "roi_crop_rect": list(rect),
            })
            recording["frame_count"] = frame_index
            self.recording_last_capture_time = now
            self.write_recording_metadata()
            if frame_index == 1 or frame_index % max(1, int(round(self.record_fps))) == 0:
                self.status = (
                    f"Recording raw-size ROI video {frame_view.shape[1]}x{frame_view.shape[0]}: "
                    f"{frame_index} frames"
                )
        except Exception as exc:
            self.status = f"ROI recording failed: {exc}"
            self.get_logger().warn(self.status)

    def review_frame_records(self) -> List[Dict]:
        recording = self.review_recording_meta.get("recording", {})
        frames = recording.get("frames", [])
        return frames if isinstance(frames, list) else []

    def current_review_recording_path(self) -> Optional[Path]:
        if 0 <= self.review_recording_index < len(self.video_recordings):
            return self.video_recordings[self.review_recording_index].path
        return None

    def current_review_frame_record(self) -> Optional[Dict]:
        frames = self.review_frame_records()
        if 0 <= self.review_frame_index < len(frames):
            frame = frames[self.review_frame_index]
            return frame if isinstance(frame, dict) else None
        return None

    def review_frame_path(self, frame: Dict) -> Optional[Path]:
        recording_path = self.current_review_recording_path()
        file_text = str(frame.get("file", ""))
        if recording_path is None or not file_text:
            return None
        file_path = Path(file_text)
        if file_path.is_absolute():
            return file_path
        return self.recording_root_for_relative_files(recording_path) / file_text

    def path_is_inside(self, path: Path, root: Path) -> bool:
        try:
            path.resolve().relative_to(root.resolve())
            return True
        except Exception:
            return False

    def read_review_video_frame(self, frame: Dict, fallback_index: int) -> Optional[np.ndarray]:
        recording_path = self.current_review_recording_path()
        recording = self.review_recording_meta.get("recording", {})
        if recording_path is None or not isinstance(recording, dict):
            return None
        video_path = self.recording_video_path_from_meta(recording_path, recording)
        if video_path is None or not video_path.exists():
            return None
        try:
            video_frame = int(frame.get("video_frame", frame.get("index", fallback_index + 1)))
        except (TypeError, ValueError):
            video_frame = fallback_index + 1
        cap = cv2.VideoCapture(str(video_path))
        try:
            if not cap.isOpened():
                return None
            cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, video_frame - 1))
            ok, image = cap.read()
            if not ok or image is None:
                return None
            return image
        finally:
            cap.release()

    def enter_video_review(self, index: int = 0, frame_index: Optional[int] = None) -> None:
        if self.recording_active:
            self.status = "Stop recording before reviewing"
            return
        self.refresh_video_recordings()
        if not self.video_recordings:
            self.status = "No ROI images in this session"
            return
        index = min(max(0, index), len(self.video_recordings) - 1)
        self.review_recording_index = index
        recording_path = self.video_recordings[index].path
        try:
            self.review_recording_meta = self.read_recording_metadata(recording_path)
            self.review_mode = True
            frames = self.review_frame_records()
            if frame_index is None:
                frame_index = next(
                    (
                        i for i, frame in enumerate(frames)
                        if isinstance(frame, dict) and frame.get("status", "pending") == "pending"
                    ),
                    0,
                )
            self.load_review_frame(frame_index, clear_prompts=True)
        except Exception as exc:
            self.status = f"Could not open ROI image review: {exc}"
            self.get_logger().warn(self.status)

    def exit_video_review(self) -> None:
        self.reset_video_review_state(clear_prompts=True)
        self.status = "Exited ROI image review"
        self.save_session()

    def write_review_recording_metadata(self) -> None:
        recording_path = self.current_review_recording_path()
        if recording_path is None:
            return
        try:
            path = self.recording_metadata_path(recording_path)
            tmp_path = path.with_suffix(".tmp")
            tmp_path.write_text(
                yaml.safe_dump(self.review_recording_meta, sort_keys=False),
                encoding="utf-8",
            )
            tmp_path.replace(path)
            self.refresh_video_recordings()
        except Exception as exc:
            self.get_logger().warn(f"Could not save ROI review metadata: {exc}")

    def review_status_label(self) -> str:
        if not self.review_mode:
            return "No image review active"
        frames = self.review_frame_records()
        total = len(frames)
        frame = self.current_review_frame_record()
        status = str(frame.get("status", "pending")) if frame else "missing"
        rec_name = (
            self.video_recordings[self.review_recording_index].path.name
            if 0 <= self.review_recording_index < len(self.video_recordings) else "recording"
        )
        return f"{rec_name} frame {self.review_frame_index + 1}/{max(1, total)} | {status}"

    def has_any_prompt(self) -> bool:
        return bool(self.positive_points or self.negative_points)

    def review_frame_sample_stems(self, frame: Optional[Dict] = None) -> List[str]:
        frame = frame if frame is not None else self.current_review_frame_record()
        if not isinstance(frame, dict):
            return []
        stems: List[str] = []
        raw_stems = frame.get("sample_stems", [])
        if isinstance(raw_stems, list):
            stems.extend(str(stem).strip() for stem in raw_stems if str(stem).strip())
        sample_stem = str(frame.get("sample_stem", "")).strip()
        if sample_stem and sample_stem not in stems:
            stems.append(sample_stem)
        return stems

    def review_frame_sample_count(self, frame: Optional[Dict] = None) -> int:
        frame = frame if frame is not None else self.current_review_frame_record()
        if not isinstance(frame, dict):
            return 0
        stems = self.review_frame_sample_stems(frame)
        if stems:
            return len(stems)
        try:
            return max(0, int(frame.get("sample_count", 0)))
        except (TypeError, ValueError):
            return 0

    def load_saved_review_mask(self, frame: Dict) -> str:
        if str(frame.get("status", "pending")) != "annotated":
            return ""
        if self.frozen_crop_bgr is None:
            return ""
        stems = self.review_frame_sample_stems(frame)
        if not stems:
            return ""
        for stem in reversed(stems):
            mask_path = self.masks_dir / f"{stem}.png"
            if not mask_path.exists():
                continue
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                continue
            crop_h, crop_w = self.frozen_crop_bgr.shape[:2]
            if mask.shape[:2] != (crop_h, crop_w):
                mask = cv2.resize(mask, (crop_w, crop_h), interpolation=cv2.INTER_NEAREST)
            self.current_mask = (mask > 0).astype(np.uint8) * 255
            return f" | loaded mask {stem}"
        return f" | {len(stems)} saved sample(s)"

    def load_review_frame(
        self,
        index: int,
        clear_prompts: bool = True,
        restore_saved_annotation: bool = True,
    ) -> None:
        frames = self.review_frame_records()
        if not frames:
            self.status = "Selected ROI image has no frames"
            return
        index = min(max(0, index), len(frames) - 1)
        frame = frames[index]
        frame_path = self.review_frame_path(frame)
        image = None
        if frame_path is not None and frame_path.exists():
            image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if image is None:
            image = self.read_review_video_frame(frame, index)
        if image is None:
            self.status = "Could not read ROI image frame"
            return
        self.review_frame_index = index
        h, w = image.shape[:2]
        raw_rect = frame.get("roi_crop_rect", [])
        use_full_view = self.review_frame_uses_roi_rect(str(frame.get("frame_view", "")))
        rect = (0, 0, w, h)
        if use_full_view and isinstance(raw_rect, list) and len(raw_rect) >= 4:
            rx, ry, rw, rh = [int(round(float(value))) for value in raw_rect[:4]]
            rx = min(max(0, rx), max(0, w - 1))
            ry = min(max(0, ry), max(0, h - 1))
            rw = max(1, min(rw, w - rx))
            rh = max(1, min(rh, h - ry))
            rect = (rx, ry, rw, rh)
        x0, y0, rect_w, rect_h = rect
        crop = image[y0:y0 + rect_h, x0:x0 + rect_w].copy()
        self.frozen_frame_bgr = image.copy()
        self.frozen_crop_bgr = crop
        self.frozen_crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        self.roi_crop_rect = rect
        self.roi_crop_mask = np.full((rect_h, rect_w), 255, dtype=np.uint8)
        if clear_prompts:
            self.positive_points.clear()
            self.negative_points.clear()
            self.current_mask = None
        self.sam2_image_key = None
        self.prediction_dirty = False
        restored_annotation = (
            self.load_saved_review_mask(frame)
            if clear_prompts and restore_saved_annotation else ""
        )
        self.status = f"{self.review_status_label()}{restored_annotation}"
        self.save_session()

    def review_previous_frame(self) -> None:
        if not self.review_mode:
            return
        frames = self.review_frame_records()
        if not frames:
            self.status = "Selected ROI image has no frames"
            return
        self.load_review_frame((self.review_frame_index - 1) % len(frames), clear_prompts=True)

    def review_next_frame(self) -> None:
        if not self.review_mode:
            return
        frames = self.review_frame_records()
        if not frames:
            self.status = "Selected ROI image has no frames"
            return
        self.load_review_frame((self.review_frame_index + 1) % len(frames), clear_prompts=True)

    def delete_current_review_frame(self) -> None:
        if not self.review_mode:
            self.status = "Open a ROI image review first"
            return
        frames = self.review_frame_records()
        if not frames:
            self.status = "No review frame to delete"
            return
        index = min(max(0, self.review_frame_index), len(frames) - 1)
        frame = frames[index]
        frame_path = self.review_frame_path(frame) if isinstance(frame, dict) else None
        recording_path = self.current_review_recording_path()
        deleted_paths = []
        if frame_path is not None and self.path_is_inside(frame_path, self.videos_dir):
            candidate_paths = [
                frame_path,
                frame_path.with_suffix(".yaml"),
            ]
            raw_path = self.raw_capture_image_path(frame_path.with_suffix(""))
            candidate_paths.extend([
                raw_path,
                raw_path.with_suffix(".yaml"),
            ])
            seen_paths = set()
            for path in candidate_paths:
                try:
                    resolved = path.resolve()
                except Exception:
                    resolved = path
                if str(resolved) in seen_paths:
                    continue
                seen_paths.add(str(resolved))
                if not self.path_is_inside(path, self.videos_dir):
                    continue
                try:
                    if path.exists():
                        path.unlink()
                        deleted_paths.append(path.name)
                except Exception as exc:
                    self.get_logger().warn(f"Could not delete review frame artifact {path}: {exc}")

        del frames[index]
        for frame_index, item in enumerate(frames, start=1):
            if isinstance(item, dict):
                item["index"] = frame_index
        recording = self.review_recording_meta.setdefault("recording", {})
        recording["frames"] = frames
        recording["frame_count"] = len(frames)
        self.review_frame_index = min(index, max(0, len(frames) - 1))
        if frames:
            self.write_review_recording_metadata()
            self.load_review_frame(self.review_frame_index, clear_prompts=True)
            deleted_label = f" and deleted {len(deleted_paths)} files" if deleted_paths else ""
            self.status = f"Deleted review frame {index + 1}{deleted_label}"
        else:
            if recording_path is not None:
                metadata_path = self.recording_metadata_path(recording_path)
                if self.path_is_inside(metadata_path, self.videos_dir):
                    try:
                        if metadata_path.exists():
                            metadata_path.unlink()
                            deleted_paths.append(metadata_path.name)
                    except Exception as exc:
                        self.get_logger().warn(f"Could not delete review frame metadata {metadata_path}: {exc}")
            self.reset_video_review_state(clear_prompts=True)
            self.refresh_video_recordings()
            deleted_label = f" and deleted {len(deleted_paths)} files" if deleted_paths else ""
            self.status = f"Deleted last review frame{deleted_label}"
            self.save_session()

    def load_next_pending_review_frame(self) -> None:
        frames = self.review_frame_records()
        if not frames:
            return
        for index in range(self.review_frame_index + 1, len(frames)):
            frame = frames[index]
            if isinstance(frame, dict) and frame.get("status", "pending") == "pending":
                self.load_review_frame(index, clear_prompts=True)
                return
        pending = [
            i for i, frame in enumerate(frames)
            if isinstance(frame, dict) and frame.get("status", "pending") == "pending"
        ]
        if pending:
            self.load_review_frame(pending[0], clear_prompts=True)
            return
        self.status = f"ROI image review complete | {self.sample_count_label()}"
        self.save_session()

    def annotate_review_frame(self) -> None:
        if not self.review_mode:
            self.status = "Open a ROI image review first"
            return
        self.load_review_frame(
            self.review_frame_index,
            clear_prompts=True,
            restore_saved_annotation=False,
        )
        self.status = "Add positive/negative SAM2 prompts on this ROI frame"

    def request_review_sam2_annotation(self) -> None:
        if not self.review_mode:
            self.status = "Open a ROI image review first"
            return
        if self.frozen_crop_bgr is None:
            self.load_review_frame(self.review_frame_index, clear_prompts=False)
        if not self.positive_points:
            self.status = "Add at least one positive prompt before SAM2"
            return
        self.prediction_dirty = True
        self.status = "SAM2 annotation queued for ROI image frame"

    def mark_current_review_frame(self, status: str, sample_stem: str = "") -> None:
        frame = self.current_review_frame_record()
        if frame is None:
            return
        frame["status"] = status
        frame["updated_at"] = _dt.datetime.now().isoformat(timespec="seconds")
        if sample_stem:
            sample_stems = frame.get("sample_stems", [])
            if not isinstance(sample_stems, list):
                sample_stems = []
            previous_stem = str(frame.get("sample_stem", "")).strip()
            if previous_stem and previous_stem not in sample_stems:
                sample_stems.append(previous_stem)
            if sample_stem not in sample_stems:
                sample_stems.append(sample_stem)
            frame["sample_stems"] = sample_stems
            frame["sample_stem"] = sample_stem
            frame["sample_count"] = len(sample_stems)
        self.write_review_recording_metadata()

    def reload_current_review_frame_after_save(self, status_text: str) -> None:
        if self.review_mode and self.review_frame_records():
            self.load_review_frame(
                self.review_frame_index,
                clear_prompts=True,
                restore_saved_annotation=False,
            )
        self.status = status_text
        self.save_session()

    def remove_review_sample_reference(self, sample_stem: str) -> None:
        sample_stem = str(sample_stem).strip()
        if not sample_stem or not isinstance(self.review_recording_meta, dict):
            return
        frames = self.review_frame_records()
        changed = False
        for frame in frames:
            if not isinstance(frame, dict):
                continue
            sample_stems = frame.get("sample_stems", [])
            if not isinstance(sample_stems, list):
                sample_stems = []
            sample_stems = [str(stem).strip() for stem in sample_stems if str(stem).strip()]
            previous_stem = str(frame.get("sample_stem", "")).strip()
            if previous_stem and previous_stem not in sample_stems:
                sample_stems.append(previous_stem)
            if sample_stem not in sample_stems:
                continue
            sample_stems = [stem for stem in sample_stems if stem != sample_stem]
            frame["sample_stems"] = sample_stems
            frame["sample_count"] = len(sample_stems)
            frame["sample_stem"] = sample_stems[-1] if sample_stems else ""
            if not sample_stems and frame.get("status") == "annotated":
                frame["status"] = "pending"
            frame["updated_at"] = _dt.datetime.now().isoformat(timespec="seconds")
            changed = True
        if changed:
            self.write_review_recording_metadata()

    def save_review_frame(self) -> None:
        if not self.review_mode:
            self.status = "Open a ROI image review first"
            return
        if self.current_mask is None:
            if not self.has_any_prompt():
                self.status = "Add at least one positive or negative prompt before saving"
                return
            if not self.positive_points and self.negative_points:
                if self.frozen_crop_bgr is None:
                    self.status = "No ROI image frame loaded"
                    return
                stem = self.save_background_crop_sample(
                    self.frozen_crop_bgr,
                    self.roi_crop_rect or (0, 0, self.frozen_crop_bgr.shape[1], self.frozen_crop_bgr.shape[0]),
                    source="roi_image_negative_prompt",
                )
                if not stem:
                    return
                self.mark_current_review_frame("annotated", stem)
                self.reload_current_review_frame_after_save(f"Saved ROI image frame as {stem}")
                return
            self.run_sam2_prediction()
        if self.current_mask is None:
            return
        stem = self.save_sample()
        if not stem:
            return
        self.mark_current_review_frame("annotated", stem)
        self.reload_current_review_frame_after_save(f"Saved ROI image frame as {stem}")

    def save_review_background_frame(self) -> None:
        if not self.review_mode:
            self.status = "Open a ROI image review first"
            return
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.frozen_crop_bgr is None:
            self.status = "No ROI image frame loaded"
            return
        stem = self.save_background_crop_sample(
            self.frozen_crop_bgr,
            self.roi_crop_rect or (0, 0, self.frozen_crop_bgr.shape[1], self.frozen_crop_bgr.shape[0]),
            source="roi_image_background_button",
        )
        if not stem:
            return
        self.mark_current_review_frame("annotated", stem)
        self.reload_current_review_frame_after_save(f"Saved ROI image frame as background {stem}")

    def skip_review_frame(self) -> None:
        if not self.review_mode:
            self.status = "Open a ROI image review first"
            return
        self.mark_current_review_frame("skipped")
        self.status = "Skipped ROI image frame"
        self.load_next_pending_review_frame()

    def saved_session_path_is_safe(self, path: Path) -> bool:
        try:
            resolved_root = self.saved_sessions_root.resolve()
            resolved_path = path.resolve()
            if resolved_path == resolved_root:
                return False
            resolved_path.relative_to(resolved_root)
            return True
        except Exception:
            return False

    def current_session_is_saved(self) -> bool:
        return self.saved_session_path_is_safe(self.session_dir)

    def unique_saved_session_dir(self) -> Path:
        self.saved_sessions_root.mkdir(parents=True, exist_ok=True)
        name = f"{safe_name(self.item_name, fallback='unnamed_item')}_{timestamp_for_path()}"
        path = self.saved_sessions_root / name
        suffix = 1
        while path.exists():
            path = self.saved_sessions_root / f"{name}_{suffix}"
            suffix += 1
        return path

    def read_saved_session_entry(self, path: Path) -> Optional[SavedSessionEntry]:
        session_yaml = path / "session.yaml"
        if not session_yaml.exists():
            return None
        try:
            root = yaml.safe_load(session_yaml.read_text(encoding="utf-8")) or {}
            params = root.get("item_teach_yolo_session", {})
            if not isinstance(params, dict):
                return None
            item_name = str(params.get("item_name", path.name)).strip() or path.name
            sample_count = int(params.get("sample_count", 0))
            background_count = int(params.get("background_sample_count", 0))
            modified_time = session_yaml.stat().st_mtime
            modified_text = _dt.datetime.fromtimestamp(modified_time).strftime("%m/%d %H:%M")
            label = (
                f"{item_name} | Item {sample_count} BG {background_count} | "
                f"{modified_text}"
            )
            return SavedSessionEntry(
                label=label,
                path=path,
                item_name=item_name,
                sample_count=sample_count,
                background_sample_count=background_count,
                modified_time=modified_time,
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not read saved YOLO teach session {path}: {exc}")
            return None

    def refresh_saved_sessions(self) -> None:
        entries: List[SavedSessionEntry] = []
        try:
            self.saved_sessions_root.mkdir(parents=True, exist_ok=True)
            for path in self.saved_sessions_root.iterdir():
                if not path.is_dir():
                    continue
                entry = self.read_saved_session_entry(path)
                if entry is not None:
                    entries.append(entry)
        except Exception as exc:
            self.get_logger().warn(f"Could not list saved YOLO teach sessions: {exc}")
        self.saved_session_entries = sorted(
            entries,
            key=lambda entry: (entry.item_name.lower(), -entry.modified_time),
        )

    def save_current_session_as_saved(self) -> None:
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot save session while training"
            return
        if self.recording_active:
            self.status = "Stop ROI recording before saving session"
            return
        try:
            self.save_session()
            if self.current_session_is_saved():
                self.write_dataset_yaml()
                self.save_session()
                self.status = f"Saved session: {self.session_dir.name}"
                self.refresh_saved_sessions()
                return

            old_session_dir = self.session_dir
            review_was_active = self.review_mode
            review_recording_path = self.current_review_recording_path()
            review_recording_name = review_recording_path.name if review_recording_path else ""
            review_frame_index = self.review_frame_index
            target = self.unique_saved_session_dir()
            shutil.copytree(old_session_dir, target)
            self.session_dir = target
            self.configure_session_storage()
            self.write_dataset_yaml()
            self.refresh_video_recordings()
            if review_was_active and review_recording_name:
                review_index = self.recording_index_for_session_ref("", review_recording_name)
                if review_index >= 0:
                    self.enter_video_review(review_index, frame_index=review_frame_index)
                else:
                    self.reset_video_review_state(clear_prompts=True)
            else:
                self.reset_video_review_state(clear_prompts=True)
            self.save_session()
            self.refresh_saved_sessions()
            self.status = f"Saved session: {target.name}"
        except Exception as exc:
            self.status = f"Save session failed: {exc}"
            self.get_logger().warn(self.status)

    def restore_active_bin_from_session(self, params: Dict) -> None:
        self.active_bin_index = -1
        self.active_bin = None
        active_bin_file = str(params.get("active_bin_file", "")).strip()
        active_bin_name = str(params.get("active_bin_name", "")).strip()
        active_bin_path: Optional[Path] = None
        if active_bin_file:
            try:
                active_bin_path = resolve_path(active_bin_file)
            except Exception:
                active_bin_path = None
        for index, entry in enumerate(self.bin_entries):
            same_file = False
            if active_bin_path is not None:
                try:
                    same_file = entry.path.resolve() == active_bin_path.resolve()
                except Exception:
                    same_file = False
            if same_file or (active_bin_name and entry.bin_name == active_bin_name):
                self.active_bin_index = index
                self.active_bin = entry
                return

    def load_saved_session(self, path: Path) -> None:
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot load session while training"
            return
        if self.recording_active:
            self.status = "Cannot load session while capturing ROI"
            return
        try:
            resolved_path = path.resolve()
            if not self.saved_session_path_is_safe(resolved_path):
                self.status = "Load session failed: unsafe path"
                return
            session_yaml = resolved_path / "session.yaml"
            root = yaml.safe_load(session_yaml.read_text(encoding="utf-8")) or {}
            params = root.get("item_teach_yolo_session", {})
            if not isinstance(params, dict):
                self.status = "Load session failed: missing session metadata"
                return

            self.clear_prompts(save=False)
            self.session_dir = resolved_path
            self.configure_session_storage()
            self.item_name = str(params.get("item_name", "")).strip()
            self.item_name_edit_buffer = self.item_name
            self.item_name_edit_active = False
            self.sample_count = max(0, int(params.get("sample_count", 0)))
            self.background_sample_count = max(0, int(params.get("background_sample_count", 0)))
            sample_history = params.get("sample_history", [])
            self.sample_history = [
                dict(entry) for entry in sample_history
                if isinstance(entry, dict) and entry.get("stem")
            ] if isinstance(sample_history, list) else []
            loaded_training_status = str(params.get("training_status", "idle"))
            if loaded_training_status.startswith("training"):
                loaded_training_status = "idle"
            self.training_status = loaded_training_status
            self.training_epoch_current = int(params.get("training_epoch_current", 0))
            self.training_epoch_total = max(1, int(params.get("training_epoch_total", self.train_epochs)))
            self.training_progress = float(params.get("training_progress", 0.0))
            self.trained_model_path = str(params.get("trained_model_path", ""))
            self.trained_onnx_path = str(params.get("trained_onnx_path", ""))
            if "train_use_gpu_if_available" in params:
                self.train_use_gpu_if_available = as_bool(params["train_use_gpu_if_available"])
            self.train_device_used = str(params.get("train_device_used", self.train_device_used))
            self.final_model_dir = str(params.get("final_model_dir", ""))
            self.latest_profile_path = str(params.get("latest_profile_path", ""))
            teach_joints = params.get("teach_joints_deg", [])
            if isinstance(teach_joints, list) and len(teach_joints) >= 6:
                self.latest_joint_positions_deg = [float(value) for value in teach_joints[:6]]
                self.has_joint_positions = True
                self.teach_joints_source = str(params.get("teach_joints_source", "saved_session"))
            else:
                self.has_joint_positions = False
                self.teach_joints_source = ""
            if "color_exposure_us" in params:
                self.color_exposure_us = clamp_exposure_usec_or_auto(
                    int(params.get("color_exposure_us", self.color_exposure_us)),
                    self.color_exposure_min_us,
                    self.color_exposure_max_us,
                )
                self.mark_camera_exposure_dirty()
            self.depth_exposure_us = 0
            self.refresh_bin_files()
            self.restore_active_bin_from_session(params)
            self.recording_active = False
            self.recording_dir = None
            self.recording_frames_dir = None
            self.recording_metadata = {}
            self.recording_writer = None
            self.recording_video_path = None
            self.recording_frame_count = 0
            self.refresh_video_recordings()
            review_restored = self.restore_video_review_from_session(params)
            if not review_restored:
                self.reset_video_review_state(clear_prompts=True)
            self.write_dataset_yaml()
            self.save_session()
            self.request_teach_joint_snapshot("loaded teach")
            self.refresh_saved_sessions()
            self.load_session_dropdown_open = False
            self.status = (
                f"Loaded session: {self.item_name or self.session_dir.name} | "
                f"{self.sample_count_label()} | updating teach position"
                f"{' | review restored' if review_restored else ''}")
        except Exception as exc:
            self.status = f"Load session failed: {exc}"
            self.get_logger().warn(self.status)

    def load_saved_session_by_index(self, index: int) -> None:
        if index < 0 or index >= len(self.saved_session_entries):
            self.status = "No saved session selected"
            return
        self.load_saved_session(self.saved_session_entries[index].path)

    def delete_saved_session_by_index(self, index: int) -> None:
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot delete session while training"
            return
        if self.recording_active:
            self.status = "Cannot delete session while capturing ROI"
            return
        if index < 0 or index >= len(self.saved_session_entries):
            self.status = "No saved session selected"
            return
        entry = self.saved_session_entries[index]
        try:
            resolved_path = entry.path.resolve()
            if not self.saved_session_path_is_safe(resolved_path):
                self.status = "Delete session failed: unsafe path"
                return
            deleting_current = False
            try:
                deleting_current = resolved_path == self.session_dir.resolve()
            except Exception:
                deleting_current = False
            if deleting_current:
                self.clear_prompts(save=False)
                self.item_name = ""
                self.item_name_edit_buffer = ""
                self.sample_count = 0
                self.background_sample_count = 0
                self.sample_history.clear()
                self.training_status = "idle"
                self.training_epoch_current = 0
                self.training_epoch_total = max(1, self.train_epochs)
                self.training_progress = 0.0
                self.trained_model_path = ""
                self.trained_onnx_path = ""
                self.train_device_used = "cpu"
                self.final_model_dir = ""
                self.latest_profile_path = ""
                self.active_bin_index = -1
                self.active_bin = None
                self.video_recordings = []
                self.reset_video_review_state(clear_prompts=True)
                self.session_dir = self.create_session_dir()
                self.configure_session_storage()
                self.write_dataset_yaml()
                self.save_session()
            shutil.rmtree(resolved_path)
            self.refresh_saved_sessions()
            self.load_session_dropdown_open = bool(self.saved_session_entries)
            self.status = f"Deleted saved session: {entry.item_name}"
        except Exception as exc:
            self.status = f"Delete session failed: {exc}"
            self.get_logger().warn(self.status)

    def refresh_bin_files(self) -> None:
        entries: List[BinEntry] = []
        if self.bin_teach_dir.exists():
            for path in sorted(
                self.bin_teach_dir.glob("*.yaml"),
                key=lambda candidate: candidate.stat().st_mtime,
                reverse=True,
            ):
                entry = self.parse_bin_file(path)
                if entry is not None:
                    entries.append(entry)
        self.bin_entries = entries
        if not entries:
            self.status = f"No compatible Bin Teach YAML in {self.bin_teach_dir}"

    def parse_bin_file(self, path: Path) -> Optional[BinEntry]:
        try:
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            bin_node = root.get("bin_teach", {})
            roi_points = parse_flat_points(bin_node.get("roi_points", []))
            if not roi_points:
                return None
            bin_name = str(bin_node.get("bin_name", path.stem))
            depth_plane = {}
            for key in [
                "depth_plane_enabled",
                "depth_plane_a",
                "depth_plane_b",
                "depth_plane_c",
                "depth_plane_reference_depth_m",
            ]:
                if key in bin_node:
                    depth_plane[key] = bin_node[key]
            teach_date = str(bin_node.get("teach_date", ""))
            label = bin_name
            if teach_date:
                label = f"{label} | {teach_date}"
            elif bin_name != path.stem:
                label = f"{label} | {path.stem}"
            return BinEntry(
                label=label,
                path=path,
                bin_name=bin_name,
                teach_date=teach_date,
                roi_points=roi_points,
                depth_plane=depth_plane,
            )
        except Exception as exc:
            self.get_logger().warn(f"Skipping bin teach file {path}: {exc}")
            return None

    def color_callback(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Color conversion failed: {exc}")
            return
        with self.lock:
            self.latest_bgr = bgr.copy()
            self.latest_header_stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

    def select_bin(self, index: int) -> None:
        if self.recording_active:
            self.status = "Stop ROI recording before changing bin"
            return
        if index < 0 or index >= len(self.bin_entries):
            return
        self.active_bin_index = index
        self.active_bin = self.bin_entries[index]
        self.bin_dropdown_open = False
        self.clear_prompts(save=False)
        self.status = f"Loaded bin ROI: {self.active_bin.bin_name}"
        self.save_session()

    def latest_frame_copy(self) -> Optional[np.ndarray]:
        with self.lock:
            frame = None if self.latest_bgr is None else self.latest_bgr.copy()
        return frame

    def roi_views_from_frame(
        self,
        frame: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray]]:
        if frame is None or self.active_bin is None:
            return None
        h, w = frame.shape[:2]
        points = np.asarray(self.active_bin.roi_points, dtype=np.float32)
        xs = np.clip(points[:, 0], 0, max(0, w - 1))
        ys = np.clip(points[:, 1], 0, max(0, h - 1))
        clipped_points = np.column_stack([xs, ys]).astype(np.float32)
        x0 = int(np.floor(np.min(xs)))
        y0 = int(np.floor(np.min(ys)))
        x1 = int(np.ceil(np.max(xs)))
        y1 = int(np.ceil(np.max(ys)))
        x1 = min(w - 1, max(x0 + 1, x1))
        y1 = min(h - 1, max(y0 + 1, y1))
        crop = frame[y0:y1 + 1, x0:x1 + 1].copy()
        rel_points = np.round(clipped_points - np.array([x0, y0], dtype=np.float32)).astype(np.int32)
        roi_mask = np.zeros(crop.shape[:2], dtype=np.uint8)
        cv2.fillPoly(roi_mask, [rel_points], 255)
        crop[roi_mask == 0] = 0

        full_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        full_points = np.round(clipped_points).astype(np.int32)
        cv2.fillPoly(full_mask, [full_points], 255)
        full_roi_frame = np.zeros_like(frame)
        full_roi_frame[full_mask > 0] = frame[full_mask > 0]
        return crop, (x0, y0, crop.shape[1], crop.shape[0]), roi_mask, full_roi_frame, full_mask

    def current_roi_crop(self) -> Optional[Tuple[np.ndarray, Tuple[int, int, int, int], np.ndarray]]:
        frame = self.latest_frame_copy()
        if frame is None:
            return None
        roi_views = self.roi_views_from_frame(frame)
        if roi_views is None:
            return None
        crop, rect, roi_mask, _, _ = roi_views
        return crop, rect, roi_mask

    def freeze_current_crop(self) -> bool:
        frame = self.latest_frame_copy()
        if frame is None:
            self.status = "Select a bin and wait for color image"
            return False
        roi = self.roi_views_from_frame(frame)
        if roi is None:
            self.status = "Select a bin and wait for color image"
            return False
        crop, rect, roi_mask, full_roi_frame, _ = roi
        self.frozen_frame_bgr = full_roi_frame
        self.frozen_crop_bgr = crop
        self.frozen_crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        self.roi_crop_rect = rect
        self.roi_crop_mask = roi_mask
        self.current_mask = None
        self.sam2_image_key = None
        return True

    def clear_prompts(self, save: bool = True) -> None:
        self.positive_points.clear()
        self.negative_points.clear()
        self.current_mask = None
        self.frozen_frame_bgr = None
        self.frozen_crop_bgr = None
        self.frozen_crop_rgb = None
        self.roi_crop_rect = None
        self.roi_crop_mask = None
        self.sam2_image_key = None
        self.prediction_dirty = False
        self.status = "Prompts cleared"
        if save:
            self.save_session()

    def run_sam2_prediction(self) -> None:
        if self.frozen_crop_rgb is None or not self.positive_points:
            self.prediction_dirty = False
            return
        self.predicting = True
        try:
            image_key = (
                self.frozen_crop_rgb.shape[1],
                self.frozen_crop_rgb.shape[0],
                int(np.sum(self.frozen_crop_rgb[:5, :5], dtype=np.int64)),
            )
            if self.sam2_image_key != image_key:
                self.predictor.set_image(self.frozen_crop_rgb)
                self.sam2_image_key = image_key
            points = self.positive_points + self.negative_points
            labels = [1] * len(self.positive_points) + [0] * len(self.negative_points)
            point_coords = np.asarray(points, dtype=np.float32)
            point_labels = np.asarray(labels, dtype=np.int32)
            with torch.inference_mode():
                masks, scores, _ = self.predictor.predict(
                    point_coords=point_coords,
                    point_labels=point_labels,
                    multimask_output=True,
                )
            best_idx = int(np.argmax(scores))
            mask = masks[best_idx].astype(np.uint8) * 255
            if self.roi_crop_mask is not None:
                mask = cv2.bitwise_and(mask, self.roi_crop_mask)
            self.current_mask = mask
            self.status = (
                f"SAM2 mask ready | score {float(scores[best_idx]):.3f} | "
                f"samples {self.sample_count}"
            )
        except Exception as exc:
            self.status = f"SAM2 failed: {exc}"
            self.get_logger().warn(self.status)
        finally:
            self.predicting = False
            self.prediction_dirty = False
            self.save_session()

    def mask_to_largest_contour(self, mask: np.ndarray) -> Optional[np.ndarray]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= 16.0]
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def contour_to_yolo_seg(self, contour: np.ndarray, width: int, height: int) -> Optional[str]:
        epsilon = max(1.0, 0.002 * cv2.arcLength(contour, True))
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if approx.shape[0] < 3:
            approx = contour
        points = approx.reshape(-1, 2)
        if points.shape[0] < 3 or polygon_area(points.astype(np.float32)) < 4.0:
            return None
        values = ["0"]
        for x, y in points:
            values.append(f"{min(max(float(x) / width, 0.0), 1.0):.6f}")
            values.append(f"{min(max(float(y) / height, 0.0), 1.0):.6f}")
        return " ".join(values) + "\n"

    def save_sample(self) -> Optional[str]:
        if not self.has_item_name():
            self.status = self.item_name_error()
            return None
        if self.recording_active:
            self.status = "Stop ROI recording before saving samples"
            return None
        if self.frozen_crop_bgr is None or self.current_mask is None:
            self.status = "No SAM2 mask to save"
            return None
        contour = self.mask_to_largest_contour(self.current_mask)
        if contour is None:
            self.status = "Mask is empty"
            return None
        h, w = self.current_mask.shape[:2]
        label_text = self.contour_to_yolo_seg(contour, w, h)
        if label_text is None:
            self.status = "Mask polygon is invalid"
            return None

        self.sample_count += 1
        stem = f"sample_{self.sample_count:06d}"
        image_path = self.images_dir / f"{stem}.png"
        label_path = self.labels_dir / f"{stem}.txt"
        mask_path = self.masks_dir / f"{stem}.png"
        preview_path = self.previews_dir / f"{stem}_overlay.png"
        prompt_path = self.prompts_dir / f"{stem}.yaml"

        cv2.imwrite(str(image_path), self.frozen_crop_bgr)
        label_path.write_text(label_text, encoding="utf-8")
        cv2.imwrite(str(mask_path), self.current_mask)
        preview = self.render_mask_overlay(self.frozen_crop_bgr.copy(), self.current_mask)
        cv2.imwrite(str(preview_path), preview)
        prompt_data = {
            "sample": stem,
            "item_name": self.item_name,
            "bin_name": self.active_bin.bin_name if self.active_bin else "",
            "bin_file": str(self.active_bin.path) if self.active_bin else "",
            "roi_crop_rect": list(self.roi_crop_rect) if self.roi_crop_rect else [],
            "positive_points": [list(p) for p in self.positive_points],
            "negative_points": [list(p) for p in self.negative_points],
            "image": str(image_path),
            "label": str(label_path),
            "mask": str(mask_path),
            "preview": str(preview_path),
        }
        prompt_path.write_text(yaml.safe_dump(prompt_data, sort_keys=False), encoding="utf-8")
        self.sample_history.append({"kind": "item", "stem": stem})
        self.write_dataset_yaml()
        self.clear_prompts(save=False)
        self.mark_dataset_changed_after_training()
        self.status = f"Saved item sample {self.sample_count} | {self.background_ratio_label()}"
        self.save_session()
        return stem

    def save_background_sample(self) -> None:
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.recording_active:
            self.status = "Stop ROI recording before saving background"
            return
        crop_info = self.current_roi_crop()
        if crop_info is None:
            self.status = "Select a bin and wait for color image"
            return

        crop, rect, _ = crop_info
        self.save_background_crop_sample(crop, rect)

    def save_background_crop_sample(
        self,
        crop: np.ndarray,
        rect: Tuple[int, int, int, int],
        source: str = "live_roi",
    ) -> Optional[str]:
        self.background_sample_count += 1
        stem = f"background_{self.background_sample_count:06d}"
        image_path = self.images_dir / f"{stem}.png"
        label_path = self.labels_dir / f"{stem}.txt"
        preview_path = self.previews_dir / f"{stem}_preview.png"
        prompt_path = self.prompts_dir / f"{stem}.yaml"

        cv2.imwrite(str(image_path), crop)
        label_path.write_text("", encoding="utf-8")
        cv2.imwrite(str(preview_path), crop)
        prompt_data = {
            "sample": stem,
            "sample_type": "background",
            "item_name": self.item_name,
            "bin_name": self.active_bin.bin_name if self.active_bin else "",
            "bin_file": str(self.active_bin.path) if self.active_bin else "",
            "roi_crop_rect": list(rect),
            "image": str(image_path),
            "label": str(label_path),
            "preview": str(preview_path),
            "positive_points": [list(p) for p in self.positive_points],
            "negative_points": [list(p) for p in self.negative_points],
            "source": source,
            "note": "Empty YOLO label file marks this ROI crop as background.",
        }
        prompt_path.write_text(yaml.safe_dump(prompt_data, sort_keys=False), encoding="utf-8")
        self.sample_history.append({"kind": "background", "stem": stem})
        self.write_dataset_yaml()
        self.clear_prompts(save=False)
        self.mark_dataset_changed_after_training()
        self.status = f"Saved background {self.background_sample_count} | {self.background_ratio_label()}"
        self.save_session()
        return stem

    def can_delete_last_sample(self) -> bool:
        training_active = self.training_thread is not None and self.training_thread.is_alive()
        return self.total_training_image_count() > 0 and not training_active and not self.recording_active

    def can_save_background_sample(self) -> bool:
        training_active = self.training_thread is not None and self.training_thread.is_alive()
        return (
            self.has_item_name() and
            self.active_bin is not None and
            self.latest_bgr is not None and
            not training_active and
            not self.recording_active
        )

    def delete_last_sample(self) -> None:
        entry = self.latest_sample_entry()
        if entry is None:
            self.status = "No saved samples to delete"
            return
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot delete samples while training"
            return
        if self.recording_active:
            self.status = "Stop ROI recording before deleting samples"
            return

        kind = entry.get("kind", "item")
        stem = entry.get("stem", "")
        if not stem:
            self.status = "No saved samples to delete"
            return
        paths = [
            self.images_dir / f"{stem}.png",
            self.labels_dir / f"{stem}.txt",
            self.prompts_dir / f"{stem}.yaml",
        ]
        if kind == "background":
            paths.append(self.previews_dir / f"{stem}_preview.png")
        else:
            paths.extend([
                self.masks_dir / f"{stem}.png",
                self.previews_dir / f"{stem}_overlay.png",
            ])
        deleted_any = False
        for path in paths:
            try:
                if path.exists():
                    path.unlink()
                    deleted_any = True
            except Exception as exc:
                self.get_logger().warn(f"Could not delete YOLO sample artifact {path}: {exc}")

        if self.sample_history and self.sample_history[-1] == entry:
            self.sample_history.pop()
        else:
            self.sample_history = [
                saved for saved in self.sample_history
                if not (saved.get("kind") == kind and saved.get("stem") == stem)
            ]
        if kind == "background":
            self.background_sample_count = max(0, self.background_sample_count - 1)
        else:
            self.sample_count = max(0, self.sample_count - 1)
        self.remove_review_sample_reference(stem)
        self.write_dataset_yaml()
        self.mark_dataset_changed_after_training()
        self.status = (
            f"Deleted {stem}; {self.sample_count_label()}"
            if deleted_any else
            f"{stem} files missing; {self.sample_count_label()}"
        )
        self.save_session()

    def latest_sample_entry(self) -> Optional[Dict[str, str]]:
        if self.sample_history:
            return dict(self.sample_history[-1])

        candidates: List[Tuple[float, Dict[str, str]]] = []
        if self.sample_count > 0:
            stem = f"sample_{self.sample_count:06d}"
            path = self.images_dir / f"{stem}.png"
            if path.exists():
                candidates.append((path.stat().st_mtime, {"kind": "item", "stem": stem}))
        if self.background_sample_count > 0:
            stem = f"background_{self.background_sample_count:06d}"
            path = self.images_dir / f"{stem}.png"
            if path.exists():
                candidates.append((path.stat().st_mtime, {"kind": "background", "stem": stem}))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0])[1]

    def total_training_image_count(self) -> int:
        return self.sample_count + self.background_sample_count

    def background_ratio_percent(self) -> float:
        total = self.total_training_image_count()
        if total <= 0:
            return 0.0
        return round((float(self.background_sample_count) / float(total)) * 100.0, 1)

    def background_ratio_label(self) -> str:
        return f"BG {self.background_ratio_percent():.1f}% target 10-20%"

    def background_ratio_color(self) -> Tuple[int, int, int]:
        ratio = self.background_ratio_percent()
        if 10.0 <= ratio <= 20.0:
            return 184, 224, 194
        return 205, 188, 170

    def has_item_name(self) -> bool:
        return bool(safe_name(self.item_name))

    def item_name_error(self) -> str:
        if not self.item_name.strip():
            return "Enter item name first"
        return "Item name needs letters or numbers"

    def sample_count_label(self) -> str:
        return (
            f"Item {self.sample_count} | BG {self.background_sample_count} | "
            f"Total {self.total_training_image_count()}"
        )

    def mark_dataset_changed_after_training(self) -> None:
        if self.training_status == "done" or self.final_model_dir:
            self.training_status = "idle"
            self.training_epoch_current = 0
            self.training_progress = 0.0

    def start_training(self) -> None:
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Training already running"
            return
        if self.recording_active:
            self.status = "Stop ROI recording before training"
            return
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.sample_count <= 0:
            self.status = "Save at least one sample first"
            return
        train_device = self.effective_train_device()
        self.train_device_used = train_device
        fallback_note = (
            " (GPU requested; CUDA unavailable)"
            if self.train_use_gpu_if_available and train_device == "cpu" else ""
        )
        self.training_status = "training"
        self.training_epoch_current = 0
        self.training_epoch_total = max(1, self.train_epochs)
        self.training_progress = 0.0
        self.status = (
            f"Training YOLO11-seg on {self.sample_count} item + "
            f"{self.background_sample_count} background samples with "
            f"{self.training_device_label(train_device)}{fallback_note}"
        )
        self.save_session()
        self.training_thread = threading.Thread(target=self.train_worker, daemon=True)
        self.training_thread.start()

    def update_training_progress(self, current_epoch: int, total_epochs: int, stage: str = "training") -> None:
        total = max(1, int(total_epochs))
        current = min(max(0, int(current_epoch)), total)
        self.training_epoch_total = total
        self.training_epoch_current = current
        self.training_progress = float(current) / float(total)
        percent = int(round(self.training_progress * 100.0))
        self.training_status = f"{stage} {current}/{total} ({percent}%)"

    def train_worker(self) -> None:
        try:
            from ultralytics import YOLO

            base_model_path = resolve_path(self.yolo_base_model)
            model_ref = str(base_model_path) if base_model_path.exists() else base_model_path.name
            model = YOLO(model_ref)
            train_device = self.effective_train_device()
            self.train_device_used = train_device
            self.get_logger().info(
                f"Starting YOLO11 training on {self.training_device_label(train_device)} "
                f"(device={train_device})")

            def on_train_start(trainer) -> None:
                total_epochs = int(getattr(trainer, "epochs", self.train_epochs))
                self.update_training_progress(0, total_epochs, "training")
                self.status = (
                    f"Training YOLO11-seg on {self.training_device_label(train_device)}: "
                    f"epoch 0/{self.training_epoch_total}"
                )
                self.save_session()

            def on_train_epoch_start(trainer) -> None:
                total_epochs = int(getattr(trainer, "epochs", self.train_epochs))
                epoch_index = int(getattr(trainer, "epoch", 0)) + 1
                completed = max(0, epoch_index - 1)
                self.update_training_progress(completed, total_epochs, "training")
                self.status = (
                    f"Training YOLO11-seg on {self.training_device_label(train_device)}: "
                    f"epoch {epoch_index}/{self.training_epoch_total}"
                )

            def on_fit_epoch_end(trainer) -> None:
                total_epochs = int(getattr(trainer, "epochs", self.train_epochs))
                completed = int(getattr(trainer, "epoch", 0)) + 1
                self.update_training_progress(completed, total_epochs, "training")
                self.status = (
                    f"Training YOLO11-seg on {self.training_device_label(train_device)}: epoch "
                    f"{self.training_epoch_current}/{self.training_epoch_total}"
                )
                self.save_session()

            model.add_callback("on_train_start", on_train_start)
            model.add_callback("on_train_epoch_start", on_train_epoch_start)
            model.add_callback("on_fit_epoch_end", on_fit_epoch_end)
            result = model.train(
                data=str(self.dataset_yaml_path),
                imgsz=self.train_imgsz,
                epochs=self.train_epochs,
                project=str(self.models_dir),
                name="train",
                device=train_device,
                exist_ok=True,
            )
            best_path = Path(result.save_dir) / "weights" / "best.pt"
            self.promote_trained_model(best_path)
            self.update_training_progress(self.training_epoch_total, self.training_epoch_total, "done")
            self.training_status = "done"
            self.status = f"Training done: {self.trained_onnx_path or self.trained_model_path}"
            self.write_profile()
        except Exception as exc:
            self.training_status = f"failed: {exc}"
            self.status = self.training_status
            self.get_logger().error(f"YOLO11 training failed: {exc}")
        finally:
            self.save_session()

    def teach_bundle_stem(self) -> str:
        active_bin = self.active_bin
        bin_suffix = f"_bin_{safe_name(active_bin.bin_name)}" if active_bin else ""
        item_token = safe_name(self.item_name)
        if not item_token:
            raise RuntimeError("Enter item name before finalizing YOLO teach.")
        return f"item_{item_token}{bin_suffix}_{compact_date_for_path()}"

    def unique_model_dir(self) -> Path:
        stem = self.teach_bundle_stem()
        self.profile_dir.mkdir(parents=True, exist_ok=True)
        path = self.profile_dir / stem
        suffix = 1
        while path.exists():
            path = self.profile_dir / f"{stem}_{suffix}"
            suffix += 1
        path.mkdir(parents=True, exist_ok=True)
        return path

    def promote_trained_model(self, best_path: Path) -> None:
        if not best_path.exists():
            raise FileNotFoundError(f"YOLO best.pt missing: {best_path}")

        from ultralytics import YOLO

        model_dir = self.unique_model_dir()
        final_onnx = model_dir / "best.onnx"

        try:
            trained_model = YOLO(str(best_path))
            export_device = self.train_device_used or self.effective_train_device()
            exported = trained_model.export(
                format="onnx",
                imgsz=self.train_imgsz,
                device=export_device,
                dynamic=False,
                simplify=False,
            )
            exported_file = Path(str(exported))
            if exported_file.exists():
                shutil.copy2(exported_file, final_onnx)
            if not final_onnx.exists():
                raise FileNotFoundError(f"YOLO ONNX export did not create {final_onnx}")
        except Exception as exc:
            shutil.rmtree(model_dir, ignore_errors=True)
            raise RuntimeError(f"YOLO ONNX export failed; detect requires ONNX: {exc}") from exc

        self.final_model_dir = str(model_dir)
        self.trained_model_path = ""
        self.trained_onnx_path = str(final_onnx)

    def write_profile(self) -> None:
        if not self.final_model_dir:
            return
        model_dir = Path(self.final_model_dir)
        model_dir.mkdir(parents=True, exist_ok=True)
        active_bin = self.active_bin
        date_match = re.search(r"_(\d{8})(?:_\d+)?$", model_dir.name)
        date_stamp = date_match.group(1) if date_match else compact_date_for_path()
        params = {
            "detection_backend": "yolo11_seg_onnx",
            "item_name": self.item_name,
            "teach_date": date_stamp,
            "class_id": 0,
            "class_name": self.item_name,
            "color_topic": self.color_topic,
            "depth_topic": self.depth_topic,
            "camera_info_topic": self.camera_info_topic,
            "overlay_topic": self.overlay_topic,
            "camera_control_service_root": self.camera_control_service_root,
            "color_exposure_us": self.color_exposure_us,
            "depth_exposure_us": 0,
            "color_exposure_percent": exposure_usec_to_percent(
                self.color_exposure_us,
                self.color_exposure_min_us,
                self.color_exposure_max_us),
            "depth_exposure_percent": 0,
            "color_exposure_min_us": self.color_exposure_min_us,
            "color_exposure_max_us": self.color_exposure_max_us,
            "depth_exposure_min_us": self.depth_exposure_min_us,
            "depth_exposure_max_us": self.depth_exposure_max_us,
            "session_dir": str(self.session_dir),
            "model_dir": self.final_model_dir,
            "model_path": self.trained_onnx_path,
            "model_onnx_path": self.trained_onnx_path,
            "model_pt_path": self.trained_model_path,
            "sample_count": self.sample_count,
            "background_sample_count": self.background_sample_count,
            "total_training_image_count": self.total_training_image_count(),
            "background_ratio_percent": self.background_ratio_percent(),
            "train_epochs": self.train_epochs,
            "train_imgsz": self.train_imgsz,
            "train_device": self.train_device,
            "train_use_gpu_if_available": self.train_use_gpu_if_available,
            "train_device_used": self.train_device_used,
            "trained_model_path": self.trained_model_path,
            "trained_onnx_path": self.trained_onnx_path,
            "associated_bin_name": active_bin.bin_name if active_bin else "",
            "bin_teach_file": str(active_bin.path) if active_bin else "",
            "detection_mode": "yolo_depth",
            "depth_window_mm": 50,
            "align_item_z_axis_to_depth_plane": True,
            "motion_service_root": self.motion_service_root,
            "get_angle_service": self.get_angle_service_name,
            "teach_joints_deg": self.latest_joint_positions_deg if self.has_joint_positions else [],
            "has_teach_joints": self.has_joint_positions,
            "teach_joints_source": self.teach_joints_source,
            "roi_points": [
                int(round(value))
                for point in (active_bin.roi_points if active_bin else [])
                for value in point
            ],
            "roi_crop_rect": list(self.roi_crop_rect) if self.roi_crop_rect else [],
            "depth_plane": active_bin.depth_plane if active_bin else {},
        }
        if active_bin:
            depth_plane = active_bin.depth_plane
            params["depth_plane_source"] = "bin_teach"
            params["depth_plane_enabled"] = bool(depth_plane.get("depth_plane_enabled", False))
            params["depth_plane_a"] = float(depth_plane.get("depth_plane_a", 0.0))
            params["depth_plane_b"] = float(depth_plane.get("depth_plane_b", 0.0))
            params["depth_plane_c"] = float(depth_plane.get("depth_plane_c", 0.0))
            params["depth_plane_reference_depth_m"] = float(
                depth_plane.get("depth_plane_reference_depth_m", 0.0))
        profile = {
            "item_detect": {"ros__parameters": params},
            "item_yolo": {"ros__parameters": params},
        }
        profile_path = model_dir / f"{model_dir.name}.yaml"
        profile_path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
        self.latest_profile_path = str(profile_path)

    def render_mask_overlay(self, image: np.ndarray, mask: Optional[np.ndarray]) -> np.ndarray:
        if mask is None or not self.overlay_enabled:
            return image
        overlay = image.copy()
        overlay[mask > 0] = (70, 210, 80)
        output = cv2.addWeighted(overlay, 0.45, image, 0.55, 0.0)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, (60, 255, 120), 2)
        return output

    def draw_roi_capture_count_overlay(self, image: np.ndarray) -> None:
        if image.size == 0:
            return
        label = f"ROI captures: {self.roi_image_capture_count}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.55, min(0.95, image.shape[1] / 1800.0))
        thickness = 2
        text_size, baseline = cv2.getTextSize(label, font, scale, thickness)
        pad_x = 14
        pad_y = 10
        x = 14
        y = 14
        width = min(image.shape[1] - x - 8, text_size[0] + pad_x * 2)
        height = min(image.shape[0] - y - 8, text_size[1] + baseline + pad_y * 2)
        if width <= 0 or height <= 0:
            return
        roi = image[y:y + height, x:x + width]
        if roi.size == 0:
            return
        panel = np.full_like(roi, (18, 22, 26))
        cv2.addWeighted(panel, 0.70, roi, 0.30, 0.0, roi)
        cv2.rectangle(image, (x, y), (x + width, y + height), (92, 122, 142), 1, cv2.LINE_AA)
        text_x = x + pad_x
        text_y = y + pad_y + text_size[1]
        cv2.putText(image, label, (text_x, text_y), font, scale, (238, 246, 250), thickness, cv2.LINE_AA)

    def draw_review_frame_sample_overlay(self, image: np.ndarray) -> None:
        if image.size == 0 or not self.review_mode:
            return
        count = self.review_frame_sample_count()
        label = f"Samples here: {count}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.55, min(0.95, image.shape[1] / 1800.0))
        thickness = 2
        text_size, baseline = cv2.getTextSize(label, font, scale, thickness)
        pad_x = 14
        pad_y = 10
        width = text_size[0] + pad_x * 2
        height = text_size[1] + baseline + pad_y * 2
        x = max(8, image.shape[1] - width - 14)
        y = 14
        width = min(width, image.shape[1] - x - 8)
        height = min(height, image.shape[0] - y - 8)
        if width <= 0 or height <= 0:
            return
        roi = image[y:y + height, x:x + width]
        if roi.size == 0:
            return
        panel = np.full_like(roi, (18, 22, 26))
        cv2.addWeighted(panel, 0.70, roi, 0.30, 0.0, roi)
        cv2.rectangle(image, (x, y), (x + width, y + height), (92, 122, 142), 1, cv2.LINE_AA)
        text_x = x + pad_x
        text_y = y + pad_y + text_size[1]
        cv2.putText(image, label, (text_x, text_y), font, scale, (238, 246, 250), thickness, cv2.LINE_AA)

    def mask_in_frame(self, frame_shape: Tuple[int, int, int]) -> Optional[np.ndarray]:
        if self.current_mask is None or self.roi_crop_rect is None:
            return None
        x0, y0, rect_w, rect_h = self.roi_crop_rect
        frame_h, frame_w = frame_shape[:2]
        if x0 >= frame_w or y0 >= frame_h:
            return None
        patch_w = min(rect_w, self.current_mask.shape[1], frame_w - x0)
        patch_h = min(rect_h, self.current_mask.shape[0], frame_h - y0)
        if patch_w <= 0 or patch_h <= 0:
            return None
        full_mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
        full_mask[y0:y0 + patch_h, x0:x0 + patch_w] = self.current_mask[:patch_h, :patch_w]
        return full_mask

    def crop_point_to_frame(self, point: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        if self.roi_crop_rect is None:
            return None
        x0, y0, _, _ = self.roi_crop_rect
        return x0 + point[0], y0 + point[1]

    def draw_button(self, canvas: np.ndarray, name: str, rect: Tuple[int, int, int, int],
                    label: str, enabled: bool = True, active: bool = False,
                    role: str = "default") -> None:
        x, y, w, h = rect
        if role == "save":
            fill = (38, 82, 55) if active or enabled else (36, 56, 44)
            border = (118, 214, 146) if active or enabled else (72, 114, 84)
            text = (230, 248, 234) if enabled else (136, 162, 142)
        elif role == "danger":
            fill = (82, 48, 46) if active or enabled else (58, 43, 42)
            border = (224, 142, 132) if active or enabled else (120, 82, 78)
            text = (250, 232, 228) if enabled else (162, 136, 132)
        elif active:
            fill = (70, 126, 186)
            border = (126, 202, 255)
            text = (238, 242, 245)
        elif enabled:
            fill = (48, 64, 76)
            border = (170, 210, 240)
            text = (238, 242, 245)
        else:
            fill = (52, 52, 52)
            border = (102, 106, 112)
            text = (150, 150, 150)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2)
        cv2.putText(canvas, fit_text(label, max(6, w // 11)), (x + 10, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, text, 1, cv2.LINE_AA)
        self.buttons[name] = Button(name, rect, enabled, role)

    def draw_progress_button(self, canvas: np.ndarray, name: str, rect: Tuple[int, int, int, int],
                             label: str, progress: float, enabled: bool = True) -> None:
        x, y, w, h = rect
        clamped = min(max(float(progress), 0.0), 1.0)
        fill = (42, 50, 58) if enabled else (52, 52, 52)
        progress_fill = (64, 138, 92)
        border = (126, 202, 255) if enabled else (102, 106, 112)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
        if clamped > 0.0:
            fill_w = int(round(w * clamped))
            cv2.rectangle(canvas, (x, y), (x + fill_w, y + h), progress_fill, -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2)
        cv2.putText(canvas, fit_text(label, max(6, w // 11)), (x + 10, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (238, 242, 245), 1, cv2.LINE_AA)
        self.buttons[name] = Button(name, rect, enabled)

    def draw_gpu_training_slider(self, canvas: np.ndarray, name: str,
                                 rect: Tuple[int, int, int, int],
                                 enabled: bool = True) -> None:
        x, y, w, h = rect
        active = self.train_use_gpu_if_available
        label_color = (220, 230, 238) if enabled else (145, 150, 156)
        state_color = (184, 224, 194) if active and enabled else (186, 191, 198)
        cv2.putText(canvas, "CUDA", (x, y + 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, label_color, 1, cv2.LINE_AA)

        track_x = x + 72
        track_y = y + 2
        track_w = min(128, max(96, w - 72))
        track_h = max(24, min(30, h - 4))
        radius = track_h // 2
        fill = (52, 150, 98) if active and enabled else (70, 76, 84)
        border = (128, 230, 168) if active and enabled else (116, 126, 136)
        if not enabled:
            fill = (54, 58, 64)
            border = (90, 96, 104)

        cv2.rectangle(
            canvas,
            (track_x + radius, track_y),
            (track_x + track_w - radius, track_y + track_h),
            fill,
            -1,
        )
        cv2.circle(canvas, (track_x + radius, track_y + radius), radius, fill, -1, cv2.LINE_AA)
        cv2.circle(canvas, (track_x + track_w - radius, track_y + radius), radius, fill, -1, cv2.LINE_AA)
        cv2.ellipse(canvas, (track_x + radius, track_y + radius), (radius, radius),
                    90, 0, 180, border, 1, cv2.LINE_AA)
        cv2.ellipse(canvas, (track_x + track_w - radius, track_y + radius), (radius, radius),
                    270, 0, 180, border, 1, cv2.LINE_AA)
        cv2.line(canvas, (track_x + radius, track_y), (track_x + track_w - radius, track_y),
                 border, 1, cv2.LINE_AA)
        cv2.line(canvas, (track_x + radius, track_y + track_h), (track_x + track_w - radius, track_y + track_h),
                 border, 1, cv2.LINE_AA)

        knob_radius = max(8, radius - 4)
        knob_x = track_x + track_w - radius if active else track_x + radius
        knob_y = track_y + radius
        cv2.circle(canvas, (knob_x, knob_y), knob_radius, (246, 248, 250), -1, cv2.LINE_AA)
        cv2.circle(canvas, (knob_x, knob_y), knob_radius, (186, 196, 204), 1, cv2.LINE_AA)
        cv2.putText(canvas, "ON" if active else "OFF",
                    (track_x + 36, y + 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (238, 244, 240) if active and enabled else (196, 202, 210),
                    1, cv2.LINE_AA)

        status_text = "fallback CPU"
        if active and self.cuda_training_available():
            status_text = "GPU ready"
        elif not active:
            status_text = "CPU"
        cv2.putText(canvas, status_text, (track_x + track_w + 12, y + 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, state_color, 1, cv2.LINE_AA)
        self.buttons[name] = Button(name, rect, enabled)

    def draw_exposure_slider(self, panel: np.ndarray, y: int) -> int:
        label = "RGB Exposure us"
        value_text = "auto" if self.color_exposure_us <= 0 else f"{self.color_exposure_us} us"
        track_x = PANEL_PAD
        track_y = y + 30
        track_w = LEFT_PANEL_WIDTH - 2 * PANEL_PAD
        track_h = 12

        cv2.putText(panel, label, (track_x, track_y - 10), cv2.FONT_HERSHEY_DUPLEX,
                    0.45, (214, 218, 224), 1, cv2.LINE_AA)
        cv2.putText(panel, value_text, (track_x + track_w - 54, track_y - 10),
                    cv2.FONT_HERSHEY_DUPLEX, 0.43, (186, 191, 198), 1, cv2.LINE_AA)

        self.exposure_slider_rect = (track_x, track_y - 8, track_w, track_h + 16)
        cv2.rectangle(panel, (track_x, track_y), (track_x + track_w, track_y + track_h),
                      (67, 72, 78), -1)
        cv2.rectangle(panel, (track_x, track_y), (track_x + track_w, track_y + track_h),
                      (92, 99, 107), 1)

        t = float(self.color_exposure_us) / float(max(1, self.color_exposure_max_us))
        t = min(max(t, 0.0), 1.0)
        fill_w = int(round(track_w * t))
        knob_x = track_x + fill_w
        knob_y = track_y + track_h // 2
        accent = (130, 170, 200) if self.color_exposure_us <= 0 else (108, 206, 224)
        cv2.line(panel, (track_x + 1, knob_y), (track_x + track_w - 1, knob_y),
                 accent, 2, cv2.LINE_AA)
        cv2.circle(panel, (knob_x, knob_y), 9, (245, 245, 245), -1, cv2.LINE_AA)
        cv2.circle(panel, (knob_x, knob_y), 9, (96, 100, 106), 1, cv2.LINE_AA)
        return y + 62

    def update_color_exposure_from_x(self, x: int, save: bool) -> None:
        sx, sy, sw, sh = self.exposure_slider_rect
        if sw <= 0:
            return
        clamped_x = min(max(x, sx), sx + sw)
        ratio = float(clamped_x - sx) / float(max(1, sw))
        new_value = int(round(ratio * float(self.color_exposure_max_us)))
        new_value = clamp_exposure_usec_or_auto(
            new_value,
            self.color_exposure_min_us,
            self.color_exposure_max_us)
        if new_value != self.color_exposure_us:
            self.color_exposure_us = new_value
            self.mark_camera_exposure_dirty()
            self.status = f"RGB exposure: {self.exposure_mode_text(self.color_exposure_us)}"
        if save:
            self.save_runtime_settings()
            self.save_session()

    def handle_exposure_slider_mouse(self, event: int, x: int, y: int) -> bool:
        if self.bin_dropdown_open and not self.exposure_slider_active:
            return False
        sx, sy, sw, sh = self.exposure_slider_rect
        inside = sx <= x <= sx + sw and sy <= y <= sy + sh
        if event == cv2.EVENT_LBUTTONDOWN and inside:
            self.exposure_slider_active = True
            self.update_color_exposure_from_x(x, save=True)
            return True
        if event == cv2.EVENT_MOUSEMOVE and self.exposure_slider_active:
            self.update_color_exposure_from_x(x, save=False)
            return True
        if event == cv2.EVENT_LBUTTONUP and self.exposure_slider_active:
            self.update_color_exposure_from_x(x, save=True)
            self.exposure_slider_active = False
            return True
        return False

    def build_no_camera_topics_placeholder(self) -> np.ndarray:
        image = np.zeros((PREVIEW_CANVAS_HEIGHT, PREVIEW_CANVAS_WIDTH, 3), dtype=np.uint8)
        image[:] = (18, 20, 24)
        cv2.putText(
            image,
            "no camera topics...",
            (44, 96),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.15,
            (0, 210, 255),
            3,
            cv2.LINE_AA,
        )

        status_lines = [
            f"color: {self.color_topic}  publishers={self.count_publishers(self.color_topic)}",
            f"depth: {self.depth_topic}  publishers={self.count_publishers(self.depth_topic)}",
            f"info:  {self.camera_info_topic}  publishers={self.count_publishers(self.camera_info_topic)}",
        ]
        y = 158
        for line in status_lines:
            cv2.putText(
                image,
                line,
                (48, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (225, 230, 235),
                2,
                cv2.LINE_AA,
            )
            y += 40
        return image

    def update_view(self) -> None:
        if self.prediction_dirty and not self.predicting:
            self.run_sam2_prediction()

        frame = None
        recording_roi = None
        if self.frozen_crop_bgr is not None:
            display_frame = self.frozen_frame_bgr.copy() if self.frozen_frame_bgr is not None else self.frozen_crop_bgr.copy()
        else:
            frame = self.latest_frame_copy()
            if frame is not None and self.active_bin is not None and self.recording_active:
                recording_roi = self.roi_views_from_frame(frame)
                if recording_roi is not None:
                    _, record_rect, _, record_frame_view, _ = recording_roi
                    self.record_roi_frame(record_frame_view, record_rect)
            if frame is None:
                display_frame = self.build_no_camera_topics_placeholder()
            elif frame is not None and self.active_bin is not None and self.live_view_enabled:
                live_roi = recording_roi if recording_roi is not None else self.roi_views_from_frame(frame)
                if live_roi is not None:
                    _, rect, roi_mask, display_frame, _ = live_roi
                    self.roi_crop_rect = rect
                    self.roi_crop_mask = roi_mask
                else:
                    display_frame = frame.copy()
            elif frame is not None and self.live_view_enabled:
                display_frame = frame.copy()
            else:
                display_frame = np.zeros((480, 848, 3), dtype=np.uint8)

        self.preview_source_size = (display_frame.shape[1], display_frame.shape[0])
        display_mask = self.mask_in_frame(display_frame.shape)
        display_frame = self.render_mask_overlay(display_frame.copy(), display_mask)
        for point in self.positive_points:
            frame_point = self.crop_point_to_frame(point)
            if frame_point is not None:
                cv2.circle(display_frame, frame_point, 6, (70, 230, 70), -1)
                cv2.circle(display_frame, frame_point, 8, (20, 80, 20), 1)
        for point in self.negative_points:
            frame_point = self.crop_point_to_frame(point)
            if frame_point is not None:
                cv2.circle(display_frame, frame_point, 6, (60, 70, 240), -1)
                cv2.circle(display_frame, frame_point, 8, (20, 20, 120), 1)
        self.draw_roi_capture_count_overlay(display_frame)
        self.draw_review_frame_sample_overlay(display_frame)

        preview, scale = self.fit_preview(display_frame)
        preview_w, preview_h = preview.shape[1], preview.shape[0]
        canvas_h = max(VIDEO_TOP_BAR_HEIGHT + preview_h, VIDEO_TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT)
        canvas_w = LEFT_PANEL_WIDTH + preview_w
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        canvas[:] = (18, 22, 26)

        self.preview_scale = scale
        self.preview_image_size = (preview_w, preview_h)
        preview_x = LEFT_PANEL_WIDTH
        preview_y = VIDEO_TOP_BAR_HEIGHT
        canvas[preview_y:preview_y + preview_h, preview_x:preview_x + preview_w] = preview
        self.preview_rect = (preview_x, preview_y, preview_w, preview_h)

        self.draw_panel(canvas)
        self.draw_video_bar(canvas)
        self.resize_window_if_needed(canvas_w, canvas_h)
        cv2.imshow(WINDOW_NAME, canvas)
        self.handle_key(cv2.waitKey(1))

    def preview_canvas_size_for_source(self, image: np.ndarray) -> Tuple[int, int]:
        h, w = image.shape[:2]
        requested_scale = self.display_scale if self.display_scale > 0.0 else 1.0
        if w <= 0 or h <= 0:
            return PREVIEW_CANVAS_WIDTH, PREVIEW_CANVAS_HEIGHT
        preview_w = max(
            PREVIEW_CANVAS_WIDTH,
            int(round(float(w) * requested_scale)),
        )
        preview_h = max(1, int(round(float(preview_w) * float(h) / float(w))))
        return preview_w, preview_h

    def fit_preview(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        h, w = image.shape[:2]
        if h <= 0 or w <= 0:
            return np.zeros((PREVIEW_CANVAS_HEIGHT, PREVIEW_CANVAS_WIDTH, 3), dtype=np.uint8), 1.0
        preview_w, preview_h = self.preview_canvas_size_for_source(image)
        scale = min(preview_w / float(w), preview_h / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        preview = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        return preview, scale

    def resize_window_if_needed(self, width: int, height: int) -> None:
        window_size = (int(width), int(height))
        if window_size == self.rendered_window_size:
            return
        try:
            cv2.resizeWindow(WINDOW_NAME, window_size[0], window_size[1])
            self.rendered_window_size = window_size
        except cv2.error as exc:
            self.get_logger().warn(f"Could not resize YOLO teach window: {exc}")

    def draw_section_header(self, panel: np.ndarray, title: str, y: int) -> int:
        cv2.putText(panel, title, (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.54, (235, 238, 242), 1, cv2.LINE_AA)
        text_size, _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.54, 1)
        line_x = PANEL_PAD + text_size[0] + 12
        if line_x < LEFT_PANEL_WIDTH - PANEL_PAD:
            cv2.line(panel, (line_x, y - 6), (LEFT_PANEL_WIDTH - PANEL_PAD, y - 6),
                     (78, 84, 92), 1, cv2.LINE_AA)
        return y + 16

    def compact_session_path_label(self) -> str:
        try:
            return str(self.session_dir.resolve().relative_to(workspace_root()))
        except Exception:
            return str(self.session_dir)

    def draw_panel(self, canvas: np.ndarray) -> None:
        panel = canvas[:, :LEFT_PANEL_WIDTH]
        panel[:] = (28, 31, 36)
        self.buttons.clear()

        cv2.putText(panel, "YOLO Teach", (PANEL_PAD, 36), cv2.FONT_HERSHEY_SIMPLEX,
                    0.82, (240, 244, 248), 2, cv2.LINE_AA)
        cv2.putText(panel, self.sample_count_label(), (PANEL_PAD, 62), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (178, 214, 190), 1, cv2.LINE_AA)

        y = 88
        y = self.draw_section_header(panel, "Setup", y)
        cv2.putText(panel, "Item Name", (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.46, (192, 202, 212), 1, cv2.LINE_AA)
        input_y = y + 10
        input_w = LEFT_PANEL_WIDTH - 2 * PANEL_PAD
        self.item_name_input_rect = (PANEL_PAD, input_y, input_w, BUTTON_HEIGHT)
        input_fill = (42, 58, 68) if self.item_name_edit_active else (38, 44, 52)
        input_border = (126, 202, 255) if self.item_name_edit_active else (112, 132, 148)
        cv2.rectangle(panel, (PANEL_PAD, input_y), (PANEL_PAD + input_w, input_y + BUTTON_HEIGHT), input_fill, -1)
        cv2.rectangle(panel, (PANEL_PAD, input_y), (PANEL_PAD + input_w, input_y + BUTTON_HEIGHT), input_border, 2)
        item_text = self.item_name_edit_buffer if self.item_name_edit_active else self.item_name
        show_item_placeholder = not item_text.strip() and not self.item_name_edit_active
        if show_item_placeholder:
            item_text = "Enter item name"
        if self.item_name_edit_active:
            item_text += "|"
        item_color = (150, 156, 164) if show_item_placeholder else (238, 242, 245)
        cv2.putText(panel, fit_text(item_text, 34), (PANEL_PAD + 10, input_y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.56, item_color, 1, cv2.LINE_AA)

        y = input_y + BUTTON_HEIGHT + 12
        self.draw_button(
            panel,
            "bin_dropdown",
            (PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, BUTTON_HEIGHT),
            self.bin_dropdown_label(),
            bool(self.bin_entries),
            self.bin_dropdown_open,
        )
        y += BUTTON_HEIGHT + 10

        y = self.draw_exposure_slider(panel, y)
        y += 6

        if self.review_mode:
            y = self.draw_section_header(panel, "Annotation", y + 8)
            prompt_text = f"Prompts  +{len(self.positive_points)}  -{len(self.negative_points)}"
            sample_text = f"Frame samples  {self.review_frame_sample_count()}"
            cv2.putText(panel, prompt_text, (PANEL_PAD, y + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (205, 218, 226), 1, cv2.LINE_AA)
            cv2.putText(panel, sample_text, (PANEL_PAD + 210, y + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (184, 224, 194), 1, cv2.LINE_AA)
            y += 36

        y = self.draw_section_header(panel, "Capture", y + 8)
        training_active = self.training_thread is not None and self.training_thread.is_alive()
        record_enabled = self.recording_active or (
            self.has_item_name() and self.active_bin is not None and not training_active
        )
        record_label = "Stop Capture" if self.recording_active else "Capture ROI"
        self.draw_button(
            panel,
            "toggle_video_recording",
            (PANEL_PAD, y, 190, BUTTON_HEIGHT),
            record_label,
            record_enabled,
            self.recording_active,
            role="danger" if self.recording_active else "save",
        )
        review_enabled = not self.recording_active and not training_active
        self.draw_button(
            panel,
            "toggle_video_review",
            (PANEL_PAD + 210, y, 190, BUTTON_HEIGHT),
            "Exit Review" if self.review_mode else "Review Images",
            review_enabled or self.review_mode,
            self.review_mode,
        )
        y += BUTTON_HEIGHT + 8
        video_status = (
            f"Capturing ROI: {self.recording_frame_count} frames"
            if self.recording_active else
            self.review_status_label() if self.review_mode else
            f"{len(self.video_recordings)} images saved"
        )
        video_lines = self.draw_wrapped_text(
            panel, video_status, PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, 2, 0.41)
        y += max(24, video_lines * 20 + 4)
        if self.review_mode:
            y += 8

        y = self.draw_section_header(panel, "Dataset", y + 8)
        cv2.putText(panel, f"Item {self.sample_count}", (PANEL_PAD, y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (190, 232, 190), 1, cv2.LINE_AA)
        cv2.putText(panel, f"BG {self.background_sample_count}", (PANEL_PAD + 118, y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, self.background_ratio_color(), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Total {self.total_training_image_count()}", (PANEL_PAD + 222, y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (205, 218, 226), 1, cv2.LINE_AA)
        cv2.putText(panel, self.background_ratio_label(), (PANEL_PAD, y + 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, self.background_ratio_color(), 1, cv2.LINE_AA)
        joint_status = "Teach Position: captured" if self.has_joint_positions else "Teach Position: waiting"
        cv2.putText(panel, joint_status, (PANEL_PAD, y + 66),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                    (184, 224, 194) if self.has_joint_positions else (205, 188, 170),
                    1,
                    cv2.LINE_AA)
        y += 76

        y = self.draw_section_header(panel, "Training", y + 8)
        self.draw_gpu_training_slider(
            panel,
            "toggle_train_gpu",
            (PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, 32),
            not training_active,
        )
        y += 32 + 8
        train_rect = (PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, BUTTON_HEIGHT)
        if training_active:
            train_label = (
                f"Training {self.training_epoch_current}/{self.training_epoch_total} "
                f"({int(round(self.training_progress * 100.0))}%)"
            )
            self.draw_progress_button(panel, "train_yolo", train_rect, train_label, self.training_progress, True)
        elif self.training_status == "done":
            self.draw_progress_button(
                panel, "train_yolo", train_rect, "Train YOLO11: done", 1.0,
                self.sample_count > 0 and self.has_item_name() and not self.recording_active)
        else:
            self.draw_button(panel, "train_yolo", train_rect, "Train YOLO11",
                             self.sample_count > 0 and self.has_item_name() and not self.recording_active)
        y += BUTTON_HEIGHT + 10

        y = self.draw_section_header(panel, "Status", y + 8)
        cv2.putText(panel, f"State: {fit_text(self.training_status, 32)}", (PANEL_PAD, y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (205, 218, 226), 1, cv2.LINE_AA)
        y += 28
        status_lines = self.draw_wrapped_text(
            panel, self.status, PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, 3, 0.43)
        y += max(26, status_lines * 20 + 6)

        y = self.draw_section_header(panel, "Session", y + 8)
        session_lines = self.draw_wrapped_text(
            panel, self.compact_session_path_label(), PANEL_PAD, y,
            LEFT_PANEL_WIDTH - 2 * PANEL_PAD, 2, 0.38)
        y += max(24, session_lines * 20 + 4)

        if self.bin_dropdown_open:
            self.draw_bin_dropdown(panel)

    def commit_item_name_edit(self) -> None:
        new_name = self.item_name_edit_buffer.strip()
        if not safe_name(new_name):
            self.status = "Item name cannot be empty" if not new_name else "Item name needs letters or numbers"
            self.item_name_edit_buffer = self.item_name
            self.item_name_edit_active = False
            return
        if new_name != self.item_name:
            if self.training_thread is not None and self.training_thread.is_alive():
                self.status = "Cannot change item name while training"
                self.item_name_edit_buffer = self.item_name
                self.item_name_edit_active = False
                return
            if self.recording_active:
                self.status = "Cannot change item name while capturing ROI"
                self.item_name_edit_buffer = self.item_name
                self.item_name_edit_active = False
                return
            self.reset_runtime_for_item_name(new_name)
        self.item_name_edit_active = False

    def cancel_item_name_edit(self) -> None:
        self.item_name_edit_buffer = self.item_name
        self.item_name_edit_active = False
        self.status = "Item name edit canceled"

    def handle_key(self, key: int) -> None:
        if key < 0:
            return
        code = key & 0xFF
        if not self.item_name_edit_active:
            if code == 32:
                now = time.monotonic()
                if now - self.last_space_capture_time < 0.35:
                    return
                self.last_space_capture_time = now
                self.capture_roi_image()
            return

        if code in (10, 13):
            self.commit_item_name_edit()
            return
        if code == 27:
            self.cancel_item_name_edit()
            return
        if code in (8, 127):
            self.item_name_edit_buffer = self.item_name_edit_buffer[:-1]
            return
        if 32 <= code <= 126 and len(self.item_name_edit_buffer) < 64:
            self.item_name_edit_buffer += chr(code)

    def draw_video_bar(self, canvas: np.ndarray) -> None:
        bar = canvas[:VIDEO_TOP_BAR_HEIGHT, LEFT_PANEL_WIDTH:]
        bar[:] = (24, 28, 34)
        self.saved_session_option_rects.clear()
        self.saved_session_delete_rects.clear()
        x = LEFT_PANEL_WIDTH + 16
        y = 16
        if self.review_mode:
            frame_count = len(self.review_frame_records())
            self.draw_button(canvas, "review_prev_frame", (x, y, 148, BUTTON_HEIGHT),
                             "Previous Frame", frame_count > 1)
            x += 158
            self.draw_button(canvas, "review_next_frame", (x, y, 120, BUTTON_HEIGHT),
                             "Next Frame", frame_count > 1)
            x += 130
            self.draw_button(canvas, "clear_prompts", (x, y, 132, BUTTON_HEIGHT),
                             "Clear Prompt", self.has_any_prompt() or self.current_mask is not None, role="danger")
            x += 142
            frame = self.current_review_frame_record()
            frame_status = str(frame.get("status", "pending")) if frame else "pending"
            save_enabled = (
                self.has_item_name()
                and (self.current_mask is not None or self.has_any_prompt())
                and (frame_status != "annotated" or self.has_any_prompt())
            )
            self.draw_button(canvas, "save_review_frame", (x, y, 176, BUTTON_HEIGHT),
                             "Save Sample", save_enabled, role="save")
            x += 186
            bg_enabled = self.has_item_name() and self.frozen_crop_bgr is not None
            self.draw_button(canvas, "save_review_background_frame", (x, y, 112, BUTTON_HEIGHT),
                             "Save BG", bg_enabled, role="save")
            x += 122
            self.draw_button(canvas, "delete_review_frame", (x, y, 120, BUTTON_HEIGHT),
                             "Del Frame", frame_count > 0, role="danger")
            status = f"Reviewing ROI image | {self.review_status_label()}"
            if self.preview_source_size != (0, 0):
                status += f" | {self.preview_source_size[0]}x{self.preview_source_size[1]}"
            if self.predicting:
                status += " | SAM2 running"
            cv2.putText(canvas, status, (LEFT_PANEL_WIDTH + 16, 74), cv2.FONT_HERSHEY_SIMPLEX,
                        0.56, (216, 224, 232), 1, cv2.LINE_AA)
            return

        self.draw_button(canvas, "live_view", (x, y, 120, BUTTON_HEIGHT),
                         "Live: ON" if self.live_view_enabled else "Live: OFF",
                         True, self.live_view_enabled)
        x += 134
        self.draw_button(canvas, "overlay", (x, y, 142, BUTTON_HEIGHT),
                         "Overlay: ON" if self.overlay_enabled else "Overlay: OFF",
                         True, self.overlay_enabled)
        x += 156
        training_active = self.training_thread is not None and self.training_thread.is_alive()
        self.draw_button(canvas, "save_session", (x, y, 150, BUTTON_HEIGHT),
                         "Save Session", not training_active and not self.recording_active, role="save")
        x += 164
        load_enabled = bool(self.saved_session_entries) and not training_active and not self.recording_active
        load_label = "Load Session" if self.saved_session_entries else "Load: none"
        self.draw_button(canvas, "load_session", (x, y, 170, BUTTON_HEIGHT),
                         load_label, load_enabled, self.load_session_dropdown_open)
        status = "Full frame ROI view"
        if self.recording_active:
            status = f"Capturing ROI | {self.recording_frame_count} frames"
        elif self.review_mode:
            status = f"Reviewing ROI image | {self.review_status_label()}"
        if self.active_bin:
            status += f" | {self.active_bin.bin_name}"
        if self.preview_source_size != (0, 0):
            status += f" | {self.preview_source_size[0]}x{self.preview_source_size[1]}"
        if self.predicting:
            status += " | SAM2 running"
        cv2.putText(canvas, status, (LEFT_PANEL_WIDTH + 16, 74), cv2.FONT_HERSHEY_SIMPLEX,
                    0.56, (216, 224, 232), 1, cv2.LINE_AA)
        if self.load_session_dropdown_open:
            self.draw_saved_session_dropdown(canvas)

    def draw_saved_session_dropdown(self, canvas: np.ndarray) -> None:
        button = self.buttons.get("load_session")
        if button is None:
            return
        x, y, w, h = button.rect
        dropdown_w = 560
        row_h = 38
        rows = min(MAX_SESSION_DROPDOWN_ROWS, len(self.saved_session_entries))
        if rows <= 0:
            return
        dropdown_x = min(x, canvas.shape[1] - dropdown_w - 8)
        dropdown_y = y + h + 4
        for index in range(rows):
            entry = self.saved_session_entries[index]
            row_y = dropdown_y + index * row_h
            is_current = False
            try:
                is_current = entry.path.resolve() == self.session_dir.resolve()
            except Exception:
                is_current = False
            fill = (56, 78, 72) if is_current else (38, 43, 50)
            border = (128, 224, 168) if is_current else (102, 116, 126)
            rect = (dropdown_x, row_y, dropdown_w, row_h)
            delete_rect = (dropdown_x + dropdown_w - 62, row_y + 5, 54, row_h - 10)
            self.saved_session_option_rects.append((rect[0], rect[1], rect[2], rect[3], index))
            self.saved_session_delete_rects.append((
                delete_rect[0], delete_rect[1], delete_rect[2], delete_rect[3], index))
            cv2.rectangle(canvas, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), fill, -1)
            cv2.rectangle(canvas, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), border, 1)
            cv2.putText(canvas, fit_text(entry.label, 58), (rect[0] + 10, rect[1] + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.47, (236, 240, 244), 1, cv2.LINE_AA)
            cv2.rectangle(
                canvas,
                (delete_rect[0], delete_rect[1]),
                (delete_rect[0] + delete_rect[2], delete_rect[1] + delete_rect[3]),
                (88, 48, 46),
                -1,
            )
            cv2.rectangle(
                canvas,
                (delete_rect[0], delete_rect[1]),
                (delete_rect[0] + delete_rect[2], delete_rect[1] + delete_rect[3]),
                (222, 134, 126),
                1,
            )
            cv2.putText(canvas, "Del", (delete_rect[0] + 12, delete_rect[1] + 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (250, 232, 228), 1, cv2.LINE_AA)

    def draw_wrapped_text(self, image: np.ndarray, text: str, x: int, y: int,
                          width: int, max_lines: int, scale: float = 0.48) -> int:
        max_chars = max(10, width // 9)
        words = text.split()
        lines: List[str] = []
        current = ""
        for word in words:
            trial = f"{current} {word}".strip()
            if len(trial) <= max_chars:
                current = trial
            else:
                if current:
                    lines.append(current)
                current = word
            if len(lines) >= max_lines:
                break
        if current and len(lines) < max_lines:
            lines.append(current)
        drawn_lines = lines[:max_lines]
        for i, line in enumerate(drawn_lines):
            if i == max_lines - 1 and len(lines) == max_lines and len(line) > max_chars - 3:
                line = line[:max_chars - 3] + "..."
            cv2.putText(image, line, (x, y + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (190, 202, 212), 1, cv2.LINE_AA)
        return max(1, len(drawn_lines))

    def bin_dropdown_label(self) -> str:
        if self.active_bin is not None:
            return f"Load Bin Teach: {self.active_bin.bin_name}"
        if not self.bin_entries:
            return "Load Bin Teach: no files"
        return "Load Bin Teach: choose file"

    def draw_bin_dropdown(self, panel: np.ndarray) -> None:
        button = self.buttons.get("bin_dropdown")
        if button is None:
            return
        x, y, w, h = button.rect
        rows = min(MAX_DROPDOWN_ROWS, len(self.bin_entries))
        for i in range(rows):
            row_y = y + h + 2 + i * DROPDOWN_ROW_HEIGHT
            fill = (42, 50, 58) if i != self.active_bin_index else (58, 82, 78)
            cv2.rectangle(panel, (x, row_y), (x + w, row_y + DROPDOWN_ROW_HEIGHT), fill, -1)
            cv2.rectangle(panel, (x, row_y), (x + w, row_y + DROPDOWN_ROW_HEIGHT), (120, 140, 150), 1)
            cv2.putText(panel, fit_text(self.bin_entries[i].label, 44), (x + 8, row_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.47, (238, 242, 245), 1, cv2.LINE_AA)

    def mouse_callback(self, event, x: int, y: int, flags, param) -> None:
        if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP):
            self.handle_exposure_slider_mouse(event, x, y)
            return
        if event not in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            return
        if event == cv2.EVENT_LBUTTONDOWN and self.handle_exposure_slider_mouse(event, x, y):
            return
        if event == cv2.EVENT_LBUTTONDOWN and self.handle_button_click(x, y):
            return
        if event == cv2.EVENT_RBUTTONDOWN and (x < LEFT_PANEL_WIDTH or y < VIDEO_TOP_BAR_HEIGHT):
            return
        self.handle_prompt_click(event, x, y)

    def handle_button_click(self, x: int, y: int) -> bool:
        if self.load_session_dropdown_open:
            for rx, ry, rw, rh, index in list(self.saved_session_delete_rects):
                if rx <= x <= rx + rw and ry <= y <= ry + rh:
                    self.delete_saved_session_by_index(index)
                    return True
            for rx, ry, rw, rh, index in list(self.saved_session_option_rects):
                if rx <= x <= rx + rw and ry <= y <= ry + rh:
                    self.load_saved_session_by_index(index)
                    return True
            load_button = self.buttons.get("load_session")
            inside_load_button = False
            if load_button is not None:
                bx, by, bw, bh = load_button.rect
                inside_load_button = bx <= x <= bx + bw and by <= y <= by + bh
            if not inside_load_button:
                self.load_session_dropdown_open = False
                return True

        if self.bin_dropdown_open:
            button = self.buttons.get("bin_dropdown")
            if button is not None:
                bx, by, bw, bh = button.rect
                if bx <= x <= bx + bw and by + bh + 2 <= y <= by + bh + 2 + MAX_DROPDOWN_ROWS * DROPDOWN_ROW_HEIGHT:
                    row = (y - (by + bh + 2)) // DROPDOWN_ROW_HEIGHT
                    if 0 <= row < min(MAX_DROPDOWN_ROWS, len(self.bin_entries)):
                        self.select_bin(int(row))
                        return True
        for name, button in list(self.buttons.items()):
            bx, by, bw, bh = button.rect
            if bx <= x <= bx + bw and by <= y <= by + bh:
                if not button.enabled:
                    return True
                if name == "bin_dropdown":
                    if self.recording_active:
                        self.status = "Stop ROI recording before changing bin"
                        return True
                    self.load_session_dropdown_open = False
                    self.bin_dropdown_open = not self.bin_dropdown_open
                elif name == "clear_prompts":
                    if self.review_mode:
                        self.annotate_review_frame()
                    else:
                        self.clear_prompts()
                elif name == "save_sample":
                    if self.review_mode:
                        self.save_review_frame()
                    else:
                        self.save_sample()
                elif name == "save_background_sample":
                    self.save_background_sample()
                elif name == "delete_last_sample":
                    self.delete_last_sample()
                elif name == "delete_review_frame":
                    self.delete_current_review_frame()
                elif name == "toggle_video_recording":
                    if self.recording_active:
                        self.stop_video_recording()
                    else:
                        self.capture_roi_image()
                elif name == "toggle_video_review":
                    if self.review_mode:
                        self.exit_video_review()
                    else:
                        self.choose_video_review()
                elif name == "review_prev_frame":
                    self.review_previous_frame()
                elif name == "review_next_frame":
                    self.review_next_frame()
                elif name == "annotate_review_frame":
                    self.annotate_review_frame()
                elif name == "sam2_review_frame":
                    self.request_review_sam2_annotation()
                elif name == "save_review_frame":
                    self.save_review_frame()
                elif name == "save_review_background_frame":
                    self.save_review_background_frame()
                elif name == "toggle_train_gpu":
                    self.toggle_gpu_training()
                elif name == "train_yolo":
                    self.start_training()
                elif name == "live_view":
                    self.load_session_dropdown_open = False
                    self.live_view_enabled = not self.live_view_enabled
                    self.status = "Live view toggled"
                    self.save_session()
                elif name == "overlay":
                    self.load_session_dropdown_open = False
                    self.overlay_enabled = not self.overlay_enabled
                    self.status = "Overlay toggled"
                    self.save_session()
                elif name == "save_session":
                    if self.recording_active:
                        self.status = "Stop ROI recording before saving session"
                        return True
                    self.bin_dropdown_open = False
                    self.load_session_dropdown_open = False
                    self.save_current_session_as_saved()
                elif name == "load_session":
                    if self.recording_active:
                        self.status = "Stop ROI recording before loading session"
                        return True
                    self.bin_dropdown_open = False
                    self.refresh_saved_sessions()
                    if not self.saved_session_entries:
                        self.load_session_dropdown_open = False
                        self.status = "No saved YOLO teach sessions"
                    else:
                        self.load_session_dropdown_open = not self.load_session_dropdown_open
                return True
        ix, iy, iw, ih = self.item_name_input_rect
        if ix <= x <= ix + iw and iy <= y <= iy + ih:
            self.item_name_edit_active = True
            self.item_name_edit_buffer = self.item_name
            self.status = "Type item name, Enter to apply, Esc to cancel"
            return True
        if x < LEFT_PANEL_WIDTH or y < VIDEO_TOP_BAR_HEIGHT:
            if self.item_name_edit_active:
                self.commit_item_name_edit()
            self.bin_dropdown_open = False
            return True
        return False

    def handle_prompt_click(self, event: int, x: int, y: int) -> None:
        if not self.review_mode:
            self.status = "Open image review to annotate frames"
            return
        if self.recording_active:
            self.status = "Stop ROI recording before annotating"
            return
        px, py, pw, ph = self.preview_rect
        if x < px or y < py or x >= px + pw or y >= py + ph:
            return
        frame_x = int(round((x - px) / max(1e-6, self.preview_scale)))
        frame_y = int(round((y - py) / max(1e-6, self.preview_scale)))
        if self.frozen_crop_bgr is None and not self.freeze_current_crop():
            return
        if self.frozen_crop_bgr is None or self.roi_crop_rect is None:
            return
        rect_x, rect_y, _, _ = self.roi_crop_rect
        crop_x = frame_x - rect_x
        crop_y = frame_y - rect_y
        h, w = self.frozen_crop_bgr.shape[:2]
        if crop_x < 0 or crop_y < 0 or crop_x >= w or crop_y >= h:
            return
        if self.roi_crop_mask is not None and self.roi_crop_mask[crop_y, crop_x] == 0:
            self.status = "Prompt is outside bin ROI"
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.positive_points.append((crop_x, crop_y))
        else:
            self.negative_points.append((crop_x, crop_y))
        self.current_mask = None
        self.prediction_dirty = bool(self.positive_points)
        self.status = f"Prompts +{len(self.positive_points)} -{len(self.negative_points)}"
        self.save_session()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemTeachYoloNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_video_recording(enter_review=False)
        node.save_runtime_settings()
        node.save_session()
        cv2.destroyWindow(WINDOW_NAME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
