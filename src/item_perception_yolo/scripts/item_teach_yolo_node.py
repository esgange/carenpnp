#!/usr/bin/env python3
import datetime as _dt
import os
import re
import shutil
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import torch
import yaml
from orbbec_camera_msgs.srv import SetInt32
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.joint_states_topic = self.declare_parameter("joint_states_topic", "/joint_states_robot").value
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
            self.declare_parameter("clear_runtime_on_start", True).value)
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
        self.train_device = self.declare_parameter("train_device", "cpu").value
        self.display_scale = float(self.declare_parameter("display_scale", 1.0).value)
        self.overlay_enabled = as_bool(self.declare_parameter("overlay_enabled", True).value)
        self.live_view_enabled = as_bool(self.declare_parameter("live_view_enabled", True).value)

        self.lock = threading.Lock()
        self.latest_bgr: Optional[np.ndarray] = None
        self.latest_header_stamp = ""
        self.latest_joint_positions_deg = [0.0] * 6
        self.has_joint_positions = False

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
        self.final_model_dir = ""
        self.latest_profile_path = ""

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
        self.save_session()

        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10,
        )
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
                    }
                }
            }
            tmp_path = self.runtime_settings_path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            tmp_path.replace(self.runtime_settings_path)
        except Exception as exc:
            self.get_logger().warn(f"YOLO teach runtime settings save failed: {exc}")

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
        for path in [
            self.images_dir,
            self.labels_dir,
            self.masks_dir,
            self.previews_dir,
            self.prompts_dir,
            self.models_dir,
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
        old_session_dir = self.session_dir
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
        self.session_dir = self.create_session_dir()
        self.configure_session_storage()
        self.write_dataset_yaml()
        self.remove_runtime_dir(old_session_dir)
        self.status = f"Item name changed to {self.item_name}; runtime reset"
        self.save_session()

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
                "final_model_dir": self.final_model_dir,
                "latest_profile_path": self.latest_profile_path,
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
                "teach_joints_deg": self.latest_joint_positions_deg if self.has_joint_positions else [],
                "positive_prompt_count": len(self.positive_points),
                "negative_prompt_count": len(self.negative_points),
            }
        }
        tmp_path = self.session_yaml_path.with_suffix(".tmp")
        tmp_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        tmp_path.replace(self.session_yaml_path)

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
        try:
            self.save_session()
            if self.current_session_is_saved():
                self.write_dataset_yaml()
                self.save_session()
                self.status = f"Saved session: {self.session_dir.name}"
                self.refresh_saved_sessions()
                return

            old_session_dir = self.session_dir
            target = self.unique_saved_session_dir()
            shutil.copytree(old_session_dir, target)
            self.session_dir = target
            self.configure_session_storage()
            self.write_dataset_yaml()
            self.save_session()
            self.remove_runtime_dir(old_session_dir)
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

            old_session_dir = self.session_dir
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
            self.final_model_dir = str(params.get("final_model_dir", ""))
            self.latest_profile_path = str(params.get("latest_profile_path", ""))
            teach_joints = params.get("teach_joints_deg", [])
            if isinstance(teach_joints, list) and len(teach_joints) >= 6:
                self.latest_joint_positions_deg = [float(value) for value in teach_joints[:6]]
                self.has_joint_positions = True
            else:
                self.has_joint_positions = False
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
            self.write_dataset_yaml()
            self.save_session()
            self.remove_runtime_dir(old_session_dir)
            self.refresh_saved_sessions()
            self.load_session_dropdown_open = False
            self.status = f"Loaded session: {self.item_name or self.session_dir.name} | {self.sample_count_label()}"
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
                self.final_model_dir = ""
                self.latest_profile_path = ""
                self.active_bin_index = -1
                self.active_bin = None
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

    def joint_state_callback(self, msg: JointState) -> None:
        if not msg.position:
            return
        rad_to_deg = 57.29577951308232
        joints_deg = [0.0] * 6
        found = [False] * 6
        joint_index = {
            "joint1": 0,
            "joint2": 1,
            "joint3": 2,
            "joint4": 3,
            "joint5": 4,
            "joint6": 5,
        }
        for i, name in enumerate(msg.name[:len(msg.position)]):
            index = joint_index.get(name)
            if index is not None:
                joints_deg[index] = float(msg.position[i]) * rad_to_deg
                found[index] = True
        valid = all(found)
        if not valid and len(msg.position) >= 6:
            joints_deg = [float(value) * rad_to_deg for value in msg.position[:6]]
            valid = True
        if not valid:
            return
        self.latest_joint_positions_deg = joints_deg
        self.has_joint_positions = True

    def select_bin(self, index: int) -> None:
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

    def save_sample(self) -> None:
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.frozen_crop_bgr is None or self.current_mask is None:
            self.status = "No SAM2 mask to save"
            return
        contour = self.mask_to_largest_contour(self.current_mask)
        if contour is None:
            self.status = "Mask is empty"
            return
        h, w = self.current_mask.shape[:2]
        label_text = self.contour_to_yolo_seg(contour, w, h)
        if label_text is None:
            self.status = "Mask polygon is invalid"
            return

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

    def save_background_sample(self) -> None:
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        crop_info = self.current_roi_crop()
        if crop_info is None:
            self.status = "Select a bin and wait for color image"
            return

        crop, rect, _ = crop_info
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
            "note": "Empty YOLO label file marks this ROI crop as background.",
        }
        prompt_path.write_text(yaml.safe_dump(prompt_data, sort_keys=False), encoding="utf-8")
        self.sample_history.append({"kind": "background", "stem": stem})
        self.write_dataset_yaml()
        self.clear_prompts(save=False)
        self.mark_dataset_changed_after_training()
        self.status = f"Saved background {self.background_sample_count} | {self.background_ratio_label()}"
        self.save_session()

    def can_delete_last_sample(self) -> bool:
        training_active = self.training_thread is not None and self.training_thread.is_alive()
        return self.total_training_image_count() > 0 and not training_active

    def can_save_background_sample(self) -> bool:
        training_active = self.training_thread is not None and self.training_thread.is_alive()
        return (
            self.has_item_name() and
            self.active_bin is not None and
            self.latest_bgr is not None and
            not training_active
        )

    def delete_last_sample(self) -> None:
        entry = self.latest_sample_entry()
        if entry is None:
            self.status = "No saved samples to delete"
            return
        if self.training_thread is not None and self.training_thread.is_alive():
            self.status = "Cannot delete samples while training"
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
        if not self.has_item_name():
            self.status = self.item_name_error()
            return
        if self.sample_count <= 0:
            self.status = "Save at least one sample first"
            return
        self.training_status = "training"
        self.training_epoch_current = 0
        self.training_epoch_total = max(1, self.train_epochs)
        self.training_progress = 0.0
        self.status = (
            f"Training YOLO11-seg on {self.sample_count} item + "
            f"{self.background_sample_count} background samples"
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

            def on_train_start(trainer) -> None:
                total_epochs = int(getattr(trainer, "epochs", self.train_epochs))
                self.update_training_progress(0, total_epochs, "training")
                self.status = f"Training YOLO11-seg: epoch 0/{self.training_epoch_total}"
                self.save_session()

            def on_train_epoch_start(trainer) -> None:
                total_epochs = int(getattr(trainer, "epochs", self.train_epochs))
                epoch_index = int(getattr(trainer, "epoch", 0)) + 1
                completed = max(0, epoch_index - 1)
                self.update_training_progress(completed, total_epochs, "training")
                self.status = f"Training YOLO11-seg: epoch {epoch_index}/{self.training_epoch_total}"

            def on_fit_epoch_end(trainer) -> None:
                total_epochs = int(getattr(trainer, "epochs", self.train_epochs))
                completed = int(getattr(trainer, "epoch", 0)) + 1
                self.update_training_progress(completed, total_epochs, "training")
                self.status = (
                    f"Training YOLO11-seg: epoch "
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
                device=self.train_device,
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
        final_pt = model_dir / "best.pt"
        final_onnx = model_dir / "best.onnx"
        shutil.copy2(best_path, final_pt)

        try:
            trained_model = YOLO(str(best_path))
            exported = trained_model.export(
                format="onnx",
                imgsz=self.train_imgsz,
                device=self.train_device,
                dynamic=False,
                simplify=False,
            )
            exported_file = Path(str(exported))
            if exported_file.exists():
                shutil.copy2(exported_file, final_onnx)
        except Exception as exc:
            self.get_logger().warn(f"YOLO ONNX export failed; detect needs ONNX for CPU speed: {exc}")

        self.final_model_dir = str(model_dir)
        self.trained_model_path = str(final_pt)
        self.trained_onnx_path = str(final_onnx) if final_onnx.exists() else ""

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
            "trained_model_path": self.trained_model_path,
            "trained_onnx_path": self.trained_onnx_path,
            "associated_bin_name": active_bin.bin_name if active_bin else "",
            "bin_teach_file": str(active_bin.path) if active_bin else "",
            "detection_mode": "yolo_depth",
            "depth_window_mm": 50,
            "align_item_z_axis_to_depth_plane": True,
            "joint_states_topic": self.joint_states_topic,
            "teach_joints_deg": self.latest_joint_positions_deg if self.has_joint_positions else [],
            "has_teach_joints": self.has_joint_positions,
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

        if self.frozen_crop_bgr is not None:
            display_frame = self.frozen_frame_bgr.copy() if self.frozen_frame_bgr is not None else self.frozen_crop_bgr.copy()
        else:
            frame = self.latest_frame_copy()
            if frame is None:
                display_frame = self.build_no_camera_topics_placeholder()
            elif frame is not None and self.active_bin is not None and self.live_view_enabled:
                live_roi = self.roi_views_from_frame(frame)
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

    def draw_panel(self, canvas: np.ndarray) -> None:
        panel = canvas[:, :LEFT_PANEL_WIDTH]
        panel[:] = (30, 34, 40)
        self.buttons.clear()

        cv2.putText(panel, "YOLO Teach", (PANEL_PAD, 36), cv2.FONT_HERSHEY_SIMPLEX,
                    0.82, (240, 244, 248), 2, cv2.LINE_AA)
        cv2.putText(panel, "Item Name", (PANEL_PAD, 70), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (220, 230, 238), 1, cv2.LINE_AA)
        input_y = 82
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

        y = 136
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

        cv2.putText(panel, "SAM2 Prompts", (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (235, 238, 242), 1, cv2.LINE_AA)
        y += 10
        cv2.putText(panel, f"Positive: {len(self.positive_points)}    Negative: {len(self.negative_points)}",
                    (PANEL_PAD, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (205, 218, 226), 1, cv2.LINE_AA)
        y += 38
        self.draw_button(panel, "clear_prompts", (PANEL_PAD, y, 190, BUTTON_HEIGHT),
                         "Clear", True, role="danger")
        self.draw_button(panel, "delete_last_sample", (PANEL_PAD + 210, y, 190, BUTTON_HEIGHT),
                         "Del Last", self.can_delete_last_sample(), role="danger")
        y += BUTTON_HEIGHT + 8
        self.draw_button(panel, "save_sample", (PANEL_PAD, y, 190, BUTTON_HEIGHT),
                         "Save Item", self.current_mask is not None and self.has_item_name(), role="save")
        self.draw_button(panel, "save_background_sample", (PANEL_PAD + 210, y, 190, BUTTON_HEIGHT),
                         "Save BG", self.can_save_background_sample(), role="save")
        y += BUTTON_HEIGHT + 14

        cv2.putText(panel, "YOLO11", (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (235, 238, 242), 1, cv2.LINE_AA)
        y += 12
        cv2.putText(panel, self.sample_count_label(), (PANEL_PAD, y + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (190, 232, 190), 1, cv2.LINE_AA)
        cv2.putText(panel, self.background_ratio_label(), (PANEL_PAD, y + 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, self.background_ratio_color(), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Status: {fit_text(self.training_status, 30)}", (PANEL_PAD, y + 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (205, 218, 226), 1, cv2.LINE_AA)
        joint_status = "Teach Position: captured" if self.has_joint_positions else "Teach Position: waiting"
        cv2.putText(panel, joint_status, (PANEL_PAD, y + 96),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (184, 224, 194) if self.has_joint_positions else (205, 188, 170),
                    1,
                    cv2.LINE_AA)
        y += 104
        training_active = self.training_thread is not None and self.training_thread.is_alive()
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
                self.sample_count > 0 and self.has_item_name())
        else:
            self.draw_button(panel, "train_yolo", train_rect, "Train YOLO11",
                             self.sample_count > 0 and self.has_item_name())
        y += BUTTON_HEIGHT + 10

        cv2.putText(panel, "Status", (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (235, 238, 242), 1, cv2.LINE_AA)
        y += 20
        status_lines = self.draw_wrapped_text(
            panel, self.status, PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, 3, 0.44)
        y += max(30, status_lines * 22 + 8)

        cv2.putText(panel, "Session", (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (235, 238, 242), 1, cv2.LINE_AA)
        y += 20
        session_lines = self.draw_wrapped_text(
            panel, str(self.session_dir), PANEL_PAD, y, LEFT_PANEL_WIDTH - 2 * PANEL_PAD, 2, 0.39)
        y += max(30, session_lines * 22 + 6)

        cv2.putText(panel, "Controls", (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (235, 238, 242), 1, cv2.LINE_AA)
        y += 20
        instructions = [
            "Left click: positive prompt",
            "Right click: negative prompt",
            "Save Item: mask to YOLO label",
            "Save BG: empty label target 10-20%",
            "Del Last: remove newest sample",
            "Train YOLO11: train segmentation model",
        ]
        for line in instructions:
            cv2.putText(panel, line, (PANEL_PAD, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.43, (184, 196, 205), 1, cv2.LINE_AA)
            y += 21

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
            self.reset_runtime_for_item_name(new_name)
        self.item_name_edit_active = False

    def cancel_item_name_edit(self) -> None:
        self.item_name_edit_buffer = self.item_name
        self.item_name_edit_active = False
        self.status = "Item name edit canceled"

    def handle_key(self, key: int) -> None:
        if key < 0 or not self.item_name_edit_active:
            return
        code = key & 0xFF
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
                         "Save Session", not training_active, role="save")
        x += 164
        load_enabled = bool(self.saved_session_entries) and not training_active
        load_label = "Load Session" if self.saved_session_entries else "Load: none"
        self.draw_button(canvas, "load_session", (x, y, 170, BUTTON_HEIGHT),
                         load_label, load_enabled, self.load_session_dropdown_open)
        status = "Full frame ROI view"
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
                if not button.enabled and name != "clear_prompts":
                    return True
                if name == "bin_dropdown":
                    self.load_session_dropdown_open = False
                    self.bin_dropdown_open = not self.bin_dropdown_open
                elif name == "clear_prompts":
                    self.clear_prompts()
                elif name == "save_sample":
                    self.save_sample()
                elif name == "save_background_sample":
                    self.save_background_sample()
                elif name == "delete_last_sample":
                    self.delete_last_sample()
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
                    self.bin_dropdown_open = False
                    self.load_session_dropdown_open = False
                    self.save_current_session_as_saved()
                elif name == "load_session":
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
        if self.active_bin is None:
            self.status = "Choose a bin before prompting"
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
        self.prediction_dirty = True
        self.status = f"Prompts +{len(self.positive_points)} -{len(self.negative_points)}"
        self.save_session()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemTeachYoloNode()
    try:
        rclpy.spin(node)
    finally:
        remove_unsaved_session = not node.current_session_is_saved()
        unsaved_session_dir = node.session_dir
        node.save_runtime_settings()
        node.save_session()
        if remove_unsaved_session:
            node.remove_runtime_dir(unsaved_session_dir)
        cv2.destroyWindow(WINDOW_NAME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
