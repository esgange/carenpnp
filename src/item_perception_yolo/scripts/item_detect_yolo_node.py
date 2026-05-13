#!/usr/bin/env python3
import math
import os
import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
import yaml
from cv_bridge import CvBridge
from dobot_msgs_v4.srv import MovJ
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker


WINDOW_NAME = "item_detect_view"
TOP_BAR_HEIGHT = 206
PREVIEW_CANVAS_WIDTH = 1080
PREVIEW_CANVAS_HEIGHT = 680
BUTTON_HEIGHT = 34
DROPDOWN_ROW_HEIGHT = 34
MAX_DROPDOWN_ROWS = 7
METERS_TO_MM = 1000.0


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists() and
            (
                (path / "README.md").exists()
                or (path / "docker-compose.yml").exists()
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


@dataclass
class Button:
    name: str
    rect: Tuple[int, int, int, int]
    enabled: bool = True


@dataclass
class DepthPlane:
    valid: bool = False
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    reference_depth_m: float = 0.0


@dataclass
class ItemProfile:
    path: Path
    label: str
    item_name: str = "item"
    class_id: int = 0
    class_name: str = "item"
    associated_bin_name: str = ""
    teach_date: str = ""
    model_path: str = ""
    model_pt_path: str = ""
    model_dir: str = ""
    color_topic: str = "/robot_camera/color/image_raw"
    depth_topic: str = "/robot_camera/depth/image_raw"
    camera_info_topic: str = "/robot_camera/color/camera_info"
    overlay_topic: str = "bin_overlay"
    roi_points: List[Tuple[float, float]] = field(default_factory=list)
    depth_plane: DepthPlane = field(default_factory=DepthPlane)
    depth_window_mm: int = 50
    align_z_to_depth_plane: bool = True
    teach_joints_deg: List[float] = field(default_factory=list)
    has_teach_joints: bool = False


@dataclass
class Detection:
    score: float
    class_id: int
    box: Tuple[int, int, int, int]
    mask: np.ndarray
    center: Tuple[float, float]
    corners: List[Tuple[float, float]]


@dataclass
class Pose3D:
    origin: np.ndarray
    rotation: np.ndarray


@dataclass
class DetectionPose:
    pose: Pose3D
    pick_pixel: Tuple[float, float]


def resolve_path(path_text: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_text))).resolve()


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def fit_text(text: str, max_chars: int) -> str:
    return text if len(text) <= max_chars else text[: max(0, max_chars - 3)] + "..."


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -80.0, 80.0)))


def normalized_image_coord(value: int, size: int) -> float:
    if size <= 1:
        return 0.0
    return (float(value) / float(size - 1)) * 2.0 - 1.0


def rotation_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(max(1e-12, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-12:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return qx, qy, qz, qw


def pose_to_msg(pose: Pose3D) -> Pose:
    msg = Pose()
    msg.position.x = float(pose.origin[0])
    msg.position.y = float(pose.origin[1])
    msg.position.z = float(pose.origin[2])
    qx, qy, qz, qw = rotation_to_quaternion(pose.rotation)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def lower_left_corner_index(corners: List[Tuple[float, float]]) -> int:
    if not corners:
        return -1
    return max(range(len(corners)), key=lambda i: (corners[i][1], -corners[i][0]))


def corner_center(corners: List[Tuple[float, float]]) -> Tuple[float, float]:
    if not corners:
        return 0.0, 0.0
    points = np.asarray(corners, dtype=np.float32)
    center = np.mean(points, axis=0)
    return float(center[0]), float(center[1])


def normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9 or not np.isfinite(norm):
        return None
    return vec / norm


class ItemDetectYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("item_detect")
        self.bridge = CvBridge()

        self.profiles_dir = resolve_path(
            self.declare_parameter(
                "profiles_dir", str(workspace_path("teach", "bins_yolo", "profiles"))
            ).value)
        self.model_root = resolve_path(
            self.declare_parameter(
                "model_root", str(workspace_path("teach", "bins_yolo", "models"))
            ).value)
        self.color_topic = self.declare_parameter("color_topic", "/robot_camera/color/image_raw").value
        self.depth_topic = self.declare_parameter("depth_topic", "/robot_camera/depth/image_raw").value
        self.camera_info_topic = self.declare_parameter("camera_info_topic", "/robot_camera/color/camera_info").value
        self.overlay_topic = self.declare_parameter("overlay_topic", "bin_overlay").value
        self.seek_pose_topic = self.declare_parameter("bin_pose_topic", "bin_seek_pose").value
        self.item_pose_array_topic = self.declare_parameter("bin_item_pose_array_topic", "bin_item_poses").value
        self.item_cube_marker_topic = self.declare_parameter("bin_cube_marker_topic", "bin_cube_marker").value
        self.seek_service_name = self.declare_parameter("seek_service", "item_detect/seek").value
        self.seek_complete_service_name = self.declare_parameter(
            "seek_complete_service", "item_detect/seek_complete").value
        self.seek_status_service_name = self.declare_parameter(
            "seek_status_service", "item_detect/seek_status").value
        self.go_to_teach_service_name = self.declare_parameter(
            "go_to_teach_service", "item_detect/go_to_teach").value
        self.movj_service_name = self.declare_parameter("movj_service", "/dobot_bringup_ros2/srv/MovJ").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_color_optical_frame").value
        self.use_calibration = as_bool(self.declare_parameter("use_calibration", True).value)
        self.publish_static_calibration_tf = as_bool(
            self.declare_parameter("publish_static_calibration_tf", True).value)
        self.calibration_parent_frame = self.declare_parameter("calibration_parent_frame", "Link6").value
        self.calibration_child_frame = self.declare_parameter(
            "calibration_child_frame", "calibrated_camera_link").value
        self.calibration_file = self.declare_parameter("calibration_file", "").value
        self.start_visualization = as_bool(self.declare_parameter("start_visualization", True).value)
        self.publish_overlay = as_bool(self.declare_parameter("publish_overlay", True).value)
        self.align_item_z_axis_to_depth_plane = as_bool(
            self.declare_parameter("align_item_z_axis_to_depth_plane", True).value)
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_conf = float(np.clip(float(self.declare_parameter("yolo_conf", 0.35).value), 0.0, 1.0))
        self.seek_window_sec = float(self.declare_parameter("seek_window_sec", 60.0).value)
        self.seek_decay_sec = float(self.declare_parameter("seek_decay_sec", 1.0).value)
        self.yolo_iou = float(self.declare_parameter("yolo_iou", 0.45).value)
        self.mask_threshold = float(self.declare_parameter("mask_threshold", 0.5).value)
        self.max_inference_hz = float(self.declare_parameter("max_inference_hz", 8.0).value)
        self.ort_threads = int(self.declare_parameter("onnxruntime_threads", 0).value)

        self.latest_depth_m: Optional[np.ndarray] = None
        self.latest_info: Optional[CameraInfo] = None
        self.latest_detections: List[Detection] = []
        self.latest_detection_poses: List[Optional[DetectionPose]] = []
        self.selected_detection: Optional[Detection] = None
        self.selected_pose: Optional[Pose3D] = None
        self.peak_pixel: Optional[Tuple[int, int]] = None
        self.last_inference_time = 0.0
        self.status = "Loading YOLO profiles"
        self.seek_mode_active = False
        self.seek_started_time = 0.0
        self.last_seek_pose: Optional[Pose3D] = None
        self.last_seek_pose_time = 0.0
        self.go_to_teach_in_progress = False
        self.pending_delete_profile: Optional[Path] = None
        self.pending_delete_deadline = 0.0
        self.view_mode = "RGB"
        self.active_slider: Optional[str] = None

        self.buttons: Dict[str, Button] = {}
        self.slider_rects: Dict[str, Tuple[int, int, int, int]] = {}
        self.profile_dropdown_open = False
        self.profile_option_rects: List[Tuple[int, int, int, int]] = []
        self.preview_scale = 1.0
        self.preview_rect = (0, TOP_BAR_HEIGHT, PREVIEW_CANVAS_WIDTH, PREVIEW_CANVAS_HEIGHT)

        self.profiles: List[ItemProfile] = []
        self.selected_profile_index = -1
        self.active_profile: Optional[ItemProfile] = None
        self.ort_session: Optional[ort.InferenceSession] = None
        self.ort_input_name = ""
        self.ort_output_names: List[str] = []

        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 5)
        self.seek_pose_pub = self.create_publisher(PoseStamped, self.seek_pose_topic, 10)
        self.item_pose_array_pub = self.create_publisher(PoseArray, self.item_pose_array_topic, 10)
        self.item_marker_pub = self.create_publisher(Marker, self.item_cube_marker_topic, 1)
        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.movj_client = self.create_client(MovJ, self.movj_service_name)
        self.create_service(Trigger, self.seek_service_name, self.handle_seek)
        self.create_service(Trigger, self.seek_complete_service_name, self.handle_seek_complete)
        self.create_service(Trigger, self.seek_status_service_name, self.handle_seek_status)
        self.create_service(Trigger, self.go_to_teach_service_name, self.handle_go_to_teach)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_calibration_tf()
        self.refresh_profiles()

        if self.start_visualization:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | getattr(cv2, "WINDOW_GUI_NORMAL", 0))
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        self.get_logger().info(
            f"item_detect YOLO ready. profiles_dir={self.profiles_dir} "
            f"pose_topic={self.seek_pose_topic} array_topic={self.item_pose_array_topic}")

    def refresh_profiles(self) -> None:
        self.profiles = []
        self.latest_detections = []
        self.latest_detection_poses = []
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        if self.profiles_dir.exists():
            for path in sorted(self.profiles_dir.glob("*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True):
                profile = self.load_profile(path)
                if profile is not None:
                    self.profiles.append(profile)
        if not self.profiles:
            self.active_profile = None
            self.selected_profile_index = -1
            self.ort_session = None
            self.status = f"No YOLO profiles in {self.profiles_dir}"
            return
        self.select_profile(0)

    def load_profile(self, path: Path) -> Optional[ItemProfile]:
        try:
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            params = None
            for key in ("item_detect", "item_yolo"):
                candidate = root.get(key, {}).get("ros__parameters")
                if isinstance(candidate, dict):
                    params = candidate
                    break
            if not isinstance(params, dict):
                return None
            model_path = (
                params.get("model_onnx_path") or
                params.get("trained_onnx_path") or
                params.get("model_path") or
                ""
            )
            if not model_path:
                return None
            roi_points = self.parse_points(params.get("roi_points", []))
            if len(roi_points) < 4:
                return None
            depth_plane_node = params.get("depth_plane", {})
            if not isinstance(depth_plane_node, dict):
                depth_plane_node = {}
            depth_plane = DepthPlane(
                valid=bool(params.get("depth_plane_enabled", depth_plane_node.get("depth_plane_enabled", False))),
                a=float(params.get("depth_plane_a", depth_plane_node.get("depth_plane_a", 0.0))),
                b=float(params.get("depth_plane_b", depth_plane_node.get("depth_plane_b", 0.0))),
                c=float(params.get("depth_plane_c", depth_plane_node.get("depth_plane_c", 0.0))),
                reference_depth_m=float(params.get(
                    "depth_plane_reference_depth_m",
                    depth_plane_node.get("depth_plane_reference_depth_m", 0.0))),
            )
            if depth_plane.reference_depth_m <= 0.0 or not np.isfinite(
                [depth_plane.a, depth_plane.b, depth_plane.c, depth_plane.reference_depth_m]).all():
                depth_plane.valid = False
            item_name = str(params.get("item_name", path.stem))
            bin_name = str(params.get("associated_bin_name", ""))
            teach_date = str(params.get("teach_date", ""))
            label = item_name
            if bin_name:
                label += f" @ {bin_name}"
            if teach_date:
                label += f" | {teach_date}"
            else:
                label += f" | {path.name}"
            teach_joints = self.parse_teach_joints(params.get("teach_joints_deg", []))
            return ItemProfile(
                path=path,
                label=label,
                item_name=item_name,
                class_id=int(params.get("class_id", 0)),
                class_name=str(params.get("class_name", item_name)),
                associated_bin_name=bin_name,
                teach_date=teach_date,
                model_path=str(resolve_path(str(model_path))),
                model_pt_path=str(resolve_path(str(params.get("model_pt_path", params.get("trained_model_path", "")))))
                if params.get("model_pt_path") or params.get("trained_model_path") else "",
                model_dir=str(resolve_path(str(params.get("model_dir", "")))) if params.get("model_dir") else "",
                color_topic=str(params.get("color_topic", self.color_topic)),
                depth_topic=str(params.get("depth_topic", self.depth_topic)),
                camera_info_topic=str(params.get("camera_info_topic", self.camera_info_topic)),
                overlay_topic=str(params.get("overlay_topic", self.overlay_topic)),
                roi_points=roi_points,
                depth_plane=depth_plane,
                depth_window_mm=int(params.get("depth_window_mm", 50)),
                align_z_to_depth_plane=bool(params.get("align_item_z_axis_to_depth_plane", True)),
                teach_joints_deg=teach_joints,
                has_teach_joints=as_bool(params.get("has_teach_joints", False)) or len(teach_joints) >= 6,
            )
        except Exception as exc:
            self.get_logger().warn(f"Skipping YOLO profile {path}: {exc}")
            return None

    def parse_points(self, value) -> List[Tuple[float, float]]:
        if not isinstance(value, list):
            return []
        points: List[Tuple[float, float]] = []
        if value and not isinstance(value[0], list):
            for i in range(0, len(value) - 1, 2):
                points.append((float(value[i]), float(value[i + 1])))
        else:
            for point in value:
                if isinstance(point, list) and len(point) >= 2:
                    points.append((float(point[0]), float(point[1])))
        return points

    def parse_teach_joints(self, value) -> List[float]:
        if not isinstance(value, list) or len(value) < 6:
            return []
        try:
            return [float(v) for v in value[:6]]
        except (TypeError, ValueError):
            return []

    def select_profile(self, index: int) -> bool:
        if index < 0 or index >= len(self.profiles):
            return False
        profile = self.profiles[index]
        model_path = Path(profile.model_path)
        if not model_path.exists():
            self.status = f"Model missing: {model_path}"
            return False
        self.selected_profile_index = index
        self.active_profile = profile
        self.load_onnx_model(model_path)
        self.status = f"Loaded {profile.label}"
        self.profile_dropdown_open = False
        self.pending_delete_profile = None
        self.pending_delete_deadline = 0.0
        return True

    def load_onnx_model(self, model_path: Path) -> None:
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        if self.ort_threads > 0:
            options.intra_op_num_threads = self.ort_threads
            options.inter_op_num_threads = 1
        self.ort_session = ort.InferenceSession(
            str(model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"])
        self.ort_input_name = self.ort_session.get_inputs()[0].name
        self.ort_output_names = [output.name for output in self.ort_session.get_outputs()]

    def publish_calibration_tf(self) -> None:
        if not self.use_calibration or not self.publish_static_calibration_tf or not self.calibration_file:
            return
        try:
            root = yaml.safe_load(resolve_path(self.calibration_file).read_text(encoding="utf-8")) or {}
            transform = root.get("transform", {})
            translation = transform.get("translation", {})
            rotation = transform.get("rotation", {})
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.calibration_parent_frame
            msg.child_frame_id = self.calibration_child_frame
            msg.transform.translation.x = float(translation.get("x", 0.0))
            msg.transform.translation.y = float(translation.get("y", 0.0))
            msg.transform.translation.z = float(translation.get("z", 0.0))
            msg.transform.rotation.x = float(rotation.get("x", 0.0))
            msg.transform.rotation.y = float(rotation.get("y", 0.0))
            msg.transform.rotation.z = float(rotation.get("z", 0.0))
            msg.transform.rotation.w = float(rotation.get("w", 1.0))
            self.static_tf_broadcaster.sendTransform(msg)
        except Exception as exc:
            self.get_logger().warn(f"Calibration TF publish failed: {exc}")

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")
            return
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)
        depth_m[~np.isfinite(depth_m)] = np.nan
        depth_m[depth_m <= 0.0] = np.nan
        self.latest_depth_m = depth_m

    def info_callback(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Color conversion failed: {exc}")
            return
        depth_m = self.latest_depth_m
        info = self.latest_info
        if depth_m is not None and depth_m.shape[:2] != frame.shape[:2]:
            depth_m = cv2.resize(depth_m, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        stamp = msg.header.stamp
        frame_id = msg.header.frame_id or self.camera_frame or "camera_color_optical_frame"

        now = time.monotonic()
        min_period = 1.0 / max(0.1, self.max_inference_hz)
        if now - self.last_inference_time >= min_period:
            self.last_inference_time = now
            self.process_frame(frame, depth_m, info)

        base_view = self.display_frame(frame, depth_m)
        output = self.render_overlay(base_view, depth_m) if self.publish_overlay else base_view.copy()
        if self.publish_overlay:
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(output, encoding="bgr8"))
        self.publish_pose_outputs(stamp, frame_id)
        if self.start_visualization:
            view = self.build_ui(output)
            cv2.imshow(WINDOW_NAME, view)
            cv2.waitKey(1)

    def process_frame(self, frame: np.ndarray, depth_m: Optional[np.ndarray], info: Optional[CameraInfo]) -> None:
        profile = self.active_profile
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        self.latest_detection_poses = []
        if profile is None or self.ort_session is None:
            return
        roi = self.roi_crop(frame, profile.roi_points)
        if roi is None:
            self.latest_detections = []
            self.latest_detection_poses = []
            self.status = "Profile ROI is outside image"
            return
        crop, rect, roi_mask = roi
        detections = self.run_yolo(crop, roi_mask, rect)
        self.latest_detections = detections
        if not detections:
            self.latest_detection_poses = []
            self.status = f"No {profile.class_name} mask above {self.yolo_conf * 100.0:.0f}%"
            return

        selected_index = max(range(len(detections)), key=lambda i: detections[i].score)
        selected = detections[selected_index]
        self.selected_detection = selected

        detection_peaks: List[Optional[Tuple[int, int]]] = [None] * len(detections)
        if depth_m is not None:
            for index, detection in enumerate(detections):
                mask = self.mask_for_frame(detection.mask, depth_m.shape[:2])
                detection_peaks[index] = self.find_highest_peak(depth_m, mask, profile)
            self.peak_pixel = detection_peaks[selected_index]

        if depth_m is not None and info is not None:
            for index, detection in enumerate(detections):
                self.latest_detection_poses.append(
                    self.estimate_detection_pose(detection, depth_m, info, profile, detection_peaks[index]))
            selected_detection_pose = self.latest_detection_poses[selected_index]
            if selected_detection_pose is not None:
                self.selected_pose = selected_detection_pose.pose
        else:
            self.latest_detection_poses = [None] * len(detections)

        pose_count = sum(1 for detection_pose in self.latest_detection_poses if detection_pose is not None)
        self.status = (
            f"Detected {len(detections)} {profile.class_name} mask(s) | "
            f"best confidence {selected.score * 100.0:.0f}% | poses {pose_count}"
        )

    def roi_crop(
        self,
        frame: np.ndarray,
        points: List[Tuple[float, float]],
    ) -> Optional[Tuple[np.ndarray, Tuple[int, int, int, int], np.ndarray]]:
        if len(points) < 3:
            return None
        h, w = frame.shape[:2]
        pts = np.asarray(points, dtype=np.float32)
        pts[:, 0] = np.clip(pts[:, 0], 0, max(0, w - 1))
        pts[:, 1] = np.clip(pts[:, 1], 0, max(0, h - 1))
        x0 = int(np.floor(np.min(pts[:, 0])))
        y0 = int(np.floor(np.min(pts[:, 1])))
        x1 = int(np.ceil(np.max(pts[:, 0])))
        y1 = int(np.ceil(np.max(pts[:, 1])))
        if x1 <= x0 or y1 <= y0:
            return None
        crop = frame[y0:y1 + 1, x0:x1 + 1].copy()
        rel = np.round(pts - np.array([x0, y0], dtype=np.float32)).astype(np.int32)
        mask = np.zeros(crop.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [rel], 255)
        crop[mask == 0] = 0
        return crop, (x0, y0, crop.shape[1], crop.shape[0]), mask

    def letterbox(self, image: np.ndarray) -> Tuple[np.ndarray, float, int, int, int, int]:
        h, w = image.shape[:2]
        scale = min(self.yolo_imgsz / float(w), self.yolo_imgsz / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.yolo_imgsz, self.yolo_imgsz, 3), 114, dtype=np.uint8)
        pad_x = (self.yolo_imgsz - new_w) // 2
        pad_y = (self.yolo_imgsz - new_h) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return canvas, scale, pad_x, pad_y, new_w, new_h

    def run_yolo(
        self,
        crop: np.ndarray,
        roi_mask: np.ndarray,
        rect: Tuple[int, int, int, int],
    ) -> List[Detection]:
        if self.ort_session is None or self.active_profile is None:
            return []
        letterboxed, scale, pad_x, pad_y, new_w, new_h = self.letterbox(crop)
        rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[None, :, :, :]
        outputs = self.ort_session.run(self.ort_output_names, {self.ort_input_name: tensor})
        if len(outputs) < 2:
            return []
        pred = outputs[0]
        proto = outputs[1]
        if pred.ndim == 4 and proto.ndim in (2, 3):
            pred, proto = proto, pred
        if pred.ndim == 3:
            pred = pred[0]
        if pred.shape[0] < pred.shape[1] and pred.shape[0] <= 256:
            pred = pred.T
        if proto.ndim == 4:
            proto = proto[0]
        mask_dim = int(proto.shape[0])
        nc = int(pred.shape[1] - 4 - mask_dim)
        if nc <= 0:
            return []
        boxes_xywh = pred[:, :4]
        scores_all = pred[:, 4:4 + nc]
        coeffs_all = pred[:, 4 + nc:4 + nc + mask_dim]
        class_ids = np.argmax(scores_all, axis=1)
        scores = scores_all[np.arange(scores_all.shape[0]), class_ids]
        keep = np.where(scores >= self.yolo_conf)[0]
        if keep.size == 0:
            return []

        boxes: List[List[int]] = []
        rows: List[int] = []
        for idx in keep:
            if int(class_ids[idx]) != int(self.active_profile.class_id):
                continue
            cx, cy, bw, bh = boxes_xywh[idx]
            x1 = (float(cx - bw / 2.0) - pad_x) / scale
            y1 = (float(cy - bh / 2.0) - pad_y) / scale
            x2 = (float(cx + bw / 2.0) - pad_x) / scale
            y2 = (float(cy + bh / 2.0) - pad_y) / scale
            x1 = int(np.clip(round(x1), 0, crop.shape[1] - 1))
            y1 = int(np.clip(round(y1), 0, crop.shape[0] - 1))
            x2 = int(np.clip(round(x2), 0, crop.shape[1] - 1))
            y2 = int(np.clip(round(y2), 0, crop.shape[0] - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            rows.append(int(idx))
        if not boxes:
            return []
        nms = cv2.dnn.NMSBoxes(boxes, [float(scores[i]) for i in rows], self.yolo_conf, self.yolo_iou)
        if len(nms) == 0:
            return []
        nms_indices = np.asarray(nms).reshape(-1)
        detections: List[Detection] = []
        for nms_idx in nms_indices:
            row = rows[int(nms_idx)]
            box = boxes[int(nms_idx)]
            crop_mask = self.decode_mask(
                coeffs_all[row],
                proto,
                crop.shape[:2],
                roi_mask,
                box,
                pad_x,
                pad_y,
                new_w,
                new_h,
            )
            if crop_mask is None:
                continue
            full_mask = np.zeros((rect[3], rect[2]), dtype=np.uint8)
            full_mask[:, :] = crop_mask
            contour = self.largest_contour(full_mask)
            if contour is None:
                continue
            corners = self.contour_corners(contour, rect[0], rect[1])
            if len(corners) != 4:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                cx = float(moments["m10"] / moments["m00"]) + rect[0]
                cy = float(moments["m01"] / moments["m00"]) + rect[1]
            else:
                cx = float(box[0] + box[2] * 0.5 + rect[0])
                cy = float(box[1] + box[3] * 0.5 + rect[1])
            placed_mask = np.zeros((rect[1] + rect[3], rect[0] + rect[2]), dtype=np.uint8)
            # Resized later by caller if needed; store crop-relative mask with full-frame box offset.
            detections.append(Detection(
                score=float(scores[row]),
                class_id=int(class_ids[row]),
                box=(box[0] + rect[0], box[1] + rect[1], box[2], box[3]),
                mask=self.place_crop_mask(full_mask, rect),
                center=(cx, cy),
                corners=corners,
            ))
        return detections

    def decode_mask(
        self,
        coeff: np.ndarray,
        proto: np.ndarray,
        crop_shape: Tuple[int, int],
        roi_mask: np.ndarray,
        box: List[int],
        pad_x: int,
        pad_y: int,
        new_w: int,
        new_h: int,
    ) -> Optional[np.ndarray]:
        mask = sigmoid(np.matmul(coeff.astype(np.float32), proto.reshape(proto.shape[0], -1)))
        mask = mask.reshape(proto.shape[1], proto.shape[2])
        mask = cv2.resize(mask, (self.yolo_imgsz, self.yolo_imgsz), interpolation=cv2.INTER_LINEAR)
        mask = mask[pad_y:pad_y + new_h, pad_x:pad_x + new_w]
        mask = cv2.resize(mask, (crop_shape[1], crop_shape[0]), interpolation=cv2.INTER_LINEAR)
        binary = (mask >= self.mask_threshold).astype(np.uint8) * 255
        limited = np.zeros_like(binary)
        x, y, w, h = box
        limited[y:y + h, x:x + w] = binary[y:y + h, x:x + w]
        limited = cv2.bitwise_and(limited, roi_mask)
        if cv2.countNonZero(limited) < 16:
            return None
        return limited

    def place_crop_mask(self, crop_mask: np.ndarray, rect: Tuple[int, int, int, int]) -> np.ndarray:
        x0, y0, w, h = rect
        full_mask = np.zeros((max(1, y0 + h), max(1, x0 + w)), dtype=np.uint8)
        full_mask[y0:y0 + h, x0:x0 + w] = crop_mask
        return full_mask

    def mask_for_frame(self, mask: np.ndarray, shape: Tuple[int, int]) -> np.ndarray:
        output = np.zeros(shape, dtype=np.uint8)
        h = min(shape[0], mask.shape[0])
        w = min(shape[1], mask.shape[1])
        output[:h, :w] = mask[:h, :w]
        return output

    def largest_contour(self, mask: np.ndarray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= 16.0]
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def contour_corners(self, contour, offset_x: int, offset_y: int) -> List[Tuple[float, float]]:
        hull = cv2.convexHull(contour)
        if hull is None or len(hull) < 3:
            return []
        rect = cv2.minAreaRect(hull)
        if rect[1][0] < 2.0 or rect[1][1] < 2.0:
            return []
        corners = cv2.boxPoints(rect)
        return [(float(x + offset_x), float(y + offset_y)) for x, y in corners]

    def select_detection_by_confidence(
        self,
        detections: List[Detection],
        depth_m: Optional[np.ndarray],
        profile: ItemProfile,
    ) -> Optional[Detection]:
        if not detections:
            return None
        selected = max(detections, key=lambda d: d.score)
        if depth_m is None:
            self.peak_pixel = None
            return selected
        selected_mask = self.mask_for_frame(selected.mask, depth_m.shape[:2])
        self.peak_pixel = self.find_highest_peak(depth_m, selected_mask, profile)
        return selected

    def find_highest_peak(
        self,
        depth_m: np.ndarray,
        candidate_mask: np.ndarray,
        profile: ItemProfile,
    ) -> Optional[Tuple[int, int]]:
        if cv2.countNonZero(candidate_mask) == 0:
            return None
        ys, xs = np.where(candidate_mask > 0)
        if ys.size == 0:
            return None
        values = depth_m[ys, xs]
        valid = np.isfinite(values) & (values > 0.0)
        if not np.any(valid):
            return None
        xs = xs[valid]
        ys = ys[valid]
        values = values[valid]
        if profile.depth_plane.valid:
            x_norm = (xs.astype(np.float32) / max(1, depth_m.shape[1] - 1)) * 2.0 - 1.0
            y_norm = (ys.astype(np.float32) / max(1, depth_m.shape[0] - 1)) * 2.0 - 1.0
            plane = profile.depth_plane.a * x_norm + profile.depth_plane.b * y_norm + profile.depth_plane.c
            heights = -(values - plane)
            idx = int(np.argmax(heights))
        else:
            idx = int(np.argmin(values))
        return int(xs[idx]), int(ys[idx])

    def estimate_detection_pose(
        self,
        detection: Detection,
        depth_m: np.ndarray,
        info: CameraInfo,
        profile: ItemProfile,
        peak_pixel: Optional[Tuple[int, int]],
    ) -> Optional[DetectionPose]:
        pose = self.estimate_pose(detection, depth_m, info, profile, peak_pixel)
        if pose is None:
            return None
        pick_pixel = self.camera_point_to_pixel(pose.origin, info, depth_m.shape[:2])
        if pick_pixel is None:
            return None
        return DetectionPose(pose=pose, pick_pixel=pick_pixel)

    def estimate_pose(
        self,
        detection: Detection,
        depth_m: np.ndarray,
        info: CameraInfo,
        profile: ItemProfile,
        peak_pixel: Optional[Tuple[int, int]] = None,
    ) -> Optional[Pose3D]:
        if len(detection.corners) != 4 or info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            return None
        mask = self.mask_for_frame(detection.mask, depth_m.shape[:2])
        depth_mask = (mask > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
        if not np.any(depth_mask):
            return None
        pose_depth = depth_m.copy()
        if profile.depth_plane.valid and peak_pixel is not None:
            peak_x, peak_y = peak_pixel
            peak_depth = depth_m[peak_y, peak_x]
            if np.isfinite(peak_depth) and peak_depth > 0.0:
                x_norm = np.arange(depth_m.shape[1], dtype=np.float32)
                x_norm = (x_norm / max(1, depth_m.shape[1] - 1)) * 2.0 - 1.0
                y_norm = np.arange(depth_m.shape[0], dtype=np.float32)
                y_norm = (y_norm / max(1, depth_m.shape[0] - 1)) * 2.0 - 1.0
                plane = profile.depth_plane.a * x_norm[None, :] + profile.depth_plane.b * y_norm[:, None] + profile.depth_plane.c
                peak_height = -(float(peak_depth) - float(plane[peak_y, peak_x]))
                heights = -(pose_depth - plane)
                keep = heights >= (peak_height - max(1, profile.depth_window_mm) / 1000.0)
                pose_depth[~keep] = np.nan
        pose_depth[(mask == 0) | ~np.isfinite(pose_depth) | (pose_depth <= 0.0)] = np.nan

        fallback_depth = float(np.nanmedian(pose_depth))
        if not np.isfinite(fallback_depth) or fallback_depth <= 0.0:
            fallback_depth = float(np.nanmedian(depth_m[depth_mask]))
        if not np.isfinite(fallback_depth) or fallback_depth <= 0.0:
            return None

        camera_points = []
        for corner in detection.corners:
            depth = self.average_depth_at(pose_depth, corner, fallback_depth)
            camera_points.append(self.project_pixel(corner, depth, info))
        origin_idx = lower_left_corner_index(detection.corners)
        if origin_idx < 0:
            return None
        prev_idx = (origin_idx + 3) % 4
        next_idx = (origin_idx + 1) % 4
        origin_corner = camera_points[origin_idx]
        dir_a = camera_points[prev_idx] - origin_corner
        dir_b = camera_points[next_idx] - origin_corner
        len_a = float(np.linalg.norm(dir_a))
        len_b = float(np.linalg.norm(dir_b))
        if len_a < 1e-9 or len_b < 1e-9:
            return None
        x_axis = dir_a if len_a >= len_b else dir_b
        y_axis = dir_b if len_a >= len_b else dir_a
        x_axis = normalize(x_axis)
        if x_axis is None:
            return None
        y_axis = y_axis - x_axis * float(np.dot(x_axis, y_axis))
        y_axis = normalize(y_axis)
        if y_axis is None:
            return None
        z_axis = normalize(np.cross(x_axis, y_axis))
        if z_axis is None:
            return None
        if float(np.dot(z_axis, origin_corner)) > 0.0:
            y_axis *= -1.0
            z_axis = normalize(np.cross(x_axis, y_axis))
            if z_axis is None:
                return None
        pick_pixel = corner_center(detection.corners)
        pick_depth = self.average_depth_at(pose_depth, pick_pixel, fallback_depth)
        pose_origin = self.project_pixel(pick_pixel, pick_depth, info)
        rotation = np.array([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]],
        ], dtype=np.float64)
        pose = Pose3D(origin=pose_origin, rotation=rotation)
        if (
            profile.depth_plane.valid and
            profile.align_z_to_depth_plane and
            self.align_item_z_axis_to_depth_plane and
            pick_depth > 0.0
        ):
            self.align_pose_z_to_depth_plane(pose, profile, info, depth_m.shape[:2], pick_pixel)
        return pose

    def average_depth_at(self, depth_m: np.ndarray, point: Tuple[float, float], fallback: float) -> float:
        x = int(round(point[0]))
        y = int(round(point[1]))
        x0 = max(0, x - 4)
        y0 = max(0, y - 4)
        x1 = min(depth_m.shape[1], x + 5)
        y1 = min(depth_m.shape[0], y + 5)
        patch = depth_m[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size:
            return float(np.median(valid))
        return fallback

    def mask_center_3d(
        self,
        depth_m: np.ndarray,
        mask: np.ndarray,
        info: CameraInfo,
        fallback_depth: float,
    ) -> np.ndarray:
        valid = (mask > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
        ys, xs = np.where(valid)
        if xs.size:
            depths = depth_m[ys, xs]
            return np.array([
                np.median((xs.astype(np.float64) - info.k[2]) * depths / info.k[0]),
                np.median((ys.astype(np.float64) - info.k[5]) * depths / info.k[4]),
                np.median(depths),
            ], dtype=np.float64)
        moments = cv2.moments(mask)
        if abs(moments["m00"]) > 1e-6:
            center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        else:
            center = (0.0, 0.0)
        return self.project_pixel(center, fallback_depth, info)

    def project_pixel(self, point: Tuple[float, float], depth: float, info: CameraInfo) -> np.ndarray:
        x = (float(point[0]) - info.k[2]) * depth / info.k[0]
        y = (float(point[1]) - info.k[5]) * depth / info.k[4]
        return np.array([x, y, depth], dtype=np.float64)

    def camera_point_to_pixel(
        self,
        point: np.ndarray,
        info: CameraInfo,
        shape: Tuple[int, int],
    ) -> Optional[Tuple[float, float]]:
        if info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            return None
        z = float(point[2])
        if not np.isfinite(z) or z <= 1e-9:
            return None
        x = float(point[0]) * info.k[0] / z + info.k[2]
        y = float(point[1]) * info.k[4] / z + info.k[5]
        if not np.isfinite(x) or not np.isfinite(y):
            return None
        return (
            float(np.clip(x, 0.0, max(0, shape[1] - 1))),
            float(np.clip(y, 0.0, max(0, shape[0] - 1))),
        )

    def depth_plane_depth_at(
        self,
        profile: ItemProfile,
        point: Tuple[float, float],
        shape: Tuple[int, int],
    ) -> Optional[float]:
        if not profile.depth_plane.valid:
            return None
        x_norm = normalized_image_coord(int(round(point[0])), shape[1])
        y_norm = normalized_image_coord(int(round(point[1])), shape[0])
        z = profile.depth_plane.a * x_norm + profile.depth_plane.b * y_norm + profile.depth_plane.c
        return float(z) if np.isfinite(z) and z > 0.0 else None

    def align_pose_z_to_depth_plane(
        self,
        pose: Pose3D,
        profile: ItemProfile,
        info: CameraInfo,
        shape: Tuple[int, int],
        center_px: Tuple[float, float],
    ) -> None:
        p0_px = (float(center_px[0]), float(center_px[1]))
        px_px = (min(shape[1] - 1.0, p0_px[0] + max(4.0, shape[1] * 0.02)), p0_px[1])
        py_px = (p0_px[0], min(shape[0] - 1.0, p0_px[1] + max(4.0, shape[0] * 0.02)))
        z0 = self.depth_plane_depth_at(profile, p0_px, shape)
        zx = self.depth_plane_depth_at(profile, px_px, shape)
        zy = self.depth_plane_depth_at(profile, py_px, shape)
        if z0 is None or zx is None or zy is None:
            return
        p0 = self.project_pixel(p0_px, z0, info)
        px = self.project_pixel(px_px, zx, info)
        py = self.project_pixel(py_px, zy, info)
        normal = normalize(np.cross(px - p0, py - p0))
        if normal is None:
            return
        original_z = pose.rotation[:, 2]
        if float(np.dot(normal, original_z)) < 0.0:
            normal *= -1.0
        original_x = pose.rotation[:, 0]
        original_y = pose.rotation[:, 1]
        x_axis = normalize(original_x - normal * float(np.dot(original_x, normal)))
        if x_axis is None:
            x_axis = normalize(original_y - normal * float(np.dot(original_y, normal)))
        if x_axis is None:
            return
        y_axis = normalize(np.cross(normal, x_axis))
        if y_axis is None:
            return
        if float(np.dot(y_axis, original_y)) < 0.0:
            x_axis *= -1.0
            y_axis *= -1.0
        pose.rotation = np.array([
            [x_axis[0], y_axis[0], normal[0]],
            [x_axis[1], y_axis[1], normal[1]],
            [x_axis[2], y_axis[2], normal[2]],
        ], dtype=np.float64)

    def publish_pose_outputs(self, stamp, frame_id: str) -> None:
        now = time.monotonic()
        if (
            self.seek_mode_active and
            self.seek_started_time > 0.0 and
            now - self.seek_started_time > max(0.1, self.seek_window_sec)
        ):
            self.set_seek_mode(False, "Seek window expired")

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame_id
        for detection_pose in self.latest_detection_poses:
            if detection_pose is not None:
                pose_array.poses.append(pose_to_msg(detection_pose.pose))
        self.item_pose_array_pub.publish(pose_array)

        seek_pose = self.selected_pose if self.seek_mode_active else None
        if self.seek_mode_active:
            if seek_pose is not None:
                self.last_seek_pose = seek_pose
                self.last_seek_pose_time = now
            elif (
                self.last_seek_pose is not None and
                now - self.last_seek_pose_time <= max(0.0, self.seek_decay_sec)
            ):
                seek_pose = self.last_seek_pose

        if self.seek_mode_active and seek_pose is not None:
            msg = PoseStamped()
            msg.header = pose_array.header
            msg.pose = pose_to_msg(seek_pose)
            self.seek_pose_pub.publish(msg)
            self.publish_marker(stamp, frame_id, seek_pose)

    def publish_marker(self, stamp, frame_id: str, pose: Pose3D) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "item_detect"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_to_msg(pose)
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.015
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.45
        self.item_marker_pub.publish(marker)

    def display_frame(self, frame: np.ndarray, depth_m: Optional[np.ndarray]) -> np.ndarray:
        if self.view_mode == "Depth":
            return self.depth_visualization(depth_m, frame.shape)
        return frame.copy()

    def depth_visualization(self, depth_m: Optional[np.ndarray], frame_shape: Tuple[int, int, int]) -> np.ndarray:
        output = np.zeros(frame_shape, dtype=np.uint8)
        if depth_m is None:
            cv2.putText(output, "No depth image", (18, 34),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (225, 230, 236), 1, cv2.LINE_AA)
            return output
        depth = depth_m.astype(np.float32, copy=False)
        if depth.shape[:2] != frame_shape[:2]:
            depth = cv2.resize(depth, (frame_shape[1], frame_shape[0]), interpolation=cv2.INTER_NEAREST)
        valid = np.isfinite(depth) & (depth > 0.0)
        if not np.any(valid):
            cv2.putText(output, "No valid depth", (18, 34),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (225, 230, 236), 1, cv2.LINE_AA)
            return output
        values = depth[valid]
        near = float(np.percentile(values, 2.0))
        far = float(np.percentile(values, 98.0))
        if not np.isfinite(near) or not np.isfinite(far) or far <= near:
            near = float(np.min(values))
            far = float(np.max(values))
        if far <= near:
            far = near + 1e-3
        normalized = np.zeros(depth.shape[:2], dtype=np.uint8)
        normalized[valid] = np.clip((depth[valid] - near) * 255.0 / (far - near), 0.0, 255.0).astype(np.uint8)
        color_map = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
        output = cv2.applyColorMap(normalized, color_map)
        output[~valid] = (0, 0, 0)
        return output

    def draw_detection_axes(
        self,
        image: np.ndarray,
        detection: Detection,
        index: int,
        detection_pose: Optional[DetectionPose],
    ) -> None:
        if len(detection.corners) != 4:
            return
        origin_idx = lower_left_corner_index(detection.corners)
        if origin_idx < 0:
            return
        corners = [np.array(point, dtype=np.float32) for point in detection.corners]
        origin = corners[origin_idx]
        prev_corner = corners[(origin_idx + 3) % 4]
        next_corner = corners[(origin_idx + 1) % 4]
        dir_a = prev_corner - origin
        dir_b = next_corner - origin
        len_a = float(np.linalg.norm(dir_a))
        len_b = float(np.linalg.norm(dir_b))
        if len_a < 1e-3 or len_b < 1e-3:
            return
        if len_a >= len_b:
            x_dir = dir_a / len_a
            y_dir = dir_b / len_b
            x_half_len = max(8.0, len_a * 0.5)
            y_half_len = max(8.0, len_b * 0.5)
        else:
            x_dir = dir_b / len_b
            y_dir = dir_a / len_a
            x_half_len = max(8.0, len_b * 0.5)
            y_half_len = max(8.0, len_a * 0.5)
        hull_center = np.array(corner_center(detection.corners), dtype=np.float32)
        if detection_pose is not None:
            axis_center = np.array(detection_pose.pick_pixel, dtype=np.float32)
        else:
            axis_center = hull_center
        x_start = axis_center - x_dir * x_half_len
        x_end = axis_center + x_dir * x_half_len
        y_start = axis_center - y_dir * y_half_len
        y_end = axis_center + y_dir * y_half_len

        corner_points = np.asarray(
            [[int(round(point[0])), int(round(point[1]))] for point in corners],
            dtype=np.int32,
        )
        cv2.polylines(image, [corner_points], True, (255, 220, 40), 2, cv2.LINE_AA)
        for corner in corners:
            cv2.circle(image, (int(round(corner[0])), int(round(corner[1]))), 4, (255, 255, 255), -1, cv2.LINE_AA)

        center_pt = (int(round(axis_center[0])), int(round(axis_center[1])))
        x_start_pt = (int(round(x_start[0])), int(round(x_start[1])))
        x_end_pt = (int(round(x_end[0])), int(round(x_end[1])))
        y_start_pt = (int(round(y_start[0])), int(round(y_start[1])))
        y_end_pt = (int(round(y_end[0])), int(round(y_end[1])))
        cv2.line(image, x_start_pt, x_end_pt, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.line(image, y_start_pt, y_end_pt, (255, 120, 0), 2, cv2.LINE_AA)
        cv2.arrowedLine(image, center_pt, x_end_pt, (0, 0, 255), 3, cv2.LINE_AA, 0, 0.15)
        cv2.arrowedLine(image, center_pt, y_end_pt, (255, 120, 0), 3, cv2.LINE_AA, 0, 0.15)
        if detection_pose is not None:
            cv2.circle(image, center_pt, 6, (40, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(image, "X", (x_end_pt[0] + 4, x_end_pt[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(image, "Y", (y_end_pt[0] + 4, y_end_pt[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 120, 0), 2, cv2.LINE_AA)
        if detection_pose is not None:
            cv2.putText(image, str(index + 1), (center_pt[0] + 8, center_pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, (40, 255, 255), 2, cv2.LINE_AA)

    def render_overlay(self, frame: np.ndarray, depth_m: Optional[np.ndarray]) -> np.ndarray:
        output = frame.copy()
        profile = self.active_profile
        if profile is not None and len(profile.roi_points) >= 3:
            pts = np.asarray(profile.roi_points, dtype=np.int32)
            cv2.polylines(output, [pts], True, (80, 220, 255), 2)
        for index, detection in enumerate(self.latest_detections):
            mask = self.mask_for_frame(detection.mask, output.shape[:2])
            color = (90, 180, 255)
            overlay = output.copy()
            overlay[mask > 0] = color
            output = cv2.addWeighted(overlay, 0.28, output, 0.72, 0.0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, color, 2)
            detection_pose = (
                self.latest_detection_poses[index]
                if index < len(self.latest_detection_poses)
                else None
            )
            self.draw_detection_axes(output, detection, index, detection_pose)
            label = f"{detection.score * 100.0:.0f}%"
            cv2.putText(output, label, (int(round(detection.center[0])) + 8, int(round(detection.center[1])) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(output, label, (int(round(detection.center[0])) + 8, int(round(detection.center[1])) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        if self.seek_mode_active and self.selected_detection is not None:
            mask = self.mask_for_frame(self.selected_detection.mask, output.shape[:2])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, (0, 255, 0), 3)
        if self.peak_pixel is not None:
            cv2.circle(output, self.peak_pixel, 12, (0, 0, 255), 2)
            cv2.circle(output, self.peak_pixel, 3, (255, 255, 255), -1)
        title = profile.item_name if profile else "No YOLO profile"
        if self.selected_detection is not None:
            title = f"{title} {self.selected_detection.score * 100.0:.0f}%"
        cv2.putText(output, title, (18, 32),
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (240, 245, 250), 1, cv2.LINE_AA)
        return output

    def build_ui(self, preview_image: np.ndarray) -> np.ndarray:
        canvas = np.zeros((TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT, PREVIEW_CANVAS_WIDTH, 3), dtype=np.uint8)
        canvas[:] = (30, 32, 36)
        self.buttons.clear()
        self.slider_rects.clear()
        self.profile_option_rects.clear()

        bar = canvas[:TOP_BAR_HEIGHT, :]
        bar[:] = (28, 30, 34)
        self.draw_top_controls(canvas)
        self.draw_summary_panel(canvas)
        self.draw_seek_controls_panel(canvas)
        self.draw_detection_quality_panel(canvas)
        self.draw_status_strip(canvas)

        preview, scale = self.fit_preview(preview_image)
        self.preview_scale = scale
        x = 0
        y = TOP_BAR_HEIGHT
        canvas[y:y + preview.shape[0], x:x + preview.shape[1]] = preview
        self.preview_rect = (x, y, preview.shape[1], preview.shape[0])

        if self.profile_dropdown_open:
            self.draw_profile_dropdown(canvas)
        return canvas

    def draw_top_controls(self, canvas: np.ndarray) -> None:
        y = 20
        h = 40
        self.draw_button(
            canvas,
            "view_mode",
            (20, y, 150, h),
            f"View: {self.view_mode}",
            True,
            True,
            fill_color=(72, 128, 68),
            border_color=(132, 215, 150),
        )
        self.draw_button(
            canvas,
            "overlay",
            (182, y, 150, h),
            "Overlay: ON" if self.publish_overlay else "Overlay: OFF",
            True,
            self.publish_overlay,
            fill_color=(48, 62, 72),
            active_fill_color=(68, 124, 154),
            border_color=(102, 106, 112),
            active_border_color=(132, 205, 236),
        )
        self.draw_button(
            canvas,
            "seek",
            (344, y, 120, h),
            "Seek: ON" if self.seek_mode_active else "Seek: OFF",
            True,
            self.seek_mode_active,
            fill_color=(48, 62, 72),
            active_fill_color=(70, 126, 186),
            border_color=(102, 106, 112),
            active_border_color=(126, 202, 255),
        )
        self.draw_button(
            canvas,
            "profile_dropdown",
            (476, y, 274, h),
            self.profile_label(),
            bool(self.profiles),
            self.profile_dropdown_open,
            fill_color=(61, 78, 96),
            active_fill_color=(61, 78, 96),
            border_color=(130, 166, 198),
            active_border_color=(130, 166, 198),
        )
        self.draw_dropdown_arrow(canvas, (476, y, 274, h), bool(self.profiles))
        self.draw_button(
            canvas,
            "go_to_teach",
            (764, y, 148, h),
            "Go Teach..." if self.go_to_teach_in_progress else "Go To Teach",
            self.can_go_to_teach(),
            self.go_to_teach_in_progress,
            fill_color=(70, 140, 94),
            active_fill_color=(70, 126, 186),
            border_color=(134, 232, 165),
            active_border_color=(126, 202, 255),
        )
        self.draw_button(
            canvas,
            "delete_profile",
            (922, y, 148, h),
            self.delete_profile_label(),
            self.can_delete_profile(),
            self.delete_pending(),
            fill_color=(86, 76, 148),
            active_fill_color=(90, 76, 152),
            border_color=(160, 146, 246),
            active_border_color=(170, 156, 245),
        )

    def draw_summary_panel(self, canvas: np.ndarray) -> None:
        x, y, w, h = (20, 72, 344, 90)
        self.draw_panel_box(canvas, (x, y, w, h), "Item Summary")
        count = len(self.latest_detections)
        cv2.putText(canvas, f"Detected masks: {count}", (x + 12, y + 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (164, 238, 144), 1, cv2.LINE_AA)
        best_text = "Best: n/a"
        if self.selected_detection is not None:
            best_text = f"Best confidence: {self.selected_detection.score * 100.0:.0f}%"
        cv2.putText(canvas, best_text, (x + 12, y + 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (205, 212, 220), 1, cv2.LINE_AA)
        if self.seek_mode_active:
            if self.selected_pose is not None:
                xyz = self.selected_pose.origin * METERS_TO_MM
                pose_text = f"Seek pose XYZ: {xyz[0]:+.1f} {xyz[1]:+.1f} {xyz[2]:+.1f} mm"
            else:
                pose_text = "Seek armed; waiting for valid depth in mask"
        else:
            pose_text = "Seek creates pose from best mask depth"
        cv2.putText(canvas, fit_text(pose_text, 48), (x + 12, y + 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, (165, 170, 176), 1, cv2.LINE_AA)

    def draw_seek_controls_panel(self, canvas: np.ndarray) -> None:
        x, y, w, h = (374, 72, 342, 90)
        self.draw_panel_box(canvas, (x, y, w, h), "Seek Controls")
        self.draw_slider(
            canvas,
            "seek_window",
            (x + 12, y + 30, w - 24, 26),
            f"Window  {self.seek_window_sec:.1f}s",
            self.seek_window_sec,
            5.0,
            120.0,
            (140, 210, 250),
        )
        self.draw_slider(
            canvas,
            "seek_decay",
            (x + 12, y + 58, w - 24, 26),
            f"Decay  {self.seek_decay_sec:.1f}s",
            self.seek_decay_sec,
            0.0,
            5.0,
            (154, 230, 170),
        )

    def draw_detection_quality_panel(self, canvas: np.ndarray) -> None:
        x, y, w, h = (726, 72, 344, 90)
        self.draw_panel_box(canvas, (x, y, w, h), "Detection Quality")
        confidence_percent = int(round(np.clip(self.yolo_conf, 0.0, 1.0) * 100.0))
        self.draw_slider(
            canvas,
            "confidence",
            (x + 12, y + 38, w - 24, 30),
            f"Confidence  {confidence_percent}%",
            self.yolo_conf,
            0.0,
            1.0,
            (85, 225, 255),
        )

    def draw_status_strip(self, canvas: np.ndarray) -> None:
        x, y, w, h = (20, 170, 1050, 28)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (34, 36, 40), -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (72, 77, 84), 1)
        cv2.putText(canvas, f"Status   {fit_text(self.status, 112)}", (x + 12, y + 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (202, 208, 214), 1, cv2.LINE_AA)

    def draw_panel_box(self, canvas: np.ndarray, rect: Tuple[int, int, int, int], title: str) -> None:
        x, y, w, h = rect
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (38, 41, 46), -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (72, 77, 84), 1)
        cv2.rectangle(canvas, (x, y), (x + w, y + 22), (46, 50, 56), -1)
        cv2.putText(canvas, title, (x + 12, y + 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (220, 224, 230), 1, cv2.LINE_AA)

    def draw_slider(
        self,
        canvas: np.ndarray,
        name: str,
        rect: Tuple[int, int, int, int],
        label: str,
        value: float,
        min_value: float,
        max_value: float,
        color: Tuple[int, int, int],
    ) -> None:
        x, y, w, h = rect
        cv2.putText(canvas, label, (x, y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (225, 230, 236), 1, cv2.LINE_AA)
        track_x = x
        track_y = y + h - 8
        track_w = max(1, w)
        norm = 0.0
        if max_value > min_value:
            norm = (float(value) - min_value) / (max_value - min_value)
        norm = float(np.clip(norm, 0.0, 1.0))
        cv2.rectangle(canvas, (track_x, track_y - 3), (track_x + track_w, track_y + 3), (68, 73, 79), -1)
        cv2.rectangle(canvas, (track_x, track_y - 3), (track_x + track_w, track_y + 3), (93, 99, 106), 1)
        cv2.rectangle(canvas, (track_x, track_y - 2), (track_x + int(round(track_w * norm)), track_y + 2),
                      color, -1)
        knob_x = track_x + int(round(track_w * norm))
        cv2.circle(canvas, (knob_x, track_y), 8, (235, 235, 235), -1)
        cv2.circle(canvas, (knob_x, track_y), 8, (96, 100, 106), 1)
        self.slider_rects[name] = (track_x, track_y - 12, track_w, 24)

    def draw_dropdown_arrow(self, canvas: np.ndarray, rect: Tuple[int, int, int, int], enabled: bool) -> None:
        x, y, w, h = rect
        cx = x + w - 16
        cy = y + h // 2 + 2
        color = (245, 245, 245) if enabled else (150, 150, 150)
        pts = np.array([[cx - 6, cy - 4], [cx + 6, cy - 4], [cx, cy + 4]], dtype=np.int32)
        cv2.fillPoly(canvas, [pts], color)

    def fit_preview(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        h, w = image.shape[:2]
        scale = min(PREVIEW_CANVAS_WIDTH / float(w), PREVIEW_CANVAS_HEIGHT / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR), scale

    def draw_button(
        self,
        canvas: np.ndarray,
        name: str,
        rect: Tuple[int, int, int, int],
        label: str,
        enabled: bool = True,
        active: bool = False,
        fill_color: Optional[Tuple[int, int, int]] = None,
        border_color: Optional[Tuple[int, int, int]] = None,
        active_fill_color: Optional[Tuple[int, int, int]] = None,
        active_border_color: Optional[Tuple[int, int, int]] = None,
    ) -> None:
        x, y, w, h = rect
        base_fill = fill_color if fill_color is not None else (48, 62, 72)
        base_border = border_color if border_color is not None else (126, 202, 255)
        active_fill = active_fill_color if active_fill_color is not None else (62, 98, 130)
        active_border = active_border_color if active_border_color is not None else base_border
        fill = active_fill if active else (base_fill if enabled else (54, 54, 54))
        border = active_border if enabled else (100, 100, 100)
        text = (238, 242, 245) if enabled else (150, 150, 150)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2)
        cv2.putText(canvas, fit_text(label, max(8, w // 9)), (x + 12, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, text, 1, cv2.LINE_AA)
        self.buttons[name] = Button(name, rect, enabled)

    def profile_label(self) -> str:
        if self.active_profile is not None:
            return self.active_profile.label
        if not self.profiles:
            return "No YOLO profiles"
        return "Select item profile"

    def draw_profile_dropdown(self, canvas: np.ndarray) -> None:
        button = self.buttons.get("profile_dropdown")
        if button is None:
            return
        x, y, w, h = button.rect
        rows = min(MAX_DROPDOWN_ROWS, len(self.profiles))
        for i in range(rows):
            row_y = y + h + 2 + i * DROPDOWN_ROW_HEIGHT
            selected = i == self.selected_profile_index
            fill = (72, 120, 72) if selected else (38, 41, 46)
            border = (120, 255, 120) if selected else (110, 110, 110)
            rect = (x, row_y, w, DROPDOWN_ROW_HEIGHT)
            self.profile_option_rects.append(rect)
            cv2.rectangle(canvas, (x, row_y), (x + w, row_y + DROPDOWN_ROW_HEIGHT), fill, -1)
            cv2.rectangle(canvas, (x, row_y), (x + w, row_y + DROPDOWN_ROW_HEIGHT), border, 1)
            cv2.putText(canvas, fit_text(self.profiles[i].label, 48), (x + 8, row_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.47, (238, 242, 245), 1, cv2.LINE_AA)

    def mouse_callback(self, event, x: int, y: int, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONUP:
            self.active_slider = None
            return
        if event == cv2.EVENT_MOUSEMOVE and self.active_slider is not None:
            self.update_slider_from_x(self.active_slider, x)
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        for i, rect in enumerate(self.profile_option_rects):
            rx, ry, rw, rh = rect
            if rx <= x <= rx + rw and ry <= y <= ry + rh:
                self.select_profile(i)
                return

        for name, rect in list(self.slider_rects.items()):
            rx, ry, rw, rh = rect
            if rx <= x <= rx + rw and ry <= y <= ry + rh:
                self.active_slider = name
                self.update_slider_from_x(name, x)
                self.profile_dropdown_open = False
                return

        for name, button in list(self.buttons.items()):
            bx, by, bw, bh = button.rect
            if bx <= x <= bx + bw and by <= y <= by + bh:
                if not button.enabled:
                    return
                if name == "profile_dropdown":
                    self.profile_dropdown_open = not self.profile_dropdown_open
                elif name == "view_mode":
                    self.view_mode = "Depth" if self.view_mode == "RGB" else "RGB"
                    self.status = f"View: {self.view_mode}"
                elif name == "refresh_profiles":
                    self.refresh_profiles()
                elif name == "seek":
                    self.set_seek_mode(not self.seek_mode_active)
                elif name == "overlay":
                    self.publish_overlay = not self.publish_overlay
                elif name == "go_to_teach":
                    self.request_go_to_teach()
                elif name == "delete_profile":
                    self.request_delete_profile()
                return
        self.profile_dropdown_open = False

    def update_slider_from_x(self, name: str, x: int) -> None:
        rect = self.slider_rects.get(name)
        if rect is None:
            return
        rx, _, rw, _ = rect
        norm = 0.0 if rw <= 0 else (float(x) - float(rx)) / float(rw)
        norm = float(np.clip(norm, 0.0, 1.0))
        if name == "confidence":
            self.yolo_conf = norm
            self.last_inference_time = 0.0
            self.status = f"Confidence threshold: {int(round(self.yolo_conf * 100.0))}%"
        elif name == "seek_window":
            self.seek_window_sec = 5.0 + norm * 115.0
            self.status = f"Seek window: {self.seek_window_sec:.1f}s"
        elif name == "seek_decay":
            self.seek_decay_sec = norm * 5.0
            self.status = f"Seek decay: {self.seek_decay_sec:.1f}s"

    def handle_seek(self, request, response):
        del request
        self.set_seek_mode(not self.seek_mode_active)
        response.success = True
        response.message = "Seek armed" if self.seek_mode_active else "Seek cancelled"
        return response

    def handle_seek_complete(self, request, response):
        del request
        self.set_seek_mode(False, "Seek released by item pick")
        response.success = True
        response.message = "Seek released by item pick"
        return response

    def handle_seek_status(self, request, response):
        del request
        response.success = self.seek_mode_active
        response.message = "Seek: ON" if self.seek_mode_active else "Seek: OFF"
        return response

    def handle_go_to_teach(self, request, response):
        del request
        response.success = self.request_go_to_teach()
        response.message = self.status
        return response

    def set_seek_mode(self, active: bool, message: Optional[str] = None) -> None:
        self.seek_mode_active = active
        if active:
            self.seek_started_time = time.monotonic()
            self.last_inference_time = 0.0
            self.status = message or "Seek armed; choosing highest-confidence mask"
        else:
            self.seek_started_time = 0.0
            self.last_seek_pose = None
            self.last_seek_pose_time = 0.0
            self.selected_pose = None
            self.peak_pixel = None
            self.status = message or "Seek cancelled"

    def can_go_to_teach(self) -> bool:
        profile = self.active_profile
        return (
            profile is not None and
            profile.has_teach_joints and
            len(profile.teach_joints_deg) >= 6 and
            not self.go_to_teach_in_progress
        )

    def can_delete_profile(self) -> bool:
        return self.active_profile is not None and not self.go_to_teach_in_progress

    def delete_pending(self) -> bool:
        profile = self.active_profile
        return (
            profile is not None and
            self.pending_delete_profile == profile.path and
            time.monotonic() < self.pending_delete_deadline
        )

    def delete_profile_label(self) -> str:
        return "Confirm Delete" if self.delete_pending() else "Delete Item"

    def request_delete_profile(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.status = "Delete Item: select an item profile"
            return False
        if not self.delete_pending():
            self.pending_delete_profile = profile.path
            self.pending_delete_deadline = time.monotonic() + 3.0
            self.status = f"Click Delete again to remove {profile.item_name} profile and model"
            return False
        try:
            deleted_name = profile.path.name
            self.ort_session = None
            profile.path.unlink()
            deleted_models = self.delete_model_artifacts(profile)
            if deleted_models:
                deleted_message = f"Deleted profile {deleted_name} and model folder"
            else:
                deleted_message = f"Deleted profile {deleted_name}; model folder missing or outside model root"
        except Exception as exc:
            self.status = f"Delete Item failed: {exc}"
            self.get_logger().warn(self.status)
            self.pending_delete_profile = None
            self.pending_delete_deadline = 0.0
            return False
        self.pending_delete_profile = None
        self.pending_delete_deadline = 0.0
        self.active_profile = None
        self.selected_profile_index = -1
        self.ort_session = None
        self.latest_detections = []
        self.latest_detection_poses = []
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        self.refresh_profiles()
        self.status = deleted_message if self.profiles else f"{deleted_message}; no profiles remaining"
        return True

    def delete_model_artifacts(self, profile: ItemProfile) -> List[Path]:
        deleted: List[Path] = []
        candidates: List[Path] = []
        for path_text in [profile.model_dir]:
            if path_text:
                candidates.append(resolve_path(path_text))
        for path_text in [profile.model_path, profile.model_pt_path]:
            if not path_text:
                continue
            path = resolve_path(path_text)
            candidates.append(path.parent if path.suffix else path)

        seen = set()
        for candidate in candidates:
            try:
                resolved = candidate.resolve()
            except Exception:
                continue
            if resolved in seen:
                continue
            seen.add(resolved)
            if not self.is_safe_model_path(resolved):
                continue
            try:
                if resolved.is_dir():
                    shutil.rmtree(resolved)
                    deleted.append(resolved)
                elif resolved.is_file():
                    resolved.unlink()
                    deleted.append(resolved)
            except Exception as exc:
                self.get_logger().warn(f"Delete Item: could not delete model path {resolved}: {exc}")
        return deleted

    def is_safe_model_path(self, path: Path) -> bool:
        try:
            resolved_root = self.model_root.resolve()
            resolved_path = path.resolve()
            if resolved_path == resolved_root:
                return False
            resolved_path.relative_to(resolved_root)
            return True
        except Exception:
            return False

    def request_go_to_teach(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.status = "Go to Teach: select an item profile"
            return False
        if not profile.has_teach_joints or len(profile.teach_joints_deg) < 6:
            self.status = "Go to Teach: selected profile has no teach joints"
            return False
        if self.go_to_teach_in_progress:
            self.status = "Go to Teach: command in progress"
            return False
        if not self.movj_client.service_is_ready():
            self.status = "Go to Teach: MovJ service not ready"
            return False

        request = MovJ.Request()
        request.mode = True
        request.a = float(profile.teach_joints_deg[0])
        request.b = float(profile.teach_joints_deg[1])
        request.c = float(profile.teach_joints_deg[2])
        request.d = float(profile.teach_joints_deg[3])
        request.e = float(profile.teach_joints_deg[4])
        request.f = float(profile.teach_joints_deg[5])
        request.param_value = []

        self.go_to_teach_in_progress = True
        self.status = "Go to Teach: sending MovJ"
        self.get_logger().info(
            f"Go to Teach MovJ -> {self.movj_service_name} with joints (deg): "
            f"[{request.a:.3f}, {request.b:.3f}, {request.c:.3f}, "
            f"{request.d:.3f}, {request.e:.3f}, {request.f:.3f}]"
        )
        future = self.movj_client.call_async(request)
        future.add_done_callback(self.handle_go_to_teach_done)
        return True

    def handle_go_to_teach_done(self, future) -> None:
        try:
            response = future.result()
            ok = response is not None and response.res != -1
            if ok:
                self.status = "Go to Teach: MovJ accepted"
                self.get_logger().info(
                    f"Go to Teach: MovJ accepted "
                    f"(res={response.res}, robot_return={response.robot_return})"
                )
            else:
                self.status = "Go to Teach: MovJ failed"
                res = response.res if response is not None else -999
                robot_return = response.robot_return if response is not None else "null"
                self.get_logger().warn(
                    f"Go to Teach: MovJ failed (res={res}, robot_return={robot_return})"
                )
        except Exception as exc:
            self.status = "Go to Teach: MovJ error"
            self.get_logger().warn(f"Go to Teach: MovJ call failed: {exc}")
        finally:
            self.go_to_teach_in_progress = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemDetectYoloNode()
    try:
        rclpy.spin(node)
    finally:
        if node.start_visualization:
            cv2.destroyWindow(WINDOW_NAME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
