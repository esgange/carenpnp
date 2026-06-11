#include "aruco_perception/aruco_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

namespace aruco_perception
{
namespace
{
constexpr int OVERLAY_INIT_WIDTH = 960;
constexpr int OVERLAY_INIT_HEIGHT = 540;
constexpr char kOverlayWindowName[] = "aruco_overlay";
constexpr double DEFAULT_AXIS_SCALE = 0.45;
constexpr double DEFAULT_AXIS_MIN_LEN = 0.015;  // meters

bool isOpenCvWindowClosed(const std::string &window_name)
{
  try
  {
    return cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) < 1.0;
  }
  catch (const cv::Exception &)
  {
    return true;
  }
}

void destroyOpenCvWindowQuietly(const std::string &window_name)
{
  try
  {
    cv::destroyWindow(window_name);
  }
  catch (const cv::Exception &)
  {
  }
}

double stampDiffSec(const rclcpp::Time &a, const rclcpp::Time &b)
{
  return std::fabs((a - b).seconds());
}

std::string normalizeFrameId(const std::string &frame_id)
{
  std::string normalized = frame_id;
  while (!normalized.empty() && normalized.front() == '/')
  {
    normalized.erase(normalized.begin());
  }
  return normalized;
}
}  // namespace

ArucoDetectorNode::ArucoDetectorNode()
: Node("aruco_detector")
{
  color_topic_ = this->declare_parameter<std::string>(
    "color_topic", "/robot_camera/color/image_raw");
  depth_topic_ = this->declare_parameter<std::string>(
    "depth_topic", "/robot_camera/depth/image_raw");
  camera_info_topic_ = this->declare_parameter<std::string>(
    "camera_info_topic", "/robot_camera/color/camera_info");
  overlay_topic_ = this->declare_parameter<std::string>(
    "overlay_topic", "/aruco_overlay");
  detections_topic_ = this->declare_parameter<std::string>(
    "detections_topic", "/aruco_detections");
  use_calibration_ = this->declare_parameter<bool>("use_calibration", true);
  publish_static_calibration_tf_ = this->declare_parameter<bool>("publish_static_calibration_tf", true);
  publish_marker_tfs_ = this->declare_parameter<bool>("publish_marker_tfs", true);
  calibration_file_ = this->declare_parameter<std::string>("calibration_file", "");
  calibration_parent_frame_ = this->declare_parameter<std::string>("calibration_parent_frame", "Link6");
  calibration_child_frame_ = this->declare_parameter<std::string>(
    "calibration_child_frame", "arm_calibrated_camera_link");
  marker_frame_prefix_ = "aruco_marker";
  const std::string default_camera_frame =
    use_calibration_ ? calibration_child_frame_ : std::string("robot_camera_color_optical_frame");
  camera_frame_id_ = this->declare_parameter<std::string>("camera_frame", default_camera_frame);
  camera_frame_id_ = normalizeFrameId(camera_frame_id_);
  target_marker_id_ = -1;
  sync_tolerance_sec_ = 0.1;
  depth_average_kernel_ = 5;
  show_overlay_window_ = this->declare_parameter<bool>("show_overlay_window", true);
  publish_overlay_ = this->declare_parameter<bool>("publish_overlay", true);
  publish_viz_ = publish_overlay_ || show_overlay_window_;
  overlay_rate_hz_ = this->declare_parameter<double>("overlay_rate_hz", 10.0);
  depth_colormap_max_ = 1.5;
  overlay_axis_scale_ = DEFAULT_AXIS_SCALE;
  overlay_axis_min_len_ = DEFAULT_AXIS_MIN_LEN;

  if (depth_average_kernel_ < 1)
  {
    depth_average_kernel_ = 1;
  }
  if (depth_average_kernel_ % 2 == 0)
  {
    ++depth_average_kernel_;
  }

  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  detector_params_ = cv::aruco::DetectorParameters::create();

  if (use_calibration_)
  {
    if (calibration_file_.empty())
    {
      throw std::runtime_error(
        "use_calibration=true but calibration_file is empty. "
        "Set calibration_file to a valid YAML path.");
    }

    std::string reason;
    if (!loadCalibrationFromFile(calibration_file_, calibration_rotation_, calibration_translation_, reason))
    {
      throw std::runtime_error("Failed to load calibration file '" + calibration_file_ + "': " + reason);
    }

    if (camera_frame_id_ != calibration_child_frame_)
    {
      RCLCPP_WARN(
        get_logger(),
        "camera_frame (%s) differs from calibration_child_frame (%s). "
        "Using calibration_child_frame for marker outputs.",
        camera_frame_id_.c_str(), calibration_child_frame_.c_str());
      camera_frame_id_ = calibration_child_frame_;
    }

    if (publish_static_calibration_tf_)
    {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      publishCalibrationTransform();
    }
  }

  if (show_overlay_window_)
  {
    cv::namedWindow(kOverlayWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(kOverlayWindowName, OVERLAY_INIT_WIDTH, OVERLAY_INIT_HEIGHT);
    cv::setMouseCallback(kOverlayWindowName, &ArucoDetectorNode::onMouseThunk, this);
  }

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", rclcpp::QoS(10));
  detections_pub_ = this->create_publisher<aruco_perception::msg::MarkerDetections>(
    detections_topic_, rclcpp::SensorDataQoS());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  if (publish_overlay_)
  {
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(overlay_topic_, rclcpp::QoS(5));
  }

  configureCameraSubscriptions();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArucoDetectorNode::handleParameterUpdate, this, std::placeholders::_1));

  if (publish_viz_)
  {
    camera_status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ArucoDetectorNode::renderNoCameraTopicsOverlay, this));
  }

  RCLCPP_INFO(
    get_logger(), "Aruco detector ready. Color: %s Depth: %s Info: %s Camera frame: %s Detections: %s",
    color_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(),
    camera_frame_id_.c_str(), detections_topic_.c_str());
  RCLCPP_INFO(
    get_logger(), "Aruco overlay publish: %s window: %s rate: %.1f Hz",
    publish_overlay_ ? "enabled" : "disabled",
    show_overlay_window_ ? "enabled" : "disabled",
    overlay_rate_hz_);
  if (use_calibration_)
  {
    RCLCPP_INFO(
      get_logger(),
      "Calibration loaded from %s. Publishing %s -> %s in-detector: %s",
      calibration_file_.c_str(), calibration_parent_frame_.c_str(), calibration_child_frame_.c_str(),
      publish_static_calibration_tf_ ? "enabled" : "disabled");
  }
  else
  {
    RCLCPP_INFO(
      get_logger(),
      "Calibration disabled. Marker poses will be published in frame: %s",
      camera_frame_id_.c_str());
  }
}

ArucoDetectorNode::~ArucoDetectorNode()
{
  if (show_overlay_window_)
  {
    destroyOpenCvWindowQuietly(kOverlayWindowName);
  }
}

void ArucoDetectorNode::configureCameraSubscriptions()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_color_.reset();
    last_depth_.reset();
    last_info_.reset();
    has_info_ = false;
  }

  color_sub_ = this->create_subscription<ImageMsg>(
    color_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::colorCallback, this, std::placeholders::_1));
  depth_sub_ = this->create_subscription<ImageMsg>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::depthCallback, this, std::placeholders::_1));
  info_sub_ = this->create_subscription<CameraInfoMsg>(
    camera_info_topic_, rclcpp::QoS(10).best_effort(),
    std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ArucoDetectorNode::handleParameterUpdate(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::string new_color_topic = color_topic_;
  std::string new_depth_topic = depth_topic_;
  std::string new_camera_info_topic = camera_info_topic_;
  std::string new_camera_frame = camera_frame_id_;

  for (const auto &parameter : parameters)
  {
    const auto &name = parameter.get_name();
    if (name != "color_topic" && name != "depth_topic" && name != "camera_info_topic" &&
        name != "camera_frame")
    {
      continue;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
      result.successful = false;
      result.reason = name + " must be a string.";
      return result;
    }
    const std::string value = parameter.as_string();
    if (name == "camera_frame")
    {
      new_camera_frame = normalizeFrameId(value);
      if (new_camera_frame.empty())
      {
        result.successful = false;
        result.reason = name + " must be non-empty.";
        return result;
      }
      continue;
    }
    if (value.empty() || value.front() != '/')
    {
      result.successful = false;
      result.reason = name + " must be non-empty and an absolute ROS topic.";
      return result;
    }
    if (name == "color_topic")
    {
      new_color_topic = value;
    }
    else if (name == "depth_topic")
    {
      new_depth_topic = value;
    }
    else if (name == "camera_info_topic")
    {
      new_camera_info_topic = value;
    }
  }

  const bool topics_changed =
    new_color_topic != color_topic_ ||
    new_depth_topic != depth_topic_ ||
    new_camera_info_topic != camera_info_topic_;
  if (!topics_changed)
  {
    if (new_camera_frame != camera_frame_id_)
    {
      camera_frame_id_ = new_camera_frame;
      RCLCPP_INFO(get_logger(), "Camera frame updated: %s", camera_frame_id_.c_str());
    }
    return result;
  }

  color_topic_ = new_color_topic;
  depth_topic_ = new_depth_topic;
  camera_info_topic_ = new_camera_info_topic;
  camera_frame_id_ = new_camera_frame;
  configureCameraSubscriptions();
  RCLCPP_INFO(
    get_logger(),
    "Camera topics updated. Color: %s Depth: %s Info: %s Frame: %s",
    color_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(),
    camera_frame_id_.c_str());
  return result;
}

void ArucoDetectorNode::colorCallback(const ImageMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_color_ = msg;
  }
  tryProcessFrame();
}

void ArucoDetectorNode::depthCallback(const ImageMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_depth_ = msg;
  }
  tryProcessFrame();
}

void ArucoDetectorNode::cameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_info_)
    {
      camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                        msg->k[3], msg->k[4], msg->k[5],
                        msg->k[6], msg->k[7], msg->k[8]);
      has_info_ = true;
    }
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_info_ = msg;
  }
  tryProcessFrame();
}

void ArucoDetectorNode::tryProcessFrame()
{
  processPendingReset();

  ImageMsg::ConstSharedPtr color;
  ImageMsg::ConstSharedPtr depth;
  CameraInfoMsg::ConstSharedPtr info;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!last_color_ || !last_depth_ || !last_info_ || !has_info_)
    {
      return;
    }

    const rclcpp::Time color_stamp(last_color_->header.stamp);
    const rclcpp::Time depth_stamp(last_depth_->header.stamp);

    if (stampDiffSec(color_stamp, depth_stamp) > sync_tolerance_sec_)
    {
      return;
    }

    color = last_color_;
    depth = last_depth_;
    info = last_info_;

    last_color_.reset();
    last_depth_.reset();
  }

  processFrame(color, depth, info);
}

void ArucoDetectorNode::processFrame(const ImageMsg::ConstSharedPtr &color,
                                     const ImageMsg::ConstSharedPtr &depth,
                                     const CameraInfoMsg::ConstSharedPtr &info)
{
  if (!has_info_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Waiting for camera intrinsics.");
    return;
  }

  cv_bridge::CvImageConstPtr color_cv;
  cv_bridge::CvImageConstPtr depth_cv;

  const std::string color_frame = normalizeFrameId(color->header.frame_id);
  const std::string depth_frame = normalizeFrameId(depth->header.frame_id);
  const std::string info_frame = normalizeFrameId(info->header.frame_id);
  if (!color_frame.empty() && color_frame != camera_frame_id_)
  {
    if (!use_calibration_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Color image frame_id is '%s' but detector camera_frame is '%s'; not publishing ArUco poses.",
        color_frame.c_str(), camera_frame_id_.c_str());
      return;
    }
    RCLCPP_INFO_ONCE(
      get_logger(),
      "Color image frame_id is '%s'; calibrated ArUco poses will be published in '%s'.",
      color_frame.c_str(), camera_frame_id_.c_str());
  }
  const std::string expected_info_frame = color_frame.empty() ? camera_frame_id_ : color_frame;
  if (!info_frame.empty() && info_frame != expected_info_frame)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "CameraInfo frame_id is '%s' but color/camera frame is '%s'; not publishing ArUco poses.",
      info_frame.c_str(), expected_info_frame.c_str());
    return;
  }
  if (!depth_frame.empty() && !color_frame.empty() && depth_frame != color_frame)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "Depth image frame_id is '%s' while color frame_id is '%s'; continuing only because "
      "depth_registration/align-to-color is expected.",
      depth_frame.c_str(), color_frame.c_str());
  }
  if (!std::isfinite(info->k[0]) || !std::isfinite(info->k[4]) ||
      info->k[0] <= 1e-6 || info->k[4] <= 1e-6)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "Invalid camera intrinsics fx=%.6f fy=%.6f; not publishing ArUco poses.",
      info->k[0], info->k[4]);
    return;
  }

  try
  {
    color_cv = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_WARN(get_logger(), "Failed to convert color image: %s", ex.what());
    return;
  }

  try
  {
    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth->encoding == sensor_msgs::image_encodings::MONO16)
    {
      depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_64FC1)
    {
      depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_64FC1);
    }
    else
    {
      RCLCPP_WARN_ONCE(get_logger(),
                       "Unsupported depth encoding: %s. Expect 16UC1 or 32FC1.",
                       depth->encoding.c_str());
      return;
    }
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_WARN(get_logger(), "Failed to convert depth image: %s", ex.what());
    return;
  }

  if (depth_cv->image.cols != color_cv->image.cols || depth_cv->image.rows != color_cv->image.rows)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "Depth image size (%d x %d) does not match color image size (%d x %d); "
      "depth must be registered to color for ArUco pose estimation.",
      depth_cv->image.cols, depth_cv->image.rows, color_cv->image.cols, color_cv->image.rows);
    return;
  }

  cv::Mat depth_filtered = filterDepth(depth_cv->image);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(color_cv->image, dictionary_, corners, ids, detector_params_);

  std::vector<MarkerPose> detected_markers;
  detected_markers.reserve(ids.size());

  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (target_marker_id_ >= 0 && ids[i] != target_marker_id_)
    {
      continue;
    }

    auto cam_points_opt = cornersToPoints(corners[i], depth_filtered, *info);
    if (!cam_points_opt)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Could not read valid depth for marker %d corners.", ids[i]);
      continue;
    }

    auto center_override = centerPointFromDepth(corners[i], depth_filtered, *info);
    auto pose_opt = estimatePoseFrom3D(*cam_points_opt, center_override);
    if (!pose_opt)
    {
      RCLCPP_WARN(get_logger(), "Pose estimation failed for marker %d.", ids[i]);
      continue;
    }

    MarkerPose marker;
    marker.id = ids[i];
    marker.cam_corners = *cam_points_opt;
    marker.center_cam = pose_opt->translation();
    marker.pose = *pose_opt;
    marker.image_corners = corners[i];
    detected_markers.push_back(marker);
  }

  std::vector<MarkerPose> selected_markers = detected_markers;

  publishDetections(color->header.stamp, *color, selected_markers);

  std::unordered_map<int, int> id_totals;
  for (const auto &marker : selected_markers)
  {
    ++id_totals[marker.id];
  }
  std::unordered_map<int, int> id_instances;
  for (auto &marker : selected_markers)
  {
    auto pose_msg = toPoseMsg(marker.pose, color->header.stamp);
    pose_pub_->publish(pose_msg);

    const int instance_index = ++id_instances[marker.id];
    std::string marker_frame = marker_frame_prefix_ + "_" + std::to_string(marker.id);
    if (id_totals[marker.id] > 1)
    {
      marker_frame += "_" + std::to_string(instance_index);
    }
    if (publish_marker_tfs_)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = pose_msg.header.stamp;
      tf_msg.header.frame_id = camera_frame_id_;
      tf_msg.child_frame_id = marker_frame;
      tf_msg.transform.translation.x = marker.pose.translation().x();
      tf_msg.transform.translation.y = marker.pose.translation().y();
      tf_msg.transform.translation.z = marker.pose.translation().z();

      Eigen::Quaterniond q(marker.pose.rotation());
      q.normalize();
      tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
      tf_msg.transform.rotation = tf2::toMsg(tf_q);
      tf_broadcaster_->sendTransform(tf_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Marker %d pose published as %s in frame %s.",
        marker.id, marker_frame.c_str(), camera_frame_id_.c_str());
    }
  }

  if (publish_viz_)
  {
    const rclcpp::Time now = this->now();
    const bool rate_allows_publish =
      overlay_rate_hz_ <= 0.0 ||
      last_overlay_stamp_.nanoseconds() == 0 ||
      (now - last_overlay_stamp_).seconds() >= (1.0 / overlay_rate_hz_);
    if (rate_allows_publish)
    {
      last_overlay_stamp_ = now;
      publishOverlay(color->header.stamp, color_cv->image, depth_filtered, *info, selected_markers);
    }
  }
}

void ArucoDetectorNode::publishDetections(const rclcpp::Time &image_stamp,
                                          const ImageMsg &color,
                                          const std::vector<MarkerPose> &markers)
{
  if (!detections_pub_)
  {
    return;
  }

  aruco_perception::msg::MarkerDetections msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = camera_frame_id_;
  msg.image_stamp = image_stamp;
  msg.image_width = color.width;
  msg.image_height = color.height;
  msg.ids.reserve(markers.size());
  msg.poses.reserve(markers.size());
  msg.pixel_centers.reserve(markers.size());
  msg.pixel_corners.reserve(markers.size());
  msg.camera_centers.reserve(markers.size());
  msg.camera_corners.reserve(markers.size());

  for (const auto &marker : markers)
  {
    msg.ids.push_back(marker.id);
    auto pose_msg = toPoseMsg(marker.pose, image_stamp);
    msg.poses.push_back(pose_msg.pose);

    cv::Point2f center_px(0.0F, 0.0F);
    for (const auto &corner : marker.image_corners)
    {
      center_px += corner;
    }
    if (!marker.image_corners.empty())
    {
      center_px *= (1.0F / static_cast<float>(marker.image_corners.size()));
    }
    geometry_msgs::msg::Point center_msg;
    center_msg.x = static_cast<double>(center_px.x);
    center_msg.y = static_cast<double>(center_px.y);
    center_msg.z = 0.0;
    msg.pixel_centers.push_back(center_msg);

    geometry_msgs::msg::Polygon corner_msg;
    corner_msg.points.reserve(marker.image_corners.size());
    for (const auto &corner : marker.image_corners)
    {
      geometry_msgs::msg::Point32 point;
      point.x = corner.x;
      point.y = corner.y;
      point.z = 0.0F;
      corner_msg.points.push_back(point);
    }
    msg.pixel_corners.push_back(corner_msg);

    geometry_msgs::msg::Point camera_center_msg;
    camera_center_msg.x = marker.center_cam.x();
    camera_center_msg.y = marker.center_cam.y();
    camera_center_msg.z = marker.center_cam.z();
    msg.camera_centers.push_back(camera_center_msg);

    geometry_msgs::msg::Polygon camera_corner_msg;
    camera_corner_msg.points.reserve(marker.cam_corners.size());
    for (const auto &corner : marker.cam_corners)
    {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(corner.x());
      point.y = static_cast<float>(corner.y());
      point.z = static_cast<float>(corner.z());
      camera_corner_msg.points.push_back(point);
    }
    msg.camera_corners.push_back(camera_corner_msg);
  }

  detections_pub_->publish(msg);
}

void ArucoDetectorNode::publishOverlay(const rclcpp::Time &stamp, const cv::Mat &color_bgr,
                                       const cv::Mat &depth_image, const CameraInfoMsg &info,
                                       const std::vector<MarkerPose> &markers)
{
  if (!overlay_pub_ && !show_overlay_window_)
  {
    return;
  }

  cv::Mat depth_float;
  if (depth_image.type() == CV_16UC1)
  {
    depth_image.convertTo(depth_float, CV_32FC1, 0.001);  // mm -> m
  }
  else if (depth_image.type() == CV_32FC1)
  {
    depth_float = depth_image;
  }
  else if (depth_image.type() == CV_64FC1)
  {
    depth_image.convertTo(depth_float, CV_32FC1);
  }
  else
  {
    return;
  }

  cv::Mat valid_mask = (depth_float > 0.0) & (depth_float == depth_float);
  double max_depth = depth_colormap_max_ > 0.0 ? depth_colormap_max_ : 1.0;
  cv::Mat depth_clamped;
  cv::min(depth_float, max_depth, depth_clamped);
  cv::Mat depth_norm;
  depth_clamped.convertTo(depth_norm, CV_8UC1, 255.0 / max_depth);
  depth_norm.setTo(0, ~valid_mask);

  cv::Mat depth_color;
  cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

  cv::Mat color_overlay = color_bgr.clone();
  cv::Mat depth_overlay = depth_color.clone();
  if (depth_overlay.size() != color_overlay.size())
  {
    cv::resize(depth_overlay, depth_overlay, color_overlay.size());
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  ids.reserve(markers.size());
  corners.reserve(markers.size());
  for (const auto &marker : markers)
  {
    ids.push_back(marker.id);
    corners.push_back(marker.image_corners);
  }

  if (!ids.empty())
  {
    for (size_t i = 0; i < markers.size(); ++i)
    {
      const auto &c = markers[i].image_corners;
      const cv::Point pts[1][4] = {{
        cv::Point(static_cast<int>(std::lround(c[0].x)), static_cast<int>(std::lround(c[0].y))),
        cv::Point(static_cast<int>(std::lround(c[1].x)), static_cast<int>(std::lround(c[1].y))),
        cv::Point(static_cast<int>(std::lround(c[2].x)), static_cast<int>(std::lround(c[2].y))),
        cv::Point(static_cast<int>(std::lround(c[3].x)), static_cast<int>(std::lround(c[3].y))) }};
      const cv::Point *ppt[1] = {pts[0]};
      int npt[] = {4};
      cv::polylines(color_overlay, ppt, npt, 1, true, cv::Scalar(0, 255, 0), 2);
      cv::polylines(depth_overlay, ppt, npt, 1, true, cv::Scalar(0, 255, 0), 2);

      const auto center_px = (markers[i].image_corners[0] +
                              markers[i].image_corners[1] +
                              markers[i].image_corners[2] +
                              markers[i].image_corners[3]) * 0.25f;
      const std::string label = "id=" + std::to_string(markers[i].id);
      cv::putText(color_overlay, label,
                  cv::Point(static_cast<int>(center_px.x), static_cast<int>(center_px.y) - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      cv::putText(depth_overlay, label,
                  cv::Point(static_cast<int>(center_px.x), static_cast<int>(center_px.y) - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info.k[0], info.k[1], info.k[2],
                          info.k[3], info.k[4], info.k[5],
                          info.k[6], info.k[7], info.k[8]);
    cv::Mat dist_coeffs;
    if (!info.d.empty())
    {
      dist_coeffs = cv::Mat(info.d).clone();
    }
    else
    {
      dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    }

    for (const auto &marker : markers)
    {
      double edge_sum = 0.0;
      edge_sum += (marker.cam_corners[0] - marker.cam_corners[1]).norm();
      edge_sum += (marker.cam_corners[1] - marker.cam_corners[2]).norm();
      edge_sum += (marker.cam_corners[2] - marker.cam_corners[3]).norm();
      edge_sum += (marker.cam_corners[3] - marker.cam_corners[0]).norm();
      const double avg_edge = edge_sum / 4.0;
      const double axis_len = std::max(overlay_axis_min_len_, avg_edge * overlay_axis_scale_);

      const Eigen::Matrix3d R = marker.pose.linear();
      const Eigen::Vector3d origin = marker.pose.translation();

      cv::Mat rmat(3, 3, CV_64F);
      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 3; ++c)
        {
          rmat.at<double>(r, c) = R(r, c);
        }
      }
      cv::Mat rvec;
      cv::Rodrigues(rmat, rvec);
      cv::Mat tvec = (cv::Mat_<double>(3, 1) << origin.x(), origin.y(), origin.z());

      cv::aruco::drawAxis(color_overlay, camera_matrix, dist_coeffs, rvec, tvec, axis_len);
      cv::aruco::drawAxis(depth_overlay, camera_matrix, dist_coeffs, rvec, tvec, axis_len);
    }
  }

  cv::Mat stacked;
  cv::vconcat(color_overlay, depth_overlay, stacked);

  cv::Scalar fill_color(40, 60, 40);
  cv::Scalar border_color(160, 220, 160);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (reset_button_pressed_)
    {
      fill_color = cv::Scalar(70, 95, 70);
      border_color = cv::Scalar(190, 255, 190);
    }
  }
  cv::rectangle(stacked, reset_button_rect_, fill_color, cv::FILLED);
  cv::rectangle(stacked, reset_button_rect_, border_color, 2);
  cv::putText(
    stacked,
    "Reset Node",
    cv::Point(reset_button_rect_.x + 14, reset_button_rect_.y + 27),
    cv::FONT_HERSHEY_SIMPLEX,
    0.65,
    cv::Scalar(245, 245, 245),
    2,
    cv::LINE_AA);

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = camera_frame_id_;
  auto out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, stacked).toImageMsg();
  if (overlay_pub_)
  {
    overlay_pub_->publish(*out_msg);
  }
  if (show_overlay_window_)
  {
    cv::imshow(kOverlayWindowName, stacked);
    processOverlayWindowEvents();
  }
  last_overlay_render_time_ = std::chrono::steady_clock::now();
}

void ArucoDetectorNode::renderNoCameraTopicsOverlay()
{
  if (!overlay_pub_ && !show_overlay_window_)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (last_overlay_render_time_.time_since_epoch().count() != 0 &&
      now - last_overlay_render_time_ < std::chrono::seconds(1))
  {
    return;
  }

  cv::Mat placeholder(
    OVERLAY_INIT_HEIGHT,
    OVERLAY_INIT_WIDTH,
    CV_8UC3,
    cv::Scalar(18, 20, 24));

  cv::putText(
    placeholder,
    "no camera topics...",
    cv::Point(48, 120),
    cv::FONT_HERSHEY_SIMPLEX,
    1.15,
    cv::Scalar(0, 210, 255),
    3,
    cv::LINE_AA);

  const std::array<std::string, 3> status_lines = {
    "color: " + color_topic_ + "  publishers=" + std::to_string(count_publishers(color_topic_)),
    "depth: " + depth_topic_ + "  publishers=" + std::to_string(count_publishers(depth_topic_)),
    "info:  " + camera_info_topic_ + "  publishers=" + std::to_string(count_publishers(camera_info_topic_))};

  int y = 180;
  for (const auto &line : status_lines)
  {
    cv::putText(
      placeholder,
      line,
      cv::Point(50, y),
      cv::FONT_HERSHEY_SIMPLEX,
      0.62,
      cv::Scalar(225, 230, 235),
      2,
      cv::LINE_AA);
    y += 42;
  }

  cv::Scalar fill_color(40, 60, 40);
  cv::Scalar border_color(160, 220, 160);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (reset_button_pressed_)
    {
      fill_color = cv::Scalar(70, 95, 70);
      border_color = cv::Scalar(190, 255, 190);
    }
  }
  cv::rectangle(placeholder, reset_button_rect_, fill_color, cv::FILLED);
  cv::rectangle(placeholder, reset_button_rect_, border_color, 2);
  cv::putText(
    placeholder,
    "Reset Node",
    cv::Point(reset_button_rect_.x + 14, reset_button_rect_.y + 27),
    cv::FONT_HERSHEY_SIMPLEX,
    0.65,
    cv::Scalar(245, 245, 245),
    2,
    cv::LINE_AA);

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = camera_frame_id_;
  auto out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, placeholder).toImageMsg();
  if (overlay_pub_)
  {
    overlay_pub_->publish(*out_msg);
  }
  if (show_overlay_window_)
  {
    cv::imshow(kOverlayWindowName, placeholder);
    processOverlayWindowEvents();
  }
  last_overlay_render_time_ = now;
}

void ArucoDetectorNode::onMouseThunk(int event, int x, int y, int flags, void *userdata)
{
  (void)flags;
  auto *self = static_cast<ArucoDetectorNode *>(userdata);
  if (self != nullptr)
  {
    self->onMouse(event, x, y, flags);
  }
}

void ArucoDetectorNode::onMouse(int event, int x, int y, int flags)
{
  (void)flags;
  const cv::Point p(x, y);
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (event == cv::EVENT_LBUTTONDOWN && reset_button_rect_.contains(p))
  {
    reset_button_pressed_ = true;
    reset_requested_ = true;
    return;
  }
  if (event == cv::EVENT_LBUTTONUP)
  {
    reset_button_pressed_ = false;
  }
}

void ArucoDetectorNode::processOverlayWindowEvents()
{
  cv::waitKey(1);
  if (isOpenCvWindowClosed(kOverlayWindowName))
  {
    requestShutdownFromWindowClose();
  }
}

void ArucoDetectorNode::requestShutdownFromWindowClose()
{
  if (overlay_window_close_requested_)
  {
    return;
  }
  overlay_window_close_requested_ = true;
  RCLCPP_INFO(get_logger(), "aruco_overlay window closed; shutting down.");
  if (camera_status_timer_)
  {
    camera_status_timer_->cancel();
  }
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void ArucoDetectorNode::processPendingReset()
{
  bool do_reset = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (reset_requested_)
    {
      do_reset = true;
      reset_requested_ = false;
      reset_button_pressed_ = false;
    }
  }
  if (do_reset)
  {
    resetDetectorState();
  }
}

void ArucoDetectorNode::resetDetectorState()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_color_.reset();
    last_depth_.reset();
    last_info_.reset();
  }
  if (use_calibration_ && publish_static_calibration_tf_)
  {
    publishCalibrationTransform();
  }
  RCLCPP_INFO(get_logger(), "Reset button clicked: cleared detector frame buffers.");
}

std::optional<double> ArucoDetectorNode::depthAt(const cv::Mat &depth, int u, int v) const
{
  const int half_k = depth_average_kernel_ / 2;
  double sum = 0.0;
  int count = 0;

  for (int dv = -half_k; dv <= half_k; ++dv)
  {
    for (int du = -half_k; du <= half_k; ++du)
    {
      const int x = u + du;
      const int y = v + dv;
      if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
      {
        continue;
      }

      double depth_m = std::numeric_limits<double>::quiet_NaN();
      if (depth.type() == CV_16UC1)
      {
        uint16_t val = depth.at<uint16_t>(y, x);
        if (val == 0)
        {
          continue;
        }
        depth_m = static_cast<double>(val) * 0.001;
      }
      else if (depth.type() == CV_32FC1)
      {
        float val = depth.at<float>(y, x);
        depth_m = static_cast<double>(val);
      }
      else if (depth.type() == CV_64FC1)
      {
        depth_m = depth.at<double>(y, x);
      }

      if (std::isfinite(depth_m) && depth_m > 0.0)
      {
        sum += depth_m;
        ++count;
      }
    }
  }

  if (count == 0)
  {
    return std::nullopt;
  }
  return sum / static_cast<double>(count);
}

std::optional<Eigen::Vector3d> ArucoDetectorNode::centerPointFromDepth(
  const std::vector<cv::Point2f> &corners, const cv::Mat &depth,
  const CameraInfoMsg &info) const
{
  if (corners.empty())
  {
    return std::nullopt;
  }

  cv::Point2f avg(0.0F, 0.0F);
  for (const auto &pt : corners)
  {
    avg += pt;
  }
  avg *= (1.0F / static_cast<float>(corners.size()));

  const int u = static_cast<int>(std::lround(avg.x));
  const int v = static_cast<int>(std::lround(avg.y));
  auto depth_m = depthAt(depth, u, v);
  if (!depth_m)
  {
    return std::nullopt;
  }

  return projectPixel(avg.x, avg.y, *depth_m, info);
}

std::optional<std::array<Eigen::Vector3d, 4>> ArucoDetectorNode::cornersToPoints(
  const std::vector<cv::Point2f> &corners, const cv::Mat &depth, const CameraInfoMsg &info) const
{
  if (corners.size() != 4)
  {
    return std::nullopt;
  }

  std::array<Eigen::Vector3d, 4> points;
  for (size_t i = 0; i < 4; ++i)
  {
    const int u = static_cast<int>(std::lround(corners[i].x));
    const int v = static_cast<int>(std::lround(corners[i].y));
    auto depth_m = depthAt(depth, u, v);
    if (!depth_m)
    {
      return std::nullopt;
    }
    points[i] = projectPixel(corners[i].x, corners[i].y, *depth_m, info);
  }
  return points;
}

Eigen::Vector3d ArucoDetectorNode::projectPixel(double u, double v, double depth,
                                                const CameraInfoMsg &info) const
{
  const double fx = info.k[0];
  const double fy = info.k[4];
  const double cx = info.k[2];
  const double cy = info.k[5];

  const double x = (u - cx) * depth / fx;
  const double y = (v - cy) * depth / fy;
  return Eigen::Vector3d(x, y, depth);
}

std::optional<Eigen::Isometry3d> ArucoDetectorNode::estimatePoseFrom3D(
  const std::array<Eigen::Vector3d, 4> &cam_points,
  const std::optional<Eigen::Vector3d> &center_override) const
{
  const Eigen::Vector3d p0 = cam_points[0];
  const Eigen::Vector3d p1 = cam_points[1];
  const Eigen::Vector3d p3 = cam_points[3];

  Eigen::Vector3d x_axis = p1 - p0;
  Eigen::Vector3d y_axis = p3 - p0;

  if (x_axis.norm() < 1e-6 || y_axis.norm() < 1e-6)
  {
    return std::nullopt;
  }

  x_axis.normalize();
  y_axis = y_axis - x_axis * (x_axis.dot(y_axis));
  if (y_axis.norm() < 1e-6)
  {
    return std::nullopt;
  }
  y_axis.normalize();
  Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  if (z_axis.norm() < 1e-6)
  {
    return std::nullopt;
  }
  z_axis.normalize();

  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;

  Eigen::Vector3d center = center_override
                             ? *center_override
                             : (cam_points[0] + cam_points[1] + cam_points[2] + cam_points[3]) / 4.0;

  if (z_axis.dot(center) > 0.0)
  {
    // Flip only Z so the normal points toward the camera, then rebuild an orthonormal basis.
    z_axis = -z_axis;
    x_axis = y_axis.cross(z_axis);
    if (x_axis.norm() < 1e-6)
    {
      return std::nullopt;
    }
    x_axis.normalize();
    y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
  }

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = R;
  iso.translation() = center;
  return iso;
}

geometry_msgs::msg::PoseStamped ArucoDetectorNode::toPoseMsg(const Eigen::Isometry3d &pose,
                                                             const rclcpp::Time &stamp) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = camera_frame_id_;
  msg.pose.position.x = pose.translation().x();
  msg.pose.position.y = pose.translation().y();
  msg.pose.position.z = pose.translation().z();

  Eigen::Quaterniond q(pose.rotation());
  q.normalize();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

cv::Point2f ArucoDetectorNode::projectPointToPixel(const Eigen::Vector3d &pt,
                                                   const CameraInfoMsg &info) const
{
  const double fx = info.k[0];
  const double fy = info.k[4];
  const double cx = info.k[2];
  const double cy = info.k[5];

  const double u = fx * (pt.x() / pt.z()) + cx;
  const double v = fy * (pt.y() / pt.z()) + cy;
  return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

cv::Mat ArucoDetectorNode::filterDepth(const cv::Mat &depth_image_raw)
{
  if (depth_image_raw.type() == CV_16UC1)
  {
    cv::Mat depth_float;
    depth_image_raw.convertTo(depth_float, CV_32FC1, 0.001);  // mm -> m
    return depth_float;
  }
  if (depth_image_raw.type() == CV_32FC1)
  {
    return depth_image_raw;
  }
  if (depth_image_raw.type() == CV_64FC1)
  {
    cv::Mat depth_float;
    depth_image_raw.convertTo(depth_float, CV_32FC1);
    return depth_float;
  }
  return depth_image_raw;
}

bool ArucoDetectorNode::loadCalibrationFromFile(const std::string &path, Eigen::Quaterniond &q,
                                                Eigen::Vector3d &t, std::string &reason) const
{
  std::string resolved_path = path;
  if (!resolved_path.empty() && resolved_path[0] == '~')
  {
    const char *home = std::getenv("HOME");
    if (home == nullptr)
    {
      reason = "path uses '~' but HOME is not set";
      return false;
    }
    if (resolved_path.size() == 1U)
    {
      resolved_path = home;
    }
    else if (resolved_path[1] == '/')
    {
      resolved_path = std::string(home) + resolved_path.substr(1);
    }
  }

  try
  {
    const std::filesystem::path p(resolved_path);
    if (!std::filesystem::exists(p))
    {
      reason = "file does not exist";
      return false;
    }
    if (!std::filesystem::is_regular_file(p))
    {
      reason = "path is not a regular file";
      return false;
    }
    if (std::filesystem::file_size(p) == 0)
    {
      reason = "file is empty";
      return false;
    }
  }
  catch (const std::exception &ex)
  {
    reason = std::string("filesystem error: ") + ex.what();
    return false;
  }

  YAML::Node root;
  try
  {
    root = YAML::LoadFile(resolved_path);
  }
  catch (const std::exception &ex)
  {
    reason = std::string("failed to read YAML: ") + ex.what();
    return false;
  }

  const auto calib = root["transform"];
  if (!calib)
  {
    reason = "missing 'transform'";
    return false;
  }
  const auto rot = calib["rotation"];
  const auto trans = calib["translation"];
  if (!rot || !trans)
  {
    reason = "missing rotation/translation block";
    return false;
  }

  try
  {
    const double w = rot["w"].as<double>();
    const double x = rot["x"].as<double>();
    const double y = rot["y"].as<double>();
    const double z = rot["z"].as<double>();
    q = Eigen::Quaterniond(w, x, y, z);
    t = Eigen::Vector3d(
      trans["x"].as<double>(),
      trans["y"].as<double>(),
      trans["z"].as<double>());
  }
  catch (const std::exception &ex)
  {
    reason = std::string("failed to parse quaternion/translation: ") + ex.what();
    return false;
  }

  if (q.norm() < 1e-9)
  {
    reason = "invalid quaternion (zero norm)";
    return false;
  }
  q.normalize();
  return true;
}

void ArucoDetectorNode::publishCalibrationTransform()
{
  if (!static_tf_broadcaster_)
  {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = calibration_parent_frame_;
  tf_msg.child_frame_id = calibration_child_frame_;
  tf_msg.transform.translation.x = calibration_translation_.x();
  tf_msg.transform.translation.y = calibration_translation_.y();
  tf_msg.transform.translation.z = calibration_translation_.z();
  tf2::Quaternion q(
    calibration_rotation_.x(),
    calibration_rotation_.y(),
    calibration_rotation_.z(),
    calibration_rotation_.w());
  tf_msg.transform.rotation = tf2::toMsg(q);
  static_tf_broadcaster_->sendTransform(tf_msg);
}
}  // namespace aruco_perception

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_perception::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
