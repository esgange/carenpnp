#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <deque>
#include <exception>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <orbbec_camera_msgs/srv/set_int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dobot_msgs_v4/srv/mov_j.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

#include <dobot_common/robot_identity.hpp>
#include <dobot_common/workspace_paths.hpp>

namespace
{
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using MovJSrv = dobot_msgs_v4::srv::MovJ;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetInt32Srv = orbbec_camera_msgs::srv::SetInt32;
using PoseArrayMsg = geometry_msgs::msg::PoseArray;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using MarkerMsg = visualization_msgs::msg::Marker;
using StringMsg = std_msgs::msg::String;
using TriggerSrv = std_srvs::srv::Trigger;

constexpr double kMinOutlierDistancePx = 4.0;
constexpr int kMaxSideTrimIterations = 24;
constexpr int kDefaultPreviousColorPercent = 60;
constexpr double kNextColorConfirmMatchRatio = 0.60;
constexpr char kDetectWindowName[] = "item_detect_view";
constexpr int kTopBarBaseHeight = 206;
constexpr int kDropdownRowHeight = 34;
constexpr int kToleranceHitPadding = 14;
constexpr int kPreviewCanvasWidth = 1080;
constexpr int kPreviewCanvasHeight = 680;
constexpr double kMetersToMillimeters = 1000.0;
constexpr double kCentimetersToMillimeters = 10.0;
constexpr double kSquareCentimetersToSquareMillimeters = 100.0;
constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr int kDepthThresholdMinMm = 1;
constexpr int kDepthThresholdMaxMm = 20;
constexpr int kDepthEdgeOffsetMinPx = 1;
constexpr int kDepthEdgeOffsetMaxPx = 20;
constexpr int kRgbHoleFillMin = 0;
constexpr int kRgbHoleFillMax = 100;
constexpr int kRgbDilateMinPx = 1;
constexpr int kRgbDilateMaxPx = 20;
constexpr int kDepthFillSensitivityMin = 0;
constexpr int kDepthFillSensitivityMax = 100;
constexpr int kDepthWindowMinMm = 1;
constexpr int kDepthWindowMaxMm = 100;
constexpr int kDepthTrimMinPx = 0;
constexpr int kDepthTrimMaxPx = 30;
constexpr int kAdaptiveDepthTrimAddMinPx = 1;
constexpr int kAdaptiveDepthTrimAddMaxPx = 50;
constexpr int kAdaptiveDepthTrimAddDefaultPx = 2;
constexpr int kAdaptiveDepthTrimHeightMinMm = 100;
constexpr int kAdaptiveDepthTrimHeightMaxMm = 400;
constexpr int kAdaptiveDepthTrimHeightDefaultMm = 200;
constexpr int kBlobToleranceMinPercent = 1;
constexpr int kBlobToleranceMaxPercent = 50;
constexpr int kBlobToleranceDefaultPercent = 10;
constexpr int kExposurePercentMin = 0;
constexpr int kExposurePercentMax = 100;
constexpr int kDefaultExposureMinUs = 1;
constexpr int kDefaultExposureMaxUs = 32000;
constexpr int kPoseReferenceSlotCount = 4;
constexpr int kSeekMotionHistoryMaxSamples = 10;
constexpr double kSeekResultFreezeSeconds = 3.0;

bool isOpenCvWindowClosed(const std::string &window_name)
{
  static bool window_was_visible = false;
  static const auto first_check_time = std::chrono::steady_clock::now();

  double visible = -1.0;
  try
  {
    visible = cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE);
  }
  catch (const cv::Exception &)
  {
    return window_was_visible;
  }

  if (visible >= 1.0)
  {
    window_was_visible = true;
    return false;
  }

  const double age_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - first_check_time).count();

  // Some OpenCV HighGUI backends briefly report WND_PROP_VISIBLE == 0 while
  // the window is still being mapped. Do not treat that startup state as a
  // user close. Once the window was visible, visible < 1 means it was closed.
  if (!window_was_visible && age_sec < 2.0)
  {
    return false;
  }

  return window_was_visible && visible < 1.0;
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

std::string shellQuote(const std::string &value)
{
  std::string quoted = "'";
  for (const char ch : value)
  {
    if (ch == '\'')
    {
      quoted += "'\\''";
    }
    else
    {
      quoted.push_back(ch);
    }
  }
  quoted.push_back('\'');
  return quoted;
}

std::string trimTrailingLineEndings(std::string text)
{
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
  {
    text.pop_back();
  }
  return text;
}

bool pathsReferToSameFile(const std::filesystem::path &a, const std::filesystem::path &b)
{
  std::error_code ec;
  if (std::filesystem::exists(a, ec) && std::filesystem::exists(b, ec))
  {
    if (std::filesystem::equivalent(a, b, ec))
    {
      return true;
    }
  }
  return a.lexically_normal() == b.lexically_normal();
}

int clampAdaptiveDepthTrimAddPx(int add_px)
{
  return std::clamp(add_px, kAdaptiveDepthTrimAddMinPx, kAdaptiveDepthTrimAddMaxPx);
}

int clampExposurePercent(int percent)
{
  return std::clamp(percent, kExposurePercentMin, kExposurePercentMax);
}

int clampExposureUsec(int value)
{
  return std::max(1, value);
}

int exposurePercentToUsec(int percent, int min_us, int max_us)
{
  const int clamped_percent = clampExposurePercent(percent);
  if (clamped_percent <= 0)
  {
    return 0;
  }
  const int clamped_min = clampExposureUsec(min_us);
  const int clamped_max = std::max(clamped_min, clampExposureUsec(max_us));
  const double t = static_cast<double>(clamped_percent) / 100.0;
  return std::clamp(
    static_cast<int>(std::lround(static_cast<double>(clamped_min) + t * static_cast<double>(clamped_max - clamped_min))),
    clamped_min,
    clamped_max);
}

int clampExposureUsecOrAuto(int value, int min_us, int max_us)
{
  if (value <= 0)
  {
    return 0;
  }
  const int clamped_min = clampExposureUsec(min_us);
  const int clamped_max = std::max(clamped_min, clampExposureUsec(max_us));
  return std::clamp(value, clamped_min, clamped_max);
}

double loadReferenceBlobFillRatio(const YAML::Node &params)
{
  if (const YAML::Node reference_fill_ratio = params["reference_blob_fill_ratio"];
    reference_fill_ratio)
  {
    return std::clamp(reference_fill_ratio.as<double>(), 0.0, 1.0);
  }
  return 0.0;
}

std::optional<double> loadAuxiliaryBlobFillRatio(const YAML::Node &params)
{
  if (const YAML::Node auxiliary_fill_ratio = params["auxiliary_blob_fill_ratio"];
    auxiliary_fill_ratio)
  {
    return std::clamp(auxiliary_fill_ratio.as<double>(), 0.0, 1.0);
  }
  return std::nullopt;
}

int parseSavedAdaptiveDepthTrimAddPx(
  const YAML::Node &node,
  int fallback_add_px = kAdaptiveDepthTrimAddDefaultPx)
{
  if (!node || !node.IsScalar())
  {
    return clampAdaptiveDepthTrimAddPx(fallback_add_px);
  }

  return clampAdaptiveDepthTrimAddPx(node.as<int>());
}

int clampAdaptiveDepthTrimHeightMm(int height_mm)
{
  return std::clamp(height_mm, kAdaptiveDepthTrimHeightMinMm, kAdaptiveDepthTrimHeightMaxMm);
}

builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time &stamp)
{
  builtin_interfaces::msg::Time output;
  const int64_t total_ns = stamp.nanoseconds();
  output.sec = static_cast<int32_t>(total_ns / 1000000000LL);
  const int64_t remainder_ns = total_ns % 1000000000LL;
  if (remainder_ns < 0)
  {
    output.sec -= 1;
    output.nanosec = static_cast<uint32_t>(remainder_ns + 1000000000LL);
  }
  else
  {
    output.nanosec = static_cast<uint32_t>(remainder_ns);
  }
  return output;
}

double remapOutlierSensitivityToFitRange(int outlier_sensitivity)
{
  const int clamped = std::clamp(outlier_sensitivity, 1, 100);
  return 50.0 + (static_cast<double>(clamped - 1) * 100.0 / 99.0);
}

double fittedRectAreaPx(const std::vector<cv::Point> &points)
{
  if (points.size() < 3)
  {
    return 0.0;
  }
  const cv::RotatedRect rect = cv::minAreaRect(points);
  return std::max(0.0, static_cast<double>(rect.size.width) * static_cast<double>(rect.size.height));
}

float fittedRectAspectRatio(const std::vector<cv::Point> &points)
{
  if (points.size() < 3)
  {
    return 1.0F;
  }
  const cv::RotatedRect rect = cv::minAreaRect(points);
  const float width = std::max(rect.size.width, 1e-3F);
  const float height = std::max(rect.size.height, 1e-3F);
  return width / height;
}

double polygonAreaPx(const std::vector<cv::Point> &points)
{
  if (points.size() < 3)
  {
    return 0.0;
  }
  return std::fabs(cv::contourArea(points));
}

double computeBlobHullFillRatio(
  const std::vector<cv::Point> &pixels,
  const std::vector<cv::Point> &hull)
{
  if (pixels.empty() || hull.size() < 3)
  {
    return 0.0;
  }

  const double hull_area_px = polygonAreaPx(hull);
  if (!std::isfinite(hull_area_px) || hull_area_px < 1.0)
  {
    return 0.0;
  }

  return std::clamp(
    static_cast<double>(pixels.size()) / hull_area_px,
    0.0,
    1.0);
}

struct LineModel
{
  cv::Point2f point;
  cv::Point2f direction;
};

struct SideFitResult
{
  LineModel line;
  std::vector<cv::Point2f> inliers;
};

struct ItemEstimate
{
  cv::RotatedRect rect;
  bool has_metric_estimate {false};
  double area_cm2 {0.0};
  double mean_depth_m {0.0};
  std::array<double, 4> edge_lengths_cm {0.0, 0.0, 0.0, 0.0};
  cv::Point2f center;
  std::vector<cv::Point2f> edge_points;
  std::vector<cv::Point2f> filtered_edge_points;
  std::vector<cv::Point2f> corners;
  std::vector<LineModel> side_lines;
  std::vector<cv::Point> polygon;
};

struct ItemMetricEstimate
{
  double area_cm2 {0.0};
  double mean_depth_m {0.0};
  // Ordered as: origin X edge, opposite X edge, origin Y edge, opposite Y edge.
  std::array<double, 4> edge_lengths_cm {0.0, 0.0, 0.0, 0.0};
};

struct DetectedItemDimensions
{
  double length_mm {0.0};
  double width_mm {0.0};
  double area_mm2 {0.0};
  std::array<double, 4> edge_lengths_mm {0.0, 0.0, 0.0, 0.0};
};

struct ItemPose3D
{
  cv::Vec3d origin;
  cv::Matx33d rotation;
};

enum class PoseTemplateMode2D
{
  kSingle = 0,
  kPair,
};

struct BinarizedPoseEstimate2D
{
  struct BlobPose2D
  {
    int label {0};
    std::vector<cv::Point> pixels;
    std::vector<cv::Point> hull_points;
    std::vector<cv::Point2f> corners;
    cv::Point2f origin;
    cv::Point2f x_axis_tip;
    cv::Point2f z_axis_tip;
    float x_length_px {0.0F};
    float z_length_px {0.0F};
    bool has_custom_anchor {false};
    cv::Point2f anchor_point_px;
    std::vector<cv::Point> anchor_pixels;
    std::vector<cv::Point> companion_pixels;
    int member_count {1};
    std::vector<int> member_labels;
    std::vector<cv::Point2f> member_centers_px;
    std::vector<cv::Point2f> member_centers_norm;
  };

  std::vector<BlobPose2D> blob_poses;
  int matched_blob_count {0};
};

struct BinarizedBlobComponent2D
{
  int label {0};
  int area_px {0};
  cv::Rect bbox;
  float aspect_ratio {1.0F};
  double fill_ratio {1.0};
  std::vector<cv::Point> pixels;
  std::vector<cv::Point> hull;
};

struct PoseBlobReference2D
{
  PoseTemplateMode2D mode {PoseTemplateMode2D::kSingle};
  int area_px {0};
  float aspect_ratio {1.0F};
  double fill_ratio {1.0};
  double companion_fill_ratio {0.0};
  std::vector<cv::Point> hull;
  std::vector<cv::Point> anchor_hull;
  std::vector<cv::Point> companion_hull;
  int companion_area_px {0};
  float companion_aspect_ratio {0.0F};
  int group_area_px {0};
  float group_aspect_ratio {1.0F};
  std::vector<cv::Point> group_hull;
  int member_count {1};
  std::vector<cv::Point2f> member_centers_norm;
  cv::Point2f anchor_center_norm;
};

struct PoseReferenceSlot2D
{
  int slot_index {0};
  PoseBlobReference2D reference;
};

struct DirectShapeFitResult2D
{
  double score {-std::numeric_limits<double>::infinity()};
  double fill_ratio {0.0};
  double angle_deg {0.0};
  double scale {1.0};
  cv::Point2f reference_center_px;
  cv::Point2f transform_offset_px;
  std::vector<cv::Point2f> polygon_points;
  std::vector<cv::Point> pixels;
};

struct CompanionSearchDebug2D
{
  std::vector<cv::Point2f> hull;
  cv::Point2f center;
  int radius_px {0};
  double angle_offset_deg {0.0};
};

struct PairDirectShapeFitDebug2D
{
  std::vector<cv::Point2f> predicted_companion_hull;
  cv::Point2f predicted_companion_center;
  std::vector<CompanionSearchDebug2D> companion_searches;
  int search_radius_px {0};
};

std::string poseTemplateModeToString(PoseTemplateMode2D mode)
{
  switch (mode)
  {
    case PoseTemplateMode2D::kPair:
      return "pair";
    case PoseTemplateMode2D::kSingle:
    default:
      return "single";
  }
}

PoseTemplateMode2D parsePoseTemplateMode(const std::string &mode_text)
{
  std::string token;
  token.reserve(mode_text.size());
  for (const unsigned char ch : mode_text)
  {
    if (std::isspace(ch) != 0)
    {
      continue;
    }
    token.push_back(static_cast<char>(std::tolower(ch)));
  }
  return token == "pair" || token == "group" || token == "two" || token == "2"
    ? PoseTemplateMode2D::kPair
    : PoseTemplateMode2D::kSingle;
}

struct SeekCapture
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  ItemPose3D pose;
  std::string frame_id;
  cv::Mat frame;
};

struct SeekMotionSample
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  ItemPose3D pose;
};

struct TimedItemEstimate
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  ItemEstimate estimate;
};

struct AxisAlignedRoiBounds
{
  int left {0};
  int top {0};
  int right {0};
  int bottom {0};
};

struct CameraCalibrationMetadata
{
  std::string calibration_type;
  std::string robot_base_frame;
  std::string transform_child_frame;
  std::string tracking_base_frame;
};

bool isValidRoiBounds(const AxisAlignedRoiBounds &bounds);
std::optional<AxisAlignedRoiBounds> roiBoundsFromSelection(const std::vector<cv::Point2f> &points);
std::vector<cv::Point2f> roiPointsFromBounds(const AxisAlignedRoiBounds &bounds);
std::vector<cv::Point2f> denormalizeRoiPoints(
  const std::vector<cv::Point2f> &normalized_points,
  const cv::Size &image_size);
std::optional<AxisAlignedRoiBounds> denormalizeRoiBounds(
  const std::array<double, 4> &normalized_bounds,
  const cv::Size &image_size);
std::optional<AxisAlignedRoiBounds> combinedRoiBounds(const std::vector<AxisAlignedRoiBounds> &roi_regions);
std::vector<cv::Point2f> mergeRoiRegionsIntoPolygon(const std::vector<AxisAlignedRoiBounds> &roi_regions);
int lowerLeftCornerIndex(const std::vector<cv::Point2f> &corners);
double medianValue(std::vector<double> values);
cv::Vec3d projectPixelToCamera(const cv::Point2f &pixel, double depth_m, const CameraInfoMsg &camera_info);

struct ItemProfile
{
  std::filesystem::path path;
  std::string item_name;
  std::string associated_bin_name;
  std::string bin_teach_file;
  std::string teach_date;
  std::string display_label;
  std::string color_topic;
  std::string depth_topic;
  std::string camera_info_topic;
  std::string overlay_topic;
  bool detection_use_depth {false};
  int depth_threshold_mm {10};
	  int red_threshold {120};
	  int green_threshold {120};
	  int blue_threshold {120};
	  int color_exposure_percent {0};
	  int depth_exposure_percent {0};
	  int color_exposure_us {0};
	  int depth_exposure_us {0};
	  int color_exposure_min_us {kDefaultExposureMinUs};
	  int color_exposure_max_us {kDefaultExposureMaxUs};
	  int depth_exposure_min_us {kDefaultExposureMinUs};
	  int depth_exposure_max_us {kDefaultExposureMaxUs};
	  int rgb_hole_fill_sensitivity {0};
  int rgb_mask_dilate_px {kRgbDilateMinPx};
  int depth_null_fill_sensitivity {0};
  int depth_window_mm {5};
  int depth_hole_fill_sensitivity {0};
  int depth_trim_px {0};
  int adaptive_depth_trim_max_add_px {kAdaptiveDepthTrimAddDefaultPx};
  int adaptive_depth_trim_max_height_mm {kAdaptiveDepthTrimHeightDefaultMm};
  int ray_step_px {3};
  int depth_edge_offset_px {4};
  int previous_color_percent {kDefaultPreviousColorPercent};
  int horizontal_ray_count {50};
  int vertical_ray_count {50};
  int outlier_sensitivity {50};
  bool detect_black_to_white {true};
  bool focus_black_mask {false};
  bool trace_out_to_in {false};
  bool has_pose_blob_reference {false};
  PoseBlobReference2D pose_blob_reference;
  std::vector<PoseReferenceSlot2D> pose_reference_slots;
  bool depth_plane_enabled {false};
  double depth_plane_a {0.0};
  double depth_plane_b {0.0};
  double depth_plane_c {0.0};
  double depth_plane_reference_depth_m {0.0};
  std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds;
  std::optional<std::array<double, 4>> depth_plane_roi_normalized;
  std::vector<AxisAlignedRoiBounds> roi_regions;
  std::vector<cv::Point2f> roi_points;
  std::vector<cv::Point2f> roi_points_normalized;
  int roi_image_width {0};
  int roi_image_height {0};
  std::array<double, 6> teach_joints_deg {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_teach_joints {false};
  bool has_tool_teach {false};
  bool has_taught_item_dimensions {false};
  double taught_item_length_mm {0.0};
  double taught_item_width_mm {0.0};
};

struct DepthPlaneModel
{
  bool valid {false};
  double a {0.0};
  double b {0.0};
  double c {0.0};
  double reference_depth_m {0.0};
};

struct SideDepthSample
{
  double t {0.0};
  double depth_m {0.0};
  cv::Vec3d camera_point {0.0, 0.0, 0.0};
};

std::vector<cv::Point2f> buildAxisAlignedRoiFromSelection(const std::vector<cv::Point2f> &points)
{
  if (points.size() < 2)
  {
    return points;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto &point : points)
  {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  return {
    cv::Point2f(min_x, min_y),
    cv::Point2f(max_x, min_y),
    cv::Point2f(max_x, max_y),
    cv::Point2f(min_x, max_y),
  };
}

bool hasValidEdgeLengthsCm(const std::array<double, 4> &edge_lengths_cm)
{
  return std::all_of(
    edge_lengths_cm.begin(),
    edge_lengths_cm.end(),
    [](double length_cm)
    {
      return length_cm > 0.0;
    });
}

std::string buildProfileLabel(
  const std::string &item_name,
  const std::string &associated_bin_name,
  const std::string &teach_date,
  const std::filesystem::path &path)
{
  std::string name = item_name.empty() ? path.stem().string() : item_name;
  if (!associated_bin_name.empty())
  {
    name += " @ " + associated_bin_name;
  }
  if (!teach_date.empty())
  {
    return name + " | " + teach_date;
  }
  if (path.empty())
  {
    return name.empty() ? "Select item profile" : name;
  }
  return name + " | " + path.filename().string();
}

std::string normalizeDetectionModeToken(const std::string &mode_text)
{
  std::string normalized;
  normalized.reserve(mode_text.size());
  for (const unsigned char ch : mode_text)
  {
    if (std::isspace(ch) != 0)
    {
      continue;
    }
    normalized.push_back(static_cast<char>(std::tolower(ch)));
  }
  return normalized;
}

bool isDepthDetectionMode(const std::string &mode_text)
{
  const std::string token = normalizeDetectionModeToken(mode_text);
  return token == "depth" || token == "d" || token == "true" || token == "1";
}

std::string detectionModeToString(bool depth_mode_enabled)
{
  return depth_mode_enabled ? "depth" : "rgb";
}

bool isPairPoseReference(const PoseBlobReference2D &reference)
{
  return reference.mode == PoseTemplateMode2D::kPair || reference.member_count >= 2;
}

bool poseBlobReferencesMatchForProfileIdentity(
  bool has_reference_a,
  const PoseBlobReference2D &reference_a,
  bool has_reference_b,
  const PoseBlobReference2D &reference_b)
{
  if (has_reference_a != has_reference_b)
  {
    return false;
  }
  if (!has_reference_a)
  {
    return true;
  }
  const int area_a =
    isPairPoseReference(reference_a) && reference_a.group_area_px > 0
    ? reference_a.group_area_px
    : reference_a.area_px;
  const int area_b =
    isPairPoseReference(reference_b) && reference_b.group_area_px > 0
    ? reference_b.group_area_px
    : reference_b.area_px;
  return
    reference_a.mode == reference_b.mode &&
    reference_a.member_count == reference_b.member_count &&
    std::abs(area_a - area_b) <= 1;
}

std::optional<ItemProfile> loadItemProfileFile(const std::filesystem::path &path)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(path.string());
    YAML::Node params;
    if (root["item_detect"] && root["item_detect"]["ros__parameters"])
    {
      params = root["item_detect"]["ros__parameters"];
    }
    if (!params || !params.IsMap())
    {
      return std::nullopt;
    }

    ItemProfile profile;
    profile.path = path;
    profile.has_tool_teach = root["tool_teach"] && root["tool_teach"].IsMap();
    profile.color_topic = params["color_topic"] ? params["color_topic"].as<std::string>() : "/bin_camera/color/image_raw";
    profile.depth_topic = params["depth_topic"] ? params["depth_topic"].as<std::string>() : "/bin_camera/depth/image_raw";
    profile.camera_info_topic = params["camera_info_topic"] ? params["camera_info_topic"].as<std::string>() : "/bin_camera/color/camera_info";
    profile.overlay_topic = params["overlay_topic"] ? params["overlay_topic"].as<std::string>() : "bin_overlay";
    if (params["detection_mode"])
    {
      profile.detection_use_depth = isDepthDetectionMode(params["detection_mode"].as<std::string>());
    }
    profile.depth_threshold_mm = std::clamp(
      params["depth_threshold_mm"] ? params["depth_threshold_mm"].as<int>() : 10,
      kDepthThresholdMinMm,
      kDepthThresholdMaxMm);
	    profile.red_threshold = params["red_threshold"] ? params["red_threshold"].as<int>() : 120;
	    profile.green_threshold = params["green_threshold"] ? params["green_threshold"].as<int>() : 120;
	    profile.blue_threshold = params["blue_threshold"] ? params["blue_threshold"].as<int>() : 120;
    if (params["color_exposure_percent"])
    {
      profile.color_exposure_percent = clampExposurePercent(params["color_exposure_percent"].as<int>());
    }
    profile.depth_exposure_percent = params["depth_exposure_percent"]
      ? clampExposurePercent(params["depth_exposure_percent"].as<int>())
      : profile.depth_exposure_percent;
	    profile.color_exposure_min_us = params["color_exposure_min_us"]
	      ? clampExposureUsec(params["color_exposure_min_us"].as<int>())
	      : profile.color_exposure_min_us;
	    profile.color_exposure_max_us = params["color_exposure_max_us"]
	      ? std::max(profile.color_exposure_min_us, clampExposureUsec(params["color_exposure_max_us"].as<int>()))
	      : profile.color_exposure_max_us;
	    profile.depth_exposure_min_us = params["depth_exposure_min_us"]
	      ? clampExposureUsec(params["depth_exposure_min_us"].as<int>())
	      : profile.depth_exposure_min_us;
	    profile.depth_exposure_max_us = params["depth_exposure_max_us"]
	      ? std::max(profile.depth_exposure_min_us, clampExposureUsec(params["depth_exposure_max_us"].as<int>()))
	      : profile.depth_exposure_max_us;
	    profile.color_exposure_us = params["color_exposure_us"]
	      ? clampExposureUsecOrAuto(params["color_exposure_us"].as<int>(), profile.color_exposure_min_us, profile.color_exposure_max_us)
	      : exposurePercentToUsec(profile.color_exposure_percent, profile.color_exposure_min_us, profile.color_exposure_max_us);
	    profile.depth_exposure_us = params["depth_exposure_us"]
	      ? clampExposureUsecOrAuto(params["depth_exposure_us"].as<int>(), profile.depth_exposure_min_us, profile.depth_exposure_max_us)
	      : exposurePercentToUsec(profile.depth_exposure_percent, profile.depth_exposure_min_us, profile.depth_exposure_max_us);
	    profile.depth_exposure_us = 0;
	    profile.rgb_hole_fill_sensitivity = params["rgb_hole_fill_sensitivity"]
      ? std::clamp(params["rgb_hole_fill_sensitivity"].as<int>(), kRgbHoleFillMin, kRgbHoleFillMax)
      : profile.rgb_hole_fill_sensitivity;
    profile.rgb_mask_dilate_px = params["rgb_mask_dilate_px"]
      ? std::clamp(params["rgb_mask_dilate_px"].as<int>(), kRgbDilateMinPx, kRgbDilateMaxPx)
      : profile.rgb_mask_dilate_px;
    profile.depth_null_fill_sensitivity = params["depth_null_fill_sensitivity"]
      ? std::clamp(params["depth_null_fill_sensitivity"].as<int>(), kDepthFillSensitivityMin, kDepthFillSensitivityMax)
      : profile.depth_null_fill_sensitivity;
    profile.depth_window_mm = params["depth_window_mm"]
      ? std::clamp(params["depth_window_mm"].as<int>(), kDepthWindowMinMm, kDepthWindowMaxMm)
      : profile.depth_window_mm;
    profile.depth_hole_fill_sensitivity = params["depth_hole_fill_sensitivity"]
      ? std::clamp(
        params["depth_hole_fill_sensitivity"].as<int>(),
        kDepthFillSensitivityMin,
        kDepthFillSensitivityMax)
      : profile.depth_hole_fill_sensitivity;
    profile.depth_trim_px = params["depth_trim_px"]
      ? std::clamp(params["depth_trim_px"].as<int>(), kDepthTrimMinPx, kDepthTrimMaxPx)
      : profile.depth_trim_px;
    if (params["adaptive_depth_trim_max_add_px"])
    {
      profile.adaptive_depth_trim_max_add_px = parseSavedAdaptiveDepthTrimAddPx(
        params["adaptive_depth_trim_max_add_px"],
        profile.adaptive_depth_trim_max_add_px);
    }
    profile.adaptive_depth_trim_max_height_mm = params["adaptive_depth_trim_max_height_mm"]
      ? clampAdaptiveDepthTrimHeightMm(params["adaptive_depth_trim_max_height_mm"].as<int>())
      : profile.adaptive_depth_trim_max_height_mm;
    profile.ray_step_px = params["ray_step_px"] ? params["ray_step_px"].as<int>() : profile.ray_step_px;
    profile.depth_edge_offset_px = std::clamp(
      params["depth_edge_offset_px"] ? params["depth_edge_offset_px"].as<int>() : 4,
      kDepthEdgeOffsetMinPx,
      kDepthEdgeOffsetMaxPx);
    profile.previous_color_percent = std::clamp(
      params["previous_color_percent"] ? params["previous_color_percent"].as<int>() : kDefaultPreviousColorPercent,
      20,
      100);
    profile.horizontal_ray_count = std::clamp(
      params["horizontal_ray_count"] ? params["horizontal_ray_count"].as<int>() : profile.horizontal_ray_count,
      50,
      100);
    profile.vertical_ray_count = std::clamp(
      params["vertical_ray_count"] ? params["vertical_ray_count"].as<int>() : profile.vertical_ray_count,
      50,
      150);
    profile.outlier_sensitivity = std::clamp(
      params["outlier_sensitivity"] ? params["outlier_sensitivity"].as<int>() : 50,
      1,
      100);
    profile.detect_black_to_white = params["detect_black_to_white"]
      ? params["detect_black_to_white"].as<bool>()
      : profile.detect_black_to_white;
    if (params["focus_black_mask"])
    {
      profile.focus_black_mask = params["focus_black_mask"].as<bool>();
    }
    else
    {
      profile.focus_black_mask = !profile.detect_black_to_white;
    }
    profile.trace_out_to_in = params["trace_out_to_in"] ? params["trace_out_to_in"].as<bool>() : false;
    const auto parsePointList = [](const YAML::Node &node) -> std::vector<cv::Point>
      {
        std::vector<cv::Point> points;
        if (!node || !node.IsSequence())
        {
          return points;
        }

        if (node.size() > 0 && node[0].IsScalar())
        {
          for (std::size_t i = 0; i + 1 < node.size(); i += 2)
          {
            points.emplace_back(node[i].as<int>(), node[i + 1].as<int>());
          }
          return points;
        }

        for (const auto &point_node : node)
        {
          if (!point_node.IsSequence() || point_node.size() < 2)
          {
            continue;
          }
          points.emplace_back(point_node[0].as<int>(), point_node[1].as<int>());
        }
        return points;
      };

    const auto parsePoint2fList = [](const YAML::Node &node) -> std::vector<cv::Point2f>
      {
        std::vector<cv::Point2f> points;
        if (!node || !node.IsSequence())
        {
          return points;
        }

        if (node.size() > 0 && node[0].IsScalar())
        {
          for (std::size_t i = 0; i + 1 < node.size(); i += 2)
          {
            points.emplace_back(node[i].as<float>(), node[i + 1].as<float>());
          }
          return points;
        }

        for (const auto &point_node : node)
        {
          if (!point_node.IsSequence() || point_node.size() < 2)
          {
            continue;
          }
          points.emplace_back(point_node[0].as<float>(), point_node[1].as<float>());
        }
        return points;
      };

    const auto parseNormalizedPointList = [&parsePoint2fList](const YAML::Node &node) -> std::vector<cv::Point2f>
      {
        std::vector<cv::Point2f> points = parsePoint2fList(node);
        if (points.size() != 4)
        {
          return {};
        }
        for (const auto &point : points)
        {
          if (!std::isfinite(point.x) || !std::isfinite(point.y))
          {
            return {};
          }
        }
        return points;
      };

    const auto parseNormalizedBounds = [](const YAML::Node &node) -> std::optional<std::array<double, 4>>
      {
        if (!node || !node.IsSequence() || node.size() < 4)
        {
          return std::nullopt;
        }

        std::array<double, 4> bounds{};
        for (std::size_t i = 0; i < bounds.size(); ++i)
        {
          const double value = node[i].as<double>();
          if (!std::isfinite(value))
          {
            return std::nullopt;
          }
          bounds[i] = std::clamp(value, 0.0, 1.0);
        }
        return bounds;
      };

    const auto loadBinTeachNormalizedRoi = [&path, &parseNormalizedPointList, &parsePoint2fList](
        const std::string &bin_teach_file) -> std::vector<cv::Point2f>
      {
        if (bin_teach_file.empty())
        {
          return {};
        }
        std::filesystem::path bin_path(bin_teach_file);
        if (bin_path.is_relative())
        {
          bin_path = path.parent_path() / bin_path;
        }
        std::error_code fs_error;
        if (!std::filesystem::exists(bin_path, fs_error) || !std::filesystem::is_regular_file(bin_path, fs_error))
        {
          return {};
        }
        try
        {
          const YAML::Node bin_root = YAML::LoadFile(bin_path.string());
          const YAML::Node bin = bin_root["bin_teach"];
          if (!bin || !bin.IsMap())
          {
            return {};
          }
          if (std::vector<cv::Point2f> normalized = parseNormalizedPointList(bin["roi_points_normalized"]);
              !normalized.empty())
          {
            return normalized;
          }
          const YAML::Node image = bin["image"];
          if (!image || !image.IsMap())
          {
            return {};
          }
          const int width = image["width"] ? image["width"].as<int>() : 0;
          const int height = image["height"] ? image["height"].as<int>() : 0;
          if (width <= 1 || height <= 1)
          {
            return {};
          }
          std::vector<cv::Point2f> roi_points = parsePoint2fList(bin["roi_points"]);
          if (roi_points.size() != 4)
          {
            return {};
          }
          for (auto &point : roi_points)
          {
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
            {
              return {};
            }
            point.x = std::clamp(point.x / static_cast<float>(width - 1), 0.0F, 1.0F);
            point.y = std::clamp(point.y / static_cast<float>(height - 1), 0.0F, 1.0F);
          }
          return roi_points;
        }
        catch (const YAML::Exception &)
        {
          return {};
        }
      };

    const auto loadBinTeachNormalizedDepthPlaneRoi = [&path, &parseNormalizedBounds](
        const std::string &bin_teach_file) -> std::optional<std::array<double, 4>>
      {
        if (bin_teach_file.empty())
        {
          return std::nullopt;
        }
        std::filesystem::path bin_path(bin_teach_file);
        if (bin_path.is_relative())
        {
          bin_path = path.parent_path() / bin_path;
        }
        std::error_code fs_error;
        if (!std::filesystem::exists(bin_path, fs_error) || !std::filesystem::is_regular_file(bin_path, fs_error))
        {
          return std::nullopt;
        }
        try
        {
          const YAML::Node bin_root = YAML::LoadFile(bin_path.string());
          const YAML::Node bin = bin_root["bin_teach"];
          if (!bin || !bin.IsMap())
          {
            return std::nullopt;
          }
          if (const auto normalized = parseNormalizedBounds(bin["depth_plane_roi_normalized"]);
            normalized.has_value())
          {
            return normalized;
          }
          const YAML::Node image = bin["image"];
          const YAML::Node roi = bin["depth_plane_roi"];
          if (!image || !image.IsMap() || !roi || !roi.IsSequence() || roi.size() < 4)
          {
            return std::nullopt;
          }
          const int width = image["width"] ? image["width"].as<int>() : 0;
          const int height = image["height"] ? image["height"].as<int>() : 0;
          if (width <= 1 || height <= 1)
          {
            return std::nullopt;
          }
          return std::array<double, 4>{
            std::clamp(static_cast<double>(roi[0].as<int>()) / static_cast<double>(width - 1), 0.0, 1.0),
            std::clamp(static_cast<double>(roi[1].as<int>()) / static_cast<double>(height - 1), 0.0, 1.0),
            std::clamp(static_cast<double>(roi[2].as<int>()) / static_cast<double>(width - 1), 0.0, 1.0),
            std::clamp(static_cast<double>(roi[3].as<int>()) / static_cast<double>(height - 1), 0.0, 1.0),
          };
        }
        catch (const YAML::Exception &)
        {
          return std::nullopt;
        }
      };

    if (params["image_width"])
    {
      profile.roi_image_width = std::max(0, params["image_width"].as<int>());
    }
    if (params["image_height"])
    {
      profile.roi_image_height = std::max(0, params["image_height"].as<int>());
    }
    if (const YAML::Node image = params["image"]; image && image.IsMap())
    {
      if (profile.roi_image_width <= 0 && image["width"])
      {
        profile.roi_image_width = std::max(0, image["width"].as<int>());
      }
      if (profile.roi_image_height <= 0 && image["height"])
      {
        profile.roi_image_height = std::max(0, image["height"].as<int>());
      }
    }
    profile.bin_teach_file = params["bin_teach_file"] ? params["bin_teach_file"].as<std::string>() : "";

    const auto set_taught_dimensions = [&](double length_mm, double width_mm) -> bool
    {
      if (
        !std::isfinite(length_mm) ||
        !std::isfinite(width_mm) ||
        length_mm <= 0.0 ||
        width_mm <= 0.0)
      {
        return false;
      }
      profile.taught_item_length_mm = std::max(length_mm, width_mm);
      profile.taught_item_width_mm = std::min(length_mm, width_mm);
      profile.has_taught_item_dimensions = true;
      return true;
    };

    if (params["item_length_mm"] && params["item_width_mm"])
    {
      set_taught_dimensions(params["item_length_mm"].as<double>(), params["item_width_mm"].as<double>());
    }
    if (
      !profile.has_taught_item_dimensions &&
      params["taught_item_average_length_mm"] &&
      params["taught_item_average_width_mm"])
    {
      set_taught_dimensions(
        params["taught_item_average_length_mm"].as<double>(),
        params["taught_item_average_width_mm"].as<double>());
    }
    if (
      !profile.has_taught_item_dimensions &&
      params["item_dimensions_mm"] &&
      params["item_dimensions_mm"].IsSequence() &&
      params["item_dimensions_mm"].size() >= 2)
    {
      set_taught_dimensions(
        params["item_dimensions_mm"][0].as<double>(),
        params["item_dimensions_mm"][1].as<double>());
    }

    const auto setReferenceFromPolygon = [](
        const std::vector<cv::Point> &polygon_points,
        PoseBlobReference2D *reference_out,
        bool *has_reference_out) -> bool
      {
        if (reference_out == nullptr || has_reference_out == nullptr || polygon_points.size() < 3)
        {
          return false;
        }

        std::vector<cv::Point> hull;
        cv::convexHull(polygon_points, hull, true, true);
        if (hull.size() < 3)
        {
          return false;
        }

        const double rect_area_px = fittedRectAreaPx(hull);
        if (!std::isfinite(rect_area_px) || rect_area_px < 1.0)
        {
          return false;
        }

        reference_out->area_px = std::max(1, static_cast<int>(std::round(rect_area_px)));
        reference_out->aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(hull));
        reference_out->fill_ratio = 1.0;
        reference_out->hull = std::move(hull);
        *has_reference_out = true;
        return true;
      };

    const auto loadPoseReferenceFromNode =
      [&](const YAML::Node &node) -> std::optional<PoseBlobReference2D>
      {
        if (!node || !node.IsMap())
        {
          return std::nullopt;
        }

        PoseBlobReference2D reference;
        bool has_reference = false;
        const PoseTemplateMode2D pose_template_mode = node["pose_template_mode"]
          ? parsePoseTemplateMode(node["pose_template_mode"].as<std::string>())
          : PoseTemplateMode2D::kSingle;
        if (
          pose_template_mode == PoseTemplateMode2D::kPair &&
          node["pose_group_reference_hull_points"] &&
          node["pose_group_reference_hull_points"].IsSequence())
        {
          has_reference = setReferenceFromPolygon(
            parsePointList(node["pose_group_reference_hull_points"]),
            &reference,
            &has_reference);
        }
        if (!has_reference)
        {
          if (const YAML::Node ref_hull = node["pose_blob_reference_hull_points"];
            ref_hull && ref_hull.IsSequence())
          {
            has_reference = setReferenceFromPolygon(
              parsePointList(ref_hull),
              &reference,
              &has_reference);
          }
        }
        if (!has_reference && node["pose_blobs"] && node["pose_blobs"].IsSequence())
        {
          for (const auto &blob_node : node["pose_blobs"])
          {
            if (!blob_node.IsMap())
            {
              continue;
            }
            const std::vector<cv::Point> quad_points = parsePointList(blob_node["quad_points"]);
            if (setReferenceFromPolygon(quad_points, &reference, &has_reference))
            {
              break;
            }
          }
        }
        if (!has_reference && node["pose_quad_points"] && node["pose_quad_points"].IsSequence())
        {
          has_reference = setReferenceFromPolygon(
            parsePointList(node["pose_quad_points"]),
            &reference,
            &has_reference);
        }
        if (!has_reference)
        {
          return std::nullopt;
        }

        reference.mode = pose_template_mode;
        reference.fill_ratio = loadReferenceBlobFillRatio(node);
        if (const auto auxiliary_fill_ratio = loadAuxiliaryBlobFillRatio(node);
          auxiliary_fill_ratio.has_value())
        {
          reference.companion_fill_ratio = *auxiliary_fill_ratio;
        }
        else if (pose_template_mode == PoseTemplateMode2D::kPair)
        {
          reference.companion_fill_ratio = reference.fill_ratio;
        }
        const std::vector<cv::Point> generic_loaded_hull = reference.hull;
        const int generic_loaded_area_px = reference.area_px;
        const float generic_loaded_aspect_ratio = reference.aspect_ratio;
        if (const YAML::Node reference_blob_hull = node["reference_blob_hull_points"];
          reference_blob_hull && reference_blob_hull.IsSequence())
        {
          reference.anchor_hull = parsePointList(reference_blob_hull);
        }
        if (const YAML::Node auxiliary_blob_hull = node["auxiliary_blob_hull_points"];
          auxiliary_blob_hull && auxiliary_blob_hull.IsSequence())
        {
          reference.companion_hull = parsePointList(auxiliary_blob_hull);
        }
        if (const YAML::Node auxiliary_area = node["auxiliary_blob_reference_area_px"]
            ? node["auxiliary_blob_reference_area_px"]
            : node["auxiliary_blob_area_px"];
          auxiliary_area)
        {
          reference.companion_area_px = std::max(1, auxiliary_area.as<int>());
        }
        if (const YAML::Node auxiliary_aspect = node["auxiliary_blob_reference_aspect_ratio"]
            ? node["auxiliary_blob_reference_aspect_ratio"]
            : node["auxiliary_blob_aspect_ratio"];
          auxiliary_aspect)
        {
          reference.companion_aspect_ratio =
            std::max(1e-3F, static_cast<float>(auxiliary_aspect.as<double>()));
        }
        if (pose_template_mode == PoseTemplateMode2D::kPair)
        {
          if (const YAML::Node group_hull = node["pose_group_reference_hull_points"];
            group_hull && group_hull.IsSequence())
          {
            reference.group_hull = parsePointList(group_hull);
          }
          else
          {
            reference.group_hull = generic_loaded_hull;
          }
          if (!reference.group_hull.empty())
          {
            reference.group_area_px = std::max(
              1,
              static_cast<int>(std::round(fittedRectAreaPx(reference.group_hull))));
            reference.group_aspect_ratio = std::max(
              1e-3F,
              fittedRectAspectRatio(reference.group_hull));
          }

          if (!reference.anchor_hull.empty())
          {
            reference.hull = reference.anchor_hull;
            reference.area_px = std::max(
              1,
              static_cast<int>(std::round(fittedRectAreaPx(reference.hull))));
            reference.aspect_ratio = std::max(
              1e-3F,
              fittedRectAspectRatio(reference.hull));
          }
          else
          {
            reference.area_px = generic_loaded_area_px;
            reference.aspect_ratio = generic_loaded_aspect_ratio;
          }
          if (!reference.companion_hull.empty())
          {
            if (reference.companion_area_px <= 0)
            {
              reference.companion_area_px = std::max(
                1,
                static_cast<int>(std::round(fittedRectAreaPx(reference.companion_hull))));
            }
            reference.companion_aspect_ratio = std::max(
              1e-3F,
              reference.companion_aspect_ratio > 1e-3F
              ? reference.companion_aspect_ratio
              : fittedRectAspectRatio(reference.companion_hull));
          }

          reference.member_count = node["pose_group_member_count"]
            ? node["pose_group_member_count"].as<int>()
            : 2;
          reference.member_centers_norm = parsePoint2fList(
            node["pose_group_reference_member_centers_norm"]);
          const auto anchor_points = parsePoint2fList(node["pose_group_reference_anchor_center_norm"]);
          if (!anchor_points.empty())
          {
            reference.anchor_center_norm = anchor_points.front();
          }
          else if (!reference.member_centers_norm.empty())
          {
            reference.anchor_center_norm = reference.member_centers_norm.front();
          }
          if (reference.member_count != 2 || reference.member_centers_norm.size() < 2)
          {
            return std::nullopt;
          }
        }
        return reference;
      };

    if (const auto root_reference = loadPoseReferenceFromNode(params);
      root_reference.has_value())
    {
      profile.pose_blob_reference = *root_reference;
      profile.has_pose_blob_reference = true;
    }

    if (const YAML::Node pose_slots = params["pose_reference_slots"];
      pose_slots && pose_slots.IsSequence())
    {
      for (const auto &slot_node : pose_slots)
      {
        if (!slot_node.IsMap())
        {
          continue;
        }
        const bool enabled = slot_node["enabled"] ? slot_node["enabled"].as<bool>() : true;
        if (!enabled)
        {
          continue;
        }
        const int slot_index = std::clamp(
          (slot_node["slot_index"] ? slot_node["slot_index"].as<int>() : 1) - 1,
          0,
          kPoseReferenceSlotCount - 1);
        if (const auto slot_reference = loadPoseReferenceFromNode(slot_node);
          slot_reference.has_value())
        {
          profile.pose_reference_slots.push_back(PoseReferenceSlot2D{slot_index, *slot_reference});
        }
      }
      std::sort(
        profile.pose_reference_slots.begin(),
        profile.pose_reference_slots.end(),
        [](const PoseReferenceSlot2D &a, const PoseReferenceSlot2D &b)
        {
          return a.slot_index < b.slot_index;
        });
      profile.pose_reference_slots.erase(
        std::unique(
          profile.pose_reference_slots.begin(),
          profile.pose_reference_slots.end(),
          [](const PoseReferenceSlot2D &a, const PoseReferenceSlot2D &b)
          {
            return a.slot_index == b.slot_index;
          }),
        profile.pose_reference_slots.end());
      if (!profile.pose_reference_slots.empty())
      {
        profile.pose_blob_reference = profile.pose_reference_slots.front().reference;
        profile.has_pose_blob_reference = true;
      }
    }
    if (profile.pose_reference_slots.empty() && profile.has_pose_blob_reference)
    {
      profile.pose_reference_slots.push_back(PoseReferenceSlot2D{0, profile.pose_blob_reference});
    }

    profile.depth_plane_enabled = params["depth_plane_enabled"] ? params["depth_plane_enabled"].as<bool>() : false;
    profile.depth_plane_a = params["depth_plane_a"] ? params["depth_plane_a"].as<double>() : 0.0;
    profile.depth_plane_b = params["depth_plane_b"] ? params["depth_plane_b"].as<double>() : 0.0;
    profile.depth_plane_c = params["depth_plane_c"] ? params["depth_plane_c"].as<double>() : 0.0;
    profile.depth_plane_reference_depth_m =
      params["depth_plane_reference_depth_m"] ? params["depth_plane_reference_depth_m"].as<double>() : 0.0;
    if (const YAML::Node depth_plane_roi = params["depth_plane_roi"];
        depth_plane_roi && depth_plane_roi.IsSequence() && depth_plane_roi.size() >= 4)
    {
      AxisAlignedRoiBounds roi{
        depth_plane_roi[0].as<int>(),
        depth_plane_roi[1].as<int>(),
        depth_plane_roi[2].as<int>(),
        depth_plane_roi[3].as<int>(),
      };
      if (isValidRoiBounds(roi))
      {
        profile.depth_plane_roi_bounds = roi;
      }
    }
    profile.depth_plane_roi_normalized = parseNormalizedBounds(params["depth_plane_roi_normalized"]);
    if (!profile.depth_plane_roi_normalized.has_value())
    {
      profile.depth_plane_roi_normalized = loadBinTeachNormalizedDepthPlaneRoi(profile.bin_teach_file);
    }
    if (!profile.depth_plane_roi_bounds.has_value() && profile.depth_plane_roi_normalized.has_value())
    {
      profile.depth_plane_roi_bounds = denormalizeRoiBounds(
        *profile.depth_plane_roi_normalized,
        cv::Size(profile.roi_image_width, profile.roi_image_height));
    }
    if (
      !std::isfinite(profile.depth_plane_a) ||
      !std::isfinite(profile.depth_plane_b) ||
      !std::isfinite(profile.depth_plane_c) ||
      !std::isfinite(profile.depth_plane_reference_depth_m) ||
      profile.depth_plane_reference_depth_m <= 0.0 ||
      (!profile.depth_plane_roi_bounds.has_value() && !profile.depth_plane_roi_normalized.has_value()))
    {
      profile.depth_plane_enabled = false;
      profile.depth_plane_a = 0.0;
      profile.depth_plane_b = 0.0;
      profile.depth_plane_c = 0.0;
      profile.depth_plane_reference_depth_m = 0.0;
    }
    if (const YAML::Node roi_regions = params["roi_regions"]; roi_regions && roi_regions.IsSequence())
    {
      for (const auto &region_node : roi_regions)
      {
        if (!region_node.IsSequence() || region_node.size() < 4)
        {
          continue;
        }

        AxisAlignedRoiBounds region{
          region_node[0].as<int>(),
          region_node[1].as<int>(),
          region_node[2].as<int>(),
          region_node[3].as<int>(),
        };
        if (isValidRoiBounds(region))
        {
          profile.roi_regions.push_back(region);
        }
      }
    }
    if (const YAML::Node roi_points = params["roi_points"]; roi_points && roi_points.IsSequence())
    {
      if (roi_points.size() > 0 && roi_points[0].IsScalar())
      {
        for (std::size_t i = 0; i + 1 < roi_points.size(); i += 2)
        {
          profile.roi_points.emplace_back(
            roi_points[i].as<float>(),
            roi_points[i + 1].as<float>());
        }
      }
      else
      {
        for (const auto &point_node : roi_points)
        {
          if (!point_node.IsSequence() || point_node.size() < 2)
          {
            continue;
          }
          profile.roi_points.emplace_back(
            point_node[0].as<float>(),
            point_node[1].as<float>());
        }
      }
    }
    profile.roi_points_normalized = parseNormalizedPointList(params["roi_points_normalized"]);
    profile.item_name = params["item_name"]
      ? params["item_name"].as<std::string>()
      : path.stem().string();
    profile.associated_bin_name = params["associated_bin_name"]
      ? params["associated_bin_name"].as<std::string>()
      : "";
    if (profile.roi_points_normalized.empty())
    {
      profile.roi_points_normalized = loadBinTeachNormalizedRoi(profile.bin_teach_file);
    }
    profile.teach_date = params["teach_date"] ? params["teach_date"].as<std::string>() : "";
    if (const YAML::Node teach_joints = params["teach_joints_deg"];
        teach_joints && teach_joints.IsSequence())
    {
      for (std::size_t i = 0; i < profile.teach_joints_deg.size() && i < teach_joints.size(); ++i)
      {
        profile.teach_joints_deg[i] = teach_joints[i].as<double>();
      }
      profile.has_teach_joints = teach_joints.size() >= profile.teach_joints_deg.size();
    }
    if (profile.roi_points.size() < 4 && !profile.roi_regions.empty())
    {
      profile.roi_points = mergeRoiRegionsIntoPolygon(profile.roi_regions);
    }
    else if (
      profile.roi_regions.empty() &&
      profile.roi_points.size() == 2)
    {
      if (const auto selected_bounds = roiBoundsFromSelection(profile.roi_points); selected_bounds.has_value())
      {
        profile.roi_points = roiPointsFromBounds(*selected_bounds);
      }
    }
    if (profile.roi_points.size() < 4)
    {
      profile.roi_points.clear();
    }
    profile.display_label = buildProfileLabel(
      profile.item_name,
      profile.associated_bin_name,
      profile.teach_date,
      path);
    if (profile.has_tool_teach)
    {
      profile.display_label += " | tool";
    }
    return profile;
  }
  catch (const YAML::Exception &)
  {
    return std::nullopt;
  }
}

std::vector<ItemProfile> loadItemProfilesFromDirectory(const std::filesystem::path &profiles_dir)
{
  std::vector<ItemProfile> profiles;
  if (profiles_dir.empty() || !std::filesystem::exists(profiles_dir))
  {
    return profiles;
  }

  for (const auto &entry : std::filesystem::directory_iterator(profiles_dir))
  {
    if (!entry.is_regular_file())
    {
      continue;
    }

    if (
      entry.path().filename() == "item_teach_settings.yaml" ||
      entry.path().filename() == "bin_teach_settings.yaml")
    {
      continue;
    }

    const std::string extension = entry.path().extension().string();
    if (extension != ".yaml" && extension != ".yml")
    {
      continue;
    }

    const auto profile = loadItemProfileFile(entry.path());
    if (!profile.has_value())
    {
      continue;
    }

    profiles.push_back(*profile);
  }

  auto by_recent_date = [](const ItemProfile &a, const ItemProfile &b)
  {
    if (a.teach_date != b.teach_date)
    {
      return a.teach_date > b.teach_date;
    }
    return a.path.filename().string() < b.path.filename().string();
  };

  std::sort(profiles.begin(), profiles.end(), by_recent_date);
  return profiles;
}

std::string fitTextToWidth(
  const std::string &text,
  int max_width,
  double scale = 0.62,
  int thickness = 2)
{
  if (max_width <= 0 || text.empty())
  {
    return "";
  }

  if (cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, nullptr).width <= max_width)
  {
    return text;
  }

  static constexpr char kEllipsis[] = "...";
  for (std::size_t keep = text.size(); keep > 0; --keep)
  {
    const std::string candidate = text.substr(0, keep) + kEllipsis;
    if (cv::getTextSize(candidate, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, nullptr).width <= max_width)
    {
      return candidate;
    }
  }

  return kEllipsis;
}

cv::Mat fillEnclosedMaskHoles(const cv::Mat &mask, int hole_fill_sensitivity)
{
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    return {};
  }

  const int clamped_sensitivity = std::clamp(hole_fill_sensitivity, kRgbHoleFillMin, kRgbHoleFillMax);
  if (clamped_sensitivity <= 0)
  {
    return mask.clone();
  }

  cv::Mat padded_mask;
  cv::copyMakeBorder(mask, padded_mask, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::Mat flood_filled = padded_mask.clone();
  cv::floodFill(flood_filled, cv::Point(0, 0), cv::Scalar(255));

  cv::Mat enclosed_holes;
  cv::bitwise_not(flood_filled, enclosed_holes);
  enclosed_holes = enclosed_holes(cv::Rect(1, 1, mask.cols, mask.rows)).clone();

  if (cv::countNonZero(enclosed_holes) == 0)
  {
    return mask.clone();
  }

  cv::Mat holes_to_fill = cv::Mat::zeros(mask.size(), CV_8UC1);
  if (clamped_sensitivity >= kRgbHoleFillMax)
  {
    holes_to_fill = enclosed_holes;
  }
  else
  {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
      enclosed_holes, labels, stats, centroids, 8, CV_32S);
    if (component_count <= 1)
    {
      return mask.clone();
    }

    const double t = static_cast<double>(clamped_sensitivity) / static_cast<double>(kRgbHoleFillMax);
    const double max_fraction = 0.15 * t * t;
    const int max_hole_area_px = std::max(
      1,
      static_cast<int>(std::round(max_fraction * static_cast<double>(mask.rows * mask.cols))));
    std::vector<unsigned char> keep_component(static_cast<std::size_t>(component_count), 0);
    for (int label = 1; label < component_count; ++label)
    {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (area <= max_hole_area_px)
      {
        keep_component[static_cast<std::size_t>(label)] = 1;
      }
    }

    for (int y = 0; y < labels.rows; ++y)
    {
      const int *label_row = labels.ptr<int>(y);
      unsigned char *fill_row = holes_to_fill.ptr<unsigned char>(y);
      for (int x = 0; x < labels.cols; ++x)
      {
        const int label = label_row[x];
        if (label > 0 && keep_component[static_cast<std::size_t>(label)] != 0)
        {
          fill_row[x] = 255;
        }
      }
    }
  }

  cv::Mat hole_filled_mask;
  cv::bitwise_or(mask, holes_to_fill, hole_filled_mask);
  return hole_filled_mask;
}

cv::Mat buildRgbMask(
  const cv::Mat &bgr,
  int red_threshold,
  int green_threshold,
  int blue_threshold,
  int hole_fill_sensitivity,
  int rgb_dilate_px,
  bool focus_black_mask)
{
  cv::Mat mask;
  cv::inRange(
    bgr,
    cv::Scalar(blue_threshold, green_threshold, red_threshold),
    cv::Scalar(255, 255, 255),
    mask);
  if (focus_black_mask)
  {
    cv::bitwise_not(mask, mask);
  }
  cv::Mat hole_filled_mask = fillEnclosedMaskHoles(mask, hole_fill_sensitivity);
  if (cv::countNonZero(hole_filled_mask) == 0)
  {
    return hole_filled_mask;
  }

  const int clamped_dilate_px = std::clamp(rgb_dilate_px, kRgbDilateMinPx, kRgbDilateMaxPx);
  const int kernel_size = clamped_dilate_px * 2 + 1;
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(kernel_size, kernel_size));
  cv::Mat dilated_mask;
  cv::dilate(hole_filled_mask, dilated_mask, kernel);
  return dilated_mask;
}

cv::Mat buildFiniteDepthMask(const cv::Mat &depth_m)
{
  cv::Mat mask;
  if (depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return mask;
  }

  mask = cv::Mat::zeros(depth_m.size(), CV_8UC1);
  for (int y = 0; y < depth_m.rows; ++y)
  {
    const float *depth_row = depth_m.ptr<float>(y);
    unsigned char *mask_row = mask.ptr<unsigned char>(y);
    for (int x = 0; x < depth_m.cols; ++x)
    {
      if (std::isfinite(depth_row[x]))
      {
        mask_row[x] = 255;
      }
    }
  }
  return mask;
}

cv::Mat buildPositiveFiniteDepthMask(const cv::Mat &depth_m)
{
  cv::Mat mask;
  if (depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return mask;
  }

  mask = cv::Mat::zeros(depth_m.size(), CV_8UC1);
  for (int y = 0; y < depth_m.rows; ++y)
  {
    const float *depth_row = depth_m.ptr<float>(y);
    unsigned char *mask_row = mask.ptr<unsigned char>(y);
    for (int x = 0; x < depth_m.cols; ++x)
    {
      const float depth = depth_row[x];
      if (std::isfinite(depth) && depth > 0.0F)
      {
        mask_row[x] = 255;
      }
    }
  }
  return mask;
}

cv::Mat buildFiniteDepthResidualMask(const cv::Mat &depth_residual_m)
{
  cv::Mat mask;
  if (depth_residual_m.empty() || depth_residual_m.type() != CV_32FC1)
  {
    return mask;
  }

  mask = cv::Mat::zeros(depth_residual_m.size(), CV_8UC1);
  for (int y = 0; y < depth_residual_m.rows; ++y)
  {
    const float *depth_row = depth_residual_m.ptr<float>(y);
    unsigned char *mask_row = mask.ptr<unsigned char>(y);
    for (int x = 0; x < depth_residual_m.cols; ++x)
    {
      const float residual = depth_row[x];
      if (std::isfinite(residual))
      {
        mask_row[x] = 255;
      }
    }
  }
  return mask;
}

struct DepthWindowPeakInfo
{
  bool valid {false};
  cv::Point pixel {-1, -1};
  float peak_height_m {0.0F};
  float min_keep_height_m {0.0F};
};

cv::Mat applyDepthTopWindowMask(
  const cv::Mat &depth_residual_m,
  const cv::Mat &candidate_mask,
  int depth_window_mm,
  DepthWindowPeakInfo *peak_info = nullptr)
{
  if (peak_info != nullptr)
  {
    *peak_info = DepthWindowPeakInfo{};
  }
  if (
    depth_residual_m.empty() ||
    depth_residual_m.type() != CV_32FC1 ||
    candidate_mask.empty() ||
    candidate_mask.type() != CV_8UC1 ||
    depth_residual_m.size() != candidate_mask.size())
  {
    return {};
  }

  float peak_height_m = -std::numeric_limits<float>::infinity();
  cv::Point peak_pixel(-1, -1);
  for (int y = 0; y < depth_residual_m.rows; ++y)
  {
    const float *depth_row = depth_residual_m.ptr<float>(y);
    const unsigned char *mask_row = candidate_mask.ptr<unsigned char>(y);
    for (int x = 0; x < depth_residual_m.cols; ++x)
    {
      if (mask_row[x] == 0)
      {
        continue;
      }
      const float residual = depth_row[x];
      if (!std::isfinite(residual))
      {
        continue;
      }
      const float height_m = -residual;
      if (height_m > peak_height_m)
      {
        peak_height_m = height_m;
        peak_pixel = cv::Point(x, y);
      }
    }
  }

  if (!std::isfinite(peak_height_m))
  {
    return cv::Mat::zeros(candidate_mask.size(), CV_8UC1);
  }

  const float depth_window_m = static_cast<float>(
    std::clamp(depth_window_mm, kDepthWindowMinMm, kDepthWindowMaxMm)) / 1000.0F;
  const float min_keep_height_m = peak_height_m - depth_window_m;
  if (peak_info != nullptr)
  {
    peak_info->valid = true;
    peak_info->pixel = peak_pixel;
    peak_info->peak_height_m = peak_height_m;
    peak_info->min_keep_height_m = min_keep_height_m;
  }

  cv::Mat output(candidate_mask.size(), CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < depth_residual_m.rows; ++y)
  {
    const float *depth_row = depth_residual_m.ptr<float>(y);
    const unsigned char *candidate_row = candidate_mask.ptr<unsigned char>(y);
    unsigned char *out_row = output.ptr<unsigned char>(y);
    for (int x = 0; x < depth_residual_m.cols; ++x)
    {
      if (candidate_row[x] == 0)
      {
        continue;
      }
      const float residual = depth_row[x];
      if (!std::isfinite(residual))
      {
        continue;
      }
      if (-residual >= min_keep_height_m)
      {
        out_row[x] = 255;
      }
    }
  }

  return output;
}

cv::Mat trimMaskInward(const cv::Mat &mask, int trim_px)
{
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    return {};
  }
  const int clamped_trim_px = std::clamp(trim_px, kDepthTrimMinPx, kDepthTrimMaxPx);
  if (clamped_trim_px <= 0)
  {
    return mask.clone();
  }

  cv::Mat binary_mask;
  cv::threshold(mask, binary_mask, 0, 255, cv::THRESH_BINARY);
  if (cv::countNonZero(binary_mask) == 0)
  {
    return binary_mask;
  }

  cv::Mat padded_mask;
  cv::copyMakeBorder(binary_mask, padded_mask, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::Mat distance_to_edge_px;
  cv::distanceTransform(padded_mask, distance_to_edge_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  distance_to_edge_px = distance_to_edge_px(cv::Rect(1, 1, mask.cols, mask.rows)).clone();

  cv::Mat trimmed_mask;
  cv::compare(distance_to_edge_px, static_cast<double>(clamped_trim_px), trimmed_mask, cv::CMP_GT);
  cv::bitwise_and(trimmed_mask, binary_mask, trimmed_mask);
  return trimmed_mask;
}

int computeAdaptiveDepthTrimPx(
  int base_trim_px,
  const DepthWindowPeakInfo &peak_info,
  int max_add_px,
  int max_height_mm)
{
  const int clamped_base = std::clamp(base_trim_px, kDepthTrimMinPx, kDepthTrimMaxPx);
  if (clamped_base <= 0 || !peak_info.valid)
  {
    return clamped_base;
  }

  const int clamped_add_px = clampAdaptiveDepthTrimAddPx(max_add_px);
  const int clamped_max_height_mm = clampAdaptiveDepthTrimHeightMm(max_height_mm);
  const float peak_height_mm = std::max(0.0F, peak_info.peak_height_m * 1000.0F);
  const float extra_ratio = std::clamp(
    peak_height_mm / static_cast<float>(clamped_max_height_mm),
    0.0F,
    1.0F);
  const int adaptive_trim =
    clamped_base + static_cast<int>(std::round(static_cast<float>(clamped_add_px) * extra_ratio));
  return std::clamp(adaptive_trim, kDepthTrimMinPx, kDepthTrimMaxPx);
}

cv::Mat fillInvalidDepthNearby(const cv::Mat &depth_values_m, int fill_sensitivity)
{
  if (depth_values_m.empty() || depth_values_m.type() != CV_32FC1)
  {
    return depth_values_m.clone();
  }

  const int clamped_sensitivity = std::clamp(fill_sensitivity, kDepthFillSensitivityMin, kDepthFillSensitivityMax);
  if (clamped_sensitivity <= 0)
  {
    return depth_values_m.clone();
  }

  cv::Mat filled_depth = depth_values_m.clone();
  cv::Mat valid_mask = buildPositiveFiniteDepthMask(filled_depth);
  if (valid_mask.empty() || cv::countNonZero(valid_mask) == 0)
  {
    return filled_depth;
  }

  const int max_iterations = std::clamp(
    static_cast<int>(std::round(1.0 + (23.0 * static_cast<double>(clamped_sensitivity) / 100.0))),
    1,
    24);
  const std::array<cv::Point, 8> kNeighborOffsets = {
    cv::Point(-1, -1), cv::Point(0, -1), cv::Point(1, -1),
    cv::Point(-1, 0),                    cv::Point(1, 0),
    cv::Point(-1, 1),  cv::Point(0, 1),  cv::Point(1, 1)};

  for (int iter = 0; iter < max_iterations; ++iter)
  {
    cv::Mat next_depth = filled_depth.clone();
    cv::Mat next_valid = valid_mask.clone();
    bool changed = false;

    for (int y = 0; y < filled_depth.rows; ++y)
    {
      const unsigned char *valid_row = valid_mask.ptr<unsigned char>(y);
      for (int x = 0; x < filled_depth.cols; ++x)
      {
        if (valid_row[x] != 0)
        {
          continue;
        }

        double sum_depth = 0.0;
        int sample_count = 0;
        for (const auto &offset : kNeighborOffsets)
        {
          const int nx = x + offset.x;
          const int ny = y + offset.y;
          if (nx < 0 || ny < 0 || nx >= filled_depth.cols || ny >= filled_depth.rows)
          {
            continue;
          }
          if (valid_mask.at<unsigned char>(ny, nx) == 0)
          {
            continue;
          }
          const float neighbor_depth = filled_depth.at<float>(ny, nx);
          if (!std::isfinite(neighbor_depth) || neighbor_depth <= 0.0F)
          {
            continue;
          }
          sum_depth += static_cast<double>(neighbor_depth);
          ++sample_count;
        }

        if (sample_count <= 0)
        {
          continue;
        }

        next_depth.at<float>(y, x) = static_cast<float>(sum_depth / static_cast<double>(sample_count));
        next_valid.at<unsigned char>(y, x) = 255;
        changed = true;
      }
    }

    filled_depth = std::move(next_depth);
    valid_mask = std::move(next_valid);
    if (!changed)
    {
      break;
    }
  }

  return filled_depth;
}

cv::Mat buildDepthMask(const cv::Mat &depth_m, int depth_threshold_mm, double reference_depth_m)
{
  cv::Mat mask;
  if (depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return mask;
  }
  if (!std::isfinite(reference_depth_m) || reference_depth_m <= 0.0)
  {
    return cv::Mat::zeros(depth_m.size(), CV_8UC1);
  }

  const float half_band_m = static_cast<float>(
    std::clamp(depth_threshold_mm, kDepthThresholdMinMm, kDepthThresholdMaxMm)) / 1000.0F;
  cv::Mat valid_depth_mask;
  cv::Mat within_band_mask;
  cv::Mat depth_abs_diff;
  cv::compare(depth_m, 0.0F, valid_depth_mask, cv::CMP_GT);
  cv::absdiff(depth_m, cv::Scalar::all(reference_depth_m), depth_abs_diff);
  cv::compare(depth_abs_diff, half_band_m, within_band_mask, cv::CMP_LE);
  cv::bitwise_and(valid_depth_mask, within_band_mask, mask);
  return mask;
}

cv::Mat colorizeDepth(const cv::Mat &depth_m)
{
  cv::Mat valid_mask = depth_m > 0.0f;
  if (cv::countNonZero(valid_mask) == 0)
  {
    return cv::Mat(depth_m.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  }

  double min_val = 0.0;
  double max_val = 0.0;
  cv::minMaxLoc(depth_m, &min_val, &max_val, nullptr, nullptr, valid_mask);
  if (max_val <= min_val + 1e-6)
  {
    return cv::Mat(depth_m.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  }

  cv::Mat normalized;
  depth_m.convertTo(
    normalized,
    CV_8UC1,
    255.0 / (max_val - min_val),
    -min_val * 255.0 / (max_val - min_val));
  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
  colored.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
  return colored;
}

double normalizedImageCoord(int value, int max_value)
{
  if (max_value <= 1)
  {
    return 0.0;
  }
  return static_cast<double>(value) / static_cast<double>(max_value - 1);
}

std::vector<cv::Point2f> denormalizeRoiPoints(
  const std::vector<cv::Point2f> &normalized_points,
  const cv::Size &image_size)
{
  if (normalized_points.size() < 4 || image_size.width <= 1 || image_size.height <= 1)
  {
    return {};
  }

  std::vector<cv::Point2f> points;
  points.reserve(normalized_points.size());
  const float max_x = static_cast<float>(image_size.width - 1);
  const float max_y = static_cast<float>(image_size.height - 1);
  for (const auto &point : normalized_points)
  {
    points.emplace_back(
      std::clamp(point.x, 0.0F, 1.0F) * max_x,
      std::clamp(point.y, 0.0F, 1.0F) * max_y);
  }
  return points;
}

std::optional<AxisAlignedRoiBounds> denormalizeRoiBounds(
  const std::array<double, 4> &normalized_bounds,
  const cv::Size &image_size)
{
  if (image_size.width <= 1 || image_size.height <= 1)
  {
    return std::nullopt;
  }

  AxisAlignedRoiBounds bounds{
    static_cast<int>(std::lround(std::clamp(normalized_bounds[0], 0.0, 1.0) * static_cast<double>(image_size.width - 1))),
    static_cast<int>(std::lround(std::clamp(normalized_bounds[1], 0.0, 1.0) * static_cast<double>(image_size.height - 1))),
    static_cast<int>(std::lround(std::clamp(normalized_bounds[2], 0.0, 1.0) * static_cast<double>(image_size.width - 1))),
    static_cast<int>(std::lround(std::clamp(normalized_bounds[3], 0.0, 1.0) * static_cast<double>(image_size.height - 1))),
  };
  if (bounds.left > bounds.right)
  {
    std::swap(bounds.left, bounds.right);
  }
  if (bounds.top > bounds.bottom)
  {
    std::swap(bounds.top, bounds.bottom);
  }
  if (!isValidRoiBounds(bounds))
  {
    return std::nullopt;
  }
  return bounds;
}

cv::Mat applyFixedDepthPlaneNormalization(const cv::Mat &depth_m, const DepthPlaneModel &plane)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1 || !plane.valid)
  {
    return depth_m.clone();
  }

  cv::Mat normalized = depth_m.clone();
  for (int y = 0; y < normalized.rows; ++y)
  {
    float *row_ptr = normalized.ptr<float>(y);
    const double y_norm = normalizedImageCoord(y, normalized.rows);
    for (int x = 0; x < normalized.cols; ++x)
    {
      const float raw_depth = row_ptr[x];
      if (!std::isfinite(raw_depth) || raw_depth <= 0.0F)
      {
        row_ptr[x] = 0.0F;
        continue;
      }

      const double x_norm = normalizedImageCoord(x, normalized.cols);
      const double plane_depth = (plane.a * x_norm) + (plane.b * y_norm) + plane.c;
      const double correction = plane_depth - plane.reference_depth_m;
      const double corrected_depth = static_cast<double>(raw_depth) - correction;
      row_ptr[x] = (std::isfinite(corrected_depth) && corrected_depth > 0.0)
        ? static_cast<float>(corrected_depth)
        : 0.0F;
    }
  }
  return normalized;
}

cv::Mat computeDepthPlaneResidual(const cv::Mat &depth_m, const DepthPlaneModel &plane)
{
  const float kNullDepth = std::numeric_limits<float>::quiet_NaN();
  if (depth_m.empty() || depth_m.type() != CV_32FC1 || !plane.valid)
  {
    return cv::Mat(depth_m.size(), CV_32FC1, cv::Scalar(kNullDepth));
  }

  cv::Mat residual(depth_m.size(), CV_32FC1, cv::Scalar(kNullDepth));
  for (int y = 0; y < depth_m.rows; ++y)
  {
    const float *depth_row = depth_m.ptr<float>(y);
    float *residual_row = residual.ptr<float>(y);
    const double y_norm = normalizedImageCoord(y, depth_m.rows);
    for (int x = 0; x < depth_m.cols; ++x)
    {
      const float raw_depth = depth_row[x];
      if (!std::isfinite(raw_depth) || raw_depth <= 0.0F)
      {
        residual_row[x] = kNullDepth;
        continue;
      }

      const double x_norm = normalizedImageCoord(x, depth_m.cols);
      const double plane_depth = (plane.a * x_norm) + (plane.b * y_norm) + plane.c;
      const double residual_depth = static_cast<double>(raw_depth) - plane_depth;
      residual_row[x] = std::isfinite(residual_depth) ? static_cast<float>(residual_depth) : kNullDepth;
    }
  }
  return residual;
}

void normalizeDepth32FToMeters(cv::Mat &depth_m)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return;
  }

  std::vector<float> sampled_depths;
  sampled_depths.reserve(512);
  const int row_step = std::max(1, depth_m.rows / 32);
  const int col_step = std::max(1, depth_m.cols / 32);
  for (int y = 0; y < depth_m.rows; y += row_step)
  {
    for (int x = 0; x < depth_m.cols; x += col_step)
    {
      const float depth = depth_m.at<float>(y, x);
      if (!std::isfinite(depth) || depth <= 0.0F)
      {
        continue;
      }
      sampled_depths.push_back(depth);
    }
  }

  if (sampled_depths.empty())
  {
    return;
  }

  const auto middle = sampled_depths.begin() + static_cast<long>(sampled_depths.size() / 2);
  std::nth_element(sampled_depths.begin(), middle, sampled_depths.end());
  const float median_depth = *middle;
  if (median_depth > 20.0F)
  {
    depth_m *= 0.001F;
  }
}

bool convertDepthToMeters(const ImageMsg::ConstSharedPtr &depth_msg, cv::Mat &depth_m)
{
  try
  {
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth_msg->encoding == sensor_msgs::image_encodings::MONO16)
    {
      const auto depth_cv = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_cv->image.convertTo(depth_m, CV_32FC1, 0.001);
      return true;
    }
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      depth_m = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone();
      normalizeDepth32FToMeters(depth_m);
      return true;
    }
  }
  catch (const cv_bridge::Exception &)
  {
    return false;
  }
  return false;
}

double pointToLineDistance(
  const cv::Point2f &point,
  const cv::Point2f &line_start,
  const cv::Point2f &line_end)
{
  const cv::Point2f line = line_end - line_start;
  const double norm = std::sqrt(static_cast<double>(line.dot(line)));
  if (norm < 1e-6)
  {
    return std::numeric_limits<double>::infinity();
  }
  const cv::Point2f relative = point - line_start;
  const double cross = std::fabs(static_cast<double>(relative.x * line.y - relative.y * line.x));
  return cross / norm;
}

LineModel fitLineToPoints(const std::vector<cv::Point2f> &points, const cv::Point2f &fallback_a, const cv::Point2f &fallback_b);

SideFitResult fitSideLineWithTrimming(
  const std::vector<cv::Point2f> &points,
  const cv::Point2f &fallback_a,
  const cv::Point2f &fallback_b,
  int outlier_sensitivity)
{
  const double remapped_sensitivity = remapOutlierSensitivityToFitRange(outlier_sensitivity);
  const double sensitivity_factor = std::clamp(4.0 - 0.03 * remapped_sensitivity, 1.0, 4.0);
  const double consensus_threshold = std::clamp(
    9.0 - 0.05 * remapped_sensitivity,
    kMinOutlierDistancePx,
    9.0);
  const bool prefer_vertical_axis =
    std::fabs(fallback_b.y - fallback_a.y) >= std::fabs(fallback_b.x - fallback_a.x);
  SideFitResult result;
  result.inliers = points;
  result.line = LineModel{fallback_a, fallback_b - fallback_a};

  if (result.inliers.empty())
  {
    return result;
  }

  if (result.inliers.size() >= 3)
  {
    std::vector<cv::Point2f> best_consensus;
    double best_coverage = -1.0;
    double best_mean_distance = std::numeric_limits<double>::infinity();
    const auto axis_value = [&](const cv::Point2f &point)
    {
      return prefer_vertical_axis ? static_cast<double>(point.y) : static_cast<double>(point.x);
    };

    const auto consider_candidate = [&](const LineModel &candidate_line)
    {
      std::vector<cv::Point2f> candidate_inliers;
      candidate_inliers.reserve(points.size());
      double min_axis = std::numeric_limits<double>::infinity();
      double max_axis = -std::numeric_limits<double>::infinity();
      double distance_sum = 0.0;

      for (const auto &point : points)
      {
        const double distance = pointToLineDistance(point, candidate_line.point, candidate_line.point + candidate_line.direction);
        if (distance > consensus_threshold)
        {
          continue;
        }

        candidate_inliers.push_back(point);
        distance_sum += distance;
        const double axis = axis_value(point);
        min_axis = std::min(min_axis, axis);
        max_axis = std::max(max_axis, axis);
      }

      if (candidate_inliers.size() < 2)
      {
        return;
      }

      const double coverage = max_axis - min_axis;
      const double mean_distance = distance_sum / static_cast<double>(candidate_inliers.size());
      const bool better_count = candidate_inliers.size() > best_consensus.size();
      const bool better_coverage =
        candidate_inliers.size() == best_consensus.size() &&
        coverage > best_coverage + 1e-3;
      const bool better_distance =
        candidate_inliers.size() == best_consensus.size() &&
        std::fabs(coverage - best_coverage) <= 1e-3 &&
        mean_distance + 1e-3 < best_mean_distance;
      if (!better_count && !better_coverage && !better_distance)
      {
        return;
      }

      best_consensus = std::move(candidate_inliers);
      best_coverage = coverage;
      best_mean_distance = mean_distance;
    };

    consider_candidate(result.line);
    for (std::size_t i = 0; i < points.size(); ++i)
    {
      for (std::size_t j = i + 1; j < points.size(); ++j)
      {
        const cv::Point2f direction = points[j] - points[i];
        if (cv::norm(direction) < 3.0F)
        {
          continue;
        }
        if (std::fabs(axis_value(points[j]) - axis_value(points[i])) < 6.0)
        {
          continue;
        }

        consider_candidate(LineModel{points[i], direction});
      }
    }

    if (best_consensus.size() >= 2)
    {
      result.inliers = std::move(best_consensus);
    }
  }

  for (int iteration = 0; iteration < kMaxSideTrimIterations; ++iteration)
  {
    result.line = fitLineToPoints(result.inliers, fallback_a, fallback_b);
    if (result.inliers.size() < 3)
    {
      break;
    }

    std::vector<double> distances;
    distances.reserve(result.inliers.size());
    for (const auto &point : result.inliers)
    {
      distances.push_back(pointToLineDistance(point, result.line.point, result.line.point + result.line.direction));
    }

    std::vector<double> sorted = distances;
    const auto median_it = sorted.begin() + static_cast<long>(sorted.size() / 2);
    std::nth_element(sorted.begin(), median_it, sorted.end());
    const double median = *median_it;
    const double threshold = std::max(kMinOutlierDistancePx, median * sensitivity_factor);
    std::vector<cv::Point2f> trimmed_inliers;
    trimmed_inliers.reserve(result.inliers.size());
    for (std::size_t i = 0; i < result.inliers.size(); ++i)
    {
      if (distances[i] <= threshold)
      {
        trimmed_inliers.push_back(result.inliers[i]);
      }
    }

    if (trimmed_inliers.size() < 2 || trimmed_inliers.size() == result.inliers.size())
    {
      break;
    }

    result.inliers = std::move(trimmed_inliers);
  }

  result.line = fitLineToPoints(result.inliers, fallback_a, fallback_b);
  return result;
}

std::vector<cv::Point2f> rejectSideOutliers(
  const std::vector<cv::Point2f> &edge_points,
  const cv::RotatedRect &rough_rect,
  int outlier_sensitivity)
{
  std::vector<cv::Point2f> corners(4);
  rough_rect.points(corners.data());
  const double remapped_sensitivity = remapOutlierSensitivityToFitRange(outlier_sensitivity);
  const double sensitivity_factor = std::clamp(4.0 - 0.03 * remapped_sensitivity, 1.0, 4.0);

  std::vector<std::vector<cv::Point2f>> side_groups(4);
  for (const auto &point : edge_points)
  {
    double best_distance = std::numeric_limits<double>::infinity();
    int best_side = 0;
    for (int side = 0; side < 4; ++side)
    {
      const double distance = pointToLineDistance(point, corners[side], corners[(side + 1) % 4]);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_side = side;
      }
    }
    side_groups[best_side].push_back(point);
  }

  std::vector<cv::Point2f> filtered_points;
  filtered_points.reserve(edge_points.size());
  for (int side = 0; side < 4; ++side)
  {
    const auto &group = side_groups[side];
    if (group.size() < 3)
    {
      filtered_points.insert(filtered_points.end(), group.begin(), group.end());
      continue;
    }

    std::vector<double> distances;
    distances.reserve(group.size());
    for (const auto &point : group)
    {
      distances.push_back(pointToLineDistance(point, corners[side], corners[(side + 1) % 4]));
    }

    std::vector<double> sorted = distances;
    const auto median_it = sorted.begin() + static_cast<long>(sorted.size() / 2);
    std::nth_element(sorted.begin(), median_it, sorted.end());
    const double median = *median_it;
    const double threshold = std::max(kMinOutlierDistancePx, median * sensitivity_factor);

    for (std::size_t i = 0; i < group.size(); ++i)
    {
      if (distances[i] <= threshold)
      {
        filtered_points.push_back(group[i]);
      }
    }
  }
  return filtered_points;
}

std::vector<std::vector<cv::Point2f>> groupPointsBySide(
  const std::vector<cv::Point2f> &points,
  const cv::RotatedRect &reference_rect)
{
  std::vector<cv::Point2f> corners(4);
  reference_rect.points(corners.data());
  std::vector<std::vector<cv::Point2f>> side_groups(4);

  for (const auto &point : points)
  {
    double best_distance = std::numeric_limits<double>::infinity();
    int best_side = 0;
    for (int side = 0; side < 4; ++side)
    {
      const double distance = pointToLineDistance(point, corners[side], corners[(side + 1) % 4]);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_side = side;
      }
    }
    side_groups[best_side].push_back(point);
  }
  return side_groups;
}

LineModel fitLineToPoints(const std::vector<cv::Point2f> &points, const cv::Point2f &fallback_a, const cv::Point2f &fallback_b)
{
  if (points.size() < 2)
  {
    return LineModel{fallback_a, fallback_b - fallback_a};
  }
  cv::Vec4f line;
  cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);
  return LineModel{cv::Point2f(line[2], line[3]), cv::Point2f(line[0], line[1])};
}

std::optional<cv::Point2f> intersectLines(const LineModel &a, const LineModel &b)
{
  const float det = a.direction.x * b.direction.y - a.direction.y * b.direction.x;
  if (std::fabs(det) < 1e-5F)
  {
    return std::nullopt;
  }
  const cv::Point2f delta = b.point - a.point;
  const float t = (delta.x * b.direction.y - delta.y * b.direction.x) / det;
  return a.point + t * a.direction;
}

std::pair<cv::Point2f, cv::Point2f> sideSampleEndpoints(
  const std::vector<cv::Point2f> &points,
  bool prefer_vertical_sort)
{
  if (points.empty())
  {
    return {cv::Point2f(), cv::Point2f()};
  }
  if (points.size() == 1)
  {
    return {points.front(), points.front()};
  }

  std::vector<cv::Point2f> sorted = points;
  std::sort(
    sorted.begin(),
    sorted.end(),
    [&](const cv::Point2f &a, const cv::Point2f &b)
    {
      if (prefer_vertical_sort)
      {
        if (std::fabs(a.y - b.y) > 1e-3F)
        {
          return a.y < b.y;
        }
        return a.x < b.x;
      }

      if (std::fabs(a.x - b.x) > 1e-3F)
      {
        return a.x < b.x;
      }
      return a.y < b.y;
    });
  return {sorted.front(), sorted.back()};
}

float sampleGrayAt(const cv::Mat &gray, const cv::Point2f &pt)
{
  const int x = static_cast<int>(std::round(pt.x));
  const int y = static_cast<int>(std::round(pt.y));
  if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows)
  {
    return 0.0F;
  }
  return static_cast<float>(gray.at<unsigned char>(y, x));
}

float sampleDepthAt(const cv::Mat &depth_m, const cv::Point2f &pt)
{
  const int x = static_cast<int>(std::round(pt.x));
  const int y = static_cast<int>(std::round(pt.y));
  if (x < 0 || y < 0 || x >= depth_m.cols || y >= depth_m.rows)
  {
    return 0.0F;
  }
  const float z = depth_m.at<float>(y, x);
  return std::isfinite(z) ? z : 0.0F;
}

std::optional<double> averageDepthAt(
  const cv::Mat &depth_m,
  const cv::Point2f &pt,
  int window_size = 5,
  const cv::Mat *allowed_mask = nullptr)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return std::nullopt;
  }
  if (allowed_mask != nullptr)
  {
    if (
      allowed_mask->empty() ||
      allowed_mask->type() != CV_8UC1 ||
      allowed_mask->rows != depth_m.rows ||
      allowed_mask->cols != depth_m.cols)
    {
      return std::nullopt;
    }
  }

  const int x = static_cast<int>(std::lround(pt.x));
  const int y = static_cast<int>(std::lround(pt.y));
  const int half_window = std::max(0, window_size / 2);

  std::vector<double> depths;
  depths.reserve(static_cast<std::size_t>((2 * half_window + 1) * (2 * half_window + 1)));
  for (int dy = -half_window; dy <= half_window; ++dy)
  {
    for (int dx = -half_window; dx <= half_window; ++dx)
    {
      const int sample_x = x + dx;
      const int sample_y = y + dy;
      if (sample_x < 0 || sample_y < 0 || sample_x >= depth_m.cols || sample_y >= depth_m.rows)
      {
        continue;
      }
      if (allowed_mask != nullptr && allowed_mask->at<unsigned char>(sample_y, sample_x) == 0)
      {
        continue;
      }

      const float depth = depth_m.at<float>(sample_y, sample_x);
      if (!std::isfinite(depth) || depth <= 0.0F)
      {
        continue;
      }
      depths.push_back(static_cast<double>(depth));
    }
  }

  if (depths.empty())
  {
    return std::nullopt;
  }

  const double median_depth = medianValue(depths);
  std::vector<double> deviations;
  deviations.reserve(depths.size());
  for (const double depth : depths)
  {
    deviations.push_back(std::fabs(depth - median_depth));
  }
  const double mad = medianValue(deviations);
  const double threshold = std::max(0.003, 3.0 * mad);

  double sum = 0.0;
  int count = 0;
  for (const double depth : depths)
  {
    if (std::fabs(depth - median_depth) <= threshold)
    {
      sum += depth;
      ++count;
    }
  }

  if (count == 0)
  {
    return median_depth;
  }

  return sum / static_cast<double>(count);
}

cv::Mat buildPixelMask(const cv::Size &size, const std::vector<cv::Point> &pixels)
{
  cv::Mat mask(size, CV_8UC1, cv::Scalar(0));
  for (const auto &pixel : pixels)
  {
    if (pixel.x < 0 || pixel.y < 0 || pixel.x >= size.width || pixel.y >= size.height)
    {
      continue;
    }
    mask.at<unsigned char>(pixel.y, pixel.x) = 255;
  }
  return mask;
}

std::optional<double> averageDepthFromPixels(
  const cv::Mat &depth_m,
  const std::vector<cv::Point> &pixels)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1 || pixels.empty())
  {
    return std::nullopt;
  }

  double sum_depth = 0.0;
  std::size_t depth_count = 0;
  for (const auto &pixel : pixels)
  {
    if (pixel.x < 0 || pixel.y < 0 || pixel.x >= depth_m.cols || pixel.y >= depth_m.rows)
    {
      continue;
    }
    const float depth = depth_m.at<float>(pixel.y, pixel.x);
    if (!std::isfinite(depth) || depth <= 0.0F)
    {
      continue;
    }
    sum_depth += static_cast<double>(depth);
    ++depth_count;
  }

  if (depth_count == 0)
  {
    return std::nullopt;
  }
  return sum_depth / static_cast<double>(depth_count);
}

std::optional<cv::Vec3d> estimateBlobCenterFromPixels(
  const std::vector<cv::Point> &pixels,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info)
{
  if (pixels.empty() || depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return std::nullopt;
  }
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6)
  {
    return std::nullopt;
  }

  cv::Vec3d point_sum(0.0, 0.0, 0.0);
  std::size_t point_count = 0;
  for (const auto &pixel : pixels)
  {
    if (pixel.x < 0 || pixel.y < 0 || pixel.x >= depth_m.cols || pixel.y >= depth_m.rows)
    {
      continue;
    }
    const float depth = depth_m.at<float>(pixel.y, pixel.x);
    if (!std::isfinite(depth) || depth <= 0.0F)
    {
      continue;
    }
    const cv::Point2f center_px(
      static_cast<float>(pixel.x) + 0.5F,
      static_cast<float>(pixel.y) + 0.5F);
    const cv::Vec3d point = projectPixelToCamera(center_px, static_cast<double>(depth), camera_info);
    point_sum += point;
    ++point_count;
  }

  if (point_count == 0)
  {
    return std::nullopt;
  }

  return point_sum * (1.0 / static_cast<double>(point_count));
}

cv::Vec3d projectPixelToCamera(const cv::Point2f &pixel, double depth_m, const CameraInfoMsg &camera_info)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  const double x = (static_cast<double>(pixel.x) - cx) * depth_m / fx;
  const double y = (static_cast<double>(pixel.y) - cy) * depth_m / fy;
  return cv::Vec3d(x, y, depth_m);
}

std::optional<cv::Point2f> projectCameraPointToPixel(const cv::Vec3d &point, const CameraInfoMsg &camera_info)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];
  if (fx <= 1e-6 || fy <= 1e-6 || point[2] <= 1e-6)
  {
    return std::nullopt;
  }

  const double u = fx * (point[0] / point[2]) + cx;
  const double v = fy * (point[1] / point[2]) + cy;
  return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

double vectorNorm(const cv::Vec3d &vec)
{
  return std::sqrt(vec.dot(vec));
}

bool normalizeVectorInPlace(cv::Vec3d &vec)
{
  const double norm = vectorNorm(vec);
  if (norm < 1e-6)
  {
    return false;
  }
  vec *= (1.0 / norm);
  return true;
}

cv::Point2f polygonCentroid(const std::vector<cv::Point2f> &points)
{
  if (points.empty())
  {
    return cv::Point2f(0.0F, 0.0F);
  }

  cv::Point2f centroid(0.0F, 0.0F);
  for (const auto &point : points)
  {
    centroid += point;
  }
  centroid *= (1.0F / static_cast<float>(points.size()));
  return centroid;
}

cv::Point2f inwardOffsetVectorForSegment(
  const cv::Point2f &start,
  const cv::Point2f &end,
  const cv::Point2f &interior_reference,
  int depth_edge_offset_px)
{
  const int clamped_offset_px = std::clamp(depth_edge_offset_px, kDepthEdgeOffsetMinPx, kDepthEdgeOffsetMaxPx);

  cv::Point2f edge_direction = end - start;
  const float edge_length = std::sqrt(edge_direction.dot(edge_direction));
  if (edge_length < 1e-3F)
  {
    return cv::Point2f(0.0F, 0.0F);
  }

  edge_direction *= (1.0F / edge_length);
  cv::Point2f normal(-edge_direction.y, edge_direction.x);
  const cv::Point2f midpoint = 0.5F * (start + end);
  if (normal.dot(interior_reference - midpoint) < 0.0F)
  {
    normal *= -1.0F;
  }
  return static_cast<float>(clamped_offset_px) * normal;
}

std::array<std::pair<cv::Point2f, cv::Point2f>, 4> buildDepthMeasurementSegments(
  const std::vector<cv::Point2f> &corners,
  int depth_edge_offset_px)
{
  std::array<std::pair<cv::Point2f, cv::Point2f>, 4> segments{};
  if (corners.size() != 4)
  {
    return segments;
  }

  const cv::Point2f interior_reference = polygonCentroid(corners);
  for (std::size_t side = 0; side < 4; ++side)
  {
    const cv::Point2f offset = inwardOffsetVectorForSegment(
      corners[side],
      corners[(side + 1) % 4],
      interior_reference,
      depth_edge_offset_px);
    segments[side] = {
      corners[side] + offset,
      corners[(side + 1) % 4] + offset,
    };
  }
  return segments;
}

std::optional<std::array<cv::Vec3d, 4>> estimateItemCornerCameraPoints(
  const std::vector<cv::Point2f> &corners,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  const cv::Mat *depth_sample_mask = nullptr,
  const std::optional<double> &fallback_depth_m = std::nullopt)
{
  if (corners.size() != 4)
  {
    return std::nullopt;
  }
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6)
  {
    return std::nullopt;
  }

  std::array<cv::Vec3d, 4> camera_points;
  for (std::size_t i = 0; i < corners.size(); ++i)
  {
    std::optional<double> depth = averageDepthAt(depth_m, corners[i], 7, depth_sample_mask);
    if (!depth.has_value() && fallback_depth_m.has_value())
    {
      depth = fallback_depth_m;
    }
    if (!depth.has_value())
    {
      return std::nullopt;
    }
    camera_points[i] = projectPixelToCamera(corners[i], *depth, camera_info);
  }

  return camera_points;
}

std::optional<double> estimateEdgeLengthCmFromDepthSamples(
  const std::vector<cv::Point2f> &side_points,
  const cv::Point2f &start,
  const cv::Point2f &end,
  const cv::Point2f &interior_reference,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px,
  std::vector<double> *sampled_depths_m = nullptr)
{
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6)
  {
    return std::nullopt;
  }

  const cv::Point2f side_vector = end - start;
  const double side_length_sq = static_cast<double>(side_vector.dot(side_vector));
  if (side_length_sq < 1e-6)
  {
    return std::nullopt;
  }

  std::vector<SideDepthSample> depth_samples;
  depth_samples.reserve(side_points.size() + 2);
  const cv::Point2f depth_offset = inwardOffsetVectorForSegment(
    start,
    end,
    interior_reference,
    depth_edge_offset_px);
  const auto append_sample = [&](const cv::Point2f &pixel, bool force_include)
  {
    const double t =
      ((static_cast<double>(pixel.x) - static_cast<double>(start.x)) * static_cast<double>(side_vector.x) +
       (static_cast<double>(pixel.y) - static_cast<double>(start.y)) * static_cast<double>(side_vector.y)) /
      side_length_sq;
    if (!force_include && (t < -0.05 || t > 1.05))
    {
      return;
    }

    const cv::Point2f depth_pixel = pixel + depth_offset;
    const auto depth = averageDepthAt(depth_m, depth_pixel, 7);
    if (!depth.has_value())
    {
      return;
    }

    depth_samples.push_back(SideDepthSample{
      std::clamp(t, 0.0, 1.0),
      *depth,
      projectPixelToCamera(depth_pixel, *depth, camera_info)
    });
  };

  append_sample(start, true);
  for (const auto &point : side_points)
  {
    append_sample(point, false);
  }
  append_sample(end, true);

  if (depth_samples.size() < 2)
  {
    return std::nullopt;
  }

  for (int iteration = 0; iteration < 2; ++iteration)
  {
    if (depth_samples.size() < 3)
    {
      break;
    }

    std::vector<cv::Point3f> points_3d;
    points_3d.reserve(depth_samples.size());
    for (const auto &sample : depth_samples)
    {
      points_3d.emplace_back(
        static_cast<float>(sample.camera_point[0]),
        static_cast<float>(sample.camera_point[1]),
        static_cast<float>(sample.camera_point[2]));
    }

    cv::Vec6f line_3d;
    cv::fitLine(points_3d, line_3d, cv::DIST_L2, 0.0, 0.01, 0.01);
    cv::Vec3d line_direction(line_3d[0], line_3d[1], line_3d[2]);
    const double direction_norm = vectorNorm(line_direction);
    if (direction_norm < 1e-9)
    {
      break;
    }
    line_direction *= (1.0 / direction_norm);
    const cv::Vec3d line_point(line_3d[3], line_3d[4], line_3d[5]);

    std::vector<double> distances;
    distances.reserve(depth_samples.size());
    for (const auto &sample : depth_samples)
    {
      const cv::Vec3d delta = sample.camera_point - line_point;
      distances.push_back(vectorNorm(delta.cross(line_direction)));
    }

    const double median_distance = medianValue(distances);
    const double threshold = std::max(0.002, 3.0 * median_distance);
    std::vector<SideDepthSample> filtered_samples;
    filtered_samples.reserve(depth_samples.size());
    for (std::size_t i = 0; i < depth_samples.size(); ++i)
    {
      if (distances[i] <= threshold)
      {
        filtered_samples.push_back(depth_samples[i]);
      }
    }

    if (filtered_samples.size() < 2 || filtered_samples.size() == depth_samples.size())
    {
      break;
    }
    depth_samples = std::move(filtered_samples);
  }

  if (depth_samples.size() < 2)
  {
    return std::nullopt;
  }

  std::vector<cv::Point3f> points_3d;
  points_3d.reserve(depth_samples.size());
  for (const auto &sample : depth_samples)
  {
    points_3d.emplace_back(
      static_cast<float>(sample.camera_point[0]),
      static_cast<float>(sample.camera_point[1]),
      static_cast<float>(sample.camera_point[2]));
  }

  cv::Vec6f line_3d;
  cv::fitLine(points_3d, line_3d, cv::DIST_L2, 0.0, 0.01, 0.01);
  cv::Vec3d line_direction(line_3d[0], line_3d[1], line_3d[2]);
  const double direction_norm = vectorNorm(line_direction);
  if (direction_norm < 1e-9)
  {
    return std::nullopt;
  }
  line_direction *= (1.0 / direction_norm);
  const cv::Vec3d line_point(line_3d[3], line_3d[4], line_3d[5]);

  double min_projection = std::numeric_limits<double>::infinity();
  double max_projection = -std::numeric_limits<double>::infinity();
  for (const auto &sample : depth_samples)
  {
    const double projection = line_direction.dot(sample.camera_point - line_point);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
    if (sampled_depths_m)
    {
      sampled_depths_m->push_back(sample.depth_m);
    }
  }

  const double length_m = max_projection - min_projection;
  return length_m > 0.0 ? std::optional<double>(length_m * 100.0) : std::nullopt;
}

double triangleArea3D(const cv::Vec3d &a, const cv::Vec3d &b, const cv::Vec3d &c)
{
  return 0.5 * vectorNorm((b - a).cross(c - a));
}

std::optional<ItemMetricEstimate> estimateItemMetricsFromCorners(
  const std::vector<cv::Point2f> &corners,
  const std::array<std::vector<cv::Point2f>, 4> &side_samples_by_segment,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px)
{
  if (corners.size() != 4)
  {
    return std::nullopt;
  }
  const int origin_idx = lowerLeftCornerIndex(corners);
  if (origin_idx < 0)
  {
    return std::nullopt;
  }

  const int prev_idx = (origin_idx + 3) % 4;
  const int next_idx = (origin_idx + 1) % 4;
  const int opposite_idx = (origin_idx + 2) % 4;
  const cv::Point2f interior_reference = polygonCentroid(corners);
  std::vector<double> sampled_depths_m;
  sampled_depths_m.reserve(128);
  std::array<std::optional<double>, 4> segment_lengths_cm;
  for (int side = 0; side < 4; ++side)
  {
    segment_lengths_cm[side] = estimateEdgeLengthCmFromDepthSamples(
      side_samples_by_segment[side],
      corners[side],
      corners[(side + 1) % 4],
      interior_reference,
      depth_m,
      camera_info,
      depth_edge_offset_px,
      &sampled_depths_m);
  }

  const auto length_for_edge = [&](int a, int b) -> std::optional<double>
  {
    if ((a + 1) % 4 == b)
    {
      return segment_lengths_cm[a];
    }
    if ((b + 1) % 4 == a)
    {
      return segment_lengths_cm[b];
    }
    return std::nullopt;
  };

  const auto origin_prev_len_cm = length_for_edge(origin_idx, prev_idx);
  const auto next_opposite_len_cm = length_for_edge(next_idx, opposite_idx);
  const auto origin_next_len_cm = length_for_edge(origin_idx, next_idx);
  const auto prev_opposite_len_cm = length_for_edge(prev_idx, opposite_idx);
  if (!origin_prev_len_cm.has_value() ||
      !next_opposite_len_cm.has_value() ||
      !origin_next_len_cm.has_value() ||
      !prev_opposite_len_cm.has_value())
  {
    return std::nullopt;
  }

  double mean_depth_m = 0.0;
  if (!sampled_depths_m.empty())
  {
    for (const double depth_m_value : sampled_depths_m)
    {
      mean_depth_m += depth_m_value;
    }
    mean_depth_m /= static_cast<double>(sampled_depths_m.size());
  }

  ItemMetricEstimate metrics;
  if (*origin_prev_len_cm >= *origin_next_len_cm)
  {
    metrics.edge_lengths_cm = {
      *origin_prev_len_cm,
      *next_opposite_len_cm,
      *origin_next_len_cm,
      *prev_opposite_len_cm,
    };
  }
  else
  {
    metrics.edge_lengths_cm = {
      *origin_next_len_cm,
      *prev_opposite_len_cm,
      *origin_prev_len_cm,
      *next_opposite_len_cm,
    };
  }

  if (!sampled_depths_m.empty())
  {
    metrics.mean_depth_m = mean_depth_m;
  }

  if (const auto camera_points = estimateItemCornerCameraPoints(corners, depth_m, camera_info); camera_points.has_value())
  {
    const double area_m2 =
      triangleArea3D((*camera_points)[0], (*camera_points)[1], (*camera_points)[2]) +
      triangleArea3D((*camera_points)[0], (*camera_points)[2], (*camera_points)[3]);
    if (area_m2 > 0.0)
    {
      metrics.area_cm2 = area_m2 * 10000.0;
    }
  }

  for (const double length_cm : metrics.edge_lengths_cm)
  {
    if (length_cm <= 0.0)
    {
      return std::nullopt;
    }
  }
  return metrics;
}

std::optional<ItemEstimate> buildItemEstimateFromIsolatedSideSamples(
  const std::vector<cv::Point2f> &edge_points,
  const std::vector<cv::Point2f> &left_samples,
  const std::vector<cv::Point2f> &right_samples,
  const std::vector<cv::Point2f> &top_samples,
  const std::vector<cv::Point2f> &bottom_samples,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px,
  int outlier_sensitivity)
{
  if (edge_points.size() < 8 ||
      left_samples.size() < 2 ||
      right_samples.size() < 2 ||
      top_samples.size() < 2 ||
      bottom_samples.size() < 2)
  {
    return std::nullopt;
  }

  const auto [left_a, left_b] = sideSampleEndpoints(left_samples, true);
  const auto [right_a, right_b] = sideSampleEndpoints(right_samples, true);
  const auto [top_a, top_b] = sideSampleEndpoints(top_samples, false);
  const auto [bottom_a, bottom_b] = sideSampleEndpoints(bottom_samples, false);

  const SideFitResult left_fit = fitSideLineWithTrimming(left_samples, left_a, left_b, outlier_sensitivity);
  const SideFitResult right_fit = fitSideLineWithTrimming(right_samples, right_a, right_b, outlier_sensitivity);
  const SideFitResult top_fit = fitSideLineWithTrimming(top_samples, top_a, top_b, outlier_sensitivity);
  const SideFitResult bottom_fit = fitSideLineWithTrimming(bottom_samples, bottom_a, bottom_b, outlier_sensitivity);
  if (left_fit.inliers.size() < 2 ||
      right_fit.inliers.size() < 2 ||
      top_fit.inliers.size() < 2 ||
      bottom_fit.inliers.size() < 2)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> filtered_edge_points;
  filtered_edge_points.reserve(
    left_fit.inliers.size() +
    right_fit.inliers.size() +
    top_fit.inliers.size() +
    bottom_fit.inliers.size());
  filtered_edge_points.insert(filtered_edge_points.end(), left_fit.inliers.begin(), left_fit.inliers.end());
  filtered_edge_points.insert(filtered_edge_points.end(), right_fit.inliers.begin(), right_fit.inliers.end());
  filtered_edge_points.insert(filtered_edge_points.end(), top_fit.inliers.begin(), top_fit.inliers.end());
  filtered_edge_points.insert(filtered_edge_points.end(), bottom_fit.inliers.begin(), bottom_fit.inliers.end());
  if (filtered_edge_points.size() < 8)
  {
    return std::nullopt;
  }

  const auto top_left = intersectLines(top_fit.line, left_fit.line);
  const auto top_right = intersectLines(top_fit.line, right_fit.line);
  const auto bottom_right = intersectLines(bottom_fit.line, right_fit.line);
  const auto bottom_left = intersectLines(bottom_fit.line, left_fit.line);
  if (!top_left.has_value() || !top_right.has_value() || !bottom_right.has_value() || !bottom_left.has_value())
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> corners_f{
    *top_left,
    *top_right,
    *bottom_right,
    *bottom_left,
  };

  const cv::RotatedRect rect = cv::minAreaRect(corners_f);
  if (rect.size.width < 20.0F || rect.size.height < 20.0F)
  {
    return std::nullopt;
  }

  std::vector<cv::Point> polygon;
  polygon.reserve(4);
  for (const auto &corner : corners_f)
  {
    polygon.emplace_back(static_cast<int>(std::round(corner.x)), static_cast<int>(std::round(corner.y)));
  }

  ItemEstimate estimate;
  estimate.rect = rect;
  estimate.center = rect.center;
  estimate.edge_points = edge_points;
  estimate.filtered_edge_points = filtered_edge_points;
  estimate.corners = corners_f;
  estimate.side_lines = {
    left_fit.line,
    right_fit.line,
    top_fit.line,
    bottom_fit.line,
  };
  estimate.polygon = polygon;
  const std::array<std::vector<cv::Point2f>, 4> side_samples_by_segment{
    top_fit.inliers,
    right_fit.inliers,
    bottom_fit.inliers,
    left_fit.inliers,
  };
  if (const auto metrics = estimateItemMetricsFromCorners(
        corners_f,
        side_samples_by_segment,
        depth_m,
        camera_info,
        depth_edge_offset_px); metrics.has_value())
  {
    estimate.has_metric_estimate = true;
    estimate.area_cm2 = metrics->area_cm2;
    estimate.mean_depth_m = metrics->mean_depth_m;
    estimate.edge_lengths_cm = metrics->edge_lengths_cm;
  }
  return estimate;
}

double medianValue(std::vector<double> values)
{
  if (values.empty())
  {
    return 0.0;
  }

  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<long>(middle), values.end());
  double median = values[middle];
  if ((values.size() % 2U) == 0U)
  {
    const auto max_lower = std::max_element(values.begin(), values.begin() + static_cast<long>(middle));
    if (max_lower != values.begin() + static_cast<long>(middle))
    {
      median = 0.5 * (median + *max_lower);
    }
  }
  return median;
}

std::optional<ItemEstimate> filterTimedItemEstimates(
  const std::deque<TimedItemEstimate> &estimate_history,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px)
{
  if (estimate_history.empty())
  {
    return std::nullopt;
  }

  const ItemEstimate &reference_estimate = estimate_history.back().estimate;
  if (reference_estimate.corners.size() != 4)
  {
    return reference_estimate;
  }

  std::vector<cv::Point2f> filtered_corners(4);
  for (std::size_t corner_index = 0; corner_index < filtered_corners.size(); ++corner_index)
  {
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(estimate_history.size());
    ys.reserve(estimate_history.size());

    for (const auto &sample : estimate_history)
    {
      if (sample.estimate.corners.size() != 4)
      {
        continue;
      }
      xs.push_back(sample.estimate.corners[corner_index].x);
      ys.push_back(sample.estimate.corners[corner_index].y);
    }

    if (xs.empty() || ys.empty())
    {
      return reference_estimate;
    }

    filtered_corners[corner_index] = cv::Point2f(
      static_cast<float>(medianValue(xs)),
      static_cast<float>(medianValue(ys)));
  }

  const cv::RotatedRect filtered_rect = cv::minAreaRect(filtered_corners);
  if (filtered_rect.size.width < 20.0F || filtered_rect.size.height < 20.0F)
  {
    return std::nullopt;
  }

  ItemEstimate filtered_estimate = reference_estimate;
  filtered_estimate.rect = filtered_rect;
  filtered_estimate.center = filtered_rect.center;
  filtered_estimate.corners = filtered_corners;
  filtered_estimate.polygon.clear();
  filtered_estimate.polygon.reserve(filtered_corners.size());
  for (const auto &corner : filtered_corners)
  {
    filtered_estimate.polygon.emplace_back(
      static_cast<int>(std::round(corner.x)),
      static_cast<int>(std::round(corner.y)));
  }

  filtered_estimate.side_lines = {
    LineModel{filtered_corners[3], filtered_corners[0] - filtered_corners[3]},
    LineModel{filtered_corners[2], filtered_corners[1] - filtered_corners[2]},
    LineModel{filtered_corners[0], filtered_corners[1] - filtered_corners[0]},
    LineModel{filtered_corners[3], filtered_corners[2] - filtered_corners[3]},
  };

  filtered_estimate.has_metric_estimate = false;
  filtered_estimate.area_cm2 = 0.0;
  filtered_estimate.mean_depth_m = 0.0;
  filtered_estimate.edge_lengths_cm = {0.0, 0.0, 0.0, 0.0};
  std::array<std::vector<cv::Point2f>, 4> side_samples_by_segment;
  for (const auto &point : filtered_estimate.filtered_edge_points)
  {
    double best_distance = std::numeric_limits<double>::infinity();
    int best_side = 0;
    for (int side = 0; side < 4; ++side)
    {
      const double distance = pointToLineDistance(point, filtered_corners[side], filtered_corners[(side + 1) % 4]);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_side = side;
      }
    }
    side_samples_by_segment[best_side].push_back(point);
  }
  if (const auto metrics = estimateItemMetricsFromCorners(
        filtered_corners,
        side_samples_by_segment,
        depth_m,
        camera_info,
        depth_edge_offset_px); metrics.has_value())
  {
    filtered_estimate.has_metric_estimate = true;
    filtered_estimate.area_cm2 = metrics->area_cm2;
    filtered_estimate.mean_depth_m = metrics->mean_depth_m;
    filtered_estimate.edge_lengths_cm = metrics->edge_lengths_cm;
  }

  return filtered_estimate;
}

int lowerLeftCornerIndex(const std::vector<cv::Point2f> &corners)
{
  if (corners.size() != 4)
  {
    return -1;
  }

  std::array<int, 4> corner_indices{0, 1, 2, 3};
  std::sort(
    corner_indices.begin(),
    corner_indices.end(),
    [&](int a, int b)
    {
      const cv::Point2f &corner_a = corners[a];
      const cv::Point2f &corner_b = corners[b];
      if (std::fabs(corner_a.y - corner_b.y) > 1e-3F)
      {
        return corner_a.y > corner_b.y;
      }
      return corner_a.x < corner_b.x;
    });

  int lower_left_idx = corner_indices[0];
  if (corners[corner_indices[1]].x < corners[corner_indices[0]].x)
  {
    lower_left_idx = corner_indices[1];
  }
  return lower_left_idx;
}

bool normalizeVector2D(cv::Point2f *vector)
{
  if (vector == nullptr)
  {
    return false;
  }
  const float norm = std::sqrt(vector->dot(*vector));
  if (norm < 1e-3F)
  {
    return false;
  }
  *vector *= (1.0F / norm);
  return true;
}

float wrapAnglePi(float angle_rad)
{
  while (angle_rad <= -static_cast<float>(CV_PI))
  {
    angle_rad += static_cast<float>(2.0 * CV_PI);
  }
  while (angle_rad > static_cast<float>(CV_PI))
  {
    angle_rad -= static_cast<float>(2.0 * CV_PI);
  }
  return angle_rad;
}

float vectorAngleRad(const cv::Point2f &vector)
{
  return std::atan2(vector.y, vector.x);
}

std::vector<BinarizedBlobComponent2D> extractConnectedBlobComponents(
  const cv::Mat &mask,
  cv::Mat *labels_out = nullptr)
{
  std::vector<BinarizedBlobComponent2D> components;
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    return components;
  }

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(
    mask, labels, stats, centroids, 8, CV_32S);
  if (labels_out != nullptr)
  {
    *labels_out = labels;
  }
  if (component_count <= 1)
  {
    return components;
  }

  std::vector<std::vector<cv::Point>> points_by_label(
    static_cast<std::size_t>(component_count));
  for (int y = 0; y < labels.rows; ++y)
  {
    const int *label_row = labels.ptr<int>(y);
    for (int x = 0; x < labels.cols; ++x)
    {
      const int label = label_row[x];
      if (label > 0)
      {
        points_by_label[static_cast<std::size_t>(label)].push_back(cv::Point(x, y));
      }
    }
  }

  components.reserve(static_cast<std::size_t>(component_count - 1));
  for (int label = 1; label < component_count; ++label)
  {
    const int area_px = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area_px < 6)
    {
      continue;
    }
    const int width = std::max(1, stats.at<int>(label, cv::CC_STAT_WIDTH));
    const int height = std::max(1, stats.at<int>(label, cv::CC_STAT_HEIGHT));
    auto &pixels = points_by_label[static_cast<std::size_t>(label)];
    if (pixels.size() < 3)
    {
      continue;
    }

    BinarizedBlobComponent2D component;
    component.label = label;
    component.area_px = area_px;
    component.bbox = cv::Rect(
      stats.at<int>(label, cv::CC_STAT_LEFT),
      stats.at<int>(label, cv::CC_STAT_TOP),
      width,
      height);
    component.pixels = std::move(pixels);
    cv::convexHull(component.pixels, component.hull, true, true);
    if (component.hull.size() < 3)
    {
      continue;
    }
    const double rect_area_px = fittedRectAreaPx(component.hull);
    if (!std::isfinite(rect_area_px) || rect_area_px < 1.0)
    {
      continue;
    }
    component.area_px = std::max(1, static_cast<int>(std::round(rect_area_px)));
    component.aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(component.hull));
    component.fill_ratio = computeBlobHullFillRatio(component.pixels, component.hull);
    components.push_back(std::move(component));
  }
  return components;
}

std::optional<BinarizedPoseEstimate2D::BlobPose2D> buildBlobPoseFromComponent(
  const BinarizedBlobComponent2D &component)
{
  if (component.hull.size() < 3)
  {
    return std::nullopt;
  }

  const cv::RotatedRect hull_quad = cv::minAreaRect(component.hull);
  if (hull_quad.size.width < 2.0F || hull_quad.size.height < 2.0F)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> corners(4);
  hull_quad.points(corners.data());
  const int origin_idx = lowerLeftCornerIndex(corners);
  if (origin_idx < 0)
  {
    return std::nullopt;
  }

  const int prev_idx = (origin_idx + 3) % 4;
  const int next_idx = (origin_idx + 1) % 4;
  const cv::Point2f dir_prev = corners[prev_idx] - corners[origin_idx];
  const cv::Point2f dir_next = corners[next_idx] - corners[origin_idx];
  const float prev_len = std::sqrt(dir_prev.dot(dir_prev));
  const float next_len = std::sqrt(dir_next.dot(dir_next));
  if (prev_len < 1e-3F || next_len < 1e-3F)
  {
    return std::nullopt;
  }

  BinarizedPoseEstimate2D::BlobPose2D blob_pose;
  blob_pose.label = component.label;
  blob_pose.pixels = component.pixels;
  blob_pose.corners = corners;
  blob_pose.hull_points.reserve(corners.size());
  for (const auto &corner : corners)
  {
    blob_pose.hull_points.emplace_back(
      static_cast<int>(std::lround(corner.x)),
      static_cast<int>(std::lround(corner.y)));
  }
  blob_pose.origin = corners[origin_idx];
  if (prev_len >= next_len)
  {
    blob_pose.x_axis_tip = corners[prev_idx];
    blob_pose.z_axis_tip = corners[next_idx];
    blob_pose.x_length_px = prev_len;
    blob_pose.z_length_px = next_len;
  }
  else
  {
    blob_pose.x_axis_tip = corners[next_idx];
    blob_pose.z_axis_tip = corners[prev_idx];
    blob_pose.x_length_px = next_len;
    blob_pose.z_length_px = prev_len;
  }
  cv::Point2f center(0.0F, 0.0F);
  for (const auto &corner : corners)
  {
    center += corner;
  }
  center *= 0.25F;
  blob_pose.member_labels.push_back(component.label);
  blob_pose.member_centers_px.push_back(center);
  blob_pose.member_centers_norm.push_back(cv::Point2f(0.5F, 0.5F));
  return blob_pose;
}

std::optional<BinarizedPoseEstimate2D::BlobPose2D> buildBlobPoseFromPolygon(
  const std::vector<cv::Point2f> &polygon_points,
  int label = 1)
{
  if (polygon_points.size() < 3)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> hull_points_f;
  cv::convexHull(polygon_points, hull_points_f, true, true);
  if (hull_points_f.size() < 3)
  {
    return std::nullopt;
  }

  const cv::RotatedRect hull_quad = cv::minAreaRect(hull_points_f);
  if (hull_quad.size.width < 2.0F || hull_quad.size.height < 2.0F)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> corners(4);
  hull_quad.points(corners.data());
  const int origin_idx = lowerLeftCornerIndex(corners);
  if (origin_idx < 0)
  {
    return std::nullopt;
  }

  const int prev_idx = (origin_idx + 3) % 4;
  const int next_idx = (origin_idx + 1) % 4;
  const cv::Point2f dir_prev = corners[prev_idx] - corners[origin_idx];
  const cv::Point2f dir_next = corners[next_idx] - corners[origin_idx];
  const float prev_len = std::sqrt(dir_prev.dot(dir_prev));
  const float next_len = std::sqrt(dir_next.dot(dir_next));
  if (prev_len < 1e-3F || next_len < 1e-3F)
  {
    return std::nullopt;
  }

  BinarizedPoseEstimate2D::BlobPose2D blob_pose;
  blob_pose.label = label;
  blob_pose.corners = corners;
  blob_pose.hull_points.reserve(corners.size());
  for (const auto &corner : corners)
  {
    blob_pose.hull_points.emplace_back(
      static_cast<int>(std::lround(corner.x)),
      static_cast<int>(std::lround(corner.y)));
  }
  blob_pose.origin = corners[origin_idx];
  if (prev_len >= next_len)
  {
    blob_pose.x_axis_tip = corners[prev_idx];
    blob_pose.z_axis_tip = corners[next_idx];
    blob_pose.x_length_px = prev_len;
    blob_pose.z_length_px = next_len;
  }
  else
  {
    blob_pose.x_axis_tip = corners[next_idx];
    blob_pose.z_axis_tip = corners[prev_idx];
    blob_pose.x_length_px = next_len;
    blob_pose.z_length_px = prev_len;
  }
  cv::Point2f center(0.0F, 0.0F);
  for (const auto &corner : corners)
  {
    center += corner;
  }
  center *= 0.25F;
  blob_pose.member_labels.push_back(label);
  blob_pose.member_centers_px.push_back(center);
  blob_pose.member_centers_norm.push_back(cv::Point2f(0.5F, 0.5F));
  return blob_pose;
}

cv::Point2f poseBlobCenterPx(const BinarizedPoseEstimate2D::BlobPose2D &blob_pose)
{
  if (blob_pose.member_count >= 2 && !blob_pose.member_centers_px.empty())
  {
    cv::Point2f center(0.0F, 0.0F);
    for (const auto &member_center : blob_pose.member_centers_px)
    {
      center += member_center;
    }
    return center * (1.0F / static_cast<float>(blob_pose.member_centers_px.size()));
  }
  if (!blob_pose.corners.empty())
  {
    cv::Point2f center(0.0F, 0.0F);
    for (const auto &corner : blob_pose.corners)
    {
      center += corner;
    }
    return center * (1.0F / static_cast<float>(blob_pose.corners.size()));
  }
  return blob_pose.origin;
}

bool blobPoseAxes(
  const BinarizedPoseEstimate2D::BlobPose2D &blob_pose,
  cv::Point2f *x_dir_out,
  cv::Point2f *z_dir_out)
{
  if (x_dir_out == nullptr || z_dir_out == nullptr)
  {
    return false;
  }

  const cv::Point2f x_vec = blob_pose.x_axis_tip - blob_pose.origin;
  const cv::Point2f z_vec = blob_pose.z_axis_tip - blob_pose.origin;
  const float x_norm = std::sqrt(x_vec.dot(x_vec));
  const float z_norm = std::sqrt(z_vec.dot(z_vec));
  if (x_norm < 1e-3F || z_norm < 1e-3F)
  {
    return false;
  }

  *x_dir_out = x_vec * (1.0F / x_norm);
  *z_dir_out = z_vec * (1.0F / z_norm);
  return true;
}

std::optional<cv::Point2f> normalizedPointInBlobPoseFrame(
  const BinarizedPoseEstimate2D::BlobPose2D &blob_pose,
  const cv::Point2f &point_px)
{
  if (blob_pose.x_length_px < 1e-3F || blob_pose.z_length_px < 1e-3F)
  {
    return std::nullopt;
  }

  cv::Point2f x_dir;
  cv::Point2f z_dir;
  if (!blobPoseAxes(blob_pose, &x_dir, &z_dir))
  {
    return std::nullopt;
  }

  const cv::Point2f delta = point_px - blob_pose.origin;
  return cv::Point2f(
    delta.dot(x_dir) / std::max(1e-3F, blob_pose.x_length_px),
    delta.dot(z_dir) / std::max(1e-3F, blob_pose.z_length_px));
}

std::optional<cv::Point2f> pointFromNormalizedBlobPoseFrame(
  const BinarizedPoseEstimate2D::BlobPose2D &blob_pose,
  const cv::Point2f &normalized_point)
{
  cv::Point2f x_dir;
  cv::Point2f z_dir;
  if (!blobPoseAxes(blob_pose, &x_dir, &z_dir))
  {
    return std::nullopt;
  }

  return blob_pose.origin +
    (x_dir * (normalized_point.x * std::max(1e-3F, blob_pose.x_length_px))) +
    (z_dir * (normalized_point.y * std::max(1e-3F, blob_pose.z_length_px)));
}

std::optional<cv::Point2f> componentCenterPx(const BinarizedBlobComponent2D &component)
{
  const auto blob_pose = buildBlobPoseFromComponent(component);
  if (!blob_pose.has_value())
  {
    return std::nullopt;
  }
  return poseBlobCenterPx(*blob_pose);
}

std::optional<double> computeSelfHullFillRatioForPixels(const std::vector<cv::Point> &pixels)
{
  if (pixels.size() < 3)
  {
    return std::nullopt;
  }

  std::vector<cv::Point> hull_points;
  cv::convexHull(pixels, hull_points, true, true);
  if (hull_points.size() < 3)
  {
    return std::nullopt;
  }

  const double fill_ratio = computeBlobHullFillRatio(pixels, hull_points);
  if (!std::isfinite(fill_ratio))
  {
    return std::nullopt;
  }
  return fill_ratio;
}

cv::Point2f applyDirectShapeFitTransform(
  const DirectShapeFitResult2D &fit_result,
  const cv::Point2f &reference_point)
{
  const double angle_rad = fit_result.angle_deg * CV_PI / 180.0;
  const double cos_theta = std::cos(angle_rad);
  const double sin_theta = std::sin(angle_rad);
  const cv::Point2f centered = reference_point - fit_result.reference_center_px;
  return cv::Point2f(
    static_cast<float>(fit_result.scale * (
        static_cast<double>(centered.x) * cos_theta -
        static_cast<double>(centered.y) * sin_theta)),
    static_cast<float>(fit_result.scale * (
        static_cast<double>(centered.x) * sin_theta +
	    static_cast<double>(centered.y) * cos_theta))) + fit_result.transform_offset_px;
}

double pairLayoutToleranceNormalized(int blob_tolerance_percent);

double pairLayoutErrorNormalized(
  const BinarizedPoseEstimate2D::BlobPose2D &candidate_pose,
  const PoseBlobReference2D &reference_blob);

std::optional<DirectShapeFitResult2D> estimateDirectShapeFitMatch(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent,
  const cv::Point &image_offset_px = cv::Point(0, 0),
  std::string *status_text = nullptr);

std::optional<BinarizedPoseEstimate2D::BlobPose2D> buildPairBlobPoseFromComponents(
  const BinarizedBlobComponent2D &anchor_component,
  const BinarizedBlobComponent2D &companion_component)
{
  if (anchor_component.hull.size() < 3 || companion_component.hull.size() < 3)
  {
    return std::nullopt;
  }

  auto anchor_center_px = componentCenterPx(anchor_component);
  auto companion_center_px = componentCenterPx(companion_component);
  if (!anchor_center_px.has_value() || !companion_center_px.has_value())
  {
    return std::nullopt;
  }

  BinarizedBlobComponent2D group_component;
  group_component.label = std::min(anchor_component.label, companion_component.label);
  group_component.pixels.reserve(anchor_component.pixels.size() + companion_component.pixels.size());
  group_component.pixels.insert(
    group_component.pixels.end(),
    anchor_component.pixels.begin(),
    anchor_component.pixels.end());
  group_component.pixels.insert(
    group_component.pixels.end(),
    companion_component.pixels.begin(),
    companion_component.pixels.end());
  cv::convexHull(group_component.pixels, group_component.hull, true, true);
  if (group_component.hull.size() < 3)
  {
    return std::nullopt;
  }
  const double rect_area_px = fittedRectAreaPx(group_component.hull);
  if (!std::isfinite(rect_area_px) || rect_area_px < 1.0)
  {
    return std::nullopt;
  }
  group_component.area_px = std::max(1, static_cast<int>(std::round(rect_area_px)));
  group_component.aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(group_component.hull));
  group_component.fill_ratio = computeBlobHullFillRatio(group_component.pixels, group_component.hull);

  auto group_pose = buildBlobPoseFromComponent(group_component);
  if (!group_pose.has_value())
  {
    return std::nullopt;
  }

  const auto anchor_center_norm = normalizedPointInBlobPoseFrame(*group_pose, *anchor_center_px);
  const auto companion_center_norm = normalizedPointInBlobPoseFrame(*group_pose, *companion_center_px);
  if (!anchor_center_norm.has_value() || !companion_center_norm.has_value())
  {
    return std::nullopt;
  }

  group_pose->has_custom_anchor = true;
  group_pose->anchor_point_px = *anchor_center_px;
  group_pose->anchor_pixels = anchor_component.pixels;
  group_pose->companion_pixels = companion_component.pixels;
  group_pose->member_count = 2;
  group_pose->member_labels = {anchor_component.label, companion_component.label};
  group_pose->member_centers_px = {*anchor_center_px, *companion_center_px};
  group_pose->member_centers_norm = {*anchor_center_norm, *companion_center_norm};
  return group_pose;
}

std::vector<cv::Point> extractMaskedPolygonPixels(
  const cv::Mat &mask,
  const std::vector<cv::Point2f> &polygon_points,
  const cv::Point &image_offset_px = cv::Point(0, 0),
  bool allow_polygon_fallback = true,
  double *overlap_ratio_out = nullptr)
{
  std::vector<cv::Point> pixels;
  if (mask.empty() || mask.type() != CV_8UC1 || polygon_points.size() < 3)
  {
    return pixels;
  }

  std::vector<cv::Point> local_polygon;
  local_polygon.reserve(polygon_points.size());
  for (const auto &point : polygon_points)
  {
    local_polygon.emplace_back(
      static_cast<int>(std::lround(point.x)) - image_offset_px.x,
      static_cast<int>(std::lround(point.y)) - image_offset_px.y);
  }

  cv::Mat polygon_mask(mask.size(), CV_8UC1, cv::Scalar(0));
  const std::vector<std::vector<cv::Point>> polygons{local_polygon};
  cv::fillPoly(polygon_mask, polygons, cv::Scalar(255));

  cv::Mat overlap_mask;
  cv::bitwise_and(mask, polygon_mask, overlap_mask);
  const int polygon_area_px = cv::countNonZero(polygon_mask);
  const int overlap_area_px = cv::countNonZero(overlap_mask);
  if (overlap_ratio_out != nullptr)
  {
    *overlap_ratio_out = static_cast<double>(overlap_area_px) / static_cast<double>(std::max(1, polygon_area_px));
  }
  const cv::Mat &pixel_source =
    (allow_polygon_fallback && overlap_area_px == 0)
    ? polygon_mask
    : overlap_mask;

  pixels.reserve(static_cast<std::size_t>(cv::countNonZero(pixel_source)));
  for (int y = 0; y < pixel_source.rows; ++y)
  {
    const unsigned char *row = pixel_source.ptr<unsigned char>(y);
    for (int x = 0; x < pixel_source.cols; ++x)
    {
      if (row[x] == 0)
      {
        continue;
      }
      pixels.emplace_back(x + image_offset_px.x, y + image_offset_px.y);
    }
  }

  return pixels;
}

std::optional<BinarizedBlobComponent2D> buildBlobComponentFromPixels(
  const std::vector<cv::Point> &pixels,
  int label = 1)
{
  if (pixels.size() < 3)
  {
    return std::nullopt;
  }

  int min_x = std::numeric_limits<int>::max();
  int min_y = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int max_y = std::numeric_limits<int>::min();
  for (const auto &pixel : pixels)
  {
    min_x = std::min(min_x, pixel.x);
    min_y = std::min(min_y, pixel.y);
    max_x = std::max(max_x, pixel.x);
    max_y = std::max(max_y, pixel.y);
  }
  if (min_x > max_x || min_y > max_y)
  {
    return std::nullopt;
  }

  BinarizedBlobComponent2D component;
  component.label = label;
  component.pixels = pixels;
  component.bbox = cv::Rect(min_x, min_y, (max_x - min_x) + 1, (max_y - min_y) + 1);
  cv::convexHull(component.pixels, component.hull, true, true);
  if (component.hull.size() < 3)
  {
    return std::nullopt;
  }

  const double rect_area_px = fittedRectAreaPx(component.hull);
  if (!std::isfinite(rect_area_px) || rect_area_px < 1.0)
  {
    return std::nullopt;
  }

  component.area_px = std::max(1, static_cast<int>(std::round(rect_area_px)));
  component.aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(component.hull));
  component.fill_ratio = computeBlobHullFillRatio(component.pixels, component.hull);
  return component;
}

bool componentMatchesReferenceBlobArea(
  const BinarizedBlobComponent2D &component,
  int reference_area_px,
  int blob_tolerance_percent)
{
  if (reference_area_px <= 0)
  {
    return true;
  }

  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);
  const double tolerance_ratio = static_cast<double>(clamped_tolerance_percent) / 100.0;
  const double min_ratio = std::max(0.01, 1.0 - tolerance_ratio);
  const double max_ratio = 1.0 + tolerance_ratio;
  const double area_ratio = static_cast<double>(component.area_px) /
    static_cast<double>(std::max(1, reference_area_px));
  return std::isfinite(area_ratio) && area_ratio >= min_ratio && area_ratio <= max_ratio;
}

std::optional<cv::Point2f> referenceCompanionCenterInAnchorFrame(
  const PoseBlobReference2D &reference_blob)
{
  const std::vector<cv::Point> &anchor_hull =
    reference_blob.anchor_hull.empty() ? reference_blob.hull : reference_blob.anchor_hull;
  if (anchor_hull.size() < 3 || reference_blob.companion_hull.size() < 3)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> anchor_hull_f;
  anchor_hull_f.reserve(anchor_hull.size());
  for (const auto &point : anchor_hull)
  {
    anchor_hull_f.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }

  std::vector<cv::Point2f> companion_hull_f;
  companion_hull_f.reserve(reference_blob.companion_hull.size());
  for (const auto &point : reference_blob.companion_hull)
  {
    companion_hull_f.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }

  const auto reference_anchor_pose = buildBlobPoseFromPolygon(anchor_hull_f, 1);
  const auto reference_companion_pose = buildBlobPoseFromPolygon(companion_hull_f, 2);
  if (!reference_anchor_pose.has_value() || !reference_companion_pose.has_value())
  {
    return std::nullopt;
  }

  return normalizedPointInBlobPoseFrame(
    *reference_anchor_pose,
    poseBlobCenterPx(*reference_companion_pose));
}

cv::Point2f polygonCenterPx(const std::vector<cv::Point2f> &polygon_points)
{
  if (polygon_points.empty())
  {
    return cv::Point2f(0.0F, 0.0F);
  }

  cv::Point2f center(0.0F, 0.0F);
  for (const auto &point : polygon_points)
  {
    center += point;
  }
  return center * (1.0F / static_cast<float>(polygon_points.size()));
}

std::vector<cv::Point2f> scalePolygonAroundCenter(
  const std::vector<cv::Point2f> &polygon_points,
  const cv::Point2f &center,
  double scale_factor)
{
  if (polygon_points.empty() || !std::isfinite(scale_factor) || scale_factor <= 0.0)
  {
    return polygon_points;
  }

  std::vector<cv::Point2f> scaled_points;
  scaled_points.reserve(polygon_points.size());
  for (const auto &point : polygon_points)
  {
    const cv::Point2f delta = point - center;
    scaled_points.push_back(center + static_cast<float>(scale_factor) * delta);
  }
  return scaled_points;
}

struct CompanionPredictionBranch2D
{
  cv::Point2f center;
  cv::Point2f center_norm;
  bool has_center_norm {false};
  std::vector<cv::Point2f> expected_hull;
  std::vector<cv::Point2f> search_hull;
  int search_radius_px {8};
  double angle_offset_deg {0.0};
};

std::optional<BinarizedBlobComponent2D> refineBlobComponentFromPredictedPolygon(
  const cv::Mat &mask,
  const std::vector<cv::Point2f> &predicted_polygon_points,
  int label,
  int search_radius_px,
  const cv::Point &image_offset_px = cv::Point(0, 0),
  int min_overlap_pixels = 12,
  double min_overlap_ratio = 0.10)
{
  if (mask.empty() || mask.type() != CV_8UC1 || predicted_polygon_points.size() < 3)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> local_polygon_points;
  local_polygon_points.reserve(predicted_polygon_points.size());
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto &point : predicted_polygon_points)
  {
    const cv::Point2f local_point(
      point.x - static_cast<float>(image_offset_px.x),
      point.y - static_cast<float>(image_offset_px.y));
    local_polygon_points.push_back(local_point);
    min_x = std::min(min_x, local_point.x);
    min_y = std::min(min_y, local_point.y);
    max_x = std::max(max_x, local_point.x);
    max_y = std::max(max_y, local_point.y);
  }

  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y))
  {
    return std::nullopt;
  }

  constexpr int kTemplateMarginPx = 4;
  const int template_width = static_cast<int>(std::ceil(max_x - min_x)) + 1 + (2 * kTemplateMarginPx);
  const int template_height = static_cast<int>(std::ceil(max_y - min_y)) + 1 + (2 * kTemplateMarginPx);
  if (template_width < 8 || template_height < 8)
  {
    return std::nullopt;
  }
  if (template_width >= mask.cols || template_height >= mask.rows)
  {
    return std::nullopt;
  }

  const cv::Point predicted_top_left(
    static_cast<int>(std::floor(min_x)) - kTemplateMarginPx,
    static_cast<int>(std::floor(min_y)) - kTemplateMarginPx);

  std::vector<cv::Point> template_polygon_pixels;
  template_polygon_pixels.reserve(local_polygon_points.size());
  for (const auto &point : local_polygon_points)
  {
    template_polygon_pixels.emplace_back(
      static_cast<int>(std::lround(point.x)) - predicted_top_left.x,
      static_cast<int>(std::lround(point.y)) - predicted_top_left.y);
  }

  cv::Mat template_mask(template_height, template_width, CV_8UC1, cv::Scalar(0));
  const std::vector<std::vector<cv::Point>> polygons{template_polygon_pixels};
  cv::fillPoly(template_mask, polygons, cv::Scalar(255));
  const int template_area_px = cv::countNonZero(template_mask);
  if (template_area_px < 24)
  {
    return std::nullopt;
  }

  const cv::Rect search_rect(
    std::max(0, predicted_top_left.x - search_radius_px),
    std::max(0, predicted_top_left.y - search_radius_px),
    std::min(mask.cols, predicted_top_left.x + template_width + search_radius_px) -
      std::max(0, predicted_top_left.x - search_radius_px),
    std::min(mask.rows, predicted_top_left.y + template_height + search_radius_px) -
      std::max(0, predicted_top_left.y - search_radius_px));
  if (
    search_rect.width < template_width ||
    search_rect.height < template_height)
  {
    return std::nullopt;
  }

  cv::Mat search_mask_f32;
  mask(search_rect).convertTo(search_mask_f32, CV_32FC1, 1.0 / 255.0);
  cv::Mat template_mask_f32;
  template_mask.convertTo(template_mask_f32, CV_32FC1, 1.0 / 255.0);
  cv::Mat response;
  cv::matchTemplate(search_mask_f32, template_mask_f32, response, cv::TM_SQDIFF_NORMED);

  double min_value = 0.0;
  cv::Point min_location;
  cv::minMaxLoc(response, &min_value, nullptr, &min_location, nullptr);
  if (!std::isfinite(min_value))
  {
    return std::nullopt;
  }

  const cv::Point best_top_left = search_rect.tl() + min_location;
  const cv::Rect candidate_roi(best_top_left.x, best_top_left.y, template_width, template_height);
  if (
    candidate_roi.x < 0 || candidate_roi.y < 0 ||
    candidate_roi.x + candidate_roi.width > mask.cols ||
    candidate_roi.y + candidate_roi.height > mask.rows)
  {
    return std::nullopt;
  }

  std::vector<cv::Point2f> refined_polygon_points;
  refined_polygon_points.reserve(predicted_polygon_points.size());
  for (const auto &point : local_polygon_points)
  {
    refined_polygon_points.emplace_back(
      point.x + static_cast<float>(best_top_left.x - predicted_top_left.x + image_offset_px.x),
      point.y + static_cast<float>(best_top_left.y - predicted_top_left.y + image_offset_px.y));
  }

  double overlap_ratio = 0.0;
  const std::vector<cv::Point> overlap_pixels = extractMaskedPolygonPixels(
    mask,
    refined_polygon_points,
    image_offset_px,
    false,
    &overlap_ratio);
  if (
    overlap_pixels.size() < static_cast<std::size_t>(std::max(1, min_overlap_pixels)) ||
    overlap_ratio < std::max(0.0, min_overlap_ratio))
  {
    return std::nullopt;
  }

  return buildBlobComponentFromPixels(overlap_pixels, label);
}

cv::Mat buildCircularSearchMask(
  const cv::Size &mask_size,
  const cv::Point2f &center_px,
  int radius_px,
  const cv::Point &image_offset_px = cv::Point(0, 0))
{
  if (mask_size.width <= 0 || mask_size.height <= 0 || radius_px <= 0)
  {
    return {};
  }

  cv::Mat search_mask(mask_size, CV_8UC1, cv::Scalar(0));
  const cv::Point local_center(
    static_cast<int>(std::lround(center_px.x)) - image_offset_px.x,
    static_cast<int>(std::lround(center_px.y)) - image_offset_px.y);
  cv::circle(search_mask, local_center, radius_px, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
  return search_mask;
}

std::optional<BinarizedBlobComponent2D> estimateCompanionBlobFromLocalSearchRoi(
  const cv::Mat &mask,
  const cv::Mat &search_area_mask,
  const cv::Point2f &expected_center_px,
  int label,
  const cv::Point &image_offset_px = cv::Point(0, 0))
{
  if (mask.empty() || mask.type() != CV_8UC1 || search_area_mask.empty() ||
    search_area_mask.size() != mask.size() || search_area_mask.type() != CV_8UC1)
  {
    return std::nullopt;
  }

  cv::Mat constrained_mask;
  cv::bitwise_and(mask, search_area_mask, constrained_mask);
  if (cv::countNonZero(constrained_mask) < 8)
  {
    return std::nullopt;
  }

  std::optional<BinarizedBlobComponent2D> best_component;
  double best_center_distance_px = std::numeric_limits<double>::infinity();
  std::size_t best_pixel_count = 0;
  for (const auto &local_component : extractConnectedBlobComponents(constrained_mask))
  {
    if (local_component.pixels.size() < 8)
    {
      continue;
    }

    std::vector<cv::Point> global_pixels;
    global_pixels.reserve(local_component.pixels.size());
    for (const auto &pixel : local_component.pixels)
    {
      global_pixels.emplace_back(pixel.x + image_offset_px.x, pixel.y + image_offset_px.y);
    }

    auto global_component = buildBlobComponentFromPixels(global_pixels, label);
    if (!global_component.has_value())
    {
      continue;
    }

    const auto component_center = componentCenterPx(*global_component);
    if (!component_center.has_value())
    {
      continue;
    }

    const double center_distance_px = cv::norm(*component_center - expected_center_px);
    const std::size_t pixel_count = global_component->pixels.size();
    const bool should_replace =
      !best_component.has_value() ||
      pixel_count > best_pixel_count ||
      (pixel_count == best_pixel_count && center_distance_px < best_center_distance_px);
    if (should_replace)
    {
      best_center_distance_px = center_distance_px;
      best_pixel_count = pixel_count;
      best_component = std::move(*global_component);
    }
  }

  return best_component;
}

std::optional<cv::Rect> suspiciousMergedBlobSearchRect(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent)
{
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    return std::nullopt;
  }
  const std::vector<cv::Point> &reference_hull =
    isPairPoseReference(reference_blob) && !reference_blob.group_hull.empty()
    ? reference_blob.group_hull
    : reference_blob.hull;
  const int reference_area_px =
    isPairPoseReference(reference_blob) && reference_blob.group_area_px > 0
    ? reference_blob.group_area_px
    : reference_blob.area_px;
  if (reference_area_px <= 0 || reference_hull.size() < 3)
  {
    return std::nullopt;
  }

  const std::vector<BinarizedBlobComponent2D> components = extractConnectedBlobComponents(mask);
  if (components.empty())
  {
    return std::nullopt;
  }

  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);
  const double tolerance_ratio = static_cast<double>(clamped_tolerance_percent) / 100.0;
  const double max_area_ratio = 1.0 + tolerance_ratio;
  const double merged_min_area_ratio = std::max(1.25, max_area_ratio * 1.10);

  std::vector<cv::Point2f> reference_hull_f;
  reference_hull_f.reserve(reference_hull.size());
  for (const auto &point : reference_hull)
  {
    reference_hull_f.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }
  const cv::RotatedRect reference_rect = cv::minAreaRect(reference_hull_f);
  const int margin_px = std::max(
    10,
    static_cast<int>(std::lround(
        0.18 * std::max(
          static_cast<double>(reference_rect.size.width),
          static_cast<double>(reference_rect.size.height)))));

  const BinarizedBlobComponent2D *best_component = nullptr;
  double best_priority = -std::numeric_limits<double>::infinity();
  for (const auto &component : components)
  {
    const double area_ratio = static_cast<double>(component.area_px) /
      static_cast<double>(std::max(1, reference_area_px));
    if (area_ratio < merged_min_area_ratio)
    {
      continue;
    }
    const double priority = area_ratio;
    if (priority > best_priority)
    {
      best_priority = priority;
      best_component = &component;
    }
  }

  if (best_component == nullptr)
  {
    return std::nullopt;
  }

  const cv::Rect image_rect(0, 0, mask.cols, mask.rows);
  cv::Rect search_rect(
    std::max(0, best_component->bbox.x - margin_px),
    std::max(0, best_component->bbox.y - margin_px),
    best_component->bbox.width + (2 * margin_px),
    best_component->bbox.height + (2 * margin_px));
  search_rect &= image_rect;
  if (search_rect.width <= 1 || search_rect.height <= 1)
  {
    return std::nullopt;
  }

  return search_rect;
}

std::optional<DirectShapeFitResult2D> estimateDirectShapeFitMatch(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent,
  const cv::Point &image_offset_px,
  std::string *status_text)
{
  if (status_text != nullptr)
  {
    status_text->clear();
  }
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    if (status_text != nullptr)
    {
      *status_text = "Direct fit unavailable: pose mask unavailable";
    }
    return std::nullopt;
  }
  if (reference_blob.hull.size() < 3 || reference_blob.area_px <= 0)
  {
    if (status_text != nullptr)
    {
      *status_text = "Direct fit unavailable: reference shape missing";
    }
    return std::nullopt;
  }

  cv::Mat mask_f32;
  mask.convertTo(mask_f32, CV_32FC1, 1.0 / 255.0);

  std::vector<cv::Point2f> reference_hull_f;
  reference_hull_f.reserve(reference_blob.hull.size());
  for (const auto &point : reference_blob.hull)
  {
    reference_hull_f.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }

  const cv::RotatedRect reference_rect = cv::minAreaRect(reference_hull_f);
  if (reference_rect.size.width < 2.0F || reference_rect.size.height < 2.0F)
  {
    if (status_text != nullptr)
    {
      *status_text = "Direct fit unavailable: taught shape too small";
    }
    return std::nullopt;
  }

  const cv::Point2f reference_center = reference_rect.center;
  std::vector<cv::Point2f> centered_hull;
  centered_hull.reserve(reference_hull_f.size());
  for (const auto &point : reference_hull_f)
  {
    centered_hull.push_back(point - reference_center);
  }

  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);
  const double tolerance_ratio = static_cast<double>(clamped_tolerance_percent) / 100.0;
  const double min_scale = std::sqrt(std::max(0.01, 1.0 - tolerance_ratio));
  const double max_scale = std::sqrt(1.0 + tolerance_ratio);
  const int scale_steps = (max_scale - min_scale) > 0.08 ? 5 : 3;
  std::vector<double> scales;
  scales.reserve(static_cast<std::size_t>(scale_steps));
  for (int i = 0; i < scale_steps; ++i)
  {
    const double t = scale_steps <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(scale_steps - 1);
    scales.push_back(min_scale + (max_scale - min_scale) * t);
  }

  DirectShapeFitResult2D best_candidate;
  best_candidate.reference_center_px = reference_center;
  constexpr double kFillRatioTieTolerance = 0.02;
  constexpr double kScaleErrorTieTolerance = 0.02;
  const auto evaluate_candidate = [&](double angle_deg, double scale)
    {
      const double angle_rad = angle_deg * CV_PI / 180.0;
      const double cos_theta = std::cos(angle_rad);
      const double sin_theta = std::sin(angle_rad);

      std::vector<cv::Point2f> transformed_points;
      transformed_points.reserve(centered_hull.size());
      float min_x = std::numeric_limits<float>::infinity();
      float min_y = std::numeric_limits<float>::infinity();
      float max_x = -std::numeric_limits<float>::infinity();
      float max_y = -std::numeric_limits<float>::infinity();
      for (const auto &point : centered_hull)
      {
        const float x = static_cast<float>(scale * (
            static_cast<double>(point.x) * cos_theta -
            static_cast<double>(point.y) * sin_theta));
        const float y = static_cast<float>(scale * (
            static_cast<double>(point.x) * sin_theta +
            static_cast<double>(point.y) * cos_theta));
        transformed_points.emplace_back(x, y);
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
      }

      if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y))
      {
        return;
      }

      const int margin_px = std::max(
        6,
        static_cast<int>(std::lround(0.12 * scale * std::max(
            static_cast<double>(reference_rect.size.width),
            static_cast<double>(reference_rect.size.height)))));
      const cv::Point2f offset(-min_x + static_cast<float>(margin_px), -min_y + static_cast<float>(margin_px));
      const int template_width = static_cast<int>(std::ceil(max_x - min_x)) + 1 + 2 * margin_px;
      const int template_height = static_cast<int>(std::ceil(max_y - min_y)) + 1 + 2 * margin_px;
      if (template_width < 8 || template_height < 8)
      {
        return;
      }
      if (template_width >= mask.cols || template_height >= mask.rows)
      {
        return;
      }

      std::vector<cv::Point2f> polygon_points;
      polygon_points.reserve(transformed_points.size());
      std::vector<cv::Point> polygon_pixels;
      polygon_pixels.reserve(transformed_points.size());
      for (const auto &point : transformed_points)
      {
        const cv::Point2f image_point = point + offset;
        polygon_points.push_back(image_point);
        polygon_pixels.emplace_back(
          static_cast<int>(std::lround(image_point.x)),
          static_cast<int>(std::lround(image_point.y)));
      }

      cv::Mat template_mask(template_height, template_width, CV_8UC1, cv::Scalar(0));
      const std::vector<std::vector<cv::Point>> polygons{polygon_pixels};
      cv::fillPoly(template_mask, polygons, cv::Scalar(255));
      const int template_area_px = cv::countNonZero(template_mask);
      if (template_area_px < 24)
      {
        return;
      }

      cv::Mat template_mask_f32;
      template_mask.convertTo(template_mask_f32, CV_32FC1, 1.0 / 255.0);
      cv::Mat response;
      cv::matchTemplate(mask_f32, template_mask_f32, response, cv::TM_SQDIFF_NORMED);
      double min_value = 0.0;
      cv::Point min_location;
      cv::minMaxLoc(response, &min_value, nullptr, &min_location, nullptr);
      if (!std::isfinite(min_value))
      {
        return;
      }

      const cv::Rect roi(min_location.x, min_location.y, template_mask.cols, template_mask.rows);
      if (roi.x < 0 || roi.y < 0 || roi.x + roi.width > mask.cols || roi.y + roi.height > mask.rows)
      {
        return;
      }
      cv::Mat overlap_mask;
      cv::bitwise_and(mask(roi), template_mask, overlap_mask);
      const double fill_ratio = static_cast<double>(cv::countNonZero(overlap_mask)) /
        static_cast<double>(std::max(1, template_area_px));
      const double score = 1.0 - min_value;
      const double scale_error = std::fabs(std::log(std::max(scale, 1e-3)));
      const double area_error = std::fabs((scale * scale) - 1.0);
      const double best_scale_error = std::fabs(std::log(std::max(best_candidate.scale, 1e-3)));
      const double best_area_error = std::fabs((best_candidate.scale * best_candidate.scale) - 1.0);
      const bool should_replace =
        !std::isfinite(best_candidate.score) ||
        (fill_ratio > best_candidate.fill_ratio + kFillRatioTieTolerance) ||
        (std::fabs(fill_ratio - best_candidate.fill_ratio) <= kFillRatioTieTolerance &&
        (
          (scale_error + kScaleErrorTieTolerance < best_scale_error) ||
          (std::fabs(scale_error - best_scale_error) <= kScaleErrorTieTolerance &&
          area_error + 1e-6 < best_area_error) ||
          (std::fabs(scale_error - best_scale_error) <= kScaleErrorTieTolerance &&
          std::fabs(area_error - best_area_error) <= 1e-6 &&
          score > best_candidate.score)
        ));
      if (should_replace)
      {
        best_candidate.score = score;
        best_candidate.fill_ratio = fill_ratio;
        best_candidate.angle_deg = angle_deg;
        best_candidate.scale = scale;
        best_candidate.transform_offset_px =
          offset + cv::Point2f(
          static_cast<float>(min_location.x + image_offset_px.x),
          static_cast<float>(min_location.y + image_offset_px.y));
        best_candidate.polygon_points.clear();
        best_candidate.polygon_points.reserve(polygon_points.size());
        for (const auto &point : polygon_points)
        {
          best_candidate.polygon_points.push_back(
            point + cv::Point2f(
              static_cast<float>(min_location.x + image_offset_px.x),
              static_cast<float>(min_location.y + image_offset_px.y)));
        }
      }
    };

  constexpr double kCoarseAngleStepDeg = 4.0;
  for (const double scale : scales)
  {
    for (double angle_deg = 0.0; angle_deg < 180.0; angle_deg += kCoarseAngleStepDeg)
    {
      evaluate_candidate(angle_deg, scale);
    }
  }

  if (std::isfinite(best_candidate.score))
  {
    const double refine_scale_min = std::max(min_scale, best_candidate.scale * 0.97);
    const double refine_scale_max = std::min(max_scale, best_candidate.scale * 1.03);
    for (int i = 0; i < 3; ++i)
    {
      const double t = (i == 0) ? 0.0 : (i == 1 ? 0.5 : 1.0);
      const double refine_scale = refine_scale_min + (refine_scale_max - refine_scale_min) * t;
      for (double angle_deg = best_candidate.angle_deg - kCoarseAngleStepDeg;
           angle_deg <= best_candidate.angle_deg + kCoarseAngleStepDeg;
           angle_deg += 1.0)
      {
        double normalized_angle_deg = std::fmod(angle_deg, 180.0);
        if (normalized_angle_deg < 0.0)
        {
          normalized_angle_deg += 180.0;
        }
        evaluate_candidate(normalized_angle_deg, refine_scale);
      }
    }
  }

  if (!std::isfinite(best_candidate.score) || best_candidate.polygon_points.size() < 3)
  {
    if (status_text != nullptr)
    {
      *status_text = "Direct fit unavailable: no taught-shape fit in merged mask";
    }
    return std::nullopt;
  }

  const double taught_fill_ratio = std::isfinite(reference_blob.fill_ratio)
    ? std::clamp(reference_blob.fill_ratio, 0.0, 1.0)
    : 0.0;
  if (taught_fill_ratio > 0.0)
  {
    const double min_fill_ratio = std::clamp(
      taught_fill_ratio * (1.0 - tolerance_ratio),
      0.0,
      1.0);
    if (best_candidate.fill_ratio + 1e-6 < min_fill_ratio)
    {
      if (status_text != nullptr)
      {
        *status_text = cv::format(
          "Direct fit unavailable: shape fill %.0f%% below taught min %.0f%%",
          best_candidate.fill_ratio * 100.0,
          min_fill_ratio * 100.0);
      }
      return std::nullopt;
    }
  }

  double final_overlap_ratio = 0.0;
  best_candidate.pixels = extractMaskedPolygonPixels(
    mask,
    best_candidate.polygon_points,
    image_offset_px,
    false,
    &final_overlap_ratio);
  if (best_candidate.pixels.size() < 12 || final_overlap_ratio < 0.05)
  {
    if (status_text != nullptr)
    {
      *status_text = "Direct fit unavailable: fitted shape has insufficient live mask overlap";
    }
    return std::nullopt;
  }
  if (status_text != nullptr)
  {
    *status_text = cv::format(
      "Direct fit: score %.2f angle %.0f deg scale %.2fx",
      best_candidate.score,
      best_candidate.angle_deg,
      best_candidate.scale);
  }
  return best_candidate;
}

std::optional<BinarizedPoseEstimate2D> estimatePoseFromDirectShapeFit(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent,
  const cv::Point &image_offset_px = cv::Point(0, 0),
  std::string *status_text = nullptr,
  std::optional<BinarizedPoseEstimate2D::BlobPose2D> *debug_preview_pose = nullptr,
  std::vector<cv::Point> *debug_first_blob_pixels = nullptr)
{
  if (debug_preview_pose != nullptr)
  {
    debug_preview_pose->reset();
  }
  if (debug_first_blob_pixels != nullptr)
  {
    debug_first_blob_pixels->clear();
  }
  const auto direct_fit = estimateDirectShapeFitMatch(
    mask,
    reference_blob,
    blob_tolerance_percent,
    image_offset_px,
    status_text);
  if (!direct_fit.has_value())
  {
    return std::nullopt;
  }
  if (debug_first_blob_pixels != nullptr)
  {
    *debug_first_blob_pixels = direct_fit->pixels;
  }

  const std::optional<BinarizedPoseEstimate2D::BlobPose2D> blob_pose = buildBlobPoseFromPolygon(
    direct_fit->polygon_points,
    1);
  if (!blob_pose.has_value())
  {
    if (status_text != nullptr)
    {
      *status_text = "Direct fit unavailable: fitted pose geometry invalid";
    }
    return std::nullopt;
  }

  if (debug_preview_pose != nullptr)
  {
    BinarizedPoseEstimate2D::BlobPose2D preview_pose = *blob_pose;
    preview_pose.pixels = direct_fit->pixels;
    *debug_preview_pose = std::move(preview_pose);
  }

  BinarizedPoseEstimate2D estimate;
  estimate.matched_blob_count = 1;
  BinarizedPoseEstimate2D::BlobPose2D fitted_blob_pose = *blob_pose;
  fitted_blob_pose.pixels = direct_fit->pixels;
  estimate.blob_poses.push_back(std::move(fitted_blob_pose));
  return estimate;
}

bool recoveredPairPoseAreaMatchesReference(
  const BinarizedPoseEstimate2D::BlobPose2D &candidate_pose,
  const PoseBlobReference2D &pair_reference,
  int blob_tolerance_percent)
{
  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);
  const double tolerance_ratio = static_cast<double>(clamped_tolerance_percent) / 100.0;
  const double min_area_ratio = std::max(0.01, 1.0 - tolerance_ratio);
  const double max_area_ratio = 1.0 + tolerance_ratio;
  const double candidate_area_px = fittedRectAreaPx(candidate_pose.hull_points);
  const double area_ratio = candidate_area_px /
    static_cast<double>(std::max(
        1,
        pair_reference.group_area_px > 0 ? pair_reference.group_area_px : pair_reference.area_px));
  return
    std::isfinite(candidate_area_px) &&
    std::isfinite(area_ratio) &&
    area_ratio >= min_area_ratio &&
    area_ratio <= max_area_ratio;
}

bool recoveredPairPoseMatchesReference(
  const BinarizedPoseEstimate2D::BlobPose2D &candidate_pose,
  const PoseBlobReference2D &pair_reference,
  int blob_tolerance_percent)
{
  const double layout_error = pairLayoutErrorNormalized(candidate_pose, pair_reference);
  return
    recoveredPairPoseAreaMatchesReference(candidate_pose, pair_reference, blob_tolerance_percent) &&
    std::isfinite(layout_error) &&
    layout_error <= pairLayoutToleranceNormalized(blob_tolerance_percent);
}

double companionPredictionErrorNormalized(
  const BinarizedBlobComponent2D &anchor_component,
  const BinarizedBlobComponent2D &companion_component,
  const CompanionPredictionBranch2D &prediction_branch)
{
  if (!prediction_branch.has_center_norm)
  {
    return std::numeric_limits<double>::infinity();
  }

  const auto anchor_pose = buildBlobPoseFromComponent(anchor_component);
  const auto companion_center = componentCenterPx(companion_component);
  if (!anchor_pose.has_value() || !companion_center.has_value())
  {
    return std::numeric_limits<double>::infinity();
  }

  const auto companion_center_norm = normalizedPointInBlobPoseFrame(*anchor_pose, *companion_center);
  if (!companion_center_norm.has_value())
  {
    return std::numeric_limits<double>::infinity();
  }

  return cv::norm(*companion_center_norm - prediction_branch.center_norm);
}


std::optional<BinarizedBlobComponent2D> findCompanionBlobGuidedByAnchor(
  const cv::Mat &mask,
  const BinarizedBlobComponent2D &anchor_component,
  const PoseBlobReference2D &pair_reference,
  const CompanionPredictionBranch2D &prediction_branch,
  int blob_tolerance_percent,
  const cv::Point &image_offset_px = cv::Point(0, 0))
{
  if (mask.empty() || mask.type() != CV_8UC1 || pair_reference.companion_area_px <= 0)
  {
    return std::nullopt;
  }

  struct CompanionCandidate
  {
    BinarizedBlobComponent2D component;
    double prediction_error_norm {std::numeric_limits<double>::infinity()};
    double prediction_distance_px {std::numeric_limits<double>::infinity()};
    double area_error {std::numeric_limits<double>::infinity()};
    std::size_t pixel_count {0};
  };

  std::vector<CompanionCandidate> candidates;
  const auto anchor_center = componentCenterPx(anchor_component);
  for (const auto &local_component : extractConnectedBlobComponents(mask))
  {
    if (local_component.pixels.size() < 8)
    {
      continue;
    }

    std::vector<cv::Point> global_pixels;
    global_pixels.reserve(local_component.pixels.size());
    for (const auto &pixel : local_component.pixels)
    {
      global_pixels.emplace_back(pixel.x + image_offset_px.x, pixel.y + image_offset_px.y);
    }

    auto global_component = buildBlobComponentFromPixels(global_pixels, 2);
    if (!global_component.has_value())
    {
      continue;
    }

    const auto companion_center = componentCenterPx(*global_component);
    if (!companion_center.has_value())
    {
      continue;
    }
    if (anchor_center.has_value() && cv::norm(*companion_center - *anchor_center) < 3.0)
    {
      continue;
    }

    if (!componentMatchesReferenceBlobArea(
        *global_component,
        pair_reference.companion_area_px,
        blob_tolerance_percent))
    {
      continue;
    }

    const auto pair_pose = buildPairBlobPoseFromComponents(anchor_component, *global_component);
    if (!pair_pose.has_value())
    {
      continue;
    }
    if (!recoveredPairPoseAreaMatchesReference(*pair_pose, pair_reference, blob_tolerance_percent))
    {
      continue;
    }

    const double prediction_error_norm = companionPredictionErrorNormalized(
      anchor_component,
      *global_component,
      prediction_branch);
    if (
      !std::isfinite(prediction_error_norm) ||
      prediction_error_norm > pairLayoutToleranceNormalized(blob_tolerance_percent))
    {
      continue;
    }

    const double area_ratio = static_cast<double>(global_component->area_px) /
      static_cast<double>(std::max(1, pair_reference.companion_area_px));
    CompanionCandidate candidate;
    candidate.component = std::move(*global_component);
    candidate.prediction_error_norm = prediction_error_norm;
    candidate.prediction_distance_px = cv::norm(*companion_center - prediction_branch.center);
    candidate.area_error = std::fabs(1.0 - area_ratio);
    candidate.pixel_count = candidate.component.pixels.size();
    candidates.push_back(std::move(candidate));
  }

  if (candidates.empty())
  {
    return std::nullopt;
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const CompanionCandidate &lhs, const CompanionCandidate &rhs)
    {
      if (std::fabs(lhs.prediction_error_norm - rhs.prediction_error_norm) > 1e-6)
      {
        return lhs.prediction_error_norm < rhs.prediction_error_norm;
      }
      if (std::fabs(lhs.prediction_distance_px - rhs.prediction_distance_px) > 1e-6)
      {
        return lhs.prediction_distance_px < rhs.prediction_distance_px;
      }
      if (std::fabs(lhs.area_error - rhs.area_error) > 1e-6)
      {
        return lhs.area_error < rhs.area_error;
      }
      return lhs.pixel_count > rhs.pixel_count;
    });
  return candidates.front().component;
}

std::optional<BinarizedPoseEstimate2D::BlobPose2D> buildPairPoseFromDirectShapeFit(
  const DirectShapeFitResult2D &reference_fit,
  const PoseBlobReference2D &pair_reference,
  int blob_tolerance_percent,
  const cv::Mat &mask,
  const cv::Point &image_offset_px = cv::Point(0, 0),
  std::string *failure_reason = nullptr,
  PairDirectShapeFitDebug2D *debug_info = nullptr,
  const cv::Mat *companion_search_mask = nullptr,
  cv::Point companion_search_offset_px = cv::Point(0, 0))
{
  if (failure_reason != nullptr)
  {
    failure_reason->clear();
  }
  if (debug_info != nullptr)
  {
    *debug_info = PairDirectShapeFitDebug2D{};
  }
  if (
    (pair_reference.group_hull.empty() ? pair_reference.hull : pair_reference.group_hull).size() < 3 ||
    pair_reference.member_centers_norm.size() < 2)
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: pair group reference missing";
    }
    return std::nullopt;
  }

  std::vector<cv::Point2f> reference_anchor_hull;
  reference_anchor_hull.reserve(pair_reference.anchor_hull.size());
  for (const auto &point : pair_reference.anchor_hull)
  {
    reference_anchor_hull.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }

  const auto reference_anchor_pose = buildBlobPoseFromPolygon(reference_anchor_hull, 1);
  const auto detected_anchor_pose = buildBlobPoseFromPolygon(reference_fit.polygon_points, 1);
  if (!reference_anchor_pose.has_value() || !detected_anchor_pose.has_value())
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: reference blob pose frame invalid";
    }
    return std::nullopt;
  }

  std::optional<cv::Point2f> reference_companion_center;
  std::vector<cv::Point2f> reference_companion_hull;
  reference_companion_hull.reserve(pair_reference.companion_hull.size());
  for (const auto &point : pair_reference.companion_hull)
  {
    reference_companion_hull.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }
  if (const auto companion_pose = buildBlobPoseFromPolygon(reference_companion_hull, 2); companion_pose.has_value())
  {
    reference_companion_center = poseBlobCenterPx(*companion_pose);
  }

  if (pair_reference.companion_hull.size() < 3)
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: auxiliary blob shape missing; re-save item";
    }
    return std::nullopt;
  }

  const cv::Point2f anchor_center = polygonCenterPx(reference_fit.polygon_points);
  const cv::Point2f reference_anchor_center = poseBlobCenterPx(*reference_anchor_pose);
  const auto reference_anchor_center_norm = normalizedPointInBlobPoseFrame(
    *reference_anchor_pose,
    reference_anchor_center);
  const cv::Point2f reference_companion_center_px = reference_companion_center.value_or(
    polygonCenterPx(reference_companion_hull));
  const auto reference_companion_center_norm = normalizedPointInBlobPoseFrame(
    *reference_anchor_pose,
    reference_companion_center_px);
  if (!reference_anchor_center_norm.has_value() || !reference_companion_center_norm.has_value())
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: companion prediction frame invalid";
    }
    return std::nullopt;
  }

  constexpr double companion_spacing_scale = 1.0;
  constexpr double companion_inflate_scale = 1.25;
  constexpr double companion_search_radius_expand_scale = 1.44;

  std::vector<CompanionPredictionBranch2D> prediction_branches;
  const auto rotate_anchor_frame_point =
    [&](const cv::Point2f &normalized_point, double angle_offset_deg)
    {
      if (std::fabs(angle_offset_deg) < 1e-6)
      {
        return normalized_point;
      }

      const double angle_rad = angle_offset_deg * CV_PI / 180.0;
      const double cos_theta = std::cos(angle_rad);
      const double sin_theta = std::sin(angle_rad);
      const cv::Point2f delta = normalized_point - *reference_anchor_center_norm;
      return *reference_anchor_center_norm + cv::Point2f(
        static_cast<float>(
          static_cast<double>(delta.x) * cos_theta - static_cast<double>(delta.y) * sin_theta),
        static_cast<float>(
          static_cast<double>(delta.x) * sin_theta + static_cast<double>(delta.y) * cos_theta));
    };

  const auto build_prediction_branch = [&](double angle_offset_deg)
    {
      CompanionPredictionBranch2D branch;
      branch.angle_offset_deg = angle_offset_deg;
      branch.center_norm = rotate_anchor_frame_point(*reference_companion_center_norm, angle_offset_deg);
      branch.has_center_norm = true;
      branch.expected_hull.reserve(reference_companion_hull.size());
      for (const auto &point : reference_companion_hull)
      {
        if (const auto normalized_point = normalizedPointInBlobPoseFrame(*reference_anchor_pose, point);
            normalized_point.has_value())
        {
          const cv::Point2f branch_point_norm =
            rotate_anchor_frame_point(*normalized_point, angle_offset_deg);
          if (const auto detected_point = pointFromNormalizedBlobPoseFrame(*detected_anchor_pose, branch_point_norm);
              detected_point.has_value())
          {
            branch.expected_hull.push_back(*detected_point);
          }
        }
      }
      if (branch.expected_hull.size() < 3)
      {
        return branch;
      }

      if (const auto detected_center = pointFromNormalizedBlobPoseFrame(
          *detected_anchor_pose,
          branch.center_norm);
        detected_center.has_value())
      {
        branch.center = *detected_center;
      }
      else
      {
        branch.center = polygonCenterPx(branch.expected_hull);
      }
      cv::Point2f companion_delta = branch.center - anchor_center;
      if (companion_spacing_scale > 1.0)
      {
        const cv::Point2f stretched_companion_center =
          anchor_center + static_cast<float>(companion_spacing_scale) * companion_delta;
        const cv::Point2f center_shift = stretched_companion_center - branch.center;
        branch.center = stretched_companion_center;
        for (auto &point : branch.expected_hull)
        {
          point += center_shift;
        }
        companion_delta = branch.center - anchor_center;
      }

      branch.search_hull = scalePolygonAroundCenter(
        branch.expected_hull,
        branch.center,
        companion_inflate_scale);
      const double companion_spacing_px = std::sqrt(static_cast<double>(companion_delta.dot(companion_delta)));
      const cv::RotatedRect search_rect = cv::minAreaRect(branch.search_hull);
      const double search_extent_px = static_cast<double>(std::max(
          search_rect.size.width,
          search_rect.size.height));
      const int full_search_radius_px = std::max(
        18,
        static_cast<int>(std::lround(
            0.22 * companion_spacing_px +
            0.30 * std::max(1.0, search_extent_px))));
      const int base_search_radius_px = std::max(8, full_search_radius_px / 2);
      branch.search_radius_px = std::max(
        8,
        static_cast<int>(std::lround(
            static_cast<double>(base_search_radius_px) * companion_search_radius_expand_scale)));
      return branch;
    };

  const auto add_prediction_branch = [&](double angle_offset_deg)
    {
      CompanionPredictionBranch2D branch = build_prediction_branch(angle_offset_deg);
      if (branch.expected_hull.size() >= 3 && branch.has_center_norm && branch.search_hull.size() >= 3)
      {
        prediction_branches.push_back(std::move(branch));
      }
    };
  add_prediction_branch(0.0);
  add_prediction_branch(180.0);
  if (prediction_branches.empty())
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: companion prediction geometry invalid";
    }
    return std::nullopt;
  }

  if (debug_info != nullptr && !prediction_branches.empty())
  {
    debug_info->predicted_companion_hull = prediction_branches.front().search_hull;
    debug_info->predicted_companion_center = prediction_branches.front().center;
    debug_info->search_radius_px = prediction_branches.front().search_radius_px;
    debug_info->companion_searches.clear();
    debug_info->companion_searches.reserve(prediction_branches.size());
    for (const auto &branch : prediction_branches)
    {
      CompanionSearchDebug2D search_debug;
      search_debug.hull = branch.search_hull;
      search_debug.center = branch.center;
      search_debug.radius_px = branch.search_radius_px;
      search_debug.angle_offset_deg = branch.angle_offset_deg;
      debug_info->companion_searches.push_back(std::move(search_debug));
    }
  }

  const auto anchor_component = buildBlobComponentFromPixels(reference_fit.pixels, 1);
  if (!anchor_component.has_value())
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: fitted reference blob pixels invalid";
    }
    return std::nullopt;
  }

  const cv::Mat &companion_detection_mask =
    (companion_search_mask != nullptr && !companion_search_mask->empty())
    ? *companion_search_mask
    : mask;
  const cv::Point companion_detection_offset_px =
    (companion_search_mask != nullptr && !companion_search_mask->empty())
    ? companion_search_offset_px
    : image_offset_px;

  std::optional<BinarizedBlobComponent2D> companion_component;
  const auto accept_companion_candidate = [&, debug_info](
      const std::optional<BinarizedBlobComponent2D> &candidate_component,
      const CompanionPredictionBranch2D &prediction_branch,
      int validation_tolerance_percent) -> bool
    {
      if (!candidate_component.has_value())
      {
        return false;
      }

      const auto searched_pair_pose = buildPairBlobPoseFromComponents(
        *anchor_component,
        *candidate_component);
      if (!searched_pair_pose.has_value())
      {
        return false;
      }
      if (!componentMatchesReferenceBlobArea(
          *candidate_component,
          pair_reference.companion_area_px,
          validation_tolerance_percent))
      {
        return false;
      }
      if (!recoveredPairPoseAreaMatchesReference(
          *searched_pair_pose,
          pair_reference,
          validation_tolerance_percent))
      {
        return false;
      }
      const double prediction_error_norm = companionPredictionErrorNormalized(
        *anchor_component,
        *candidate_component,
        prediction_branch);
      if (
        !std::isfinite(prediction_error_norm) ||
        prediction_error_norm > pairLayoutToleranceNormalized(validation_tolerance_percent))
      {
        return false;
      }

      companion_component = *candidate_component;
      if (debug_info != nullptr)
      {
        debug_info->predicted_companion_hull = prediction_branch.search_hull;
        debug_info->predicted_companion_center = prediction_branch.center;
        debug_info->search_radius_px = prediction_branch.search_radius_px;
      }
      return true;
    };

  for (const auto &prediction_branch : prediction_branches)
  {
    const auto guided_component = findCompanionBlobGuidedByAnchor(
      companion_detection_mask,
      *anchor_component,
      pair_reference,
      prediction_branch,
      blob_tolerance_percent,
      companion_detection_offset_px);
    if (accept_companion_candidate(guided_component, prediction_branch, blob_tolerance_percent))
    {
      break;
    }

    const std::vector<cv::Point2f> &refine_polygon = prediction_branch.expected_hull.empty()
      ? prediction_branch.search_hull
      : prediction_branch.expected_hull;
    const int refine_search_radius_px = std::max(prediction_branch.search_radius_px, 24);
    const auto refined_component = refineBlobComponentFromPredictedPolygon(
      companion_detection_mask,
      refine_polygon,
      2,
      refine_search_radius_px,
      companion_detection_offset_px,
      8,
      0.03);
    const int fallback_validation_tolerance_percent = std::max(blob_tolerance_percent, 35);
    if (accept_companion_candidate(
        refined_component,
        prediction_branch,
        fallback_validation_tolerance_percent))
    {
      break;
    }
  }

  if (!companion_component.has_value())
  {
    if (failure_reason != nullptr)
    {
      *failure_reason =
        "Pair direct fit unavailable: real companion blob not detected near taught pair offset or 180 branch";
    }
    return std::nullopt;
  }

  const auto live_pair_pose = buildPairBlobPoseFromComponents(*anchor_component, *companion_component);
  if (!live_pair_pose.has_value())
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = "Pair direct fit unavailable: recovered companion blob did not form a valid pair";
    }
    return std::nullopt;
  }

  return live_pair_pose;
}

std::optional<BinarizedPoseEstimate2D> estimatePairPoseFromDirectShapeFit(
  const cv::Mat &mask,
  const PoseBlobReference2D &pair_reference,
  int blob_tolerance_percent,
  const cv::Point &image_offset_px = cv::Point(0, 0),
  std::string *status_text = nullptr,
  std::optional<BinarizedPoseEstimate2D::BlobPose2D> *debug_anchor_pose = nullptr,
  std::optional<PairDirectShapeFitDebug2D> *debug_pair_info = nullptr,
  const cv::Mat *companion_search_mask = nullptr,
  cv::Point companion_search_offset_px = cv::Point(0, 0),
  std::vector<cv::Point> *debug_first_blob_pixels = nullptr)
{
  if (status_text != nullptr)
  {
    status_text->clear();
  }
  if (debug_anchor_pose != nullptr)
  {
    debug_anchor_pose->reset();
  }
  if (debug_pair_info != nullptr)
  {
    debug_pair_info->reset();
  }
  if (debug_first_blob_pixels != nullptr)
  {
    debug_first_blob_pixels->clear();
  }
  if (pair_reference.anchor_hull.size() < 3)
  {
    if (status_text != nullptr)
    {
      *status_text = "Pair direct fit unavailable: reference blob shape missing; re-save item";
    }
    return std::nullopt;
  }

  PoseBlobReference2D anchor_reference;
  anchor_reference.mode = PoseTemplateMode2D::kSingle;
  anchor_reference.fill_ratio = pair_reference.fill_ratio;
  anchor_reference.hull = pair_reference.anchor_hull;
  anchor_reference.area_px = std::max(
    1,
    static_cast<int>(std::round(fittedRectAreaPx(anchor_reference.hull))));
  anchor_reference.aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(anchor_reference.hull));

  const auto direct_fit = estimateDirectShapeFitMatch(
    mask,
    anchor_reference,
    blob_tolerance_percent,
    image_offset_px,
    status_text);
  if (!direct_fit.has_value())
  {
    return std::nullopt;
  }
  if (debug_first_blob_pixels != nullptr)
  {
    *debug_first_blob_pixels = direct_fit->pixels;
  }

  if (debug_anchor_pose != nullptr)
  {
    if (auto preview_pose = buildBlobPoseFromPolygon(direct_fit->polygon_points, 1); preview_pose.has_value())
    {
      preview_pose->pixels = direct_fit->pixels;
      *debug_anchor_pose = std::move(*preview_pose);
    }
  }

  std::string pair_failure_reason;
  PairDirectShapeFitDebug2D pair_debug_info;
  const auto pair_pose = buildPairPoseFromDirectShapeFit(
    *direct_fit,
    pair_reference,
    blob_tolerance_percent,
    mask,
    image_offset_px,
    &pair_failure_reason,
    &pair_debug_info,
    companion_search_mask,
    companion_search_offset_px);
  if (
    debug_pair_info != nullptr &&
    (!pair_debug_info.predicted_companion_hull.empty() || pair_debug_info.search_radius_px > 0))
  {
    *debug_pair_info = pair_debug_info;
  }
  if (!pair_pose.has_value())
  {
    if (status_text != nullptr)
    {
      *status_text = pair_failure_reason.empty()
        ? "Pair direct fit unavailable: pair reconstruction failed"
        : pair_failure_reason;
    }
    return std::nullopt;
  }

  BinarizedPoseEstimate2D estimate;
  estimate.matched_blob_count = 1;
  estimate.blob_poses.push_back(*pair_pose);
  if (status_text != nullptr)
  {
    *status_text = cv::format(
      "Pair direct fit: score %.2f angle %.0f deg scale %.2fx",
      direct_fit->score,
      direct_fit->angle_deg,
      direct_fit->scale);
  }
  return estimate;
}

double pairLayoutToleranceNormalized(int blob_tolerance_percent)
{
  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);
  return std::clamp(
    0.04 + (0.004 * static_cast<double>(clamped_tolerance_percent)),
    0.08,
    0.22);
}

double pairLayoutErrorNormalized(
  const BinarizedPoseEstimate2D::BlobPose2D &candidate_pose,
  const PoseBlobReference2D &reference_blob)
{
  if (candidate_pose.member_centers_norm.size() < 2 || reference_blob.member_centers_norm.size() < 2)
  {
    return std::numeric_limits<double>::infinity();
  }

  const double anchor_error = cv::norm(candidate_pose.member_centers_norm[0] - reference_blob.member_centers_norm[0]);
  const double companion_error = cv::norm(candidate_pose.member_centers_norm[1] - reference_blob.member_centers_norm[1]);
  return std::max(anchor_error, companion_error);
}

std::optional<BinarizedPoseEstimate2D> estimateSingleBlobPoseFromBinarizedMask(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent,
  std::string *status_text = nullptr)
{
  if (status_text != nullptr)
  {
    status_text->clear();
  }
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    if (status_text != nullptr)
    {
      *status_text = "Pose mask unavailable";
    }
    return std::nullopt;
  }
  if (reference_blob.area_px <= 0 || reference_blob.hull.size() < 3)
  {
    if (status_text != nullptr)
    {
      *status_text = "Reference blob missing in profile";
    }
    return std::nullopt;
  }

  cv::Mat labels;
  std::vector<BinarizedBlobComponent2D> components = extractConnectedBlobComponents(mask, &labels);
  if (components.empty())
  {
    if (status_text != nullptr)
    {
      *status_text = "No blobs in final binarized mask";
    }
    return std::nullopt;
  }

  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);

  std::vector<const BinarizedBlobComponent2D *> matched_components;
  matched_components.reserve(components.size());
  for (const auto &component : components)
  {
    if (!componentMatchesReferenceBlobArea(
        component,
        reference_blob.area_px,
        blob_tolerance_percent))
    {
      continue;
    }

    matched_components.push_back(&component);
  }

  if (matched_components.empty())
  {
    if (status_text != nullptr)
    {
      *status_text = cv::format(
        "No single blobs matched ref size (edge +/- %d%%)",
        clamped_tolerance_percent);
    }
    return std::nullopt;
  }

  BinarizedPoseEstimate2D estimate;
  estimate.matched_blob_count = static_cast<int>(matched_components.size());
  estimate.blob_poses.reserve(matched_components.size());

  for (const auto *component : matched_components)
  {
    if (component == nullptr)
    {
      continue;
    }
    const std::optional<BinarizedPoseEstimate2D::BlobPose2D> blob_pose = buildBlobPoseFromComponent(*component);
    if (!blob_pose.has_value())
    {
      continue;
    }
    estimate.blob_poses.push_back(*blob_pose);
  }

  if (estimate.blob_poses.empty())
  {
    if (status_text != nullptr)
    {
      *status_text = "No valid per-blob pose from matched blobs";
    }
    return std::nullopt;
  }

  std::sort(
    estimate.blob_poses.begin(),
    estimate.blob_poses.end(),
    [](const BinarizedPoseEstimate2D::BlobPose2D &a, const BinarizedPoseEstimate2D::BlobPose2D &b)
    {
      if (std::fabs(a.origin.y - b.origin.y) > 1e-3F)
      {
        return a.origin.y < b.origin.y;
      }
      return a.origin.x < b.origin.x;
    });

  if (status_text != nullptr)
  {
    *status_text = "Matched single blobs: " + std::to_string(estimate.matched_blob_count) +
      " | poses: " + std::to_string(estimate.blob_poses.size()) +
      " | ref rect: " + std::to_string(reference_blob.area_px) + " px" +
      " | edge tol: +/-" + std::to_string(clamped_tolerance_percent) + "%";
  }
  return estimate;
}

std::optional<BinarizedPoseEstimate2D> estimatePairPoseFromBinarizedMask(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent,
  std::string *status_text = nullptr)
{
  if (status_text != nullptr)
  {
    status_text->clear();
  }
  const std::vector<cv::Point> &pair_reference_hull =
    reference_blob.group_hull.empty() ? reference_blob.hull : reference_blob.group_hull;
  const int pair_reference_area_px =
    reference_blob.group_area_px > 0 ? reference_blob.group_area_px : reference_blob.area_px;
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    if (status_text != nullptr)
    {
      *status_text = "Pose mask unavailable";
    }
    return std::nullopt;
  }
  if (
    pair_reference_area_px <= 0 ||
    pair_reference_hull.size() < 3 ||
    reference_blob.member_count != 2 ||
    reference_blob.member_centers_norm.size() < 2)
  {
    if (status_text != nullptr)
    {
      *status_text = "Pair reference missing in profile";
    }
    return std::nullopt;
  }

  std::vector<BinarizedBlobComponent2D> components = extractConnectedBlobComponents(mask);
  if (components.size() < 2)
  {
    if (status_text != nullptr)
    {
      *status_text = "Need at least 2 blobs in final binarized mask";
    }
    return std::nullopt;
  }

  const int clamped_tolerance_percent = std::clamp(
    blob_tolerance_percent,
    kBlobToleranceMinPercent,
    kBlobToleranceMaxPercent);
  const double tolerance_ratio = static_cast<double>(clamped_tolerance_percent) / 100.0;
  const double min_area_ratio = std::max(0.01, 1.0 - tolerance_ratio);
  const double max_area_ratio = 1.0 + tolerance_ratio;
  const double layout_tolerance_norm = pairLayoutToleranceNormalized(blob_tolerance_percent);

  struct PairCandidate
  {
    int first_index {0};
    int second_index {0};
    BinarizedPoseEstimate2D::BlobPose2D pose;
    double prediction_error {std::numeric_limits<double>::infinity()};
    double layout_error {std::numeric_limits<double>::infinity()};
    double area_error {std::numeric_limits<double>::infinity()};
  };

  const auto reference_companion_center_norm = referenceCompanionCenterInAnchorFrame(reference_blob);
  if (!reference_companion_center_norm.has_value())
  {
    if (status_text != nullptr)
    {
      *status_text = "Pair reference missing 2/2 prediction geometry";
    }
    return std::nullopt;
  }
  const double companion_prediction_tolerance_norm = layout_tolerance_norm;

  std::vector<PairCandidate> pair_candidates;
  for (std::size_t i = 0; i < components.size(); ++i)
  {
    const auto &anchor_component = components[i];
    if (!componentMatchesReferenceBlobArea(
        anchor_component,
        reference_blob.area_px,
        blob_tolerance_percent))
    {
      continue;
    }
    const auto anchor_pose = buildBlobPoseFromComponent(anchor_component);
    if (!anchor_pose.has_value())
    {
      continue;
    }
    const auto predicted_companion_center = pointFromNormalizedBlobPoseFrame(
      *anchor_pose,
      *reference_companion_center_norm);
    if (!predicted_companion_center.has_value())
    {
      continue;
    }

    for (std::size_t j = 0; j < components.size(); ++j)
    {
      if (i == j)
      {
        continue;
      }
      const auto &companion_component = components[j];
      if (!componentMatchesReferenceBlobArea(
          companion_component,
          reference_blob.companion_area_px,
          blob_tolerance_percent))
      {
        continue;
      }
      const auto companion_center = componentCenterPx(companion_component);
      if (!companion_center.has_value())
      {
        continue;
      }
      const auto companion_center_norm = normalizedPointInBlobPoseFrame(*anchor_pose, *companion_center);
      if (!companion_center_norm.has_value())
      {
        continue;
      }
      const double prediction_error = cv::norm(*companion_center_norm - *reference_companion_center_norm);
      if (!std::isfinite(prediction_error) || prediction_error > companion_prediction_tolerance_norm)
      {
        continue;
      }

      const auto pair_pose = buildPairBlobPoseFromComponents(anchor_component, companion_component);
      if (!pair_pose.has_value())
      {
        continue;
      }
      const double layout_error = pairLayoutErrorNormalized(*pair_pose, reference_blob);
      if (!std::isfinite(layout_error) || layout_error > layout_tolerance_norm)
      {
        continue;
      }

      const double candidate_area_px = fittedRectAreaPx(pair_pose->hull_points);
      if (!std::isfinite(candidate_area_px) || candidate_area_px < 1.0)
      {
        continue;
      }
      const double area_ratio = candidate_area_px /
        static_cast<double>(std::max(1, pair_reference_area_px));
      if (area_ratio < min_area_ratio || area_ratio > max_area_ratio)
      {
        continue;
      }

      PairCandidate candidate;
      candidate.first_index = static_cast<int>(i);
      candidate.second_index = static_cast<int>(j);
      candidate.pose = *pair_pose;
      candidate.prediction_error = prediction_error;
      candidate.layout_error = layout_error;
      candidate.area_error = std::fabs(1.0 - area_ratio);
      pair_candidates.push_back(std::move(candidate));
    }
  }

  if (pair_candidates.empty())
  {
    if (status_text != nullptr)
    {
      *status_text = cv::format(
        "No 2-blob groups matched ref size/layout (edge +/- %d%%, layout <= %.2f)",
        clamped_tolerance_percent,
        layout_tolerance_norm);
    }
    return std::nullopt;
  }

  std::sort(
    pair_candidates.begin(),
    pair_candidates.end(),
    [](const PairCandidate &lhs, const PairCandidate &rhs)
    {
      if (std::fabs(lhs.prediction_error - rhs.prediction_error) > 1e-6)
      {
        return lhs.prediction_error < rhs.prediction_error;
      }
      if (std::fabs(lhs.layout_error - rhs.layout_error) > 1e-6)
      {
        return lhs.layout_error < rhs.layout_error;
      }
      if (std::fabs(lhs.area_error - rhs.area_error) > 1e-6)
      {
        return lhs.area_error < rhs.area_error;
      }
      if (lhs.first_index != rhs.first_index)
      {
        return lhs.first_index < rhs.first_index;
      }
      return lhs.second_index < rhs.second_index;
    });

  std::vector<bool> used_components(components.size(), false);
  BinarizedPoseEstimate2D estimate;
  for (const auto &candidate : pair_candidates)
  {
    if (used_components[static_cast<std::size_t>(candidate.first_index)] ||
      used_components[static_cast<std::size_t>(candidate.second_index)])
    {
      continue;
    }
    used_components[static_cast<std::size_t>(candidate.first_index)] = true;
    used_components[static_cast<std::size_t>(candidate.second_index)] = true;
    estimate.blob_poses.push_back(candidate.pose);
  }

  if (estimate.blob_poses.empty())
  {
    if (status_text != nullptr)
    {
      *status_text = "Matched pair candidates overlapped; no distinct 2-blob groups remained";
    }
    return std::nullopt;
  }

  estimate.matched_blob_count = static_cast<int>(estimate.blob_poses.size());
  std::sort(
    estimate.blob_poses.begin(),
    estimate.blob_poses.end(),
    [](const BinarizedPoseEstimate2D::BlobPose2D &a, const BinarizedPoseEstimate2D::BlobPose2D &b)
    {
      const cv::Point2f center_a = poseBlobCenterPx(a);
      const cv::Point2f center_b = poseBlobCenterPx(b);
      if (std::fabs(center_a.y - center_b.y) > 1e-3F)
      {
        return center_a.y < center_b.y;
      }
      return center_a.x < center_b.x;
    });

  if (status_text != nullptr)
  {
    *status_text = cv::format(
      "Matched 2-blob groups: %d | layout <= %.2f | ref rect %d px",
      estimate.matched_blob_count,
      layout_tolerance_norm,
      pair_reference_area_px);
  }
  return estimate;
}

std::optional<BinarizedPoseEstimate2D> estimatePoseFromBinarizedMask(
  const cv::Mat &mask,
  const PoseBlobReference2D &reference_blob,
  int blob_tolerance_percent,
  std::string *status_text = nullptr)
{
  if (reference_blob.mode == PoseTemplateMode2D::kPair || reference_blob.member_count >= 2)
  {
    return estimatePairPoseFromBinarizedMask(
      mask,
      reference_blob,
      blob_tolerance_percent,
      status_text);
  }

  return estimateSingleBlobPoseFromBinarizedMask(
    mask,
    reference_blob,
    blob_tolerance_percent,
    status_text);
}

int collapsePoseEstimateToSingleBlob(
  std::optional<BinarizedPoseEstimate2D> &pose_estimate,
  const std::optional<cv::Point> &preferred_pixel = std::nullopt)
{
  if (!pose_estimate.has_value() || pose_estimate->blob_poses.empty())
  {
    return 0;
  }

  auto &blob_poses = pose_estimate->blob_poses;
  const int original_match_count = std::max(
    pose_estimate->matched_blob_count,
    static_cast<int>(blob_poses.size()));

  auto best_it = blob_poses.begin();
  if (preferred_pixel.has_value())
  {
    const cv::Point2f preferred_point(
      static_cast<float>(preferred_pixel->x),
      static_cast<float>(preferred_pixel->y));
    best_it = std::min_element(
      blob_poses.begin(),
      blob_poses.end(),
      [&](const BinarizedPoseEstimate2D::BlobPose2D &lhs, const BinarizedPoseEstimate2D::BlobPose2D &rhs)
      {
        const cv::Point2f lhs_delta = lhs.origin - preferred_point;
        const cv::Point2f rhs_delta = rhs.origin - preferred_point;
        const double lhs_sq_distance = static_cast<double>(lhs_delta.dot(lhs_delta));
        const double rhs_sq_distance = static_cast<double>(rhs_delta.dot(rhs_delta));
        if (std::fabs(lhs_sq_distance - rhs_sq_distance) > 1e-6)
        {
          return lhs_sq_distance < rhs_sq_distance;
        }
        if (std::fabs(lhs.origin.y - rhs.origin.y) > 1e-3F)
        {
          return lhs.origin.y < rhs.origin.y;
        }
        return lhs.origin.x < rhs.origin.x;
      });
  }

  if (best_it != blob_poses.begin())
  {
    std::iter_swap(blob_poses.begin(), best_it);
  }
  blob_poses.resize(1);
  pose_estimate->matched_blob_count = 1;
  return original_match_count;
}

cv::Rect blobPoseBounds(const BinarizedPoseEstimate2D::BlobPose2D &blob_pose)
{
  std::vector<cv::Point> points;
  points.reserve(
    blob_pose.hull_points.size() +
    blob_pose.pixels.size() +
    blob_pose.corners.size() +
    blob_pose.anchor_pixels.size() +
    blob_pose.companion_pixels.size() +
    1);
  points.insert(points.end(), blob_pose.hull_points.begin(), blob_pose.hull_points.end());
  points.insert(points.end(), blob_pose.pixels.begin(), blob_pose.pixels.end());
  points.insert(points.end(), blob_pose.anchor_pixels.begin(), blob_pose.anchor_pixels.end());
  points.insert(points.end(), blob_pose.companion_pixels.begin(), blob_pose.companion_pixels.end());
  for (const auto &corner : blob_pose.corners)
  {
    points.emplace_back(
      static_cast<int>(std::lround(corner.x)),
      static_cast<int>(std::lround(corner.y)));
  }
  if (points.empty())
  {
    points.emplace_back(
      static_cast<int>(std::lround(blob_pose.origin.x)),
      static_cast<int>(std::lround(blob_pose.origin.y)));
  }
  return cv::boundingRect(points);
}

bool blobPosesOverlap(
  const BinarizedPoseEstimate2D::BlobPose2D &a,
  const BinarizedPoseEstimate2D::BlobPose2D &b)
{
  const cv::Rect bounds_a = blobPoseBounds(a);
  const cv::Rect bounds_b = blobPoseBounds(b);
  if (bounds_a.empty() || bounds_b.empty())
  {
    return false;
  }
  const cv::Rect intersection = bounds_a & bounds_b;
  if (intersection.empty())
  {
    return false;
  }
  const double intersection_area = static_cast<double>(intersection.area());
  const double smaller_area = static_cast<double>(std::max(1, std::min(bounds_a.area(), bounds_b.area())));
  return intersection_area >= 0.20 * smaller_area;
}

bool blobPoseOverlapsAnyAccepted(
  const BinarizedPoseEstimate2D::BlobPose2D &candidate,
  const std::vector<BinarizedPoseEstimate2D::BlobPose2D> &accepted_poses)
{
  return std::any_of(
    accepted_poses.begin(),
    accepted_poses.end(),
    [&](const BinarizedPoseEstimate2D::BlobPose2D &accepted)
    {
      return blobPosesOverlap(candidate, accepted);
    });
}

std::optional<ItemPose3D> estimateBlobPose3D(
  const BinarizedPoseEstimate2D::BlobPose2D &blob_pose,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info)
{
  if (blob_pose.corners.size() != 4)
  {
    return std::nullopt;
  }
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6)
  {
    return std::nullopt;
  }

  cv::Mat blob_depth_mask;
  if (!blob_pose.pixels.empty())
  {
    blob_depth_mask = buildPixelMask(depth_m.size(), blob_pose.pixels);
  }
  const cv::Mat *mask_ptr = blob_depth_mask.empty() ? nullptr : &blob_depth_mask;
  const std::optional<double> fallback_blob_depth = averageDepthFromPixels(depth_m, blob_pose.pixels);

  const auto camera_points = estimateItemCornerCameraPoints(
    blob_pose.corners,
    depth_m,
    camera_info,
    mask_ptr,
    fallback_blob_depth);
  if (!camera_points.has_value())
  {
    return std::nullopt;
  }

  const int origin_idx = lowerLeftCornerIndex(blob_pose.corners);
  if (origin_idx < 0)
  {
    return std::nullopt;
  }

  const int prev_idx = (origin_idx + 3) % 4;
  const int next_idx = (origin_idx + 1) % 4;

  const cv::Vec3d origin_corner = (*camera_points)[origin_idx];
  cv::Vec3d dir_a = (*camera_points)[prev_idx] - origin_corner;
  cv::Vec3d dir_b = (*camera_points)[next_idx] - origin_corner;
  const double len_a = vectorNorm(dir_a);
  const double len_b = vectorNorm(dir_b);
  if (len_a < 1e-9 || len_b < 1e-9)
  {
    return std::nullopt;
  }

  cv::Vec3d x_axis = len_a >= len_b ? dir_a : dir_b;
  cv::Vec3d y_axis = len_a >= len_b ? dir_b : dir_a;
  if (!normalizeVectorInPlace(x_axis))
  {
    return std::nullopt;
  }

  y_axis = y_axis - x_axis * x_axis.dot(y_axis);
  if (!normalizeVectorInPlace(y_axis))
  {
    return std::nullopt;
  }

  cv::Vec3d z_axis = x_axis.cross(y_axis);
  if (!normalizeVectorInPlace(z_axis))
  {
    return std::nullopt;
  }

  cv::Vec3d center(0.0, 0.0, 0.0);
  cv::Point2f hull_center_px(0.0F, 0.0F);
  for (const auto &corner : *camera_points)
  {
    center += corner;
  }
  for (const auto &corner_px : blob_pose.corners)
  {
    hull_center_px += corner_px;
  }
  center *= 0.25;
  hull_center_px *= 0.25F;
  if (blob_pose.member_count >= 2)
  {
    if (fallback_blob_depth.has_value())
    {
      center = projectPixelToCamera(hull_center_px, *fallback_blob_depth, camera_info);
    }
  }
  else
  {
    const auto robust_center = estimateBlobCenterFromPixels(blob_pose.pixels, depth_m, camera_info);
    if (robust_center.has_value())
    {
      center = *robust_center;
    }
  }
  cv::Vec3d pose_origin = center;
  if (blob_pose.member_count < 2 && blob_pose.has_custom_anchor)
  {
    const auto robust_anchor = estimateBlobCenterFromPixels(blob_pose.anchor_pixels, depth_m, camera_info);
    if (robust_anchor.has_value())
    {
      pose_origin = *robust_anchor;
    }
    else if (const auto anchor_depth = averageDepthAt(depth_m, blob_pose.anchor_point_px, 7, mask_ptr);
      anchor_depth.has_value())
    {
      pose_origin = projectPixelToCamera(blob_pose.anchor_point_px, *anchor_depth, camera_info);
    }
  }

  // Match tray_detect orientation convention for axis handedness.
  if (z_axis.dot(origin_corner) > 0.0)
  {
    y_axis *= -1.0;
    z_axis = x_axis.cross(y_axis);
    if (!normalizeVectorInPlace(z_axis))
    {
      return std::nullopt;
    }
  }

  ItemPose3D pose;
  pose.origin = pose_origin;
  pose.rotation = cv::Matx33d(
    x_axis[0], y_axis[0], z_axis[0],
    x_axis[1], y_axis[1], z_axis[1],
    x_axis[2], y_axis[2], z_axis[2]);
  return pose;
}

std::optional<ItemPose3D> estimateItemPose3D(
  const ItemEstimate &estimate,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info)
{
  const auto camera_points = estimateItemCornerCameraPoints(estimate.corners, depth_m, camera_info);
  if (!camera_points.has_value())
  {
    return std::nullopt;
  }

  const int origin_idx = lowerLeftCornerIndex(estimate.corners);
  if (origin_idx < 0)
  {
    return std::nullopt;
  }

  const cv::Vec3d origin = (*camera_points)[origin_idx];
  cv::Vec3d dir_a = (*camera_points)[(origin_idx + 3) % 4] - origin;
  cv::Vec3d dir_b = (*camera_points)[(origin_idx + 1) % 4] - origin;

  const double len_a = vectorNorm(dir_a);
  const double len_b = vectorNorm(dir_b);
  if (len_a < 1e-6 || len_b < 1e-6)
  {
    return std::nullopt;
  }

  cv::Vec3d x_axis = len_a >= len_b ? dir_a : dir_b;
  cv::Vec3d y_axis = len_a >= len_b ? dir_b : dir_a;

  x_axis *= (1.0 / vectorNorm(x_axis));
  y_axis = y_axis - x_axis * x_axis.dot(y_axis);
  const double y_norm = vectorNorm(y_axis);
  if (y_norm < 1e-6)
  {
    return std::nullopt;
  }
  y_axis *= (1.0 / y_norm);

  cv::Vec3d z_axis = x_axis.cross(y_axis);
  const double z_norm = vectorNorm(z_axis);
  if (z_norm < 1e-6)
  {
    return std::nullopt;
  }
  z_axis *= (1.0 / z_norm);

  if (z_axis.dot(origin) > 0.0)
  {
    y_axis *= -1.0;
    z_axis = x_axis.cross(y_axis);
    const double z_norm_flipped = vectorNorm(z_axis);
    if (z_norm_flipped < 1e-6)
    {
      return std::nullopt;
    }
    z_axis *= (1.0 / z_norm_flipped);
  }

  ItemPose3D pose;
  pose.origin = origin;
  pose.rotation = cv::Matx33d(
    x_axis[0], y_axis[0], z_axis[0],
    x_axis[1], y_axis[1], z_axis[1],
    x_axis[2], y_axis[2], z_axis[2]);
  return pose;
}

geometry_msgs::msg::Quaternion rotationToQuaternionMsg(const cv::Matx33d &rotation)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);

  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  if (trace > 0.0)
  {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (rotation(2, 1) - rotation(1, 2)) / s;
    qy = (rotation(0, 2) - rotation(2, 0)) / s;
    qz = (rotation(1, 0) - rotation(0, 1)) / s;
  }
  else if (rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2))
  {
    const double s = std::sqrt(1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)) * 2.0;
    qw = (rotation(2, 1) - rotation(1, 2)) / s;
    qx = 0.25 * s;
    qy = (rotation(0, 1) + rotation(1, 0)) / s;
    qz = (rotation(0, 2) + rotation(2, 0)) / s;
  }
  else if (rotation(1, 1) > rotation(2, 2))
  {
    const double s = std::sqrt(1.0 + rotation(1, 1) - rotation(0, 0) - rotation(2, 2)) * 2.0;
    qw = (rotation(0, 2) - rotation(2, 0)) / s;
    qx = (rotation(0, 1) + rotation(1, 0)) / s;
    qy = 0.25 * s;
    qz = (rotation(1, 2) + rotation(2, 1)) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + rotation(2, 2) - rotation(0, 0) - rotation(1, 1)) * 2.0;
    qw = (rotation(1, 0) - rotation(0, 1)) / s;
    qx = (rotation(0, 2) + rotation(2, 0)) / s;
    qy = (rotation(1, 2) + rotation(2, 1)) / s;
    qz = 0.25 * s;
  }

  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 1e-12)
  {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }

  quaternion.x = qx;
  quaternion.y = qy;
  quaternion.z = qz;
  quaternion.w = qw;
  return quaternion;
}

cv::Vec3d rotationToRpyDegrees(const cv::Matx33d &rotation)
{
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return cv::Vec3d(roll, pitch, yaw) * kRadiansToDegrees;
}

std::string formatPose6D(const ItemPose3D &pose)
{
  const cv::Vec3d xyz_mm = pose.origin * kMetersToMillimeters;
  const cv::Vec3d rpy_deg = rotationToRpyDegrees(pose.rotation);
  return cv::format(
    "X%+.1f Y%+.1f Z%+.1f | R%+.1f P%+.1f Y%+.1f",
    xyz_mm[0],
    xyz_mm[1],
    xyz_mm[2],
    rpy_deg[0],
    rpy_deg[1],
    rpy_deg[2]);
}

int sampleMaskAt(const cv::Mat &mask, const cv::Point2f &pt)
{
  const int x = static_cast<int>(std::round(pt.x));
  const int y = static_cast<int>(std::round(pt.y));
  if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows)
  {
    return -1;
  }
  return mask.at<unsigned char>(y, x) > 0 ? 1 : 0;
}

bool hasValidRoiPoints(const std::vector<cv::Point2f> &roi_points)
{
  return roi_points.size() >= 4;
}

bool isValidRoiBounds(const AxisAlignedRoiBounds &bounds)
{
  return bounds.right > bounds.left && bounds.bottom > bounds.top;
}

std::optional<AxisAlignedRoiBounds> roiBoundsFromSelection(const std::vector<cv::Point2f> &points)
{
  if (points.size() < 2)
  {
    return std::nullopt;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto &point : points)
  {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  const int left = static_cast<int>(std::round(min_x));
  const int top = static_cast<int>(std::round(min_y));
  const int right = static_cast<int>(std::round(max_x));
  const int bottom = static_cast<int>(std::round(max_y));
  AxisAlignedRoiBounds bounds{left, top, right, bottom};
  if (!isValidRoiBounds(bounds))
  {
    return std::nullopt;
  }
  return bounds;
}

std::vector<cv::Point2f> roiPointsFromBounds(const AxisAlignedRoiBounds &bounds)
{
  if (!isValidRoiBounds(bounds))
  {
    return {};
  }
  return {
    cv::Point2f(static_cast<float>(bounds.left), static_cast<float>(bounds.top)),
    cv::Point2f(static_cast<float>(bounds.right), static_cast<float>(bounds.top)),
    cv::Point2f(static_cast<float>(bounds.right), static_cast<float>(bounds.bottom)),
    cv::Point2f(static_cast<float>(bounds.left), static_cast<float>(bounds.bottom)),
  };
}

bool hasValidRoiRegions(const std::vector<AxisAlignedRoiBounds> &roi_regions)
{
  return std::any_of(
    roi_regions.begin(),
    roi_regions.end(),
    [](const AxisAlignedRoiBounds &bounds)
    {
      return isValidRoiBounds(bounds);
    });
}

std::optional<AxisAlignedRoiBounds> combinedRoiBounds(const std::vector<AxisAlignedRoiBounds> &roi_regions)
{
  bool found_valid_region = false;
  AxisAlignedRoiBounds combined;
  for (const auto &region : roi_regions)
  {
    if (!isValidRoiBounds(region))
    {
      continue;
    }
    if (!found_valid_region)
    {
      combined = region;
      found_valid_region = true;
      continue;
    }
    combined.left = std::min(combined.left, region.left);
    combined.top = std::min(combined.top, region.top);
    combined.right = std::max(combined.right, region.right);
    combined.bottom = std::max(combined.bottom, region.bottom);
  }

  if (!found_valid_region)
  {
    return std::nullopt;
  }
  return combined;
}

std::vector<cv::Point> roiPolygonForImage(const std::vector<cv::Point2f> &roi_points, const cv::Size &size)
{
  std::vector<cv::Point> polygon;
  if (!hasValidRoiPoints(roi_points) || size.width <= 0 || size.height <= 0)
  {
    return polygon;
  }

  polygon.reserve(roi_points.size());
  for (const auto &point : roi_points)
  {
    polygon.emplace_back(
      std::clamp(static_cast<int>(std::round(point.x)), 0, size.width - 1),
      std::clamp(static_cast<int>(std::round(point.y)), 0, size.height - 1));
  }
  return polygon;
}

cv::Mat buildRoiMask(const cv::Size &size, const std::vector<cv::Point2f> &roi_points)
{
  cv::Mat roi_mask(size, CV_8UC1, cv::Scalar(0));
  const auto polygon = roiPolygonForImage(roi_points, size);
  if (polygon.size() >= 3)
  {
    const std::vector<std::vector<cv::Point>> polygons{polygon};
    cv::fillPoly(roi_mask, polygons, cv::Scalar(255));
  }
  return roi_mask;
}

std::optional<AxisAlignedRoiBounds> roiBoundsForImage(
  const std::vector<cv::Point2f> &roi_points,
  const cv::Size &size)
{
  const auto polygon = roiPolygonForImage(roi_points, size);
  if (polygon.size() < 3)
  {
    return std::nullopt;
  }

  int min_x = size.width - 1;
  int min_y = size.height - 1;
  int max_x = 0;
  int max_y = 0;
  for (const auto &point : polygon)
  {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  if (max_x <= min_x || max_y <= min_y)
  {
    return std::nullopt;
  }

  return AxisAlignedRoiBounds{min_x, min_y, max_x, max_y};
}

std::optional<AxisAlignedRoiBounds> roiBoundsForImage(
  const std::vector<AxisAlignedRoiBounds> &roi_regions,
  const cv::Size &size)
{
  if (size.width <= 0 || size.height <= 0)
  {
    return std::nullopt;
  }

  bool found_valid_region = false;
  AxisAlignedRoiBounds combined;
  for (const auto &region : roi_regions)
  {
    if (!isValidRoiBounds(region))
    {
      continue;
    }

    AxisAlignedRoiBounds clamped{
      std::clamp(region.left, 0, size.width - 1),
      std::clamp(region.top, 0, size.height - 1),
      std::clamp(region.right, 0, size.width - 1),
      std::clamp(region.bottom, 0, size.height - 1),
    };
    if (!isValidRoiBounds(clamped))
    {
      continue;
    }

    if (!found_valid_region)
    {
      combined = clamped;
      found_valid_region = true;
      continue;
    }
    combined.left = std::min(combined.left, clamped.left);
    combined.top = std::min(combined.top, clamped.top);
    combined.right = std::max(combined.right, clamped.right);
    combined.bottom = std::max(combined.bottom, clamped.bottom);
  }

  if (!found_valid_region)
  {
    return std::nullopt;
  }

  return combined;
}

std::optional<std::pair<int, int>> widestRowInterval(const cv::Mat &roi_mask, int y)
{
  if (roi_mask.empty() || y < 0 || y >= roi_mask.rows)
  {
    return std::nullopt;
  }

  int best_start = -1;
  int best_end = -1;
  int run_start = -1;
  for (int x = 0; x < roi_mask.cols; ++x)
  {
    const bool inside = roi_mask.at<unsigned char>(y, x) > 0;
    if (inside)
    {
      if (run_start < 0)
      {
        run_start = x;
      }
      continue;
    }

    if (run_start >= 0)
    {
      const int run_end = x - 1;
      if (best_start < 0 || (run_end - run_start) > (best_end - best_start))
      {
        best_start = run_start;
        best_end = run_end;
      }
      run_start = -1;
    }
  }

  if (run_start >= 0)
  {
    const int run_end = roi_mask.cols - 1;
    if (best_start < 0 || (run_end - run_start) > (best_end - best_start))
    {
      best_start = run_start;
      best_end = run_end;
    }
  }

  if (best_start < 0 || best_end <= best_start)
  {
    return std::nullopt;
  }
  return std::make_pair(best_start, best_end);
}

std::optional<std::pair<int, int>> widestColumnInterval(const cv::Mat &roi_mask, int x)
{
  if (roi_mask.empty() || x < 0 || x >= roi_mask.cols)
  {
    return std::nullopt;
  }

  int best_start = -1;
  int best_end = -1;
  int run_start = -1;
  for (int y = 0; y < roi_mask.rows; ++y)
  {
    const bool inside = roi_mask.at<unsigned char>(y, x) > 0;
    if (inside)
    {
      if (run_start < 0)
      {
        run_start = y;
      }
      continue;
    }

    if (run_start >= 0)
    {
      const int run_end = y - 1;
      if (best_start < 0 || (run_end - run_start) > (best_end - best_start))
      {
        best_start = run_start;
        best_end = run_end;
      }
      run_start = -1;
    }
  }

  if (run_start >= 0)
  {
    const int run_end = roi_mask.rows - 1;
    if (best_start < 0 || (run_end - run_start) > (best_end - best_start))
    {
      best_start = run_start;
      best_end = run_end;
    }
  }

  if (best_start < 0 || best_end <= best_start)
  {
    return std::nullopt;
  }
  return std::make_pair(best_start, best_end);
}

std::vector<cv::Point2f> simplifyPolygonPoints(const std::vector<cv::Point> &contour, const cv::Point &offset = cv::Point())
{
  if (contour.size() < 3)
  {
    return {};
  }

  std::vector<cv::Point> unique_points;
  unique_points.reserve(contour.size());
  for (const auto &point : contour)
  {
    if (unique_points.empty() || unique_points.back() != point)
    {
      unique_points.push_back(point);
    }
  }

  bool removed = true;
  while (removed && unique_points.size() >= 3)
  {
    removed = false;
    for (std::size_t i = 0; i < unique_points.size(); ++i)
    {
      const cv::Point &prev = unique_points[(i + unique_points.size() - 1) % unique_points.size()];
      const cv::Point &curr = unique_points[i];
      const cv::Point &next = unique_points[(i + 1) % unique_points.size()];
      const bool collinear_vertical = prev.x == curr.x && curr.x == next.x;
      const bool collinear_horizontal = prev.y == curr.y && curr.y == next.y;
      if (collinear_vertical || collinear_horizontal)
      {
        unique_points.erase(unique_points.begin() + static_cast<long>(i));
        removed = true;
        break;
      }
    }
  }

  std::vector<cv::Point2f> polygon;
  polygon.reserve(unique_points.size());
  for (const auto &point : unique_points)
  {
    polygon.emplace_back(
      static_cast<float>(point.x + offset.x),
      static_cast<float>(point.y + offset.y));
  }
  return polygon;
}

std::vector<cv::Point2f> extractRoiPolygonFromMask(const cv::Mat &roi_mask, const cv::Point &offset = cv::Point())
{
  if (roi_mask.empty())
  {
    return {};
  }

  cv::Mat contour_mask = roi_mask.clone();
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(contour_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty())
  {
    return {};
  }

  const auto largest_it = std::max_element(
    contours.begin(),
    contours.end(),
    [](const auto &a, const auto &b)
    {
      return std::fabs(cv::contourArea(a)) < std::fabs(cv::contourArea(b));
    });
  return simplifyPolygonPoints(*largest_it, offset);
}

std::vector<cv::Point2f> mergeRoiRegionsIntoPolygon(const std::vector<AxisAlignedRoiBounds> &roi_regions)
{
  const auto combined = combinedRoiBounds(roi_regions);
  if (!combined.has_value())
  {
    return {};
  }

  cv::Size mask_size(
    std::max(1, combined->right - combined->left + 1),
    std::max(1, combined->bottom - combined->top + 1));
  cv::Mat merged_mask(mask_size, CV_8UC1, cv::Scalar(0));

  for (const auto &region : roi_regions)
  {
    if (!isValidRoiBounds(region))
    {
      continue;
    }

    const AxisAlignedRoiBounds shifted{
      region.left - combined->left,
      region.top - combined->top,
      region.right - combined->left,
      region.bottom - combined->top,
    };
    const auto polygon = roiPolygonForImage(roiPointsFromBounds(shifted), mask_size);
    if (polygon.size() >= 3)
    {
      const std::vector<std::vector<cv::Point>> polygons{polygon};
      cv::fillPoly(merged_mask, polygons, cv::Scalar(255));
    }
  }

  return extractRoiPolygonFromMask(merged_mask, cv::Point(combined->left, combined->top));
}

cv::Mat buildRoiMask(const cv::Size &size, const std::vector<AxisAlignedRoiBounds> &roi_regions)
{
  cv::Mat roi_mask(size, CV_8UC1, cv::Scalar(0));
  for (const auto &region : roi_regions)
  {
    const auto polygon = roiPolygonForImage(roiPointsFromBounds(region), size);
    if (polygon.size() == 4)
    {
      cv::fillConvexPoly(roi_mask, polygon, cv::Scalar(255));
    }
  }
  return roi_mask;
}

std::vector<std::pair<int, int>> mergeIntervals(std::vector<std::pair<int, int>> intervals)
{
  if (intervals.empty())
  {
    return {};
  }

  std::sort(intervals.begin(), intervals.end());
  std::vector<std::pair<int, int>> merged;
  merged.push_back(intervals.front());
  for (std::size_t i = 1; i < intervals.size(); ++i)
  {
    auto &back = merged.back();
    if (intervals[i].first <= back.second + 1)
    {
      back.second = std::max(back.second, intervals[i].second);
    }
    else
    {
      merged.push_back(intervals[i]);
    }
  }
  return merged;
}

std::optional<std::pair<int, int>> widestRowInterval(
  const std::vector<AxisAlignedRoiBounds> &roi_regions,
  int y,
  const cv::Size &size)
{
  std::vector<std::pair<int, int>> intervals;
  for (const auto &region : roi_regions)
  {
    if (!isValidRoiBounds(region) || y < region.top || y > region.bottom)
    {
      continue;
    }
    intervals.emplace_back(
      std::clamp(region.left, 0, size.width - 1),
      std::clamp(region.right, 0, size.width - 1));
  }
  const auto merged = mergeIntervals(std::move(intervals));
  if (merged.empty())
  {
    return std::nullopt;
  }

  return *std::max_element(
    merged.begin(),
    merged.end(),
    [](const auto &a, const auto &b)
    {
      return (a.second - a.first) < (b.second - b.first);
    });
}

std::optional<std::pair<int, int>> widestColumnInterval(
  const std::vector<AxisAlignedRoiBounds> &roi_regions,
  int x,
  const cv::Size &size)
{
  std::vector<std::pair<int, int>> intervals;
  for (const auto &region : roi_regions)
  {
    if (!isValidRoiBounds(region) || x < region.left || x > region.right)
    {
      continue;
    }
    intervals.emplace_back(
      std::clamp(region.top, 0, size.height - 1),
      std::clamp(region.bottom, 0, size.height - 1));
  }
  const auto merged = mergeIntervals(std::move(intervals));
  if (merged.empty())
  {
    return std::nullopt;
  }

  return *std::max_element(
    merged.begin(),
    merged.end(),
    [](const auto &a, const auto &b)
    {
      return (a.second - a.first) < (b.second - b.first);
    });
}

std::vector<int> sampleAxisPositions(int start, int end, int count)
{
  std::vector<int> positions;
  if (end < start)
  {
    return positions;
  }

  const int clamped_count = std::max(1, count);
  positions.reserve(static_cast<std::size_t>(clamped_count));
  if (start == end || clamped_count == 1)
  {
    positions.push_back((start + end) / 2);
    return positions;
  }

  for (int idx = 0; idx < clamped_count; ++idx)
  {
    const double t = static_cast<double>(idx) / static_cast<double>(clamped_count - 1);
    const int value = static_cast<int>(std::round(
      static_cast<double>(start) + t * static_cast<double>(end - start)));
    if (positions.empty() || positions.back() != value)
    {
      positions.push_back(value);
    }
  }
  return positions;
}

float medianCoordinate(std::vector<float> values)
{
  if (values.empty())
  {
    return 0.0F;
  }

  const auto middle_it = values.begin() + static_cast<long>(values.size() / 2);
  std::nth_element(values.begin(), middle_it, values.end());
  float median = *middle_it;
  if (values.size() % 2U == 0U)
  {
    const auto lower_it = std::max_element(values.begin(), middle_it);
    median = 0.5F * (median + *lower_it);
  }
  return median;
}

struct AxisSideFitResult
{
  float coordinate {0.0F};
  std::vector<cv::Point2f> inliers;
};

std::optional<AxisSideFitResult> fitAxisAlignedSide(
  const std::vector<cv::Point2f> &samples,
  bool vertical_side,
  int outlier_sensitivity)
{
  if (samples.empty())
  {
    return std::nullopt;
  }

  std::vector<float> coordinates;
  coordinates.reserve(samples.size());
  for (const auto &sample : samples)
  {
    coordinates.push_back(vertical_side ? sample.x : sample.y);
  }

  const float median = medianCoordinate(coordinates);
  std::vector<float> deviations;
  deviations.reserve(coordinates.size());
  for (const float coordinate : coordinates)
  {
    deviations.push_back(std::fabs(coordinate - median));
  }

  const double remapped_sensitivity = remapOutlierSensitivityToFitRange(outlier_sensitivity);
  const double sensitivity_factor = std::clamp(4.0 - 0.03 * remapped_sensitivity, 1.0, 4.0);
  const float median_deviation = medianCoordinate(deviations);
  const float threshold = static_cast<float>(std::max(
    kMinOutlierDistancePx,
    static_cast<double>(median_deviation) * sensitivity_factor));

  AxisSideFitResult result;
  for (const auto &sample : samples)
  {
    const float coordinate = vertical_side ? sample.x : sample.y;
    if (std::fabs(coordinate - median) <= threshold)
    {
      result.inliers.push_back(sample);
    }
  }

  if (result.inliers.empty())
  {
    return std::nullopt;
  }

  coordinates.clear();
  coordinates.reserve(result.inliers.size());
  for (const auto &sample : result.inliers)
  {
    coordinates.push_back(vertical_side ? sample.x : sample.y);
  }
  result.coordinate = medianCoordinate(coordinates);
  return result;
}

std::optional<cv::Point2f> findConfirmedTransitionAlongLine(
  const cv::Mat &mask,
  const cv::Point &start,
  const cv::Point &end,
  int old_value,
  int new_value,
  int confirm_px,
  int previous_color_percent)
{
  const int dx = (end.x > start.x) ? 1 : (end.x < start.x ? -1 : 0);
  const int dy = (end.y > start.y) ? 1 : (end.y < start.y ? -1 : 0);
  const int steps = std::max(std::abs(end.x - start.x), std::abs(end.y - start.y));
  if (steps <= 0 || (dx == 0 && dy == 0))
  {
    return std::nullopt;
  }

  const int clamped_confirm_px = std::clamp(confirm_px, 1, 100);
  const int clamped_previous_color_percent = std::clamp(previous_color_percent, 20, 100);
  const int required_previous_matches = std::max(
    1,
    static_cast<int>(std::ceil(
      static_cast<double>(clamped_previous_color_percent) * static_cast<double>(clamped_confirm_px) / 100.0)));
  const int required_next_matches = std::max(
    1,
    static_cast<int>(std::ceil(kNextColorConfirmMatchRatio * static_cast<double>(clamped_confirm_px))));
  const auto sample_at = [&](int step_index) -> int
  {
    if (step_index < 0 || step_index > steps)
    {
      return -1;
    }
    const int x = start.x + step_index * dx;
    const int y = start.y + step_index * dy;
    if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows)
    {
      return -1;
    }
    return mask.at<unsigned char>(y, x) > 0 ? 1 : 0;
  };

  for (int step_index = 0; step_index <= steps; ++step_index)
  {
    if (sample_at(step_index) != new_value)
    {
      continue;
    }

    int previous_matches = 0;
    int next_matches = 0;
    for (int offset = 1; offset <= clamped_confirm_px; ++offset)
    {
      if (sample_at(step_index - offset) == old_value)
      {
        ++previous_matches;
      }
      if (sample_at(step_index + offset) == new_value)
      {
        ++next_matches;
      }
    }

    if (previous_matches >= required_previous_matches && next_matches >= required_next_matches)
    {
      return cv::Point2f(
        static_cast<float>(start.x + step_index * dx),
        static_cast<float>(start.y + step_index * dy));
    }
  }

  return std::nullopt;
}

void drawRoiOverlay(cv::Mat &image, const std::vector<cv::Point2f> &roi_points)
{
  if (!hasValidRoiPoints(roi_points))
  {
    return;
  }

  const cv::Mat roi_mask = buildRoiMask(image.size(), roi_points);
  cv::Mat dimmed;
  cv::addWeighted(image, 0.42, cv::Mat(image.size(), image.type(), cv::Scalar(0, 0, 0)), 0.58, 0.0, dimmed);
  image.copyTo(dimmed, roi_mask);
  image = dimmed;

  for (std::size_t i = 0; i < roi_points.size(); ++i)
  {
    const cv::Point current(
      static_cast<int>(std::round(roi_points[i].x)),
      static_cast<int>(std::round(roi_points[i].y)));
    const cv::Point next(
      static_cast<int>(std::round(roi_points[(i + 1) % roi_points.size()].x)),
      static_cast<int>(std::round(roi_points[(i + 1) % roi_points.size()].y)));
    cv::line(image, current, next, cv::Scalar(255, 220, 0), 2, cv::LINE_AA);
    cv::circle(image, current, 6, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, current, 4, cv::Scalar(0, 255, 255), -1);
  }
}

void drawRoiOverlay(cv::Mat &image, const std::vector<AxisAlignedRoiBounds> &roi_regions)
{
  if (!hasValidRoiRegions(roi_regions))
  {
    return;
  }

  const cv::Mat roi_mask = buildRoiMask(image.size(), roi_regions);
  cv::Mat dimmed;
  cv::addWeighted(image, 0.42, cv::Mat(image.size(), image.type(), cv::Scalar(0, 0, 0)), 0.58, 0.0, dimmed);
  image.copyTo(dimmed, roi_mask);
  image = dimmed;

  for (const auto &region : roi_regions)
  {
    if (!isValidRoiBounds(region))
    {
      continue;
    }
    drawRoiOverlay(image, roiPointsFromBounds(region));
  }
}

std::optional<cv::Point2f> findConfirmedMaskTransition(
  const cv::Mat &mask,
  const cv::Point2f &center,
  const cv::Point2f &direction,
  float search_start_radius,
  float search_end_radius,
  int old_value,
  int new_value,
  int confirm_px,
  int previous_color_percent,
  bool trace_out_to_in)
{
  const int clamped_confirm_px = std::clamp(confirm_px, 1, 100);
  const int clamped_previous_color_percent = std::clamp(previous_color_percent, 20, 100);
  const int required_previous_matches = std::max(
    1,
    static_cast<int>(std::ceil(
      static_cast<double>(clamped_previous_color_percent) * static_cast<double>(clamped_confirm_px) / 100.0)));
  const int required_next_matches = std::max(
    1,
    static_cast<int>(std::ceil(kNextColorConfirmMatchRatio * static_cast<double>(clamped_confirm_px))));
  int start_radius = static_cast<int>(std::round(search_start_radius));
  const int end_radius = static_cast<int>(std::round(search_end_radius));
  const int step = trace_out_to_in ? -1 : 1;
  const auto should_continue = [step, end_radius](int radius)
  {
    return step > 0 ? radius <= end_radius : radius >= end_radius;
  };

  for (int radius = start_radius; should_continue(radius); radius += step)
  {
    const cv::Point2f candidate = center + static_cast<float>(radius) * direction;
    if (sampleMaskAt(mask, candidate) != new_value)
    {
      continue;
    }

    int previous_matches = 0;
    for (int offset = 1; offset <= clamped_confirm_px; ++offset)
    {
      const int prev_radius = radius - offset * step;
      const cv::Point2f prev_point = center + static_cast<float>(prev_radius) * direction;
      if (sampleMaskAt(mask, prev_point) == old_value)
      {
        ++previous_matches;
      }
    }
    if (previous_matches < required_previous_matches)
    {
      continue;
    }

    int next_matches = 0;
    for (int offset = 1; offset <= clamped_confirm_px; ++offset)
    {
      const int next_radius = radius + offset * step;
      const cv::Point2f next_point = center + static_cast<float>(next_radius) * direction;
      if (sampleMaskAt(mask, next_point) == new_value)
      {
        ++next_matches;
      }
    }
    if (next_matches < required_next_matches)
    {
      continue;
    }

    return candidate;
  }

  return std::nullopt;
}

std::optional<ItemEstimate> detectItemFromAxisAlignedRoi(
  const cv::Mat &mask,
  const cv::Mat &depth_m,
  const CameraInfoMsg::ConstSharedPtr &camera_info,
  const std::vector<cv::Point2f> &roi_points,
  int ray_step_px,
  int depth_edge_offset_px,
  int previous_color_percent,
  int horizontal_ray_count,
  int vertical_ray_count,
  int outlier_sensitivity,
  bool detect_black_to_white,
  bool trace_out_to_in)
{
  if (!camera_info)
  {
    return std::nullopt;
  }

  const auto roi_bounds = roiBoundsForImage(roi_points, mask.size());
  if (!roi_bounds.has_value())
  {
    return std::nullopt;
  }
  const cv::Mat roi_mask = buildRoiMask(mask.size(), roi_points);

  const int confirm_px = std::clamp(ray_step_px, 1, 100);
  const int row_scan_count = std::clamp(horizontal_ray_count, 50, 100);
  const int col_scan_count = std::clamp(vertical_ray_count, 50, 150);
  const int old_value = detect_black_to_white ? 0 : 1;
  const int new_value = detect_black_to_white ? 1 : 0;
  const int left = roi_bounds->left;
  const int right = roi_bounds->right;
  const int top = roi_bounds->top;
  const int bottom = roi_bounds->bottom;
  if (right - left < 20 || bottom - top < 20)
  {
    return std::nullopt;
  }

  const int min_row_width = std::max(20, (right - left + 1) / 2);
  const int min_col_height = std::max(20, (bottom - top + 1) / 2);
  const auto row_samples = sampleAxisPositions(top, bottom, row_scan_count);
  const auto col_samples = sampleAxisPositions(left, right, col_scan_count);

  std::vector<cv::Point2f> edge_points;
  std::vector<cv::Point2f> left_samples;
  std::vector<cv::Point2f> right_samples;
  std::vector<cv::Point2f> top_samples;
  std::vector<cv::Point2f> bottom_samples;
  edge_points.reserve(2U * (row_samples.size() + col_samples.size()));

  for (const int y : row_samples)
  {
    const auto interval = widestRowInterval(roi_mask, y);
    if (!interval.has_value())
    {
      continue;
    }

    const int interval_left = interval->first;
    const int interval_right = interval->second;
    if (interval_right - interval_left + 1 < min_row_width)
    {
      continue;
    }

    const int center_x = (interval_left + interval_right) / 2;
    const cv::Point left_start = trace_out_to_in ? cv::Point(interval_left, y) : cv::Point(center_x, y);
    const cv::Point left_end = trace_out_to_in ? cv::Point(center_x, y) : cv::Point(interval_left, y);
    const cv::Point right_start = trace_out_to_in ? cv::Point(interval_right, y) : cv::Point(center_x, y);
    const cv::Point right_end = trace_out_to_in ? cv::Point(center_x, y) : cv::Point(interval_right, y);

    const auto left_edge = findConfirmedTransitionAlongLine(
      mask, left_start, left_end, old_value, new_value, confirm_px, previous_color_percent);
    if (left_edge.has_value())
    {
      left_samples.push_back(*left_edge);
      edge_points.push_back(*left_edge);
    }

    const auto right_edge = findConfirmedTransitionAlongLine(
      mask, right_start, right_end, old_value, new_value, confirm_px, previous_color_percent);
    if (right_edge.has_value())
    {
      right_samples.push_back(*right_edge);
      edge_points.push_back(*right_edge);
    }
  }

  for (const int x : col_samples)
  {
    const auto interval = widestColumnInterval(roi_mask, x);
    if (!interval.has_value())
    {
      continue;
    }

    const int interval_top = interval->first;
    const int interval_bottom = interval->second;
    if (interval_bottom - interval_top + 1 < min_col_height)
    {
      continue;
    }

    const int center_y = (interval_top + interval_bottom) / 2;
    const cv::Point top_start = trace_out_to_in ? cv::Point(x, interval_top) : cv::Point(x, center_y);
    const cv::Point top_end = trace_out_to_in ? cv::Point(x, center_y) : cv::Point(x, interval_top);
    const cv::Point bottom_start = trace_out_to_in ? cv::Point(x, interval_bottom) : cv::Point(x, center_y);
    const cv::Point bottom_end = trace_out_to_in ? cv::Point(x, center_y) : cv::Point(x, interval_bottom);

    const auto top_edge = findConfirmedTransitionAlongLine(
      mask, top_start, top_end, old_value, new_value, confirm_px, previous_color_percent);
    if (top_edge.has_value())
    {
      top_samples.push_back(*top_edge);
      edge_points.push_back(*top_edge);
    }

    const auto bottom_edge = findConfirmedTransitionAlongLine(
      mask, bottom_start, bottom_end, old_value, new_value, confirm_px, previous_color_percent);
    if (bottom_edge.has_value())
    {
      bottom_samples.push_back(*bottom_edge);
      edge_points.push_back(*bottom_edge);
    }
  }

  return buildItemEstimateFromIsolatedSideSamples(
    edge_points,
    left_samples,
    right_samples,
    top_samples,
    bottom_samples,
    depth_m,
    *camera_info,
    depth_edge_offset_px,
    outlier_sensitivity);
}

std::optional<ItemEstimate> detectItemFromAxisAlignedRoi(
  const cv::Mat &mask,
  const cv::Mat &depth_m,
  const CameraInfoMsg::ConstSharedPtr &camera_info,
  const std::vector<AxisAlignedRoiBounds> &roi_regions,
  int ray_step_px,
  int depth_edge_offset_px,
  int previous_color_percent,
  int horizontal_ray_count,
  int vertical_ray_count,
  int outlier_sensitivity,
  bool detect_black_to_white,
  bool trace_out_to_in)
{
  if (!camera_info)
  {
    return std::nullopt;
  }

  const auto roi_bounds = roiBoundsForImage(roi_regions, mask.size());
  if (!roi_bounds.has_value())
  {
    return std::nullopt;
  }

  const int confirm_px = std::clamp(ray_step_px, 1, 100);
  const int row_scan_count = std::clamp(horizontal_ray_count, 50, 100);
  const int col_scan_count = std::clamp(vertical_ray_count, 50, 150);
  const int old_value = detect_black_to_white ? 0 : 1;
  const int new_value = detect_black_to_white ? 1 : 0;
  const int left = roi_bounds->left;
  const int right = roi_bounds->right;
  const int top = roi_bounds->top;
  const int bottom = roi_bounds->bottom;
  if (right - left < 20 || bottom - top < 20)
  {
    return std::nullopt;
  }

  const int min_row_width = std::max(20, (right - left + 1) / 2);
  const int min_col_height = std::max(20, (bottom - top + 1) / 2);
  const auto row_samples = sampleAxisPositions(top, bottom, row_scan_count);
  const auto col_samples = sampleAxisPositions(left, right, col_scan_count);

  std::vector<cv::Point2f> edge_points;
  std::vector<cv::Point2f> left_samples;
  std::vector<cv::Point2f> right_samples;
  std::vector<cv::Point2f> top_samples;
  std::vector<cv::Point2f> bottom_samples;
  edge_points.reserve(2U * (row_samples.size() + col_samples.size()));

  for (const int y : row_samples)
  {
    const auto interval = widestRowInterval(roi_regions, y, mask.size());
    if (!interval.has_value())
    {
      continue;
    }
    const int interval_left = interval->first;
    const int interval_right = interval->second;
    if (interval_right - interval_left + 1 < min_row_width)
    {
      continue;
    }

    const int center_x = (interval_left + interval_right) / 2;
    const cv::Point left_start = trace_out_to_in ? cv::Point(interval_left, y) : cv::Point(center_x, y);
    const cv::Point left_end = trace_out_to_in ? cv::Point(center_x, y) : cv::Point(interval_left, y);
    const cv::Point right_start = trace_out_to_in ? cv::Point(interval_right, y) : cv::Point(center_x, y);
    const cv::Point right_end = trace_out_to_in ? cv::Point(center_x, y) : cv::Point(interval_right, y);

    const auto left_edge = findConfirmedTransitionAlongLine(
      mask, left_start, left_end, old_value, new_value, confirm_px, previous_color_percent);
    if (left_edge.has_value())
    {
      left_samples.push_back(*left_edge);
      edge_points.push_back(*left_edge);
    }

    const auto right_edge = findConfirmedTransitionAlongLine(
      mask, right_start, right_end, old_value, new_value, confirm_px, previous_color_percent);
    if (right_edge.has_value())
    {
      right_samples.push_back(*right_edge);
      edge_points.push_back(*right_edge);
    }
  }

  for (const int x : col_samples)
  {
    const auto interval = widestColumnInterval(roi_regions, x, mask.size());
    if (!interval.has_value())
    {
      continue;
    }
    const int interval_top = interval->first;
    const int interval_bottom = interval->second;
    if (interval_bottom - interval_top + 1 < min_col_height)
    {
      continue;
    }

    const int center_y = (interval_top + interval_bottom) / 2;
    const cv::Point top_start = trace_out_to_in ? cv::Point(x, interval_top) : cv::Point(x, center_y);
    const cv::Point top_end = trace_out_to_in ? cv::Point(x, center_y) : cv::Point(x, interval_top);
    const cv::Point bottom_start = trace_out_to_in ? cv::Point(x, interval_bottom) : cv::Point(x, center_y);
    const cv::Point bottom_end = trace_out_to_in ? cv::Point(x, center_y) : cv::Point(x, interval_bottom);

    const auto top_edge = findConfirmedTransitionAlongLine(
      mask, top_start, top_end, old_value, new_value, confirm_px, previous_color_percent);
    if (top_edge.has_value())
    {
      top_samples.push_back(*top_edge);
      edge_points.push_back(*top_edge);
    }

    const auto bottom_edge = findConfirmedTransitionAlongLine(
      mask, bottom_start, bottom_end, old_value, new_value, confirm_px, previous_color_percent);
    if (bottom_edge.has_value())
    {
      bottom_samples.push_back(*bottom_edge);
      edge_points.push_back(*bottom_edge);
    }
  }

  return buildItemEstimateFromIsolatedSideSamples(
    edge_points,
    left_samples,
    right_samples,
    top_samples,
    bottom_samples,
    depth_m,
    *camera_info,
    depth_edge_offset_px,
    outlier_sensitivity);
}

bool isAreaWithinTaughtBand(double measured_area_cm2, double taught_area_cm2, int tolerance_percent)
{
  if (taught_area_cm2 <= 0.0)
  {
    return true;
  }
  const double tolerance_fraction = static_cast<double>(std::clamp(tolerance_percent, 1, 20)) / 100.0;
  const double lower = taught_area_cm2 * (1.0 - tolerance_fraction);
  const double upper = taught_area_cm2 * (1.0 + tolerance_fraction);
  return measured_area_cm2 >= lower && measured_area_cm2 <= upper;
}

bool areEdgeLengthsWithinTaughtBand(
  const std::array<double, 4> &measured_edge_lengths_cm,
  const std::array<double, 4> &taught_edge_lengths_cm,
  int tolerance_percent)
{
  if (!hasValidEdgeLengthsCm(measured_edge_lengths_cm) || !hasValidEdgeLengthsCm(taught_edge_lengths_cm))
  {
    return true;
  }

  std::array<double, 4> measured_sorted = measured_edge_lengths_cm;
  std::array<double, 4> taught_sorted = taught_edge_lengths_cm;
  std::sort(measured_sorted.begin(), measured_sorted.end(), std::greater<double>());
  std::sort(taught_sorted.begin(), taught_sorted.end(), std::greater<double>());

  const double tolerance_fraction = static_cast<double>(std::clamp(tolerance_percent, 1, 20)) / 100.0;
  for (std::size_t i = 0; i < measured_sorted.size(); ++i)
  {
    const double lower = taught_sorted[i] * (1.0 - tolerance_fraction);
    const double upper = taught_sorted[i] * (1.0 + tolerance_fraction);
    if (measured_sorted[i] < lower || measured_sorted[i] > upper)
    {
      return false;
    }
  }
  return true;
}

double maxEdgeLengthDeviationPercent(
  const std::array<double, 4> &measured_edge_lengths_cm,
  const std::array<double, 4> &taught_edge_lengths_cm)
{
  if (!hasValidEdgeLengthsCm(measured_edge_lengths_cm) || !hasValidEdgeLengthsCm(taught_edge_lengths_cm))
  {
    return 0.0;
  }

  std::array<double, 4> measured_sorted = measured_edge_lengths_cm;
  std::array<double, 4> taught_sorted = taught_edge_lengths_cm;
  std::sort(measured_sorted.begin(), measured_sorted.end(), std::greater<double>());
  std::sort(taught_sorted.begin(), taught_sorted.end(), std::greater<double>());

  double max_error_percent = 0.0;
  for (std::size_t i = 0; i < measured_sorted.size(); ++i)
  {
    if (taught_sorted[i] <= 1e-6)
    {
      continue;
    }
    const double error_percent =
      100.0 * std::fabs(measured_sorted[i] - taught_sorted[i]) / taught_sorted[i];
    max_error_percent = std::max(max_error_percent, error_percent);
  }
  return max_error_percent;
}

void drawItemEstimate(cv::Mat &binary_bgr, const std::optional<ItemEstimate> &estimate, int depth_edge_offset_px)
{
  if (!estimate.has_value())
  {
    return;
  }

  cv::Mat overlay = binary_bgr.clone();
  cv::fillConvexPoly(overlay, estimate->polygon, cv::Scalar(40, 180, 40));
  cv::addWeighted(overlay, 0.30, binary_bgr, 0.70, 0.0, binary_bgr);

  for (std::size_t i = 0; i < estimate->corners.size(); ++i)
  {
    cv::line(
      binary_bgr,
      estimate->corners[i],
      estimate->corners[(i + 1) % estimate->corners.size()],
      cv::Scalar(0, 255, 0),
      3);
  }

  for (const auto &segment : buildDepthMeasurementSegments(estimate->corners, depth_edge_offset_px))
  {
    cv::line(binary_bgr, segment.first, segment.second, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
  }

  for (const auto &point : estimate->filtered_edge_points)
  {
    cv::circle(binary_bgr, point, 4, cv::Scalar(0, 165, 255), -1);
  }

  const std::string area_label =
    estimate->has_metric_estimate && estimate->area_cm2 > 0.0
      ? cv::format("item area=%.0f mm2", estimate->area_cm2 * kSquareCentimetersToSquareMillimeters)
      : "item area=n/a";
  int area_baseline = 0;
  const cv::Size area_text_size = cv::getTextSize(
    area_label,
    cv::FONT_HERSHEY_SIMPLEX,
    0.65,
    2,
    &area_baseline);
  const cv::Point area_text_origin(
    static_cast<int>(std::round(estimate->rect.center.x - 0.5F * static_cast<float>(area_text_size.width))),
    static_cast<int>(std::round(estimate->rect.center.y + 0.5F * static_cast<float>(area_text_size.height))));
  const cv::Rect area_text_box(
    area_text_origin.x - 8,
    area_text_origin.y - area_text_size.height - 8,
    area_text_size.width + 16,
    area_text_size.height + area_baseline + 12);
  cv::rectangle(binary_bgr, area_text_box, cv::Scalar(24, 24, 24), cv::FILLED);
  cv::rectangle(binary_bgr, area_text_box, cv::Scalar(0, 255, 0), 1);
  cv::putText(
    binary_bgr,
    area_label,
    area_text_origin,
    cv::FONT_HERSHEY_SIMPLEX,
    0.65,
    cv::Scalar(0, 255, 0),
    2);

  if (!estimate->has_metric_estimate || estimate->corners.size() != 4)
  {
    return;
  }

  const int origin_idx = lowerLeftCornerIndex(estimate->corners);
  if (origin_idx < 0)
  {
    return;
  }

  const int prev_idx = (origin_idx + 3) % 4;
  const int next_idx = (origin_idx + 1) % 4;
  const int opposite_idx = (origin_idx + 2) % 4;
  const float origin_prev_px = cv::norm(estimate->corners[origin_idx] - estimate->corners[prev_idx]);
  const float origin_next_px = cv::norm(estimate->corners[origin_idx] - estimate->corners[next_idx]);

  std::array<std::pair<int, int>, 4> edge_segments;
  if (origin_prev_px >= origin_next_px)
  {
    edge_segments = {{
      {origin_idx, prev_idx},
      {next_idx, opposite_idx},
      {origin_idx, next_idx},
      {prev_idx, opposite_idx},
    }};
  }
  else
  {
    edge_segments = {{
      {origin_idx, next_idx},
      {prev_idx, opposite_idx},
      {origin_idx, prev_idx},
      {next_idx, opposite_idx},
    }};
  }

  for (std::size_t i = 0; i < edge_segments.size(); ++i)
  {
    const cv::Point2f start = estimate->corners[edge_segments[i].first];
    const cv::Point2f end = estimate->corners[edge_segments[i].second];
    const cv::Point2f mid = 0.5F * (start + end);
    cv::Point2f edge_dir = end - start;
    const float edge_len = std::sqrt(edge_dir.dot(edge_dir));
    if (edge_len < 1e-3F)
    {
      continue;
    }

    edge_dir *= (1.0F / edge_len);
    cv::Point2f normal(-edge_dir.y, edge_dir.x);
    const cv::Point2f outward_a = mid + 18.0F * normal;
    const cv::Point2f outward_b = mid - 18.0F * normal;
    const float dist_a = cv::norm(outward_a - estimate->rect.center);
    const float dist_b = cv::norm(outward_b - estimate->rect.center);
    const cv::Point2f label_anchor = dist_a >= dist_b ? outward_a : outward_b;

    const std::string label = cv::format("%.1f mm", estimate->edge_lengths_cm[i] * kCentimetersToMillimeters);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.52, 2, &baseline);
    const cv::Point text_origin(
      static_cast<int>(std::round(label_anchor.x - 0.5F * static_cast<float>(text_size.width))),
      static_cast<int>(std::round(label_anchor.y + 0.5F * static_cast<float>(text_size.height))));
    const cv::Rect text_box(
      text_origin.x - 6,
      text_origin.y - text_size.height - 6,
      text_size.width + 12,
      text_size.height + baseline + 10);
    cv::rectangle(binary_bgr, text_box, cv::Scalar(24, 24, 24), cv::FILLED);
    cv::rectangle(binary_bgr, text_box, cv::Scalar(0, 255, 0), 1);
    cv::putText(
      binary_bgr,
      label,
      text_origin,
      cv::FONT_HERSHEY_SIMPLEX,
      0.52,
      cv::Scalar(0, 255, 0),
      2);
  }
}

void drawItemName(cv::Mat &image, const std::string &item_name)
{
  if (item_name.empty())
  {
    return;
  }

  cv::putText(
    image,
    item_name,
    cv::Point(18, 42),
    cv::FONT_HERSHEY_SIMPLEX,
    1.08,
    cv::Scalar(0, 255, 255),
    3);
}

void drawCenterCursor(cv::Mat &image, const std::vector<cv::Point2f> &roi_points)
{
  if (image.empty())
  {
    return;
  }

  cv::Point center(image.cols / 2, image.rows / 2);
  if (const auto roi_bounds = roiBoundsForImage(roi_points, image.size()); roi_bounds.has_value())
  {
    center = cv::Point(
      (roi_bounds->left + roi_bounds->right) / 2,
      (roi_bounds->top + roi_bounds->bottom) / 2);
  }
  constexpr int kHalfSize = 12;
  constexpr int kGap = 3;

  cv::line(
    image,
    cv::Point(center.x - kHalfSize, center.y),
    cv::Point(center.x - kGap, center.y),
    cv::Scalar(0, 0, 0),
    4,
    cv::LINE_AA);
  cv::line(
    image,
    cv::Point(center.x + kGap, center.y),
    cv::Point(center.x + kHalfSize, center.y),
    cv::Scalar(0, 0, 0),
    4,
    cv::LINE_AA);
  cv::line(
    image,
    cv::Point(center.x, center.y - kHalfSize),
    cv::Point(center.x, center.y - kGap),
    cv::Scalar(0, 0, 0),
    4,
    cv::LINE_AA);
  cv::line(
    image,
    cv::Point(center.x, center.y + kGap),
    cv::Point(center.x, center.y + kHalfSize),
    cv::Scalar(0, 0, 0),
    4,
    cv::LINE_AA);

  cv::line(
    image,
    cv::Point(center.x - kHalfSize, center.y),
    cv::Point(center.x - kGap, center.y),
    cv::Scalar(0, 255, 255),
    2,
    cv::LINE_AA);
  cv::line(
    image,
    cv::Point(center.x + kGap, center.y),
    cv::Point(center.x + kHalfSize, center.y),
    cv::Scalar(0, 255, 255),
    2,
    cv::LINE_AA);
  cv::line(
    image,
    cv::Point(center.x, center.y - kHalfSize),
    cv::Point(center.x, center.y - kGap),
    cv::Scalar(0, 255, 255),
    2,
    cv::LINE_AA);
  cv::line(
    image,
    cv::Point(center.x, center.y + kGap),
    cv::Point(center.x, center.y + kHalfSize),
    cv::Scalar(0, 255, 255),
    2,
    cv::LINE_AA);
}

void drawItemAxes(cv::Mat &image, const std::optional<ItemEstimate> &estimate)
{
  if (!estimate.has_value() || estimate->corners.size() != 4)
  {
    return;
  }

  const int lower_left_idx = lowerLeftCornerIndex(estimate->corners);
  if (lower_left_idx < 0)
  {
    return;
  }

  const cv::Point2f origin = estimate->corners[lower_left_idx];
  const cv::Point2f prev_corner = estimate->corners[(lower_left_idx + 3) % 4];
  const cv::Point2f next_corner = estimate->corners[(lower_left_idx + 1) % 4];

  cv::Point2f dir_a = prev_corner - origin;
  cv::Point2f dir_b = next_corner - origin;

  const float len_a = std::sqrt(dir_a.dot(dir_a));
  const float len_b = std::sqrt(dir_b.dot(dir_b));
  if (len_a < 1e-3F || len_b < 1e-3F)
  {
    return;
  }

  cv::Point2f x_dir = len_a >= len_b ? dir_a : dir_b;
  cv::Point2f y_dir = len_a >= len_b ? dir_b : dir_a;
  const float x_norm = std::sqrt(x_dir.dot(x_dir));
  const float y_norm = std::sqrt(y_dir.dot(y_dir));

  x_dir *= (1.0F / x_norm);
  y_dir *= (1.0F / y_norm);

  const float axis_len = 60.0F;
  const cv::Point2f x_end = origin + axis_len * x_dir;
  const cv::Point2f y_end = origin + axis_len * y_dir;
  cv::Point2f z_dir = -(x_dir + y_dir);
  const float z_norm = std::sqrt(z_dir.dot(z_dir));
  if (z_norm < 1e-3F)
  {
    return;
  }
  z_dir *= (1.0F / z_norm);
  const cv::Point2f z_end = origin + axis_len * 0.75F * z_dir;

  cv::arrowedLine(image, origin, x_end, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.15);
  cv::arrowedLine(image, origin, y_end, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, 0.15);
  cv::arrowedLine(image, origin, z_end, cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0, 0.15);
  cv::putText(image, "X", x_end + cv::Point2f(6.0F, -6.0F), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2);
  cv::putText(image, "Y", y_end + cv::Point2f(6.0F, -6.0F), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
  cv::putText(image, "Z", z_end + cv::Point2f(6.0F, -6.0F), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 0, 0), 2);
}

void drawPoseHullOverlay(
  cv::Mat &image,
  const std::optional<BinarizedPoseEstimate2D> &estimate,
  bool axis_only = false)
{
  if (!estimate.has_value() || estimate->blob_poses.empty())
  {
    return;
  }

  for (std::size_t pose_index = 0; pose_index < estimate->blob_poses.size(); ++pose_index)
  {
    const auto &blob_pose = estimate->blob_poses[pose_index];
    if (!axis_only && blob_pose.hull_points.size() >= 3)
    {
      const std::vector<std::vector<cv::Point>> hull_polys{blob_pose.hull_points};
      cv::polylines(image, hull_polys, true, cv::Scalar(60, 200, 60), 2, cv::LINE_AA);
    }

    if (!axis_only)
    {
      for (std::size_t i = 0; i < blob_pose.corners.size(); ++i)
      {
        const cv::Point2f start = blob_pose.corners[i];
        const cv::Point2f end = blob_pose.corners[(i + 1) % blob_pose.corners.size()];
        cv::line(image, start, end, cv::Scalar(255, 220, 40), 2, cv::LINE_AA);
        cv::circle(image, start, 4, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
      }
    }

    cv::Point2f center(0.0F, 0.0F);
    if (!blob_pose.corners.empty())
    {
      for (const auto &corner : blob_pose.corners)
      {
        center += corner;
      }
      center *= (1.0F / static_cast<float>(blob_pose.corners.size()));
    }
    else
    {
      center = blob_pose.origin;
    }

    cv::Point2f x_dir = blob_pose.x_axis_tip - blob_pose.origin;
    cv::Point2f y_dir = blob_pose.z_axis_tip - blob_pose.origin;
    const float x_norm = std::sqrt(x_dir.dot(x_dir));
    const float y_norm = std::sqrt(y_dir.dot(y_dir));
    if (x_norm >= 1e-3F)
    {
      x_dir *= (1.0F / x_norm);
    }
    if (y_norm >= 1e-3F)
    {
      y_dir *= (1.0F / y_norm);
    }

    const float x_half_len = std::max(8.0F, blob_pose.x_length_px * 0.5F);
    const float y_half_len = std::max(8.0F, blob_pose.z_length_px * 0.5F);
    const cv::Point2f x_start = center - x_dir * x_half_len;
    const cv::Point2f x_end = center + x_dir * x_half_len;
    const cv::Point2f y_start = center - y_dir * y_half_len;
    const cv::Point2f y_end = center + y_dir * y_half_len;

    cv::line(image, x_start, x_end, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::line(image, y_start, y_end, cv::Scalar(255, 120, 0), 2, cv::LINE_AA);
    cv::arrowedLine(image, center, x_end, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.15);
    cv::arrowedLine(image, center, y_end, cv::Scalar(255, 120, 0), 3, cv::LINE_AA, 0, 0.15);
    cv::circle(image, center, 6, cv::Scalar(40, 255, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(
      image,
      "X",
      x_end + cv::Point2f(4.0F, -4.0F),
      cv::FONT_HERSHEY_SIMPLEX,
      0.50,
      cv::Scalar(0, 0, 255),
      2,
      cv::LINE_AA);
    cv::putText(
      image,
      "Y",
      y_end + cv::Point2f(4.0F, -4.0F),
      cv::FONT_HERSHEY_SIMPLEX,
      0.50,
      cv::Scalar(255, 120, 0),
      2,
      cv::LINE_AA);

    if (!axis_only)
    {
      cv::putText(
        image,
        std::to_string(pose_index + 1),
        center + cv::Point2f(8.0F, -6.0F),
        cv::FONT_HERSHEY_SIMPLEX,
        0.56,
        cv::Scalar(40, 255, 255),
        2,
        cv::LINE_AA);

      if (blob_pose.member_count >= 2)
      {
        for (std::size_t member_index = 0; member_index < blob_pose.member_centers_px.size(); ++member_index)
        {
          const cv::Point2f &member_center = blob_pose.member_centers_px[member_index];
          const cv::Scalar member_color = member_index == 0
            ? cv::Scalar(40, 255, 255)
            : cv::Scalar(255, 160, 40);
          cv::circle(image, member_center, 7, member_color, 2, cv::LINE_AA);
          cv::putText(
            image,
            std::to_string(static_cast<int>(member_index + 1)),
            member_center + cv::Point2f(6.0F, -8.0F),
            cv::FONT_HERSHEY_SIMPLEX,
            0.46,
            member_color,
            2,
            cv::LINE_AA);
        }
      }
      if (blob_pose.has_custom_anchor)
      {
        cv::circle(image, blob_pose.anchor_point_px, 5, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
        cv::putText(
          image,
          "A",
          blob_pose.anchor_point_px + cv::Point2f(8.0F, 10.0F),
          cv::FONT_HERSHEY_SIMPLEX,
          0.46,
          cv::Scalar(0, 255, 255),
          2,
          cv::LINE_AA);
      }
    }
  }
}

void drawPredictedCompanionOverlay(
  cv::Mat &image,
  const std::optional<PairDirectShapeFitDebug2D> &debug_info)
{
  if (!debug_info.has_value())
  {
    return;
  }

  const auto draw_search_area = [&](const CompanionSearchDebug2D &search, std::size_t index)
  {
    const cv::Scalar hull_color = index < 3
      ? cv::Scalar(220, 90, 255)
      : cv::Scalar(0, 180, 255);
    const cv::Scalar radius_color = index < 3
      ? cv::Scalar(0, 220, 255)
      : cv::Scalar(0, 150, 255);
    if (search.hull.size() < 3)
    {
      return;
    }
    std::vector<cv::Point> hull_pixels;
    hull_pixels.reserve(search.hull.size());
    for (const auto &point : search.hull)
    {
      hull_pixels.emplace_back(
        static_cast<int>(std::lround(point.x)),
        static_cast<int>(std::lround(point.y)));
    }
    const std::vector<std::vector<cv::Point>> hull_polys{hull_pixels};
    cv::polylines(image, hull_polys, true, hull_color, 2, cv::LINE_AA);
    if (search.radius_px > 0)
    {
      cv::circle(image, search.center, search.radius_px, radius_color, 1, cv::LINE_AA);
    }
    cv::drawMarker(image, search.center, radius_color, cv::MARKER_CROSS, 16, 2, cv::LINE_AA);
    cv::circle(
      image,
      search.center,
      6,
      hull_color,
      2,
      cv::LINE_AA);
    cv::putText(
      image,
      cv::format("2? %.0f", search.angle_offset_deg),
      search.center + cv::Point2f(8.0F, -8.0F),
      cv::FONT_HERSHEY_SIMPLEX,
      0.44,
      hull_color,
      2,
      cv::LINE_AA);
  };

  if (!debug_info->companion_searches.empty())
  {
    for (std::size_t i = 0; i < debug_info->companion_searches.size(); ++i)
    {
      draw_search_area(debug_info->companion_searches[i], i);
    }
    return;
  }

  CompanionSearchDebug2D fallback_search;
  fallback_search.hull = debug_info->predicted_companion_hull;
  fallback_search.center = debug_info->predicted_companion_center;
  fallback_search.radius_px = debug_info->search_radius_px;
  draw_search_area(fallback_search, 0);

  cv::circle(
    image,
    debug_info->predicted_companion_center,
    7,
    cv::Scalar(220, 90, 255),
    2,
    cv::LINE_AA);
  cv::drawMarker(
    image,
    debug_info->predicted_companion_center,
    cv::Scalar(0, 220, 255),
    cv::MARKER_CROSS,
    18,
    2,
    cv::LINE_AA);
  cv::putText(
    image,
    "2?",
    debug_info->predicted_companion_center + cv::Point2f(10.0F, -8.0F),
    cv::FONT_HERSHEY_SIMPLEX,
    0.56,
    cv::Scalar(220, 90, 255),
    2,
    cv::LINE_AA);
}

void drawDepthWindowPeakOverlay(cv::Mat &image, const DepthWindowPeakInfo &peak_info)
{
  if (image.empty() || !peak_info.valid)
  {
    return;
  }
  if (
    peak_info.pixel.x < 0 || peak_info.pixel.y < 0 ||
    peak_info.pixel.x >= image.cols || peak_info.pixel.y >= image.rows)
  {
    return;
  }

  cv::circle(image, peak_info.pixel, 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  cv::circle(image, peak_info.pixel, 3, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);

  const std::string label_text = cv::format("Peak %.1f mm", peak_info.peak_height_m * 1000.0F);
  const cv::Point text_origin(
    std::clamp(peak_info.pixel.x + 16, 10, std::max(10, image.cols - 220)),
    std::clamp(peak_info.pixel.y - 14, 22, std::max(22, image.rows - 10)));
  cv::putText(
    image,
    label_text,
    text_origin,
    cv::FONT_HERSHEY_DUPLEX,
    0.55,
    cv::Scalar(0, 0, 255),
    2,
    cv::LINE_AA);
  cv::putText(
    image,
    label_text,
    text_origin,
    cv::FONT_HERSHEY_DUPLEX,
    0.55,
    cv::Scalar(255, 255, 255),
    1,
    cv::LINE_AA);
}
}  // namespace

class ItemDetectNode : public rclcpp::Node
{
public:
  enum class DisplayView
  {
    kRgb = 0,
    kBinarized,
    kDepth,
  };

  ItemDetectNode()
  : Node("item_detect")
  {
    profiles_dir_ = declare_parameter<std::string>(
      "profiles_dir",
      dobot_common::paths::workspacePath({"teach", "item_teach"}, __FILE__).string());
    const std::string selected_profile_path_param = declare_parameter<std::string>(
      "selected_profile_path",
      "");
    color_topic_ = declare_parameter<std::string>("color_topic", "/bin_camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/bin_camera/depth/image_raw");
	    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/bin_camera/color/camera_info");
	    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "bin_overlay");
	    camera_control_service_root_ = declare_parameter<std::string>(
	      "camera_control_service_root",
	      "/bin_camera");
	    normalizeCameraControlServiceRoot();
	    const int color_exposure_percent = clampExposurePercent(
	      declare_parameter<int>("color_exposure_percent", 0));
	    const int depth_exposure_percent = clampExposurePercent(
	      declare_parameter<int>("depth_exposure_percent", 0));
	    color_exposure_min_us_ = clampExposureUsec(
	      declare_parameter<int>("color_exposure_min_us", kDefaultExposureMinUs));
	    color_exposure_max_us_ = std::max(
	      color_exposure_min_us_,
	      clampExposureUsec(declare_parameter<int>("color_exposure_max_us", kDefaultExposureMaxUs)));
	    depth_exposure_min_us_ = clampExposureUsec(
	      declare_parameter<int>("depth_exposure_min_us", kDefaultExposureMinUs));
	    depth_exposure_max_us_ = std::max(
	      depth_exposure_min_us_,
	      clampExposureUsec(declare_parameter<int>("depth_exposure_max_us", kDefaultExposureMaxUs)));
	    color_exposure_us_ = clampExposureUsecOrAuto(
	      declare_parameter<int>(
	        "color_exposure_us",
	        exposurePercentToUsec(color_exposure_percent, color_exposure_min_us_, color_exposure_max_us_)),
	      color_exposure_min_us_,
	      color_exposure_max_us_);
	    depth_exposure_us_ = clampExposureUsecOrAuto(
	      declare_parameter<int>(
	        "depth_exposure_us",
	        exposurePercentToUsec(depth_exposure_percent, depth_exposure_min_us_, depth_exposure_max_us_)),
	      depth_exposure_min_us_,
	      depth_exposure_max_us_);
	    depth_exposure_us_ = 0;
	    seek_pose_topic_ = declare_parameter<std::string>("bin_pose_topic", "bin_seek_pose");
    item_pose_array_topic_ = declare_parameter<std::string>(
      "bin_item_pose_array_topic",
      "bin_item_poses");
    item_cube_marker_topic_ = declare_parameter<std::string>("bin_cube_marker_topic", "bin_cube_marker");
    seek_service_name_ = declare_parameter<std::string>("seek_service", "item_detect/seek");
    repick_service_name_ = declare_parameter<std::string>("repick_service", "item_detect/repick");
    go_to_teach_service_name_ = declare_parameter<std::string>(
      "go_to_teach_service",
      "item_detect/go_to_teach");
    publish_item_cube_marker_ = declare_parameter<bool>("publish_bin_cube_marker", true);
    item_marker_thickness_mm_ = std::max(0.1, declare_parameter<double>("bin_thickness_mm", 15.0));
    movj_service_name_ = declare_parameter<std::string>("movj_service", "/dobot_bringup_ros2/srv/MovJ");
    seek_complete_service_name_ = declare_parameter<std::string>(
      "seek_complete_service",
      "item_detect/seek_complete");
    seek_status_service_name_ = declare_parameter<std::string>(
      "seek_status_service",
      "item_detect/seek_status");
    use_calibration_ = declare_parameter<bool>("use_calibration", true);
    publish_static_calibration_tf_ = declare_parameter<bool>("publish_static_calibration_tf", true);
    align_item_z_axis_to_depth_plane_ = declare_parameter<bool>(
      "align_item_z_axis_to_depth_plane",
      true);
    calibration_parent_frame_ = declare_parameter<std::string>("calibration_parent_frame", "base_link");
    calibration_child_frame_ = declare_parameter<std::string>(
      "calibration_child_frame", "bin_calibrated_camera_link");
    calibration_dir_ = declare_parameter<std::string>(
      "calibration_dir", defaultCalibrationDir());
    calibration_file_ = declare_parameter<std::string>("calibration_file", "");
    robot_ip_address_ = dobot_common::robot_identity::resolveRobotIpAddress(
      declare_parameter<std::string>("robot_ip_address", ""), __FILE__);
    auto_discover_calibration_ = declare_parameter<bool>("auto_discover_calibration", true);
    const std::string runtime_settings_file_param = declare_parameter<std::string>(
      "runtime_settings_file",
      defaultRuntimeSettingsFile(profiles_dir_));
    runtime_settings_path_ = resolvePath(runtime_settings_file_param);
    const std::string selected_profile_export_file_param = declare_parameter<std::string>(
      "selected_profile_export_file",
      defaultSelectedProfileExportFile(profiles_dir_));
    selected_profile_export_path_ = resolvePath(selected_profile_export_file_param);
    selected_profile_topic_ = declare_parameter<std::string>(
      "selected_profile_topic",
      "item_detect/selected_profile");
    const std::string default_camera_frame =
      use_calibration_ ? calibration_child_frame_ : std::string("");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame", default_camera_frame);
    item_frame_id_ = declare_parameter<std::string>("bin_frame_id", "bin");
    motion_update_period_sec_ = declare_parameter<double>("motion_update_period_sec", 0.1);
    seek_motion_history_max_samples_ = static_cast<std::size_t>(kSeekMotionHistoryMaxSamples);
    const double seek_window_sec = declare_parameter<double>("seek_window_sec", 60.0);
    seek_window_tenths_ = std::clamp(static_cast<int>(std::round(seek_window_sec)), 1, 60);
    const double seek_decay_sec = declare_parameter<double>("seek_decay_sec", 1.0);
    seek_decay_tenths_ = std::clamp(static_cast<int>(std::round(seek_decay_sec * 10.0)), 1, 10);
    seek_snapshots_dir_ = declare_parameter<std::string>(
      "seek_snapshots_dir",
      dobot_common::paths::workspacePath({"debug files", "seek_frames"}, __FILE__).string());
    publish_overlay_ = declare_parameter<bool>("publish_overlay", true);
    headless_ = declare_parameter<bool>("headless", false);
    const bool start_visualization = declare_parameter<bool>("start_visualization", true);
    display_view_ = start_visualization ? DisplayView::kRgb : DisplayView::kBinarized;
    const std::string detection_mode_param =
      declare_parameter<std::string>("detection_mode", "rgb");
    detection_use_depth_ = isDepthDetectionMode(detection_mode_param);
    depth_threshold_mm_ = std::clamp(
      static_cast<int>(declare_parameter<int>("depth_threshold_mm", 10)),
      kDepthThresholdMinMm,
      kDepthThresholdMaxMm);
    const auto parameter_overrides = this->get_node_parameters_interface()->get_parameter_overrides();
    const auto declare_double_param = [this, &parameter_overrides](
      const std::string &name,
      double default_value) -> double
    {
      const auto override_it = parameter_overrides.find(name);
      if (
        override_it != parameter_overrides.end() &&
        override_it->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        return static_cast<double>(declare_parameter<int64_t>(
          name,
          static_cast<int64_t>(std::llround(default_value))));
      }
      return declare_parameter<double>(name, default_value);
    };
    depth_plane_model_.valid = declare_parameter<bool>("depth_plane_enabled", false);
    depth_plane_model_.a = declare_double_param("depth_plane_a", 0.0);
    depth_plane_model_.b = declare_double_param("depth_plane_b", 0.0);
    depth_plane_model_.c = declare_double_param("depth_plane_c", 0.0);
    depth_plane_model_.reference_depth_m = declare_double_param("depth_plane_reference_depth_m", 0.0);
    const auto depth_plane_roi_values = declare_parameter<std::vector<int64_t>>(
      "depth_plane_roi",
      std::vector<int64_t>{0, 0, 0, 0});
    if (depth_plane_roi_values.size() >= 4)
    {
      AxisAlignedRoiBounds roi{
        static_cast<int>(depth_plane_roi_values[0]),
        static_cast<int>(depth_plane_roi_values[1]),
        static_cast<int>(depth_plane_roi_values[2]),
        static_cast<int>(depth_plane_roi_values[3]),
      };
      if (isValidRoiBounds(roi))
      {
        depth_plane_roi_bounds_ = roi;
      }
    }
    if (
      !std::isfinite(depth_plane_model_.a) ||
      !std::isfinite(depth_plane_model_.b) ||
      !std::isfinite(depth_plane_model_.c) ||
      !std::isfinite(depth_plane_model_.reference_depth_m) ||
      depth_plane_model_.reference_depth_m <= 0.0 ||
      !depth_plane_roi_bounds_.has_value())
    {
      depth_plane_model_ = DepthPlaneModel{};
    }
    red_threshold_ = declare_parameter<int>("red_threshold", 120);
    green_threshold_ = declare_parameter<int>("green_threshold", 120);
    blue_threshold_ = declare_parameter<int>("blue_threshold", 120);
    rgb_hole_fill_sensitivity_ = std::clamp(
      static_cast<int>(declare_parameter<int>("rgb_hole_fill_sensitivity", 0)),
      kRgbHoleFillMin,
      kRgbHoleFillMax);
    rgb_mask_dilate_px_ = std::clamp(
      static_cast<int>(declare_parameter<int>("rgb_mask_dilate_px", kRgbDilateMinPx)),
      kRgbDilateMinPx,
      kRgbDilateMaxPx);
    depth_null_fill_sensitivity_ = std::clamp(
      static_cast<int>(declare_parameter<int>("depth_null_fill_sensitivity", 0)),
      kDepthFillSensitivityMin,
      kDepthFillSensitivityMax);
    depth_window_mm_ = std::clamp(
      static_cast<int>(declare_parameter<int>("depth_window_mm", 5)),
      kDepthWindowMinMm,
      kDepthWindowMaxMm);
    depth_hole_fill_sensitivity_ = std::clamp(
      static_cast<int>(declare_parameter<int>("depth_hole_fill_sensitivity", 0)),
      kDepthFillSensitivityMin,
      kDepthFillSensitivityMax);
    depth_trim_px_ = std::clamp(
      static_cast<int>(declare_parameter<int>("depth_trim_px", 0)),
      kDepthTrimMinPx,
      kDepthTrimMaxPx);
    if (const auto add_px_override = parameter_overrides.find("adaptive_depth_trim_max_add_px");
        add_px_override != parameter_overrides.end())
    {
      adaptive_depth_trim_max_add_px_ = clampAdaptiveDepthTrimAddPx(
        static_cast<int>(declare_parameter<int>(
            "adaptive_depth_trim_max_add_px",
            kAdaptiveDepthTrimAddDefaultPx)));
    }
    else
    {
      adaptive_depth_trim_max_add_px_ = clampAdaptiveDepthTrimAddPx(
        static_cast<int>(declare_parameter<int>(
            "adaptive_depth_trim_max_add_px",
            kAdaptiveDepthTrimAddDefaultPx)));
    }
    adaptive_depth_trim_max_height_mm_ = clampAdaptiveDepthTrimHeightMm(
      static_cast<int>(declare_parameter<int>(
          "adaptive_depth_trim_max_height_mm",
          kAdaptiveDepthTrimHeightDefaultMm)));
    focus_black_mask_ = declare_parameter<bool>("focus_black_mask", false);
    ray_step_px_ = declare_parameter<int>("ray_step_px", 3);
    depth_edge_offset_px_ = std::clamp(
      static_cast<int>(declare_parameter<int>("depth_edge_offset_px", 4)),
      kDepthEdgeOffsetMinPx,
      kDepthEdgeOffsetMaxPx);
    previous_color_percent_ = std::clamp(
      static_cast<int>(declare_parameter<int>("previous_color_percent", kDefaultPreviousColorPercent)),
      20,
      100);
    horizontal_ray_count_ = std::clamp(
      static_cast<int>(declare_parameter<int>("horizontal_ray_count", 50)),
      50,
      100);
    vertical_ray_count_ = std::clamp(
      static_cast<int>(declare_parameter<int>("vertical_ray_count", 50)),
      50,
      150);
    outlier_sensitivity_ = std::clamp(
      static_cast<int>(declare_parameter<int>("outlier_sensitivity", 50)),
      1,
      100);
    detect_black_to_white_ = declare_parameter<bool>("detect_black_to_white", !focus_black_mask_);
    trace_out_to_in_ = declare_parameter<bool>("trace_out_to_in", false);
    blob_tolerance_percent_ = std::clamp(
      static_cast<int>(declare_parameter<int>("blob_tolerance_percent", kBlobToleranceDefaultPercent)),
      kBlobToleranceMinPercent,
      kBlobToleranceMaxPercent);
    item_name_ = declare_parameter<std::string>("item_name", "item");
    teach_date_ = declare_parameter<std::string>("teach_date", "");
    if (motion_update_period_sec_ <= 0.0)
    {
      motion_update_period_sec_ = 0.1;
    }
    if (use_calibration_)
    {
      if (calibration_file_.empty() && auto_discover_calibration_)
      {
        calibration_file_ = findLatestCalibrationFile();
      }
      if (calibration_file_.empty())
      {
        throw std::runtime_error(
                "use_calibration=true but no calibration file is available. "
                "Set calibration_file or add a YAML to calibration_dir.");
      }

      std::string reason;
      if (!loadCalibrationFromFile(calibration_file_, reason))
      {
        throw std::runtime_error(
                "Failed to load calibration file '" + calibration_file_ + "': " + reason);
      }

      if (!camera_frame_id_.empty() && camera_frame_id_ != calibration_child_frame_)
      {
        RCLCPP_WARN(
          get_logger(),
          "camera_frame (%s) differs from calibration_child_frame (%s). "
          "Using calibration_child_frame for item outputs.",
          camera_frame_id_.c_str(), calibration_child_frame_.c_str());
      }
      camera_frame_id_ = calibration_child_frame_;

      if (publish_static_calibration_tf_)
      {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishCalibrationTransform();
      }
    }

    refreshItemProfiles();
    bool selected_profile_loaded = false;
    const std::filesystem::path launch_selected_profile_path = resolvePath(selected_profile_path_param);
    if (!launch_selected_profile_path.empty())
    {
      selected_profile_loaded = selectProfileFromFile(launch_selected_profile_path, false);
      if (!selected_profile_loaded)
      {
        RCLCPP_WARN(
          get_logger(),
          "Launch-selected item detect profile could not be loaded: %s",
          launch_selected_profile_path.c_str());
      }
    }
    if (!selected_profile_loaded)
    {
      selectInitialProfile();
    }
    loadRuntimeUiSettings();
    saveRuntimeUiSettings();
    saveSelectedProfileExportFile();
    createRosInterfaces();
    movj_client_ = create_client<MovJSrv>(movj_service_name_);
    createCameraExposureClients();
    camera_exposure_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&ItemDetectNode::applyPendingCameraExposureSettings, this));

    if (!headless_)
    {
      cv::namedWindow(kDetectWindowName, cv::WINDOW_NORMAL);
      cv::resizeWindow(kDetectWindowName, kPreviewCanvasWidth, kTopBarBaseHeight + kPreviewCanvasHeight);
      cv::setMouseCallback(kDetectWindowName, &ItemDetectNode::onMouseThunk, this);
      visualization_window_created_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "item_detect ready. Overlay topic=%s seek_pose topic=%s item_pose_array topic=%s item_marker topic=%s "
      "(enabled=%s thickness=%.1fmm) detect_mode=%s depth_threshold=+/- %dmm depth_plane=%s "
      "z_axis_align=%s movj_service=%s seek_service=%s repick_service=%s go_to_teach_service=%s "
      "selected_profile=%s profiles=%zu headless=%s",
      overlay_topic_.c_str(),
      seek_pose_topic_.c_str(),
      item_pose_array_topic_.c_str(),
      item_cube_marker_topic_.c_str(),
      publish_item_cube_marker_ ? "true" : "false",
      item_marker_thickness_mm_,
      detectionModeToString(detection_use_depth_).c_str(),
      depth_threshold_mm_,
      depth_plane_model_.valid ? "fixed" : "missing",
      align_item_z_axis_to_depth_plane_ ? "depth-plane" : "off",
      movj_service_name_.c_str(),
      seek_service_name_.c_str(),
      repick_service_name_.c_str(),
      go_to_teach_service_name_.c_str(),
	      selectedProfileDisplayText().c_str(),
	      item_profiles_.size(),
      headless_ ? "true" : "false");
	    RCLCPP_INFO(
	      get_logger(),
	      "item_detect camera exposure controls. service_root=%s color=%s depth=%s",
	      camera_control_service_root_.c_str(),
	      exposureModeText(color_exposure_us_).c_str(),
	      exposureModeText(depth_exposure_us_).c_str());
    if (use_calibration_)
    {
      RCLCPP_INFO(
        get_logger(),
        "Calibration loaded from %s. type=%s yaml_tracking_base=%s publishing %s -> %s in-node: %s",
        calibration_file_.c_str(),
        calibration_metadata_.calibration_type.empty() ? "<missing>" : calibration_metadata_.calibration_type.c_str(),
        calibration_metadata_.tracking_base_frame.empty() ? "<missing>" : calibration_metadata_.tracking_base_frame.c_str(),
        calibration_parent_frame_.c_str(),
        calibration_child_frame_.c_str(),
        publish_static_calibration_tf_ ? "enabled" : "disabled");
    }
    else
    {
      RCLCPP_INFO(
        get_logger(),
        "Calibration disabled. Item outputs use camera frame resolved from parameter/header.");
    }
  }

  ~ItemDetectNode() override
  {
    saveRuntimeUiSettings();
    if (visualization_window_created_)
    {
      destroyOpenCvWindowQuietly(kDetectWindowName);
    }
  }

private:
  struct UiButton
  {
    std::string label;
    cv::Rect rect;
  };

  struct UiSlider
  {
    std::string label;
    cv::Rect track_rect;
    int min_value {1};
    int max_value {20};
  };

  void processWindowEvents()
  {
    if (headless_ || !visualization_window_created_)
    {
      return;
    }
    cv::waitKey(1);
    if (isOpenCvWindowClosed(kDetectWindowName))
    {
      requestShutdownFromWindowClose();
    }
  }

  void requestShutdownFromWindowClose()
  {
    if (window_close_requested_)
    {
      return;
    }
    window_close_requested_ = true;
    RCLCPP_INFO(get_logger(), "item_detect window closed; shutting down.");
    saveRuntimeUiSettings();
    if (camera_status_timer_)
    {
      camera_status_timer_->cancel();
    }
    if (camera_exposure_timer_)
    {
      camera_exposure_timer_->cancel();
    }
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  struct ItemSummary
  {
    int detected_items {0};
    bool has_best_candidate {false};
    cv::Vec3d best_candidate_position_m {0.0, 0.0, 0.0};
    std::string frame_id;
  };

  struct SeekMotionData
  {
    double dt_sec {0.0};
    cv::Vec3d delta_position_mm {0.0, 0.0, 0.0};
    cv::Vec3d velocity_camera_mmps {0.0, 0.0, 0.0};
    double speed_camera_mmps {0.0};
    cv::Vec3d direction_camera {0.0, 0.0, 0.0};
    cv::Vec3d velocity_child_mmps {0.0, 0.0, 0.0};
    double speed_child_mmps {0.0};
    cv::Vec3d direction_child {0.0, 0.0, 0.0};
  };

  SeekMotionData buildSeekMotionData(
    const cv::Vec3d &delta_position_m,
    double dt_sec,
    const cv::Matx33d &last_pose_rotation) const
  {
    SeekMotionData data;
    data.dt_sec = std::max(0.0, dt_sec);
    const cv::Vec3d velocity_camera_mps = data.dt_sec > 1e-6
      ? (delta_position_m * (1.0 / data.dt_sec))
      : cv::Vec3d(0.0, 0.0, 0.0);
    const cv::Vec3d velocity_child_mps = last_pose_rotation.t() * velocity_camera_mps;
    const double speed_camera_mps = cv::norm(velocity_camera_mps);
    const double speed_child_mps = cv::norm(velocity_child_mps);

    data.delta_position_mm = delta_position_m * kMetersToMillimeters;
    data.velocity_camera_mmps = velocity_camera_mps * kMetersToMillimeters;
    data.velocity_child_mmps = velocity_child_mps * kMetersToMillimeters;
    data.speed_camera_mmps = speed_camera_mps * kMetersToMillimeters;
    data.speed_child_mmps = speed_child_mps * kMetersToMillimeters;
    data.direction_camera = speed_camera_mps > 1e-9
      ? (velocity_camera_mps * (1.0 / speed_camera_mps))
      : cv::Vec3d(0.0, 0.0, 0.0);
    data.direction_child = speed_child_mps > 1e-9
      ? (velocity_child_mps * (1.0 / speed_child_mps))
      : cv::Vec3d(0.0, 0.0, 0.0);
    return data;
  }

  static std::string defaultCalibrationDir()
  {
    return dobot_common::paths::workspacePath({"calibration"}, __FILE__).string();
  }

  static std::string defaultRuntimeSettingsFile(const std::string &profiles_dir)
  {
    (void)profiles_dir;
    return dobot_common::paths::workspacePath(
      {"config", "item_perception", "item_detect_runtime_settings.yaml"}, __FILE__).string();
  }

  static std::string defaultSelectedProfileExportFile(const std::string &profiles_dir)
  {
    (void)profiles_dir;
    return dobot_common::paths::workspacePath(
      {"config", "item_perception", "item_detect_selected_profile.txt"}, __FILE__).string();
  }

  static std::filesystem::path resolvePath(const std::string &path_text)
  {
    if (path_text.empty())
    {
      return {};
    }
    if (path_text[0] != '~')
    {
      return std::filesystem::path(path_text);
    }

    const char *home = std::getenv("HOME");
    if (home == nullptr)
    {
      return std::filesystem::path(path_text);
    }
    if (path_text == "~")
    {
      return std::filesystem::path(home);
    }
    if (path_text.rfind("~/", 0) == 0)
    {
      return std::filesystem::path(home) / path_text.substr(2);
    }
    return std::filesystem::path(path_text);
  }

  std::string findLatestCalibrationFile() const
  {
    try
    {
      const std::filesystem::path base = resolvePath(calibration_dir_);
      if (!std::filesystem::exists(base) || !std::filesystem::is_directory(base))
      {
        return {};
      }

      dobot_common::robot_identity::LatestRobotFileSelection selection;
      for (const auto &entry : std::filesystem::directory_iterator(base))
      {
        if (!entry.is_regular_file())
        {
          continue;
        }
        const auto &p = entry.path();
        if (p.extension() != ".yaml")
        {
          continue;
        }
        if (std::filesystem::file_size(p) == 0)
        {
          continue;
        }
        const std::string filename = p.filename().string();
        if (filename.rfind("axab_calibration_eyetohand_", 0) != 0)
        {
          continue;
        }
        selection.consider(p, entry.last_write_time(), robot_ip_address_);
      }
      return selection.selected().string();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Failed to discover calibration files: %s", ex.what());
      return {};
    }
  }

  static std::string yamlScalarString(const YAML::Node &node)
  {
    if (!node || !node.IsScalar())
    {
      return {};
    }
    try
    {
      return node.as<std::string>();
    }
    catch (const std::exception &)
    {
      return {};
    }
  }

  static std::string normalizeCalibrationType(std::string type)
  {
    std::string normalized;
    normalized.reserve(type.size());
    for (const unsigned char ch : type)
    {
      if (std::isalnum(ch) != 0)
      {
        normalized.push_back(static_cast<char>(std::tolower(ch)));
      }
      else if (ch == '_' || ch == '-')
      {
        normalized.push_back('_');
      }
    }
    return normalized;
  }

  bool loadCalibrationFromFile(const std::string &file_path, std::string &reason)
  {
    const auto resolved_path = resolvePath(file_path);
    try
    {
      if (!std::filesystem::exists(resolved_path))
      {
        reason = "File does not exist";
        return false;
      }
      if (std::filesystem::file_size(resolved_path) == 0)
      {
        reason = "Calibration file is empty";
        return false;
      }
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Filesystem error: ") + ex.what();
      return false;
    }

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(resolved_path.string());
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Could not read YAML: ") + ex.what();
      return false;
    }

    const auto calib = root["transform"];
    if (!calib)
    {
      reason = "Missing 'transform' key";
      return false;
    }
    const auto rot = calib["rotation"];
    const auto trans = calib["translation"];
    if (!rot || !trans)
    {
      reason = "Missing rotation/translation keys";
      return false;
    }

    CameraCalibrationMetadata metadata;
    if (const auto params = root["parameters"]; params && params.IsMap())
    {
      metadata.calibration_type = yamlScalarString(params["calibration_type"]);
      metadata.robot_base_frame = yamlScalarString(params["robot_base_frame"]);
      metadata.transform_child_frame = yamlScalarString(params["transform_child_frame"]);
      metadata.tracking_base_frame = yamlScalarString(params["tracking_base_frame"]);
    }

    // camera_calibration writes eye-to-hand as eye_on_base and broadcasts the
    // transform directly from robot_base_frame to the saved calibrated camera frame.
    const std::string normalized_type = normalizeCalibrationType(metadata.calibration_type);
    if (normalized_type != "eye_on_base")
    {
      reason = "Expected eye-to-hand calibration YAML with parameters.calibration_type=eye_on_base, got '" +
        (metadata.calibration_type.empty() ? std::string("<missing>") : metadata.calibration_type) + "'";
      return false;
    }

    if (metadata.robot_base_frame.empty())
    {
      metadata.robot_base_frame = calibration_parent_frame_;
      RCLCPP_WARN(
        get_logger(),
        "Calibration YAML %s has no parameters.robot_base_frame; keeping configured parent frame %s.",
        resolved_path.string().c_str(),
        calibration_parent_frame_.c_str());
    }
    else if (metadata.robot_base_frame != calibration_parent_frame_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Calibration YAML robot_base_frame is %s but item_detect was configured with parent %s. "
        "Using YAML parent frame so eye-to-hand TF matches camera_calibration.",
        metadata.robot_base_frame.c_str(),
        calibration_parent_frame_.c_str());
      calibration_parent_frame_ = metadata.robot_base_frame;
    }
    if (!metadata.transform_child_frame.empty() && metadata.transform_child_frame != calibration_child_frame_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Calibration YAML transform_child_frame is %s but item_detect was configured with child %s. "
        "Using YAML child frame so eye-to-hand TF matches camera_calibration.",
        metadata.transform_child_frame.c_str(),
        calibration_child_frame_.c_str());
      calibration_child_frame_ = metadata.transform_child_frame;
    }

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    try
    {
      qw = rot["w"].as<double>();
      qx = rot["x"].as<double>();
      qy = rot["y"].as<double>();
      qz = rot["z"].as<double>();
      calibration_translation_.x = trans["x"].as<double>();
      calibration_translation_.y = trans["y"].as<double>();
      calibration_translation_.z = trans["z"].as<double>();
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Failed to parse rotation/translation: ") + ex.what();
      return false;
    }

    const double norm = std::sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw));
    if (norm < 1e-9)
    {
      reason = "Invalid quaternion (zero norm)";
      return false;
    }
    const double inv = 1.0 / norm;
    calibration_rotation_.x = qx * inv;
    calibration_rotation_.y = qy * inv;
    calibration_rotation_.z = qz * inv;
    calibration_rotation_.w = qw * inv;
    calibration_metadata_ = metadata;
    return true;
  }

  void publishCalibrationTransform()
  {
    if (!static_tf_broadcaster_)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = calibration_parent_frame_;
    tf_msg.child_frame_id = calibration_child_frame_;
    tf_msg.transform.translation = calibration_translation_;
    tf_msg.transform.rotation = calibration_rotation_;
    static_tf_broadcaster_->sendTransform(tf_msg);
  }

	  void markCameraExposureDirty()
	  {
	    camera_exposure_dirty_ = true;
	  }

	  void normalizeCameraControlServiceRoot()
	  {
	    if (camera_control_service_root_.empty())
	    {
	      camera_control_service_root_ = "/bin_camera";
	    }
	    while (camera_control_service_root_.size() > 1 && camera_control_service_root_.back() == '/')
	    {
	      camera_control_service_root_.pop_back();
	    }
	    if (camera_control_service_root_.front() != '/')
	    {
	      camera_control_service_root_ = "/" + camera_control_service_root_;
	    }
	  }

	  std::string cameraControlServiceName(const std::string &leaf) const
	  {
	    return camera_control_service_root_ + "/" + leaf;
	  }

	  void createCameraExposureClients()
	  {
	    color_auto_exposure_client_ =
	      create_client<SetBoolSrv>(cameraControlServiceName("set_color_auto_exposure"));
	    color_exposure_client_ =
	      create_client<SetInt32Srv>(cameraControlServiceName("set_color_exposure"));
	    depth_auto_exposure_client_ =
	      create_client<SetBoolSrv>(cameraControlServiceName("set_depth_auto_exposure"));
	    depth_exposure_client_ =
	      create_client<SetInt32Srv>(cameraControlServiceName("set_depth_exposure"));
	    markCameraExposureDirty();
	  }

	  std::string exposureModeText(int exposure_us) const
	  {
	    if (exposure_us <= 0)
	    {
	      return "auto";
	    }
	    return std::to_string(exposure_us) + "us";
	  }

	  bool applyCameraExposureSetting(
	    const std::string &label,
	    int exposure_us,
	    const rclcpp::Client<SetBoolSrv>::SharedPtr &auto_client,
	    const rclcpp::Client<SetInt32Srv>::SharedPtr &exposure_client,
	    int &last_applied_exposure_us)
	  {
	    if (last_applied_exposure_us == exposure_us)
	    {
	      return true;
	    }
	    if (!auto_client || !auto_client->service_is_ready())
	    {
	      return false;
	    }
	    if (exposure_us > 0 && (!exposure_client || !exposure_client->service_is_ready()))
	    {
	      return false;
	    }

	    auto auto_request = std::make_shared<SetBoolSrv::Request>();
	    auto_request->data = exposure_us <= 0;
	    auto_client->async_send_request(
	      auto_request,
	      [this, label](rclcpp::Client<SetBoolSrv>::SharedFuture future)
	      {
	        try
	        {
	          const auto response = future.get();
	          if (!response || !response->success)
	          {
	            RCLCPP_WARN(
	              get_logger(),
	              "%s auto exposure request failed: %s",
	              label.c_str(),
	              response ? response->message.c_str() : "no response");
	          }
	        }
	        catch (const std::exception &ex)
	        {
	          RCLCPP_WARN(get_logger(), "%s auto exposure request error: %s", label.c_str(), ex.what());
	        }
	      });

	    if (exposure_us > 0)
	    {
	      auto exposure_request = std::make_shared<SetInt32Srv::Request>();
	      exposure_request->data = exposure_us;
	      exposure_client->async_send_request(
	        exposure_request,
	        [this, label, exposure_us](rclcpp::Client<SetInt32Srv>::SharedFuture future)
	        {
	          try
	          {
	            const auto response = future.get();
	            if (!response || !response->success)
	            {
	              RCLCPP_WARN(
	                get_logger(),
	                "%s exposure %dus request failed: %s",
	                label.c_str(),
	                exposure_us,
	                response ? response->message.c_str() : "no response");
	            }
	          }
	          catch (const std::exception &ex)
	          {
	            RCLCPP_WARN(get_logger(), "%s exposure request error: %s", label.c_str(), ex.what());
	          }
	        });
	    }

	    last_applied_exposure_us = exposure_us;
	    RCLCPP_INFO(
	      get_logger(),
	      "%s exposure set to %s",
	      label.c_str(),
	      exposureModeText(exposure_us).c_str());
	    return true;
	  }

	  void applyPendingCameraExposureSettings()
	  {
	    if (!camera_exposure_dirty_)
	    {
	      return;
	    }

	    const rclcpp::Time now = this->now();
	    if (last_camera_exposure_attempt_time_.nanoseconds() != 0 &&
	        (now - last_camera_exposure_attempt_time_).seconds() < 0.5)
	    {
	      return;
	    }
	    last_camera_exposure_attempt_time_ = now;

	    const bool color_ok = applyCameraExposureSetting(
	      "RGB",
	      color_exposure_us_,
	      color_auto_exposure_client_,
	      color_exposure_client_,
	      last_applied_color_exposure_us_);
	    depth_exposure_us_ = 0;
	    const bool depth_ok = applyCameraExposureSetting(
	      "Depth",
	      depth_exposure_us_,
	      depth_auto_exposure_client_,
	      depth_exposure_client_,
	      last_applied_depth_exposure_us_);

	    camera_exposure_dirty_ = !(color_ok && depth_ok);
	  }

	  void createRosInterfaces()
	  {
    overlay_pub_ = create_publisher<ImageMsg>(overlay_topic_, rclcpp::QoS(5));
    seek_pose_pub_ = create_publisher<PoseStampedMsg>(seek_pose_topic_, rclcpp::QoS(10));
    item_pose_array_pub_ = create_publisher<PoseArrayMsg>(item_pose_array_topic_, rclcpp::QoS(10));
    item_cube_marker_pub_ = create_publisher<MarkerMsg>(
      item_cube_marker_topic_,
      rclcpp::QoS(1).reliable().transient_local());
    selected_profile_pub_ = create_publisher<StringMsg>(
      selected_profile_topic_,
      rclcpp::QoS(1).reliable().transient_local());
    color_sub_ = create_subscription<ImageMsg>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ItemDetectNode::colorCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<ImageMsg>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ItemDetectNode::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<CameraInfoMsg>(
      camera_info_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&ItemDetectNode::cameraInfoCallback, this, std::placeholders::_1));
    if (!seek_service_)
    {
      seek_service_ = create_service<TriggerSrv>(
        seek_service_name_,
        std::bind(
          &ItemDetectNode::handleSeekService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!seek_complete_service_)
    {
      seek_complete_service_ = create_service<TriggerSrv>(
        seek_complete_service_name_,
        std::bind(
          &ItemDetectNode::handleSeekCompleteService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!repick_service_)
    {
      repick_service_ = create_service<TriggerSrv>(
        repick_service_name_,
        std::bind(
          &ItemDetectNode::handleRepickService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!seek_status_service_)
    {
      seek_status_service_ = create_service<TriggerSrv>(
        seek_status_service_name_,
        std::bind(
          &ItemDetectNode::handleSeekStatusService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!camera_status_timer_)
    {
      camera_status_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ItemDetectNode::renderNoCameraTopicsOverlay, this));
    }
    if (!go_to_teach_service_)
    {
      go_to_teach_service_ = create_service<TriggerSrv>(
        go_to_teach_service_name_,
        std::bind(
          &ItemDetectNode::handleGoToTeachService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    publishSelectedProfile();

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_.release();
    latest_camera_info_.reset();
  }

  void handleSeekService(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response)
  {
    (void)request;
    if (seek_mode_active_ || seek_result_latched_)
    {
      seek_mode_active_ = false;
      seek_result_latched_ = false;
      clearSeekResultFreeze();
      resetSeekSessionState();
      profile_status_message_ = "Seek cancelled";
      response->success = true;
      response->message = profile_status_message_;
      return;
    }

    seek_mode_active_ = true;
    seek_result_latched_ = false;
    clearSeekResultFreeze();
    resetSeekSessionState();
    profile_status_message_ = "Seek armed: waiting for pose publish";
    response->success = true;
    response->message = profile_status_message_;
  }

  void handleSeekCompleteService(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response)
  {
    (void)request;
    seek_mode_active_ = false;
    seek_result_latched_ = false;
    clearSeekResultFreeze();
    resetSeekSessionState();
    profile_status_message_ = "Seek released by item pick";
    response->success = true;
    response->message = profile_status_message_;
  }

  void handleRepickService(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response)
  {
    (void)request;
    if (seek_mode_active_)
    {
      response->success = false;
      response->message = "Repick rejected: seek is already acquiring";
      return;
    }
    if (!seek_result_latched_)
    {
      response->success = false;
      response->message = "Repick rejected: no latched seek result";
      return;
    }

    seek_mode_active_ = true;
    seek_result_latched_ = false;
    clearSeekResultFreeze();
    resetSeekSessionState();
    profile_status_message_ = "Repick requested: reacquiring item pose";
    response->success = true;
    response->message = profile_status_message_;
  }

  void handleSeekStatusService(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response)
  {
    (void)request;
    const bool active = seek_mode_active_ || seek_result_latched_;
    response->success = active;
    response->message = active ? "Seek: ON" : "Seek: OFF";
  }

  void handleGoToTeachService(
    const std::shared_ptr<TriggerSrv::Request> request,
    std::shared_ptr<TriggerSrv::Response> response)
  {
    (void)request;
    const bool started = requestGoToTeach();
    response->success = started;
    response->message = profile_status_message_;
  }

  cv::Matx33d quaternionToRotationMatrix(const geometry_msgs::msg::Quaternion &quaternion) const
  {
    double qw = quaternion.w;
    double qx = quaternion.x;
    double qy = quaternion.y;
    double qz = quaternion.z;
    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-12)
    {
      const double inv = 1.0 / norm;
      qw *= inv;
      qx *= inv;
      qy *= inv;
      qz *= inv;
    }

    const double xx = qx * qx;
    const double yy = qy * qy;
    const double zz = qz * qz;
    const double xy = qx * qy;
    const double xz = qx * qz;
    const double yz = qy * qz;
    const double wx = qw * qx;
    const double wy = qw * qy;
    const double wz = qw * qz;

    return cv::Matx33d(
      1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
      2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
      2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy));
  }

  std::optional<double> depthPlaneDepthAtPixel(
    const DepthPlaneModel &plane,
    const cv::Point2f &pixel,
    const cv::Size &image_size) const
  {
    if (!plane.valid || image_size.width <= 0 || image_size.height <= 0)
    {
      return std::nullopt;
    }

    const double clamped_x = std::clamp(
      static_cast<double>(pixel.x),
      0.0,
      static_cast<double>(std::max(0, image_size.width - 1)));
    const double clamped_y = std::clamp(
      static_cast<double>(pixel.y),
      0.0,
      static_cast<double>(std::max(0, image_size.height - 1)));
    const double x_norm = image_size.width <= 1
      ? 0.0
      : clamped_x / static_cast<double>(image_size.width - 1);
    const double y_norm = image_size.height <= 1
      ? 0.0
      : clamped_y / static_cast<double>(image_size.height - 1);
    const double plane_depth_m = (plane.a * x_norm) + (plane.b * y_norm) + plane.c;
    if (!std::isfinite(plane_depth_m) || plane_depth_m <= 0.0)
    {
      return std::nullopt;
    }
    return plane_depth_m;
  }

  std::optional<DetectedItemDimensions> measureBlobDimensionsOnDepthPlane(
    const BinarizedPoseEstimate2D::BlobPose2D &blob_pose,
    const cv::Size &image_size,
    const CameraInfoMsg &camera_info) const
  {
    if (
      blob_pose.corners.size() != 4 ||
      !depth_plane_model_.valid ||
      image_size.width <= 0 ||
      image_size.height <= 0 ||
      camera_info.k[0] <= 1e-6 ||
      camera_info.k[4] <= 1e-6)
    {
      return std::nullopt;
    }

    std::array<cv::Vec3d, 4> camera_points;
    for (std::size_t i = 0; i < blob_pose.corners.size(); ++i)
    {
      const auto depth = depthPlaneDepthAtPixel(depth_plane_model_, blob_pose.corners[i], image_size);
      if (!depth.has_value())
      {
        return std::nullopt;
      }
      camera_points[i] = projectPixelToCamera(blob_pose.corners[i], *depth, camera_info);
    }

    const int origin_idx = lowerLeftCornerIndex(blob_pose.corners);
    if (origin_idx < 0)
    {
      return std::nullopt;
    }

    const auto edge_length_mm = [&](int a, int b) -> double
    {
      return vectorNorm(
        camera_points[static_cast<std::size_t>(a)] -
        camera_points[static_cast<std::size_t>(b)]) * kMetersToMillimeters;
    };

    const int prev_idx = (origin_idx + 3) % 4;
    const int next_idx = (origin_idx + 1) % 4;
    const int opposite_idx = (origin_idx + 2) % 4;
    const double origin_prev_len_mm = edge_length_mm(origin_idx, prev_idx);
    const double next_opposite_len_mm = edge_length_mm(next_idx, opposite_idx);
    const double origin_next_len_mm = edge_length_mm(origin_idx, next_idx);
    const double prev_opposite_len_mm = edge_length_mm(prev_idx, opposite_idx);

    std::array<double, 4> ordered_edges_mm;
    if (origin_prev_len_mm >= origin_next_len_mm)
    {
      ordered_edges_mm = {
        origin_prev_len_mm,
        next_opposite_len_mm,
        origin_next_len_mm,
        prev_opposite_len_mm,
      };
    }
    else
    {
      ordered_edges_mm = {
        origin_next_len_mm,
        prev_opposite_len_mm,
        origin_prev_len_mm,
        next_opposite_len_mm,
      };
    }

    for (const double edge_mm : ordered_edges_mm)
    {
      if (!std::isfinite(edge_mm) || edge_mm <= 0.0)
      {
        return std::nullopt;
      }
    }

    const double x_size_mm = 0.5 * (ordered_edges_mm[0] + ordered_edges_mm[1]);
    const double y_size_mm = 0.5 * (ordered_edges_mm[2] + ordered_edges_mm[3]);
    const double area_m2 =
      triangleArea3D(camera_points[0], camera_points[1], camera_points[2]) +
      triangleArea3D(camera_points[0], camera_points[2], camera_points[3]);
    const double area_mm2 = area_m2 > 0.0
      ? area_m2 * kMetersToMillimeters * kMetersToMillimeters
      : x_size_mm * y_size_mm;
    if (
      !std::isfinite(x_size_mm) ||
      !std::isfinite(y_size_mm) ||
      !std::isfinite(area_mm2) ||
      x_size_mm <= 0.0 ||
      y_size_mm <= 0.0 ||
      area_mm2 <= 0.0)
    {
      return std::nullopt;
    }

    DetectedItemDimensions dimensions;
    dimensions.length_mm = std::max(x_size_mm, y_size_mm);
    dimensions.width_mm = std::min(x_size_mm, y_size_mm);
    dimensions.area_mm2 = area_mm2;
    dimensions.edge_lengths_mm = ordered_edges_mm;
    return dimensions;
  }

  bool detectedDimensionsWithinTaughtBand(
    const DetectedItemDimensions &dimensions,
    int tolerance_percent,
    std::string *status_text = nullptr) const
  {
    if (!has_taught_item_dimensions_)
    {
      if (status_text != nullptr)
      {
        *status_text = "Dimension gate unavailable: selected teach file has no item dimensions.";
      }
      return true;
    }

    const int clamped_tolerance_percent = std::clamp(
      tolerance_percent,
      kBlobToleranceMinPercent,
      kBlobToleranceMaxPercent);
    const double tolerance_fraction = static_cast<double>(clamped_tolerance_percent) / 100.0;
    const double length_lower = taught_item_length_mm_ * (1.0 - tolerance_fraction);
    const double length_upper = taught_item_length_mm_ * (1.0 + tolerance_fraction);
    const double width_lower = taught_item_width_mm_ * (1.0 - tolerance_fraction);
    const double width_upper = taught_item_width_mm_ * (1.0 + tolerance_fraction);
    const bool length_ok = dimensions.length_mm >= length_lower && dimensions.length_mm <= length_upper;
    const bool width_ok = dimensions.width_mm >= width_lower && dimensions.width_mm <= width_upper;

    if (status_text != nullptr)
    {
      const double length_error_percent = taught_item_length_mm_ > 1e-6
        ? 100.0 * std::fabs(dimensions.length_mm - taught_item_length_mm_) / taught_item_length_mm_
        : 0.0;
      const double width_error_percent = taught_item_width_mm_ > 1e-6
        ? 100.0 * std::fabs(dimensions.width_mm - taught_item_width_mm_) / taught_item_width_mm_
        : 0.0;
      *status_text = cv::format(
        "%s: measured %.1f x %.1f mm, taught %.1f x %.1f mm, err %.1f/%.1f%%, tol +/- %d%%",
        (length_ok && width_ok) ? "Dimension OK" : "Rejected size",
        dimensions.length_mm,
        dimensions.width_mm,
        taught_item_length_mm_,
        taught_item_width_mm_,
        length_error_percent,
        width_error_percent,
        clamped_tolerance_percent);
    }

    return length_ok && width_ok;
  }

  std::optional<cv::Vec3d> depthPlaneNormalInFrame(
    const DepthPlaneModel &plane,
    const CameraInfoMsg &camera_info,
    const cv::Size &image_size,
    const cv::Point2f &center_px,
    const std::optional<cv::Vec3d> &preferred_sign = std::nullopt) const
  {
    if (
      !align_item_z_axis_to_depth_plane_ ||
      !plane.valid ||
      image_size.width < 2 ||
      image_size.height < 2 ||
      camera_info.k[0] <= 1e-6 ||
      camera_info.k[4] <= 1e-6)
    {
      return std::nullopt;
    }

    const auto clamp_pixel = [&](const cv::Point2f &pixel)
    {
      return cv::Point2f(
        static_cast<float>(std::clamp(
          static_cast<double>(pixel.x),
          0.0,
          static_cast<double>(image_size.width - 1))),
        static_cast<float>(std::clamp(
          static_cast<double>(pixel.y),
          0.0,
          static_cast<double>(image_size.height - 1))));
    };

    const cv::Point2f p0_px = clamp_pixel(center_px);
    const float step_x = std::max(4.0F, static_cast<float>(image_size.width) * 0.02F);
    const float step_y = std::max(4.0F, static_cast<float>(image_size.height) * 0.02F);
    cv::Point2f px_px = clamp_pixel(p0_px + cv::Point2f(step_x, 0.0F));
    if (cv::norm(px_px - p0_px) < 1.0F)
    {
      px_px = clamp_pixel(p0_px - cv::Point2f(step_x, 0.0F));
    }
    cv::Point2f py_px = clamp_pixel(p0_px + cv::Point2f(0.0F, step_y));
    if (cv::norm(py_px - p0_px) < 1.0F)
    {
      py_px = clamp_pixel(p0_px - cv::Point2f(0.0F, step_y));
    }
    if (cv::norm(px_px - p0_px) < 1.0F || cv::norm(py_px - p0_px) < 1.0F)
    {
      return std::nullopt;
    }

    const auto z0 = depthPlaneDepthAtPixel(plane, p0_px, image_size);
    const auto zx = depthPlaneDepthAtPixel(plane, px_px, image_size);
    const auto zy = depthPlaneDepthAtPixel(plane, py_px, image_size);
    if (!z0.has_value() || !zx.has_value() || !zy.has_value())
    {
      return std::nullopt;
    }

    const cv::Vec3d p0 = projectPixelToCamera(p0_px, *z0, camera_info);
    const cv::Vec3d px = projectPixelToCamera(px_px, *zx, camera_info);
    const cv::Vec3d py = projectPixelToCamera(py_px, *zy, camera_info);
    cv::Vec3d normal = (px - p0).cross(py - p0);
    if (!normalizeVectorInPlace(normal))
    {
      return std::nullopt;
    }

    if (preferred_sign.has_value())
    {
      cv::Vec3d sign_reference = *preferred_sign;
      if (normalizeVectorInPlace(sign_reference) && normal.dot(sign_reference) < 0.0)
      {
        normal *= -1.0;
      }
    }
    return normal;
  }

  void alignItemPoseZAxisToNormal(
    ItemPose3D &pose,
    const std::optional<cv::Vec3d> &normal_opt)
  {
    if (!normal_opt.has_value())
    {
      return;
    }

    cv::Vec3d z_axis = *normal_opt;
    if (!normalizeVectorInPlace(z_axis))
    {
      return;
    }

    const cv::Vec3d original_x(
      pose.rotation(0, 0),
      pose.rotation(1, 0),
      pose.rotation(2, 0));
    const cv::Vec3d original_y(
      pose.rotation(0, 1),
      pose.rotation(1, 1),
      pose.rotation(2, 1));

    cv::Vec3d x_axis = original_x - z_axis * original_x.dot(z_axis);
    if (!normalizeVectorInPlace(x_axis))
    {
      x_axis = original_y - z_axis * original_y.dot(z_axis);
      if (!normalizeVectorInPlace(x_axis))
      {
        return;
      }
    }

    cv::Vec3d y_axis = z_axis.cross(x_axis);
    if (!normalizeVectorInPlace(y_axis))
    {
      return;
    }

    // Keep heading continuity with the detected in-plane orientation.
    if (y_axis.dot(original_y) < 0.0)
    {
      x_axis *= -1.0;
      y_axis *= -1.0;
    }

    pose.rotation = cv::Matx33d(
      x_axis[0], y_axis[0], z_axis[0],
      x_axis[1], y_axis[1], z_axis[1],
      x_axis[2], y_axis[2], z_axis[2]);
  }

  void refreshItemProfiles()
  {
    item_profiles_ = loadItemProfilesFromDirectory(profiles_dir_);
    syncSelectedProfileIndex();
    if (item_profiles_.empty())
    {
      profile_status_message_ = "No item profiles found";
    }
  }

  void syncSelectedProfileIndex()
  {
    selected_profile_index_ = -1;

    if (!selected_profile_path_.empty())
    {
      for (int i = 0; i < static_cast<int>(item_profiles_.size()); ++i)
      {
        if (
          pathsReferToSameFile(item_profiles_[i].path, selected_profile_path_) ||
          item_profiles_[i].path.filename() == selected_profile_path_.filename())
        {
          selected_profile_index_ = i;
          return;
        }
      }
    }

    for (int i = 0; i < static_cast<int>(item_profiles_.size()); ++i)
    {
      const bool name_matches = item_profiles_[i].item_name == item_name_;
      const bool date_matches =
        teach_date_.empty() || item_profiles_[i].teach_date.empty() || item_profiles_[i].teach_date == teach_date_;
      if (name_matches && date_matches)
      {
        selected_profile_index_ = i;
        return;
      }
    }
  }

  void selectInitialProfile()
  {
    if (item_profiles_.empty())
    {
      return;
    }

    if (selected_profile_index_ < 0)
    {
      selected_profile_index_ = 0;
    }

    applyProfile(item_profiles_[selected_profile_index_], false);
    selected_profile_path_ = item_profiles_[selected_profile_index_].path;
    profile_status_message_ = "Loaded " + item_profiles_[selected_profile_index_].display_label;
    saveSelectedProfileExportFile();
  }

  void applyProfile(const ItemProfile &profile, bool recreate_interfaces)
  {
    const bool topics_changed =
      color_topic_ != profile.color_topic ||
      depth_topic_ != profile.depth_topic ||
      camera_info_topic_ != profile.camera_info_topic ||
      overlay_topic_ != profile.overlay_topic;

    color_topic_ = profile.color_topic;
    depth_topic_ = profile.depth_topic;
    camera_info_topic_ = profile.camera_info_topic;
    overlay_topic_ = profile.overlay_topic;
    detection_use_depth_ = profile.detection_use_depth;
    depth_threshold_mm_ = std::clamp(
      profile.depth_threshold_mm,
      kDepthThresholdMinMm,
      kDepthThresholdMaxMm);
	    red_threshold_ = profile.red_threshold;
	    green_threshold_ = profile.green_threshold;
	    blue_threshold_ = profile.blue_threshold;
	    color_exposure_min_us_ = clampExposureUsec(profile.color_exposure_min_us);
	    color_exposure_max_us_ = std::max(
	      color_exposure_min_us_,
	      clampExposureUsec(profile.color_exposure_max_us));
	    depth_exposure_min_us_ = clampExposureUsec(profile.depth_exposure_min_us);
	    depth_exposure_max_us_ = std::max(
	      depth_exposure_min_us_,
	      clampExposureUsec(profile.depth_exposure_max_us));
	    color_exposure_us_ = clampExposureUsecOrAuto(
	      profile.color_exposure_us,
	      color_exposure_min_us_,
	      color_exposure_max_us_);
	    depth_exposure_us_ = clampExposureUsecOrAuto(
	      profile.depth_exposure_us,
	      depth_exposure_min_us_,
	      depth_exposure_max_us_);
	    depth_exposure_us_ = 0;
	    markCameraExposureDirty();
	    rgb_hole_fill_sensitivity_ = std::clamp(
      profile.rgb_hole_fill_sensitivity,
      kRgbHoleFillMin,
      kRgbHoleFillMax);
    rgb_mask_dilate_px_ = std::clamp(
      profile.rgb_mask_dilate_px,
      kRgbDilateMinPx,
      kRgbDilateMaxPx);
    depth_null_fill_sensitivity_ = std::clamp(
      profile.depth_null_fill_sensitivity,
      kDepthFillSensitivityMin,
      kDepthFillSensitivityMax);
    depth_window_mm_ = std::clamp(
      profile.depth_window_mm,
      kDepthWindowMinMm,
      kDepthWindowMaxMm);
    depth_hole_fill_sensitivity_ = std::clamp(
      profile.depth_hole_fill_sensitivity,
      kDepthFillSensitivityMin,
      kDepthFillSensitivityMax);
    depth_trim_px_ = std::clamp(
      profile.depth_trim_px,
      kDepthTrimMinPx,
      kDepthTrimMaxPx);
    adaptive_depth_trim_max_add_px_ =
      clampAdaptiveDepthTrimAddPx(profile.adaptive_depth_trim_max_add_px);
    adaptive_depth_trim_max_height_mm_ =
      clampAdaptiveDepthTrimHeightMm(profile.adaptive_depth_trim_max_height_mm);
    ray_step_px_ = profile.ray_step_px;
    depth_edge_offset_px_ = std::clamp(
      profile.depth_edge_offset_px,
      kDepthEdgeOffsetMinPx,
      kDepthEdgeOffsetMaxPx);
    previous_color_percent_ = profile.previous_color_percent;
    horizontal_ray_count_ = std::clamp(profile.horizontal_ray_count, 50, 100);
    vertical_ray_count_ = std::clamp(profile.vertical_ray_count, 50, 150);
    outlier_sensitivity_ = std::clamp(profile.outlier_sensitivity, 1, 100);
    focus_black_mask_ = profile.focus_black_mask;
    detect_black_to_white_ = !focus_black_mask_;
    trace_out_to_in_ = profile.trace_out_to_in;
    pose_reference_slots_.clear();
    for (const auto &slot : profile.pose_reference_slots)
    {
      if (slot.reference.area_px > 0 && slot.reference.hull.size() >= 3)
      {
        pose_reference_slots_.push_back(slot);
      }
    }
    if (
      pose_reference_slots_.empty() &&
      profile.has_pose_blob_reference &&
      profile.pose_blob_reference.area_px > 0 &&
      profile.pose_blob_reference.hull.size() >= 3)
    {
      pose_reference_slots_.push_back(PoseReferenceSlot2D{0, profile.pose_blob_reference});
    }
    std::sort(
      pose_reference_slots_.begin(),
      pose_reference_slots_.end(),
      [](const PoseReferenceSlot2D &a, const PoseReferenceSlot2D &b)
      {
        return a.slot_index < b.slot_index;
      });
    fallback_pose_slot_cursor_ = 0;
    clearFailedFirstBlobMasks();
    if (!pose_reference_slots_.empty())
    {
      pose_blob_reference_ = pose_reference_slots_.front().reference;
    }
    else
    {
      pose_blob_reference_.reset();
    }
    depth_plane_model_.valid = profile.depth_plane_enabled;
    depth_plane_model_.a = profile.depth_plane_a;
    depth_plane_model_.b = profile.depth_plane_b;
    depth_plane_model_.c = profile.depth_plane_c;
    depth_plane_model_.reference_depth_m = profile.depth_plane_reference_depth_m;
    depth_plane_roi_bounds_ = profile.depth_plane_roi_bounds;
    depth_plane_roi_normalized_ = profile.depth_plane_roi_normalized;
    if (
      !depth_plane_model_.valid ||
      !std::isfinite(depth_plane_model_.a) ||
      !std::isfinite(depth_plane_model_.b) ||
      !std::isfinite(depth_plane_model_.c) ||
      !std::isfinite(depth_plane_model_.reference_depth_m) ||
      depth_plane_model_.reference_depth_m <= 0.0 ||
      (!depth_plane_roi_bounds_.has_value() && !depth_plane_roi_normalized_.has_value()))
    {
      depth_plane_model_ = DepthPlaneModel{};
      depth_plane_roi_normalized_.reset();
    }
    roi_points_ = profile.roi_points;
    roi_points_normalized_ = profile.roi_points_normalized;
    roi_points_source_image_size_ = cv::Size(profile.roi_image_width, profile.roi_image_height);
    item_name_ = profile.item_name;
    teach_date_ = profile.teach_date;
    teach_joints_deg_ = profile.teach_joints_deg;
    has_teach_joints_ = profile.has_teach_joints;
    has_taught_item_dimensions_ = profile.has_taught_item_dimensions;
    taught_item_length_mm_ = profile.taught_item_length_mm;
    taught_item_width_mm_ = profile.taught_item_width_mm;
    resetMotionTracking();

    if (
      recreate_interfaces &&
      (
        topics_changed || !overlay_pub_ || !seek_pose_pub_ || !item_pose_array_pub_ || !color_sub_ || !depth_sub_ ||
        !camera_info_sub_))
    {
      createRosInterfaces();
    }
  }

  bool selectProfileByIndex(int index, bool persist_runtime = true)
  {
    if (index < 0 || index >= static_cast<int>(item_profiles_.size()))
    {
      return false;
    }

    selected_profile_index_ = index;
    selected_profile_path_ = item_profiles_[index].path;
    const bool interfaces_ready =
      static_cast<bool>(overlay_pub_) ||
      static_cast<bool>(seek_pose_pub_) ||
      static_cast<bool>(item_pose_array_pub_) ||
      static_cast<bool>(color_sub_) ||
      static_cast<bool>(depth_sub_) ||
      static_cast<bool>(camera_info_sub_);
    applyProfile(item_profiles_[index], interfaces_ready);
    profile_status_message_ = "Loaded " + item_profiles_[index].display_label;
    saveSelectedProfileExportFile();
    publishSelectedProfile();
    if (persist_runtime)
    {
      saveRuntimeUiSettings();
    }
    return true;
  }

  bool selectProfileByPath(const std::filesystem::path &path, bool persist_runtime = true)
  {
    if (path.empty())
    {
      return false;
    }
    const std::filesystem::path requested = path.lexically_normal();
    for (int i = 0; i < static_cast<int>(item_profiles_.size()); ++i)
    {
      const std::filesystem::path candidate = item_profiles_[i].path.lexically_normal();
      if (pathsReferToSameFile(candidate, requested) || candidate.filename() == requested.filename())
      {
        return selectProfileByIndex(i, persist_runtime);
      }
    }
    return false;
  }

  bool selectProfileFromFile(const std::filesystem::path &path, bool persist_runtime = true)
  {
    if (path.empty())
    {
      return false;
    }

    refreshItemProfiles();
    if (selectProfileByPath(path, persist_runtime))
    {
      return true;
    }

    const auto profile = loadItemProfileFile(path);
    if (!profile.has_value())
    {
      profile_status_message_ = "Open Teach: selected YAML is not an item teach profile";
      return false;
    }

    item_profiles_.push_back(*profile);
    return selectProfileByIndex(static_cast<int>(item_profiles_.size()) - 1, persist_runtime);
  }

  std::optional<std::filesystem::path> openTeachFileDialog()
  {
    std::filesystem::path start_dir = resolvePath(profiles_dir_);
    std::string filename_arg = start_dir.string();
    if (!filename_arg.empty() && filename_arg.back() != '/')
    {
      filename_arg.push_back('/');
    }

    const std::string command =
      "if command -v zenity >/dev/null 2>&1; then "
      "zenity --file-selection --title='Open Teach' --filename=" +
      shellQuote(filename_arg) +
      " --file-filter='YAML files | *.yaml *.yml' --file-filter='All files | *'; "
      "elif command -v kdialog >/dev/null 2>&1; then "
      "kdialog --title 'Open Teach' --getopenfilename " +
      shellQuote(start_dir.string()) +
      " 'YAML files (*.yaml *.yml)'; "
      "fi 2>/dev/null";

    FILE *pipe = popen(command.c_str(), "r");
    if (pipe == nullptr)
    {
      profile_status_message_ = "Open Teach: failed to start file picker";
      return std::nullopt;
    }

    std::array<char, 512> buffer {};
    std::string selected_path;
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr)
    {
      selected_path += buffer.data();
    }
    const int status = pclose(pipe);
    (void)status;

    selected_path = trimTrailingLineEndings(selected_path);
    if (selected_path.empty())
    {
      return std::nullopt;
    }
    return resolvePath(selected_path);
  }

  void requestOpenTeachFile()
  {
    profile_dropdown_open_ = false;
    profile_status_message_ = "Open Teach: select item teach YAML";
    const auto selected_path = openTeachFileDialog();
    if (!selected_path.has_value())
    {
      profile_status_message_ = "Open Teach cancelled";
      return;
    }

    if (!selectProfileFromFile(*selected_path))
    {
      RCLCPP_WARN(
        get_logger(),
        "Open Teach failed to load selected profile: %s",
        selected_path->string().c_str());
      return;
    }
    saveRuntimeUiSettings();
    saveSelectedProfileExportFile();
    profile_status_message_ = "Loaded " + selectedProfileDisplayText();
  }

  std::string runtimeViewModeToken() const
  {
    switch (display_view_)
    {
      case DisplayView::kRgb:
        return "rgb";
      case DisplayView::kDepth:
        return "depth";
      case DisplayView::kBinarized:
      default:
        return "binarized";
    }
  }

  void applyRuntimeViewModeToken(const std::string &view_mode)
  {
    const std::string token = normalizeDetectionModeToken(view_mode);
    if (token == "rgb")
    {
      display_view_ = DisplayView::kRgb;
      return;
    }
    if (token == "depth")
    {
      display_view_ = DisplayView::kDepth;
      return;
    }
    if (token == "binarized" || token == "bw" || token == "binary")
    {
      display_view_ = DisplayView::kBinarized;
    }
  }

  void loadRuntimeUiSettings()
  {
    std::filesystem::path source_path = runtime_settings_path_;
    if (source_path.empty())
    {
      return;
    }
    if (!std::filesystem::exists(source_path))
    {
      return;
    }

    try
    {
      const YAML::Node root = YAML::LoadFile(source_path.string());
      if (!root || !root.IsMap())
      {
        return;
      }
      if (const YAML::Node view_mode = root["view_mode"]; view_mode && view_mode.IsScalar())
      {
        applyRuntimeViewModeToken(view_mode.as<std::string>());
      }
      if (const YAML::Node overlay_enabled = root["overlay_enabled"]; overlay_enabled)
      {
        overlay_enabled_ = overlay_enabled.as<bool>();
      }
      if (const YAML::Node blob_tolerance = root["blob_tolerance_percent"]; blob_tolerance)
      {
        blob_tolerance_percent_ = std::clamp(
          blob_tolerance.as<int>(),
          tolerance_slider_.min_value,
          tolerance_slider_.max_value);
      }
      if (const YAML::Node seek_window_sec = root["seek_window_sec"]; seek_window_sec)
      {
        seek_window_tenths_ = std::clamp(
          static_cast<int>(std::round(seek_window_sec.as<double>())),
          seek_window_slider_.min_value,
          seek_window_slider_.max_value);
      }
      if (const YAML::Node seek_decay_sec = root["seek_decay_sec"]; seek_decay_sec)
      {
        seek_decay_tenths_ = std::clamp(
          static_cast<int>(std::round(seek_decay_sec.as<double>() * 10.0)),
          seek_decay_slider_.min_value,
          seek_decay_slider_.max_value);
      }
      RCLCPP_INFO(
        get_logger(),
        "Loaded item detect runtime UI settings from %s",
        source_path.c_str());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load item detect runtime UI settings from %s: %s",
        source_path.c_str(),
        ex.what());
    }
  }

  void saveRuntimeUiSettings() const
  {
    if (runtime_settings_path_.empty())
    {
      return;
    }

    try
    {
      const std::filesystem::path parent = runtime_settings_path_.parent_path();
      if (!parent.empty())
      {
        std::error_code fs_error;
        std::filesystem::create_directories(parent, fs_error);
        if (fs_error)
        {
          RCLCPP_WARN(
            get_logger(),
            "Unable to create runtime settings directory %s: %s",
            parent.c_str(),
            fs_error.message().c_str());
          return;
        }
      }

      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "view_mode" << YAML::Value << runtimeViewModeToken();
      out << YAML::Key << "overlay_enabled" << YAML::Value << overlay_enabled_;
      out << YAML::Key << "blob_tolerance_percent" << YAML::Value << blob_tolerance_percent_;
      out << YAML::Key << "seek_window_sec" << YAML::Value << seekWindowSeconds();
      out << YAML::Key << "seek_decay_sec" << YAML::Value << seekDecaySeconds();
      out << YAML::EndMap;

      std::ofstream file(runtime_settings_path_);
      file << out.c_str();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to save item detect runtime UI settings to %s: %s",
        runtime_settings_path_.c_str(),
        ex.what());
    }
  }

  void publishSelectedProfile()
  {
    if (!selected_profile_pub_)
    {
      return;
    }

    const std::filesystem::path selected_profile_path =
      (selected_profile_index_ >= 0 && selected_profile_index_ < static_cast<int>(item_profiles_.size()))
      ? item_profiles_[selected_profile_index_].path
      : selected_profile_path_;
    StringMsg msg;
    msg.data = selected_profile_path.string();
    selected_profile_pub_->publish(msg);
  }

  void saveSelectedProfileExportFile() const
  {
    if (selected_profile_export_path_.empty())
    {
      return;
    }

    try
    {
      const std::filesystem::path parent = selected_profile_export_path_.parent_path();
      if (!parent.empty())
      {
        std::error_code fs_error;
        std::filesystem::create_directories(parent, fs_error);
        if (fs_error)
        {
          RCLCPP_WARN(
            get_logger(),
            "Unable to create selected profile export directory %s: %s",
            parent.c_str(),
            fs_error.message().c_str());
          return;
        }
      }

      const std::filesystem::path selected_profile_path =
        (selected_profile_index_ >= 0 && selected_profile_index_ < static_cast<int>(item_profiles_.size()))
        ? item_profiles_[selected_profile_index_].path
        : selected_profile_path_;
      std::ofstream file(selected_profile_export_path_, std::ios::trunc);
      if (!file.is_open())
      {
        RCLCPP_WARN(
          get_logger(),
          "Unable to write selected profile export file %s",
          selected_profile_export_path_.c_str());
        return;
      }
      file << selected_profile_path.string() << "\n";
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to save selected profile export file %s: %s",
        selected_profile_export_path_.c_str(),
        ex.what());
    }
  }

  bool canDeleteSelectedProfile() const
  {
    const bool has_selected_profile = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(item_profiles_.size());
    if (!has_selected_profile)
    {
      return false;
    }
    return true;
  }

  void requestDeleteSelectedProfile()
  {
    if (selected_profile_index_ < 0 || selected_profile_index_ >= static_cast<int>(item_profiles_.size()))
    {
      profile_status_message_ = "No item profile selected";
      return;
    }
    if (!canDeleteSelectedProfile())
    {
      profile_status_message_ = "Select a dated item profile";
      return;
    }

    profile_dropdown_open_ = false;
    tolerance_slider_active_ = false;
    seek_window_slider_active_ = false;
    seek_decay_slider_active_ = false;
    delete_confirm_active_ = true;
    profile_status_message_ = "Confirm delete selected item profile";
  }

  void cancelDeleteConfirmation()
  {
    if (delete_confirm_active_)
    {
      profile_status_message_ = "Delete cancelled";
    }
    delete_confirm_active_ = false;
  }

  void confirmDeleteSelectedProfile()
  {
    if (!delete_confirm_active_)
    {
      return;
    }
    delete_confirm_active_ = false;
    deleteSelectedProfile();
  }

  bool deleteSelectedProfile()
  {
    if (selected_profile_index_ < 0 || selected_profile_index_ >= static_cast<int>(item_profiles_.size()))
    {
      profile_status_message_ = "No item profile selected";
      return false;
    }

    const ItemProfile deleted_profile = item_profiles_[selected_profile_index_];
    const std::filesystem::path delete_path = deleted_profile.path;

    std::error_code fs_error;
    const bool removed = std::filesystem::remove(delete_path, fs_error);
    if (!removed || fs_error)
    {
      profile_status_message_ = "Delete failed";
      return false;
    }

    selected_profile_path_.clear();
    refreshItemProfiles();

    if (!item_profiles_.empty())
    {
      const int next_index = std::clamp(selected_profile_index_, 0, static_cast<int>(item_profiles_.size()) - 1);
      selectProfileByIndex(next_index);
      profile_status_message_ = "Deleted " + delete_path.filename().string();
    }
    else
    {
      selected_profile_index_ = -1;
      profile_status_message_ = "Deleted " + delete_path.filename().string();
      saveSelectedProfileExportFile();
      publishSelectedProfile();
      saveRuntimeUiSettings();
    }

    return true;
  }

  std::string selectedProfileDisplayText() const
  {
    if (selected_profile_index_ >= 0 && selected_profile_index_ < static_cast<int>(item_profiles_.size()))
    {
      return item_profiles_[selected_profile_index_].display_label;
    }

    if (item_profiles_.empty())
    {
      return "No item profiles";
    }

    return "Select item profile";
  }

  bool canGoToTeach() const
  {
    const bool has_selected_profile = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(item_profiles_.size());
    return has_selected_profile && has_teach_joints_ && !go_to_teach_in_progress_;
  }

  bool requestGoToTeach()
  {
    const bool has_selected_profile = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(item_profiles_.size());
    if (!has_selected_profile)
    {
      profile_status_message_ = "Go to Teach: select an item profile";
      return false;
    }
    if (!has_teach_joints_)
    {
      profile_status_message_ = "Go to Teach: selected profile has no teach joints";
      return false;
    }
    if (go_to_teach_in_progress_)
    {
      profile_status_message_ = "Go to Teach: command in progress";
      return false;
    }
    if (!movj_client_)
    {
      profile_status_message_ = "Go to Teach: MovJ client unavailable";
      return false;
    }
    if (!movj_client_->service_is_ready())
    {
      profile_status_message_ = "Go to Teach: MovJ service not ready";
      return false;
    }

    auto request = std::make_shared<MovJSrv::Request>();
    request->mode = true;
    request->a = teach_joints_deg_[0];
    request->b = teach_joints_deg_[1];
    request->c = teach_joints_deg_[2];
    request->d = teach_joints_deg_[3];
    request->e = teach_joints_deg_[4];
    request->f = teach_joints_deg_[5];
    request->param_value.clear();

    go_to_teach_in_progress_ = true;
    profile_status_message_ = "Go to Teach: sending MovJ";
    RCLCPP_INFO(
      get_logger(),
      "Go to Teach MovJ -> %s with joints (deg): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      movj_service_name_.c_str(),
      teach_joints_deg_[0],
      teach_joints_deg_[1],
      teach_joints_deg_[2],
      teach_joints_deg_[3],
      teach_joints_deg_[4],
      teach_joints_deg_[5]);

    movj_client_->async_send_request(
      request,
      [this](rclcpp::Client<MovJSrv>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();
          const bool ok = response && response->res != -1;
          if (ok)
          {
            profile_status_message_ = "Go to Teach: MovJ accepted";
            RCLCPP_INFO(get_logger(), "Go to Teach: MovJ accepted (res=%d, robot_return=%s)", response->res, response->robot_return.c_str());
          }
          else
          {
            profile_status_message_ = "Go to Teach: MovJ failed";
            RCLCPP_WARN(
              get_logger(),
              "Go to Teach: MovJ failed (res=%d, robot_return=%s)",
              response ? response->res : -999,
              response ? response->robot_return.c_str() : "null");
          }
        }
        catch (const std::exception &ex)
        {
          profile_status_message_ = "Go to Teach: MovJ error";
          RCLCPP_WARN(get_logger(), "Go to Teach: MovJ call failed: %s", ex.what());
        }
        go_to_teach_in_progress_ = false;
      });
    return true;
  }

  int topBarHeight() const
  {
    return kTopBarBaseHeight;
  }

  void layoutTopBar(int width)
  {
    const int margin = 16;
    const int top_row_y = 14;
    const int top_row_height = 38;
    const int control_gap = 10;

    const int button_count = 7;
    const int top_button_width = std::max(
      112,
      std::min(
        132,
        (width - 2 * margin - (button_count - 1) * control_gap) / button_count));
    int button_x = margin;
    view_toggle_button_.rect = cv::Rect(button_x, top_row_y, top_button_width, top_row_height);
    button_x += top_button_width + control_gap;
    overlay_toggle_button_.rect = cv::Rect(
      button_x,
      top_row_y,
      top_button_width,
      top_row_height);
    button_x += top_button_width + control_gap;
    seek_toggle_button_.rect = cv::Rect(
      button_x,
      top_row_y,
      top_button_width,
      top_row_height);
    button_x += top_button_width + control_gap;
    debug_images_button_.rect = cv::Rect(
      button_x,
      top_row_y,
      top_button_width,
      top_row_height);
    button_x += top_button_width + control_gap;
    go_to_teach_button_.rect = cv::Rect(
      button_x,
      top_row_y,
      top_button_width,
      top_row_height);
    button_x += top_button_width + control_gap;
    profile_dropdown_rect_ = cv::Rect(
      button_x,
      top_row_y,
      top_button_width,
      top_row_height);
    button_x += top_button_width + control_gap;
    delete_button_.rect = cv::Rect(button_x, top_row_y, top_button_width, top_row_height);

    const int panel_y = top_row_y + top_row_height + 12;
    const int panel_gap = 10;
    const int panel_height = 92;
    const int panel_total_width = std::max(360, width - 2 * margin);
    const int panel_col_width = std::max(120, (panel_total_width - 2 * panel_gap) / 3);

    seek_vector_panel_rect_ = cv::Rect(margin, panel_y, panel_col_width, panel_height);
    seek_controls_panel_rect_ = cv::Rect(
      seek_vector_panel_rect_.x + seek_vector_panel_rect_.width + panel_gap,
      panel_y,
      panel_col_width,
      panel_height);
    quality_panel_rect_ = cv::Rect(
      seek_controls_panel_rect_.x + seek_controls_panel_rect_.width + panel_gap,
      panel_y,
      std::max(120, width - margin - (seek_controls_panel_rect_.x + seek_controls_panel_rect_.width + panel_gap)),
      panel_height);
    status_panel_rect_ = cv::Rect(
      margin,
      panel_y + panel_height + 8,
      std::max(120, width - 2 * margin),
      28);

    const int panel_pad = 12;
    seek_vector_label_origin_ = cv::Point(
      seek_vector_panel_rect_.x + panel_pad,
      seek_vector_panel_rect_.y + 43);
    seek_vector_value_origin_ = cv::Point(
      seek_vector_panel_rect_.x + panel_pad,
      seek_vector_panel_rect_.y + 64);
    seek_vector_time_origin_ = cv::Point(
      seek_vector_panel_rect_.x + panel_pad,
      seek_vector_panel_rect_.y + 82);

    seek_window_label_origin_ = cv::Point(
      seek_controls_panel_rect_.x + panel_pad,
      seek_controls_panel_rect_.y + 43);
    seek_window_slider_.track_rect = cv::Rect(
      seek_controls_panel_rect_.x + panel_pad,
      seek_controls_panel_rect_.y + 49,
      std::max(80, seek_controls_panel_rect_.width - 2 * panel_pad),
      10);
    seek_decay_label_origin_ = cv::Point(
      seek_controls_panel_rect_.x + panel_pad,
      seek_controls_panel_rect_.y + 73);
    seek_decay_slider_.track_rect = cv::Rect(
      seek_controls_panel_rect_.x + panel_pad,
      seek_controls_panel_rect_.y + 79,
      std::max(80, seek_controls_panel_rect_.width - 2 * panel_pad),
      10);

    tolerance_label_origin_ = cv::Point(
      quality_panel_rect_.x + panel_pad,
      quality_panel_rect_.y + 43);
    tolerance_slider_.track_rect = cv::Rect(
      quality_panel_rect_.x + panel_pad,
      quality_panel_rect_.y + 49,
      std::max(80, quality_panel_rect_.width - 2 * panel_pad),
      10);

    profile_option_rects_.clear();
    if (profile_dropdown_open_)
    {
      for (int i = 0; i < static_cast<int>(item_profiles_.size()); ++i)
      {
        profile_option_rects_.push_back(
          cv::Rect(
            profile_dropdown_rect_.x,
            kTopBarBaseHeight + i * kDropdownRowHeight,
            profile_dropdown_rect_.width,
            kDropdownRowHeight));
      }
    }
  }

  cv::Rect toleranceHitRect() const
  {
    return cv::Rect(
      tolerance_slider_.track_rect.x,
      tolerance_slider_.track_rect.y - kToleranceHitPadding,
      tolerance_slider_.track_rect.width,
      tolerance_slider_.track_rect.height + 2 * kToleranceHitPadding);
  }

  cv::Rect seekWindowHitRect() const
  {
    return cv::Rect(
      seek_window_slider_.track_rect.x,
      seek_window_slider_.track_rect.y - kToleranceHitPadding,
      seek_window_slider_.track_rect.width,
      seek_window_slider_.track_rect.height + 2 * kToleranceHitPadding);
  }

  cv::Rect seekDecayHitRect() const
  {
    return cv::Rect(
      seek_decay_slider_.track_rect.x,
      seek_decay_slider_.track_rect.y - kToleranceHitPadding,
      seek_decay_slider_.track_rect.width,
      seek_decay_slider_.track_rect.height + 2 * kToleranceHitPadding);
  }

  void updateToleranceFromPoint(const cv::Point &point)
  {
    if (tolerance_slider_.track_rect.width <= 0)
    {
      return;
    }

    const int clamped_x = std::clamp(
      point.x,
      tolerance_slider_.track_rect.x,
      tolerance_slider_.track_rect.x + tolerance_slider_.track_rect.width);
    const double t = static_cast<double>(clamped_x - tolerance_slider_.track_rect.x) /
      std::max(1, tolerance_slider_.track_rect.width);
    blob_tolerance_percent_ = static_cast<int>(std::round(
      tolerance_slider_.min_value + t * (tolerance_slider_.max_value - tolerance_slider_.min_value)));
  }

  void updateSeekWindowFromPoint(const cv::Point &point)
  {
    if (seek_window_slider_.track_rect.width <= 0)
    {
      return;
    }

    const int clamped_x = std::clamp(
      point.x,
      seek_window_slider_.track_rect.x,
      seek_window_slider_.track_rect.x + seek_window_slider_.track_rect.width);
    const double t = static_cast<double>(clamped_x - seek_window_slider_.track_rect.x) /
      std::max(1, seek_window_slider_.track_rect.width);
    const int updated_tenths = static_cast<int>(std::round(
      seek_window_slider_.min_value + t * (seek_window_slider_.max_value - seek_window_slider_.min_value)));
    seek_window_tenths_ = std::clamp(updated_tenths, seek_window_slider_.min_value, seek_window_slider_.max_value);
  }

  void updateSeekDecayFromPoint(const cv::Point &point)
  {
    if (seek_decay_slider_.track_rect.width <= 0)
    {
      return;
    }

    const int clamped_x = std::clamp(
      point.x,
      seek_decay_slider_.track_rect.x,
      seek_decay_slider_.track_rect.x + seek_decay_slider_.track_rect.width);
    const double t = static_cast<double>(clamped_x - seek_decay_slider_.track_rect.x) /
      std::max(1, seek_decay_slider_.track_rect.width);
    const int updated_tenths = static_cast<int>(std::round(
      seek_decay_slider_.min_value + t * (seek_decay_slider_.max_value - seek_decay_slider_.min_value)));
    seek_decay_tenths_ = std::clamp(updated_tenths, seek_decay_slider_.min_value, seek_decay_slider_.max_value);
  }

  int profileIndexAtPoint(const cv::Point &point) const
  {
    for (int i = 0; i < static_cast<int>(profile_option_rects_.size()); ++i)
    {
      if (profile_option_rects_[i].contains(point))
      {
        return i;
      }
    }
    return -1;
  }

  std::string resolvedCameraFrameId(
    const std_msgs::msg::Header &header,
    const CameraInfoMsg::ConstSharedPtr &info) const
  {
    if (!camera_frame_id_.empty())
    {
      return camera_frame_id_;
    }
    if (!header.frame_id.empty())
    {
      return header.frame_id;
    }
    if (info && !info->header.frame_id.empty())
    {
      return info->header.frame_id;
    }
    return "camera_color_optical_frame";
  }

  double seekWindowSeconds() const
  {
    return static_cast<double>(seek_window_tenths_);
  }

  double seekDecaySeconds() const
  {
    return static_cast<double>(seek_decay_tenths_) * 0.1;
  }

  SeekMotionData computeSeekMotionData(const SeekCapture &first_capture, const SeekCapture &last_capture) const
  {
    const cv::Vec3d delta_position_m = last_capture.pose.origin - first_capture.pose.origin;
    const int64_t dt_ns = last_capture.stamp.nanoseconds() - first_capture.stamp.nanoseconds();
    const double dt_sec = std::max(0.0, static_cast<double>(dt_ns) * 1e-9);
    return buildSeekMotionData(delta_position_m, dt_sec, last_capture.pose.rotation);
  }

  SeekMotionData computeSeekMotionData(const std::deque<SeekMotionSample> &samples) const
  {
    if (samples.size() < 2)
    {
      return SeekMotionData{};
    }

    SeekCapture first_capture;
    first_capture.stamp = samples.front().stamp;
    first_capture.pose = samples.front().pose;
    SeekCapture last_capture;
    last_capture.stamp = samples.back().stamp;
    last_capture.pose = samples.back().pose;

    const double dt_total_sec = std::max(0.0, (samples.back().stamp - samples.front().stamp).seconds());
    if (dt_total_sec <= 1e-6)
    {
      return computeSeekMotionData(first_capture, last_capture);
    }

    const double sample_count = static_cast<double>(samples.size());
    double sum_t = 0.0;
    double sum_t2 = 0.0;
    cv::Vec3d sum_p(0.0, 0.0, 0.0);
    cv::Vec3d sum_tp(0.0, 0.0, 0.0);
    const rclcpp::Time t0 = samples.front().stamp;

    for (const auto &sample : samples)
    {
      const double t_sec = std::max(0.0, (sample.stamp - t0).seconds());
      sum_t += t_sec;
      sum_t2 += t_sec * t_sec;
      sum_p += sample.pose.origin;
      sum_tp += sample.pose.origin * t_sec;
    }

    const double denominator = sample_count * sum_t2 - sum_t * sum_t;
    if (std::fabs(denominator) <= 1e-12)
    {
      return computeSeekMotionData(first_capture, last_capture);
    }

    const cv::Vec3d velocity_camera_mps = (sum_tp * sample_count - sum_p * sum_t) * (1.0 / denominator);
    const cv::Vec3d delta_position_m = velocity_camera_mps * dt_total_sec;
    return buildSeekMotionData(delta_position_m, dt_total_sec, samples.back().pose.rotation);
  }

  bool publishContinuousItemPose(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<ItemPose3D> &detected_pose)
  {
    if (!seek_pose_pub_ || !detected_pose.has_value())
    {
      return false;
    }

    const std::string resolved_frame = frame_id.empty() ? "camera_color_optical_frame" : frame_id;
    ItemPose3D pose_to_publish = *detected_pose;

    PoseStampedMsg msg;
    msg.header.stamp = toBuiltinTime(stamp);
    msg.header.frame_id = resolved_frame;
    msg.pose.position.x = pose_to_publish.origin[0];
    msg.pose.position.y = pose_to_publish.origin[1];
    msg.pose.position.z = pose_to_publish.origin[2];
    msg.pose.orientation = rotationToQuaternionMsg(pose_to_publish.rotation);
    seek_pose_pub_->publish(msg);
    return true;
  }

  void publishDetectedItemPoseArray(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::vector<std::optional<ItemPose3D>> &detected_item_poses)
  {
    if (!item_pose_array_pub_)
    {
      return;
    }

    const std::string resolved_frame = frame_id.empty() ? "camera_color_optical_frame" : frame_id;

    PoseArrayMsg msg;
    msg.header.stamp = toBuiltinTime(stamp);
    msg.header.frame_id = resolved_frame;
    msg.poses.reserve(detected_item_poses.size());

    for (const auto &detected_pose : detected_item_poses)
    {
      if (!detected_pose.has_value())
      {
        continue;
      }

      ItemPose3D pose_to_publish = *detected_pose;

      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = pose_to_publish.origin[0];
      pose_msg.position.y = pose_to_publish.origin[1];
      pose_msg.position.z = pose_to_publish.origin[2];
      pose_msg.orientation = rotationToQuaternionMsg(pose_to_publish.rotation);
      msg.poses.push_back(pose_msg);
    }

    item_pose_array_pub_->publish(msg);
  }

  void clearFailedFirstBlobMasks()
  {
    failed_first_blob_masks_by_slot_.clear();
    failed_first_blob_mask_size_ = cv::Size();
  }

  void ensureFailedFirstBlobMaskSize(const cv::Size &size)
  {
    if (size.width <= 0 || size.height <= 0)
    {
      clearFailedFirstBlobMasks();
      return;
    }
    if (failed_first_blob_mask_size_ != size)
    {
      failed_first_blob_masks_by_slot_.clear();
      failed_first_blob_mask_size_ = size;
    }
  }

  cv::Mat applyFailedFirstBlobMaskForSlot(const cv::Mat &mask, int slot_index) const
  {
    if (mask.empty() || mask.type() != CV_8UC1)
    {
      return mask.clone();
    }

    cv::Mat filtered_mask = mask.clone();
    const auto mask_it = failed_first_blob_masks_by_slot_.find(slot_index);
    if (
      mask_it == failed_first_blob_masks_by_slot_.end() ||
      mask_it->second.empty() ||
      mask_it->second.size() != mask.size())
    {
      return filtered_mask;
    }

    filtered_mask.setTo(cv::Scalar(0), mask_it->second);
    return filtered_mask;
  }

  bool recordFailedFirstBlobMaskForSlot(
    int slot_index,
    const cv::Mat &source_mask,
    const std::vector<cv::Point> &seed_pixels)
  {
    if (!seek_mode_active_ || source_mask.empty() || source_mask.type() != CV_8UC1 || seed_pixels.empty())
    {
      return false;
    }

    ensureFailedFirstBlobMaskSize(source_mask.size());

    cv::Mat labels;
    const int label_count = cv::connectedComponents(source_mask, labels, 8, CV_32S);
    if (label_count <= 1 || labels.empty())
    {
      return false;
    }

    std::vector<int> selected_labels;
    selected_labels.reserve(4);
    for (const auto &pixel : seed_pixels)
    {
      if (pixel.x < 0 || pixel.y < 0 || pixel.x >= labels.cols || pixel.y >= labels.rows)
      {
        continue;
      }
      const int label = labels.at<int>(pixel.y, pixel.x);
      if (label <= 0)
      {
        continue;
      }
      if (std::find(selected_labels.begin(), selected_labels.end(), label) == selected_labels.end())
      {
        selected_labels.push_back(label);
      }
    }

    if (selected_labels.empty())
    {
      return false;
    }

    const auto is_selected_label = [&](int label)
      {
        return std::find(selected_labels.begin(), selected_labels.end(), label) != selected_labels.end();
      };

    cv::Mat component_mask(source_mask.size(), CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < labels.rows; ++y)
    {
      const int *label_row = labels.ptr<int>(y);
      unsigned char *mask_row = component_mask.ptr<unsigned char>(y);
      for (int x = 0; x < labels.cols; ++x)
      {
        if (is_selected_label(label_row[x]))
        {
          mask_row[x] = 255;
        }
      }
    }

    if (cv::countNonZero(component_mask) <= 0)
    {
      return false;
    }

    cv::Mat &slot_mask = failed_first_blob_masks_by_slot_[slot_index];
    if (slot_mask.empty() || slot_mask.size() != source_mask.size() || slot_mask.type() != CV_8UC1)
    {
      slot_mask = cv::Mat::zeros(source_mask.size(), CV_8UC1);
    }
    cv::bitwise_or(slot_mask, component_mask, slot_mask);
    return true;
  }

  std::optional<std::pair<double, double>> itemPlanarSizeMeters(
    const std::optional<ItemEstimate> &accepted_estimate) const
  {
    std::array<double, 4> edge_lengths_cm {0.0, 0.0, 0.0, 0.0};
    if (accepted_estimate.has_value() &&
        accepted_estimate->has_metric_estimate &&
        hasValidEdgeLengthsCm(accepted_estimate->edge_lengths_cm))
    {
      edge_lengths_cm = accepted_estimate->edge_lengths_cm;
    }
    else
    {
      return std::nullopt;
    }

    const double x_size_m = 0.5 * (edge_lengths_cm[0] + edge_lengths_cm[1]) / 100.0;
    const double y_size_m = 0.5 * (edge_lengths_cm[2] + edge_lengths_cm[3]) / 100.0;
    if (x_size_m <= 1e-6 || y_size_m <= 1e-6)
    {
      return std::nullopt;
    }
    return std::make_pair(x_size_m, y_size_m);
  }

  void publishItemCubeMarker(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<ItemPose3D> &detected_pose,
    const std::optional<ItemEstimate> &accepted_estimate)
  {
    if (!publish_item_cube_marker_ || !item_cube_marker_pub_)
    {
      return;
    }

    MarkerMsg marker;
    marker.header.stamp = toBuiltinTime(stamp);
    marker.header.frame_id = frame_id.empty() ? "camera_color_optical_frame" : frame_id;
    marker.ns = "item_detect";
    marker.id = 0;
    marker.type = MarkerMsg::CUBE;
    marker.frame_locked = false;

    const auto item_size_m = itemPlanarSizeMeters(accepted_estimate);
    if (!detected_pose.has_value() || !item_size_m.has_value())
    {
      marker.action = MarkerMsg::DELETE;
      item_cube_marker_pub_->publish(marker);
      return;
    }

    const double thickness_m = std::max(1e-4, item_marker_thickness_mm_ / kMetersToMillimeters);
    const cv::Vec3d local_center(
      0.5 * item_size_m->first,
      0.5 * item_size_m->second,
      0.5 * thickness_m);
    const cv::Matx33d &rotation = detected_pose->rotation;
    const cv::Vec3d center(
      detected_pose->origin[0] + rotation(0, 0) * local_center[0] + rotation(0, 1) * local_center[1] +
        rotation(0, 2) * local_center[2],
      detected_pose->origin[1] + rotation(1, 0) * local_center[0] + rotation(1, 1) * local_center[1] +
        rotation(1, 2) * local_center[2],
      detected_pose->origin[2] + rotation(2, 0) * local_center[0] + rotation(2, 1) * local_center[1] +
        rotation(2, 2) * local_center[2]);

    marker.action = MarkerMsg::ADD;
    marker.pose.position.x = center[0];
    marker.pose.position.y = center[1];
    marker.pose.position.z = center[2];
    marker.pose.orientation = rotationToQuaternionMsg(rotation);
    marker.scale.x = item_size_m->first;
    marker.scale.y = item_size_m->second;
    marker.scale.z = thickness_m;
    marker.color.r = 0.15f;
    marker.color.g = 0.85f;
    marker.color.b = 0.95f;
    marker.color.a = 0.65f;
    item_cube_marker_pub_->publish(marker);
  }

  void resetSeekSessionState()
  {
    seek_last_valid_capture_.reset();
    seek_valid_motion_samples_.clear();
    seek_valid_frame_count_ = 0;
    seek_window_started_ = false;
    seek_window_start_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    fallback_pose_slot_cursor_ = 0;
    clearFailedFirstBlobMasks();
  }

  void resetSeekEvidenceState()
  {
    seek_last_valid_capture_.reset();
    seek_valid_motion_samples_.clear();
    seek_valid_frame_count_ = 0;
  }

  void clearSeekResultFreeze()
  {
    seek_result_freeze_frame_.release();
    seek_result_freeze_until_ = std::chrono::steady_clock::time_point{};
  }

  void startSeekResultFreeze(const cv::Mat &frame)
  {
    if (frame.empty())
    {
      clearSeekResultFreeze();
      return;
    }

    seek_result_freeze_frame_ = frame.clone();
    seek_result_freeze_until_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(std::round(kSeekResultFreezeSeconds * 1000.0)));
  }

  void applySeekResultFreeze(cv::Mat &output)
  {
    if (seek_result_freeze_frame_.empty())
    {
      return;
    }

    if (std::chrono::steady_clock::now() >= seek_result_freeze_until_)
    {
      clearSeekResultFreeze();
      return;
    }

    output = seek_result_freeze_frame_.clone();
  }

  void toggleSeek()
  {
    if (seek_mode_active_ || seek_result_latched_)
    {
      seek_mode_active_ = false;
      seek_result_latched_ = false;
      clearSeekResultFreeze();
      resetSeekSessionState();
      profile_status_message_ = "Seek cancelled";
      return;
    }

    seek_mode_active_ = true;
    seek_result_latched_ = false;
    clearSeekResultFreeze();
    resetSeekSessionState();
    profile_status_message_ = "Seek armed: publish nearest-to-peak pose on next valid frame";
  }

  void toggleDebugImages()
  {
    debug_images_enabled_ = !debug_images_enabled_;
    saveRuntimeUiSettings();
    profile_status_message_ = debug_images_enabled_
      ? "Debug images enabled"
      : "Debug images disabled";
  }

  bool writeSeekPoseData(
    const std::filesystem::path &pose_path,
    const SeekCapture &last_capture,
    const SeekMotionData &motion,
    std::size_t motion_sample_count,
    const std::filesystem::path &last_image_path,
    double effective_decay_sec) const
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "seek_window_sec" << YAML::Value << seekWindowSeconds();
    out << YAML::Key << "seek_decay_sec" << YAML::Value << seekDecaySeconds();
    out << YAML::Key << "effective_decay_sec" << YAML::Value << std::max(0.0, effective_decay_sec);
    out << YAML::Key << "motion_source" << YAML::Value << "average_all_valid_frames_linear_fit";
    out << YAML::Key << "motion_sample_count" << YAML::Value << static_cast<int>(motion_sample_count);
    out << YAML::Key << "last_frame_path" << YAML::Value << last_image_path.string();
    out << YAML::Key << "units" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::Value << "mm";
    out << YAML::Key << "delta_position" << YAML::Value << "mm";
    out << YAML::Key << "velocity" << YAML::Value << "mm/s";
    out << YAML::Key << "speed" << YAML::Value << "mm/s";
    out << YAML::EndMap;

    const auto emit_capture = [&](const char *key, const SeekCapture &capture)
    {
      const auto q = rotationToQuaternionMsg(capture.pose.rotation);
      const cv::Vec3d position_mm = capture.pose.origin * kMetersToMillimeters;
      out << YAML::Key << key << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "timestamp_ns" << YAML::Value << capture.stamp.nanoseconds();
      out << YAML::Key << "frame_id" << YAML::Value << capture.frame_id;
      out << YAML::Key << "position_mm" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << position_mm[0];
      out << YAML::Key << "y" << YAML::Value << position_mm[1];
      out << YAML::Key << "z" << YAML::Value << position_mm[2];
      out << YAML::EndMap;
      out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "w" << YAML::Value << q.w;
      out << YAML::Key << "x" << YAML::Value << q.x;
      out << YAML::Key << "y" << YAML::Value << q.y;
      out << YAML::Key << "z" << YAML::Value << q.z;
      out << YAML::EndMap;
      out << YAML::Key << "rotation_matrix" << YAML::Value << YAML::BeginSeq;
      for (int row = 0; row < 3; ++row)
      {
        out << YAML::Flow << YAML::BeginSeq
            << capture.pose.rotation(row, 0)
            << capture.pose.rotation(row, 1)
            << capture.pose.rotation(row, 2)
            << YAML::EndSeq;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    };

    emit_capture("last_valid", last_capture);

    out << YAML::Key << "motion_camera_frame" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "delta_t_sec" << YAML::Value << motion.dt_sec;
    out << YAML::Key << "delta_position_mm" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << motion.delta_position_mm[0];
    out << YAML::Key << "y" << YAML::Value << motion.delta_position_mm[1];
    out << YAML::Key << "z" << YAML::Value << motion.delta_position_mm[2];
    out << YAML::EndMap;
    out << YAML::Key << "velocity_mmps" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << motion.velocity_camera_mmps[0];
    out << YAML::Key << "y" << YAML::Value << motion.velocity_camera_mmps[1];
    out << YAML::Key << "z" << YAML::Value << motion.velocity_camera_mmps[2];
    out << YAML::EndMap;
    out << YAML::Key << "speed_mmps" << YAML::Value << motion.speed_camera_mmps;
    out << YAML::Key << "direction_unit" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << motion.direction_camera[0];
    out << YAML::Key << "y" << YAML::Value << motion.direction_camera[1];
    out << YAML::Key << "z" << YAML::Value << motion.direction_camera[2];
    out << YAML::EndMap;
    out << YAML::EndMap;

    out << YAML::Key << "motion_child_frame" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "frame_id" << YAML::Value << item_frame_id_;
    out << YAML::Key << "delta_t_sec" << YAML::Value << motion.dt_sec;
    out << YAML::Key << "velocity_mmps" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << motion.velocity_child_mmps[0];
    out << YAML::Key << "y" << YAML::Value << motion.velocity_child_mmps[1];
    out << YAML::Key << "z" << YAML::Value << motion.velocity_child_mmps[2];
    out << YAML::EndMap;
    out << YAML::Key << "speed_mmps" << YAML::Value << motion.speed_child_mmps;
    out << YAML::Key << "direction_unit" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << motion.direction_child[0];
    out << YAML::Key << "y" << YAML::Value << motion.direction_child[1];
    out << YAML::Key << "z" << YAML::Value << motion.direction_child[2];
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream pose_file(pose_path);
    if (!pose_file.is_open())
    {
      return false;
    }
    pose_file << out.c_str() << '\n';
    return pose_file.good();
  }

  bool saveSeekScreenshotsAndPoseData(
    const rclcpp::Time &fallback_stamp,
    bool bypass_decay_on_publish = false)
  {
    if (!seek_last_valid_capture_.has_value())
    {
      seek_mode_active_ = true;
      seek_result_latched_ = false;
      resetSeekSessionState();
      profile_status_message_ = cv::format(
        "Seek still ON: no valid item frame in %.1fs window; reacquiring",
        seekWindowSeconds());
      return false;
    }

    const auto &last_capture = *seek_last_valid_capture_;
    const double effective_decay_sec = bypass_decay_on_publish ? 0.0 : seekDecaySeconds();
    SeekMotionData motion = computeSeekMotionData(seek_valid_motion_samples_);
    const int64_t stamp_ns = (last_capture.stamp.nanoseconds() != 0)
      ? last_capture.stamp.nanoseconds()
      : fallback_stamp.nanoseconds();
    const std::filesystem::path last_path =
      std::filesystem::path(seek_snapshots_dir_) / ("seek_" + std::to_string(stamp_ns) + "_last.png");
    const std::filesystem::path pose_path =
      std::filesystem::path(seek_snapshots_dir_) / ("seek_" + std::to_string(stamp_ns) + "_pose.yaml");

    bool debug_save_failed = false;
    if (!debug_images_enabled_)
    {
      profile_status_message_ = "Seek done, sent " + formatPose6D(last_capture.pose) + " (debug save off)";
    }
    else
    {
      std::filesystem::path output_dir(seek_snapshots_dir_);
      std::error_code fs_error;
      std::filesystem::create_directories(output_dir, fs_error);
      bool wrote_pose = false;
      if (fs_error)
      {
        profile_status_message_ = "Seek done: failed to prepare screenshot directory";
        debug_save_failed = true;
      }
      else
      {
        const bool wrote_last =
          !last_capture.frame.empty() && cv::imwrite(last_path.string(), last_capture.frame);
        wrote_pose = writeSeekPoseData(
          pose_path,
          last_capture,
          motion,
          seek_valid_motion_samples_.size(),
          last_path,
          effective_decay_sec);

        if (wrote_last && wrote_pose)
        {
          profile_status_message_ = "Seek done, sent " + formatPose6D(last_capture.pose);
          RCLCPP_INFO(
            get_logger(),
            "Seek data saved:\n  last frame: %s\n  pose yaml:  %s",
            last_path.c_str(),
            pose_path.c_str());
        }
        else
        {
          debug_save_failed = true;
          profile_status_message_ = "Seek done, sent " + formatPose6D(last_capture.pose) + " (partial save)";
          RCLCPP_WARN(
            get_logger(),
            "Seek save partial. last=%s pose=%s",
            wrote_last ? "ok" : "fail",
            wrote_pose ? "ok" : "fail");
        }
      }
    }

    startSeekResultFreeze(last_capture.frame);
    seek_mode_active_ = false;
    seek_result_latched_ = true;
    resetSeekSessionState();
    if (debug_save_failed)
    {
      profile_status_message_ = "Seek done, handed off item target (debug save failed)";
    }
    profile_status_message_ += " | waiting for item pick release";
    return true;
  }

  void updateSeekSession(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<ItemPose3D> &detected_pose,
    const cv::Mat &output_frame)
  {
    if (!seek_mode_active_)
    {
      return;
    }

    if (!seek_window_started_)
    {
      seek_window_started_ = true;
      seek_window_start_stamp_ = stamp;
    }

    const double elapsed_sec = (stamp - seek_window_start_stamp_).seconds();
    if (!detected_pose.has_value())
    {
      if (elapsed_sec >= seekWindowSeconds())
      {
        saveSeekScreenshotsAndPoseData(stamp, true);
        return;
      }
      profile_status_message_ = cv::format(
        "Seek waiting: no valid item frame (%.1f/%.1fs)",
        elapsed_sec,
        seekWindowSeconds());
      return;
    }

    SeekCapture capture;
    capture.stamp = stamp;
    capture.pose = *detected_pose;
    capture.frame_id = frame_id;
    capture.frame = output_frame.clone();

    seek_last_valid_capture_ = capture;
    seek_valid_motion_samples_.push_back(SeekMotionSample{stamp, *detected_pose});
    if (seek_valid_motion_samples_.size() > seek_motion_history_max_samples_)
    {
      seek_valid_motion_samples_.pop_front();
    }
    ++seek_valid_frame_count_;

    profile_status_message_ = cv::format(
      "Seek ready: pose candidate acquired (%d frame%s)",
      seek_valid_frame_count_,
      seek_valid_frame_count_ == 1 ? "" : "s");
  }

  void resetMotionTracking()
  {
    seek_mode_active_ = false;
    seek_result_latched_ = false;
    clearSeekResultFreeze();
    resetSeekSessionState();
    item_summary_ = ItemSummary{};
  }

  void drawVelocityArrow(
    cv::Mat &image,
    const std::optional<ItemPose3D> &item_pose,
    const CameraInfoMsg::ConstSharedPtr &info) const
  {
    (void)image;
    (void)item_pose;
    (void)info;
  }

  std::vector<cv::Point2f> effectiveRoiPointsForImage(const cv::Size &image_size) const
  {
    if (
      hasValidRoiPoints(roi_points_) &&
      roi_points_source_image_size_.width > 1 &&
      roi_points_source_image_size_.height > 1 &&
      image_size.width > 1 &&
      image_size.height > 1 &&
      (roi_points_source_image_size_.width != image_size.width ||
       roi_points_source_image_size_.height != image_size.height))
    {
      const float scale_x = static_cast<float>(image_size.width - 1) /
        static_cast<float>(roi_points_source_image_size_.width - 1);
      const float scale_y = static_cast<float>(image_size.height - 1) /
        static_cast<float>(roi_points_source_image_size_.height - 1);
      std::vector<cv::Point2f> scaled;
      scaled.reserve(roi_points_.size());
      for (const auto &point : roi_points_)
      {
        scaled.emplace_back(
          std::clamp(point.x * scale_x, 0.0F, static_cast<float>(image_size.width - 1)),
          std::clamp(point.y * scale_y, 0.0F, static_cast<float>(image_size.height - 1)));
      }
      return scaled;
    }

    if (hasValidRoiPoints(roi_points_))
    {
      return roi_points_;
    }

    if (roi_points_normalized_.size() >= 4)
    {
      const std::vector<cv::Point2f> points = denormalizeRoiPoints(roi_points_normalized_, image_size);
      if (hasValidRoiPoints(points))
      {
        return points;
      }
    }

    return {};
  }

  std::optional<AxisAlignedRoiBounds> effectiveDepthPlaneRoiBoundsForImage(const cv::Size &image_size) const
  {
    if (
      depth_plane_roi_bounds_.has_value() &&
      roi_points_source_image_size_.width > 1 &&
      roi_points_source_image_size_.height > 1 &&
      image_size.width > 1 &&
      image_size.height > 1 &&
      (roi_points_source_image_size_.width != image_size.width ||
       roi_points_source_image_size_.height != image_size.height))
    {
      const double scale_x = static_cast<double>(image_size.width - 1) /
        static_cast<double>(roi_points_source_image_size_.width - 1);
      const double scale_y = static_cast<double>(image_size.height - 1) /
        static_cast<double>(roi_points_source_image_size_.height - 1);
      AxisAlignedRoiBounds scaled{
        static_cast<int>(std::lround(static_cast<double>(depth_plane_roi_bounds_->left) * scale_x)),
        static_cast<int>(std::lround(static_cast<double>(depth_plane_roi_bounds_->top) * scale_y)),
        static_cast<int>(std::lround(static_cast<double>(depth_plane_roi_bounds_->right) * scale_x)),
        static_cast<int>(std::lround(static_cast<double>(depth_plane_roi_bounds_->bottom) * scale_y)),
      };
      scaled.left = std::clamp(scaled.left, 0, image_size.width - 1);
      scaled.right = std::clamp(scaled.right, 0, image_size.width - 1);
      scaled.top = std::clamp(scaled.top, 0, image_size.height - 1);
      scaled.bottom = std::clamp(scaled.bottom, 0, image_size.height - 1);
      if (isValidRoiBounds(scaled))
      {
        return scaled;
      }
    }

    if (depth_plane_roi_bounds_.has_value())
    {
      return depth_plane_roi_bounds_;
    }

    if (depth_plane_roi_normalized_.has_value())
    {
      const auto bounds = denormalizeRoiBounds(*depth_plane_roi_normalized_, image_size);
      if (bounds.has_value())
      {
        return bounds;
      }
    }

    return depth_plane_roi_bounds_;
  }

  void colorCallback(const ImageMsg::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr color_cv;
    try
    {
      color_cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Color conversion failed: %s", ex.what());
      return;
    }

    cv::Mat depth_m;
    CameraInfoMsg::ConstSharedPtr info;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (latest_depth_.empty() || !latest_camera_info_)
      {
        return;
      }
      depth_m = latest_depth_.clone();
      info = latest_camera_info_;
    }
    const cv::Size color_size = color_cv->image.size();
    if (depth_m.size() != color_size)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Depth frame size (%d x %d) differs from color (%d x %d); resizing depth to color size.",
        depth_m.cols,
        depth_m.rows,
        color_size.width,
        color_size.height);
      cv::resize(depth_m, depth_m, color_size, 0.0, 0.0, cv::INTER_NEAREST);
    }

    const rclcpp::Time stamp(msg->header.stamp);
    const std::string resolved_frame_id = resolvedCameraFrameId(msg->header, info);
    const std::vector<cv::Point2f> effective_roi_points = effectiveRoiPointsForImage(color_size);
    const std::optional<AxisAlignedRoiBounds> effective_depth_plane_roi =
      effectiveDepthPlaneRoiBoundsForImage(color_size);
    const bool roi_ready = hasValidRoiPoints(effective_roi_points);
    cv::Mat roi_mask;
    if (roi_ready)
    {
      roi_mask = buildRoiMask(color_cv->image.size(), effective_roi_points);
    }

    const cv::Mat color_mask = buildRgbMask(
      color_cv->image,
      red_threshold_,
      green_threshold_,
      blue_threshold_,
      rgb_hole_fill_sensitivity_,
      rgb_mask_dilate_px_,
      focus_black_mask_);
    cv::Mat detection_mask = color_mask.clone();
    ensureFailedFirstBlobMaskSize(detection_mask.size());
    cv::Mat depth_for_pose_m = depth_m;
    DepthWindowPeakInfo depth_window_peak_info;
    if (roi_ready)
    {
      cv::bitwise_and(detection_mask, roi_mask, detection_mask);
    }
    const bool depth_plane_ready = depth_plane_model_.valid && effective_depth_plane_roi.has_value();

    if (detection_use_depth_)
    {
      depth_m = fillInvalidDepthNearby(depth_m, depth_null_fill_sensitivity_);
      if (depth_plane_ready)
      {
        const cv::Mat depth_residual_m = computeDepthPlaneResidual(depth_m, depth_plane_model_);
        cv::Mat finite_depth_residual_mask = buildFiniteDepthResidualMask(depth_residual_m);
        if (roi_ready)
        {
          cv::bitwise_and(finite_depth_residual_mask, roi_mask, finite_depth_residual_mask);
        }

        // Select the depth-window peak only from finite depth pixels inside the
        // current RGB/ROI detection mask. This prevents random depth noise
        // outside the binarized RGB blob from becoming the peak that drives the
        // top-window filter.
        cv::Mat peak_candidate_mask;
        cv::bitwise_and(finite_depth_residual_mask, detection_mask, peak_candidate_mask);

        cv::Mat depth_window_mask = applyDepthTopWindowMask(
          depth_residual_m,
          peak_candidate_mask,
          depth_window_mm_,
          &depth_window_peak_info);
        cv::Mat windowed_depth_residual = depth_residual_m.clone();
        windowed_depth_residual.setTo(
          cv::Scalar(std::numeric_limits<float>::quiet_NaN()),
          depth_window_mask == 0);
        const cv::Mat depth_retain_mask = buildFiniteDepthMask(windowed_depth_residual);
        cv::bitwise_and(detection_mask, depth_retain_mask, detection_mask);
        detection_mask = fillEnclosedMaskHoles(detection_mask, depth_hole_fill_sensitivity_);
        // Keep item_detect depth-for-pose in sync with item_teach:
        // estimate 3D pose from post-window measured metric depth only.
        cv::Mat windowed_depth_for_pose = depth_m.clone();
        windowed_depth_for_pose.setTo(
          cv::Scalar(std::numeric_limits<float>::quiet_NaN()),
          depth_window_mask == 0);
        const cv::Mat valid_depth_mask = buildPositiveFiniteDepthMask(windowed_depth_for_pose);
        windowed_depth_for_pose.setTo(
          cv::Scalar(std::numeric_limits<float>::quiet_NaN()),
          valid_depth_mask == 0);
        depth_for_pose_m = windowed_depth_for_pose;
        const int effective_depth_trim_px = computeAdaptiveDepthTrimPx(
          depth_trim_px_,
          depth_window_peak_info,
          adaptive_depth_trim_max_add_px_,
          adaptive_depth_trim_max_height_mm_);
        detection_mask = trimMaskInward(detection_mask, effective_depth_trim_px);
      }
      else if (!depth_m.empty() && depth_m.type() == CV_32FC1)
      {
        detection_mask = cv::Mat::zeros(depth_m.size(), CV_8UC1);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 2000,
          "Depth mode enabled but fixed depth plane is missing; run item_teach depth-plane ROI and save profile.");
      }
    }

    std::optional<BinarizedPoseEstimate2D> pose_estimate;
    std::optional<BinarizedPoseEstimate2D::BlobPose2D> fallback_anchor_preview_pose;
    std::optional<PairDirectShapeFitDebug2D> fallback_pair_debug_info;
    std::string pose_status_text;
    int matched_pose_reference_slot_index = -1;
    bool matched_pose_reference_used_fallback = false;
    if (roi_ready)
    {
      if (!pose_reference_slots_.empty())
      {
        std::string first_failure_status;
        std::string first_fallback_failure_status;
        std::optional<BinarizedPoseEstimate2D::BlobPose2D> first_failure_anchor_preview_pose;
        std::optional<PairDirectShapeFitDebug2D> first_failure_pair_debug_info;

        struct PoseSlotDetectionResult
        {
          int slot_index {0};
          std::optional<BinarizedPoseEstimate2D> pose_estimate;
          std::string status_text;
          bool used_direct_shape_fit {false};
          std::optional<BinarizedPoseEstimate2D::BlobPose2D> fallback_anchor_preview_pose;
          std::optional<PairDirectShapeFitDebug2D> fallback_pair_debug_info;
          std::vector<cv::Point> fallback_first_blob_pixels;
        };

        const cv::Mat detection_mask_snapshot = detection_mask;
        const std::vector<cv::Point2f> roi_points_snapshot = effective_roi_points;
        const DepthWindowPeakInfo depth_window_peak_info_snapshot = depth_window_peak_info;
        const int blob_tolerance_percent_snapshot = blob_tolerance_percent_;
        const auto collapse_single_result_if_needed =
          [&](PoseSlotDetectionResult &result, const PoseBlobReference2D &reference)
          {
            if (isPairPoseReference(reference))
            {
              return;
            }
            const std::optional<cv::Point> preferred_blob_pixel =
              depth_window_peak_info_snapshot.valid
              ? std::optional<cv::Point>(depth_window_peak_info_snapshot.pixel)
              : std::nullopt;
            collapsePoseEstimateToSingleBlob(result.pose_estimate, preferred_blob_pixel);
          };
        const auto detect_normal_pose_slot =
          [=](const PoseReferenceSlot2D &slot) -> PoseSlotDetectionResult
          {
            PoseSlotDetectionResult result;
            result.slot_index = slot.slot_index;
            try
            {
              const auto &reference = slot.reference;
              const cv::Mat slot_detection_mask = seek_mode_active_
                ? applyFailedFirstBlobMaskForSlot(detection_mask_snapshot, slot.slot_index)
                : detection_mask_snapshot;
              result.pose_estimate = estimatePoseFromBinarizedMask(
                slot_detection_mask,
                reference,
                blob_tolerance_percent_snapshot,
                &result.status_text);
              collapse_single_result_if_needed(result, reference);
              if (result.pose_estimate.has_value())
              {
                result.status_text.clear();
              }
            }
            catch (const std::exception &ex)
            {
              result.pose_estimate.reset();
              result.status_text = std::string("Normal detection error: ") + ex.what();
            }
            return result;
          };
        const auto detect_fallback_pose_slot =
          [=](const PoseReferenceSlot2D &slot) -> PoseSlotDetectionResult
          {
            PoseSlotDetectionResult result;
            result.slot_index = slot.slot_index;
            try
            {
              const auto &reference = slot.reference;
              const bool pair_pose_mode = isPairPoseReference(reference);
              const cv::Mat slot_detection_mask = seek_mode_active_
                ? applyFailedFirstBlobMaskForSlot(detection_mask_snapshot, slot.slot_index)
                : detection_mask_snapshot;
              cv::Mat direct_fit_mask = slot_detection_mask;
              cv::Point direct_fit_offset_px(0, 0);
              if (const auto roi_bounds = roiBoundsForImage(
                  roi_points_snapshot,
                  slot_detection_mask.size());
                roi_bounds.has_value())
              {
                const cv::Rect roi_rect(
                  roi_bounds->left,
                  roi_bounds->top,
                  (roi_bounds->right - roi_bounds->left) + 1,
                  (roi_bounds->bottom - roi_bounds->top) + 1);
                if (
                  roi_rect.width > 1 &&
                  roi_rect.height > 1 &&
                  roi_rect.x >= 0 &&
                  roi_rect.y >= 0 &&
                  roi_rect.x + roi_rect.width <= slot_detection_mask.cols &&
                  roi_rect.y + roi_rect.height <= slot_detection_mask.rows)
                {
                  direct_fit_mask = slot_detection_mask(roi_rect).clone();
                  direct_fit_offset_px = roi_rect.tl();
                }
              }
              cv::Mat pair_search_mask = direct_fit_mask;
              cv::Point pair_search_offset_px = direct_fit_offset_px;
              const std::optional<cv::Rect> merged_rect = suspiciousMergedBlobSearchRect(
                direct_fit_mask,
                reference,
                blob_tolerance_percent_snapshot);
              if (merged_rect.has_value())
              {
                direct_fit_mask = direct_fit_mask(*merged_rect).clone();
                direct_fit_offset_px += merged_rect->tl();
              }
              if (pair_pose_mode || merged_rect.has_value())
              {
                result.pose_estimate = pair_pose_mode
                  ? estimatePairPoseFromDirectShapeFit(
                  direct_fit_mask,
                  reference,
                  blob_tolerance_percent_snapshot,
                  direct_fit_offset_px,
                  &result.status_text,
                  &result.fallback_anchor_preview_pose,
                  &result.fallback_pair_debug_info,
                  &pair_search_mask,
                  pair_search_offset_px,
                  &result.fallback_first_blob_pixels)
                  : estimatePoseFromDirectShapeFit(
                  direct_fit_mask,
                  reference,
                  blob_tolerance_percent_snapshot,
                  direct_fit_offset_px,
                  &result.status_text,
                  &result.fallback_anchor_preview_pose,
                  &result.fallback_first_blob_pixels);
                result.used_direct_shape_fit = result.pose_estimate.has_value();
              }
              else
              {
                result.status_text = pair_pose_mode
                  ? "No suspicious merged blob for pair shape-fit fallback"
                  : "No suspicious merged blob for shape-fit fallback";
              }
              collapse_single_result_if_needed(result, reference);
            }
            catch (const std::exception &ex)
            {
              result.pose_estimate.reset();
              result.fallback_anchor_preview_pose.reset();
              result.fallback_pair_debug_info.reset();
              result.fallback_first_blob_pixels.clear();
              result.status_text = std::string("Fallback detection error: ") + ex.what();
            }
            return result;
          };

        for (const auto &slot : pose_reference_slots_)
        {
          const PoseSlotDetectionResult slot_result = detect_normal_pose_slot(slot);
          if (slot_result.pose_estimate.has_value())
          {
            pose_estimate = slot_result.pose_estimate;
            matched_pose_reference_slot_index = slot_result.slot_index;
            matched_pose_reference_used_fallback = false;
            fallback_anchor_preview_pose.reset();
            fallback_pair_debug_info.reset();
            break;
          }

          if (first_failure_status.empty() && !slot_result.status_text.empty())
          {
            first_failure_status =
              "Slot " + std::to_string(slot_result.slot_index + 1) + ": " + slot_result.status_text;
          }
          if (
            !first_failure_anchor_preview_pose.has_value() &&
            slot_result.fallback_anchor_preview_pose.has_value())
          {
            first_failure_anchor_preview_pose = slot_result.fallback_anchor_preview_pose;
          }
          if (!first_failure_pair_debug_info.has_value() && slot_result.fallback_pair_debug_info.has_value())
          {
            first_failure_pair_debug_info = slot_result.fallback_pair_debug_info;
          }
        }

        if (!pose_estimate.has_value() && seek_mode_active_)
        {
          if (fallback_pose_slot_cursor_ >= pose_reference_slots_.size())
          {
            fallback_pose_slot_cursor_ = 0;
          }
          const std::size_t active_fallback_slot_vector_index = fallback_pose_slot_cursor_;
          const auto &slot = pose_reference_slots_[active_fallback_slot_vector_index];
          const PoseSlotDetectionResult slot_result = detect_fallback_pose_slot(slot);
          if (slot_result.pose_estimate.has_value())
          {
            pose_estimate = slot_result.pose_estimate;
            matched_pose_reference_slot_index = slot_result.slot_index;
            matched_pose_reference_used_fallback = slot_result.used_direct_shape_fit;
            fallback_anchor_preview_pose.reset();
            fallback_pair_debug_info.reset();
          }
          else
          {
            bool masked_failed_first_blob = false;
            if (!slot_result.fallback_first_blob_pixels.empty())
            {
              const cv::Mat slot_detection_mask =
                applyFailedFirstBlobMaskForSlot(detection_mask_snapshot, slot.slot_index);
              masked_failed_first_blob = recordFailedFirstBlobMaskForSlot(
                slot.slot_index,
                slot_detection_mask,
                slot_result.fallback_first_blob_pixels);
            }

            fallback_pose_slot_cursor_ =
              (active_fallback_slot_vector_index + 1) % pose_reference_slots_.size();
            std::string fallback_status =
              slot_result.status_text.empty() ? "Fallback no pose" : slot_result.status_text;
            if (masked_failed_first_blob)
            {
              fallback_status += " | masked failed first blob for slot " +
                std::to_string(slot_result.slot_index + 1);
            }
            first_fallback_failure_status =
              "Slot " + std::to_string(slot_result.slot_index + 1) + ": " + fallback_status;
            if (slot_result.fallback_anchor_preview_pose.has_value())
            {
              first_failure_anchor_preview_pose = slot_result.fallback_anchor_preview_pose;
            }
            if (slot_result.fallback_pair_debug_info.has_value())
            {
              first_failure_pair_debug_info = slot_result.fallback_pair_debug_info;
            }
          }
        }

        if (!pose_estimate.has_value())
        {
          if (!seek_mode_active_)
          {
            pose_status_text = first_failure_status.empty()
              ? "No normal pose references matched | press Seek for merged fallback"
              : first_failure_status + " | press Seek for merged fallback";
          }
          else
          {
            pose_status_text = !first_fallback_failure_status.empty()
              ? first_fallback_failure_status
              : (first_failure_status.empty() ? "No pose references matched" : first_failure_status);
          }
          fallback_anchor_preview_pose = first_failure_anchor_preview_pose;
          fallback_pair_debug_info = first_failure_pair_debug_info;
        }
      }
      else
      {
        pose_status_text = "Profile missing pose template; run item_teach and save again";
      }
    }

    std::vector<std::optional<ItemPose3D>> blob_poses_3d;
    int selected_blob_index = -1;
    std::optional<ItemPose3D> selected_blob_pose_3d;
    std::optional<BinarizedPoseEstimate2D> accepted_pose_estimate;
    int accepted_pose_count = 0;
    std::string dimension_status_text;
    if (pose_estimate.has_value())
    {
      blob_poses_3d.resize(pose_estimate->blob_poses.size());
      accepted_pose_estimate = BinarizedPoseEstimate2D{};
      for (std::size_t i = 0; i < pose_estimate->blob_poses.size(); ++i)
      {
        if (has_taught_item_dimensions_)
        {
          const auto dimensions = measureBlobDimensionsOnDepthPlane(
            pose_estimate->blob_poses[i],
            color_size,
            *info);
          if (!dimensions.has_value())
          {
            if (dimension_status_text.empty())
            {
              dimension_status_text = depth_plane_model_.valid
                ? "Rejected size: unable to measure detected item on taught depth plane."
                : "Rejected size: taught depth plane unavailable for item dimension check.";
            }
            continue;
          }

          std::string candidate_dimension_status;
          if (!detectedDimensionsWithinTaughtBand(
              *dimensions,
              blob_tolerance_percent_,
              &candidate_dimension_status))
          {
            if (dimension_status_text.empty())
            {
              dimension_status_text = candidate_dimension_status;
            }
            continue;
          }
        }

        const auto pose_3d = estimateBlobPose3D(pose_estimate->blob_poses[i], depth_for_pose_m, *info);
        if (!pose_3d.has_value())
        {
          continue;
        }
        ItemPose3D aligned_pose = *pose_3d;
        const cv::Vec3d detected_z_axis(
          pose_3d->rotation(0, 2),
          pose_3d->rotation(1, 2),
          pose_3d->rotation(2, 2));
        const std::optional<cv::Vec3d> plane_normal_opt = depthPlaneNormalInFrame(
          depth_plane_model_,
          *info,
          color_size,
          poseBlobCenterPx(pose_estimate->blob_poses[i]),
          detected_z_axis);

        // No base-normal fallback: if the taught depth plane is unavailable,
        // keep the original detected pose orientation.
        alignItemPoseZAxisToNormal(aligned_pose, plane_normal_opt);
        blob_poses_3d[i] = aligned_pose;
        accepted_pose_estimate->blob_poses.push_back(pose_estimate->blob_poses[i]);
        ++accepted_pose_count;
        accepted_pose_estimate->matched_blob_count = accepted_pose_count;
      }
      if (accepted_pose_count <= 0)
      {
        accepted_pose_estimate.reset();
      }

      if (depth_window_peak_info.valid)
      {
        double best_sq_distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < pose_estimate->blob_poses.size(); ++i)
        {
          if (!blob_poses_3d[i].has_value())
          {
            continue;
          }
          const cv::Point2f candidate_point_px =
            pose_estimate->blob_poses[i].member_count >= 2
            ? poseBlobCenterPx(pose_estimate->blob_poses[i])
            : (pose_estimate->blob_poses[i].has_custom_anchor
            ? pose_estimate->blob_poses[i].anchor_point_px
            : pose_estimate->blob_poses[i].origin);
          const cv::Point2f delta = candidate_point_px -
            cv::Point2f(
            static_cast<float>(depth_window_peak_info.pixel.x),
            static_cast<float>(depth_window_peak_info.pixel.y));
          const double sq_distance = static_cast<double>(delta.dot(delta));
          if (sq_distance < best_sq_distance)
          {
            best_sq_distance = sq_distance;
            selected_blob_index = static_cast<int>(i);
          }
        }
      }
      if (selected_blob_index < 0)
      {
        for (std::size_t i = 0; i < blob_poses_3d.size(); ++i)
        {
          if (blob_poses_3d[i].has_value())
          {
            selected_blob_index = static_cast<int>(i);
            break;
          }
        }
      }
      if (selected_blob_index >= 0)
      {
        selected_blob_pose_3d = blob_poses_3d[static_cast<std::size_t>(selected_blob_index)];
      }
      else
      {
        if (dimension_status_text.empty())
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Matched blobs found, but 3D pose estimation failed for all blob items.");
        }
      }
    }

    if (pose_estimate.has_value() && !selected_blob_pose_3d.has_value() && !dimension_status_text.empty())
    {
      pose_status_text = dimension_status_text;
    }

    item_summary_.detected_items = accepted_pose_count;
    item_summary_.has_best_candidate = selected_blob_pose_3d.has_value();
    if (selected_blob_pose_3d.has_value())
    {
      item_summary_.best_candidate_position_m = selected_blob_pose_3d->origin;
      item_summary_.frame_id = resolved_frame_id;
    }
    else
    {
      item_summary_.best_candidate_position_m = cv::Vec3d(0.0, 0.0, 0.0);
      item_summary_.frame_id.clear();
    }

    const std::optional<ItemPose3D> filtered_item_pose_3d = selected_blob_pose_3d;

    cv::Mat overlay;
    switch (display_view_)
    {
      case DisplayView::kRgb:
        overlay = color_cv->image.clone();
        break;
      case DisplayView::kDepth:
        overlay = colorizeDepth(depth_m);
        if (overlay.size() != color_cv->image.size())
        {
          cv::resize(overlay, overlay, color_cv->image.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        break;
      case DisplayView::kBinarized:
      default:
        if (focus_black_mask_)
        {
          overlay = cv::Mat(detection_mask.size(), CV_8UC3, cv::Scalar(240, 240, 240));
          overlay.setTo(cv::Scalar(8, 8, 8), detection_mask);
        }
        else
        {
          cv::cvtColor(detection_mask, overlay, cv::COLOR_GRAY2BGR);
        }
        break;
    }
    if (overlay_enabled_)
    {
      drawRoiOverlay(overlay, effective_roi_points);
      drawCenterCursor(overlay, effective_roi_points);
      drawItemName(overlay, item_name_);
      drawPoseHullOverlay(overlay, accepted_pose_estimate);
      if (!accepted_pose_estimate.has_value() && fallback_anchor_preview_pose.has_value())
      {
        BinarizedPoseEstimate2D preview_estimate;
        preview_estimate.matched_blob_count = 1;
        preview_estimate.blob_poses.push_back(*fallback_anchor_preview_pose);
        drawPoseHullOverlay(overlay, std::optional<BinarizedPoseEstimate2D>(preview_estimate), true);
      }
      if (!accepted_pose_estimate.has_value() && !pose_estimate.has_value())
      {
        drawPredictedCompanionOverlay(overlay, fallback_pair_debug_info);
      }
      if (detection_use_depth_ && depth_window_peak_info.valid)
      {
        drawDepthWindowPeakOverlay(overlay, depth_window_peak_info);
      }
      drawVelocityArrow(overlay, filtered_item_pose_3d, info);
      if (!roi_ready)
      {
        cv::putText(
          overlay,
          "Selected item profile is missing ROI",
          cv::Point(18, 68),
          cv::FONT_HERSHEY_SIMPLEX,
          0.72,
          cv::Scalar(0, 180, 255),
          2);
        cv::putText(
          overlay,
          "Teach and save ROI before detect",
          cv::Point(18, 102),
          cv::FONT_HERSHEY_SIMPLEX,
          0.68,
          cv::Scalar(0, 180, 255),
          2);
      }
      else if (detection_use_depth_ && !depth_plane_model_.valid)
      {
        cv::putText(
          overlay,
          "Depth plane missing in profile",
          cv::Point(18, 68),
          cv::FONT_HERSHEY_SIMPLEX,
          0.72,
          cv::Scalar(0, 180, 255),
          2);
        cv::putText(
          overlay,
          "Teach depth plane ROI and save profile",
          cv::Point(18, 94),
          cv::FONT_HERSHEY_SIMPLEX,
          0.62,
          cv::Scalar(0, 200, 255),
          2);
      }
      else if (!pose_status_text.empty())
      {
        cv::putText(
          overlay,
          pose_status_text,
          cv::Point(18, 68),
          cv::FONT_HERSHEY_SIMPLEX,
          0.72,
          cv::Scalar(0, 180, 255),
          2);
        if (fallback_anchor_preview_pose.has_value())
        {
          cv::putText(
            overlay,
            "Previewing fitted blob 1/2 only",
            cv::Point(18, 96),
            cv::FONT_HERSHEY_SIMPLEX,
            0.62,
            cv::Scalar(0, 200, 255),
            2);
        }
        if (fallback_pair_debug_info.has_value())
        {
          cv::putText(
            overlay,
            "Predicted blob 2/2 search area shown",
            cv::Point(18, fallback_anchor_preview_pose.has_value() ? 124 : 96),
            cv::FONT_HERSHEY_SIMPLEX,
            0.58,
            cv::Scalar(220, 90, 255),
            2);
        }
      }
      else if (accepted_pose_estimate.has_value())
      {
        const bool multi_pose_mode = pose_reference_slots_.size() > 1;
        const bool pair_pose_mode =
          !pose_reference_slots_.empty() && isPairPoseReference(pose_reference_slots_.front().reference);
        const std::string ready_text = multi_pose_mode
          ? cv::format(
            "Multi slot %d %sready: matches=%d refs=%d selected=%d edge=+/-%d%%",
            matched_pose_reference_slot_index >= 0
            ? matched_pose_reference_slot_index + 1
            : 0,
            matched_pose_reference_used_fallback ? "fallback " : "",
            accepted_pose_estimate->matched_blob_count,
            static_cast<int>(pose_reference_slots_.size()),
            selected_blob_index >= 0 ? (selected_blob_index + 1) : 0,
            blob_tolerance_percent_)
          : (
          pair_pose_mode
          ? cv::format(
            "Pair pose ready: groups=%d selected=%d edge=+/-%d%%",
            accepted_pose_estimate->matched_blob_count,
            selected_blob_index >= 0 ? (selected_blob_index + 1) : 0,
            blob_tolerance_percent_)
          : cv::format(
            "Single pose ready: matches=%d selected=%d edge=+/-%d%%",
            accepted_pose_estimate->matched_blob_count,
            selected_blob_index >= 0 ? (selected_blob_index + 1) : 0,
            blob_tolerance_percent_));
        cv::putText(
          overlay,
          ready_text,
          cv::Point(18, 68),
          cv::FONT_HERSHEY_SIMPLEX,
          0.72,
          cv::Scalar(0, 180, 255),
          2);
      }
    }

    publishDetectedItemPoseArray(stamp, resolved_frame_id, blob_poses_3d);

    cv::Mat output = drawWindowFrame(overlay);
    bool seek_pose_published = false;
    if (seek_mode_active_)
    {
      updateSeekSession(stamp, resolved_frame_id, filtered_item_pose_3d, output);
      seek_pose_published = publishContinuousItemPose(stamp, resolved_frame_id, filtered_item_pose_3d);
      publishItemCubeMarker(stamp, resolved_frame_id, filtered_item_pose_3d, std::nullopt);
      if (seek_pose_published)
      {
        const bool seek_completed = saveSeekScreenshotsAndPoseData(stamp, true);
        if (!seek_completed)
        {
          profile_status_message_ = "Seek warning: pose sent, but pose-data save failed";
        }
      }
    }
    else
    {
      publishItemCubeMarker(stamp, resolved_frame_id, std::nullopt, std::nullopt);
    }

    applySeekResultFreeze(output);

    if (publish_overlay_)
    {
      cv_bridge::CvImage overlay_image;
      overlay_image.header = msg->header;
      overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_image.image = output;
      overlay_pub_->publish(*overlay_image.toImageMsg());
    }

    if (!headless_)
    {
      const cv::Size window_size(output.cols, output.rows);
      if (window_size != rendered_window_size_)
      {
        cv::resizeWindow(kDetectWindowName, window_size.width, window_size.height);
        rendered_window_size_ = window_size;
      }
      cv::imshow(kDetectWindowName, output);
      processWindowEvents();
    }
    last_camera_render_time_ = std::chrono::steady_clock::now();
  }

  void renderNoCameraTopicsOverlay()
  {
    const auto now_time = std::chrono::steady_clock::now();
    if (last_camera_render_time_ != std::chrono::steady_clock::time_point{} &&
      now_time - last_camera_render_time_ < std::chrono::seconds(1))
    {
      return;
    }

    cv::Mat placeholder(kPreviewCanvasHeight, kPreviewCanvasWidth, CV_8UC3, cv::Scalar(18, 18, 18));
    cv::rectangle(
      placeholder,
      cv::Rect(0, 0, placeholder.cols, placeholder.rows),
      cv::Scalar(34, 34, 34),
      2);

    const std::string title = "no camera topics...";
    const std::string color_line =
      "color: " + color_topic_ + "  publishers=" + std::to_string(count_publishers(color_topic_));
    const std::string depth_line =
      "depth: " + depth_topic_ + "  publishers=" + std::to_string(count_publishers(depth_topic_));
    const std::string info_line =
      "info:  " + camera_info_topic_ + "  publishers=" + std::to_string(count_publishers(camera_info_topic_));
    const std::array<std::string, 4> lines = {title, color_line, depth_line, info_line};
    const std::array<double, 4> scales = {1.35, 0.68, 0.68, 0.68};
    const std::array<int, 4> thicknesses = {3, 1, 1, 1};
    const std::array<cv::Scalar, 4> colors = {
      cv::Scalar(80, 220, 255),
      cv::Scalar(220, 220, 220),
      cv::Scalar(220, 220, 220),
      cv::Scalar(220, 220, 220)};
    int y = (placeholder.rows / 2) - 55;
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(
        lines[i], cv::FONT_HERSHEY_SIMPLEX, scales[i], thicknesses[i], &baseline);
      const int x = std::max(20, (placeholder.cols - text_size.width) / 2);
      cv::putText(
        placeholder,
        lines[i],
        cv::Point(x, y),
        cv::FONT_HERSHEY_SIMPLEX,
        scales[i],
        colors[i],
        thicknesses[i],
        cv::LINE_AA);
      y += (i == 0) ? 52 : 32;
    }

    cv::Mat output = drawWindowFrame(placeholder);
    if (publish_overlay_ && overlay_pub_)
    {
      cv_bridge::CvImage overlay_image;
      overlay_image.header.stamp = toBuiltinTime(get_clock()->now());
      overlay_image.header.frame_id = camera_frame_id_.empty()
        ? std::string("camera_color_optical_frame")
        : camera_frame_id_;
      overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_image.image = output;
      overlay_pub_->publish(*overlay_image.toImageMsg());
    }
    if (!headless_)
    {
      cv::imshow(kDetectWindowName, output);
      processWindowEvents();
    }
    last_camera_render_time_ = std::chrono::steady_clock::now();
  }

  static void onMouseThunk(int event, int x, int y, int flags, void *userdata)
  {
    static_cast<ItemDetectNode *>(userdata)->onMouse(event, x, y, flags);
  }

  void onMouse(int event, int x, int y, int /*flags*/)
  {
    const cv::Point point(x, y);

    if (delete_confirm_active_)
    {
      if (event == cv::EVENT_LBUTTONDOWN)
      {
        if (delete_confirm_accept_button_rect_.contains(point))
        {
          confirmDeleteSelectedProfile();
          return;
        }
        if (delete_confirm_cancel_button_rect_.contains(point) ||
            !delete_confirm_dialog_rect_.contains(point))
        {
          cancelDeleteConfirmation();
          return;
        }
      }

      if (event == cv::EVENT_LBUTTONUP)
      {
        tolerance_slider_active_ = false;
        seek_window_slider_active_ = false;
        seek_decay_slider_active_ = false;
      }
      return;
    }

    if (event == cv::EVENT_LBUTTONDOWN)
    {
      if (view_toggle_button_.rect.contains(point))
      {
        advanceViewMode();
        profile_dropdown_open_ = false;
        return;
      }

      if (overlay_toggle_button_.rect.contains(point))
      {
        overlay_enabled_ = !overlay_enabled_;
        saveRuntimeUiSettings();
        profile_dropdown_open_ = false;
        return;
      }

      if (seek_toggle_button_.rect.contains(point))
      {
        profile_dropdown_open_ = false;
        toggleSeek();
        return;
      }

      if (debug_images_button_.rect.contains(point))
      {
        profile_dropdown_open_ = false;
        toggleDebugImages();
        return;
      }

      if (delete_button_.rect.contains(point))
      {
        profile_dropdown_open_ = false;
        requestDeleteSelectedProfile();
        return;
      }

      if (go_to_teach_button_.rect.contains(point))
      {
        profile_dropdown_open_ = false;
        requestGoToTeach();
        return;
      }

      if (profile_dropdown_rect_.contains(point))
      {
        requestOpenTeachFile();
        return;
      }

      if (toleranceHitRect().contains(point))
      {
        tolerance_slider_active_ = true;
        seek_window_slider_active_ = false;
        seek_decay_slider_active_ = false;
        updateToleranceFromPoint(point);
        profile_dropdown_open_ = false;
        return;
      }

      if (seekWindowHitRect().contains(point))
      {
        seek_window_slider_active_ = true;
        tolerance_slider_active_ = false;
        seek_decay_slider_active_ = false;
        updateSeekWindowFromPoint(point);
        profile_dropdown_open_ = false;
        return;
      }

      if (seekDecayHitRect().contains(point))
      {
        seek_decay_slider_active_ = true;
        tolerance_slider_active_ = false;
        seek_window_slider_active_ = false;
        updateSeekDecayFromPoint(point);
        profile_dropdown_open_ = false;
        return;
      }

      profile_dropdown_open_ = false;
    }

    if (event == cv::EVENT_MOUSEMOVE && tolerance_slider_active_)
    {
      updateToleranceFromPoint(point);
    }
    else if (event == cv::EVENT_MOUSEMOVE && seek_window_slider_active_)
    {
      updateSeekWindowFromPoint(point);
    }
    else if (event == cv::EVENT_MOUSEMOVE && seek_decay_slider_active_)
    {
      updateSeekDecayFromPoint(point);
    }

    if (event == cv::EVENT_LBUTTONUP)
    {
      const bool any_slider_active = tolerance_slider_active_ || seek_window_slider_active_ ||
        seek_decay_slider_active_;
      tolerance_slider_active_ = false;
      seek_window_slider_active_ = false;
      seek_decay_slider_active_ = false;
      if (any_slider_active)
      {
        saveRuntimeUiSettings();
      }
    }
  }

  std::string buttonText() const
  {
    return "View: " + currentViewLabel();
  }

  void advanceViewMode()
  {
    switch (display_view_)
    {
      case DisplayView::kRgb:
        display_view_ = DisplayView::kBinarized;
        break;
      case DisplayView::kBinarized:
        display_view_ = DisplayView::kDepth;
        break;
      case DisplayView::kDepth:
      default:
        display_view_ = DisplayView::kRgb;
        break;
    }
    saveRuntimeUiSettings();
  }

  std::string currentViewLabel() const
  {
    switch (display_view_)
    {
      case DisplayView::kRgb:
        return "RGB";
      case DisplayView::kDepth:
        return "Depth";
      case DisplayView::kBinarized:
      default:
        return "Binarized";
    }
  }

  std::string overlayButtonText() const
  {
    return overlay_enabled_ ? "Overlay: ON" : "Overlay: OFF";
  }

  std::string seekButtonText() const
  {
    return (seek_mode_active_ || seek_result_latched_) ? "Seek: ON" : "Seek: OFF";
  }

  cv::Size previewCanvasSizeForSource(const cv::Size &source_size) const
  {
    if (source_size.width <= 0 || source_size.height <= 0)
    {
      return cv::Size(kPreviewCanvasWidth, kPreviewCanvasHeight);
    }

    const int preview_height = std::max(
      1,
      static_cast<int>(std::round(
        static_cast<double>(kPreviewCanvasWidth) *
        static_cast<double>(source_size.height) /
        static_cast<double>(source_size.width))));
    return cv::Size(kPreviewCanvasWidth, preview_height);
  }

  cv::Rect previewImageRectForSource(const cv::Size &source_size) const
  {
    return cv::Rect(cv::Point(0, 0), previewCanvasSizeForSource(source_size));
  }

  cv::Mat buildPreviewCanvas(const cv::Mat &image) const
  {
    const cv::Size canvas_size = previewCanvasSizeForSource(image.size());
    cv::Mat canvas(canvas_size.height, canvas_size.width, CV_8UC3, cv::Scalar(30, 32, 36));
    if (image.empty())
    {
      return canvas;
    }

    if (canvas_size.width <= 0 || canvas_size.height <= 0)
    {
      return canvas;
    }

    if (canvas_size == image.size())
    {
      return image.clone();
    }

    cv::Mat resized;
    const int interpolation = canvas_size.area() >= image.size().area() ? cv::INTER_LINEAR : cv::INTER_AREA;
    cv::resize(image, resized, canvas_size, 0.0, 0.0, interpolation);
    return resized;
  }

  cv::Mat drawTopBar(int width)
  {
    layoutTopBar(width);
    cv::Mat bar(topBarHeight(), width, CV_8UC3, cv::Scalar(28, 30, 34));
    cv::line(bar, cv::Point(0, 58), cv::Point(width, 58), cv::Scalar(52, 56, 62), 1);

    const bool can_delete = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(item_profiles_.size());
    const bool can_go_to_teach = canGoToTeach();

    const auto draw_button = [&](const UiButton &button,
      const std::string &text,
      bool enabled,
      const cv::Scalar &fill_on,
      const cv::Scalar &border_on)
    {
      const cv::Scalar fill = enabled ? fill_on : cv::Scalar(60, 63, 68);
      const cv::Scalar border = enabled ? border_on : cv::Scalar(102, 106, 112);
      cv::rectangle(bar, button.rect, fill, cv::FILLED);
      cv::rectangle(bar, button.rect, border, 2);
      const std::string fitted_text = fitTextToWidth(text, button.rect.width - 18, 0.52, 1);
      cv::putText(
        bar,
        fitted_text,
        cv::Point(button.rect.x + 9, button.rect.y + 26),
        cv::FONT_HERSHEY_DUPLEX,
        0.52,
        cv::Scalar(245, 245, 245),
        1,
        cv::LINE_AA);
    };

    draw_button(view_toggle_button_, buttonText(), true, cv::Scalar(70, 132, 82), cv::Scalar(132, 215, 150));
    draw_button(overlay_toggle_button_, overlayButtonText(), overlay_enabled_, cv::Scalar(68, 124, 154), cv::Scalar(132, 205, 236));
    draw_button(
      seek_toggle_button_,
      seekButtonText(),
      seek_mode_active_ || seek_result_latched_,
      cv::Scalar(70, 126, 186),
      cv::Scalar(126, 202, 255));
    draw_button(
      debug_images_button_,
      "Debug Img",
      true,
      debug_images_enabled_ ? cv::Scalar(70, 126, 186) : cv::Scalar(58, 64, 72),
      debug_images_enabled_ ? cv::Scalar(126, 202, 255) : cv::Scalar(112, 120, 130));
    draw_button(go_to_teach_button_, go_to_teach_in_progress_ ? "Go Teach..." : "Go To Teach", can_go_to_teach, cv::Scalar(70, 140, 94), cv::Scalar(134, 232, 165));
    draw_button(delete_button_, "Delete Item", can_delete, cv::Scalar(86, 76, 148), cv::Scalar(160, 146, 246));
    draw_button(
      UiButton{"Open Teach", profile_dropdown_rect_},
      "Open Teach",
      true,
      cv::Scalar(61, 78, 96),
      cv::Scalar(130, 166, 198));

    const auto draw_panel = [&](const cv::Rect &rect, const std::string &title)
    {
      cv::rectangle(bar, rect, cv::Scalar(38, 41, 46), cv::FILLED);
      cv::rectangle(bar, rect, cv::Scalar(72, 77, 84), 1);
      const cv::Rect header(rect.x, rect.y, rect.width, 24);
      cv::rectangle(bar, header, cv::Scalar(46, 50, 56), cv::FILLED);
      cv::line(bar, cv::Point(rect.x, rect.y + 24), cv::Point(rect.x + rect.width, rect.y + 24), cv::Scalar(72, 77, 84), 1);
      cv::putText(
        bar,
        title,
        cv::Point(rect.x + 10, rect.y + 17),
        cv::FONT_HERSHEY_DUPLEX,
        0.45,
        cv::Scalar(220, 224, 230),
        1,
        cv::LINE_AA);
    };

    draw_panel(seek_vector_panel_rect_, "Item Summary");
    draw_panel(seek_controls_panel_rect_, "Seek Controls");
    draw_panel(quality_panel_rect_, "Detection Quality");

    cv::putText(
      bar,
      cv::format("Detected items: %d", std::max(0, item_summary_.detected_items)),
      seek_vector_label_origin_,
      cv::FONT_HERSHEY_DUPLEX,
      0.45,
      cv::Scalar(170, 238, 185),
      1,
      cv::LINE_AA);
    if (item_summary_.has_best_candidate)
    {
      const cv::Vec3d xyz_mm = item_summary_.best_candidate_position_m * kMetersToMillimeters;
      cv::putText(
        bar,
        cv::format("Best candidate XYZ: %+0.1f %+0.1f %+0.1f mm", xyz_mm[0], xyz_mm[1], xyz_mm[2]),
        seek_vector_value_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.45,
        cv::Scalar(205, 212, 220),
        1,
        cv::LINE_AA);
      cv::putText(
        bar,
        "Best = nearest blob to highest peak",
        seek_vector_time_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.40,
        cv::Scalar(165, 170, 176),
        1,
        cv::LINE_AA);
    }
    else
    {
      cv::putText(
        bar,
        "Best candidate XYZ: n/a",
        seek_vector_value_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.45,
        cv::Scalar(170, 174, 180),
        1,
        cv::LINE_AA);
      cv::putText(
        bar,
        "Need valid depth on at least one matched blob",
        seek_vector_time_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.40,
        cv::Scalar(150, 154, 160),
        1,
        cv::LINE_AA);
    }

    const auto draw_slider = [&](const UiSlider &slider, int value, const cv::Scalar &accent)
    {
      cv::rectangle(bar, slider.track_rect, cv::Scalar(68, 73, 79), cv::FILLED);
      cv::rectangle(bar, slider.track_rect, cv::Scalar(93, 99, 106), 1);
      const int center_y = slider.track_rect.y + slider.track_rect.height / 2;
      cv::line(
        bar,
        cv::Point(slider.track_rect.x + 1, center_y),
        cv::Point(slider.track_rect.x + slider.track_rect.width - 1, center_y),
        accent,
        2,
        cv::LINE_AA);
      const double ratio = static_cast<double>(value - slider.min_value) /
        std::max(1, slider.max_value - slider.min_value);
      const int knob_x = slider.track_rect.x +
        static_cast<int>(std::round(ratio * slider.track_rect.width));
      cv::circle(bar, cv::Point(knob_x, center_y), 8, cv::Scalar(245, 245, 245), -1, cv::LINE_AA);
      cv::circle(bar, cv::Point(knob_x, center_y), 8, cv::Scalar(96, 100, 106), 1, cv::LINE_AA);
    };

    cv::putText(
      bar,
      cv::format("Window %.1fs", seekWindowSeconds()),
      seek_window_label_origin_,
      cv::FONT_HERSHEY_DUPLEX,
      0.48,
      cv::Scalar(225, 230, 236),
      1,
      cv::LINE_AA);
    draw_slider(seek_window_slider_, seek_window_tenths_, cv::Scalar(140, 210, 250));

    cv::putText(
      bar,
      cv::format("Decay %.1fs", seekDecaySeconds()),
      seek_decay_label_origin_,
      cv::FONT_HERSHEY_DUPLEX,
      0.48,
      cv::Scalar(225, 230, 236),
      1,
      cv::LINE_AA);
    draw_slider(seek_decay_slider_, seek_decay_tenths_, cv::Scalar(154, 230, 170));

    cv::putText(
      bar,
      cv::format("Edge Tolerance +/-%d%%", blob_tolerance_percent_),
      tolerance_label_origin_,
      cv::FONT_HERSHEY_DUPLEX,
      0.48,
      cv::Scalar(225, 230, 236),
      1,
      cv::LINE_AA);
    draw_slider(tolerance_slider_, blob_tolerance_percent_, cv::Scalar(85, 225, 255));

    cv::rectangle(bar, status_panel_rect_, cv::Scalar(34, 36, 40), cv::FILLED);
    cv::rectangle(bar, status_panel_rect_, cv::Scalar(72, 77, 84), 1);
    const std::string default_status = selected_profile_index_ >= 0 &&
        selected_profile_index_ < static_cast<int>(item_profiles_.size())
      ? "Ready | Profile: " + item_profiles_[selected_profile_index_].path.filename().string()
      : "Ready";
    const std::string status_text = profile_status_message_.empty()
      ? default_status
      : profile_status_message_;
    cv::putText(
      bar,
      "Status",
      cv::Point(status_panel_rect_.x + 10, status_panel_rect_.y + 19),
      cv::FONT_HERSHEY_DUPLEX,
      0.43,
      cv::Scalar(202, 208, 214),
      1,
      cv::LINE_AA);
    cv::putText(
      bar,
      fitTextToWidth(status_text, status_panel_rect_.width - 88, 0.44, 1),
      cv::Point(status_panel_rect_.x + 70, status_panel_rect_.y + 19),
      cv::FONT_HERSHEY_DUPLEX,
      0.44,
      cv::Scalar(194, 199, 206),
      1,
      cv::LINE_AA);

    return bar;
  }

  cv::Mat drawWindowFrame(const cv::Mat &image)
  {
    const cv::Mat preview_canvas = buildPreviewCanvas(image);
    cv::Mat top_bar = drawTopBar(preview_canvas.cols);
    cv::Mat combined;
    cv::vconcat(top_bar, preview_canvas, combined);
    drawDeleteConfirmationOverlay(combined);
    return combined;
  }

  void layoutDeleteConfirmationDialog(const cv::Size &size)
  {
    const int dialog_width = std::clamp(size.width - 120, 420, 640);
    const int dialog_height = 170;
    const int dialog_x = std::max(20, (size.width - dialog_width) / 2);
    const int dialog_y = std::max(20, (size.height - dialog_height) / 2);
    delete_confirm_dialog_rect_ = cv::Rect(dialog_x, dialog_y, dialog_width, dialog_height);

    const int button_width = 126;
    const int button_height = 36;
    const int button_gap = 12;
    const int button_y = dialog_y + dialog_height - button_height - 16;
    delete_confirm_cancel_button_rect_ = cv::Rect(
      dialog_x + dialog_width - (2 * button_width + button_gap + 16),
      button_y,
      button_width,
      button_height);
    delete_confirm_accept_button_rect_ = cv::Rect(
      delete_confirm_cancel_button_rect_.x + button_width + button_gap,
      button_y,
      button_width,
      button_height);
  }

  void drawDeleteConfirmationOverlay(cv::Mat &frame)
  {
    if (!delete_confirm_active_)
    {
      return;
    }

    layoutDeleteConfirmationDialog(frame.size());

    cv::Mat shaded = frame.clone();
    cv::rectangle(
      shaded,
      cv::Rect(0, 0, frame.cols, frame.rows),
      cv::Scalar(0, 0, 0),
      cv::FILLED);
    cv::addWeighted(shaded, 0.38, frame, 0.62, 0.0, frame);

    const cv::Rect clipped_dialog = delete_confirm_dialog_rect_ & cv::Rect(0, 0, frame.cols, frame.rows);
    cv::rectangle(frame, clipped_dialog, cv::Scalar(42, 45, 50), cv::FILLED);
    cv::rectangle(frame, clipped_dialog, cv::Scalar(122, 126, 132), 2);

    const std::string title = "Confirm Item Delete";
    const std::string target_name = (selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(item_profiles_.size()))
      ? item_profiles_[selected_profile_index_].path.filename().string()
      : "selected profile";
    const std::string message = "Delete profile: " + target_name;
    const std::string warning = "This action cannot be undone.";

    cv::putText(
      frame,
      title,
      cv::Point(delete_confirm_dialog_rect_.x + 16, delete_confirm_dialog_rect_.y + 34),
      cv::FONT_HERSHEY_DUPLEX,
      0.66,
      cv::Scalar(242, 242, 242),
      1,
      cv::LINE_AA);
    cv::putText(
      frame,
      fitTextToWidth(message, delete_confirm_dialog_rect_.width - 32, 0.56, 1),
      cv::Point(delete_confirm_dialog_rect_.x + 16, delete_confirm_dialog_rect_.y + 74),
      cv::FONT_HERSHEY_SIMPLEX,
      0.56,
      cv::Scalar(214, 218, 224),
      1,
      cv::LINE_AA);
    cv::putText(
      frame,
      warning,
      cv::Point(delete_confirm_dialog_rect_.x + 16, delete_confirm_dialog_rect_.y + 100),
      cv::FONT_HERSHEY_SIMPLEX,
      0.52,
      cv::Scalar(175, 182, 190),
      1,
      cv::LINE_AA);

    cv::rectangle(frame, delete_confirm_cancel_button_rect_, cv::Scalar(74, 78, 84), cv::FILLED);
    cv::rectangle(frame, delete_confirm_cancel_button_rect_, cv::Scalar(140, 144, 150), 2);
    cv::putText(
      frame,
      "Cancel",
      cv::Point(delete_confirm_cancel_button_rect_.x + 30, delete_confirm_cancel_button_rect_.y + 24),
      cv::FONT_HERSHEY_DUPLEX,
      0.52,
      cv::Scalar(245, 245, 245),
      1,
      cv::LINE_AA);

    cv::rectangle(frame, delete_confirm_accept_button_rect_, cv::Scalar(90, 76, 152), cv::FILLED);
    cv::rectangle(frame, delete_confirm_accept_button_rect_, cv::Scalar(170, 156, 245), 2);
    cv::putText(
      frame,
      "Delete",
      cv::Point(delete_confirm_accept_button_rect_.x + 30, delete_confirm_accept_button_rect_.y + 24),
      cv::FONT_HERSHEY_DUPLEX,
      0.52,
      cv::Scalar(245, 245, 245),
      1,
      cv::LINE_AA);
  }

  void depthCallback(const ImageMsg::ConstSharedPtr msg)
  {
    cv::Mat depth_m;
    if (!convertDepthToMeters(msg, depth_m))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = depth_m;
  }

  void cameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_camera_info_ = msg;
  }

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string overlay_topic_;
  std::string seek_pose_topic_;
  std::string item_pose_array_topic_;
  std::string item_cube_marker_topic_;
  std::string selected_profile_topic_;
  std::string seek_service_name_;
  std::string repick_service_name_;
  std::string seek_complete_service_name_;
  std::string seek_status_service_name_;
	  std::string go_to_teach_service_name_;
	  std::string movj_service_name_;
	  std::string camera_control_service_root_ {"/bin_camera"};
	  std::string calibration_parent_frame_;
  std::string calibration_child_frame_;
  std::string calibration_dir_;
  std::string calibration_file_;
  std::string robot_ip_address_;
  std::filesystem::path runtime_settings_path_;
  std::filesystem::path selected_profile_export_path_;
  std::string profiles_dir_;
  std::string teach_date_;
  std::string profile_status_message_;
  std::string camera_frame_id_;
  std::string item_frame_id_;
  bool use_calibration_ {true};
  bool publish_static_calibration_tf_ {true};
  bool auto_discover_calibration_ {true};
  bool publish_overlay_ {true};
  bool publish_item_cube_marker_ {true};
  bool align_item_z_axis_to_depth_plane_ {true};
  bool headless_ {false};
  bool visualization_window_created_ {false};
  bool detection_use_depth_ {false};
  cv::Size rendered_window_size_ {};
	  int red_threshold_ {120};
	  int green_threshold_ {120};
	  int blue_threshold_ {120};
	  int color_exposure_us_ {0};
	  int depth_exposure_us_ {0};
	  int color_exposure_min_us_ {kDefaultExposureMinUs};
	  int color_exposure_max_us_ {kDefaultExposureMaxUs};
	  int depth_exposure_min_us_ {kDefaultExposureMinUs};
	  int depth_exposure_max_us_ {kDefaultExposureMaxUs};
	  int rgb_hole_fill_sensitivity_ {0};
  int rgb_mask_dilate_px_ {kRgbDilateMinPx};
  int depth_null_fill_sensitivity_ {0};
  int depth_window_mm_ {5};
  int depth_hole_fill_sensitivity_ {0};
  int depth_trim_px_ {0};
  int adaptive_depth_trim_max_add_px_ {kAdaptiveDepthTrimAddDefaultPx};
  int adaptive_depth_trim_max_height_mm_ {kAdaptiveDepthTrimHeightDefaultMm};
  int depth_threshold_mm_ {10};
  DepthPlaneModel depth_plane_model_;
  int ray_step_px_ {3};
  int depth_edge_offset_px_ {4};
  int previous_color_percent_ {kDefaultPreviousColorPercent};
  int horizontal_ray_count_ {50};
  int vertical_ray_count_ {50};
  int outlier_sensitivity_ {50};
  int blob_tolerance_percent_ {kBlobToleranceDefaultPercent};
  bool detect_black_to_white_ {true};
  bool focus_black_mask_ {false};
  bool trace_out_to_in_ {false};
  DisplayView display_view_ {DisplayView::kBinarized};
  bool overlay_enabled_ {true};
  bool seek_mode_active_ {false};
  bool seek_result_latched_ {false};
  bool seek_window_started_ {false};
  bool delete_confirm_active_ {false};
  bool profile_dropdown_open_ {false};
  bool tolerance_slider_active_ {false};
  bool seek_window_slider_active_ {false};
		  bool seek_decay_slider_active_ {false};
		  bool camera_exposure_dirty_ {true};
		  bool window_close_requested_ {false};
		  std::string item_name_ {"item"};
  std::array<double, 6> teach_joints_deg_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_teach_joints_ {false};
  bool has_taught_item_dimensions_ {false};
  double taught_item_length_mm_ {0.0};
  double taught_item_width_mm_ {0.0};
  bool go_to_teach_in_progress_ {false};
  double item_marker_thickness_mm_ {15.0};
  double motion_update_period_sec_ {0.1};
  std::size_t seek_motion_history_max_samples_ {kSeekMotionHistoryMaxSamples};
  int seek_window_tenths_ {1};
  int seek_decay_tenths_ {1};
  int seek_valid_frame_count_ {0};
  bool debug_images_enabled_ {false};
  std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds_;
  std::optional<std::array<double, 4>> depth_plane_roi_normalized_;
  UiButton view_toggle_button_ {"View", cv::Rect(18, 14, 180, 42)};
  UiButton overlay_toggle_button_ {"Overlay", cv::Rect(18, 14, 150, 42)};
  UiButton seek_toggle_button_ {"Seek", cv::Rect(18, 14, 120, 42)};
  UiButton debug_images_button_ {"Debug Img", cv::Rect(18, 14, 160, 42)};
  UiButton delete_button_ {"Delete Item", cv::Rect(18, 14, 160, 42)};
  UiButton go_to_teach_button_ {"Go to Teach", cv::Rect(18, 14, 160, 42)};
  UiSlider tolerance_slider_ {"Tolerance", cv::Rect(), kBlobToleranceMinPercent, kBlobToleranceMaxPercent};
  UiSlider seek_window_slider_ {"Seek Window", cv::Rect(), 1, 60};
  UiSlider seek_decay_slider_ {"Seek Decay", cv::Rect(), 1, 10};
  cv::Rect profile_dropdown_rect_;
  cv::Rect delete_confirm_dialog_rect_;
  cv::Rect delete_confirm_cancel_button_rect_;
  cv::Rect delete_confirm_accept_button_rect_;
  cv::Rect seek_vector_panel_rect_;
  cv::Rect seek_controls_panel_rect_;
  cv::Rect quality_panel_rect_;
  cv::Rect status_panel_rect_;
  cv::Point tolerance_label_origin_;
  cv::Point seek_window_label_origin_;
  cv::Point seek_decay_label_origin_;
  cv::Point seek_vector_label_origin_;
  cv::Point seek_vector_value_origin_;
  cv::Point seek_vector_time_origin_;
  std::vector<cv::Rect> profile_option_rects_;
  std::vector<ItemProfile> item_profiles_;
  std::vector<cv::Point2f> roi_points_;
  std::vector<cv::Point2f> roi_points_normalized_;
  cv::Size roi_points_source_image_size_;
  std::optional<PoseBlobReference2D> pose_blob_reference_;
  std::vector<PoseReferenceSlot2D> pose_reference_slots_;
  std::size_t fallback_pose_slot_cursor_ {0};
  std::map<int, cv::Mat> failed_first_blob_masks_by_slot_;
  cv::Size failed_first_blob_mask_size_;
  int selected_profile_index_ {-1};
  std::filesystem::path selected_profile_path_;
	  rclcpp::Time seek_window_start_stamp_ {0, 0, RCL_ROS_TIME};
	  rclcpp::Time last_camera_exposure_attempt_time_ {0, 0, RCL_ROS_TIME};
	  int last_applied_color_exposure_us_ {-1};
	  int last_applied_depth_exposure_us_ {-1};
	  std::optional<SeekCapture> seek_last_valid_capture_;
  std::deque<SeekMotionSample> seek_valid_motion_samples_;
  cv::Mat seek_result_freeze_frame_;
  std::chrono::steady_clock::time_point seek_result_freeze_until_;
  ItemSummary item_summary_;
  std::string seek_snapshots_dir_;

  rclcpp::Publisher<ImageMsg>::SharedPtr overlay_pub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr seek_pose_pub_;
  rclcpp::Publisher<PoseArrayMsg>::SharedPtr item_pose_array_pub_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr item_cube_marker_pub_;
  rclcpp::Publisher<StringMsg>::SharedPtr selected_profile_pub_;
  rclcpp::Service<TriggerSrv>::SharedPtr seek_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr repick_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr seek_complete_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr seek_status_service_;
	  rclcpp::Service<TriggerSrv>::SharedPtr go_to_teach_service_;
	  rclcpp::TimerBase::SharedPtr camera_status_timer_;
	  rclcpp::TimerBase::SharedPtr camera_exposure_timer_;
	  rclcpp::Client<MovJSrv>::SharedPtr movj_client_;
	  rclcpp::Client<SetBoolSrv>::SharedPtr color_auto_exposure_client_;
	  rclcpp::Client<SetInt32Srv>::SharedPtr color_exposure_client_;
	  rclcpp::Client<SetBoolSrv>::SharedPtr depth_auto_exposure_client_;
	  rclcpp::Client<SetInt32Srv>::SharedPtr depth_exposure_client_;
	  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;

  geometry_msgs::msg::Quaternion calibration_rotation_;
  geometry_msgs::msg::Vector3 calibration_translation_;
  CameraCalibrationMetadata calibration_metadata_;

  std::mutex data_mutex_;
  cv::Mat latest_depth_;
  CameraInfoMsg::ConstSharedPtr latest_camera_info_;
  std::chrono::steady_clock::time_point last_camera_render_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ItemDetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
