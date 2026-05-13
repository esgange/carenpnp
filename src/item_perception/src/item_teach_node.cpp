#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <dobot_msgs_v4/srv/mov_j.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <orbbec_camera_msgs/srv/set_int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <dobot_common/workspace_paths.hpp>

namespace
{
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using JointStateMsg = sensor_msgs::msg::JointState;
using MovJSrv = dobot_msgs_v4::srv::MovJ;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetInt32Srv = orbbec_camera_msgs::srv::SetInt32;
using PoseArrayMsg = geometry_msgs::msg::PoseArray;
using QuaternionMsg = geometry_msgs::msg::Quaternion;

constexpr char kWindowName[] = "item_teach_view";
constexpr char kColorExposureTrackbar[] = "RGB Exposure us";
constexpr char kRedTrackbar[] = "R Threshold";
constexpr char kGreenTrackbar[] = "G Threshold";
constexpr char kBlueTrackbar[] = "B Threshold";
constexpr char kRgbHoleFillTrackbar[] = "Hole Fill Sens.";
constexpr char kRgbDilateTrackbar[] = "RGB Dilate Px";
constexpr char kDepthNullFillTrackbar[] = "Depth Null Fill";
constexpr char kDepthWindowTrackbar[] = "Depth Window mm";
constexpr char kDepthHoleFillTrackbar[] = "Depth Hole Fill";
constexpr char kDepthTrimTrackbar[] = "Depth Trim Px";
constexpr char kAdaptiveDepthTrimFactorTrackbar[] = "Trim Add Px Max";
constexpr char kAdaptiveDepthTrimHeightTrackbar[] = "Trim Max Height mm";
constexpr char kBlobToleranceTrackbar[] = "Edge Tolerance +/- %";
constexpr int kButtonBarHeight = 122;
constexpr int kLeftPanelWidth = 440;
constexpr int kVideoTopBarHeight = 122;
constexpr int kPreviewCanvasWidth = 1080;
constexpr int kPreviewCanvasHeight = 680;
constexpr double kMinOutlierDistancePx = 4.0;
constexpr int kMaxSideTrimIterations = 24;
constexpr int kDefaultPreviousColorPercent = 60;
constexpr double kNextColorConfirmMatchRatio = 0.60;
constexpr double kRuntimeSettingsSaveIntervalSec = 0.75;
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
constexpr int kAdaptiveDepthTrimFactorMinTenths = 1;
constexpr int kAdaptiveDepthTrimFactorMaxTenths = 50;
constexpr int kAdaptiveDepthTrimFactorDefaultTenths = 2;
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
constexpr int kTeachColorExposureMaxUs = 100;
constexpr int kPoseReferenceSlotCount = 4;
constexpr int kDepthEdgeOffsetMinPx = 1;
constexpr int kDepthEdgeOffsetMaxPx = 20;
constexpr int kTeachFixedRayStepPx = 3;
constexpr int kTeachFixedDepthEdgeOffsetPx = 4;
constexpr int kTeachFixedPreviousColorPercent = kDefaultPreviousColorPercent;
constexpr int kTeachFixedHorizontalRayCount = 50;
constexpr int kTeachFixedVerticalRayCount = 50;
constexpr int kTeachFixedOutlierSensitivity = 50;
constexpr bool kTeachFixedDetectBlackToWhite = true;
constexpr bool kTeachFixedTraceOutToIn = false;

double remapOutlierSensitivityToFitRange(int outlier_sensitivity)
{
  const int clamped = std::clamp(outlier_sensitivity, 1, 100);
  return 50.0 + (static_cast<double>(clamped - 1) * 100.0 / 99.0);
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

int exposureUsecToPercent(int exposure_us, int min_us, int max_us)
{
  if (exposure_us <= 0)
  {
    return 0;
  }
  const int clamped_min = clampExposureUsec(min_us);
  const int clamped_max = std::max(clamped_min, clampExposureUsec(max_us));
  if (clamped_max == clamped_min)
  {
    return 100;
  }
  const int clamped_exposure = std::clamp(exposure_us, clamped_min, clamped_max);
  const double t =
    static_cast<double>(clamped_exposure - clamped_min) /
    static_cast<double>(clamped_max - clamped_min);
  return clampExposurePercent(static_cast<int>(std::lround(t * 100.0)));
}

int parseSavedAdaptiveDepthTrimAddPx(
  const YAML::Node &node,
  int fallback_add_px = kAdaptiveDepthTrimFactorDefaultTenths)
{
  if (!node || !node.IsScalar())
  {
    return std::clamp(
      fallback_add_px,
      kAdaptiveDepthTrimFactorMinTenths,
      kAdaptiveDepthTrimFactorMaxTenths);
  }

  return std::clamp(
    node.as<int>(),
    kAdaptiveDepthTrimFactorMinTenths,
    kAdaptiveDepthTrimFactorMaxTenths);
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

struct DateStamp
{
  std::string iso_date;
  std::string compact_date;
};

std::tm toLocalTime(std::time_t time_value)
{
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &time_value);
#else
  localtime_r(&time_value, &tm);
#endif
  return tm;
}

DateStamp currentDateStamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  const std::tm local_tm = toLocalTime(now_time);

  std::ostringstream iso_stream;
  iso_stream << std::put_time(&local_tm, "%Y-%m-%d");

  std::ostringstream compact_stream;
  compact_stream << std::put_time(&local_tm, "%d%m%Y");

  return DateStamp{iso_stream.str(), compact_stream.str()};
}

std::string makeFilenameSafeItemName(const std::string &item_name)
{
  std::string safe_name;
  safe_name.reserve(item_name.size());

  for (const unsigned char ch : item_name)
  {
    if (std::isalnum(ch) != 0)
    {
      safe_name.push_back(static_cast<char>(std::tolower(ch)));
      continue;
    }

    if (ch == '_' || ch == '-')
    {
      safe_name.push_back(static_cast<char>(ch));
      continue;
    }

    if (ch == ' ' && (safe_name.empty() || safe_name.back() != '_'))
    {
      safe_name.push_back('_');
    }
  }

  while (!safe_name.empty() && safe_name.front() == '_')
  {
    safe_name.erase(safe_name.begin());
  }
  while (!safe_name.empty() && safe_name.back() == '_')
  {
    safe_name.pop_back();
  }

  return safe_name.empty() ? "item" : safe_name;
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

  // Fill enclosed voids in the binary foreground while keeping outside background intact.
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
      static_cast<int>(std::round(
        max_fraction * static_cast<double>(mask.rows * mask.cols))));
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
      const float height_m = -residual;
      if (height_m >= min_keep_height_m)
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

  // Trim as an exact inward edge offset: keep only pixels whose Euclidean
  // distance to the nearest background pixel is greater than trim_px.
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

  const int clamped_add_px = std::clamp(
    max_add_px,
    kAdaptiveDepthTrimFactorMinTenths,
    kAdaptiveDepthTrimFactorMaxTenths);
  const int clamped_max_height_mm = std::clamp(
    max_height_mm,
    kAdaptiveDepthTrimHeightMinMm,
    kAdaptiveDepthTrimHeightMaxMm);
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

  const int clamped_sensitivity = std::clamp(
    fill_sensitivity,
    kDepthFillSensitivityMin,
    kDepthFillSensitivityMax);
  if (clamped_sensitivity <= 0)
  {
    return depth_values_m.clone();
  }

  cv::Mat filled_depth = depth_values_m.clone();
  // Null-fill targets sensor-null depth (0/NaN/inf), so only positive finite depth counts as valid seed data.
  cv::Mat valid_mask = buildPositiveFiniteDepthMask(filled_depth);
  if (valid_mask.empty() || cv::countNonZero(valid_mask) == 0)
  {
    return filled_depth;
  }

  // Fill invalid/raw-null depth from nearby valid neighbors.
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
  if (max_val - min_val < 1e-6)
  {
    return cv::Mat(depth_m.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  }

  cv::Mat normalized;
  depth_m.convertTo(
    normalized,
    CV_8UC1,
    255.0 / (max_val - min_val),
    -min_val * 255.0 / (max_val - min_val));
  normalized.setTo(0, ~valid_mask);

  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
  colored.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
  return colored;
}

cv::Mat colorizeDepthResidual(const cv::Mat &depth_residual_m)
{
  if (depth_residual_m.empty() || depth_residual_m.type() != CV_32FC1)
  {
    return {};
  }

  const cv::Mat valid_mask = buildFiniteDepthMask(depth_residual_m);
  if (valid_mask.empty() || cv::countNonZero(valid_mask) == 0)
  {
    return cv::Mat(depth_residual_m.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  }

  double min_val = 0.0;
  double max_val = 0.0;
  cv::minMaxLoc(depth_residual_m, &min_val, &max_val, nullptr, nullptr, valid_mask);

  cv::Mat normalized(depth_residual_m.size(), CV_8UC1, cv::Scalar(0));
  if (max_val - min_val < 1e-6)
  {
    normalized.setTo(cv::Scalar(127), valid_mask);
  }
  else
  {
    depth_residual_m.convertTo(
      normalized,
      CV_8UC1,
      255.0 / (max_val - min_val),
      -min_val * 255.0 / (max_val - min_val));
    normalized.setTo(0, ~valid_mask);
  }

  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
  colored.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
  return colored;
}

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
  std::optional<PoseBlobReference2D> reference;
  std::vector<cv::Point> clicks_px;
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

struct SideDepthSample
{
  double t {0.0};
  double depth_m {0.0};
  cv::Vec3d camera_point {0.0, 0.0, 0.0};
};

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

double pointToLineDistance(const cv::Point2f &point, const LineModel &line)
{
  return pointToLineDistance(point, line.point, line.point + line.direction);
}

LineModel fitLineToPoints(
  const std::vector<cv::Point2f> &points,
  const cv::Point2f &fallback_a,
  const cv::Point2f &fallback_b);

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
        const double distance = pointToLineDistance(point, candidate_line);
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
      distances.push_back(pointToLineDistance(point, result.line));
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

double medianValue(std::vector<double> values)
{
  if (values.empty())
  {
    return 0.0;
  }

  const auto middle = values.begin() + static_cast<long>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  double median = *middle;
  if (values.size() % 2 == 0)
  {
    const auto lower = std::max_element(values.begin(), middle);
    median = 0.5 * (median + *lower);
  }
  return median;
}

LineModel fitLineToPoints(const std::vector<cv::Point2f> &points, const cv::Point2f &fallback_a, const cv::Point2f &fallback_b)
{
  if (points.size() < 2)
  {
    const cv::Point2f dir = fallback_b - fallback_a;
    return LineModel{fallback_a, dir};
  }

  cv::Vec4f line;
  cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);
  return LineModel{
    cv::Point2f(line[2], line[3]),
    cv::Point2f(line[0], line[1])
  };
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

cv::Vec3d projectPixelToCamera(const cv::Point2f &pixel, double depth_m, const CameraInfoMsg &camera_info);

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
    if (allowed_mask->empty() || allowed_mask->type() != CV_8UC1 ||
      allowed_mask->rows != depth_m.rows || allowed_mask->cols != depth_m.cols)
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

std::optional<cv::Vec3d> estimatePoseMemberCenter3D(
  const cv::Point2f &member_center_px,
  const std::vector<cv::Point> &member_pixels,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  const cv::Mat *depth_sample_mask = nullptr)
{
  if (!member_pixels.empty())
  {
    if (const auto robust_center = estimateBlobCenterFromPixels(member_pixels, depth_m, camera_info);
      robust_center.has_value())
    {
      return robust_center;
    }
  }
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6)
  {
    return std::nullopt;
  }
  if (const auto depth = averageDepthAt(depth_m, member_center_px, 7, depth_sample_mask);
    depth.has_value())
  {
    return projectPixelToCamera(member_center_px, *depth, camera_info);
  }
  if (const auto depth = averageDepthAt(depth_m, member_center_px, 7, nullptr);
    depth.has_value())
  {
    return projectPixelToCamera(member_center_px, *depth, camera_info);
  }
  return std::nullopt;
}

std::optional<cv::Vec3d> estimatePairPoseCenter3D(
  const BinarizedPoseEstimate2D::BlobPose2D &blob_pose,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  const cv::Mat *depth_sample_mask = nullptr)
{
  if (blob_pose.member_count < 2 || blob_pose.member_centers_px.size() < 2)
  {
    return std::nullopt;
  }

  const cv::Point2f anchor_center_px =
    blob_pose.has_custom_anchor ? blob_pose.anchor_point_px : blob_pose.member_centers_px.front();
  const auto anchor_center = estimatePoseMemberCenter3D(
    anchor_center_px,
    blob_pose.anchor_pixels,
    depth_m,
    camera_info,
    depth_sample_mask);
  const auto companion_center = estimatePoseMemberCenter3D(
    blob_pose.member_centers_px[1],
    blob_pose.companion_pixels,
    depth_m,
    camera_info,
    depth_sample_mask);
  if (!anchor_center.has_value() || !companion_center.has_value())
  {
    return std::nullopt;
  }

  return (*anchor_center + *companion_center) * 0.5;
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

std::optional<cv::Point2f> projectCameraToPixel(
  const cv::Vec3d &point,
  const CameraInfoMsg &camera_info)
{
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6 || point[2] <= 1e-6)
  {
    return std::nullopt;
  }

  return cv::Point2f(
    static_cast<float>((point[0] * camera_info.k[0] / point[2]) + camera_info.k[2]),
    static_cast<float>((point[1] * camera_info.k[4] / point[2]) + camera_info.k[5]));
}

double vectorNorm(const cv::Vec3d &vec)
{
  return std::sqrt(vec.dot(vec));
}

bool normalizeVectorInPlace(cv::Vec3d &vec)
{
  const double norm = vectorNorm(vec);
  if (norm < 1e-9)
  {
    return false;
  }
  vec *= (1.0 / norm);
  return true;
}

QuaternionMsg rotationToQuaternionMsg(const cv::Matx33d &rotation)
{
  QuaternionMsg quaternion;
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

  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm < 1e-9)
  {
    quaternion.w = 1.0;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    return quaternion;
  }

  quaternion.w = qw / norm;
  quaternion.x = qx / norm;
  quaternion.y = qy / norm;
  quaternion.z = qz / norm;
  return quaternion;
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
  blob_pose.hull_points = component.hull;
  blob_pose.corners = corners;
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

std::optional<BinarizedBlobComponent2D> selectPoseBlobComponentFromMask(
  const cv::Mat &mask,
  const cv::Point &seed_point_px,
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
  if (
    seed_point_px.x < 0 || seed_point_px.y < 0 ||
    seed_point_px.x >= mask.cols || seed_point_px.y >= mask.rows)
  {
    if (status_text != nullptr)
    {
      *status_text = "Clicked blob is outside image";
    }
    return std::nullopt;
  }

  cv::Mat labels;
  std::vector<BinarizedBlobComponent2D> components = extractConnectedBlobComponents(mask, &labels);
  if (components.empty() || labels.empty())
  {
    if (status_text != nullptr)
    {
      *status_text = "No blobs in final binarized mask";
    }
    return std::nullopt;
  }

  const int reference_label = labels.at<int>(seed_point_px.y, seed_point_px.x);
  if (reference_label <= 0)
  {
    if (status_text != nullptr)
    {
      *status_text = "Click directly on a white blob";
    }
    return std::nullopt;
  }

  const int max_label = labels.empty() ? 0 : (*std::max_element(labels.begin<int>(), labels.end<int>()));
  std::vector<int> label_to_component_idx(static_cast<std::size_t>(std::max(0, max_label + 1)), -1);
  for (std::size_t i = 0; i < components.size(); ++i)
  {
    const int label = components[i].label;
    if (label >= 0 && label < static_cast<int>(label_to_component_idx.size()))
    {
      label_to_component_idx[static_cast<std::size_t>(label)] = static_cast<int>(i);
    }
  }
  if (
    reference_label < 0 ||
    reference_label >= static_cast<int>(label_to_component_idx.size()) ||
    label_to_component_idx[static_cast<std::size_t>(reference_label)] < 0)
  {
    if (status_text != nullptr)
    {
      *status_text = "Clicked blob is too small; choose a larger blob";
    }
    return std::nullopt;
  }

  const BinarizedBlobComponent2D &selected_component = components[static_cast<std::size_t>(
      label_to_component_idx[static_cast<std::size_t>(reference_label)])];
  if (selected_component.hull.size() < 3)
  {
    if (status_text != nullptr)
    {
      *status_text = "Selected blob is invalid; choose another blob";
    }
    return std::nullopt;
  }

  if (status_text != nullptr)
  {
    *status_text = cv::format(
      "Blob selected: hull fill %.0f%% | rect area %d px",
      std::clamp(selected_component.fill_ratio, 0.0, 1.0) * 100.0,
      std::max(1, selected_component.area_px));
  }
  return selected_component;
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

std::optional<PoseBlobReference2D> buildPoseBlobReferenceFromBlobPose(
  const BinarizedPoseEstimate2D::BlobPose2D &blob_pose)
{
  std::vector<cv::Point> group_reference_hull = blob_pose.hull_points;
  if (group_reference_hull.size() < 3 && !blob_pose.corners.empty())
  {
    group_reference_hull.reserve(blob_pose.corners.size());
    for (const auto &corner : blob_pose.corners)
    {
      group_reference_hull.emplace_back(
        static_cast<int>(std::lround(corner.x)),
        static_cast<int>(std::lround(corner.y)));
    }
  }
  if (group_reference_hull.size() < 3)
  {
    return std::nullopt;
  }

  PoseBlobReference2D reference_blob;
  reference_blob.mode = blob_pose.member_count >= 2 ? PoseTemplateMode2D::kPair : PoseTemplateMode2D::kSingle;
  reference_blob.group_hull = group_reference_hull;
  reference_blob.group_area_px = std::max(
    1,
    static_cast<int>(std::round(fittedRectAreaPx(reference_blob.group_hull))));
  reference_blob.group_aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(reference_blob.group_hull));
  reference_blob.hull = group_reference_hull;
  reference_blob.area_px = reference_blob.group_area_px;
  reference_blob.aspect_ratio = reference_blob.group_aspect_ratio;
  double fill_ratio = computeBlobHullFillRatio(blob_pose.pixels, reference_blob.group_hull);
  if (reference_blob.mode == PoseTemplateMode2D::kPair)
  {
    if (const auto anchor_fill_ratio = computeSelfHullFillRatioForPixels(blob_pose.anchor_pixels);
      anchor_fill_ratio.has_value())
    {
      fill_ratio = *anchor_fill_ratio;
    }
    if (blob_pose.anchor_pixels.size() >= 3)
    {
      cv::convexHull(blob_pose.anchor_pixels, reference_blob.anchor_hull, true, true);
      reference_blob.hull = reference_blob.anchor_hull;
      reference_blob.area_px = std::max(
        1,
        static_cast<int>(std::round(fittedRectAreaPx(reference_blob.hull))));
      reference_blob.aspect_ratio = std::max(1e-3F, fittedRectAspectRatio(reference_blob.hull));
    }
    if (blob_pose.companion_pixels.size() >= 3)
    {
      if (const auto companion_fill_ratio = computeSelfHullFillRatioForPixels(blob_pose.companion_pixels);
        companion_fill_ratio.has_value())
      {
        reference_blob.companion_fill_ratio = *companion_fill_ratio;
      }
      cv::convexHull(blob_pose.companion_pixels, reference_blob.companion_hull, true, true);
      if (reference_blob.companion_hull.size() >= 3)
      {
        reference_blob.companion_area_px = std::max(
          1,
          static_cast<int>(std::round(fittedRectAreaPx(reference_blob.companion_hull))));
        reference_blob.companion_aspect_ratio = std::max(
          1e-3F,
          fittedRectAspectRatio(reference_blob.companion_hull));
        if (reference_blob.companion_fill_ratio <= 0.0)
        {
          reference_blob.companion_fill_ratio = computeBlobHullFillRatio(
            blob_pose.companion_pixels,
            reference_blob.companion_hull);
        }
      }
    }
  }
  reference_blob.fill_ratio = fill_ratio > 0.0 ? fill_ratio : 1.0;
  reference_blob.member_count = std::max(1, blob_pose.member_count);
  reference_blob.member_centers_norm = blob_pose.member_centers_norm;
  if (!blob_pose.member_centers_norm.empty())
  {
    reference_blob.anchor_center_norm = blob_pose.member_centers_norm.front();
  }
  return reference_blob;
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
      *status_text = "Reference blob missing. Click a blob";
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
        "No blobs matched ref size (edge +/- %d%%)",
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
    *status_text = "Matched blobs: " + std::to_string(estimate.matched_blob_count) +
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
      *status_text = "Pair reference missing. Select 2 blobs";
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
      return false;
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
  for (const auto &corner : *camera_points)
  {
    center += corner;
  }
  center *= 0.25;
  const auto robust_center = estimateBlobCenterFromPixels(blob_pose.pixels, depth_m, camera_info);
  if (robust_center.has_value())
  {
    center = *robust_center;
  }
  cv::Vec3d pose_origin = center;
  if (blob_pose.member_count >= 2)
  {
    if (const auto pair_center = estimatePairPoseCenter3D(
          blob_pose,
          depth_m,
          camera_info,
          mask_ptr);
      pair_center.has_value())
    {
      pose_origin = *pair_center;
    }
  }
  else if (blob_pose.has_custom_anchor)
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
    polygon.emplace_back(
      static_cast<int>(std::round(corner.x)),
      static_cast<int>(std::round(corner.y)));
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

std::vector<cv::Point2f> mergeAxisAlignedRois(
  const std::vector<cv::Point2f> &existing_roi,
  const std::vector<cv::Point2f> &new_roi)
{
  if (!hasValidRoiPoints(existing_roi))
  {
    return new_roi;
  }
  if (!hasValidRoiPoints(new_roi))
  {
    return existing_roi;
  }

  std::vector<cv::Point2f> all_points;
  all_points.reserve(existing_roi.size() + new_roi.size());
  all_points.insert(all_points.end(), existing_roi.begin(), existing_roi.end());
  all_points.insert(all_points.end(), new_roi.begin(), new_roi.end());
  return buildAxisAlignedRoiFromSelection(all_points);
}

struct AxisAlignedRoiBounds
{
  int left {0};
  int top {0};
  int right {0};
  int bottom {0};
};

struct DepthPlaneModel
{
  double a {0.0};
  double b {0.0};
  double c {0.0};
  double reference_depth_m {0.0};
  bool valid {false};
};

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

double normalizedImageCoord(int value, int max_value)
{
  if (max_value <= 1)
  {
    return 0.0;
  }
  return static_cast<double>(value) / static_cast<double>(max_value - 1);
}

bool fitDepthPlaneFromCorners(
  const cv::Mat &depth_m,
  const std::vector<cv::Point2f> &corners,
  DepthPlaneModel &plane_out)
{
  plane_out = DepthPlaneModel{};
  if (depth_m.empty() || depth_m.type() != CV_32FC1 || corners.size() != 4)
  {
    return false;
  }

  cv::Mat A(4, 3, CV_64F);
  cv::Mat b(4, 1, CV_64F);
  double positive_depth_sum = 0.0;
  int positive_depth_count = 0;
  for (int i = 0; i < 4; ++i)
  {
    const cv::Point2f corner = corners[static_cast<std::size_t>(i)];
    const auto averaged_depth = averageDepthAt(depth_m, corner, 7);
    const double depth_value = averaged_depth.has_value() ? *averaged_depth : 0.0;
    const int px = static_cast<int>(std::lround(corner.x));
    const int py = static_cast<int>(std::lround(corner.y));
    const double x_norm = normalizedImageCoord(std::clamp(px, 0, depth_m.cols - 1), depth_m.cols);
    const double y_norm = normalizedImageCoord(std::clamp(py, 0, depth_m.rows - 1), depth_m.rows);
    A.at<double>(i, 0) = x_norm;
    A.at<double>(i, 1) = y_norm;
    A.at<double>(i, 2) = 1.0;
    b.at<double>(i, 0) = depth_value;
    if (std::isfinite(depth_value) && depth_value > 0.0)
    {
      positive_depth_sum += depth_value;
      ++positive_depth_count;
    }
  }

  if (positive_depth_count == 0)
  {
    return false;
  }

  cv::Mat x;
  if (!cv::solve(A, b, x, cv::DECOMP_SVD))
  {
    return false;
  }

  const double a = x.at<double>(0, 0);
  const double b_coeff = x.at<double>(1, 0);
  const double c = x.at<double>(2, 0);
  const double reference_depth_m = positive_depth_sum / static_cast<double>(positive_depth_count);
  if (!std::isfinite(a) || !std::isfinite(b_coeff) || !std::isfinite(c) || !std::isfinite(reference_depth_m) ||
      reference_depth_m <= 0.0)
  {
    return false;
  }

  plane_out.a = a;
  plane_out.b = b_coeff;
  plane_out.c = c;
  plane_out.reference_depth_m = reference_depth_m;
  plane_out.valid = true;
  return true;
}

bool fitDepthPlaneFromRoiBounds(
  const cv::Mat &depth_m,
  const AxisAlignedRoiBounds &bounds,
  DepthPlaneModel &plane_out)
{
  if (!isValidRoiBounds(bounds))
  {
    return false;
  }
  const auto corners = roiPointsFromBounds(bounds);
  return fitDepthPlaneFromCorners(depth_m, corners, plane_out);
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
      residual_row[x] = std::isfinite(residual_depth)
        ? static_cast<float>(residual_depth)
        : kNullDepth;
    }
  }
  return residual;
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

std::optional<double> depthPlaneDepthAtPixel(
  const DepthPlaneModel &plane,
  const cv::Point2f &pixel,
  const cv::Size &image_size)
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

std::optional<cv::Vec3d> depthPlaneNormalInCameraFrame(
  const DepthPlaneModel &plane,
  const CameraInfoMsg &camera_info,
  const cv::Size &image_size,
  const cv::Point2f &center_px,
  const std::optional<cv::Vec3d> &preferred_sign = std::nullopt)
{
  if (
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

cv::Mat largestConnectedComponentMask(const cv::Mat &mask, int min_component_pixels)
{
  if (mask.empty() || mask.type() != CV_8UC1)
  {
    return {};
  }

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
  if (component_count <= 1)
  {
    return cv::Mat::zeros(mask.size(), CV_8UC1);
  }

  int best_label = -1;
  int best_area = 0;
  for (int label = 1; label < component_count; ++label)
  {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area > best_area)
    {
      best_area = area;
      best_label = label;
    }
  }

  if (best_label < 1 || best_area < std::max(1, min_component_pixels))
  {
    return cv::Mat::zeros(mask.size(), CV_8UC1);
  }

  cv::Mat output(mask.size(), CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < labels.rows; ++y)
  {
    const int *label_row = labels.ptr<int>(y);
    unsigned char *out_row = output.ptr<unsigned char>(y);
    for (int x = 0; x < labels.cols; ++x)
    {
      if (label_row[x] == best_label)
      {
        out_row[x] = 255;
      }
    }
  }

  return output;
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

std::vector<cv::Point2f> mergeRoiPolygonWithRegion(
  const std::vector<cv::Point2f> &existing_roi,
  const AxisAlignedRoiBounds &new_region,
  const cv::Size &image_size)
{
  if (!isValidRoiBounds(new_region) || image_size.width <= 0 || image_size.height <= 0)
  {
    return existing_roi;
  }

  if (!hasValidRoiPoints(existing_roi))
  {
    return roiPointsFromBounds(new_region);
  }

  cv::Mat merged_mask(image_size, CV_8UC1, cv::Scalar(0));
  const cv::Mat existing_mask = buildRoiMask(image_size, existing_roi);
  merged_mask.setTo(255, existing_mask);

  const auto new_polygon = roiPolygonForImage(roiPointsFromBounds(new_region), image_size);
  if (new_polygon.size() >= 3)
  {
    const std::vector<std::vector<cv::Point>> polygons{new_polygon};
    cv::fillPoly(merged_mask, polygons, cv::Scalar(255));
  }

  return extractRoiPolygonFromMask(merged_mask);
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

void drawRoiOverlay(cv::Mat &image, const std::vector<cv::Point2f> &roi_points, bool roi_selection_active)
{
  if (roi_points.empty())
  {
    return;
  }

  const cv::Scalar line_color = roi_selection_active ? cv::Scalar(0, 180, 255) : cv::Scalar(255, 220, 0);
  const cv::Scalar point_color = roi_selection_active ? cv::Scalar(0, 120, 255) : cv::Scalar(0, 255, 255);
  const bool label_points = roi_selection_active || roi_points.size() <= 4;

  if (roi_points.size() >= 3)
  {
    cv::Mat overlay = image.clone();
    std::vector<cv::Point> polygon;
    polygon.reserve(roi_points.size());
    for (const auto &point : roi_points)
    {
      polygon.emplace_back(
        static_cast<int>(std::round(point.x)),
        static_cast<int>(std::round(point.y)));
    }
    const std::vector<std::vector<cv::Point>> polygons{polygon};
    cv::fillPoly(overlay, polygons, roi_selection_active ? cv::Scalar(0, 90, 160) : cv::Scalar(0, 130, 180));
    cv::addWeighted(overlay, 0.18, image, 0.82, 0.0, image);
  }

  for (std::size_t i = 0; i < roi_points.size(); ++i)
  {
    const cv::Point current(
      static_cast<int>(std::round(roi_points[i].x)),
      static_cast<int>(std::round(roi_points[i].y)));
    if (i > 0)
    {
      const cv::Point previous(
        static_cast<int>(std::round(roi_points[i - 1].x)),
        static_cast<int>(std::round(roi_points[i - 1].y)));
      cv::line(image, previous, current, line_color, 2, cv::LINE_AA);
    }
    if (!roi_selection_active && roi_points.size() >= 3 && i == roi_points.size() - 1)
    {
      const cv::Point first(
        static_cast<int>(std::round(roi_points.front().x)),
        static_cast<int>(std::round(roi_points.front().y)));
      cv::line(image, current, first, line_color, 2, cv::LINE_AA);
    }
    cv::circle(image, current, 7, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, current, 5, point_color, -1);
    if (label_points)
    {
      cv::putText(
        image,
        std::to_string(i + 1),
        current + cv::Point(10, -10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 255, 255),
        2);
    }
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
    drawRoiOverlay(image, roiPointsFromBounds(region), false);
  }
}

std::optional<cv::Point2f> findConfirmedMaskTransition(
  const cv::Mat &mask,
  const cv::Point2f &origin,
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
    const cv::Point2f candidate = origin + static_cast<float>(radius) * direction;
    if (sampleMaskAt(mask, candidate) != new_value)
    {
      continue;
    }

    int previous_matches = 0;
    int next_matches = 0;
    for (int offset = 1; offset <= clamped_confirm_px; ++offset)
    {
      const int prev_radius = radius - offset * step;
      const int next_radius = radius + offset * step;
      const cv::Point2f prev_point = origin + static_cast<float>(prev_radius) * direction;
      const cv::Point2f next_point = origin + static_cast<float>(next_radius) * direction;
      if (sampleMaskAt(mask, prev_point) == old_value)
      {
        ++previous_matches;
      }
      if (sampleMaskAt(mask, next_point) == new_value)
      {
        ++next_matches;
      }
    }

    if (previous_matches >= required_previous_matches && next_matches >= required_next_matches)
    {
      return candidate;
    }
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

void drawMeasurementAxes(cv::Mat &image, const std::optional<ItemEstimate> &estimate)
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
  x_dir *= (1.0F / std::sqrt(x_dir.dot(x_dir)));
  y_dir *= (1.0F / std::sqrt(y_dir.dot(y_dir)));

  constexpr float kAxisLenPx = 60.0F;
  const cv::Point2f x_end = origin + kAxisLenPx * x_dir;
  const cv::Point2f y_end = origin + kAxisLenPx * y_dir;

  cv::arrowedLine(image, origin, x_end, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.15);
  cv::arrowedLine(image, origin, y_end, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, 0.15);
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
    if (!axis_only && blob_pose.member_count >= 2)
    {
      const auto draw_member_hull = [&](const std::vector<cv::Point> &pixels, const cv::Scalar &color)
      {
        if (pixels.size() < 3)
        {
          return;
        }
        std::vector<cv::Point> member_hull;
        cv::convexHull(pixels, member_hull, true, true);
        if (member_hull.size() < 3)
        {
          return;
        }
        const std::vector<std::vector<cv::Point>> member_polys{member_hull};
        cv::polylines(image, member_polys, true, color, 2, cv::LINE_AA);
      };
      draw_member_hull(blob_pose.anchor_pixels, cv::Scalar(40, 255, 255));
      draw_member_hull(blob_pose.companion_pixels, cv::Scalar(255, 160, 40));
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

      const float long_side_px = std::max(blob_pose.x_length_px, blob_pose.z_length_px);
      const float short_side_px = std::min(blob_pose.x_length_px, blob_pose.z_length_px);
      const std::string size_label = cv::format("L %.1f px | H %.1f px", long_side_px, short_side_px);
      const cv::Size label_size = cv::getTextSize(size_label, cv::FONT_HERSHEY_SIMPLEX, 0.48, 2, nullptr);
      const int label_x = std::clamp(
        static_cast<int>(std::lround(center.x)) + 12,
        8,
        std::max(8, image.cols - label_size.width - 8));
      const int label_y = std::clamp(
        static_cast<int>(std::lround(center.y)) + 22,
        std::max(18, label_size.height + 8),
        std::max(18, image.rows - 8));
      cv::putText(
        image,
        size_label,
        cv::Point(label_x, label_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.48,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
        cv::putText(
          image,
          size_label,
          cv::Point(label_x, label_y),
          cv::FONT_HERSHEY_SIMPLEX,
          0.48,
          cv::Scalar(235, 245, 255),
          1,
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
          std::string member_label = std::to_string(static_cast<int>(member_index + 1)) + "/2";
          if (member_index < blob_pose.member_centers_norm.size())
          {
            const cv::Point2f &member_norm = blob_pose.member_centers_norm[member_index];
            member_label += cv::format(" %.2f %.2f", member_norm.x, member_norm.y);
          }
          cv::putText(
            image,
            member_label,
            member_center + cv::Point2f(6.0F, -8.0F),
            cv::FONT_HERSHEY_SIMPLEX,
            0.42,
            cv::Scalar(0, 0, 0),
            3,
            cv::LINE_AA);
          cv::putText(
            image,
            member_label,
            member_center + cv::Point2f(6.0F, -8.0F),
            cv::FONT_HERSHEY_SIMPLEX,
            0.42,
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

void drawViewLabel(cv::Mat &image, const std::string &label)
{
  if (image.empty())
  {
    return;
  }

  const cv::Rect box(14, 12, std::max(120, 18 + static_cast<int>(label.size()) * 12), 34);
  cv::rectangle(image, box, cv::Scalar(24, 24, 24), cv::FILLED);
  cv::rectangle(image, box, cv::Scalar(210, 210, 210), 2);
  cv::putText(
    image,
    label,
    cv::Point(box.x + 10, box.y + 23),
    cv::FONT_HERSHEY_SIMPLEX,
    0.62,
    cv::Scalar(255, 255, 255),
    2);
}

void tintMaskOverlay(
  cv::Mat &image,
  const cv::Mat &mask,
  const cv::Scalar &bgr_color,
  double alpha)
{
  if (image.empty() || mask.empty() || image.size() != mask.size() || mask.type() != CV_8UC1)
  {
    return;
  }

  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  if (clamped_alpha <= 1e-6 || cv::countNonZero(mask) == 0)
  {
    return;
  }

  cv::Mat tinted = image.clone();
  tinted.setTo(bgr_color, mask);
  cv::addWeighted(tinted, clamped_alpha, image, 1.0 - clamped_alpha, 0.0, image);
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

  std::ostringstream label_stream;
  label_stream << "Peak " << std::fixed << std::setprecision(1)
               << (peak_info.peak_height_m * 1000.0F) << " mm";
  const std::string label_text = label_stream.str();

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

void drawDepthPlaneNormalOverlay(
  cv::Mat &image,
  const DepthPlaneModel &plane,
  const CameraInfoMsg &camera_info,
  const cv::Point2f &center_px)
{
  if (image.empty() || !plane.valid)
  {
    return;
  }

  const auto plane_depth = depthPlaneDepthAtPixel(plane, center_px, image.size());
  const auto plane_normal = depthPlaneNormalInCameraFrame(
    plane,
    camera_info,
    image.size(),
    center_px,
    cv::Vec3d(0.0, 0.0, 1.0));
  if (!plane_depth.has_value() || !plane_normal.has_value())
  {
    return;
  }

  const cv::Vec3d center_camera = projectPixelToCamera(center_px, *plane_depth, camera_info);
  cv::Vec3d normal = *plane_normal;
  constexpr double kPreviewLengthM = 0.08;
  cv::Vec3d tip_camera = center_camera + (normal * kPreviewLengthM);
  if (tip_camera[2] <= 1e-6)
  {
    normal *= -1.0;
    tip_camera = center_camera + (normal * kPreviewLengthM);
  }

  const auto tip_px = projectCameraToPixel(tip_camera, camera_info);
  const cv::Point center_point(
    std::clamp(static_cast<int>(std::lround(center_px.x)), 0, std::max(0, image.cols - 1)),
    std::clamp(static_cast<int>(std::lround(center_px.y)), 0, std::max(0, image.rows - 1)));
  cv::circle(image, center_point, 8, cv::Scalar(255, 80, 255), 2, cv::LINE_AA);

  if (tip_px.has_value())
  {
    const cv::Point tip_point(
      std::clamp(static_cast<int>(std::lround(tip_px->x)), 0, std::max(0, image.cols - 1)),
      std::clamp(static_cast<int>(std::lround(tip_px->y)), 0, std::max(0, image.rows - 1)));
    if (cv::norm(tip_point - center_point) >= 4.0)
    {
      cv::arrowedLine(image, center_point, tip_point, cv::Scalar(255, 80, 255), 3, cv::LINE_AA, 0, 0.20);
    }
  }

  const std::string label_text = cv::format("Z plane n %.2f %.2f %.2f", normal[0], normal[1], normal[2]);
  const cv::Point label_origin(
    std::clamp(center_point.x + 14, 10, std::max(10, image.cols - 250)),
    std::clamp(center_point.y + 28, 24, std::max(24, image.rows - 10)));
  cv::putText(
    image,
    label_text,
    label_origin,
    cv::FONT_HERSHEY_DUPLEX,
    0.48,
    cv::Scalar(0, 0, 0),
    3,
    cv::LINE_AA);
  cv::putText(
    image,
    label_text,
    label_origin,
    cv::FONT_HERSHEY_DUPLEX,
    0.48,
    cv::Scalar(255, 170, 255),
    1,
    cv::LINE_AA);
}

}  // namespace

class ItemTeachNode : public rclcpp::Node
{
public:
  ItemTeachNode()
  : Node("item_teach")
  {
    color_topic_ = declare_parameter<std::string>("color_topic", "/robot_camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/robot_camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/robot_camera/color/camera_info");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states_robot");
	    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "bin_overlay");
	    camera_control_service_root_ = declare_parameter<std::string>(
	      "camera_control_service_root",
	      "/robot_camera");
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
	    color_exposure_min_us_ = std::clamp(color_exposure_min_us_, 1, kTeachColorExposureMaxUs);
	    color_exposure_max_us_ = kTeachColorExposureMaxUs;
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
	    use_calibration_ = declare_parameter<bool>("use_calibration", true);
    publish_static_calibration_tf_ = declare_parameter<bool>("publish_static_calibration_tf", true);
    calibration_parent_frame_ = declare_parameter<std::string>("calibration_parent_frame", "Link6");
    const std::string requested_calibration_child_frame =
      declare_parameter<std::string>("calibration_child_frame", "calibrated_camera_link");
    calibration_child_frame_ = "calibrated_camera_link";
    if (requested_calibration_child_frame != calibration_child_frame_)
    {
      RCLCPP_WARN(
        get_logger(),
        "calibration_child_frame is fixed to %s for item_teach. Ignoring requested value '%s'.",
        calibration_child_frame_.c_str(),
        requested_calibration_child_frame.c_str());
    }
    calibration_dir_ = declare_parameter<std::string>("calibration_dir", defaultCalibrationDir());
    calibration_file_ = declare_parameter<std::string>("calibration_file", "");
    auto_discover_calibration_ = declare_parameter<bool>("auto_discover_calibration", true);
    const std::string requested_item_tf_parent_frame =
      declare_parameter<std::string>("item_tf_parent_frame", "calibrated_camera_link");
    item_tf_parent_frame_ = calibration_child_frame_;
    if (requested_item_tf_parent_frame != item_tf_parent_frame_)
    {
      RCLCPP_WARN(
        get_logger(),
        "item_tf_parent_frame is fixed to %s for item_teach item poses. Ignoring requested value '%s'.",
        item_tf_parent_frame_.c_str(),
        requested_item_tf_parent_frame.c_str());
    }
    (void)this->set_parameter(rclcpp::Parameter("calibration_child_frame", calibration_child_frame_));
    (void)this->set_parameter(rclcpp::Parameter("item_tf_parent_frame", item_tf_parent_frame_));
    align_item_z_axis_to_depth_plane_ = declare_parameter<bool>(
      "align_item_z_axis_to_depth_plane",
      true);
    profiles_dir_ = declare_parameter<std::string>(
      "profiles_dir",
      dobot_common::paths::workspacePath({"teach", "item_teach"}, __FILE__).string());
    runtime_settings_path_ = declare_parameter<std::string>(
      "runtime_settings_path",
      dobot_common::paths::workspacePath(
        {"config", "item_perception", "item_teach_runtime.yaml"}, __FILE__).string());
    publish_overlay_ = declare_parameter<bool>("publish_overlay", true);
    publish_item_pose_array_ = declare_parameter<bool>("publish_item_pose_array", true);
    item_pose_array_topic_ = declare_parameter<std::string>("item_pose_array_topic", "bin_item_poses");
    bin_teach_dir_ = declare_parameter<std::string>(
      "bin_teach_dir",
      defaultBinTeachDir());
    motion_service_root_ = declare_parameter<std::string>(
      "motion_service_root",
      "/dobot_bringup_ros2/srv");
    while (!motion_service_root_.empty() && motion_service_root_.back() == '/')
    {
      motion_service_root_.pop_back();
    }
    if (motion_service_root_.empty())
    {
      motion_service_root_ = "/dobot_bringup_ros2/srv";
    }
    movj_service_name_ = motion_service_root_ + "/MovJ";
    bin_roi_move_speed_percent_ = std::clamp(
      static_cast<int>(declare_parameter<int>("bin_roi_move_speed_percent", 100)),
      1,
      100);
    display_scale_ = declare_parameter<double>("display_scale", 1.0);
    syncDetectionModeWithTeachStage();
    loadRuntimeSettingsFromFile();
    refreshBinTeachFiles();

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

      if (publish_static_calibration_tf_)
      {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishCalibrationTransform();
      }

      RCLCPP_INFO(
        get_logger(),
        "Calibration loaded from %s. Publishing %s -> %s in-node: %s",
        calibration_file_.c_str(),
        calibration_parent_frame_.c_str(),
        calibration_child_frame_.c_str(),
        publish_static_calibration_tf_ ? "enabled" : "disabled");
    }

    overlay_pub_ = create_publisher<ImageMsg>(overlay_topic_, rclcpp::QoS(5));
    if (publish_item_pose_array_)
    {
      item_pose_array_pub_ = create_publisher<PoseArrayMsg>(item_pose_array_topic_, rclcpp::QoS(10));
    }
	    movj_client_ = create_client<MovJSrv>(movj_service_name_);
	    createCameraExposureClients();
	    color_sub_ = create_subscription<ImageMsg>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ItemTeachNode::colorCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<ImageMsg>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ItemTeachNode::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<CameraInfoMsg>(
      camera_info_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&ItemTeachNode::cameraInfoCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<JointStateMsg>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ItemTeachNode::jointStateCallback, this, std::placeholders::_1));

    createUi();
    cv::setMouseCallback(kWindowName, &ItemTeachNode::onMouseThunk, this);

	    render_timer_ = create_wall_timer(
	      std::chrono::milliseconds(33),
	      std::bind(&ItemTeachNode::renderFrame, this));
	    camera_exposure_timer_ = create_wall_timer(
	      std::chrono::milliseconds(250),
	      std::bind(&ItemTeachNode::applyPendingCameraExposureSettings, this));

    RCLCPP_INFO(
      get_logger(),
      "item_teach ready. Color topic=%s depth topic=%s info topic=%s joints topic=%s overlay topic=%s item_tf_parent=%s z_axis_align=%s profiles_dir=%s",
      color_topic_.c_str(),
      depth_topic_.c_str(),
      camera_info_topic_.c_str(),
      joint_states_topic_.c_str(),
      overlay_topic_.c_str(),
      item_tf_parent_frame_.c_str(),
      align_item_z_axis_to_depth_plane_ ? "depth-plane" : "off",
      profiles_dir_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "item_teach bin ROI reuse. bin_teach_dir=%s movj_service=%s speed=%d%% files=%zu",
      bin_teach_dir_.c_str(),
      movj_service_name_.c_str(),
      bin_roi_move_speed_percent_,
      bin_roi_entries_.size());
    RCLCPP_INFO(
      get_logger(),
      "item_teach outputs. PoseArray=%s (topic=%s)",
      publish_item_pose_array_ ? "on" : "off",
	      item_pose_array_topic_.c_str());
	    RCLCPP_INFO(
	      get_logger(),
	      "item_teach camera exposure controls. service_root=%s color=%s depth=%s",
	      camera_control_service_root_.c_str(),
	      exposureModeText(color_exposure_us_).c_str(),
	      exposureModeText(depth_exposure_us_).c_str());
	  }

  ~ItemTeachNode() override
  {
    saveRuntimeSettingsToFile(true);
    cv::destroyWindow(kWindowName);
  }

private:
  enum class ViewMode
  {
    kBinarized = 0,
    kRgb,
    kDepth,
  };

  enum class TeachStage
  {
    kRoi = 0,
    kColorMask,
    kDepthNormalize,
    kFlatLocate,
    kPosePerception,
  };

  struct UiButton
  {
    std::string label;
    cv::Rect rect;
    bool *state {nullptr};
  };

  struct UiSlider
  {
    std::string label;
    cv::Rect track_rect;
    int *value {nullptr};
    int min_value {0};
    int max_value {255};
  };

  struct BinTeachRoiEntry
  {
    std::string label;
    std::string path;
    std::string bin_name;
    int image_width {0};
    int image_height {0};
    std::vector<cv::Point2f> roi_points;
    std::vector<cv::Point2f> roi_points_normalized;
    bool has_arm_pose {false};
    double x {0.0};
    double y {0.0};
    double z {0.0};
    double rx {0.0};
    double ry {0.0};
    double rz {0.0};
    bool has_depth_plane {false};
    DepthPlaneModel depth_plane;
    std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds;
    std::vector<cv::Point2f> depth_plane_roi_points;
  };

  void layoutSlidersForCurrentStage()
  {
    if (sliders_.empty())
    {
      return;
    }

    const int base_x = sliders_.front().track_rect.x;
    const int base_w = sliders_.front().track_rect.width;
    const int base_h = sliders_.front().track_rect.height;
    const int start_y = 332;
    const int gap = 52;
    int next_y = start_y;

    for (auto &slider : sliders_)
    {
      slider.track_rect = cv::Rect(base_x, next_y, base_w, base_h);
      if (isSliderVisible(slider))
      {
        next_y += gap;
      }
    }
  }

  void createUi()
  {
    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(
      kWindowName,
      kLeftPanelWidth + kPreviewCanvasWidth,
      kVideoTopBarHeight + kPreviewCanvasHeight);

    const int top_y = 12;
    const int top_h = 36;
    const int top_gap = 8;
    int button_x = kLeftPanelWidth + 16;
    buttons_.clear();
    buttons_.push_back({"Live", cv::Rect(button_x, top_y, 112, top_h), &live_view_enabled_});
    button_x += 112 + top_gap;
    buttons_.push_back({"View", cv::Rect(button_x, top_y, 142, top_h), nullptr});
    button_x += 142 + top_gap;
    buttons_.push_back({"Overlay", cv::Rect(button_x, top_y, 128, top_h), &overlay_enabled_});
    button_x += 128 + top_gap;
    buttons_.push_back({"Focus", cv::Rect(button_x, top_y, 140, top_h), &focus_black_mask_});

    sliders_.clear();
    name_box_rect_ = cv::Rect(20, 94, kLeftPanelWidth - 40, 40);
    delete_bin_roi_button_rect_ = cv::Rect(kLeftPanelWidth - 116, 158, 96, 34);
    bin_roi_dropdown_rect_ = cv::Rect(20, 158, delete_bin_roi_button_rect_.x - 28, 34);
    back_button_rect_ = cv::Rect(20, 208, 84, 40);
    save_button_rect_ = cv::Rect(112, 208, 96, 40);
    const int slot_box_y = save_button_rect_.y;
    const int slot_box_size = 40;
    const int slot_gap = 8;
    const int slot_start_x = save_button_rect_.x + save_button_rect_.width + 14;
    for (int i = 0; i < kPoseReferenceSlotCount; ++i)
    {
      pose_reference_slot_rects_[static_cast<std::size_t>(i)] =
        cv::Rect(slot_start_x + i * (slot_box_size + slot_gap), slot_box_y, slot_box_size, slot_box_size);
    }
    int y = 332;
    const int track_x = 20;
	    const int track_w = kLeftPanelWidth - 40;
	    const int track_h = 12;
	    const int gap = 52;
	    sliders_.push_back(
	      {kColorExposureTrackbar, cv::Rect(track_x, y, track_w, track_h), &color_exposure_us_, 0, kTeachColorExposureMaxUs}); y += gap;
	    sliders_.push_back({kRedTrackbar, cv::Rect(track_x, y, track_w, track_h), &red_threshold_, 0, 255}); y += gap;
	    sliders_.push_back({kGreenTrackbar, cv::Rect(track_x, y, track_w, track_h), &green_threshold_, 0, 255}); y += gap;
	    sliders_.push_back({kBlueTrackbar, cv::Rect(track_x, y, track_w, track_h), &blue_threshold_, 0, 255}); y += gap;
    sliders_.push_back(
      {kRgbHoleFillTrackbar, cv::Rect(track_x, y, track_w, track_h), &rgb_hole_fill_sensitivity_, kRgbHoleFillMin, kRgbHoleFillMax}); y += gap;
    sliders_.push_back(
      {kRgbDilateTrackbar, cv::Rect(track_x, y, track_w, track_h), &rgb_mask_dilate_px_, kRgbDilateMinPx, kRgbDilateMaxPx}); y += gap;
	    sliders_.push_back(
	      {kDepthNullFillTrackbar, cv::Rect(track_x, y, track_w, track_h), &depth_null_fill_sensitivity_, kDepthFillSensitivityMin, kDepthFillSensitivityMax}); y += gap;
	    sliders_.push_back(
	      {kDepthWindowTrackbar, cv::Rect(track_x, y, track_w, track_h), &depth_window_mm_, kDepthWindowMinMm, kDepthWindowMaxMm}); y += gap;
	    sliders_.push_back(
	      {kDepthHoleFillTrackbar, cv::Rect(track_x, y, track_w, track_h), &depth_hole_fill_sensitivity_, kDepthFillSensitivityMin, kDepthFillSensitivityMax}); y += gap;
    sliders_.push_back(
      {kDepthTrimTrackbar, cv::Rect(track_x, y, track_w, track_h), &depth_trim_px_, kDepthTrimMinPx, kDepthTrimMaxPx}); y += gap;
    sliders_.push_back(
      {kAdaptiveDepthTrimFactorTrackbar, cv::Rect(track_x, y, track_w, track_h), &adaptive_depth_trim_max_factor_tenths_, kAdaptiveDepthTrimFactorMinTenths, kAdaptiveDepthTrimFactorMaxTenths}); y += gap;
    sliders_.push_back(
      {kAdaptiveDepthTrimHeightTrackbar, cv::Rect(track_x, y, track_w, track_h), &adaptive_depth_trim_max_height_mm_, kAdaptiveDepthTrimHeightMinMm, kAdaptiveDepthTrimHeightMaxMm}); y += gap;
    sliders_.push_back(
      {kBlobToleranceTrackbar, cv::Rect(track_x, y, track_w, track_h), &blob_tolerance_percent_, kBlobToleranceMinPercent, kBlobToleranceMaxPercent});
    layoutSlidersForCurrentStage();
  }

  void advanceViewMode()
  {
    switch (view_mode_)
    {
      case ViewMode::kBinarized:
        view_mode_ = ViewMode::kRgb;
        break;
      case ViewMode::kRgb:
        view_mode_ = ViewMode::kDepth;
        break;
      case ViewMode::kDepth:
        view_mode_ = ViewMode::kBinarized;
        break;
    }
    markRuntimeSettingsDirty();
  }

  std::string currentViewLabel() const
  {
    switch (view_mode_)
    {
      case ViewMode::kRgb:
        return "RGB";
      case ViewMode::kDepth:
        return "Depth";
      case ViewMode::kBinarized:
      default:
        return "Binarized";
    }
  }

  std::string teachStageLabel() const
  {
    switch (teach_stage_)
    {
      case TeachStage::kRoi:
        return "ROI";
      case TeachStage::kColorMask:
        return "RGB Mask";
      case TeachStage::kDepthNormalize:
        return "Depth Normalize";
      case TeachStage::kFlatLocate:
        return "Depth Filter";
      case TeachStage::kPosePerception:
      default:
        return "Pose";
    }
  }

  std::string saveActionButtonLabel() const
  {
    return teach_stage_ == TeachStage::kPosePerception ? "Save Item" : "Next";
  }

  bool isFinalTeachStage() const
  {
    return teach_stage_ == TeachStage::kPosePerception;
  }

  bool canGoBack() const
  {
    return teach_stage_ != TeachStage::kRoi;
  }

  void setStatusMessage(const std::string &message, double duration_sec = 1.5)
  {
    save_status_message_ = message;
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(duration_sec);
  }

  bool actionButtonReady() const
  {
    switch (teach_stage_)
    {
      case TeachStage::kRoi:
        return hasValidRoiPoints(roi_points_);
      case TeachStage::kColorMask:
        return hasValidRoiPoints(roi_points_);
      case TeachStage::kDepthNormalize:
        return hasDepthPlaneReference();
      case TeachStage::kFlatLocate:
        return hasValidRoiPoints(roi_points_) && hasDepthPlaneReference();
      case TeachStage::kPosePerception:
      default:
        return
          hasValidRoiPoints(roi_points_) &&
          hasDepthPlaneReference() &&
          filledPoseReferenceSlotCount() > 0;
    }
  }

  void syncDetectionModeWithTeachStage()
  {
    detection_use_depth_ = teach_stage_ >= TeachStage::kDepthNormalize;
  }

  void setTeachStage(TeachStage stage, bool mark_dirty = true)
  {
    teach_stage_ = stage;
    syncDetectionModeWithTeachStage();
    layoutSlidersForCurrentStage();
    if (teach_stage_ != TeachStage::kPosePerception)
    {
      pose_blob_seed_point_px_.reset();
      pose_blob_reference_.reset();
      pose_blob_reference_clicks_px_.clear();
      clearAllPoseReferenceSlots();
      pose_stage_status_.clear();
    }
    if (mark_dirty)
    {
      markRuntimeSettingsDirty();
    }
  }

  bool awaitingDepthNormalizePoints() const
  {
    return teach_stage_ == TeachStage::kDepthNormalize && !hasDepthPlaneReference();
  }

  int depthNormalizePointCount() const
  {
    if (hasDepthPlaneReference())
    {
      return 4;
    }
    return std::clamp(static_cast<int>(pending_depth_plane_points_.size()), 0, 4);
  }

  std::string depthNormalizeProgressText() const
  {
    if (teach_stage_ != TeachStage::kDepthNormalize)
    {
      return "";
    }
    return "Depth points " + std::to_string(depthNormalizePointCount()) + "/4";
  }

	  bool isRgbThresholdSliderLabel(const std::string &label) const
	  {
	    return
	      label == kColorExposureTrackbar ||
	      label == kRedTrackbar ||
	      label == kGreenTrackbar ||
	      label == kBlueTrackbar ||
      label == kRgbHoleFillTrackbar ||
      label == kRgbDilateTrackbar;
  }

	  bool isDepthThresholdSliderLabel(const std::string &label) const
	  {
	    return
	      label == kDepthNullFillTrackbar ||
	      label == kDepthWindowTrackbar ||
	      label == kDepthHoleFillTrackbar ||
	      label == kDepthTrimTrackbar ||
	      label == kAdaptiveDepthTrimFactorTrackbar ||
      label == kAdaptiveDepthTrimHeightTrackbar;
  }

  bool isPoseThresholdSliderLabel(const std::string &label) const
  {
    return label == kBlobToleranceTrackbar;
  }

  bool isSliderVisible(const UiSlider &slider) const
  {
    if (teach_stage_ == TeachStage::kColorMask)
    {
      return isRgbThresholdSliderLabel(slider.label);
    }
    if (
      teach_stage_ == TeachStage::kDepthNormalize ||
      teach_stage_ == TeachStage::kFlatLocate)
    {
      return isDepthThresholdSliderLabel(slider.label);
    }
    if (teach_stage_ == TeachStage::kPosePerception)
    {
      return isDepthThresholdSliderLabel(slider.label) || isPoseThresholdSliderLabel(slider.label);
    }
    return false;
  }

	  bool isSliderEnabled(const UiSlider &slider) const
	  {
	    return isSliderVisible(slider);
	  }

	  bool isExposureSliderLabel(const std::string &label) const
	  {
	    return label == kColorExposureTrackbar;
	  }

  bool hasDepthPlaneReference() const
  {
    return depth_plane_model_.valid && depth_plane_roi_bounds_.has_value();
  }

  void clearDepthPlaneReference(bool mark_dirty = true)
  {
    depth_plane_model_ = DepthPlaneModel{};
    depth_plane_roi_bounds_.reset();
    depth_plane_roi_points_.clear();
    pending_depth_plane_points_.clear();
    depth_plane_from_bin_teach_ = false;
    clearAllPoseReferenceSlots();
    pose_stage_status_.clear();
    if (mark_dirty)
    {
      markRuntimeSettingsDirty();
    }
  }

  bool getDepthFrameForPlaneFit(cv::Mat &depth_frame_m)
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!live_view_enabled_ && !frozen_depth_.empty())
    {
      depth_frame_m = frozen_depth_.clone();
    }
    else
    {
      depth_frame_m = latest_depth_.clone();
    }
    return !depth_frame_m.empty() && depth_frame_m.type() == CV_32FC1;
  }

  std::optional<cv::Point2f> windowPointToImagePoint(const cv::Point &window_point)
  {
    cv::Size source_size;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!latest_frame_.empty())
      {
        source_size = latest_frame_.size();
      }
      else if (!frozen_frame_.empty())
      {
        source_size = frozen_frame_.size();
      }
    }

    if (source_size.width <= 0 || source_size.height <= 0)
    {
      return std::nullopt;
    }

    const cv::Rect preview_rect = previewImageRectForSource(source_size);
    const cv::Rect image_rect(
      kLeftPanelWidth + preview_rect.x,
      kVideoTopBarHeight + preview_rect.y,
      preview_rect.width,
      preview_rect.height);
    if (!image_rect.contains(window_point))
    {
      return std::nullopt;
    }

    const float image_x = static_cast<float>(
      static_cast<double>(window_point.x - image_rect.x) *
      static_cast<double>(source_size.width) /
      static_cast<double>(image_rect.width));
    const float image_y = static_cast<float>(
      static_cast<double>(window_point.y - image_rect.y) *
      static_cast<double>(source_size.height) /
      static_cast<double>(image_rect.height));
    return cv::Point2f(
      std::clamp(image_x, 0.0F, static_cast<float>(source_size.width - 1)),
      std::clamp(image_y, 0.0F, static_cast<float>(source_size.height - 1)));
  }

  cv::Size previewCanvasSizeForSource(const cv::Size &source_size) const
  {
    const double requested_scale = display_scale_ > 0.0 ? display_scale_ : 1.0;
    const int preview_width = std::max(
      kPreviewCanvasWidth,
      static_cast<int>(std::round(static_cast<double>(kPreviewCanvasWidth) * requested_scale)));
    if (source_size.width <= 0 || source_size.height <= 0)
    {
      return cv::Size(preview_width, kPreviewCanvasHeight);
    }

    const int preview_height = std::max(
      1,
      static_cast<int>(std::round(
        static_cast<double>(preview_width) *
        static_cast<double>(source_size.height) /
        static_cast<double>(source_size.width))));
    return cv::Size(preview_width, preview_height);
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

  int minimumLeftPanelHeight() const
  {
    const int slider_card_y = std::max(
      200,
      sliders_.empty() ? 200 : (sliders_.front().track_rect.y - 54));
    int last_visible_slider_bottom = slider_card_y + 180;
    for (const auto &slider : sliders_)
    {
      if (!isSliderVisible(slider))
      {
        continue;
      }
      last_visible_slider_bottom = std::max(last_visible_slider_bottom, slider.track_rect.y + 34);
    }
    return last_visible_slider_bottom + 12;
  }

  static void onMouseThunk(int event, int x, int y, int flags, void *userdata)
  {
    static_cast<ItemTeachNode *>(userdata)->onMouse(event, x, y, flags);
  }

  void updateSliderFromPoint(int index, const cv::Point &raw_point)
  {
    if (index < 0 || index >= static_cast<int>(sliders_.size()))
    {
      return;
    }
    auto &slider = sliders_[index];
    if (slider.value == nullptr)
    {
      return;
    }
    if (!isSliderEnabled(slider))
    {
      return;
    }
    const int clamped_x = std::clamp(raw_point.x, slider.track_rect.x, slider.track_rect.x + slider.track_rect.width);
    const double t = static_cast<double>(clamped_x - slider.track_rect.x) / std::max(1, slider.track_rect.width);
    const int new_value = static_cast<int>(std::round(slider.min_value + t * (slider.max_value - slider.min_value)));
	    if (*slider.value != new_value)
	    {
	      *slider.value = new_value;
	      if (isExposureSliderLabel(slider.label))
	      {
	        markCameraExposureDirty();
	      }
	      markRuntimeSettingsDirty();
	    }
	  }

  std::string sliderValueText(const UiSlider &slider, int value) const
  {
    if (slider.label == kBlobToleranceTrackbar)
    {
      return "+/-" + std::to_string(value) + "%";
    }
    if (slider.label == kAdaptiveDepthTrimFactorTrackbar)
    {
      return std::to_string(value) + " px";
    }
	    if (slider.label == kAdaptiveDepthTrimHeightTrackbar)
	    {
	      return std::to_string(value) + " mm";
	    }
	    if (isExposureSliderLabel(slider.label))
	    {
	      return value <= 0 ? "auto" : std::to_string(value) + " us";
	    }
	    return std::to_string(value);
	  }

  int sliderValueTextOffsetX(const UiSlider &slider) const
  {
    if (slider.label == kBlobToleranceTrackbar)
    {
      return 74;
    }
    if (slider.label == kAdaptiveDepthTrimFactorTrackbar)
    {
      return 60;
    }
	    if (slider.label == kAdaptiveDepthTrimHeightTrackbar)
	    {
	      return 68;
	    }
	    if (isExposureSliderLabel(slider.label))
	    {
	      return 54;
	    }
	    return 36;
	  }

  void rebuildMergedRoiPolygon()
  {
    roi_points_ = mergeRoiRegionsIntoPolygon(roi_regions_);
    clearDepthPlaneReference(false);
    active_bin_name_.clear();
    active_bin_teach_path_.clear();
    pose_estimate_.reset();
    pose_blob_seed_point_px_.reset();
    pose_blob_reference_.reset();
    pose_blob_reference_clicks_px_.clear();
    clearAllPoseReferenceSlots();
    pose_stage_status_.clear();
    markRuntimeSettingsDirty();
  }

  void resetTaughtMetrics()
  {
    // Mask-only item_teach flow: edge/area metrics are not tracked.
  }

  void persistActivePoseReferenceSlot()
  {
    if (
      active_pose_reference_slot_index_ < 0 ||
      active_pose_reference_slot_index_ >= kPoseReferenceSlotCount)
    {
      return;
    }
    auto &slot = pose_reference_slots_[static_cast<std::size_t>(active_pose_reference_slot_index_)];
    slot.reference = pose_blob_reference_;
    slot.clicks_px = pose_blob_reference_clicks_px_;
  }

  void loadActivePoseReferenceSlot()
  {
    if (
      active_pose_reference_slot_index_ < 0 ||
      active_pose_reference_slot_index_ >= kPoseReferenceSlotCount)
    {
      pose_blob_reference_.reset();
      pose_blob_reference_clicks_px_.clear();
      return;
    }
    const auto &slot = pose_reference_slots_[static_cast<std::size_t>(active_pose_reference_slot_index_)];
    pose_blob_reference_ = slot.reference;
    pose_blob_reference_clicks_px_ = slot.clicks_px;
  }

  void clearPoseReferenceSlot(int slot_index)
  {
    if (slot_index < 0 || slot_index >= kPoseReferenceSlotCount)
    {
      return;
    }
    auto &slot = pose_reference_slots_[static_cast<std::size_t>(slot_index)];
    slot.reference.reset();
    slot.clicks_px.clear();
    if (slot_index == active_pose_reference_slot_index_)
    {
      pose_blob_reference_.reset();
      pose_blob_reference_clicks_px_.clear();
      pose_blob_seed_point_px_.reset();
      pose_estimate_.reset();
    }
  }

  void clearAllPoseReferenceSlots()
  {
    for (auto &slot : pose_reference_slots_)
    {
      slot.reference.reset();
      slot.clicks_px.clear();
    }
    active_pose_reference_slot_index_ = 0;
    pose_blob_reference_.reset();
    pose_blob_reference_clicks_px_.clear();
    pose_blob_seed_point_px_.reset();
    pose_estimate_.reset();
  }

  int filledPoseReferenceSlotCount() const
  {
    int count = 0;
    for (std::size_t i = 0; i < pose_reference_slots_.size(); ++i)
    {
      const bool active_slot = static_cast<int>(i) == active_pose_reference_slot_index_;
      const bool filled = active_slot ? pose_blob_reference_.has_value() : pose_reference_slots_[i].reference.has_value();
      if (filled)
      {
        ++count;
      }
    }
    return count;
  }

  std::optional<PoseBlobReference2D> poseReferenceForSlot(int slot_index) const
  {
    if (slot_index < 0 || slot_index >= kPoseReferenceSlotCount)
    {
      return std::nullopt;
    }
    if (slot_index == active_pose_reference_slot_index_)
    {
      return pose_blob_reference_;
    }
    return pose_reference_slots_[static_cast<std::size_t>(slot_index)].reference;
  }

  std::optional<int> firstFilledPoseReferenceSlotIndex() const
  {
    for (int i = 0; i < kPoseReferenceSlotCount; ++i)
    {
      if (poseReferenceForSlot(i).has_value())
      {
        return i;
      }
    }
    return std::nullopt;
  }

  void selectPoseReferenceSlot(int slot_index)
  {
    if (slot_index < 0 || slot_index >= kPoseReferenceSlotCount)
    {
      return;
    }
    persistActivePoseReferenceSlot();
    active_pose_reference_slot_index_ = slot_index;
    clearPoseReferenceSlot(slot_index);
    loadActivePoseReferenceSlot();
    pose_stage_status_ = "Slot " + std::to_string(slot_index + 1) + " selected. Click blob 1/2";
    save_status_message_ = "Pose slot " + std::to_string(slot_index + 1) + " reset";
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.2);
    markRuntimeSettingsDirty();
  }

  static std::string defaultCalibrationDir()
  {
    return dobot_common::paths::workspacePath({"calibration"}, __FILE__).string();
  }

  static std::string defaultBinTeachDir()
  {
    return dobot_common::paths::workspacePath({"teach", "bin_teach"}, __FILE__).string();
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

  static bool readFiniteYamlDouble(const YAML::Node &node, double &value)
  {
    if (!node || !node.IsScalar())
    {
      return false;
    }
    try
    {
      value = node.as<double>();
    }
    catch (const std::exception &)
    {
      return false;
    }
    return std::isfinite(value);
  }

  static bool readPositiveYamlInt(const YAML::Node &node, int &value)
  {
    if (!node || !node.IsScalar())
    {
      return false;
    }
    try
    {
      value = node.as<int>();
    }
    catch (const std::exception &)
    {
      return false;
    }
    return value > 0;
  }

  static bool readYamlBool(const YAML::Node &node, bool &value)
  {
    if (!node || !node.IsScalar())
    {
      return false;
    }
    try
    {
      value = node.as<bool>();
    }
    catch (const std::exception &)
    {
      return false;
    }
    return true;
  }

  bool parseBinTeachRoiFile(
    const std::filesystem::path &path,
    BinTeachRoiEntry &entry,
    std::string &reason) const
  {
    entry = BinTeachRoiEntry{};
    entry.path = path.string();

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(path.string());
    }
    catch (const std::exception &ex)
    {
      reason = std::string("read failed: ") + ex.what();
      return false;
    }

    const YAML::Node bin = root["bin_teach"];
    if (!bin || !bin.IsMap())
    {
      reason = "missing bin_teach map";
      return false;
    }

    try
    {
      entry.bin_name = bin["bin_name"] ? bin["bin_name"].as<std::string>() : "";
    }
    catch (const std::exception &)
    {
      entry.bin_name.clear();
    }

    const std::string file_label = path.stem().string();
    if (!entry.bin_name.empty() && entry.bin_name != file_label)
    {
      entry.label = entry.bin_name + "/" + file_label;
    }
    else
    {
      entry.label = file_label;
    }

    const YAML::Node image = bin["image"];
    if (!image || !image.IsMap())
    {
      reason = "missing image metadata";
      return false;
    }
    if (!readPositiveYamlInt(image["width"], entry.image_width) ||
        !readPositiveYamlInt(image["height"], entry.image_height))
    {
      reason = "missing valid image width/height";
      return false;
    }
    std::string coordinate_frame;
    try
    {
      coordinate_frame = image["coordinate_frame"] ? image["coordinate_frame"].as<std::string>() : "";
    }
    catch (const std::exception &)
    {
      coordinate_frame.clear();
    }
    if (coordinate_frame != "color_image_pixels")
    {
      reason = "image coordinate_frame is not color_image_pixels";
      return false;
    }

    const YAML::Node roi = bin["roi_points"];
    if (!roi || !roi.IsSequence() || roi.size() != 8)
    {
      reason = "roi_points must contain exactly 8 numbers";
      return false;
    }

    entry.roi_points.clear();
    entry.roi_points.reserve(4);
    for (std::size_t i = 0; i < roi.size(); i += 2)
    {
      double x = 0.0;
      double y = 0.0;
      if (!readFiniteYamlDouble(roi[i], x) || !readFiniteYamlDouble(roi[i + 1], y))
      {
        reason = "roi_points contains a non-numeric value";
        return false;
      }
      entry.roi_points.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }

    const YAML::Node roi_normalized = bin["roi_points_normalized"];
    if (roi_normalized && roi_normalized.IsSequence() && roi_normalized.size() == 8)
    {
      entry.roi_points_normalized.clear();
      entry.roi_points_normalized.reserve(4);
      bool normalized_valid = true;
      for (std::size_t i = 0; i < roi_normalized.size(); i += 2)
      {
        double x_norm = 0.0;
        double y_norm = 0.0;
        if (!readFiniteYamlDouble(roi_normalized[i], x_norm) ||
            !readFiniteYamlDouble(roi_normalized[i + 1], y_norm))
        {
          normalized_valid = false;
          break;
        }
        entry.roi_points_normalized.emplace_back(
          static_cast<float>(x_norm),
          static_cast<float>(y_norm));
      }
      if (!normalized_valid)
      {
        entry.roi_points_normalized.clear();
      }
    }

    const YAML::Node depth_plane_map = bin["depth_plane"];
    YAML::Node depth_plane_enabled_node = bin["depth_plane_enabled"];
    YAML::Node depth_plane_a_node = bin["depth_plane_a"];
    YAML::Node depth_plane_b_node = bin["depth_plane_b"];
    YAML::Node depth_plane_c_node = bin["depth_plane_c"];
    YAML::Node depth_plane_reference_node = bin["depth_plane_reference_depth_m"];
    YAML::Node depth_plane_roi_node = bin["depth_plane_roi"];
    if (depth_plane_map && depth_plane_map.IsMap())
    {
      if (!depth_plane_enabled_node) { depth_plane_enabled_node = depth_plane_map["enabled"]; }
      if (!depth_plane_a_node) { depth_plane_a_node = depth_plane_map["a"]; }
      if (!depth_plane_b_node) { depth_plane_b_node = depth_plane_map["b"]; }
      if (!depth_plane_c_node) { depth_plane_c_node = depth_plane_map["c"]; }
      if (!depth_plane_reference_node) { depth_plane_reference_node = depth_plane_map["reference_depth_m"]; }
      if (!depth_plane_roi_node) { depth_plane_roi_node = depth_plane_map["roi"]; }
    }

    bool depth_plane_enabled = false;
    if (readYamlBool(depth_plane_enabled_node, depth_plane_enabled) && depth_plane_enabled)
    {
      DepthPlaneModel loaded_plane;
      double a = 0.0;
      double b = 0.0;
      double c = 0.0;
      double reference_depth_m = 0.0;
      if (readFiniteYamlDouble(depth_plane_a_node, a) &&
          readFiniteYamlDouble(depth_plane_b_node, b) &&
          readFiniteYamlDouble(depth_plane_c_node, c) &&
          readFiniteYamlDouble(depth_plane_reference_node, reference_depth_m) &&
          reference_depth_m > 0.0)
      {
        std::optional<AxisAlignedRoiBounds> plane_bounds;
        if (depth_plane_roi_node && depth_plane_roi_node.IsSequence() && depth_plane_roi_node.size() >= 4)
        {
          try
          {
            AxisAlignedRoiBounds parsed_bounds{
              depth_plane_roi_node[0].as<int>(),
              depth_plane_roi_node[1].as<int>(),
              depth_plane_roi_node[2].as<int>(),
              depth_plane_roi_node[3].as<int>(),
            };
            if (isValidRoiBounds(parsed_bounds))
            {
              plane_bounds = parsed_bounds;
            }
          }
          catch (const std::exception &)
          {
            plane_bounds.reset();
          }
        }
        if (!plane_bounds.has_value())
        {
          plane_bounds = roiBoundsFromSelection(entry.roi_points);
        }

        if (plane_bounds.has_value())
        {
          loaded_plane.valid = true;
          loaded_plane.a = a;
          loaded_plane.b = b;
          loaded_plane.c = c;
          loaded_plane.reference_depth_m = reference_depth_m;
          entry.has_depth_plane = true;
          entry.depth_plane = loaded_plane;
          entry.depth_plane_roi_bounds = *plane_bounds;
          entry.depth_plane_roi_points = roiPointsFromBounds(*plane_bounds);
        }
      }
    }

    const YAML::Node arm_pose = bin["arm_pose_at_save"];
    bool pose_valid = false;
    if (arm_pose && arm_pose.IsMap() && arm_pose["valid"])
    {
      try
      {
        pose_valid = arm_pose["valid"].as<bool>();
      }
      catch (const std::exception &)
      {
        pose_valid = false;
      }
    }
    if (pose_valid)
    {
      const YAML::Node tcp = arm_pose["tcp"];
      if (tcp && tcp.IsMap() &&
          readFiniteYamlDouble(tcp["x"], entry.x) &&
          readFiniteYamlDouble(tcp["y"], entry.y) &&
          readFiniteYamlDouble(tcp["z"], entry.z) &&
          readFiniteYamlDouble(tcp["rx"], entry.rx) &&
          readFiniteYamlDouble(tcp["ry"], entry.ry) &&
          readFiniteYamlDouble(tcp["rz"], entry.rz))
      {
        entry.has_arm_pose = true;
      }
    }

    return true;
  }

  int visibleBinRoiOptionCount() const
  {
    return std::min(6, static_cast<int>(bin_roi_entries_.size()));
  }

  cv::Rect binRoiOptionRect(int visible_index) const
  {
    constexpr int kRowHeight = 32;
    return cv::Rect(
      bin_roi_dropdown_rect_.x,
      bin_roi_dropdown_rect_.y + bin_roi_dropdown_rect_.height + 2 + visible_index * kRowHeight,
      bin_roi_dropdown_rect_.width,
      kRowHeight);
  }

  std::string binRoiDropdownText() const
  {
    if (pending_bin_roi_index_ >= 0 &&
        pending_bin_roi_index_ < static_cast<int>(bin_roi_entries_.size()))
    {
      return "Load Bin Teach: " + bin_roi_entries_[static_cast<std::size_t>(pending_bin_roi_index_)].label + " (pending)";
    }
    if (selected_bin_roi_index_ >= 0 &&
        selected_bin_roi_index_ < static_cast<int>(bin_roi_entries_.size()))
    {
      return "Load Bin Teach: " + bin_roi_entries_[static_cast<std::size_t>(selected_bin_roi_index_)].label;
    }
    if (bin_roi_entries_.empty())
    {
      if (bin_teach_yaml_file_count_ > 0)
      {
        return "Load Bin Teach: no compatible files";
      }
      return "Load Bin Teach: no files";
    }
    return "Load Bin Teach: choose file";
  }

  void refreshBinTeachFiles()
  {
    const std::string selected_path =
      selected_bin_roi_index_ >= 0 &&
      selected_bin_roi_index_ < static_cast<int>(bin_roi_entries_.size())
      ? bin_roi_entries_[static_cast<std::size_t>(selected_bin_roi_index_)].path
      : "";
    const std::string pending_path =
      pending_bin_roi_index_ >= 0 &&
      pending_bin_roi_index_ < static_cast<int>(bin_roi_entries_.size())
      ? bin_roi_entries_[static_cast<std::size_t>(pending_bin_roi_index_)].path
      : "";

    bin_roi_entries_.clear();
    selected_bin_roi_index_ = -1;
    pending_bin_roi_index_ = -1;
    bin_teach_yaml_file_count_ = 0;
    bin_teach_skipped_file_count_ = 0;

    const std::filesystem::path base = resolvePath(bin_teach_dir_);
    std::error_code fs_error;
    if (!std::filesystem::exists(base, fs_error) || !std::filesystem::is_directory(base, fs_error))
    {
      return;
    }

    std::vector<std::filesystem::path> paths;
    for (const auto &entry : std::filesystem::directory_iterator(base, fs_error))
    {
      if (fs_error)
      {
        break;
      }
      if (!entry.is_regular_file(fs_error))
      {
        continue;
      }
      const auto &path = entry.path();
      if (path.extension() == ".yaml")
      {
        paths.push_back(path);
      }
    }
    bin_teach_yaml_file_count_ = static_cast<int>(paths.size());

    std::sort(
      paths.begin(),
      paths.end(),
      [](const std::filesystem::path &a, const std::filesystem::path &b)
      {
        std::error_code error_a;
        std::error_code error_b;
        const auto time_a = std::filesystem::last_write_time(a, error_a);
        const auto time_b = std::filesystem::last_write_time(b, error_b);
        if (!error_a && !error_b && time_a != time_b)
        {
          return time_a > time_b;
        }
        return a.filename().string() < b.filename().string();
      });

    for (const auto &path : paths)
    {
      BinTeachRoiEntry parsed;
      std::string reason;
      if (!parseBinTeachRoiFile(path, parsed, reason))
      {
        ++bin_teach_skipped_file_count_;
        RCLCPP_WARN(
          get_logger(),
          "Skipping bin_teach file %s: %s",
          path.c_str(),
          reason.c_str());
        continue;
      }
      bin_roi_entries_.push_back(parsed);
    }

    for (int i = 0; i < static_cast<int>(bin_roi_entries_.size()); ++i)
    {
      const auto &entry = bin_roi_entries_[static_cast<std::size_t>(i)];
      if (!selected_path.empty() && entry.path == selected_path)
      {
        selected_bin_roi_index_ = i;
      }
      if (!pending_path.empty() && entry.path == pending_path)
      {
        pending_bin_roi_index_ = i;
      }
    }
  }

  void setSaveStatus(const std::string &message, double seconds = 1.8)
  {
    save_status_message_ = message;
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(seconds);
  }

  std::string loadBinTeachEmptyStatus() const
  {
    if (bin_teach_yaml_file_count_ <= 0)
    {
      return "No Bin Teach files found";
    }
    if (bin_teach_skipped_file_count_ > 0)
    {
      return "No compatible Bin Teach files; save bin_teach again";
    }
    return "No compatible Bin Teach files found";
  }

  std::optional<cv::Size> currentColorFrameSize()
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!live_view_enabled_ && !frozen_frame_.empty())
    {
      return frozen_frame_.size();
    }
    if (!latest_frame_.empty())
    {
      return latest_frame_.size();
    }
    if (!frozen_frame_.empty())
    {
      return frozen_frame_.size();
    }
    return std::nullopt;
  }

  std::string sendBinRoiMove(const BinTeachRoiEntry &entry)
  {
    if (!entry.has_arm_pose)
    {
      return "Bin Teach loaded; robot move unavailable (missing pose)";
    }
    if (!movj_client_)
    {
      return "Bin Teach loaded; robot move unavailable (MovJ client)";
    }
    if (!movj_client_->service_is_ready())
    {
      return "Bin Teach loaded; robot move unavailable (MovJ service)";
    }

    const int speed = std::clamp(bin_roi_move_speed_percent_, 1, 100);
    auto request = std::make_shared<MovJSrv::Request>();
    request->mode = false;
    request->a = entry.x;
    request->b = entry.y;
    request->c = entry.z;
    request->d = entry.rx;
    request->e = entry.ry;
    request->f = entry.rz;
    request->param_value = {"v=" + std::to_string(speed) + ",a=" + std::to_string(speed)};

    RCLCPP_INFO(
      get_logger(),
      "Load Bin Teach MovJ -> %s pose x=%.3f y=%.3f z=%.3f rx=%.3f ry=%.3f rz=%.3f speed=%d",
      movj_service_name_.c_str(),
      entry.x,
      entry.y,
      entry.z,
      entry.rx,
      entry.ry,
      entry.rz,
      speed);

    const std::string label = entry.label;
    movj_client_->async_send_request(
      request,
      [this, label](rclcpp::Client<MovJSrv>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();
          const bool ok = response && response->res != -1;
          if (ok)
          {
            setSaveStatus("Bin Teach loaded; MovJ accepted: " + label, 2.0);
            RCLCPP_INFO(
              get_logger(),
              "Load Bin Teach MovJ accepted for %s (res=%d, robot_return=%s)",
              label.c_str(),
              response->res,
              response->robot_return.c_str());
          }
          else
          {
            setSaveStatus("Bin Teach loaded; robot move failed", 2.0);
            RCLCPP_WARN(
              get_logger(),
              "Load Bin Teach MovJ failed for %s (res=%d, robot_return=%s)",
              label.c_str(),
              response ? response->res : -999,
              response ? response->robot_return.c_str() : "null");
          }
        }
        catch (const std::exception &ex)
        {
          setSaveStatus("Bin Teach loaded; robot move error", 2.0);
          RCLCPP_WARN(get_logger(), "Load Bin Teach MovJ call failed: %s", ex.what());
        }
      });

    return "Bin Teach loaded; robot moving to saved pose";
  }

  bool applyBinTeachRoi(int index, const cv::Size &current_size)
  {
    if (index < 0 || index >= static_cast<int>(bin_roi_entries_.size()))
    {
      setSaveStatus("Load Bin Teach selection is no longer available", 1.8);
      return false;
    }

    const auto &entry = bin_roi_entries_[static_cast<std::size_t>(index)];
    if (!hasValidRoiPoints(entry.roi_points) || entry.roi_points.size() != 4)
    {
      setSaveStatus("Bin Teach file has invalid roi_points", 2.0);
      return false;
    }
    const bool same_image_size =
      current_size.width == entry.image_width &&
      current_size.height == entry.image_height;
    std::vector<cv::Point2f> loaded_roi_points = entry.roi_points;
    if (!same_image_size)
    {
      loaded_roi_points.clear();
      loaded_roi_points.reserve(4);
      if (entry.roi_points_normalized.size() == 4)
      {
        for (const auto &point : entry.roi_points_normalized)
        {
          loaded_roi_points.emplace_back(
            std::clamp(point.x * static_cast<float>(current_size.width), 0.0F, static_cast<float>(current_size.width - 1)),
            std::clamp(point.y * static_cast<float>(current_size.height), 0.0F, static_cast<float>(current_size.height - 1)));
        }
      }
      else
      {
        const float scale_x = static_cast<float>(current_size.width) / static_cast<float>(std::max(1, entry.image_width));
        const float scale_y = static_cast<float>(current_size.height) / static_cast<float>(std::max(1, entry.image_height));
        for (const auto &point : entry.roi_points)
        {
          loaded_roi_points.emplace_back(
            std::clamp(point.x * scale_x, 0.0F, static_cast<float>(current_size.width - 1)),
            std::clamp(point.y * scale_y, 0.0F, static_cast<float>(current_size.height - 1)));
        }
      }
    }

    roi_regions_.clear();
    roi_points_ = loaded_roi_points;
    clearDepthPlaneReference(false);
    if (entry.has_depth_plane && entry.depth_plane_roi_bounds.has_value())
    {
      std::optional<AxisAlignedRoiBounds> plane_bounds = entry.depth_plane_roi_bounds;
      if (!same_image_size)
      {
        const double scale_x = static_cast<double>(current_size.width) / static_cast<double>(std::max(1, entry.image_width));
        const double scale_y = static_cast<double>(current_size.height) / static_cast<double>(std::max(1, entry.image_height));
        AxisAlignedRoiBounds scaled_bounds{
          static_cast<int>(std::lround(static_cast<double>(entry.depth_plane_roi_bounds->left) * scale_x)),
          static_cast<int>(std::lround(static_cast<double>(entry.depth_plane_roi_bounds->top) * scale_y)),
          static_cast<int>(std::lround(static_cast<double>(entry.depth_plane_roi_bounds->right) * scale_x)),
          static_cast<int>(std::lround(static_cast<double>(entry.depth_plane_roi_bounds->bottom) * scale_y)),
        };
        scaled_bounds.left = std::clamp(scaled_bounds.left, 0, current_size.width - 1);
        scaled_bounds.right = std::clamp(scaled_bounds.right, 0, current_size.width - 1);
        scaled_bounds.top = std::clamp(scaled_bounds.top, 0, current_size.height - 1);
        scaled_bounds.bottom = std::clamp(scaled_bounds.bottom, 0, current_size.height - 1);
        if (isValidRoiBounds(scaled_bounds))
        {
          plane_bounds = scaled_bounds;
        }
        else
        {
          plane_bounds = roiBoundsFromSelection(roi_points_);
        }
      }
      depth_plane_model_ = entry.depth_plane;
      depth_plane_roi_points_ = loaded_roi_points;
      depth_plane_roi_bounds_ = roiBoundsFromSelection(depth_plane_roi_points_);
      if (!depth_plane_roi_bounds_.has_value())
      {
        depth_plane_roi_bounds_ = plane_bounds;
      }
      pending_depth_plane_points_.clear();
      depth_plane_from_bin_teach_ = depth_plane_model_.valid && depth_plane_roi_bounds_.has_value();
    }
    else
    {
      depth_plane_from_bin_teach_ = false;
    }
    active_bin_name_ = entry.bin_name.empty() ? entry.label : entry.bin_name;
    active_bin_teach_path_ = entry.path;
    pose_estimate_.reset();
    pose_blob_seed_point_px_.reset();
    setTeachStage(TeachStage::kColorMask, false);
    view_mode_ = ViewMode::kBinarized;
    resetTaughtMetrics();
    markRuntimeSettingsDirty();

    setSaveStatus(sendBinRoiMove(entry), 2.0);
    return true;
  }

  void selectBinTeachRoi(int index)
  {
    if (index < 0 || index >= static_cast<int>(bin_roi_entries_.size()))
    {
      setSaveStatus("Load Bin Teach selection is not valid", 1.6);
      return;
    }

    selected_bin_roi_index_ = index;
    pending_bin_roi_index_ = -1;
    const auto frame_size = currentColorFrameSize();
    if (!frame_size.has_value())
    {
      pending_bin_roi_index_ = index;
      setSaveStatus("Bin Teach selected; waiting for camera frame", 2.0);
      return;
    }
    applyBinTeachRoi(index, *frame_size);
  }

  void tryApplyPendingBinTeachRoi(const cv::Size &current_size)
  {
    if (pending_bin_roi_index_ < 0)
    {
      return;
    }
    const int index = pending_bin_roi_index_;
    pending_bin_roi_index_ = -1;
    applyBinTeachRoi(index, current_size);
  }

  bool canDeleteSelectedBinTeach() const
  {
    return selected_bin_roi_index_ >= 0 &&
      selected_bin_roi_index_ < static_cast<int>(bin_roi_entries_.size());
  }

  bool deleteSelectedBinTeachArmed() const
  {
    if (!canDeleteSelectedBinTeach())
    {
      return false;
    }
    const auto &entry = bin_roi_entries_[static_cast<std::size_t>(selected_bin_roi_index_)];
    return !pending_delete_bin_teach_path_.empty() &&
      pending_delete_bin_teach_path_ == entry.path &&
      this->now() < pending_delete_bin_teach_deadline_;
  }

  void clearLoadedBinTeachState()
  {
    active_bin_name_.clear();
    active_bin_teach_path_.clear();
    roi_regions_.clear();
    roi_points_.clear();
    clearDepthPlaneReference(false);
    pose_estimate_.reset();
    pose_blob_seed_point_px_.reset();
    pose_blob_reference_.reset();
    pose_blob_reference_clicks_px_.clear();
    clearAllPoseReferenceSlots();
    pose_stage_status_.clear();
    setTeachStage(TeachStage::kRoi, false);
    view_mode_ = ViewMode::kRgb;
    markRuntimeSettingsDirty();
  }

  void deleteSelectedBinTeach()
  {
    if (!canDeleteSelectedBinTeach())
    {
      setSaveStatus("Select a Bin Teach file to delete", 1.8);
      return;
    }

    const BinTeachRoiEntry entry = bin_roi_entries_[static_cast<std::size_t>(selected_bin_roi_index_)];
    if (!deleteSelectedBinTeachArmed())
    {
      pending_delete_bin_teach_path_ = entry.path;
      pending_delete_bin_teach_deadline_ = this->now() + rclcpp::Duration::from_seconds(3.0);
      bin_roi_dropdown_open_ = false;
      setSaveStatus("Click Delete again to remove " + entry.label, 3.0);
      return;
    }

    const std::filesystem::path delete_path(entry.path);
    std::error_code fs_error;
    const bool removed = std::filesystem::remove(delete_path, fs_error);
    bin_roi_dropdown_open_ = false;
    pending_bin_roi_index_ = -1;
    pending_delete_bin_teach_path_.clear();
    pending_delete_bin_teach_deadline_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (!removed || fs_error)
    {
      setSaveStatus("Delete Bin failed", 2.0);
      RCLCPP_WARN(
        get_logger(),
        "Failed to delete Bin Teach file %s: %s",
        delete_path.c_str(),
        fs_error ? fs_error.message().c_str() : "file was not removed");
      return;
    }

    const bool deleted_active = !active_bin_teach_path_.empty() && active_bin_teach_path_ == entry.path;
    refreshBinTeachFiles();
    if (deleted_active)
    {
      clearLoadedBinTeachState();
    }
    setSaveStatus("Deleted Bin Teach: " + delete_path.filename().string(), 2.2);
  }

  bool handleBinRoiDropdownMouseDown(const cv::Point &ui_point)
  {
    if (delete_bin_roi_button_rect_.contains(ui_point))
    {
      name_edit_active_ = false;
      active_slider_index_ = -1;
      deleteSelectedBinTeach();
      return true;
    }

    if (bin_roi_dropdown_open_)
    {
      const int visible_count = visibleBinRoiOptionCount();
      for (int i = 0; i < visible_count; ++i)
      {
        if (binRoiOptionRect(i).contains(ui_point))
        {
          bin_roi_dropdown_open_ = false;
          name_edit_active_ = false;
          active_slider_index_ = -1;
          selectBinTeachRoi(i);
          return true;
        }
      }

      if (bin_roi_dropdown_rect_.contains(ui_point))
      {
        bin_roi_dropdown_open_ = false;
        name_edit_active_ = false;
        active_slider_index_ = -1;
        return true;
      }

      bin_roi_dropdown_open_ = false;
      return true;
    }

    if (bin_roi_dropdown_rect_.contains(ui_point))
    {
      refreshBinTeachFiles();
      bin_roi_dropdown_open_ = !bin_roi_entries_.empty();
      name_edit_active_ = false;
      active_slider_index_ = -1;
      if (bin_roi_entries_.empty())
      {
        setSaveStatus(loadBinTeachEmptyStatus(), 2.3);
      }
      return true;
    }

    return false;
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

      std::filesystem::path preferred_path;
      std::filesystem::file_time_type preferred_time;
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
        if (filename.rfind("axab_calibration_eyeonhand_", 0) != 0)
        {
          continue;
        }
        if (preferred_path.empty() || entry.last_write_time() > preferred_time)
        {
          preferred_path = p;
          preferred_time = entry.last_write_time();
        }
      }
      return preferred_path.string();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Failed to discover calibration files: %s", ex.what());
      return {};
    }
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

	  void markRuntimeSettingsDirty()
	  {
	    runtime_settings_dirty_ = true;
	  }

	  void markCameraExposureDirty()
	  {
	    camera_exposure_dirty_ = true;
	  }

	  void normalizeCameraControlServiceRoot()
	  {
	    if (camera_control_service_root_.empty())
	    {
	      camera_control_service_root_ = "/robot_camera";
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

	  bool saveRuntimeSettingsToFile(bool force)
	  {
    if (runtime_settings_path_.empty())
    {
      return false;
    }
    depth_exposure_us_ = 0;

    const rclcpp::Time now = this->now();
    if (!force)
    {
      if (!runtime_settings_dirty_)
      {
        return false;
      }
      if (last_runtime_settings_save_time_.nanoseconds() != 0 &&
          (now - last_runtime_settings_save_time_).seconds() < kRuntimeSettingsSaveIntervalSec)
      {
        return false;
      }
    }

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "item_teach_runtime";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "ros__parameters";
    out << YAML::Value << YAML::BeginMap;
	    out << YAML::Key << "red_threshold" << YAML::Value << red_threshold_;
	    out << YAML::Key << "green_threshold" << YAML::Value << green_threshold_;
	    out << YAML::Key << "blue_threshold" << YAML::Value << blue_threshold_;
	    out << YAML::Key << "color_exposure_us" << YAML::Value << color_exposure_us_;
	    out << YAML::Key << "depth_exposure_us" << YAML::Value << 0;
	    out << YAML::Key << "color_exposure_percent" << YAML::Value
	        << exposureUsecToPercent(color_exposure_us_, color_exposure_min_us_, color_exposure_max_us_);
	    out << YAML::Key << "depth_exposure_percent" << YAML::Value << 0;
	    out << YAML::Key << "color_exposure_min_us" << YAML::Value << color_exposure_min_us_;
	    out << YAML::Key << "color_exposure_max_us" << YAML::Value << color_exposure_max_us_;
	    out << YAML::Key << "depth_exposure_min_us" << YAML::Value << depth_exposure_min_us_;
	    out << YAML::Key << "depth_exposure_max_us" << YAML::Value << depth_exposure_max_us_;
	    out << YAML::Key << "rgb_hole_fill_sensitivity" << YAML::Value << rgb_hole_fill_sensitivity_;
    out << YAML::Key << "rgb_mask_dilate_px" << YAML::Value << rgb_mask_dilate_px_;
    out << YAML::Key << "depth_null_fill_sensitivity" << YAML::Value << depth_null_fill_sensitivity_;
    out << YAML::Key << "depth_window_mm" << YAML::Value << depth_window_mm_;
    out << YAML::Key << "depth_hole_fill_sensitivity" << YAML::Value << depth_hole_fill_sensitivity_;
    out << YAML::Key << "depth_trim_px" << YAML::Value << depth_trim_px_;
    out << YAML::Key << "adaptive_depth_trim_max_add_px" << YAML::Value
        << adaptive_depth_trim_max_factor_tenths_;
    out << YAML::Key << "adaptive_depth_trim_max_height_mm" << YAML::Value << adaptive_depth_trim_max_height_mm_;
    out << YAML::Key << "blob_tolerance_percent" << YAML::Value << blob_tolerance_percent_;
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;

    const std::filesystem::path runtime_path = resolvePath(runtime_settings_path_);
    const std::filesystem::path runtime_dir = runtime_path.parent_path();
    std::error_code fs_error;
    if (!runtime_dir.empty())
    {
      std::filesystem::create_directories(runtime_dir, fs_error);
      if (fs_error)
      {
        return false;
      }
    }

    std::ofstream runtime_file(runtime_path);
    if (!runtime_file.is_open())
    {
      return false;
    }
    runtime_file << out.c_str() << '\n';
    runtime_file.close();

    runtime_settings_dirty_ = false;
    last_runtime_settings_save_time_ = now;
    return true;
  }

  void loadRuntimeSettingsFromFile()
  {
    if (runtime_settings_path_.empty())
    {
      return;
    }

    try
    {
      std::filesystem::path source_path = resolvePath(runtime_settings_path_);
      if (source_path.empty())
      {
        return;
      }
      if (!std::filesystem::exists(source_path))
      {
        return;
      }

      const YAML::Node root = YAML::LoadFile(source_path.string());
      YAML::Node params;
      if (root["item_teach_runtime"] && root["item_teach_runtime"]["ros__parameters"])
      {
        params = root["item_teach_runtime"]["ros__parameters"];
      }

      if (!params || !params.IsMap())
      {
        return;
      }

	      red_threshold_ = params["red_threshold"] ? std::clamp(params["red_threshold"].as<int>(), 0, 255) : red_threshold_;
	      green_threshold_ = params["green_threshold"] ? std::clamp(params["green_threshold"].as<int>(), 0, 255) : green_threshold_;
	      blue_threshold_ = params["blue_threshold"] ? std::clamp(params["blue_threshold"].as<int>(), 0, 255) : blue_threshold_;
	      color_exposure_min_us_ = params["color_exposure_min_us"]
	        ? clampExposureUsec(params["color_exposure_min_us"].as<int>())
	        : color_exposure_min_us_;
	    color_exposure_max_us_ = params["color_exposure_max_us"]
	        ? std::max(color_exposure_min_us_, clampExposureUsec(params["color_exposure_max_us"].as<int>()))
	        : color_exposure_max_us_;
	      color_exposure_min_us_ = std::clamp(color_exposure_min_us_, 1, kTeachColorExposureMaxUs);
	      color_exposure_max_us_ = kTeachColorExposureMaxUs;
	      depth_exposure_min_us_ = params["depth_exposure_min_us"]
	        ? clampExposureUsec(params["depth_exposure_min_us"].as<int>())
	        : depth_exposure_min_us_;
	      depth_exposure_max_us_ = params["depth_exposure_max_us"]
	        ? std::max(depth_exposure_min_us_, clampExposureUsec(params["depth_exposure_max_us"].as<int>()))
	        : depth_exposure_max_us_;
	      color_exposure_us_ = params["color_exposure_us"]
	        ? clampExposureUsecOrAuto(params["color_exposure_us"].as<int>(), color_exposure_min_us_, color_exposure_max_us_)
	        : (
	            params["color_exposure_percent"]
	            ? exposurePercentToUsec(
	                clampExposurePercent(params["color_exposure_percent"].as<int>()),
	                color_exposure_min_us_,
	                color_exposure_max_us_)
	            : color_exposure_us_);
	      depth_exposure_us_ = 0;
	      markCameraExposureDirty();
	      rgb_hole_fill_sensitivity_ = params["rgb_hole_fill_sensitivity"]
        ? std::clamp(params["rgb_hole_fill_sensitivity"].as<int>(), kRgbHoleFillMin, kRgbHoleFillMax)
        : rgb_hole_fill_sensitivity_;
      rgb_mask_dilate_px_ = params["rgb_mask_dilate_px"]
        ? std::clamp(params["rgb_mask_dilate_px"].as<int>(), kRgbDilateMinPx, kRgbDilateMaxPx)
        : rgb_mask_dilate_px_;
      depth_null_fill_sensitivity_ = params["depth_null_fill_sensitivity"]
        ? std::clamp(params["depth_null_fill_sensitivity"].as<int>(), kDepthFillSensitivityMin, kDepthFillSensitivityMax)
        : depth_null_fill_sensitivity_;
      depth_window_mm_ = params["depth_window_mm"]
        ? std::clamp(params["depth_window_mm"].as<int>(), kDepthWindowMinMm, kDepthWindowMaxMm)
        : depth_window_mm_;
      depth_hole_fill_sensitivity_ = params["depth_hole_fill_sensitivity"]
        ? std::clamp(
            params["depth_hole_fill_sensitivity"].as<int>(),
            kDepthFillSensitivityMin,
            kDepthFillSensitivityMax)
        : depth_hole_fill_sensitivity_;
      depth_trim_px_ = params["depth_trim_px"]
        ? std::clamp(params["depth_trim_px"].as<int>(), kDepthTrimMinPx, kDepthTrimMaxPx)
        : depth_trim_px_;
      if (params["adaptive_depth_trim_max_add_px"])
      {
        adaptive_depth_trim_max_factor_tenths_ = parseSavedAdaptiveDepthTrimAddPx(
          params["adaptive_depth_trim_max_add_px"],
          adaptive_depth_trim_max_factor_tenths_);
      }
      adaptive_depth_trim_max_height_mm_ = params["adaptive_depth_trim_max_height_mm"]
        ? std::clamp(
            params["adaptive_depth_trim_max_height_mm"].as<int>(),
            kAdaptiveDepthTrimHeightMinMm,
            kAdaptiveDepthTrimHeightMaxMm)
        : adaptive_depth_trim_max_height_mm_;
      blob_tolerance_percent_ = params["blob_tolerance_percent"]
        ? std::clamp(
            params["blob_tolerance_percent"].as<int>(),
            kBlobToleranceMinPercent,
            kBlobToleranceMaxPercent)
        : blob_tolerance_percent_;
      clearDepthPlaneReference(false);
      roi_regions_.clear();
      roi_points_.clear();
      active_bin_name_.clear();
      active_bin_teach_path_.clear();
      clearAllPoseReferenceSlots();
      setTeachStage(TeachStage::kRoi, false);

      resetTaughtMetrics();
      runtime_settings_dirty_ = false;
      last_runtime_settings_save_time_ = this->now();
      RCLCPP_INFO(
        get_logger(),
        "Loaded item teach runtime settings from %s",
        source_path.c_str());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Runtime settings load failed: %s", ex.what());
    }
  }

  bool poseBlobReferenceReadyForSave(const PoseBlobReference2D &reference_blob) const
  {
    const auto finite_point = [](const cv::Point2f &point)
    {
      return std::isfinite(point.x) && std::isfinite(point.y);
    };

    if (reference_blob.area_px <= 0 || reference_blob.hull.size() < 3)
    {
      return false;
    }
    if (reference_blob.mode == PoseTemplateMode2D::kSingle)
    {
      return true;
    }

    const std::vector<cv::Point> &anchor_hull =
      reference_blob.anchor_hull.empty() ? reference_blob.hull : reference_blob.anchor_hull;
    const std::vector<cv::Point> &group_hull =
      reference_blob.group_hull.empty() ? reference_blob.hull : reference_blob.group_hull;
    if (
      reference_blob.member_count != 2 ||
      anchor_hull.size() < 3 ||
      reference_blob.companion_hull.size() < 3 ||
      reference_blob.companion_area_px <= 0 ||
      reference_blob.companion_aspect_ratio <= 1e-3F ||
      reference_blob.companion_fill_ratio <= 0.0 ||
      group_hull.size() < 3 ||
      reference_blob.group_area_px <= 0 ||
      reference_blob.group_aspect_ratio <= 1e-3F ||
      reference_blob.member_centers_norm.size() < 2 ||
      !finite_point(reference_blob.anchor_center_norm))
    {
      return false;
    }

    return finite_point(reference_blob.member_centers_norm[0]) &&
      finite_point(reference_blob.member_centers_norm[1]);
  }

  std::optional<int> firstInvalidPoseReferenceSlotIndexForSave() const
  {
    for (int i = 0; i < kPoseReferenceSlotCount; ++i)
    {
      const auto reference = poseReferenceForSlot(i);
      if (reference.has_value() && !poseBlobReferenceReadyForSave(*reference))
      {
        return i;
      }
    }
    return std::nullopt;
  }

  std::optional<PoseBlobReference2D> currentPoseBlobReferenceForSave() const
  {
    if (const auto first_slot = firstFilledPoseReferenceSlotIndex(); first_slot.has_value())
    {
      return poseReferenceForSlot(*first_slot);
    }
    return std::nullopt;
  }

  void emitPoseBlobReferenceYaml(YAML::Emitter &out, const PoseBlobReference2D &reference_blob) const
  {
    out << YAML::Key << "pose_template_mode" << YAML::Value
        << poseTemplateModeToString(reference_blob.mode);
    out << YAML::Key << "pose_blob_reference_area_px" << YAML::Value << reference_blob.area_px;
    out << YAML::Key << "pose_blob_reference_aspect_ratio" << YAML::Value
        << static_cast<double>(reference_blob.aspect_ratio);
    out << YAML::Key << "reference_blob_fill_ratio" << YAML::Value
        << reference_blob.fill_ratio;
    out << YAML::Key << "pose_blob_reference_hull_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto &point : reference_blob.hull)
    {
      out << point.x << point.y;
    }
    out << YAML::EndSeq;
    if (reference_blob.mode == PoseTemplateMode2D::kPair)
    {
      const std::vector<cv::Point> &anchor_hull_for_save =
        reference_blob.anchor_hull.empty() ? reference_blob.hull : reference_blob.anchor_hull;
      const std::vector<cv::Point> &group_hull_for_save =
        reference_blob.group_hull.empty() ? reference_blob.hull : reference_blob.group_hull;

      out << YAML::Key << "pose_group_member_count" << YAML::Value << reference_blob.member_count;
      out << YAML::Key << "member_count" << YAML::Value << reference_blob.member_count;
      out << YAML::Key << "group_area_px" << YAML::Value << std::max(1, reference_blob.group_area_px);
      out << YAML::Key << "group_aspect_ratio" << YAML::Value
          << static_cast<double>(std::max(1e-3F, reference_blob.group_aspect_ratio));
      out << YAML::Key << "pose_group_reference_area_px" << YAML::Value
          << std::max(1, reference_blob.group_area_px);
      out << YAML::Key << "pose_group_reference_aspect_ratio" << YAML::Value
          << static_cast<double>(std::max(1e-3F, reference_blob.group_aspect_ratio));
      out << YAML::Key << "anchor_hull" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : anchor_hull_for_save)
      {
        out << point.x << point.y;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "companion_hull" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : reference_blob.companion_hull)
      {
        out << point.x << point.y;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "companion_area_px" << YAML::Value
          << std::max(1, reference_blob.companion_area_px);
      out << YAML::Key << "companion_aspect_ratio" << YAML::Value
          << static_cast<double>(std::max(1e-3F, reference_blob.companion_aspect_ratio));
      out << YAML::Key << "companion_fill_ratio" << YAML::Value
          << std::clamp(reference_blob.companion_fill_ratio, 0.0, 1.0);
      out << YAML::Key << "group_hull" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : group_hull_for_save)
      {
        out << point.x << point.y;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "member_centers_norm" << YAML::Value
          << YAML::Flow << YAML::BeginSeq;
      for (const auto &center_norm : reference_blob.member_centers_norm)
      {
        out << static_cast<double>(center_norm.x);
        out << static_cast<double>(center_norm.y);
      }
      out << YAML::EndSeq;
      out << YAML::Key << "anchor_center_norm" << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << static_cast<double>(reference_blob.anchor_center_norm.x)
          << static_cast<double>(reference_blob.anchor_center_norm.y)
          << YAML::EndSeq;
      out << YAML::Key << "reference_blob_hull_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : anchor_hull_for_save)
      {
        out << point.x << point.y;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "auxiliary_blob_hull_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : reference_blob.companion_hull)
      {
        out << point.x << point.y;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "auxiliary_blob_reference_area_px" << YAML::Value
          << std::max(1, reference_blob.companion_area_px);
      out << YAML::Key << "auxiliary_blob_reference_aspect_ratio" << YAML::Value
          << static_cast<double>(std::max(1e-3F, reference_blob.companion_aspect_ratio));
      out << YAML::Key << "auxiliary_blob_fill_ratio" << YAML::Value
          << reference_blob.companion_fill_ratio;
      out << YAML::Key << "pose_group_reference_hull_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : group_hull_for_save)
      {
        out << point.x << point.y;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "pose_group_reference_member_centers_norm" << YAML::Value
          << YAML::Flow << YAML::BeginSeq;
      for (const auto &center_norm : reference_blob.member_centers_norm)
      {
        out << static_cast<double>(center_norm.x);
        out << static_cast<double>(center_norm.y);
      }
      out << YAML::EndSeq;
      out << YAML::Key << "pose_group_reference_anchor_center_norm" << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << static_cast<double>(reference_blob.anchor_center_norm.x)
          << static_cast<double>(reference_blob.anchor_center_norm.y)
          << YAML::EndSeq;
    }
  }

  void emitPoseReferenceSlotsYaml(YAML::Emitter &out) const
  {
    out << YAML::Key << "pose_reference_slots" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < kPoseReferenceSlotCount; ++i)
    {
      const auto reference = poseReferenceForSlot(i);
      out << YAML::BeginMap;
      out << YAML::Key << "slot_index" << YAML::Value << (i + 1);
      out << YAML::Key << "enabled" << YAML::Value << reference.has_value();
      if (reference.has_value())
      {
        emitPoseBlobReferenceYaml(out, *reference);
      }
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  const BinarizedPoseEstimate2D::BlobPose2D *currentPrimaryPoseForSave() const
  {
    if (!pose_estimate_.has_value() || pose_estimate_->blob_poses.empty())
    {
      return nullptr;
    }

    const BinarizedPoseEstimate2D::BlobPose2D *selected_pose = &pose_estimate_->blob_poses.front();
    if (!pose_blob_reference_clicks_px_.empty())
    {
      const cv::Point2f clicked_anchor(
        static_cast<float>(pose_blob_reference_clicks_px_.front().x),
        static_cast<float>(pose_blob_reference_clicks_px_.front().y));
      double best_sq_distance = std::numeric_limits<double>::infinity();
      for (const auto &blob_pose : pose_estimate_->blob_poses)
      {
        const cv::Point2f anchor_point = blob_pose.has_custom_anchor ? blob_pose.anchor_point_px : poseBlobCenterPx(blob_pose);
        const cv::Point2f delta = anchor_point - clicked_anchor;
        const double sq_distance = static_cast<double>(delta.dot(delta));
        if (sq_distance < best_sq_distance)
        {
          best_sq_distance = sq_distance;
          selected_pose = &blob_pose;
        }
      }
    }
    return selected_pose;
  }

  void saveSettingsToFile()
  {
    persistActivePoseReferenceSlot();
    if (!isFinalTeachStage())
    {
      save_status_message_ = "Complete stages then Save Item";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    if (!hasValidRoiPoints(roi_points_))
    {
      save_status_message_ = "ROI required";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    if (!hasDepthPlaneReference())
    {
      save_status_message_ = "Depth normalization required";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    if (filledPoseReferenceSlotCount() <= 0)
    {
      save_status_message_ = "Pose reference slot required";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    if (const auto invalid_slot = firstInvalidPoseReferenceSlotIndexForSave(); invalid_slot.has_value())
    {
      save_status_message_ =
        "Pose slot " + std::to_string(*invalid_slot + 1) + " incomplete; reselect 1/2 and 2/2";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(2.0);
      return;
    }

    std::array<double, 6> joints_deg_snapshot {};
    bool has_joint_snapshot = false;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joints_deg_snapshot = latest_joint_positions_deg_;
      has_joint_snapshot = has_joint_positions_;
    }

    const std::string item_name = sanitizeItemName();
    const DateStamp date_stamp = currentDateStamp();
    depth_exposure_us_ = 0;

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "item_detect";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "ros__parameters";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "color_topic" << YAML::Value << color_topic_;
    out << YAML::Key << "depth_topic" << YAML::Value << depth_topic_;
    out << YAML::Key << "camera_info_topic" << YAML::Value << camera_info_topic_;
    out << YAML::Key << "overlay_topic" << YAML::Value << overlay_topic_;
	    out << YAML::Key << "red_threshold" << YAML::Value << red_threshold_;
	    out << YAML::Key << "green_threshold" << YAML::Value << green_threshold_;
	    out << YAML::Key << "blue_threshold" << YAML::Value << blue_threshold_;
	    out << YAML::Key << "color_exposure_us" << YAML::Value << color_exposure_us_;
	    out << YAML::Key << "depth_exposure_us" << YAML::Value << 0;
	    out << YAML::Key << "color_exposure_percent" << YAML::Value
	        << exposureUsecToPercent(color_exposure_us_, color_exposure_min_us_, color_exposure_max_us_);
	    out << YAML::Key << "depth_exposure_percent" << YAML::Value << 0;
	    out << YAML::Key << "color_exposure_min_us" << YAML::Value << color_exposure_min_us_;
	    out << YAML::Key << "color_exposure_max_us" << YAML::Value << color_exposure_max_us_;
	    out << YAML::Key << "depth_exposure_min_us" << YAML::Value << depth_exposure_min_us_;
	    out << YAML::Key << "depth_exposure_max_us" << YAML::Value << depth_exposure_max_us_;
	    out << YAML::Key << "rgb_hole_fill_sensitivity" << YAML::Value << rgb_hole_fill_sensitivity_;
    out << YAML::Key << "rgb_mask_dilate_px" << YAML::Value << rgb_mask_dilate_px_;
    out << YAML::Key << "depth_null_fill_sensitivity" << YAML::Value << depth_null_fill_sensitivity_;
    out << YAML::Key << "depth_window_mm" << YAML::Value << depth_window_mm_;
    out << YAML::Key << "depth_hole_fill_sensitivity" << YAML::Value << depth_hole_fill_sensitivity_;
    out << YAML::Key << "depth_trim_px" << YAML::Value << depth_trim_px_;
    out << YAML::Key << "adaptive_depth_trim_max_add_px" << YAML::Value
        << adaptive_depth_trim_max_factor_tenths_;
    out << YAML::Key << "adaptive_depth_trim_max_height_mm" << YAML::Value << adaptive_depth_trim_max_height_mm_;
    out << YAML::Key << "focus_black_mask" << YAML::Value << focus_black_mask_;
    out << YAML::Key << "detection_mode" << YAML::Value << detectionModeToString(detection_use_depth_);
    out << YAML::Key << "associated_bin_name" << YAML::Value << active_bin_name_;
    out << YAML::Key << "bin_teach_file" << YAML::Value << active_bin_teach_path_;
    out << YAML::Key << "depth_plane_source" << YAML::Value
        << (depth_plane_from_bin_teach_ ? "bin_teach" : "item_teach");
    out << YAML::Key << "depth_plane_enabled" << YAML::Value << depth_plane_model_.valid;
    out << YAML::Key << "depth_plane_a" << YAML::Value << depth_plane_model_.a;
    out << YAML::Key << "depth_plane_b" << YAML::Value << depth_plane_model_.b;
    out << YAML::Key << "depth_plane_c" << YAML::Value << depth_plane_model_.c;
    out << YAML::Key << "depth_plane_reference_depth_m" << YAML::Value << depth_plane_model_.reference_depth_m;
    out << YAML::Key << "align_item_z_axis_to_depth_plane" << YAML::Value
        << (depth_plane_model_.valid && align_item_z_axis_to_depth_plane_);
    out << YAML::Key << "depth_plane_roi" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    if (depth_plane_roi_bounds_.has_value())
    {
      out << depth_plane_roi_bounds_->left
          << depth_plane_roi_bounds_->top
          << depth_plane_roi_bounds_->right
          << depth_plane_roi_bounds_->bottom;
    }
    else
    {
      out << 0 << 0 << 0 << 0;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "roi_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto &point : roi_points_)
    {
      out << static_cast<int>(std::round(point.x));
      out << static_cast<int>(std::round(point.y));
    }
    out << YAML::EndSeq;
    std::optional<PoseBlobReference2D> reference_blob_to_save = currentPoseBlobReferenceForSave();
    if (reference_blob_to_save.has_value())
    {
      emitPoseBlobReferenceYaml(out, *reference_blob_to_save);
    }
    emitPoseReferenceSlotsYaml(out);
    const bool has_pose_preview =
      pose_estimate_.has_value() &&
      !pose_estimate_->blob_poses.empty();
    out << YAML::Key << "pose_blob_count" << YAML::Value <<
      (has_pose_preview ? static_cast<int>(pose_estimate_->blob_poses.size()) : 0);
    out << YAML::Key << "pose_blobs" << YAML::Value << YAML::BeginSeq;
    if (has_pose_preview)
    {
      for (const auto &blob_pose : pose_estimate_->blob_poses)
      {
        out << YAML::BeginMap;
        out << YAML::Key << "label" << YAML::Value << blob_pose.label;
        out << YAML::Key << "quad_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (const auto &corner : blob_pose.corners)
        {
          out << static_cast<int>(std::round(corner.x));
          out << static_cast<int>(std::round(corner.y));
        }
        out << YAML::EndSeq;
        out << YAML::Key << "origin_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << static_cast<int>(std::round(blob_pose.origin.x))
            << static_cast<int>(std::round(blob_pose.origin.y))
            << YAML::EndSeq;
        out << YAML::Key << "x_axis_tip_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << static_cast<int>(std::round(blob_pose.x_axis_tip.x))
            << static_cast<int>(std::round(blob_pose.x_axis_tip.y))
            << YAML::EndSeq;
        out << YAML::Key << "z_axis_tip_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << static_cast<int>(std::round(blob_pose.z_axis_tip.x))
            << static_cast<int>(std::round(blob_pose.z_axis_tip.y))
            << YAML::EndSeq;
        out << YAML::Key << "x_length_px" << YAML::Value << static_cast<double>(blob_pose.x_length_px);
        out << YAML::Key << "z_length_px" << YAML::Value << static_cast<double>(blob_pose.z_length_px);
        out << YAML::Key << "member_count" << YAML::Value << blob_pose.member_count;
        if (blob_pose.has_custom_anchor)
        {
          out << YAML::Key << "anchor_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
              << static_cast<double>(blob_pose.anchor_point_px.x)
              << static_cast<double>(blob_pose.anchor_point_px.y)
              << YAML::EndSeq;
        }
        if (!blob_pose.member_centers_norm.empty())
        {
          out << YAML::Key << "member_centers_norm" << YAML::Value << YAML::Flow << YAML::BeginSeq;
          for (const auto &center_norm : blob_pose.member_centers_norm)
          {
            out << static_cast<double>(center_norm.x);
            out << static_cast<double>(center_norm.y);
          }
          out << YAML::EndSeq;
        }
        out << YAML::EndMap;
      }
    }
    out << YAML::EndSeq;

    if (has_pose_preview)
    {
      const BinarizedPoseEstimate2D::BlobPose2D *primary_pose = currentPrimaryPoseForSave();
      if (primary_pose == nullptr)
      {
        primary_pose = &pose_estimate_->blob_poses.front();
      }
      out << YAML::Key << "pose_quad_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &corner : primary_pose->corners)
      {
        out << static_cast<int>(std::round(corner.x));
        out << static_cast<int>(std::round(corner.y));
      }
      out << YAML::EndSeq;
      out << YAML::Key << "pose_origin_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << static_cast<int>(std::round(primary_pose->origin.x))
          << static_cast<int>(std::round(primary_pose->origin.y))
          << YAML::EndSeq;
      out << YAML::Key << "pose_x_axis_tip_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << static_cast<int>(std::round(primary_pose->x_axis_tip.x))
          << static_cast<int>(std::round(primary_pose->x_axis_tip.y))
          << YAML::EndSeq;
      out << YAML::Key << "pose_z_axis_tip_px" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << static_cast<int>(std::round(primary_pose->z_axis_tip.x))
          << static_cast<int>(std::round(primary_pose->z_axis_tip.y))
          << YAML::EndSeq;
      out << YAML::Key << "pose_x_length_px" << YAML::Value << static_cast<double>(primary_pose->x_length_px);
      out << YAML::Key << "pose_z_length_px" << YAML::Value << static_cast<double>(primary_pose->z_length_px);
    }
    out << YAML::Key << "item_name" << YAML::Value << item_name;
    out << YAML::Key << "teach_date" << YAML::Value << date_stamp.iso_date;
    if (has_joint_snapshot)
    {
      out << YAML::Key << "teach_joints_deg" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const double value_deg : joints_deg_snapshot)
      {
        out << value_deg;
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::error_code fs_error;
    std::filesystem::create_directories(profiles_dir_, fs_error);
    if (fs_error)
    {
      save_status_message_ = "Save failed";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }

    const std::filesystem::path dated_profile_path =
      std::filesystem::path(profiles_dir_) /
      ("item_" + makeFilenameSafeItemName(item_name) +
       (active_bin_name_.empty() ? "" : "_bin_" + makeFilenameSafeItemName(active_bin_name_)) +
       "_" + date_stamp.compact_date + ".yaml");

    std::ofstream dated_file(dated_profile_path);
    if (!dated_file.is_open())
    {
      save_status_message_ = "Save failed";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    dated_file << out.c_str() << '\n';
    dated_file.close();

    latest_saved_profile_path_ = dated_profile_path.string();
    save_status_message_ = "Item saved | Pose refs " + std::to_string(filledPoseReferenceSlotCount()) + "/4";
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
  }

  bool fitDepthNormalizeFromSelectedPoints()
  {
    if (pending_depth_plane_points_.size() != 4)
    {
      save_status_message_ = "Depth points " + std::to_string(pending_depth_plane_points_.size()) + "/4";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return false;
    }

    cv::Mat depth_for_fit;
    if (!getDepthFrameForPlaneFit(depth_for_fit))
    {
      save_status_message_ = "Depth frame unavailable";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return false;
    }

    DepthPlaneModel fitted_plane;
    if (!fitDepthPlaneFromCorners(depth_for_fit, pending_depth_plane_points_, fitted_plane))
    {
      pending_depth_plane_points_.clear();
      save_status_message_ = "Depth fit failed. Reclick 4 points";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.8);
      markRuntimeSettingsDirty();
      return false;
    }

    depth_plane_model_ = fitted_plane;
    depth_plane_roi_points_ = pending_depth_plane_points_;
    depth_plane_from_bin_teach_ = false;
    pending_depth_plane_points_.clear();
    depth_plane_roi_bounds_ = roiBoundsFromSelection(depth_plane_roi_points_);
    if (!depth_plane_roi_bounds_.has_value())
    {
      depth_plane_roi_bounds_ = roiBoundsFromSelection(roi_points_);
    }
    setTeachStage(TeachStage::kFlatLocate, false);
    save_status_message_ = "Depth plane set (4/4). Stage: tune depth window/hole/trim";
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.8);
    markRuntimeSettingsDirty();
    return true;
  }

  bool fitDepthNormalizeFromRoiPoints()
  {
    if (!hasValidRoiPoints(roi_points_))
    {
      return false;
    }

    std::vector<cv::Point2f> depth_points = roi_points_;
    if (depth_points.size() != 4)
    {
      const auto roi_bounds = roiBoundsFromSelection(depth_points);
      if (!roi_bounds.has_value())
      {
        return false;
      }
      depth_points = roiPointsFromBounds(*roi_bounds);
    }
    if (depth_points.size() != 4)
    {
      return false;
    }

    cv::Mat depth_for_fit;
    if (!getDepthFrameForPlaneFit(depth_for_fit))
    {
      return false;
    }

    DepthPlaneModel fitted_plane;
    if (!fitDepthPlaneFromCorners(depth_for_fit, depth_points, fitted_plane))
    {
      return false;
    }

    depth_plane_model_ = fitted_plane;
    depth_plane_roi_points_ = depth_points;
    depth_plane_from_bin_teach_ = false;
    pending_depth_plane_points_.clear();
    depth_plane_roi_bounds_ = roiBoundsFromSelection(depth_plane_roi_points_);
    if (!depth_plane_roi_bounds_.has_value())
    {
      depth_plane_roi_bounds_ = roiBoundsFromSelection(roi_points_);
    }
    markRuntimeSettingsDirty();
    return true;
  }

  void backTeachSequence()
  {
    switch (teach_stage_)
    {
      case TeachStage::kRoi:
      {
        setStatusMessage("Already at ROI stage");
        return;
      }

      case TeachStage::kColorMask:
      {
        setTeachStage(TeachStage::kRoi);
        view_mode_ = ViewMode::kRgb;
        setStatusMessage("Back: load or adjust Bin Teach ROI");
        return;
      }

      case TeachStage::kDepthNormalize:
      {
        clearDepthPlaneReference(false);
        setTeachStage(TeachStage::kColorMask);
        view_mode_ = ViewMode::kRgb;
        setStatusMessage("Back: tune RGB mask");
        return;
      }

      case TeachStage::kFlatLocate:
      {
        if (depth_plane_from_bin_teach_)
        {
          setTeachStage(TeachStage::kColorMask);
          view_mode_ = ViewMode::kRgb;
          setStatusMessage("Back: tune RGB mask");
          return;
        }
        clearDepthPlaneReference(false);
        setTeachStage(TeachStage::kDepthNormalize);
        view_mode_ = ViewMode::kDepth;
        setStatusMessage("Back: click 4 depth normalize points (0/4)");
        return;
      }

      case TeachStage::kPosePerception:
      default:
      {
        setTeachStage(TeachStage::kFlatLocate);
        view_mode_ = ViewMode::kBinarized;
        setStatusMessage("Back: tune depth window/hole/trim", 1.8);
        return;
      }
    }
  }

  void advanceTeachSequence()
  {
    switch (teach_stage_)
    {
      case TeachStage::kRoi:
      {
        if (!hasValidRoiPoints(roi_points_))
        {
          save_status_message_ = "ROI required";
          save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
          return;
        }
        setTeachStage(TeachStage::kColorMask);
        save_status_message_ = "Stage: tune RGB mask";
        save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
        return;
      }

      case TeachStage::kColorMask:
      {
        if (hasDepthPlaneReference())
        {
          setTeachStage(TeachStage::kFlatLocate);
          view_mode_ = ViewMode::kDepth;
          save_status_message_ = depth_plane_from_bin_teach_
            ? "Using Bin Teach depth plane. Stage: tune depth window/hole/trim"
            : "Stage: tune depth window/hole/trim";
          save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.8);
          markRuntimeSettingsDirty();
          return;
        }
        if (fitDepthNormalizeFromRoiPoints())
        {
          setTeachStage(TeachStage::kFlatLocate);
          view_mode_ = ViewMode::kDepth;
          save_status_message_ = "Using ROI points for depth normalize. Stage: tune depth window/hole/trim";
          save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.8);
          return;
        }
        clearDepthPlaneReference(false);
        setTeachStage(TeachStage::kDepthNormalize);
        view_mode_ = ViewMode::kDepth;
        save_status_message_ = "ROI depth fit unavailable; click 4 depth normalize points (0/4)";
        save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
        markRuntimeSettingsDirty();
        return;
      }

      case TeachStage::kDepthNormalize:
      {
        if (!hasDepthPlaneReference())
        {
          save_status_message_ = "Depth points " + std::to_string(depthNormalizePointCount()) + "/4";
          save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
          return;
        }
        setTeachStage(TeachStage::kFlatLocate);
        save_status_message_ = "Stage: tune depth window/hole/trim";
        save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
        return;
      }

      case TeachStage::kFlatLocate:
      {
        clearAllPoseReferenceSlots();
        pose_stage_status_ = "Click blob 1/2 to set anchor";
        setTeachStage(TeachStage::kPosePerception);
        view_mode_ = ViewMode::kBinarized;
        save_status_message_ = "Stage: select 2 blobs for grouped pose";
        save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.8);
        markRuntimeSettingsDirty();
        return;
      }

      case TeachStage::kPosePerception:
      default:
      {
        saveSettingsToFile();
        return;
      }
    }
  }

  void onMouse(int event, int x, int y, int /*flags*/)
  {
    const cv::Point ui_point(x, y);

    if (event == cv::EVENT_LBUTTONDOWN)
    {
      if (handleBinRoiDropdownMouseDown(ui_point))
      {
        return;
      }

      if (awaitingDepthNormalizePoints())
      {
        const auto image_point = windowPointToImagePoint(ui_point);
        if (image_point.has_value())
        {
          if (pending_depth_plane_points_.size() >= 4)
          {
            pending_depth_plane_points_.clear();
          }
          pending_depth_plane_points_.push_back(*image_point);
          markRuntimeSettingsDirty();
          if (pending_depth_plane_points_.size() < 4)
          {
            save_status_message_ = "Depth points " + std::to_string(pending_depth_plane_points_.size()) + "/4";
            save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.2);
            return;
          }
          fitDepthNormalizeFromSelectedPoints();
          return;
        }
      }

      if (teach_stage_ == TeachStage::kPosePerception)
      {
        for (int i = 0; i < kPoseReferenceSlotCount; ++i)
        {
          if (pose_reference_slot_rects_[static_cast<std::size_t>(i)].contains(ui_point))
          {
            selectPoseReferenceSlot(i);
            return;
          }
        }

        const auto image_point = windowPointToImagePoint(ui_point);
        if (image_point.has_value())
        {
          if (pose_blob_reference_clicks_px_.size() >= 2)
          {
            clearPoseReferenceSlot(active_pose_reference_slot_index_);
          }
          pose_blob_seed_point_px_ = cv::Point(
            static_cast<int>(std::round(image_point->x)),
            static_cast<int>(std::round(image_point->y)));
          pose_estimate_.reset();
          pose_stage_status_ = pose_blob_reference_clicks_px_.empty()
            ? "Blob 1/2 click received. Selecting anchor..."
            : "Blob 2/2 click received. Building pair reference...";
          save_status_message_ = pose_blob_reference_clicks_px_.empty()
            ? "Blob 1/2 click received"
            : "Blob 2/2 click received";
          save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.2);
          markRuntimeSettingsDirty();
          return;
        }
      }

      if (name_box_rect_.contains(ui_point))
      {
        name_edit_active_ = true;
        active_slider_index_ = -1;
        return;
      }

      name_edit_active_ = false;
      for (auto &button : buttons_)
      {
        if (!button.rect.contains(ui_point))
        {
          continue;
        }

        if (button.label == "View")
        {
          advanceViewMode();
          return;
        }

        if (button.state != nullptr)
        {
          *button.state = !*button.state;
          markRuntimeSettingsDirty();
          return;
        }
      }

      if (back_button_rect_.contains(ui_point))
      {
        if (canGoBack())
        {
          backTeachSequence();
        }
        else
        {
          setStatusMessage("Already at ROI stage");
        }
        return;
      }

      if (save_button_rect_.contains(ui_point))
      {
        advanceTeachSequence();
        return;
      }

      for (int i = 0; i < static_cast<int>(sliders_.size()); ++i)
      {
        if (!isSliderEnabled(sliders_[i]))
        {
          continue;
        }
        const cv::Rect hit_box(
          sliders_[i].track_rect.x,
          sliders_[i].track_rect.y - 16,
          sliders_[i].track_rect.width,
          sliders_[i].track_rect.height + 32);
        if (hit_box.contains(ui_point))
        {
          active_slider_index_ = i;
          updateSliderFromPoint(i, ui_point);
          return;
        }
      }
    }

    if (event == cv::EVENT_MOUSEMOVE && active_slider_index_ >= 0)
    {
      updateSliderFromPoint(active_slider_index_, ui_point);
    }

    if (event == cv::EVENT_LBUTTONUP)
    {
      active_slider_index_ = -1;
    }
  }

  void handleKeypress(int key)
  {
    if (key < 0)
    {
      return;
    }

    if (!name_edit_active_)
    {
      return;
    }

    if (key == 13 || key == 10 || key == 27)
    {
      name_edit_active_ = false;
      return;
    }

    if (key == 8 || key == 127)
    {
      if (!item_name_.empty())
      {
        item_name_.pop_back();
        markRuntimeSettingsDirty();
      }
      return;
    }

    if (item_name_.size() >= 32)
    {
      return;
    }

    if ((key >= 'a' && key <= 'z') ||
        (key >= 'A' && key <= 'Z') ||
        (key >= '0' && key <= '9') ||
        key == '_' || key == '-' || key == ' ')
    {
      item_name_.push_back(static_cast<char>(key));
      markRuntimeSettingsDirty();
    }
  }

  std::string sanitizeItemName() const
  {
    std::string trimmed = item_name_;
    while (!trimmed.empty() && trimmed.front() == ' ')
    {
      trimmed.erase(trimmed.begin());
    }
    while (!trimmed.empty() && trimmed.back() == ' ')
    {
      trimmed.pop_back();
    }
    return trimmed.empty() ? "item" : trimmed;
  }

  std::string buttonDisplayText(const UiButton &button) const
  {
    if (button.label == "Live")
    {
      return live_view_enabled_ ? "Live ON" : "Live OFF";
    }
    if (button.label == "View")
    {
      if (view_mode_ == ViewMode::kRgb)
      {
        return "View RGB";
      }
      if (view_mode_ == ViewMode::kDepth)
      {
        return "View Depth";
      }
      return "View Item";
    }
    if (button.label == "Overlay")
    {
      return overlay_enabled_ ? "Overlay ON" : "Overlay OFF";
    }
    if (button.label == "Focus")
    {
      return focus_black_mask_ ? "Focus Black" : "Focus White";
    }
    return button.label + ": " + ((button.state != nullptr && *button.state) ? "ON" : "OFF");
  }

  cv::Mat drawButtonBar(int width) const
  {
    cv::Mat bar(kVideoTopBarHeight, width, CV_8UC3, cv::Scalar(28, 30, 34));
    cv::line(bar, cv::Point(0, 56), cv::Point(width, 56), cv::Scalar(52, 56, 62), 1);

    const auto fit_text = [&](const std::string &text, int max_width, double scale, int thickness) -> std::string
    {
      if (max_width <= 0)
      {
        return "";
      }
      int baseline = 0;
      if (cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, scale, thickness, &baseline).width <= max_width)
      {
        return text;
      }
      std::string trimmed = text;
      const std::string ellipsis = "...";
      while (!trimmed.empty())
      {
        trimmed.pop_back();
        const std::string candidate = trimmed + ellipsis;
        if (cv::getTextSize(candidate, cv::FONT_HERSHEY_DUPLEX, scale, thickness, &baseline).width <= max_width)
        {
          return candidate;
        }
      }
      return "";
    };

    const auto draw_button = [&](const UiButton &button, bool enabled, const cv::Scalar &fill_on, const cv::Scalar &border_on)
    {
      cv::Rect rect = button.rect;
      rect.x -= kLeftPanelWidth;
      if (rect.x >= bar.cols || rect.x + rect.width <= 0)
      {
        return;
      }
      rect &= cv::Rect(0, 0, bar.cols, bar.rows);

      const cv::Scalar fill = enabled ? fill_on : cv::Scalar(60, 63, 68);
      const cv::Scalar border = enabled ? border_on : cv::Scalar(102, 106, 112);
      cv::rectangle(bar, rect, fill, cv::FILLED);
      cv::rectangle(bar, rect, border, 2);

      const std::string text = fit_text(buttonDisplayText(button), rect.width - 16, 0.50, 1);
      cv::putText(
        bar,
        text,
        cv::Point(rect.x + 8, rect.y + 24),
        cv::FONT_HERSHEY_DUPLEX,
        0.50,
        cv::Scalar(245, 245, 245),
        1,
        cv::LINE_AA);
    };

    for (const auto &button : buttons_)
    {
      bool enabled = button.state != nullptr ? *button.state : false;
      if (button.label == "View")
      {
        enabled = true;
      }
      cv::Scalar fill_on(70, 132, 82);
      cv::Scalar border_on(132, 215, 150);
      if (button.label == "Live")
      {
        fill_on = cv::Scalar(70, 132, 82);
        border_on = cv::Scalar(132, 215, 150);
      }
      else if (button.label == "View")
      {
        fill_on = cv::Scalar(69, 115, 148);
        border_on = cv::Scalar(130, 190, 232);
      }
      else if (button.label == "Overlay")
      {
        fill_on = cv::Scalar(72, 132, 145);
        border_on = cv::Scalar(140, 220, 235);
      }
      else if (button.label == "Focus")
      {
        fill_on = focus_black_mask_ ? cv::Scalar(65, 74, 160) : cv::Scalar(150, 150, 150);
        border_on = focus_black_mask_ ? cv::Scalar(132, 145, 255) : cv::Scalar(225, 225, 225);
      }
      draw_button(button, enabled, fill_on, border_on);
    }

    const cv::Rect status_rect(12, 66, std::max(120, width - 24), std::max(38, kVideoTopBarHeight - 76));
    cv::rectangle(bar, status_rect, cv::Scalar(35, 37, 41), cv::FILLED);
    cv::rectangle(bar, status_rect, cv::Scalar(72, 77, 84), 1);
    const bool show_save_status = this->now() < save_status_deadline_;
    const std::string roi_status = hasValidRoiPoints(roi_points_)
      ? "Bin Teach loaded"
      : "Load Bin Teach required";
    const std::string depth_status = depthNormalizeProgressText();
    const std::string runtime_status =
      show_save_status
      ? save_status_message_
      : ("Stage " + teachStageLabel() + " | " +
      (depth_status.empty() ? roi_status : depth_status) + " | View " + currentViewLabel());
    const std::string status_label = "Teach Status";
    int status_label_baseline = 0;
    const int status_label_width = cv::getTextSize(
      status_label,
      cv::FONT_HERSHEY_DUPLEX,
      0.43,
      1,
      &status_label_baseline).width;
    const int status_value_x = status_rect.x + 10 + status_label_width + 14;
    cv::putText(
      bar,
      status_label,
      cv::Point(status_rect.x + 10, status_rect.y + 20),
      cv::FONT_HERSHEY_DUPLEX,
      0.43,
      cv::Scalar(205, 210, 216),
      1,
      cv::LINE_AA);
    cv::putText(
      bar,
      fit_text(runtime_status, status_rect.x + status_rect.width - status_value_x - 10, 0.44, 1),
      cv::Point(status_value_x, status_rect.y + 20),
      cv::FONT_HERSHEY_DUPLEX,
      0.44,
      cv::Scalar(194, 200, 206),
      1,
      cv::LINE_AA);
    return bar;
  }

  cv::Mat drawLeftPanel(int panel_height) const
  {
    cv::Mat panel(panel_height, kLeftPanelWidth, CV_8UC3, cv::Scalar(30, 32, 36));
    cv::line(panel, cv::Point(panel.cols - 1, 0), cv::Point(panel.cols - 1, panel.rows), cv::Scalar(66, 71, 78), 1);

    const auto fit_text = [&](const std::string &text, int max_width, double scale, int thickness) -> std::string
    {
      if (max_width <= 0)
      {
        return "";
      }
      int baseline = 0;
      if (cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, scale, thickness, &baseline).width <= max_width)
      {
        return text;
      }
      std::string trimmed = text;
      const std::string ellipsis = "...";
      while (!trimmed.empty())
      {
        trimmed.pop_back();
        const std::string candidate = trimmed + ellipsis;
        if (cv::getTextSize(candidate, cv::FONT_HERSHEY_DUPLEX, scale, thickness, &baseline).width <= max_width)
        {
          return candidate;
        }
      }
      return "";
    };

    const auto draw_card = [&](const cv::Rect &rect, const std::string &title)
    {
      cv::rectangle(panel, rect, cv::Scalar(38, 41, 46), cv::FILLED);
      cv::rectangle(panel, rect, cv::Scalar(72, 77, 84), 1);
      const cv::Rect header(rect.x, rect.y, rect.width, 24);
      cv::rectangle(panel, header, cv::Scalar(46, 50, 56), cv::FILLED);
      cv::line(panel, cv::Point(rect.x, rect.y + 24), cv::Point(rect.x + rect.width, rect.y + 24), cv::Scalar(72, 77, 84), 1);
      cv::putText(
        panel,
        title,
        cv::Point(rect.x + 10, rect.y + 17),
        cv::FONT_HERSHEY_DUPLEX,
        0.45,
        cv::Scalar(220, 224, 230),
        1,
        cv::LINE_AA);
    };

    cv::putText(
      panel,
      "Item Teach Console",
      cv::Point(24, 25),
      cv::FONT_HERSHEY_DUPLEX,
      0.62,
      cv::Scalar(236, 239, 244),
      1,
      cv::LINE_AA);

    const cv::Rect setup_card(12, 34, kLeftPanelWidth - 24, 236);
    draw_card(setup_card, "Item Setup");
    cv::putText(
      panel,
      "Item Name",
      cv::Point(24, 84),
      cv::FONT_HERSHEY_DUPLEX,
      0.44,
      cv::Scalar(188, 194, 201),
      1,
      cv::LINE_AA);

    cv::rectangle(panel, name_box_rect_, cv::Scalar(47, 50, 56), cv::FILLED);
    cv::rectangle(
      panel,
      name_box_rect_,
      name_edit_active_ ? cv::Scalar(134, 205, 236) : cv::Scalar(91, 98, 108),
      2);
    const std::string item_name_text = sanitizeItemName();
    cv::putText(
      panel,
      fit_text(item_name_text, name_box_rect_.width - 20, 0.60, 1),
      cv::Point(name_box_rect_.x + 10, name_box_rect_.y + 27),
      cv::FONT_HERSHEY_DUPLEX,
      0.60,
      cv::Scalar(240, 242, 246),
      1,
      cv::LINE_AA);
    if (name_edit_active_)
    {
      int baseline = 0;
      const int text_width = cv::getTextSize(
        fit_text(item_name_text, name_box_rect_.width - 20, 0.60, 1),
        cv::FONT_HERSHEY_DUPLEX,
        0.60,
        1,
        &baseline).width;
      cv::line(
        panel,
        cv::Point(name_box_rect_.x + 12 + text_width, name_box_rect_.y + 8),
        cv::Point(name_box_rect_.x + 12 + text_width, name_box_rect_.y + 31),
        cv::Scalar(134, 205, 236),
        2);
    }

    cv::putText(
      panel,
      "Load Bin Teach",
      cv::Point(24, 150),
      cv::FONT_HERSHEY_DUPLEX,
      0.44,
      cv::Scalar(188, 194, 201),
      1,
      cv::LINE_AA);
    const cv::Scalar dropdown_fill = bin_roi_entries_.empty()
      ? cv::Scalar(43, 46, 51)
      : cv::Scalar(47, 50, 56);
    const cv::Scalar dropdown_border = bin_roi_dropdown_open_
      ? cv::Scalar(134, 205, 236)
      : cv::Scalar(91, 98, 108);
    cv::rectangle(panel, bin_roi_dropdown_rect_, dropdown_fill, cv::FILLED);
    cv::rectangle(panel, bin_roi_dropdown_rect_, dropdown_border, 2);
    cv::putText(
      panel,
      fit_text(binRoiDropdownText(), bin_roi_dropdown_rect_.width - 46, 0.48, 1),
      cv::Point(bin_roi_dropdown_rect_.x + 10, bin_roi_dropdown_rect_.y + 23),
      cv::FONT_HERSHEY_DUPLEX,
      0.48,
      bin_roi_entries_.empty() ? cv::Scalar(150, 154, 162) : cv::Scalar(235, 238, 242),
      1,
      cv::LINE_AA);
    const cv::Point arrow_center(
      bin_roi_dropdown_rect_.x + bin_roi_dropdown_rect_.width - 21,
      bin_roi_dropdown_rect_.y + bin_roi_dropdown_rect_.height / 2 + 1);
    const std::vector<cv::Point> arrow_points = bin_roi_dropdown_open_
      ? std::vector<cv::Point>{
        cv::Point(arrow_center.x - 6, arrow_center.y + 4),
        cv::Point(arrow_center.x + 6, arrow_center.y + 4),
        cv::Point(arrow_center.x, arrow_center.y - 5)}
      : std::vector<cv::Point>{
        cv::Point(arrow_center.x - 6, arrow_center.y - 4),
        cv::Point(arrow_center.x + 6, arrow_center.y - 4),
        cv::Point(arrow_center.x, arrow_center.y + 5)};
    cv::fillConvexPoly(panel, arrow_points, cv::Scalar(205, 210, 216), cv::LINE_AA);

    const bool delete_bin_ready = canDeleteSelectedBinTeach();
    const bool delete_bin_armed = deleteSelectedBinTeachArmed();
    const cv::Scalar delete_fill = delete_bin_ready ? cv::Scalar(82, 54, 58) : cv::Scalar(52, 54, 58);
    const cv::Scalar delete_border = delete_bin_ready ? cv::Scalar(186, 96, 102) : cv::Scalar(92, 96, 102);
    cv::rectangle(panel, delete_bin_roi_button_rect_, delete_fill, cv::FILLED);
    cv::rectangle(panel, delete_bin_roi_button_rect_, delete_border, 2);
    cv::putText(
      panel,
      delete_bin_armed ? "Confirm" : "Delete",
      cv::Point(delete_bin_roi_button_rect_.x + (delete_bin_armed ? 10 : 16), delete_bin_roi_button_rect_.y + 23),
      cv::FONT_HERSHEY_DUPLEX,
      delete_bin_armed ? 0.43 : 0.46,
      delete_bin_ready ? cv::Scalar(246, 226, 226) : cv::Scalar(160, 164, 170),
      1,
      cv::LINE_AA);

    const bool back_ready = canGoBack();
    const cv::Scalar back_fill = back_ready ? cv::Scalar(69, 115, 148) : cv::Scalar(62, 65, 70);
    const cv::Scalar back_border = back_ready ? cv::Scalar(130, 190, 232) : cv::Scalar(102, 106, 112);
    cv::rectangle(panel, back_button_rect_, back_fill, cv::FILLED);
    cv::rectangle(panel, back_button_rect_, back_border, 2);
    cv::putText(
      panel,
      "Back",
      cv::Point(back_button_rect_.x + 14, back_button_rect_.y + 25),
      cv::FONT_HERSHEY_DUPLEX,
      0.50,
      back_ready ? cv::Scalar(245, 245, 245) : cv::Scalar(170, 174, 180),
      1,
      cv::LINE_AA);

    const bool save_ready = actionButtonReady();
    const cv::Scalar save_fill = save_ready ? cv::Scalar(70, 132, 82) : cv::Scalar(66, 70, 76);
    const cv::Scalar save_border = save_ready ? cv::Scalar(132, 215, 150) : cv::Scalar(102, 106, 112);
    cv::rectangle(panel, save_button_rect_, save_fill, cv::FILLED);
    cv::rectangle(panel, save_button_rect_, save_border, 2);
    cv::putText(
      panel,
      saveActionButtonLabel(),
      cv::Point(save_button_rect_.x + 12, save_button_rect_.y + 25),
      cv::FONT_HERSHEY_DUPLEX,
      0.50,
      cv::Scalar(245, 245, 245),
      1,
      cv::LINE_AA);

    const bool show_save_status = this->now() < save_status_deadline_;
    if (show_save_status && teach_stage_ != TeachStage::kPosePerception)
    {
      cv::putText(
        panel,
        fit_text(
          save_status_message_,
          setup_card.x + setup_card.width - (save_button_rect_.x + save_button_rect_.width + 22),
          0.43,
          1),
        cv::Point(save_button_rect_.x + save_button_rect_.width + 12, save_button_rect_.y + 24),
        cv::FONT_HERSHEY_DUPLEX,
        0.43,
        save_status_message_.rfind("Item saved", 0) == 0 ? cv::Scalar(150, 232, 165) : cv::Scalar(236, 180, 126),
        1,
        cv::LINE_AA);
    }
    if (teach_stage_ == TeachStage::kPosePerception)
    {
      for (int i = 0; i < kPoseReferenceSlotCount; ++i)
      {
        const cv::Rect &slot_rect = pose_reference_slot_rects_[static_cast<std::size_t>(i)];
        const bool active_slot = i == active_pose_reference_slot_index_;
        const bool filled_slot = poseReferenceForSlot(i).has_value();
        const cv::Scalar fill = active_slot
          ? cv::Scalar(78, 148, 92)
          : (filled_slot ? cv::Scalar(58, 104, 72) : cv::Scalar(54, 57, 64));
        const cv::Scalar border = active_slot
          ? cv::Scalar(160, 235, 176)
          : (filled_slot ? cv::Scalar(118, 200, 136) : cv::Scalar(100, 106, 116));
        cv::rectangle(panel, slot_rect, fill, cv::FILLED);
        cv::rectangle(panel, slot_rect, border, active_slot ? 3 : 2);
        cv::putText(
          panel,
          std::to_string(i + 1),
          cv::Point(slot_rect.x + 13, slot_rect.y + 27),
          cv::FONT_HERSHEY_DUPLEX,
          0.72,
          cv::Scalar(246, 248, 250),
          1,
          cv::LINE_AA);
        if (filled_slot)
        {
          cv::circle(
            panel,
            cv::Point(slot_rect.x + slot_rect.width - 8, slot_rect.y + 8),
            4,
            cv::Scalar(60, 235, 130),
            cv::FILLED,
            cv::LINE_AA);
        }
      }
      const std::string refs_text =
        "Pose refs saved: " + std::to_string(filledPoseReferenceSlotCount()) + "/4";
      cv::putText(
        panel,
        show_save_status ? fit_text(save_status_message_, setup_card.width - 24, 0.40, 1) : refs_text,
        cv::Point(24, 258),
        cv::FONT_HERSHEY_DUPLEX,
        0.40,
        show_save_status && save_status_message_.rfind("Item saved", 0) != 0
          ? cv::Scalar(236, 180, 126)
          : cv::Scalar(188, 218, 196),
        1,
        cv::LINE_AA);
    }

    const int slider_card_y = std::max(
      200,
      sliders_.empty() ? 200 : (sliders_.front().track_rect.y - 54));
    const int available_slider_h = std::max(180, panel.rows - slider_card_y - 12);
    bool has_visible_slider = false;
    int last_visible_slider_bottom = slider_card_y + 180;
    for (const auto &slider : sliders_)
    {
      if (!isSliderVisible(slider))
      {
        continue;
      }
      has_visible_slider = true;
      last_visible_slider_bottom = std::max(last_visible_slider_bottom, slider.track_rect.y + 34);
    }
    if (has_visible_slider)
    {
      const int target_slider_h = last_visible_slider_bottom - slider_card_y;
      const int slider_card_h = std::clamp(target_slider_h, 180, available_slider_h);
      const cv::Rect slider_card(12, slider_card_y, kLeftPanelWidth - 24, slider_card_h);
      draw_card(slider_card, "Detection Tuning");

      for (const auto &slider : sliders_)
      {
        if (!isSliderVisible(slider))
        {
          continue;
        }
        const int value = slider.value ? *slider.value : 0;
        const bool slider_enabled = isSliderEnabled(slider);
        cv::Scalar accent(120, 200, 255);
        if (slider.label == kRedTrackbar)
        {
          accent = cv::Scalar(70, 92, 240);
        }
        else if (slider.label == kGreenTrackbar)
        {
          accent = cv::Scalar(90, 205, 110);
        }
	        else if (slider.label == kBlueTrackbar)
	        {
	          accent = cv::Scalar(240, 140, 75);
	        }
	        else if (slider.label == kColorExposureTrackbar)
	        {
	          accent = value <= 0 ? cv::Scalar(130, 170, 200) : cv::Scalar(108, 206, 224);
	        }
        if (!slider_enabled)
        {
          accent = cv::Scalar(104, 108, 116);
        }

        cv::putText(
          panel,
          slider.label,
          cv::Point(slider.track_rect.x, slider.track_rect.y - 10),
          cv::FONT_HERSHEY_DUPLEX,
          0.45,
          slider_enabled ? cv::Scalar(214, 218, 224) : cv::Scalar(130, 135, 142),
          1,
          cv::LINE_AA);
        const std::string slider_value_text = sliderValueText(slider, value);
        cv::putText(
          panel,
          slider_value_text,
          cv::Point(
            slider.track_rect.x + slider.track_rect.width -
            sliderValueTextOffsetX(slider),
            slider.track_rect.y - 10),
          cv::FONT_HERSHEY_DUPLEX,
          0.43,
          slider_enabled ? cv::Scalar(186, 191, 198) : cv::Scalar(122, 126, 133),
          1,
          cv::LINE_AA);
        const cv::Scalar track_fill = slider_enabled ? cv::Scalar(67, 72, 78) : cv::Scalar(54, 57, 62);
        const cv::Scalar track_border = slider_enabled ? cv::Scalar(92, 99, 107) : cv::Scalar(76, 81, 88);
        cv::rectangle(panel, slider.track_rect, track_fill, cv::FILLED);
        cv::rectangle(panel, slider.track_rect, track_border, 1);
        const double t = static_cast<double>(value - slider.min_value) / std::max(1, slider.max_value - slider.min_value);
        const int knob_x = slider.track_rect.x + static_cast<int>(std::round(t * slider.track_rect.width));
        const int knob_y = slider.track_rect.y + slider.track_rect.height / 2;
        cv::line(
          panel,
          cv::Point(slider.track_rect.x + 1, knob_y),
          cv::Point(slider.track_rect.x + slider.track_rect.width - 1, knob_y),
          accent,
          2,
          cv::LINE_AA);
        const cv::Scalar knob_fill = slider_enabled ? cv::Scalar(245, 245, 245) : cv::Scalar(170, 174, 180);
        const cv::Scalar knob_border = slider_enabled ? cv::Scalar(96, 100, 106) : cv::Scalar(112, 116, 123);
        cv::circle(panel, cv::Point(knob_x, knob_y), 9, knob_fill, -1, cv::LINE_AA);
        cv::circle(panel, cv::Point(knob_x, knob_y), 9, knob_border, 1, cv::LINE_AA);
      }
    }

    if (bin_roi_dropdown_open_)
    {
      const int visible_count = visibleBinRoiOptionCount();
      for (int i = 0; i < visible_count; ++i)
      {
        const cv::Rect option_rect = binRoiOptionRect(i) & cv::Rect(0, 0, panel.cols, panel.rows);
        if (option_rect.empty())
        {
          continue;
        }
        const bool selected = i == selected_bin_roi_index_;
        cv::rectangle(
          panel,
          option_rect,
          selected ? cv::Scalar(60, 93, 118) : cv::Scalar(43, 47, 53),
          cv::FILLED);
        cv::rectangle(
          panel,
          option_rect,
          selected ? cv::Scalar(134, 205, 236) : cv::Scalar(77, 83, 92),
          1);
        cv::putText(
          panel,
          fit_text(
            bin_roi_entries_[static_cast<std::size_t>(i)].label,
            option_rect.width - 18,
            0.43,
            1),
          cv::Point(option_rect.x + 9, option_rect.y + 21),
          cv::FONT_HERSHEY_DUPLEX,
          0.43,
          cv::Scalar(236, 239, 244),
          1,
          cv::LINE_AA);
      }
      if (static_cast<int>(bin_roi_entries_.size()) > visible_count && visible_count > 0)
      {
        const cv::Rect last_rect = binRoiOptionRect(visible_count - 1) & cv::Rect(0, 0, panel.cols, panel.rows);
        cv::putText(
          panel,
          "...",
          cv::Point(last_rect.x + last_rect.width - 30, last_rect.y + 21),
          cv::FONT_HERSHEY_DUPLEX,
          0.43,
          cv::Scalar(180, 185, 192),
          1,
          cv::LINE_AA);
      }
    }

    return panel;
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

    cv::Size frame_size;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = color_cv->image.clone();
      latest_header_ = msg->header;
      frame_size = latest_frame_.size();
    }
    tryApplyPendingBinTeachRoi(frame_size);
  }

  void depthCallback(const ImageMsg::ConstSharedPtr msg)
  {
    cv::Mat depth_m;
    if (!convertDepthToMeters(msg, depth_m))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Depth conversion failed.");
      return;
    }

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_depth_ = depth_m;
  }

  void cameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_camera_info_ = msg;
  }

  void jointStateCallback(const JointStateMsg::ConstSharedPtr msg)
  {
    if (msg->position.empty())
    {
      return;
    }

    constexpr double kRadToDeg = 57.29577951308232;
    std::array<double, 6> joints_deg {};
    std::array<bool, 6> has_named_joint {false, false, false, false, false, false};

    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
      const std::string &name = msg->name[i];
      int index = -1;
      if (name == "joint1")
      {
        index = 0;
      }
      else if (name == "joint2")
      {
        index = 1;
      }
      else if (name == "joint3")
      {
        index = 2;
      }
      else if (name == "joint4")
      {
        index = 3;
      }
      else if (name == "joint5")
      {
        index = 4;
      }
      else if (name == "joint6")
      {
        index = 5;
      }

      if (index >= 0)
      {
        joints_deg[static_cast<std::size_t>(index)] = msg->position[i] * kRadToDeg;
        has_named_joint[static_cast<std::size_t>(index)] = true;
      }
    }

    bool valid_sample =
      std::all_of(has_named_joint.begin(), has_named_joint.end(), [](const bool value) { return value; });
    if (!valid_sample && msg->position.size() >= 6)
    {
      for (std::size_t i = 0; i < 6; ++i)
      {
        joints_deg[i] = msg->position[i] * kRadToDeg;
      }
      valid_sample = true;
    }

    if (!valid_sample)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_positions_deg_ = joints_deg;
    has_joint_positions_ = true;
  }

  cv::Mat buildNoCameraTopicsPlaceholder()
  {
    cv::Mat image(kPreviewCanvasHeight, kPreviewCanvasWidth, CV_8UC3, cv::Scalar(18, 20, 24));
    cv::putText(
      image,
      "no camera topics...",
      cv::Point(44, 96),
      cv::FONT_HERSHEY_SIMPLEX,
      1.15,
      cv::Scalar(0, 210, 255),
      3,
      cv::LINE_AA);

    const std::array<std::string, 3> status_lines = {
      "color: " + color_topic_ + "  publishers=" + std::to_string(count_publishers(color_topic_)),
      "depth: " + depth_topic_ + "  publishers=" + std::to_string(count_publishers(depth_topic_)),
      "info:  " + camera_info_topic_ + "  publishers=" + std::to_string(count_publishers(camera_info_topic_))};

    int y = 158;
    for (const auto &line : status_lines)
    {
      cv::putText(
        image,
        line,
        cv::Point(48, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.60,
        cv::Scalar(225, 230, 235),
        2,
        cv::LINE_AA);
      y += 40;
    }
    return image;
  }

  void renderNoCameraTopicsFrame()
  {
    const cv::Mat preview_canvas = buildPreviewCanvas(buildNoCameraTopicsPlaceholder());
    const cv::Mat video_top = drawButtonBar(preview_canvas.cols);
    cv::Mat right_panel;
    cv::vconcat(video_top, preview_canvas, right_panel);
    const int minimum_panel_height = minimumLeftPanelHeight();
    if (right_panel.rows < minimum_panel_height)
    {
      cv::copyMakeBorder(
        right_panel,
        right_panel,
        0,
        minimum_panel_height - right_panel.rows,
        0,
        0,
        cv::BORDER_CONSTANT,
        cv::Scalar(30, 32, 36));
    }
    const cv::Mat left_panel = drawLeftPanel(right_panel.rows);

    cv::Mat combined;
    cv::hconcat(left_panel, right_panel, combined);
    const cv::Size window_size(combined.cols, combined.rows);
    if (window_size != rendered_window_size_)
    {
      cv::resizeWindow(kWindowName, window_size.width, window_size.height);
      rendered_window_size_ = window_size;
    }
    cv::imshow(kWindowName, combined);
    handleKeypress(cv::waitKey(1));
    saveRuntimeSettingsToFile(false);

    if (publish_overlay_)
    {
      cv_bridge::CvImage overlay_image;
      overlay_image.header.stamp = now();
      overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_image.image = combined;
      overlay_pub_->publish(*overlay_image.toImageMsg());
    }
  }

  void renderFrame()
  {
    cv::Mat current_frame;
    cv::Mat current_depth;
    CameraInfoMsg::ConstSharedPtr current_camera_info;
    std_msgs::msg::Header current_header;
    bool missing_camera_frame = false;

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.empty() || latest_depth_.empty() || !latest_camera_info_)
      {
        missing_camera_frame = true;
      }
      else if (!live_view_enabled_)
      {
        if (!freeze_latched_ || frozen_frame_.empty())
        {
          frozen_frame_ = latest_frame_.clone();
          frozen_depth_ = latest_depth_.clone();
          frozen_header_ = latest_header_;
          frozen_camera_info_ = latest_camera_info_;
          freeze_latched_ = true;
        }
        current_frame = frozen_frame_.clone();
        current_depth = frozen_depth_.clone();
        current_header = frozen_header_;
        current_camera_info = frozen_camera_info_;
      }
      else
      {
        freeze_latched_ = false;
        current_frame = latest_frame_.clone();
        current_depth = latest_depth_.clone();
        current_header = latest_header_;
        current_camera_info = latest_camera_info_;
      }
    }
    if (missing_camera_frame)
    {
      renderNoCameraTopicsFrame();
      return;
    }

    const bool roi_ready = hasValidRoiPoints(roi_points_);

    cv::Mat roi_mask;
    if (roi_ready)
    {
      roi_mask = buildRoiMask(current_frame.size(), roi_points_);
    }
    const cv::Mat color_mask = buildRgbMask(
      current_frame,
      red_threshold_,
      green_threshold_,
      blue_threshold_,
      rgb_hole_fill_sensitivity_,
      rgb_mask_dilate_px_,
      focus_black_mask_);

    cv::Mat color_detection_mask = color_mask.clone();
    if (roi_ready)
    {
      cv::bitwise_and(color_detection_mask, roi_mask, color_detection_mask);
    }

    const cv::Mat color_retain_mask = color_detection_mask.clone();
    const cv::Mat rgb_focus_display_mask = roi_ready ? color_detection_mask : color_mask;

    if (detection_use_depth_)
    {
      current_depth = fillInvalidDepthNearby(current_depth, depth_null_fill_sensitivity_);
    }

    bool normalized_depth_ready = false;
    cv::Mat depth_residual_m;
    cv::Mat depth_residual_display_m;
    cv::Mat finite_depth_residual_mask = cv::Mat::zeros(current_depth.size(), CV_8UC1);
    cv::Mat depth_window_mask = cv::Mat::zeros(current_depth.size(), CV_8UC1);
    cv::Mat depth_retain_mask = cv::Mat::zeros(current_depth.size(), CV_8UC1);
    DepthWindowPeakInfo depth_window_peak_info;
    if (hasDepthPlaneReference())
    {
      depth_residual_m = computeDepthPlaneResidual(current_depth, depth_plane_model_);
      normalized_depth_ready = !depth_residual_m.empty() && depth_residual_m.type() == CV_32FC1;
      if (normalized_depth_ready)
      {
        depth_residual_display_m = depth_residual_m.clone();
      }
    }

    if (normalized_depth_ready)
    {
      finite_depth_residual_mask = buildFiniteDepthResidualMask(depth_residual_m);
      if (roi_ready)
      {
        cv::bitwise_and(finite_depth_residual_mask, roi_mask, finite_depth_residual_mask);
      }
      cv::Mat peak_candidate_mask;
      cv::bitwise_and(finite_depth_residual_mask, color_detection_mask, peak_candidate_mask);
      depth_window_mask = applyDepthTopWindowMask(
        depth_residual_m,
        peak_candidate_mask,
        depth_window_mm_,
        &depth_window_peak_info);
      cv::Mat windowed_depth_residual = depth_residual_m.clone();
      windowed_depth_residual.setTo(
        cv::Scalar(std::numeric_limits<float>::quiet_NaN()),
        depth_window_mask == 0);
      depth_retain_mask = buildFiniteDepthMask(windowed_depth_residual);
    }

    // Depth used for 3D item pose must be post-window metric depth only.
    // Keep it non-normalized and exclude any non-positive/null values.
    cv::Mat depth_for_pose_m = current_depth;
    if (teach_stage_ >= TeachStage::kDepthNormalize && normalized_depth_ready)
    {
      cv::Mat windowed_depth_for_pose = current_depth.clone();
      windowed_depth_for_pose.setTo(
        cv::Scalar(std::numeric_limits<float>::quiet_NaN()),
        depth_window_mask == 0);
      const cv::Mat valid_depth_mask = buildPositiveFiniteDepthMask(windowed_depth_for_pose);
      windowed_depth_for_pose.setTo(
        cv::Scalar(std::numeric_limits<float>::quiet_NaN()),
        valid_depth_mask == 0);
      depth_for_pose_m = windowed_depth_for_pose;
    }

    cv::Mat combined_mask = color_retain_mask.clone();
    if (teach_stage_ >= TeachStage::kDepthNormalize && normalized_depth_ready)
    {
      cv::bitwise_and(combined_mask, depth_retain_mask, combined_mask);
      combined_mask = fillEnclosedMaskHoles(combined_mask, depth_hole_fill_sensitivity_);
      const int effective_depth_trim_px = computeAdaptiveDepthTrimPx(
        depth_trim_px_,
        depth_window_peak_info,
        adaptive_depth_trim_max_factor_tenths_,
        adaptive_depth_trim_max_height_mm_);
      combined_mask = trimMaskInward(combined_mask, effective_depth_trim_px);
    }

    cv::Mat flat_mask = combined_mask.clone();
    std::optional<BinarizedPoseEstimate2D> pose_estimate;
    std::optional<BinarizedPoseEstimate2D> pose_estimate_overlay;
    std::string pose_status_text;
    if (teach_stage_ >= TeachStage::kPosePerception)
    {
      if (pose_blob_seed_point_px_.has_value())
      {
        const cv::Point clicked_seed_px = *pose_blob_seed_point_px_;
        pose_blob_seed_point_px_.reset();
        std::string selection_status;
        if (pose_blob_reference_clicks_px_.empty())
        {
          const auto selected_anchor_component = selectPoseBlobComponentFromMask(
            flat_mask,
            clicked_seed_px,
            &selection_status);
          if (selected_anchor_component.has_value())
          {
            pose_blob_reference_clicks_px_ = {clicked_seed_px};
            if (const auto preview_pose = buildBlobPoseFromComponent(*selected_anchor_component);
              preview_pose.has_value())
            {
              if (const auto single_reference = buildPoseBlobReferenceFromBlobPose(*preview_pose);
                single_reference.has_value())
              {
                pose_blob_reference_ = *single_reference;
                persistActivePoseReferenceSlot();
              }
              else
              {
                pose_blob_reference_.reset();
                persistActivePoseReferenceSlot();
              }
              BinarizedPoseEstimate2D preview_estimate;
              preview_estimate.matched_blob_count = 1;
              preview_estimate.blob_poses.push_back(*preview_pose);
              pose_estimate_overlay = preview_estimate;
            }
            else
            {
              pose_blob_reference_.reset();
              persistActivePoseReferenceSlot();
            }
            pose_status_text = "Blob 1/2 selected. Save now for single blob, or click blob 2/2 for pair";
          }
          else if (!selection_status.empty())
          {
            pose_status_text = selection_status;
          }
        }
        else
        {
          std::string anchor_status;
          const auto selected_anchor_component = selectPoseBlobComponentFromMask(
            flat_mask,
            pose_blob_reference_clicks_px_.front(),
            &anchor_status);
          const auto selected_companion_component = selectPoseBlobComponentFromMask(
            flat_mask,
            clicked_seed_px,
            &selection_status);
          if (!selected_anchor_component.has_value())
          {
            pose_blob_reference_clicks_px_.clear();
            pose_blob_reference_.reset();
            persistActivePoseReferenceSlot();
            pose_status_text = anchor_status.empty()
              ? "Anchor blob disappeared. Click blob 1/2 again"
              : "Anchor blob disappeared. " + anchor_status;
          }
          else if (!selected_companion_component.has_value())
          {
            pose_status_text = selection_status.empty()
              ? "Blob 2/2 is invalid. Save now for single blob, or click another blob for pair"
              : selection_status + " Save now for single blob, or click another blob for pair";
          }
          else if (selected_anchor_component->label == selected_companion_component->label)
          {
            pose_blob_reference_clicks_px_.resize(1);
            pose_status_text = "Blob 2/2 must be different from blob 1/2. Save now for single blob, or click another blob for pair";
          }
          else
          {
            const auto pair_pose = buildPairBlobPoseFromComponents(
              *selected_anchor_component,
              *selected_companion_component);
            if (!pair_pose.has_value())
            {
              pose_status_text = "Selected blobs could not form a grouped pose. Save now for single blob, or click another blob for pair";
            }
            else if (const auto pair_reference = buildPoseBlobReferenceFromBlobPose(*pair_pose);
              pair_reference.has_value())
            {
              pose_blob_reference_ = *pair_reference;
              pose_blob_reference_clicks_px_ = {
                pose_blob_reference_clicks_px_.front(),
                clicked_seed_px};
              persistActivePoseReferenceSlot();
              BinarizedPoseEstimate2D preview_estimate;
              preview_estimate.matched_blob_count = 1;
              preview_estimate.blob_poses.push_back(*pair_pose);
              pose_estimate_overlay = preview_estimate;
              pose_status_text = "2-blob reference set. Matching grouped poses...";
            }
            else
            {
              pose_status_text = "Selected 2-blob group is invalid. Save now for single blob, or click another blob for pair";
            }
          }
        }
      }

      if (pose_blob_reference_.has_value())
      {
        pose_estimate = estimatePoseFromBinarizedMask(
          flat_mask,
          *pose_blob_reference_,
          blob_tolerance_percent_,
          &pose_status_text);
        const bool preserving_single_anchor_preview =
          pose_blob_reference_->mode == PoseTemplateMode2D::kSingle &&
          pose_blob_reference_clicks_px_.size() == 1 &&
          pose_estimate_overlay.has_value();
        if (!preserving_single_anchor_preview)
        {
          pose_estimate_overlay = pose_estimate;
        }
        if (pose_estimate.has_value())
        {
          const auto &selected_group_pose = pose_estimate->blob_poses.front();
          const float long_side_px = std::max(selected_group_pose.x_length_px, selected_group_pose.z_length_px);
          const float short_side_px = std::min(selected_group_pose.x_length_px, selected_group_pose.z_length_px);
          if (pose_blob_reference_->mode == PoseTemplateMode2D::kPair)
          {
            pose_status_text = pose_estimate->matched_blob_count > 1
              ? cv::format(
                  "2-blob groups ready: %d | anchor preserved | L %.1f px H %.1f px",
                  pose_estimate->matched_blob_count,
                  long_side_px,
                  short_side_px)
              : cv::format(
                  "2-blob group ready | anchor preserved | L %.1f px H %.1f px",
                  long_side_px,
                  short_side_px);
          }
          else if (pose_blob_reference_clicks_px_.size() == 1)
          {
            pose_status_text = pose_estimate->matched_blob_count > 1
              ? cv::format(
                  "Single-blob ready: %d matches | save now, or click blob 2/2 for pair | L %.1f px H %.1f px",
                  pose_estimate->matched_blob_count,
                  long_side_px,
                  short_side_px)
              : cv::format(
                  "Single-blob ready | save now, or click blob 2/2 for pair | L %.1f px H %.1f px",
                  long_side_px,
                  short_side_px);
          }
          else
          {
            pose_status_text = pose_estimate->matched_blob_count > 1
              ? cv::format(
                  "Single-blob matches: %d | L %.1f px H %.1f px",
                  pose_estimate->matched_blob_count,
                  long_side_px,
                  short_side_px)
              : cv::format(
                  "Single-blob ready | L %.1f px H %.1f px",
                  long_side_px,
                  short_side_px);
          }
        }
        else if (
          pose_blob_reference_->mode == PoseTemplateMode2D::kPair &&
          !pose_blob_reference_clicks_px_.empty())
        {
          std::string anchor_status;
          const auto selected_anchor_component = selectPoseBlobComponentFromMask(
            flat_mask,
            pose_blob_reference_clicks_px_.front(),
            &anchor_status);
          if (selected_anchor_component.has_value())
          {
            if (const auto preview_pose = buildBlobPoseFromComponent(*selected_anchor_component);
              preview_pose.has_value())
            {
              BinarizedPoseEstimate2D preview_estimate;
              preview_estimate.matched_blob_count = 1;
              preview_estimate.blob_poses.push_back(*preview_pose);
              pose_estimate_overlay = preview_estimate;
              pose_status_text = pose_status_text.empty()
                ? "2-blob pair not ready. Previewing blob 1/2 only"
                : pose_status_text + " | previewing blob 1/2 only";
            }
          }
          else if (pose_status_text.empty() && !anchor_status.empty())
          {
            pose_status_text = "Blob 1/2 no longer visible. Reselect anchor blob";
          }
        }
      }
      else if (pose_blob_reference_clicks_px_.size() == 1)
      {
        std::string anchor_status;
        const auto selected_anchor_component = selectPoseBlobComponentFromMask(
          flat_mask,
          pose_blob_reference_clicks_px_.front(),
          &anchor_status);
        if (selected_anchor_component.has_value())
        {
          if (const auto preview_pose = buildBlobPoseFromComponent(*selected_anchor_component);
            preview_pose.has_value())
          {
            BinarizedPoseEstimate2D preview_estimate;
            preview_estimate.matched_blob_count = 1;
            preview_estimate.blob_poses.push_back(*preview_pose);
            pose_estimate_overlay = preview_estimate;
          }
          if (pose_status_text.empty())
          {
            pose_status_text = "Blob 1/2 selected. Save now for single blob, or click blob 2/2 for pair";
          }
        }
        else if (pose_status_text.empty() && !anchor_status.empty())
        {
          pose_status_text = "Blob 1/2 no longer visible. Reselect anchor blob";
        }
      }
      else if (pose_status_text.empty())
      {
        pose_status_text = pose_blob_reference_clicks_px_.empty()
          ? "Click blob 1/2 to set anchor"
          : "Blob 1/2 selected. Save now for single blob, or click blob 2/2 for pair";
      }
    }
    pose_estimate_ = pose_estimate;
    pose_stage_status_ = pose_status_text;

    cv::Mat binarized_display_mask = color_retain_mask;
    if (teach_stage_ >= TeachStage::kDepthNormalize && normalized_depth_ready)
    {
      binarized_display_mask = combined_mask;
      if (teach_stage_ >= TeachStage::kFlatLocate && cv::countNonZero(flat_mask) > 0)
      {
        binarized_display_mask = flat_mask;
      }
    }

    cv::Mat display_image;
    switch (view_mode_)
    {
      case ViewMode::kRgb:
        display_image = current_frame.clone();
        if (teach_stage_ == TeachStage::kColorMask)
        {
          tintMaskOverlay(display_image, rgb_focus_display_mask, cv::Scalar(255, 255, 255), 0.32);
        }
        break;
      case ViewMode::kDepth:
        display_image = normalized_depth_ready ? colorizeDepthResidual(depth_residual_display_m) : colorizeDepth(current_depth);
        if (display_image.size() != current_frame.size())
        {
          cv::resize(display_image, display_image, current_frame.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        break;
      case ViewMode::kBinarized:
      default:
      {
        const cv::Mat active_mask = roi_ready ? binarized_display_mask : color_mask;
        if (focus_black_mask_)
        {
          display_image = cv::Mat(active_mask.size(), CV_8UC3, cv::Scalar(240, 240, 240));
          display_image.setTo(cv::Scalar(8, 8, 8), active_mask);
        }
        else
        {
          display_image = cv::Mat(active_mask.size(), CV_8UC3, cv::Scalar(8, 8, 8));
          display_image.setTo(cv::Scalar(240, 240, 240), active_mask);
        }
        break;
      }
    }
    if (overlay_enabled_ && hasValidRoiPoints(roi_points_))
    {
      drawRoiOverlay(display_image, roi_points_, false);
    }
    if (overlay_enabled_ && hasDepthPlaneReference())
    {
      const std::vector<cv::Point2f> depth_roi_points =
        depth_plane_roi_points_.empty() && depth_plane_roi_bounds_.has_value()
        ? roiPointsFromBounds(*depth_plane_roi_bounds_)
        : depth_plane_roi_points_;
      if (!depth_roi_points.empty())
      {
        drawRoiOverlay(display_image, depth_roi_points, true);
      }

      if (current_camera_info != nullptr)
      {
        cv::Point2f normal_center(
          static_cast<float>(display_image.cols) * 0.5F,
          static_cast<float>(display_image.rows) * 0.5F);
        const auto &normal_pose_source = pose_estimate_overlay.has_value() ? pose_estimate_overlay : pose_estimate_;
        if (normal_pose_source.has_value() && !normal_pose_source->blob_poses.empty())
        {
          normal_center = poseBlobCenterPx(normal_pose_source->blob_poses.front());
        }
        else if (!depth_roi_points.empty())
        {
          normal_center = polygonCentroid(depth_roi_points);
        }
        drawDepthPlaneNormalOverlay(display_image, depth_plane_model_, *current_camera_info, normal_center);
      }
    }
    if (awaitingDepthNormalizePoints() && !pending_depth_plane_points_.empty())
    {
      drawRoiOverlay(display_image, pending_depth_plane_points_, true);
    }
    if (overlay_enabled_ && teach_stage_ >= TeachStage::kPosePerception)
    {
      drawPoseHullOverlay(
        display_image,
        pose_estimate_overlay.has_value() ? pose_estimate_overlay : pose_estimate_);
    }
    if (overlay_enabled_ && teach_stage_ >= TeachStage::kDepthNormalize && depth_window_peak_info.valid)
    {
      drawDepthWindowPeakOverlay(display_image, depth_window_peak_info);
    }
    drawViewLabel(display_image, currentViewLabel());
    if (!roi_ready)
    {
      cv::putText(
        display_image,
        "Load Bin Teach before mask processing",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.80,
        cv::Scalar(0, 180, 255),
        2);
    }
    else if (detection_use_depth_ && !hasDepthPlaneReference())
    {
      cv::putText(
        display_image,
        "Click 4 depth normalize points " + std::to_string(depthNormalizePointCount()) + "/4",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.72,
        cv::Scalar(0, 180, 255),
        2);
    }
    else if (teach_stage_ == TeachStage::kColorMask)
    {
      cv::putText(
        display_image,
        "Tune RGB mask, then Next",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.62,
        cv::Scalar(0, 180, 255),
        2);
    }
    else if (teach_stage_ == TeachStage::kFlatLocate && cv::countNonZero(flat_mask) == 0)
    {
      cv::putText(
        display_image,
        "No depth pixels after filter",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.72,
        cv::Scalar(0, 180, 255),
        2);
    }
    else if (teach_stage_ == TeachStage::kFlatLocate)
    {
      cv::putText(
        display_image,
        "Tune depth window/hole/trim, then Next",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.60,
        cv::Scalar(0, 180, 255),
        2);
    }
    else if (teach_stage_ == TeachStage::kPosePerception && !pose_estimate_.has_value())
    {
      cv::putText(
        display_image,
        pose_stage_status_.empty()
          ? "Click blob 1/2 to set anchor"
          : pose_stage_status_,
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.52,
        cv::Scalar(0, 180, 255),
        2);
    }
    else if (teach_stage_ == TeachStage::kPosePerception)
    {
      cv::putText(
        display_image,
        "Pose ready: " + pose_stage_status_ + " | group axes, X=long, Y=short",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.50,
        cv::Scalar(0, 180, 255),
        2);
    }

    const cv::Mat preview_canvas = buildPreviewCanvas(display_image);
    const cv::Mat video_top = drawButtonBar(preview_canvas.cols);
    cv::Mat right_panel;
    cv::vconcat(video_top, preview_canvas, right_panel);
    const int minimum_panel_height = minimumLeftPanelHeight();
    if (right_panel.rows < minimum_panel_height)
    {
      cv::copyMakeBorder(
        right_panel,
        right_panel,
        0,
        minimum_panel_height - right_panel.rows,
        0,
        0,
        cv::BORDER_CONSTANT,
        cv::Scalar(30, 32, 36));
    }
    const cv::Mat left_panel = drawLeftPanel(right_panel.rows);

    cv::Mat combined;
    cv::hconcat(left_panel, right_panel, combined);
    const cv::Size window_size(combined.cols, combined.rows);
    if (window_size != rendered_window_size_)
    {
      cv::resizeWindow(kWindowName, window_size.width, window_size.height);
      rendered_window_size_ = window_size;
    }
    cv::imshow(kWindowName, combined);
    handleKeypress(cv::waitKey(1));
    saveRuntimeSettingsToFile(false);
    publishItemPoses(current_header, depth_for_pose_m, current_camera_info, pose_estimate_);

    if (publish_overlay_)
    {
      cv_bridge::CvImage overlay_image;
      overlay_image.header = current_header;
      overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_image.image = combined;
      overlay_pub_->publish(*overlay_image.toImageMsg());
    }
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

    // Keep heading continuity with detected in-plane orientation (X=long side, Y=short side).
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

  void publishItemPoses(
    const std_msgs::msg::Header &header,
    const cv::Mat &depth_m,
    const CameraInfoMsg::ConstSharedPtr &camera_info,
    const std::optional<BinarizedPoseEstimate2D> &pose_estimate)
  {
    if (!publish_item_pose_array_ || !item_pose_array_pub_ || item_tf_parent_frame_.empty())
    {
      return;
    }
    if (
      teach_stage_ < TeachStage::kPosePerception ||
      camera_info == nullptr ||
      !pose_estimate.has_value() ||
      pose_estimate->blob_poses.empty())
    {
      return;
    }

    PoseArrayMsg pose_array_msg;
    pose_array_msg.header.stamp = header.stamp;
    pose_array_msg.header.frame_id = item_tf_parent_frame_;
    pose_array_msg.poses.reserve(pose_estimate->blob_poses.size());
    const cv::Size depth_image_size(depth_m.cols, depth_m.rows);

    for (const auto &blob_pose : pose_estimate->blob_poses)
    {
      const auto pose_3d = estimateBlobPose3D(blob_pose, depth_m, *camera_info);
      if (!pose_3d.has_value())
      {
        continue;
      }

      // Keep tray-style depth geometry: publish raw estimated pose without applying
      // an additional source->calibrated frame transform.
      ItemPose3D pose_to_publish = *pose_3d;
      const cv::Vec3d detected_z_axis(
        pose_3d->rotation(0, 2),
        pose_3d->rotation(1, 2),
        pose_3d->rotation(2, 2));
      const std::optional<cv::Vec3d> plane_normal_opt =
        align_item_z_axis_to_depth_plane_
        ? depthPlaneNormalInCameraFrame(
            depth_plane_model_,
            *camera_info,
            depth_image_size,
            poseBlobCenterPx(blob_pose),
            detected_z_axis)
        : std::nullopt;
      alignItemPoseZAxisToNormal(pose_to_publish, plane_normal_opt);
      const auto orientation = rotationToQuaternionMsg(pose_to_publish.rotation);

      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = pose_to_publish.origin[0];
      pose_msg.position.y = pose_to_publish.origin[1];
      pose_msg.position.z = pose_to_publish.origin[2];
      pose_msg.orientation = orientation;
      pose_array_msg.poses.push_back(pose_msg);
    }
    item_pose_array_pub_->publish(pose_array_msg);
  }

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string joint_states_topic_;
  std::string overlay_topic_;
  std::string calibration_parent_frame_;
  std::string calibration_child_frame_;
  std::string calibration_dir_;
  std::string calibration_file_;
  std::string item_tf_parent_frame_;
  std::string item_pose_array_topic_;
  std::string profiles_dir_;
	  std::string runtime_settings_path_;
	  std::string bin_teach_dir_;
	  std::string motion_service_root_;
	  std::string movj_service_name_;
	  std::string camera_control_service_root_ {"/robot_camera"};
	  std::string latest_saved_profile_path_;
  std::string save_status_message_ {"Saved"};
  std::string item_name_ {"item"};
  rclcpp::Time save_status_deadline_ {0, 0, RCL_ROS_TIME};
  bool use_calibration_ {true};
  bool publish_static_calibration_tf_ {true};
  bool auto_discover_calibration_ {true};
  bool publish_overlay_ {true};
  bool publish_item_pose_array_ {true};
  bool align_item_z_axis_to_depth_plane_ {true};
  double display_scale_ {1.0};
  cv::Size rendered_window_size_ {};

	  int red_threshold_ {120};
	  int green_threshold_ {0};
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
  int adaptive_depth_trim_max_factor_tenths_ {kAdaptiveDepthTrimFactorDefaultTenths};
  int adaptive_depth_trim_max_height_mm_ {kAdaptiveDepthTrimHeightDefaultMm};
  int blob_tolerance_percent_ {kBlobToleranceDefaultPercent};
  int bin_roi_move_speed_percent_ {100};
  bool live_view_enabled_ {true};
  ViewMode view_mode_ {ViewMode::kRgb};
  TeachStage teach_stage_ {TeachStage::kRoi};
  bool overlay_enabled_ {true};
  bool focus_black_mask_ {false};
  bool detection_use_depth_ {false};
  bool freeze_latched_ {false};
	  bool name_edit_active_ {false};
	  bool runtime_settings_dirty_ {false};
	  bool camera_exposure_dirty_ {true};
	  rclcpp::Time last_runtime_settings_save_time_ {0, 0, RCL_ROS_TIME};
	  rclcpp::Time last_camera_exposure_attempt_time_ {0, 0, RCL_ROS_TIME};
	  int last_applied_color_exposure_us_ {-1};
	  int last_applied_depth_exposure_us_ {-1};
  std::vector<UiButton> buttons_;
  std::vector<UiSlider> sliders_;
  std::vector<AxisAlignedRoiBounds> roi_regions_;
  std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds_;
  std::vector<cv::Point2f> roi_points_;
  std::vector<cv::Point2f> depth_plane_roi_points_;
  std::vector<cv::Point2f> pending_depth_plane_points_;
  DepthPlaneModel depth_plane_model_;
  bool depth_plane_from_bin_teach_ {false};
  std::string active_bin_name_;
  std::string active_bin_teach_path_;
  std::optional<BinarizedPoseEstimate2D> pose_estimate_;
  std::optional<cv::Point> pose_blob_seed_point_px_;
  std::optional<PoseBlobReference2D> pose_blob_reference_;
  std::vector<cv::Point> pose_blob_reference_clicks_px_;
  std::array<PoseReferenceSlot2D, kPoseReferenceSlotCount> pose_reference_slots_;
  std::array<cv::Rect, kPoseReferenceSlotCount> pose_reference_slot_rects_;
  int active_pose_reference_slot_index_ {0};
  std::string pose_stage_status_;
  cv::Rect name_box_rect_;
  cv::Rect bin_roi_dropdown_rect_;
  cv::Rect delete_bin_roi_button_rect_;
  cv::Rect back_button_rect_;
  cv::Rect save_button_rect_;
  bool bin_roi_dropdown_open_ {false};
  int selected_bin_roi_index_ {-1};
  int pending_bin_roi_index_ {-1};
  int bin_teach_yaml_file_count_ {0};
  int bin_teach_skipped_file_count_ {0};
  std::vector<BinTeachRoiEntry> bin_roi_entries_;
  std::string pending_delete_bin_teach_path_;
  rclcpp::Time pending_delete_bin_teach_deadline_ {0, 0, RCL_ROS_TIME};
  int active_slider_index_ {-1};
  geometry_msgs::msg::Vector3 calibration_translation_;
  QuaternionMsg calibration_rotation_;

	  rclcpp::Publisher<ImageMsg>::SharedPtr overlay_pub_;
	  rclcpp::Publisher<PoseArrayMsg>::SharedPtr item_pose_array_pub_;
	  rclcpp::Client<MovJSrv>::SharedPtr movj_client_;
	  rclcpp::Client<SetBoolSrv>::SharedPtr color_auto_exposure_client_;
	  rclcpp::Client<SetInt32Srv>::SharedPtr color_exposure_client_;
	  rclcpp::Client<SetBoolSrv>::SharedPtr depth_auto_exposure_client_;
	  rclcpp::Client<SetInt32Srv>::SharedPtr depth_exposure_client_;
	  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;
	  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
	  rclcpp::TimerBase::SharedPtr render_timer_;
	  rclcpp::TimerBase::SharedPtr camera_exposure_timer_;

  std::mutex frame_mutex_;
  std::mutex joint_state_mutex_;
  cv::Mat latest_frame_;
  cv::Mat latest_depth_;
  cv::Mat frozen_frame_;
  cv::Mat frozen_depth_;
  std_msgs::msg::Header latest_header_;
  std_msgs::msg::Header frozen_header_;
  CameraInfoMsg::ConstSharedPtr latest_camera_info_;
  CameraInfoMsg::ConstSharedPtr frozen_camera_info_;
  std::array<double, 6> latest_joint_positions_deg_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_joint_positions_ {false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ItemTeachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
