#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include <dobot_common/workspace_paths.hpp>

namespace
{
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using JointStateMsg = sensor_msgs::msg::JointState;

constexpr char kWindowName[] = "tray_rgb_tuner";
constexpr char kRedTrackbar[] = "R Threshold";
constexpr char kGreenTrackbar[] = "G Threshold";
constexpr char kBlueTrackbar[] = "B Threshold";
constexpr char kDepthThresholdTrackbar[] = "D Threshold (+/- mm)";
constexpr char kRayStepTrackbar[] = "Ray Step Px";
constexpr char kDepthEdgeOffsetTrackbar[] = "Depth Edge Offset Px";
constexpr char kPreviousColorTrackbar[] = "Prev Color %";
constexpr char kHorizontalRayCountTrackbar[] = "H Ray Count";
constexpr char kVerticalRayCountTrackbar[] = "V Ray Count";
constexpr char kOutlierTrackbar[] = "Outlier Sens";
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
constexpr int kDepthThresholdMinMm = 1;
constexpr int kDepthThresholdMaxMm = 20;
constexpr int kHorizontalRayCountMin = 30;
constexpr int kHorizontalRayCountMax = 60;
constexpr int kDepthEdgeOffsetMinPx = 1;
constexpr int kDepthEdgeOffsetMaxPx = 20;
constexpr std::size_t kTeachRoiPointCount = 4;

double remapOutlierSensitivityToFitRange(int outlier_sensitivity)
{
  const int clamped = std::clamp(outlier_sensitivity, 1, 100);
  return 50.0 + (static_cast<double>(clamped - 1) * 100.0 / 99.0);
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

std::string makeFilenameSafeTrayName(const std::string &tray_name)
{
  std::string safe_name;
  safe_name.reserve(tray_name.size());

  for (const unsigned char ch : tray_name)
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

  return safe_name.empty() ? "tray" : safe_name;
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

cv::Mat buildRgbMask(const cv::Mat &bgr, int red_threshold, int green_threshold, int blue_threshold)
{
  cv::Mat mask;
  cv::inRange(
    bgr,
    cv::Scalar(blue_threshold, green_threshold, red_threshold),
    cv::Scalar(255, 255, 255),
    mask);
  return mask;
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

struct TrayEstimate
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

struct TrayMetricEstimate
{
  double area_cm2 {0.0};
  double mean_depth_m {0.0};
  // Ordered as: origin X edge, opposite X edge, origin Y edge, opposite Y edge.
  std::array<double, 4> edge_lengths_cm {0.0, 0.0, 0.0, 0.0};
};

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

std::optional<double> averageDepthAt(const cv::Mat &depth_m, const cv::Point2f &pt, int window_size = 5)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return std::nullopt;
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

double vectorNorm(const cv::Vec3d &vec)
{
  return std::sqrt(vec.dot(vec));
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

int lowerRightCornerIndex(const std::vector<cv::Point2f> &corners)
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
      return corner_a.x > corner_b.x;
    });

  int lower_right_idx = corner_indices[0];
  if (corners[corner_indices[1]].x > corners[corner_indices[0]].x)
  {
    lower_right_idx = corner_indices[1];
  }
  return lower_right_idx;
}

std::optional<std::array<cv::Vec3d, 4>> estimateTrayCornerCameraPoints(
  const std::vector<cv::Point2f> &corners,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info)
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
    const auto depth = averageDepthAt(depth_m, corners[i]);
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

std::optional<TrayMetricEstimate> estimateTrayMetricsFromCorners(
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
  const int origin_idx = lowerRightCornerIndex(corners);
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

  TrayMetricEstimate metrics;
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

  if (const auto camera_points = estimateTrayCornerCameraPoints(corners, depth_m, camera_info); camera_points.has_value())
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

std::optional<TrayEstimate> buildTrayEstimateFromIsolatedSideSamples(
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

  TrayEstimate estimate;
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
  if (const auto metrics = estimateTrayMetricsFromCorners(
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

std::vector<cv::Point2f> roiPointsFromYamlNode(const YAML::Node &points_node)
{
  std::vector<cv::Point2f> points;
  if (!points_node || !points_node.IsSequence())
  {
    return points;
  }

  if (points_node.size() > 0 && points_node[0].IsScalar())
  {
    for (std::size_t i = 0; i + 1 < points_node.size(); i += 2)
    {
      points.emplace_back(
        points_node[i].as<float>(),
        points_node[i + 1].as<float>());
    }
    return points;
  }

  for (const auto &point_node : points_node)
  {
    if (!point_node.IsSequence() || point_node.size() < 2)
    {
      continue;
    }
    points.emplace_back(point_node[0].as<float>(), point_node[1].as<float>());
  }
  return points;
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

bool fitDepthPlaneFromRoiPoints(
  const cv::Mat &depth_m,
  const std::vector<cv::Point2f> &corners,
  DepthPlaneModel &plane_out)
{
  plane_out = DepthPlaneModel{};
  if (depth_m.empty() || depth_m.type() != CV_32FC1 || corners.size() != kTeachRoiPointCount)
  {
    return false;
  }

  cv::Mat A(4, 3, CV_64F);
  cv::Mat b(4, 1, CV_64F);
  double depth_sum = 0.0;
  int valid_count = 0;
  for (int i = 0; i < 4; ++i)
  {
    const auto depth = averageDepthAt(depth_m, corners[static_cast<std::size_t>(i)], 7);
    if (!depth.has_value() || !std::isfinite(*depth) || *depth <= 0.0)
    {
      return false;
    }
    const int px = static_cast<int>(std::lround(corners[static_cast<std::size_t>(i)].x));
    const int py = static_cast<int>(std::lround(corners[static_cast<std::size_t>(i)].y));
    const double x_norm = normalizedImageCoord(std::clamp(px, 0, depth_m.cols - 1), depth_m.cols);
    const double y_norm = normalizedImageCoord(std::clamp(py, 0, depth_m.rows - 1), depth_m.rows);
    A.at<double>(i, 0) = x_norm;
    A.at<double>(i, 1) = y_norm;
    A.at<double>(i, 2) = 1.0;
    b.at<double>(i, 0) = *depth;
    depth_sum += *depth;
    ++valid_count;
  }

  if (valid_count < 3)
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
  const double reference_depth_m = depth_sum / static_cast<double>(valid_count);
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
    plane_out = DepthPlaneModel{};
    return false;
  }
  return fitDepthPlaneFromRoiPoints(depth_m, roiPointsFromBounds(bounds), plane_out);
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

std::optional<TrayEstimate> detectTrayFromAxisAlignedRoi(
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
  const cv::Point roi_center((left + right) / 2, (top + bottom) / 2);

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

    const int center_x = std::clamp(roi_center.x, interval_left, interval_right);
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

    const int center_y = std::clamp(roi_center.y, interval_top, interval_bottom);
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

  return buildTrayEstimateFromIsolatedSideSamples(
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

std::optional<TrayEstimate> detectTrayFromAxisAlignedRoi(
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
  const cv::Point roi_center((left + right) / 2, (top + bottom) / 2);

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

    const int center_x = std::clamp(roi_center.x, interval_left, interval_right);
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

    const int center_y = std::clamp(roi_center.y, interval_top, interval_bottom);
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

  return buildTrayEstimateFromIsolatedSideSamples(
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

void drawTrayEstimate(cv::Mat &binary_bgr, const std::optional<TrayEstimate> &estimate, int depth_edge_offset_px)
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

  for (const auto &point : estimate->edge_points)
  {
    cv::circle(binary_bgr, point, 3, cv::Scalar(60, 60, 60), -1);
  }

  for (const auto &point : estimate->filtered_edge_points)
  {
    cv::circle(binary_bgr, point, 5, cv::Scalar(0, 0, 0), -1);
    cv::circle(binary_bgr, point, 3, cv::Scalar(0, 165, 255), -1);
  }

  if (!estimate->has_metric_estimate || estimate->corners.size() != 4)
  {
    return;
  }

  const int origin_idx = lowerRightCornerIndex(estimate->corners);
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

    const std::string label = cv::format("%.2f cm", estimate->edge_lengths_cm[i]);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
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
      0.55,
      cv::Scalar(0, 255, 0),
      2);
  }
}

void drawMeasurementAxes(cv::Mat &image, const std::optional<TrayEstimate> &estimate)
{
  if (!estimate.has_value() || estimate->corners.size() != 4)
  {
    return;
  }

  const int lower_right_idx = lowerRightCornerIndex(estimate->corners);
  if (lower_right_idx < 0)
  {
    return;
  }

  const cv::Point2f origin = estimate->corners[lower_right_idx];
  const cv::Point2f prev_corner = estimate->corners[(lower_right_idx + 3) % 4];
  const cv::Point2f next_corner = estimate->corners[(lower_right_idx + 1) % 4];

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

void drawModeLabel(cv::Mat &image, const std::string &label)
{
  if (image.empty())
  {
    return;
  }

  const std::string text = "Mode " + label;
  int baseline = 0;
  const cv::Size text_size = cv::getTextSize(
    text,
    cv::FONT_HERSHEY_SIMPLEX,
    0.62,
    2,
    &baseline);
  const int box_w = std::max(130, text_size.width + 20);
  const int box_h = 34;
  const int margin = 14;
  if (image.cols <= margin * 2 + 20 || image.rows <= margin + box_h)
  {
    return;
  }
  const int box_x = std::max(margin, image.cols - box_w - margin);
  const cv::Rect box(box_x, 12, box_w, box_h);
  cv::rectangle(image, box, cv::Scalar(24, 24, 24), cv::FILLED);
  cv::rectangle(image, box, cv::Scalar(210, 210, 210), 2);
  cv::putText(
    image,
    text,
    cv::Point(box.x + 10, box.y + 23),
    cv::FONT_HERSHEY_SIMPLEX,
    0.62,
    cv::Scalar(255, 255, 255),
    2);
}

void drawRays(
  cv::Mat &binary_bgr,
  const std::vector<cv::Point2f> &roi_points,
  int horizontal_ray_count,
  int vertical_ray_count)
{
  const auto roi_bounds = roiBoundsForImage(roi_points, binary_bgr.size());
  if (!roi_bounds.has_value())
  {
    return;
  }
  const cv::Mat roi_mask = buildRoiMask(binary_bgr.size(), roi_points);

  const int row_scan_count = std::clamp(horizontal_ray_count, 50, 100);
  const int col_scan_count = std::clamp(vertical_ray_count, 50, 150);
  const auto rows = sampleAxisPositions(roi_bounds->top, roi_bounds->bottom, row_scan_count);
  const auto cols = sampleAxisPositions(roi_bounds->left, roi_bounds->right, col_scan_count);
  const cv::Point center(
    (roi_bounds->left + roi_bounds->right) / 2,
    (roi_bounds->top + roi_bounds->bottom) / 2);

  for (const int y : rows)
  {
    const auto interval = widestRowInterval(roi_mask, y);
    if (!interval.has_value())
    {
      continue;
    }
    const int center_x = std::clamp(center.x, interval->first, interval->second);
    cv::line(
      binary_bgr,
      cv::Point(interval->first, y),
      cv::Point(center_x, y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
    cv::line(
      binary_bgr,
      cv::Point(interval->second, y),
      cv::Point(center_x, y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
  }
  for (const int x : cols)
  {
    const auto interval = widestColumnInterval(roi_mask, x);
    if (!interval.has_value())
    {
      continue;
    }
    const int center_y = std::clamp(center.y, interval->first, interval->second);
    cv::line(
      binary_bgr,
      cv::Point(x, interval->first),
      cv::Point(x, center_y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
    cv::line(
      binary_bgr,
      cv::Point(x, interval->second),
      cv::Point(x, center_y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
  }

  cv::circle(binary_bgr, center, 5, cv::Scalar(255, 255, 255), -1);
  cv::circle(binary_bgr, center, 7, cv::Scalar(0, 0, 0), 1);
}

void drawRays(
  cv::Mat &binary_bgr,
  const std::vector<AxisAlignedRoiBounds> &roi_regions,
  int horizontal_ray_count,
  int vertical_ray_count)
{
  const auto roi_bounds = roiBoundsForImage(roi_regions, binary_bgr.size());
  if (!roi_bounds.has_value())
  {
    return;
  }

  const int row_scan_count = std::clamp(horizontal_ray_count, 50, 100);
  const int col_scan_count = std::clamp(vertical_ray_count, 50, 150);
  const auto rows = sampleAxisPositions(roi_bounds->top, roi_bounds->bottom, row_scan_count);
  const auto cols = sampleAxisPositions(roi_bounds->left, roi_bounds->right, col_scan_count);
  const cv::Point center(
    (roi_bounds->left + roi_bounds->right) / 2,
    (roi_bounds->top + roi_bounds->bottom) / 2);

  for (const int y : rows)
  {
    const auto interval = widestRowInterval(roi_regions, y, binary_bgr.size());
    if (!interval.has_value())
    {
      continue;
    }
    const int center_x = std::clamp(center.x, interval->first, interval->second);
    cv::line(
      binary_bgr,
      cv::Point(interval->first, y),
      cv::Point(center_x, y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
    cv::line(
      binary_bgr,
      cv::Point(interval->second, y),
      cv::Point(center_x, y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
  }
  for (const int x : cols)
  {
    const auto interval = widestColumnInterval(roi_regions, x, binary_bgr.size());
    if (!interval.has_value())
    {
      continue;
    }
    const int center_y = std::clamp(center.y, interval->first, interval->second);
    cv::line(
      binary_bgr,
      cv::Point(x, interval->first),
      cv::Point(x, center_y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
    cv::line(
      binary_bgr,
      cv::Point(x, interval->second),
      cv::Point(x, center_y),
      cv::Scalar(90, 90, 90),
      1,
      cv::LINE_AA);
  }

  cv::circle(binary_bgr, center, 5, cv::Scalar(255, 255, 255), -1);
  cv::circle(binary_bgr, center, 7, cv::Scalar(0, 0, 0), 1);
}
}  // namespace

class TrayDetectorNode : public rclcpp::Node
{
public:
  TrayDetectorNode()
  : Node("tray_teach")
  {
    color_topic_ = declare_parameter<std::string>("color_topic", "/robot_camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/robot_camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/robot_camera/color/camera_info");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states_robot");
    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "tray_overlay");
    profiles_dir_ = declare_parameter<std::string>(
      "profiles_dir",
      dobot_common::paths::workspacePath({"teach", "tray_teach"}, __FILE__).string());
    settings_path_ = declare_parameter<std::string>(
      "settings_path",
      dobot_common::paths::workspacePath(
        {"config", "tray_perception", "tray_teach_settings.yaml"}, __FILE__).string());
    runtime_settings_path_ = declare_parameter<std::string>(
      "runtime_settings_path",
      dobot_common::paths::workspacePath(
        {"config", "tray_perception", "tray_teach_runtime.yaml"}, __FILE__).string());
    publish_overlay_ = declare_parameter<bool>("publish_overlay", true);
    display_scale_ = declare_parameter<double>("display_scale", 1.0);
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
      kHorizontalRayCountMin,
      kHorizontalRayCountMax);
    vertical_ray_count_ = std::clamp(
      static_cast<int>(declare_parameter<int>("vertical_ray_count", 50)),
      50,
      150);
    trace_out_to_in_ = declare_parameter<bool>("trace_out_to_in", false);
    loadRuntimeSettingsFromFile();

    overlay_pub_ = create_publisher<ImageMsg>(overlay_topic_, rclcpp::QoS(5));
    color_sub_ = create_subscription<ImageMsg>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrayDetectorNode::colorCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<ImageMsg>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrayDetectorNode::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<CameraInfoMsg>(
      camera_info_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&TrayDetectorNode::cameraInfoCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<JointStateMsg>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrayDetectorNode::jointStateCallback, this, std::placeholders::_1));

    createUi();
    cv::setMouseCallback(kWindowName, &TrayDetectorNode::onMouseThunk, this);

    render_timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&TrayDetectorNode::renderFrame, this));

    RCLCPP_INFO(
      get_logger(),
      "Tray teach ready. Color topic=%s depth topic=%s info topic=%s joints topic=%s overlay topic=%s settings=%s",
      color_topic_.c_str(),
      depth_topic_.c_str(),
      camera_info_topic_.c_str(),
      joint_states_topic_.c_str(),
      overlay_topic_.c_str(),
      settings_path_.c_str());
  }

  ~TrayDetectorNode() override
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
    buttons_.push_back({"Add ROI", cv::Rect(button_x, top_y, 116, top_h), nullptr});
    button_x += 116 + top_gap;
    buttons_.push_back({"Clear ROI", cv::Rect(button_x, top_y, 122, top_h), nullptr});
    button_x += 122 + top_gap;
    buttons_.push_back({"View", cv::Rect(button_x, top_y, 142, top_h), nullptr});
    button_x += 142 + top_gap;
    buttons_.push_back({"Overlay", cv::Rect(button_x, top_y, 128, top_h), &overlay_enabled_});
    button_x += 128 + top_gap;
    buttons_.push_back({"Rays", cv::Rect(button_x, top_y, 104, top_h), &show_rays_enabled_});
    button_x += 104 + top_gap;
    buttons_.push_back({"EdgeDir", cv::Rect(button_x, top_y, 132, top_h), &detect_black_to_white_});
    button_x += 132 + top_gap;
    buttons_.push_back({"RayDir", cv::Rect(button_x, top_y, 132, top_h), &trace_out_to_in_});

    sliders_.clear();
    name_box_rect_ = cv::Rect(20, 94, kLeftPanelWidth - 40, 40);
    save_button_rect_ = cv::Rect(20, 150, 188, 40);
    int y = 260;
    const int track_x = 20;
    const int track_w = kLeftPanelWidth - 40;
    const int track_h = 12;
    const int gap = 52;
    sliders_.push_back({kRedTrackbar, cv::Rect(track_x, y, track_w, track_h), &red_threshold_, 0, 255}); y += gap;
    sliders_.push_back({kGreenTrackbar, cv::Rect(track_x, y, track_w, track_h), &green_threshold_, 0, 255}); y += gap;
    sliders_.push_back({kBlueTrackbar, cv::Rect(track_x, y, track_w, track_h), &blue_threshold_, 0, 255}); y += gap;
    sliders_.push_back(
      {kDepthThresholdTrackbar, cv::Rect(track_x, y, track_w, track_h), &depth_threshold_mm_, kDepthThresholdMinMm, kDepthThresholdMaxMm}); y += gap;
    sliders_.push_back({kRayStepTrackbar, cv::Rect(track_x, y, track_w, track_h), &ray_step_px_, 1, 100}); y += gap;
    sliders_.push_back(
      {kDepthEdgeOffsetTrackbar, cv::Rect(track_x, y, track_w, track_h), &depth_edge_offset_px_, kDepthEdgeOffsetMinPx, kDepthEdgeOffsetMaxPx}); y += gap;
    sliders_.push_back({kPreviousColorTrackbar, cv::Rect(track_x, y, track_w, track_h), &previous_color_percent_, 20, 100}); y += gap;
    sliders_.push_back(
      {kHorizontalRayCountTrackbar,
        cv::Rect(track_x, y, track_w, track_h),
        &horizontal_ray_count_,
        kHorizontalRayCountMin,
        kHorizontalRayCountMax}); y += gap;
    sliders_.push_back({kVerticalRayCountTrackbar, cv::Rect(track_x, y, track_w, track_h), &vertical_ray_count_, 50, 150}); y += gap;
    sliders_.push_back({kOutlierTrackbar, cv::Rect(track_x, y, track_w, track_h), &outlier_sensitivity_, 1, 100});
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

  bool isRgbThresholdSliderLabel(const std::string &label) const
  {
    return label == kRedTrackbar || label == kGreenTrackbar || label == kBlueTrackbar;
  }

  bool isDepthThresholdSliderLabel(const std::string &label) const
  {
    return label == kDepthThresholdTrackbar;
  }

  bool isSliderEnabled(const UiSlider &slider) const
  {
    if (isDepthThresholdSliderLabel(slider.label))
    {
      return detection_use_depth_;
    }
    if (isRgbThresholdSliderLabel(slider.label))
    {
      return !detection_use_depth_;
    }
    return true;
  }

  void updateDetectionModeFromCurrentView()
  {
    if (view_mode_ == ViewMode::kDepth)
    {
      detection_use_depth_ = true;
    }
    else if (view_mode_ == ViewMode::kRgb)
    {
      detection_use_depth_ = false;
    }
  }

  bool hasDepthPlaneReference() const
  {
    return depth_plane_model_.valid &&
      (depth_plane_roi_bounds_.has_value() || hasValidRoiPoints(depth_plane_roi_points_));
  }

  void clearDepthPlaneReference(bool mark_dirty = true)
  {
    depth_plane_model_ = DepthPlaneModel{};
    depth_plane_roi_bounds_.reset();
    depth_plane_roi_points_.clear();
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
    int last_slider_bottom = slider_card_y + 180;
    for (const auto &slider : sliders_)
    {
      last_slider_bottom = std::max(last_slider_bottom, slider.track_rect.y + 34);
    }
    return last_slider_bottom + 12;
  }

  static void onMouseThunk(int event, int x, int y, int flags, void *userdata)
  {
    static_cast<TrayDetectorNode *>(userdata)->onMouse(event, x, y, flags);
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
      markRuntimeSettingsDirty();
    }
  }

  void rebuildMergedRoiPolygon()
  {
    roi_points_ = mergeRoiRegionsIntoPolygon(roi_regions_);
    markRuntimeSettingsDirty();
  }

  void resetTaughtMetrics()
  {
    latest_tray_area_cm2_.reset();
    latest_tray_edge_lengths_cm_.reset();
  }

  void markRuntimeSettingsDirty()
  {
    runtime_settings_dirty_ = true;
  }

  bool saveRuntimeSettingsToFile(bool force)
  {
    if (runtime_settings_path_.empty())
    {
      return false;
    }

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
    out << YAML::Key << "tray_teach_runtime";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "ros__parameters";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "tray_name" << YAML::Value << sanitizeTrayName();
    out << YAML::Key << "red_threshold" << YAML::Value << red_threshold_;
    out << YAML::Key << "green_threshold" << YAML::Value << green_threshold_;
    out << YAML::Key << "blue_threshold" << YAML::Value << blue_threshold_;
    out << YAML::Key << "depth_threshold_mm" << YAML::Value << depth_threshold_mm_;
    out << YAML::Key << "detection_mode" << YAML::Value << detectionModeToString(detection_use_depth_);
    out << YAML::Key << "ray_step_px" << YAML::Value << ray_step_px_;
    out << YAML::Key << "depth_edge_offset_px" << YAML::Value << depth_edge_offset_px_;
    out << YAML::Key << "previous_color_percent" << YAML::Value << previous_color_percent_;
    out << YAML::Key << "horizontal_ray_count" << YAML::Value << horizontal_ray_count_;
    out << YAML::Key << "vertical_ray_count" << YAML::Value << vertical_ray_count_;
    out << YAML::Key << "outlier_sensitivity" << YAML::Value << outlier_sensitivity_;
    out << YAML::Key << "live_view_enabled" << YAML::Value << live_view_enabled_;
    out << YAML::Key << "view_mode" << YAML::Value << static_cast<int>(view_mode_);
    out << YAML::Key << "overlay_enabled" << YAML::Value << overlay_enabled_;
    out << YAML::Key << "show_rays_enabled" << YAML::Value << show_rays_enabled_;
    out << YAML::Key << "detect_black_to_white" << YAML::Value << detect_black_to_white_;
    out << YAML::Key << "trace_out_to_in" << YAML::Value << trace_out_to_in_;
    out << YAML::Key << "depth_plane_enabled" << YAML::Value << depth_plane_model_.valid;
    out << YAML::Key << "depth_plane_a" << YAML::Value << depth_plane_model_.a;
    out << YAML::Key << "depth_plane_b" << YAML::Value << depth_plane_model_.b;
    out << YAML::Key << "depth_plane_c" << YAML::Value << depth_plane_model_.c;
    out << YAML::Key << "depth_plane_reference_depth_m" << YAML::Value << depth_plane_model_.reference_depth_m;
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
    if (!depth_plane_roi_points_.empty())
    {
      out << YAML::Key << "depth_plane_roi_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : depth_plane_roi_points_)
      {
        out << static_cast<int>(std::round(point.x));
        out << static_cast<int>(std::round(point.y));
      }
      out << YAML::EndSeq;
    }
    if (!roi_points_.empty())
    {
      out << YAML::Key << "roi_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : roi_points_)
      {
        out << static_cast<int>(std::round(point.x));
        out << static_cast<int>(std::round(point.y));
      }
      out << YAML::EndSeq;
    }
    if (!roi_regions_.empty())
    {
      out << YAML::Key << "roi_regions" << YAML::Value << YAML::BeginSeq;
      for (const auto &region : roi_regions_)
      {
        out << YAML::Flow << YAML::BeginSeq
            << region.left << region.top << region.right << region.bottom
            << YAML::EndSeq;
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::filesystem::path runtime_path(runtime_settings_path_);
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
      const YAML::Node root = YAML::LoadFile(runtime_settings_path_);
      YAML::Node params;
      if (root["tray_teach_runtime"] && root["tray_teach_runtime"]["ros__parameters"])
      {
        params = root["tray_teach_runtime"]["ros__parameters"];
      }
      else if (root["tray_detect"] && root["tray_detect"]["ros__parameters"])
      {
        params = root["tray_detect"]["ros__parameters"];
      }

      if (!params || !params.IsMap())
      {
        return;
      }

      if (params["tray_name"])
      {
        tray_name_ = params["tray_name"].as<std::string>();
      }
      red_threshold_ = params["red_threshold"] ? std::clamp(params["red_threshold"].as<int>(), 0, 255) : red_threshold_;
      green_threshold_ = params["green_threshold"] ? std::clamp(params["green_threshold"].as<int>(), 0, 255) : green_threshold_;
      blue_threshold_ = params["blue_threshold"] ? std::clamp(params["blue_threshold"].as<int>(), 0, 255) : blue_threshold_;
      depth_threshold_mm_ = params["depth_threshold_mm"]
        ? std::clamp(params["depth_threshold_mm"].as<int>(), kDepthThresholdMinMm, kDepthThresholdMaxMm)
        : depth_threshold_mm_;
      if (params["detection_mode"])
      {
        detection_use_depth_ = isDepthDetectionMode(params["detection_mode"].as<std::string>());
      }
      ray_step_px_ = params["ray_step_px"] ? std::clamp(params["ray_step_px"].as<int>(), 1, 100) : ray_step_px_;
      depth_edge_offset_px_ = params["depth_edge_offset_px"]
        ? std::clamp(params["depth_edge_offset_px"].as<int>(), kDepthEdgeOffsetMinPx, kDepthEdgeOffsetMaxPx)
        : depth_edge_offset_px_;
      previous_color_percent_ = params["previous_color_percent"] ? std::clamp(params["previous_color_percent"].as<int>(), 20, 100) : previous_color_percent_;
      horizontal_ray_count_ = params["horizontal_ray_count"]
        ? std::clamp(
            params["horizontal_ray_count"].as<int>(),
            kHorizontalRayCountMin,
            kHorizontalRayCountMax)
        : horizontal_ray_count_;
      vertical_ray_count_ = params["vertical_ray_count"] ? std::clamp(params["vertical_ray_count"].as<int>(), 50, 150) : vertical_ray_count_;
      outlier_sensitivity_ = params["outlier_sensitivity"] ? std::clamp(params["outlier_sensitivity"].as<int>(), 1, 100) : outlier_sensitivity_;
      live_view_enabled_ = params["live_view_enabled"] ? params["live_view_enabled"].as<bool>() : live_view_enabled_;
      overlay_enabled_ = params["overlay_enabled"] ? params["overlay_enabled"].as<bool>() : overlay_enabled_;
      show_rays_enabled_ = params["show_rays_enabled"] ? params["show_rays_enabled"].as<bool>() : show_rays_enabled_;
      detect_black_to_white_ = params["detect_black_to_white"] ? params["detect_black_to_white"].as<bool>() : detect_black_to_white_;
      trace_out_to_in_ = params["trace_out_to_in"] ? params["trace_out_to_in"].as<bool>() : trace_out_to_in_;
      clearDepthPlaneReference(false);
      if (params["depth_plane_roi"] && params["depth_plane_roi"].IsSequence() && params["depth_plane_roi"].size() >= 4)
      {
        AxisAlignedRoiBounds plane_bounds{
          params["depth_plane_roi"][0].as<int>(),
          params["depth_plane_roi"][1].as<int>(),
          params["depth_plane_roi"][2].as<int>(),
          params["depth_plane_roi"][3].as<int>(),
        };
        if (isValidRoiBounds(plane_bounds))
        {
          depth_plane_roi_bounds_ = plane_bounds;
          depth_plane_roi_points_ = roiPointsFromBounds(plane_bounds);
        }
      }
      if (const auto depth_plane_points = roiPointsFromYamlNode(params["depth_plane_roi_points"]);
          depth_plane_points.size() == kTeachRoiPointCount)
      {
        depth_plane_roi_points_ = depth_plane_points;
        depth_plane_roi_bounds_ = roiBoundsFromSelection(depth_plane_points);
      }
      const bool depth_plane_enabled =
        params["depth_plane_enabled"] ? params["depth_plane_enabled"].as<bool>() : false;
      if (depth_plane_enabled && (depth_plane_roi_bounds_.has_value() || hasValidRoiPoints(depth_plane_roi_points_)))
      {
        const double a = params["depth_plane_a"] ? params["depth_plane_a"].as<double>() : 0.0;
        const double b = params["depth_plane_b"] ? params["depth_plane_b"].as<double>() : 0.0;
        const double c = params["depth_plane_c"] ? params["depth_plane_c"].as<double>() : 0.0;
        const double ref_depth_m = params["depth_plane_reference_depth_m"]
          ? params["depth_plane_reference_depth_m"].as<double>()
          : 0.0;
        if (std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(ref_depth_m) && ref_depth_m > 0.0)
        {
          depth_plane_model_.a = a;
          depth_plane_model_.b = b;
          depth_plane_model_.c = c;
          depth_plane_model_.reference_depth_m = ref_depth_m;
          depth_plane_model_.valid = true;
        }
      }

      const int view_mode_value =
        params["view_mode"] ? std::clamp(params["view_mode"].as<int>(), 0, 2) : static_cast<int>(view_mode_);
      view_mode_ = static_cast<ViewMode>(view_mode_value);

      roi_regions_.clear();
      roi_points_.clear();
      const auto saved_roi_points = roiPointsFromYamlNode(params["roi_points"]);
      if (hasValidRoiPoints(saved_roi_points))
      {
        roi_points_ = saved_roi_points;
      }
      else if (const YAML::Node roi_regions = params["roi_regions"]; roi_regions && roi_regions.IsSequence())
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
            roi_regions_.push_back(region);
          }
        }

        if (!roi_regions_.empty())
        {
          roi_points_ = mergeRoiRegionsIntoPolygon(roi_regions_);
          roi_regions_.clear();
        }
      }
      if (!hasValidRoiPoints(roi_points_))
      {
        roi_points_.clear();
        clearDepthPlaneReference(false);
      }
      pending_roi_points_.clear();
      roi_selection_active_ = false;
      depth_plane_roi_selection_active_ = false;
      resetTaughtMetrics();
      runtime_settings_dirty_ = false;
      last_runtime_settings_save_time_ = this->now();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Runtime settings load failed: %s", ex.what());
    }
  }

  void clearLastRoiRegion()
  {
    const bool had_pending_selection = !pending_roi_points_.empty() || roi_selection_active_;
    pending_roi_points_.clear();

    if (had_pending_selection)
    {
      roi_selection_active_ = false;
      const bool was_depth_plane_selection = depth_plane_roi_selection_active_;
      depth_plane_roi_selection_active_ = false;
      save_status_message_ = was_depth_plane_selection ? "Depth plane ROI cancelled" : "ROI selection cancelled";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      markRuntimeSettingsDirty();
      return;
    }

    roi_regions_.clear();
    roi_points_.clear();
    clearDepthPlaneReference(false);
    resetTaughtMetrics();
    depth_plane_roi_selection_active_ = false;
    save_status_message_ = "ROI cleared";
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
    markRuntimeSettingsDirty();
  }

  void saveSettingsToFile()
  {
    if (!hasValidRoiPoints(roi_points_))
    {
      save_status_message_ = "ROI required";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }

    if (!latest_tray_edge_lengths_cm_.has_value())
    {
      save_status_message_ = "No tray edge lengths";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    if (detection_use_depth_ && !hasDepthPlaneReference())
    {
      save_status_message_ = "Depth plane ROI required";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }

    std::array<double, 6> joints_deg_snapshot {};
    bool has_joint_snapshot = false;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joints_deg_snapshot = latest_joint_positions_deg_;
      has_joint_snapshot = has_joint_positions_;
    }

    const std::string tray_name = sanitizeTrayName();
    const DateStamp date_stamp = currentDateStamp();

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "tray_detect";
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
    out << YAML::Key << "depth_threshold_mm" << YAML::Value << depth_threshold_mm_;
    out << YAML::Key << "detection_mode" << YAML::Value << detectionModeToString(detection_use_depth_);
    out << YAML::Key << "ray_step_px" << YAML::Value << ray_step_px_;
    out << YAML::Key << "depth_edge_offset_px" << YAML::Value << depth_edge_offset_px_;
    out << YAML::Key << "previous_color_percent" << YAML::Value << previous_color_percent_;
    out << YAML::Key << "horizontal_ray_count" << YAML::Value << horizontal_ray_count_;
    out << YAML::Key << "vertical_ray_count" << YAML::Value << vertical_ray_count_;
    out << YAML::Key << "outlier_sensitivity" << YAML::Value << outlier_sensitivity_;
    out << YAML::Key << "detect_black_to_white" << YAML::Value << detect_black_to_white_;
    out << YAML::Key << "trace_out_to_in" << YAML::Value << trace_out_to_in_;
    out << YAML::Key << "depth_plane_enabled" << YAML::Value << depth_plane_model_.valid;
    out << YAML::Key << "depth_plane_a" << YAML::Value << depth_plane_model_.a;
    out << YAML::Key << "depth_plane_b" << YAML::Value << depth_plane_model_.b;
    out << YAML::Key << "depth_plane_c" << YAML::Value << depth_plane_model_.c;
    out << YAML::Key << "depth_plane_reference_depth_m" << YAML::Value << depth_plane_model_.reference_depth_m;
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
    if (!depth_plane_roi_points_.empty())
    {
      out << YAML::Key << "depth_plane_roi_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &point : depth_plane_roi_points_)
      {
        out << static_cast<int>(std::round(point.x));
        out << static_cast<int>(std::round(point.y));
      }
      out << YAML::EndSeq;
    }
    out << YAML::Key << "roi_points" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto &point : roi_points_)
    {
      out << static_cast<int>(std::round(point.x));
      out << static_cast<int>(std::round(point.y));
    }
    out << YAML::EndSeq;
    out << YAML::Key << "tray_name" << YAML::Value << tray_name;
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
    out << YAML::Key << "taught_edge_lengths_cm" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const double edge_length_cm : *latest_tray_edge_lengths_cm_)
    {
      out << edge_length_cm;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "taught_area_cm2" << YAML::Value << latest_tray_area_cm2_.value_or(0.0);
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
      ("tray_" + makeFilenameSafeTrayName(tray_name) + "_" + date_stamp.compact_date + ".yaml");

    std::ofstream dated_file(dated_profile_path);
    if (!dated_file.is_open())
    {
      save_status_message_ = "Save failed";
      save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
      return;
    }
    dated_file << out.c_str() << '\n';
    dated_file.close();

    std::ofstream latest_file(settings_path_);
    if (latest_file.is_open())
    {
      latest_file << out.c_str() << '\n';
      latest_file.close();
    }

    latest_saved_profile_path_ = dated_profile_path.string();
    save_status_message_ = "Tray saved";
    save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
  }

  void onMouse(int event, int x, int y, int /*flags*/)
  {
    const cv::Point ui_point(x, y);

    if (event == cv::EVENT_LBUTTONDOWN)
    {
      if (roi_selection_active_)
      {
        for (const auto &button : buttons_)
        {
          if (button.label == "Add ROI" && button.rect.contains(ui_point))
          {
            pending_roi_points_.clear();
            save_status_message_ = depth_plane_roi_selection_active_
              ? "Depth plane ROI: click 4 corners"
              : "Click 4 ROI corners";
            save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
            return;
          }
          if (button.label == "Clear ROI" && button.rect.contains(ui_point))
          {
            clearLastRoiRegion();
            return;
          }
        }

        const auto image_point = windowPointToImagePoint(ui_point);
        if (!image_point.has_value())
        {
          return;
        }

        pending_roi_points_.push_back(*image_point);
        if (pending_roi_points_.size() == kTeachRoiPointCount)
        {
          const auto new_region_bounds = roiBoundsFromSelection(pending_roi_points_);
          if (new_region_bounds.has_value() && depth_plane_roi_selection_active_)
          {
            cv::Mat depth_for_fit;
            if (!getDepthFrameForPlaneFit(depth_for_fit))
            {
              save_status_message_ = "Depth frame unavailable";
              save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
              pending_roi_points_.clear();
              return;
            }
            DepthPlaneModel fitted_plane;
            if (!fitDepthPlaneFromRoiPoints(depth_for_fit, pending_roi_points_, fitted_plane))
            {
              save_status_message_ = "Invalid depth plane ROI";
              save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
              pending_roi_points_.clear();
              return;
            }
            depth_plane_model_ = fitted_plane;
            depth_plane_roi_bounds_ = *new_region_bounds;
            depth_plane_roi_points_ = pending_roi_points_;
            roi_selection_active_ = false;
            depth_plane_roi_selection_active_ = false;
            save_status_message_ = "Depth plane set";
            save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
            markRuntimeSettingsDirty();
          }
          else if (new_region_bounds.has_value())
          {
            const bool selected_depth_mode = (view_mode_ == ViewMode::kDepth);
            roi_regions_.clear();
            roi_points_ = pending_roi_points_;
            updateDetectionModeFromCurrentView();
            markRuntimeSettingsDirty();
            if (selected_depth_mode)
            {
              clearDepthPlaneReference(false);
              depth_plane_roi_selection_active_ = true;
              roi_selection_active_ = true;
              save_status_message_ = "Depth plane ROI: click 4 corners";
              save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
              markRuntimeSettingsDirty();
            }
            else
            {
              roi_selection_active_ = false;
              depth_plane_roi_selection_active_ = false;
              save_status_message_ = "ROI added";
              save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
            }
            if (!selected_depth_mode && view_mode_ != ViewMode::kBinarized)
            {
              view_mode_ = ViewMode::kBinarized;
              markRuntimeSettingsDirty();
            }
          }
          pending_roi_points_.clear();
          resetTaughtMetrics();
        }
        return;
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

        if (button.label == "Add ROI")
        {
          roi_selection_active_ = true;
          depth_plane_roi_selection_active_ = false;
          pending_roi_points_.clear();
          save_status_message_ = hasValidRoiPoints(roi_points_) ? "Replace ROI: click 4 corners" : "Click 4 ROI corners";
          save_status_deadline_ = this->now() + rclcpp::Duration::from_seconds(1.5);
          markRuntimeSettingsDirty();
          return;
        }
        if (button.label == "Clear ROI")
        {
          clearLastRoiRegion();
          return;
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

      if (save_button_rect_.contains(ui_point))
      {
        saveSettingsToFile();
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
      if (!tray_name_.empty())
      {
        tray_name_.pop_back();
        markRuntimeSettingsDirty();
      }
      return;
    }

    if (tray_name_.size() >= 32)
    {
      return;
    }

    if ((key >= 'a' && key <= 'z') ||
        (key >= 'A' && key <= 'Z') ||
        (key >= '0' && key <= '9') ||
        key == '_' || key == '-' || key == ' ')
    {
      tray_name_.push_back(static_cast<char>(key));
      markRuntimeSettingsDirty();
    }
  }

  std::string sanitizeTrayName() const
  {
    std::string trimmed = tray_name_;
    while (!trimmed.empty() && trimmed.front() == ' ')
    {
      trimmed.erase(trimmed.begin());
    }
    while (!trimmed.empty() && trimmed.back() == ' ')
    {
      trimmed.pop_back();
    }
    return trimmed.empty() ? "tray" : trimmed;
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
      return "View Bin";
    }
    if (button.label == "Overlay")
    {
      return overlay_enabled_ ? "Overlay ON" : "Overlay OFF";
    }
    if (button.label == "Rays")
    {
      return show_rays_enabled_ ? "Rays ON" : "Rays OFF";
    }
    if (button.label == "EdgeDir")
    {
      return detect_black_to_white_ ? "Edge B->W" : "Edge W->B";
    }
    if (button.label == "RayDir")
    {
      return trace_out_to_in_ ? "Ray O->I" : "Ray I->O";
    }
    if (button.label == "Add ROI")
    {
      if (roi_selection_active_)
      {
        return depth_plane_roi_selection_active_
          ? ("Plane " + std::to_string(pending_roi_points_.size()) + "/4")
          : ("ROI " + std::to_string(pending_roi_points_.size()) + "/4");
      }
      return hasValidRoiPoints(roi_points_) ? "Replace ROI" : "Add ROI";
    }
    if (button.label == "Clear ROI")
    {
      return "Clear ROI";
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
      if (button.label == "Add ROI")
      {
        enabled = true;
      }
      if (button.label == "View")
      {
        enabled = true;
      }
      if (button.label == "Clear ROI")
      {
        enabled = roi_selection_active_ || !pending_roi_points_.empty() || hasValidRoiPoints(roi_points_);
      }

      cv::Scalar fill_on(70, 132, 82);
      cv::Scalar border_on(132, 215, 150);
      if (button.label == "Live")
      {
        fill_on = cv::Scalar(70, 132, 82);
        border_on = cv::Scalar(132, 215, 150);
      }
      else if (button.label == "Add ROI")
      {
        fill_on = roi_selection_active_ ? cv::Scalar(70, 126, 186) : cv::Scalar(68, 124, 154);
        border_on = roi_selection_active_ ? cv::Scalar(126, 202, 255) : cv::Scalar(132, 205, 236);
      }
      else if (button.label == "Clear ROI")
      {
        fill_on = cv::Scalar(92, 80, 152);
        border_on = cv::Scalar(166, 152, 245);
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
      else if (button.label == "Rays")
      {
        fill_on = cv::Scalar(86, 118, 150);
        border_on = cv::Scalar(160, 201, 235);
      }
      else if (button.label == "EdgeDir")
      {
        fill_on = cv::Scalar(96, 84, 152);
        border_on = cv::Scalar(175, 160, 240);
      }
      else if (button.label == "RayDir")
      {
        fill_on = cv::Scalar(78, 134, 98);
        border_on = cv::Scalar(142, 222, 168);
      }

      draw_button(button, enabled, fill_on, border_on);
    }

    const cv::Rect status_rect(12, 66, std::max(120, width - 24), std::max(38, kVideoTopBarHeight - 76));
    cv::rectangle(bar, status_rect, cv::Scalar(35, 37, 41), cv::FILLED);
    cv::rectangle(bar, status_rect, cv::Scalar(72, 77, 84), 1);
    const bool show_save_status = this->now() < save_status_deadline_;
    const std::string roi_status = roi_selection_active_
      ? (
      depth_plane_roi_selection_active_
      ? ("Depth plane " + std::to_string(pending_roi_points_.size()) + "/4")
      : ("ROI selecting " + std::to_string(pending_roi_points_.size()) + "/4"))
      : (
      !hasValidRoiPoints(roi_points_)
      ? "ROI required"
      : ((detection_use_depth_ && !hasDepthPlaneReference()) ? "Depth plane ROI required" : "ROI ready"));
    const std::string runtime_status =
      show_save_status
      ? save_status_message_
      : ("Ready | " + roi_status + " | View " + currentViewLabel());
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
      "Tray Teach Console",
      cv::Point(24, 25),
      cv::FONT_HERSHEY_DUPLEX,
      0.62,
      cv::Scalar(236, 239, 244),
      1,
      cv::LINE_AA);

    const cv::Rect setup_card(12, 34, kLeftPanelWidth - 24, 170);
    draw_card(setup_card, "Tray Setup");
    cv::putText(
      panel,
      "Tray Name",
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
    const std::string tray_name_text = sanitizeTrayName();
    cv::putText(
      panel,
      fit_text(tray_name_text, name_box_rect_.width - 20, 0.60, 1),
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
        fit_text(tray_name_text, name_box_rect_.width - 20, 0.60, 1),
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

    const bool save_ready = hasValidRoiPoints(roi_points_) &&
      latest_tray_edge_lengths_cm_.has_value() &&
      (!detection_use_depth_ || hasDepthPlaneReference());
    const cv::Scalar save_fill = save_ready ? cv::Scalar(70, 132, 82) : cv::Scalar(66, 70, 76);
    const cv::Scalar save_border = save_ready ? cv::Scalar(132, 215, 150) : cv::Scalar(102, 106, 112);
    cv::rectangle(panel, save_button_rect_, save_fill, cv::FILLED);
    cv::rectangle(panel, save_button_rect_, save_border, 2);
    cv::putText(
      panel,
      "Save Tray Profile",
      cv::Point(save_button_rect_.x + 12, save_button_rect_.y + 25),
      cv::FONT_HERSHEY_DUPLEX,
      0.50,
      cv::Scalar(245, 245, 245),
      1,
      cv::LINE_AA);

    const bool show_save_status = this->now() < save_status_deadline_;
    if (show_save_status)
    {
      cv::putText(
        panel,
        fit_text(save_status_message_, setup_card.width - save_button_rect_.width - 46, 0.43, 1),
        cv::Point(save_button_rect_.x + save_button_rect_.width + 12, save_button_rect_.y + 24),
        cv::FONT_HERSHEY_DUPLEX,
        0.43,
        save_status_message_ == "Tray saved" ? cv::Scalar(150, 232, 165) : cv::Scalar(236, 180, 126),
        1,
        cv::LINE_AA);
    }

    const int slider_card_y = std::max(
      200,
      sliders_.empty() ? 200 : (sliders_.front().track_rect.y - 54));
    const int available_slider_h = std::max(180, panel.rows - slider_card_y - 12);
    const int target_slider_h = sliders_.empty() ? 280 : (sliders_.back().track_rect.y + 34 - slider_card_y);
    const int slider_card_h = std::clamp(target_slider_h, 180, available_slider_h);
    const cv::Rect slider_card(12, slider_card_y, kLeftPanelWidth - 24, slider_card_h);
    draw_card(slider_card, "Detection Tuning");

    for (const auto &slider : sliders_)
    {
      const int value = slider.value ? *slider.value : 0;
      const bool slider_enabled = isSliderEnabled(slider);
      cv::Scalar accent(120, 200, 255);
      if (slider.label == kRedTrackbar)
      {
        accent = cv::Scalar(70, 92, 240);
      }
      else if (slider.label == kGreenTrackbar)
      {
        accent = cv::Scalar(80, 210, 110);
      }
      else if (slider.label == kBlueTrackbar)
      {
        accent = cv::Scalar(240, 140, 75);
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
      cv::putText(
        panel,
        std::to_string(value),
        cv::Point(slider.track_rect.x + slider.track_rect.width - 36, slider.track_rect.y - 10),
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

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = color_cv->image.clone();
    latest_header_ = msg->header;
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

    cv::Mat mask;
    if (detection_use_depth_)
    {
      if (hasDepthPlaneReference())
      {
        const cv::Mat normalized_depth = applyFixedDepthPlaneNormalization(current_depth, depth_plane_model_);
        mask = buildDepthMask(
          normalized_depth,
          depth_threshold_mm_,
          depth_plane_model_.reference_depth_m);
      }
      else if (!current_depth.empty() && current_depth.type() == CV_32FC1)
      {
        mask = cv::Mat::zeros(current_depth.size(), CV_8UC1);
      }
    }
    else
    {
      mask = buildRgbMask(
        current_frame,
        red_threshold_,
        green_threshold_,
        blue_threshold_);
    }

    const bool roi_ready = hasValidRoiPoints(roi_points_);
    cv::Mat detection_mask = mask.clone();
    if (roi_ready)
    {
      const cv::Mat roi_mask = buildRoiMask(mask.size(), roi_points_);
      cv::bitwise_and(detection_mask, roi_mask, detection_mask);
    }

    std::optional<TrayEstimate> tray_estimate;
    if (roi_ready)
    {
      tray_estimate = detectTrayFromAxisAlignedRoi(
        detection_mask,
        current_depth,
        current_camera_info,
        roi_points_,
        ray_step_px_,
        depth_edge_offset_px_,
        previous_color_percent_,
        horizontal_ray_count_,
        vertical_ray_count_,
        outlier_sensitivity_,
        detect_black_to_white_,
        trace_out_to_in_);
    }

    if (tray_estimate.has_value() && tray_estimate->has_metric_estimate)
    {
      latest_tray_edge_lengths_cm_ = tray_estimate->edge_lengths_cm;
      if (tray_estimate->area_cm2 > 0.0)
      {
        latest_tray_area_cm2_ = tray_estimate->area_cm2;
      }
      else
      {
        latest_tray_area_cm2_.reset();
      }
    }

    cv::Mat display_image;
    switch (view_mode_)
    {
      case ViewMode::kRgb:
        display_image = current_frame.clone();
        break;
      case ViewMode::kDepth:
        display_image = colorizeDepth(current_depth);
        if (display_image.size() != current_frame.size())
        {
          cv::resize(display_image, display_image, current_frame.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        break;
      case ViewMode::kBinarized:
      default:
        cv::cvtColor(roi_ready ? detection_mask : mask, display_image, cv::COLOR_GRAY2BGR);
        break;
    }
    if (overlay_enabled_ && hasValidRoiPoints(roi_points_))
    {
      drawRoiOverlay(display_image, roi_points_, false);
    }
    if (overlay_enabled_ && !depth_plane_roi_points_.empty())
    {
      drawRoiOverlay(display_image, depth_plane_roi_points_, false);
      if (depth_plane_roi_points_.size() >= 2)
      {
        const cv::Point text_origin(
          static_cast<int>(std::round(depth_plane_roi_points_[0].x)) + 8,
          static_cast<int>(std::round(depth_plane_roi_points_[0].y)) + 24);
        cv::putText(
          display_image,
          "Depth Plane ROI",
          text_origin,
          cv::FONT_HERSHEY_SIMPLEX,
          0.52,
          cv::Scalar(0, 255, 255),
          2,
          cv::LINE_AA);
      }
    }
    if (roi_selection_active_ && !pending_roi_points_.empty())
    {
      drawRoiOverlay(display_image, pending_roi_points_, true);
    }
    if (show_rays_enabled_)
    {
      drawRays(display_image, roi_points_, horizontal_ray_count_, vertical_ray_count_);
    }
    if (overlay_enabled_)
    {
      drawTrayEstimate(display_image, tray_estimate, depth_edge_offset_px_);
      drawMeasurementAxes(display_image, tray_estimate);
    }
    drawViewLabel(display_image, currentViewLabel());
    drawModeLabel(display_image, detection_use_depth_ ? "Depth" : "RGB");
    if (!roi_ready)
    {
      cv::putText(
        display_image,
        roi_selection_active_ ? "Click 4 ROI corners" : "ROI required before edge detect",
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
        "Depth plane ROI required",
        cv::Point(24, 72),
        cv::FONT_HERSHEY_SIMPLEX,
        0.80,
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

    if (publish_overlay_)
    {
      cv_bridge::CvImage overlay_image;
      overlay_image.header = current_header;
      overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_image.image = combined;
      overlay_pub_->publish(*overlay_image.toImageMsg());
    }
  }

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string joint_states_topic_;
  std::string overlay_topic_;
  std::string profiles_dir_;
  std::string settings_path_;
  std::string runtime_settings_path_;
  std::string latest_saved_profile_path_;
  std::string save_status_message_ {"Saved"};
  std::string tray_name_ {"tray"};
  rclcpp::Time save_status_deadline_ {0, 0, RCL_ROS_TIME};
  bool publish_overlay_ {true};
  double display_scale_ {1.0};
  cv::Size rendered_window_size_ {};

  int red_threshold_ {120};
  int green_threshold_ {120};
  int blue_threshold_ {120};
  int depth_threshold_mm_ {10};
  int ray_step_px_ {3};
  int depth_edge_offset_px_ {4};
  int previous_color_percent_ {kDefaultPreviousColorPercent};
  int horizontal_ray_count_ {50};
  int vertical_ray_count_ {50};
  int outlier_sensitivity_ {50};
  bool live_view_enabled_ {true};
  ViewMode view_mode_ {ViewMode::kBinarized};
  bool overlay_enabled_ {true};
  bool show_rays_enabled_ {false};
  bool detection_use_depth_ {false};
  bool detect_black_to_white_ {true};
  bool trace_out_to_in_ {false};
  bool roi_selection_active_ {false};
  bool depth_plane_roi_selection_active_ {false};
  bool freeze_latched_ {false};
  bool name_edit_active_ {false};
  std::optional<double> latest_tray_area_cm2_;
  std::optional<std::array<double, 4>> latest_tray_edge_lengths_cm_;
  bool runtime_settings_dirty_ {false};
  rclcpp::Time last_runtime_settings_save_time_ {0, 0, RCL_ROS_TIME};
  std::vector<UiButton> buttons_;
  std::vector<UiSlider> sliders_;
  std::vector<AxisAlignedRoiBounds> roi_regions_;
  std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds_;
  std::vector<cv::Point2f> roi_points_;
  std::vector<cv::Point2f> depth_plane_roi_points_;
  std::vector<cv::Point2f> pending_roi_points_;
  DepthPlaneModel depth_plane_model_;
  cv::Rect name_box_rect_;
  cv::Rect save_button_rect_;
  int active_slider_index_ {-1};

  rclcpp::Publisher<ImageMsg>::SharedPtr overlay_pub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr render_timer_;

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
  auto node = std::make_shared<TrayDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
