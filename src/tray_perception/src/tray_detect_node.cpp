#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <deque>
#include <exception>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dobot_msgs_v4/msg/tray_vector.hpp>
#include <dobot_msgs_v4/srv/get_tray_dimensions.hpp>
#include <dobot_msgs_v4/srv/mov_j.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

#include <dobot_common/workspace_paths.hpp>

namespace
{
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using MovJSrv = dobot_msgs_v4::srv::MovJ;
using GetTrayDimensionsSrv = dobot_msgs_v4::srv::GetTrayDimensions;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PolygonStampedMsg = geometry_msgs::msg::PolygonStamped;
using TrayVectorMsg = dobot_msgs_v4::msg::TrayVector;
using MarkerMsg = visualization_msgs::msg::Marker;
using TriggerSrv = std_srvs::srv::Trigger;

constexpr double kMinOutlierDistancePx = 4.0;
constexpr int kMaxSideTrimIterations = 24;
constexpr int kDefaultPreviousColorPercent = 60;
constexpr double kNextColorConfirmMatchRatio = 0.60;
constexpr char kDetectWindowName[] = "tray_detect_view";
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

struct TrayOverlayAxes
{
  cv::Point2f origin;
  cv::Point2f x_dir;
  cv::Point2f y_dir;
  int origin_idx {-1};
  int x_idx {-1};
  int y_idx {-1};
};

struct TrayMetricEstimate
{
  double area_cm2 {0.0};
  double mean_depth_m {0.0};
  // Ordered as: origin X edge, opposite X edge, origin Y edge, opposite Y edge.
  std::array<double, 4> edge_lengths_cm {0.0, 0.0, 0.0, 0.0};
};

struct TrayPose3D
{
  cv::Vec3d origin;
  cv::Matx33d rotation;
};

struct TimedTrayPose3D
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  TrayPose3D pose;
  std::string frame_id;
};

struct SeekCapture
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  TrayPose3D pose;
  std::string frame_id;
  cv::Mat frame;
};

struct SeekMotionSample
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  TrayPose3D pose;
};

struct TimedTrayEstimate
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  TrayEstimate estimate;
};

struct AxisAlignedRoiBounds
{
  int left {0};
  int top {0};
  int right {0};
  int bottom {0};
};

bool isValidRoiBounds(const AxisAlignedRoiBounds &bounds);
std::optional<AxisAlignedRoiBounds> roiBoundsFromSelection(const std::vector<cv::Point2f> &points);
std::vector<cv::Point2f> roiPointsFromBounds(const AxisAlignedRoiBounds &bounds);
std::optional<AxisAlignedRoiBounds> combinedRoiBounds(const std::vector<AxisAlignedRoiBounds> &roi_regions);
std::vector<cv::Point2f> mergeRoiRegionsIntoPolygon(const std::vector<AxisAlignedRoiBounds> &roi_regions);
int lowerLeftCornerIndex(const std::vector<cv::Point2f> &corners);
std::optional<TrayOverlayAxes> computeTrayOverlayAxes(const TrayEstimate &estimate);
double medianValue(std::vector<double> values);

struct TrayProfile
{
  std::filesystem::path path;
  std::string tray_name;
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
  int ray_step_px {3};
  int depth_edge_offset_px {4};
  int previous_color_percent {kDefaultPreviousColorPercent};
  int horizontal_ray_count {50};
  int vertical_ray_count {50};
  int outlier_sensitivity {50};
  bool detect_black_to_white {true};
  bool trace_out_to_in {false};
  bool depth_plane_enabled {false};
  double depth_plane_a {0.0};
  double depth_plane_b {0.0};
  double depth_plane_c {0.0};
  double depth_plane_reference_depth_m {0.0};
  std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds;
  std::vector<AxisAlignedRoiBounds> roi_regions;
  std::vector<cv::Point2f> roi_points;
  std::array<double, 6> teach_joints_deg {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_teach_joints {false};
  std::array<double, 4> taught_edge_lengths_cm {0.0, 0.0, 0.0, 0.0};
  double taught_area_cm2 {0.0};
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
  const std::string &tray_name,
  const std::string &teach_date,
  const std::filesystem::path &path)
{
  const std::string name = tray_name.empty() ? path.stem().string() : tray_name;
  if (!teach_date.empty())
  {
    return name + " | " + teach_date;
  }
  if (path.empty())
  {
    return name.empty() ? "Select tray profile" : name;
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

std::optional<TrayProfile> loadTrayProfileFile(const std::filesystem::path &path)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node params = root["tray_detect"]["ros__parameters"];
    if (!params || !params.IsMap())
    {
      return std::nullopt;
    }

    TrayProfile profile;
    profile.path = path;
    profile.color_topic = params["color_topic"] ? params["color_topic"].as<std::string>() : "/robot_camera/color/image_raw";
    profile.depth_topic = params["depth_topic"] ? params["depth_topic"].as<std::string>() : "/robot_camera/depth/image_raw";
    profile.camera_info_topic = params["camera_info_topic"] ? params["camera_info_topic"].as<std::string>() : "/robot_camera/color/camera_info";
    profile.overlay_topic = params["overlay_topic"] ? params["overlay_topic"].as<std::string>() : "tray_overlay";
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
    profile.detect_black_to_white = params["detect_black_to_white"] ? params["detect_black_to_white"].as<bool>() : true;
    profile.trace_out_to_in = params["trace_out_to_in"] ? params["trace_out_to_in"].as<bool>() : false;
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
    if (
      !std::isfinite(profile.depth_plane_a) ||
      !std::isfinite(profile.depth_plane_b) ||
      !std::isfinite(profile.depth_plane_c) ||
      !std::isfinite(profile.depth_plane_reference_depth_m) ||
      profile.depth_plane_reference_depth_m <= 0.0 ||
      !profile.depth_plane_roi_bounds.has_value())
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
    profile.tray_name = params["tray_name"] ? params["tray_name"].as<std::string>() : path.stem().string();
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
    if (const YAML::Node taught_edge_lengths = params["taught_edge_lengths_cm"];
        taught_edge_lengths && taught_edge_lengths.IsSequence())
    {
      for (std::size_t i = 0; i < profile.taught_edge_lengths_cm.size() && i < taught_edge_lengths.size(); ++i)
      {
        profile.taught_edge_lengths_cm[i] = taught_edge_lengths[i].as<double>();
      }
    }
    profile.taught_area_cm2 = params["taught_area_cm2"] ? params["taught_area_cm2"].as<double>() : 0.0;
    if (profile.roi_points.size() < 4 && !profile.roi_regions.empty())
    {
      profile.roi_points = mergeRoiRegionsIntoPolygon(profile.roi_regions);
    }
    else if (
      profile.roi_regions.empty() &&
      profile.roi_points.size() >= 2 &&
      profile.roi_points.size() <= 4)
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
    profile.display_label = buildProfileLabel(profile.tray_name, profile.teach_date, path);
    return profile;
  }
  catch (const YAML::Exception &)
  {
    return std::nullopt;
  }
}

std::vector<TrayProfile> loadTrayProfilesFromDirectory(const std::filesystem::path &profiles_dir)
{
  std::vector<TrayProfile> profiles;
  if (profiles_dir.empty() || !std::filesystem::exists(profiles_dir))
  {
    return profiles;
  }

  std::vector<TrayProfile> alias_profiles;
  for (const auto &entry : std::filesystem::directory_iterator(profiles_dir))
  {
    if (!entry.is_regular_file())
    {
      continue;
    }

    const std::string extension = entry.path().extension().string();
    if (extension != ".yaml" && extension != ".yml")
    {
      continue;
    }

    const auto profile = loadTrayProfileFile(entry.path());
    if (!profile.has_value())
    {
      continue;
    }

    if (entry.path().filename() == "tray_teach_settings.yaml")
    {
      alias_profiles.push_back(*profile);
    }
    else
    {
      profiles.push_back(*profile);
    }
  }

  auto by_recent_date = [](const TrayProfile &a, const TrayProfile &b)
  {
    if (a.teach_date != b.teach_date)
    {
      return a.teach_date > b.teach_date;
    }
    return a.path.filename().string() < b.path.filename().string();
  };

  std::sort(profiles.begin(), profiles.end(), by_recent_date);
  std::sort(alias_profiles.begin(), alias_profiles.end(), by_recent_date);

  auto same_teach_profile = [](const TrayProfile &a, const TrayProfile &b)
  {
    const bool has_edges_a = hasValidEdgeLengthsCm(a.taught_edge_lengths_cm);
    const bool has_edges_b = hasValidEdgeLengthsCm(b.taught_edge_lengths_cm);
    const bool edge_lengths_match =
      has_edges_a && has_edges_b &&
      std::equal(
        a.taught_edge_lengths_cm.begin(),
        a.taught_edge_lengths_cm.end(),
        b.taught_edge_lengths_cm.begin(),
        [](double lhs, double rhs)
        {
          return std::fabs(lhs - rhs) < 1e-6;
        });
    return a.tray_name == b.tray_name &&
      a.teach_date == b.teach_date &&
      (edge_lengths_match || std::fabs(a.taught_area_cm2 - b.taught_area_cm2) < 1e-6);
  };

  for (const auto &alias_profile : alias_profiles)
  {
    const auto existing_it = std::find_if(
      profiles.begin(),
      profiles.end(),
      [&](const TrayProfile &profile)
      {
        return same_teach_profile(profile, alias_profile);
      });

    if (existing_it == profiles.end())
    {
      profiles.push_back(alias_profile);
      continue;
    }

    const bool existing_missing_roi = existing_it->roi_points.size() < 4;
    const bool alias_has_roi = alias_profile.roi_points.size() >= 4;
    if (existing_missing_roi && alias_has_roi)
    {
      *existing_it = alias_profile;
    }
  }

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

cv::Vec3d rotationColumn(const cv::Matx33d &rotation, int column)
{
  return cv::Vec3d(rotation(0, column), rotation(1, column), rotation(2, column));
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

std::optional<TrayPose3D> averageTimedTrayPoses(const std::deque<TimedTrayPose3D> &pose_samples)
{
  if (pose_samples.empty())
  {
    return std::nullopt;
  }

  const TrayPose3D &reference_pose = pose_samples.back().pose;
  const cv::Vec3d ref_x = rotationColumn(reference_pose.rotation, 0);
  const cv::Vec3d ref_y = rotationColumn(reference_pose.rotation, 1);
  const cv::Vec3d ref_z = rotationColumn(reference_pose.rotation, 2);

  cv::Vec3d origin_sum(0.0, 0.0, 0.0);
  cv::Vec3d x_sum(0.0, 0.0, 0.0);
  cv::Vec3d y_sum(0.0, 0.0, 0.0);
  cv::Vec3d z_sum(0.0, 0.0, 0.0);
  for (const auto &sample : pose_samples)
  {
    origin_sum += sample.pose.origin;

    cv::Vec3d x_axis = rotationColumn(sample.pose.rotation, 0);
    cv::Vec3d y_axis = rotationColumn(sample.pose.rotation, 1);
    cv::Vec3d z_axis = rotationColumn(sample.pose.rotation, 2);
    if (x_axis.dot(ref_x) < 0.0)
    {
      x_axis *= -1.0;
    }
    if (y_axis.dot(ref_y) < 0.0)
    {
      y_axis *= -1.0;
    }
    if (z_axis.dot(ref_z) < 0.0)
    {
      z_axis *= -1.0;
    }

    x_sum += x_axis;
    y_sum += y_axis;
    z_sum += z_axis;
  }

  cv::Vec3d filtered_x = x_sum;
  if (!normalizeVectorInPlace(filtered_x))
  {
    filtered_x = ref_x;
  }

  cv::Vec3d filtered_y = y_sum - filtered_x * filtered_x.dot(y_sum);
  if (!normalizeVectorInPlace(filtered_y))
  {
    filtered_y = ref_y - filtered_x * filtered_x.dot(ref_y);
    if (!normalizeVectorInPlace(filtered_y))
    {
      return reference_pose;
    }
  }

  cv::Vec3d filtered_z = filtered_x.cross(filtered_y);
  if (!normalizeVectorInPlace(filtered_z))
  {
    filtered_z = z_sum;
    if (!normalizeVectorInPlace(filtered_z))
    {
      filtered_z = ref_z;
      if (!normalizeVectorInPlace(filtered_z))
      {
        return reference_pose;
      }
    }
    filtered_y = filtered_z.cross(filtered_x);
    if (!normalizeVectorInPlace(filtered_y))
    {
      return reference_pose;
    }
  }

  if (filtered_z.dot(ref_z) < 0.0)
  {
    filtered_y *= -1.0;
    filtered_z = filtered_x.cross(filtered_y);
    if (!normalizeVectorInPlace(filtered_z))
    {
      return reference_pose;
    }
  }

  TrayPose3D filtered_pose;
  filtered_pose.origin = origin_sum * (1.0 / static_cast<double>(pose_samples.size()));
  filtered_pose.rotation = cv::Matx33d(
    filtered_x[0], filtered_y[0], filtered_z[0],
    filtered_x[1], filtered_y[1], filtered_z[1],
    filtered_x[2], filtered_y[2], filtered_z[2]);
  return filtered_pose;
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

struct PlaneFit3D
{
  cv::Vec3d point;
  cv::Vec3d normal;
};

std::optional<PlaneFit3D> fitPlaneToCameraPoints(const std::vector<cv::Vec3d> &points)
{
  if (points.size() < 3)
  {
    return std::nullopt;
  }

  cv::Vec3d mean(0.0, 0.0, 0.0);
  for (const auto &point : points)
  {
    mean += point;
  }
  mean *= (1.0 / static_cast<double>(points.size()));

  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (const auto &point : points)
  {
    const cv::Vec3d delta = point - mean;
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        covariance(row, col) += delta[row] * delta[col];
      }
    }
  }
  covariance *= (1.0 / static_cast<double>(points.size()));

  cv::Mat eigenvalues;
  cv::Mat eigenvectors;
  if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors) || eigenvectors.rows != 3 || eigenvectors.cols != 3)
  {
    return std::nullopt;
  }

  cv::Vec3d normal(
    eigenvectors.at<double>(2, 0),
    eigenvectors.at<double>(2, 1),
    eigenvectors.at<double>(2, 2));
  const double normal_norm = vectorNorm(normal);
  if (normal_norm < 1e-9)
  {
    return std::nullopt;
  }
  normal *= (1.0 / normal_norm);

  return PlaneFit3D{mean, normal};
}

std::optional<cv::Vec3d> intersectPixelRayWithPlane(
  const cv::Point2f &pixel,
  const PlaneFit3D &plane,
  const CameraInfoMsg &camera_info)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];
  if (fx <= 1e-6 || fy <= 1e-6)
  {
    return std::nullopt;
  }

  const cv::Vec3d ray(
    (static_cast<double>(pixel.x) - cx) / fx,
    (static_cast<double>(pixel.y) - cy) / fy,
    1.0);
  const double denominator = plane.normal.dot(ray);
  if (std::fabs(denominator) < 1e-9)
  {
    return std::nullopt;
  }

  const double scale = plane.normal.dot(plane.point) / denominator;
  if (!std::isfinite(scale) || scale <= 1e-6)
  {
    return std::nullopt;
  }
  return ray * scale;
}

std::optional<std::array<cv::Vec3d, 4>> estimateTrayCornerCameraPointsFromDepthLines(
  const std::vector<cv::Point2f> &corners,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px)
{
  if (corners.size() != 4 || depth_m.empty() || depth_m.type() != CV_32FC1)
  {
    return std::nullopt;
  }
  if (camera_info.k[0] <= 1e-6 || camera_info.k[4] <= 1e-6)
  {
    return std::nullopt;
  }

  std::vector<cv::Vec3d> camera_points;
  camera_points.reserve(256);
  for (const auto &segment : buildDepthMeasurementSegments(corners, depth_edge_offset_px))
  {
    const cv::Point2f delta = segment.second - segment.first;
    const double length_px = std::sqrt(static_cast<double>(delta.dot(delta)));
    const int sample_count = std::max(2, static_cast<int>(std::ceil(length_px / 2.0)));
    for (int i = 0; i <= sample_count; ++i)
    {
      const float t = static_cast<float>(i) / static_cast<float>(sample_count);
      const cv::Point2f pixel = segment.first + t * delta;
      const auto depth = averageDepthAt(depth_m, pixel, 5);
      if (!depth.has_value())
      {
        continue;
      }
      camera_points.push_back(projectPixelToCamera(pixel, *depth, camera_info));
    }
  }

  if (camera_points.size() < 12)
  {
    return std::nullopt;
  }

  auto plane = fitPlaneToCameraPoints(camera_points);
  if (!plane.has_value())
  {
    return std::nullopt;
  }

  std::vector<double> distances;
  distances.reserve(camera_points.size());
  for (const auto &point : camera_points)
  {
    distances.push_back(std::fabs((point - plane->point).dot(plane->normal)));
  }
  const double median_distance = medianValue(distances);
  const double threshold = std::max(0.004, 3.0 * median_distance);

  std::vector<cv::Vec3d> filtered_points;
  filtered_points.reserve(camera_points.size());
  for (std::size_t i = 0; i < camera_points.size(); ++i)
  {
    if (distances[i] <= threshold)
    {
      filtered_points.push_back(camera_points[i]);
    }
  }
  if (filtered_points.size() >= 12 && filtered_points.size() < camera_points.size())
  {
    plane = fitPlaneToCameraPoints(filtered_points);
    if (!plane.has_value())
    {
      return std::nullopt;
    }
  }

  std::array<cv::Vec3d, 4> corner_points;
  for (std::size_t i = 0; i < corners.size(); ++i)
  {
    const auto point = intersectPixelRayWithPlane(corners[i], *plane, camera_info);
    if (!point.has_value())
    {
      return std::nullopt;
    }
    corner_points[i] = *point;
  }
  return corner_points;
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
    polygon.emplace_back(static_cast<int>(std::round(corner.x)), static_cast<int>(std::round(corner.y)));
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

std::optional<TrayEstimate> filterTimedTrayEstimates(
  const std::deque<TimedTrayEstimate> &estimate_history,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px)
{
  if (estimate_history.empty())
  {
    return std::nullopt;
  }

  const TrayEstimate &reference_estimate = estimate_history.back().estimate;
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

  TrayEstimate filtered_estimate = reference_estimate;
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
  if (const auto metrics = estimateTrayMetricsFromCorners(
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

  int lower_idx = corner_indices[0];
  if (corners[corner_indices[1]].x < corners[corner_indices[0]].x)
  {
    lower_idx = corner_indices[1];
  }
  return lower_idx;
}

std::optional<TrayPose3D> estimateTrayPose3D(
  const TrayEstimate &estimate,
  const cv::Mat &depth_m,
  const CameraInfoMsg &camera_info,
  int depth_edge_offset_px)
{
  auto camera_points = estimateTrayCornerCameraPointsFromDepthLines(
    estimate.corners,
    depth_m,
    camera_info,
    depth_edge_offset_px);
  if (!camera_points.has_value())
  {
    camera_points = estimateTrayCornerCameraPoints(estimate.corners, depth_m, camera_info);
  }
  if (!camera_points.has_value())
  {
    return std::nullopt;
  }

  const auto overlay_axes = computeTrayOverlayAxes(estimate);
  if (!overlay_axes.has_value() ||
      overlay_axes->origin_idx < 0 ||
      overlay_axes->x_idx < 0 ||
      overlay_axes->y_idx < 0)
  {
    return std::nullopt;
  }

  const cv::Vec3d origin = (*camera_points)[overlay_axes->origin_idx];
  const cv::Vec3d x_edge = (*camera_points)[overlay_axes->x_idx] - origin;
  const cv::Vec3d y_edge = (*camera_points)[overlay_axes->y_idx] - origin;
  cv::Vec3d x_axis = x_edge;
  if (!normalizeVectorInPlace(x_axis))
  {
    return std::nullopt;
  }

  cv::Vec3d z_axis = x_axis.cross(y_edge);
  if (!normalizeVectorInPlace(z_axis))
  {
    return std::nullopt;
  }

  cv::Vec3d y_axis = z_axis.cross(x_axis);
  if (!normalizeVectorInPlace(y_axis))
  {
    return std::nullopt;
  }

  TrayPose3D pose;
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

  for (const auto &point : estimate->filtered_edge_points)
  {
    cv::circle(binary_bgr, point, 4, cv::Scalar(0, 165, 255), -1);
  }

  const std::string area_label =
    estimate->has_metric_estimate && estimate->area_cm2 > 0.0
      ? cv::format("tray area=%.0f mm2", estimate->area_cm2 * kSquareCentimetersToSquareMillimeters)
      : "tray area=n/a";
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

void drawTrayName(cv::Mat &image, const std::string &tray_name)
{
  if (tray_name.empty())
  {
    return;
  }

  cv::putText(
    image,
    tray_name,
    cv::Point(18, 42),
    cv::FONT_HERSHEY_SIMPLEX,
    1.08,
    cv::Scalar(0, 255, 255),
    3);
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

std::optional<TrayOverlayAxes> computeTrayOverlayAxes(const TrayEstimate &estimate)
{
  if (estimate.corners.size() != 4)
  {
    return std::nullopt;
  }

  const int origin_idx = lowerLeftCornerIndex(estimate.corners);
  if (origin_idx < 0)
  {
    return std::nullopt;
  }

  const cv::Point2f origin = estimate.corners[origin_idx];
  const int prev_idx = (origin_idx + 3) % 4;
  const int next_idx = (origin_idx + 1) % 4;
  const cv::Point2f prev_corner = estimate.corners[prev_idx];
  const cv::Point2f next_corner = estimate.corners[next_idx];

  cv::Point2f dir_a = prev_corner - origin;
  cv::Point2f dir_b = next_corner - origin;

  const float len_a = std::sqrt(dir_a.dot(dir_a));
  const float len_b = std::sqrt(dir_b.dot(dir_b));
  if (len_a < 1e-3F || len_b < 1e-3F)
  {
    return std::nullopt;
  }

  const bool prev_is_x = len_a >= len_b;
  cv::Point2f x_dir = prev_is_x ? dir_a : dir_b;
  cv::Point2f y_dir = prev_is_x ? dir_b : dir_a;
  const float x_norm = std::sqrt(x_dir.dot(x_dir));
  const float y_norm = std::sqrt(y_dir.dot(y_dir));

  x_dir *= (1.0F / x_norm);
  y_dir *= (1.0F / y_norm);

  TrayOverlayAxes axes;
  axes.origin = origin;
  axes.x_dir = x_dir;
  axes.y_dir = y_dir;
  axes.origin_idx = origin_idx;
  axes.x_idx = prev_is_x ? prev_idx : next_idx;
  axes.y_idx = prev_is_x ? next_idx : prev_idx;
  return axes;
}

void drawTrayAxes(cv::Mat &image, const std::optional<TrayEstimate> &estimate)
{
  if (!estimate.has_value())
  {
    return;
  }

  const auto axes = computeTrayOverlayAxes(*estimate);
  if (!axes.has_value())
  {
    return;
  }

  const cv::Point2f origin = axes->origin;
  const cv::Point2f x_dir = axes->x_dir;
  const cv::Point2f y_dir = axes->y_dir;
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
}  // namespace

class TrayDetectNode : public rclcpp::Node
{
public:
  enum class DisplayView
  {
    kRgb = 0,
    kBinarized,
    kDepth,
  };

  TrayDetectNode()
  : Node("tray_detect")
  {
    profiles_dir_ = declare_parameter<std::string>(
      "profiles_dir",
      dobot_common::paths::workspacePath({"teach", "tray_teach"}, __FILE__).string());
    color_topic_ = declare_parameter<std::string>("color_topic", "/robot_camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/robot_camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/robot_camera/color/camera_info");
    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "tray_overlay");
    tray_pose_topic_ = declare_parameter<std::string>("tray_pose_topic", "tray_pose");
    tray_axis_overlay_topic_ = declare_parameter<std::string>("tray_axis_overlay_topic", "tray_axis_overlay");
    tray_vector_topic_ = declare_parameter<std::string>("tray_vector_topic", "tray_vector");
    tray_cube_marker_topic_ = declare_parameter<std::string>("tray_cube_marker_topic", "tray_cube_marker");
    tray_dimensions_service_name_ = declare_parameter<std::string>(
      "tray_dimensions_service",
      "tray_detect/get_tray_dimensions");
    seek_service_name_ = declare_parameter<std::string>("seek_service", "tray_detect/seek");
    seek_complete_service_name_ = declare_parameter<std::string>(
      "seek_complete_service",
      "tray_detect/seek_complete");
    seek_status_service_name_ = declare_parameter<std::string>(
      "seek_status_service",
      "tray_detect/seek_status");
    go_to_teach_service_name_ = declare_parameter<std::string>(
      "go_to_teach_service",
      "tray_detect/go_to_teach");
    publish_tray_cube_marker_ = declare_parameter<bool>("publish_tray_cube_marker", true);
    tray_thickness_mm_ = std::max(0.1, declare_parameter<double>("tray_thickness_mm", 15.0));
    movj_service_name_ = declare_parameter<std::string>("movj_service", "/dobot_bringup_ros2/srv/MovJ");
    use_calibration_ = declare_parameter<bool>("use_calibration", true);
    publish_static_calibration_tf_ = declare_parameter<bool>("publish_static_calibration_tf", true);
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    calibration_parent_frame_ = declare_parameter<std::string>("calibration_parent_frame", "Link6");
    calibration_child_frame_ = declare_parameter<std::string>(
      "calibration_child_frame", "calibrated_camera_link");
    calibration_dir_ = declare_parameter<std::string>(
      "calibration_dir", defaultCalibrationDir());
    calibration_file_ = declare_parameter<std::string>("calibration_file", "");
    auto_discover_calibration_ = declare_parameter<bool>("auto_discover_calibration", true);
    const std::string runtime_settings_file_param = declare_parameter<std::string>(
      "runtime_settings_file",
      defaultRuntimeSettingsFile());
    runtime_settings_path_ = resolvePath(runtime_settings_file_param);
    const std::string default_camera_frame =
      use_calibration_ ? calibration_child_frame_ : std::string("");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame", default_camera_frame);
    tray_frame_id_ = declare_parameter<std::string>("tray_frame_id", "tray");
    motion_update_period_sec_ = declare_parameter<double>("motion_update_period_sec", 0.1);
    pose_filter_window_sec_ = std::max(
      0.1,
      declare_parameter<double>("pose_filter_window_sec", 0.8));
    pose_filter_min_samples_ = std::clamp(
      static_cast<int>(declare_parameter<int>("pose_filter_min_samples", 3)),
      2,
      30);
    pose_outlier_position_mm_ = std::clamp(
      declare_parameter<double>("pose_outlier_position_mm", 20.0),
      1.0,
      200.0);
    pose_outlier_angle_deg_ = std::clamp(
      declare_parameter<double>("pose_outlier_angle_deg", 12.0),
      1.0,
      45.0);
    const double seek_window_sec = declare_parameter<double>("seek_window_sec", 60.0);
    seek_window_tenths_ = std::clamp(static_cast<int>(std::round(seek_window_sec)), 1, 60);
    const double seek_decay_sec = declare_parameter<double>("seek_decay_sec", 1.0);
    seek_decay_tenths_ = std::clamp(static_cast<int>(std::round(seek_decay_sec * 10.0)), 1, 10);
    seek_valid_confidence_frames_ = std::clamp(
      static_cast<int>(declare_parameter<int>("seek_valid_frames_confidence", 5)),
      2,
      30);
    seek_snapshots_dir_ = declare_parameter<std::string>(
      "seek_snapshots_dir",
      dobot_common::paths::workspacePath({"debug files", "seek_frames"}, __FILE__).string());
    publish_overlay_ = declare_parameter<bool>("publish_overlay", true);
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
    detect_black_to_white_ = declare_parameter<bool>("detect_black_to_white", true);
    trace_out_to_in_ = declare_parameter<bool>("trace_out_to_in", false);
    tray_name_ = declare_parameter<std::string>("tray_name", "tray");
    teach_date_ = declare_parameter<std::string>("teach_date", "");
    const auto declared_taught_edge_lengths = declare_parameter<std::vector<double>>("taught_edge_lengths_cm", std::vector<double>{});
    for (std::size_t i = 0; i < taught_edge_lengths_cm_.size() && i < declared_taught_edge_lengths.size(); ++i)
    {
      taught_edge_lengths_cm_[i] = declared_taught_edge_lengths[i];
    }
    const auto taught_area_override = parameter_overrides.find("taught_area_cm2");
    if (taught_area_override != parameter_overrides.end() &&
        taught_area_override->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      taught_area_cm2_ = static_cast<double>(declare_parameter<int64_t>("taught_area_cm2", 0));
    }
    else
    {
      taught_area_cm2_ = declare_parameter<double>("taught_area_cm2", 0.0);
    }
    area_tolerance_percent_ = declare_parameter<int>("area_tolerance_percent", 15);
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
          "Using calibration_child_frame for tray outputs.",
          camera_frame_id_.c_str(), calibration_child_frame_.c_str());
      }
      camera_frame_id_ = calibration_child_frame_;

      if (publish_static_calibration_tf_)
      {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishCalibrationTransform();
      }
    }

    refreshTrayProfiles();
    selectInitialProfile();
    loadRuntimeUiSettings();
    createRosInterfaces();
    movj_client_ = create_client<MovJSrv>(movj_service_name_);
    cv::namedWindow(kDetectWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(kDetectWindowName, kPreviewCanvasWidth, kTopBarBaseHeight + kPreviewCanvasHeight);
    cv::setMouseCallback(kDetectWindowName, &TrayDetectNode::onMouseThunk, this);

    RCLCPP_INFO(
      get_logger(),
      "Tray detect ready. Overlay topic=%s tray_pose topic=%s tray_vector topic=%s tray_cube_marker topic=%s "
      "(enabled=%s thickness=%.1fmm) detect_mode=%s depth_threshold=+/- %dmm depth_plane=%s "
      "z_axis=natural_edge base_frame=%s pose_filter=window %.2fs min=%d z<=%.1fmm ang<=%.1fdeg movj_service=%s seek_service=%s go_to_teach_service=%s "
      "selected_profile=%s profiles=%zu",
      overlay_topic_.c_str(),
      tray_pose_topic_.c_str(),
      tray_vector_topic_.c_str(),
      tray_cube_marker_topic_.c_str(),
      publish_tray_cube_marker_ ? "true" : "false",
      tray_thickness_mm_,
      detectionModeToString(detection_use_depth_).c_str(),
      depth_threshold_mm_,
      depth_plane_model_.valid ? "fixed" : "missing",
      robot_base_frame_.c_str(),
      pose_filter_window_sec_,
      pose_filter_min_samples_,
      pose_outlier_position_mm_,
      pose_outlier_angle_deg_,
      movj_service_name_.c_str(),
      seek_service_name_.c_str(),
      go_to_teach_service_name_.c_str(),
      selectedProfileDisplayText().c_str(),
      tray_profiles_.size());
    if (use_calibration_)
    {
      RCLCPP_INFO(
        get_logger(),
        "Calibration loaded from %s. Publishing %s -> %s in-node: %s",
        calibration_file_.c_str(), calibration_parent_frame_.c_str(), calibration_child_frame_.c_str(),
        publish_static_calibration_tf_ ? "enabled" : "disabled");
    }
    else
    {
      RCLCPP_INFO(
        get_logger(),
        "Calibration disabled. Tray outputs use camera frame resolved from parameter/header.");
    }
  }

  ~TrayDetectNode() override
  {
    saveRuntimeUiSettings();
    cv::destroyWindow(kDetectWindowName);
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

  struct SeekVectorSummary
  {
    bool has_value {false};
    cv::Vec3d delta_m {0.0, 0.0, 0.0};
    double delta_t_sec {0.0};
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

  static std::string defaultRuntimeSettingsFile()
  {
    return dobot_common::paths::workspacePath(
      {"config", "tray_perception", "tray_detect_runtime_settings.yaml"}, __FILE__).string();
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

  void createRosInterfaces()
  {
    overlay_pub_ = create_publisher<ImageMsg>(overlay_topic_, rclcpp::QoS(5));
    tray_pose_pub_ = create_publisher<PoseStampedMsg>(tray_pose_topic_, rclcpp::QoS(10));
    tray_axis_overlay_pub_ = create_publisher<PolygonStampedMsg>(tray_axis_overlay_topic_, rclcpp::QoS(10));
    tray_vector_pub_ = create_publisher<TrayVectorMsg>(
      tray_vector_topic_,
      rclcpp::QoS(1).reliable().transient_local());
    tray_cube_marker_pub_ = create_publisher<MarkerMsg>(
      tray_cube_marker_topic_,
      rclcpp::QoS(1).reliable().transient_local());
    color_sub_ = create_subscription<ImageMsg>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrayDetectNode::colorCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<ImageMsg>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrayDetectNode::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<CameraInfoMsg>(
      camera_info_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&TrayDetectNode::cameraInfoCallback, this, std::placeholders::_1));
    if (!tray_dimensions_service_)
    {
      tray_dimensions_service_ = create_service<GetTrayDimensionsSrv>(
        tray_dimensions_service_name_,
        std::bind(
          &TrayDetectNode::handleGetTrayDimensions,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!seek_service_)
    {
      seek_service_ = create_service<TriggerSrv>(
        seek_service_name_,
        std::bind(
          &TrayDetectNode::handleSeekService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!seek_complete_service_)
    {
      seek_complete_service_ = create_service<TriggerSrv>(
        seek_complete_service_name_,
        std::bind(
          &TrayDetectNode::handleSeekCompleteService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!seek_status_service_)
    {
      seek_status_service_ = create_service<TriggerSrv>(
        seek_status_service_name_,
        std::bind(
          &TrayDetectNode::handleSeekStatusService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
    if (!camera_status_timer_)
    {
      camera_status_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TrayDetectNode::renderNoCameraTopicsOverlay, this));
    }
    if (!go_to_teach_service_)
    {
      go_to_teach_service_ = create_service<TriggerSrv>(
        go_to_teach_service_name_,
        std::bind(
          &TrayDetectNode::handleGoToTeachService,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }

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
      resetSeekSessionState();
      profile_status_message_ = "Seek cancelled";
      response->success = true;
      response->message = profile_status_message_;
      return;
    }

    seek_mode_active_ = true;
    seek_result_latched_ = false;
    seek_vector_summary_.has_value = false;
    resetSeekSessionState();
    profile_status_message_ = cv::format(
      "Seek armed: waiting for tray vector publish (%.1fs window, %.1fs decay, %d valid frames)",
      seekWindowSeconds(),
      seekDecaySeconds(),
      seek_valid_confidence_frames_);
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
    resetSeekSessionState();
    profile_status_message_ = "Seek released by tray intercept";
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

  void clearPoseHistory()
  {
    tray_pose_history_.clear();
    pose_history_frame_id_.clear();
  }

  double angleBetweenVectorsDeg(const cv::Vec3d &lhs, const cv::Vec3d &rhs) const
  {
    cv::Vec3d lhs_unit = lhs;
    cv::Vec3d rhs_unit = rhs;
    if (!normalizeVectorInPlace(lhs_unit) || !normalizeVectorInPlace(rhs_unit))
    {
      return 180.0;
    }
    const double dot_product = std::clamp(lhs_unit.dot(rhs_unit), -1.0, 1.0);
    return std::acos(dot_product) * kRadiansToDegrees;
  }

  std::optional<TrayPose3D> filterTrayPoseOutliers(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<TrayPose3D> &raw_pose)
  {
    if (!raw_pose.has_value())
    {
      return std::nullopt;
    }

    if (!pose_history_frame_id_.empty() && pose_history_frame_id_ != frame_id)
    {
      clearPoseHistory();
    }
    pose_history_frame_id_ = frame_id;

    tray_pose_history_.push_back(TimedTrayPose3D{stamp, *raw_pose, frame_id});
    const rclcpp::Time oldest_allowed = stamp - rclcpp::Duration::from_seconds(pose_filter_window_sec_);
    while (!tray_pose_history_.empty() && tray_pose_history_.front().stamp < oldest_allowed)
    {
      tray_pose_history_.pop_front();
    }
    while (tray_pose_history_.size() > pose_filter_max_samples_)
    {
      tray_pose_history_.pop_front();
    }

    if (tray_pose_history_.size() < static_cast<std::size_t>(pose_filter_min_samples_))
    {
      return raw_pose;
    }

    const auto reference_pose = averageTimedTrayPoses(tray_pose_history_);
    if (!reference_pose.has_value())
    {
      return raw_pose;
    }

    const double z_threshold_m = pose_outlier_position_mm_ / kMetersToMillimeters;
    const cv::Vec3d ref_origin = reference_pose->origin;
    const cv::Vec3d ref_x = rotationColumn(reference_pose->rotation, 0);
    const cv::Vec3d ref_z = rotationColumn(reference_pose->rotation, 2);

    std::deque<TimedTrayPose3D> inliers;
    for (const auto &sample : tray_pose_history_)
    {
      const double z_error_m = std::fabs(sample.pose.origin[2] - ref_origin[2]);
      const cv::Vec3d sample_x = rotationColumn(sample.pose.rotation, 0);
      const cv::Vec3d sample_z = rotationColumn(sample.pose.rotation, 2);
      const double x_angle_error_deg = angleBetweenVectorsDeg(sample_x, ref_x);
      const double z_angle_error_deg = angleBetweenVectorsDeg(sample_z, ref_z);
      if (
        z_error_m <= z_threshold_m &&
        x_angle_error_deg <= pose_outlier_angle_deg_ &&
        z_angle_error_deg <= pose_outlier_angle_deg_)
      {
        inliers.push_back(sample);
      }
    }

    if (inliers.size() < static_cast<std::size_t>(pose_filter_min_samples_))
    {
      auto fallback_pose = reference_pose;
      if (fallback_pose.has_value())
      {
        // Keep X/Y live and only filter Z + orientation.
        fallback_pose->origin[0] = raw_pose->origin[0];
        fallback_pose->origin[1] = raw_pose->origin[1];
      }
      return fallback_pose;
    }

    tray_pose_history_ = std::move(inliers);
    auto filtered_pose = averageTimedTrayPoses(tray_pose_history_);
    if (filtered_pose.has_value())
    {
      // Keep X/Y live and only filter Z + orientation.
      filtered_pose->origin[0] = raw_pose->origin[0];
      filtered_pose->origin[1] = raw_pose->origin[1];
    }
    return filtered_pose;
  }

  void refreshTrayProfiles()
  {
    tray_profiles_ = loadTrayProfilesFromDirectory(profiles_dir_);
    syncSelectedProfileIndex();
    if (tray_profiles_.empty())
    {
      profile_status_message_ = "No tray profiles found";
    }
  }

  void syncSelectedProfileIndex()
  {
    selected_profile_index_ = -1;

    if (!selected_profile_path_.empty())
    {
      for (int i = 0; i < static_cast<int>(tray_profiles_.size()); ++i)
      {
        if (tray_profiles_[i].path == selected_profile_path_)
        {
          selected_profile_index_ = i;
          return;
        }
      }
    }

    for (int i = 0; i < static_cast<int>(tray_profiles_.size()); ++i)
    {
      const bool name_matches = tray_profiles_[i].tray_name == tray_name_;
      const bool date_matches =
        teach_date_.empty() || tray_profiles_[i].teach_date.empty() || tray_profiles_[i].teach_date == teach_date_;
      if (name_matches && date_matches)
      {
        selected_profile_index_ = i;
        return;
      }
    }
  }

  void selectInitialProfile()
  {
    if (tray_profiles_.empty())
    {
      return;
    }

    if (selected_profile_index_ < 0)
    {
      selected_profile_index_ = 0;
    }

    applyProfile(tray_profiles_[selected_profile_index_], false);
    selected_profile_path_ = tray_profiles_[selected_profile_index_].path;
    profile_status_message_ = "Loaded " + tray_profiles_[selected_profile_index_].display_label;
  }

  std::filesystem::path latestAliasProfilePath() const
  {
    return std::filesystem::path(profiles_dir_) / "tray_teach_settings.yaml";
  }

  static bool profilesMatchForAliasSync(const TrayProfile &a, const TrayProfile &b)
  {
    const bool has_edges_a = hasValidEdgeLengthsCm(a.taught_edge_lengths_cm);
    const bool has_edges_b = hasValidEdgeLengthsCm(b.taught_edge_lengths_cm);
    const bool edge_lengths_match =
      has_edges_a && has_edges_b &&
      std::equal(
        a.taught_edge_lengths_cm.begin(),
        a.taught_edge_lengths_cm.end(),
        b.taught_edge_lengths_cm.begin(),
        [](double lhs, double rhs)
        {
          return std::fabs(lhs - rhs) < 1e-6;
        });
    return a.tray_name == b.tray_name &&
      a.teach_date == b.teach_date &&
      (edge_lengths_match || std::fabs(a.taught_area_cm2 - b.taught_area_cm2) < 1e-6);
  }

  void syncLatestAliasProfileAfterDelete(const TrayProfile &deleted_profile)
  {
    const std::filesystem::path alias_path = latestAliasProfilePath();
    if (!std::filesystem::exists(alias_path))
    {
      return;
    }

    const auto alias_profile = loadTrayProfileFile(alias_path);
    if (!alias_profile.has_value() || !profilesMatchForAliasSync(*alias_profile, deleted_profile))
    {
      return;
    }

    std::error_code fs_error;
    if (!tray_profiles_.empty())
    {
      std::filesystem::copy_file(
        tray_profiles_.front().path,
        alias_path,
        std::filesystem::copy_options::overwrite_existing,
        fs_error);
    }
    else
    {
      std::filesystem::remove(alias_path, fs_error);
    }
  }

  void applyProfile(const TrayProfile &profile, bool recreate_interfaces)
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
    ray_step_px_ = profile.ray_step_px;
    depth_edge_offset_px_ = std::clamp(
      profile.depth_edge_offset_px,
      kDepthEdgeOffsetMinPx,
      kDepthEdgeOffsetMaxPx);
    previous_color_percent_ = profile.previous_color_percent;
    horizontal_ray_count_ = std::clamp(profile.horizontal_ray_count, 50, 100);
    vertical_ray_count_ = std::clamp(profile.vertical_ray_count, 50, 150);
    outlier_sensitivity_ = std::clamp(profile.outlier_sensitivity, 1, 100);
    detect_black_to_white_ = profile.detect_black_to_white;
    trace_out_to_in_ = profile.trace_out_to_in;
    depth_plane_model_.valid = profile.depth_plane_enabled;
    depth_plane_model_.a = profile.depth_plane_a;
    depth_plane_model_.b = profile.depth_plane_b;
    depth_plane_model_.c = profile.depth_plane_c;
    depth_plane_model_.reference_depth_m = profile.depth_plane_reference_depth_m;
    depth_plane_roi_bounds_ = profile.depth_plane_roi_bounds;
    if (
      !depth_plane_model_.valid ||
      !std::isfinite(depth_plane_model_.a) ||
      !std::isfinite(depth_plane_model_.b) ||
      !std::isfinite(depth_plane_model_.c) ||
      !std::isfinite(depth_plane_model_.reference_depth_m) ||
      depth_plane_model_.reference_depth_m <= 0.0 ||
      !depth_plane_roi_bounds_.has_value())
    {
      depth_plane_model_ = DepthPlaneModel{};
    }
    roi_points_ = profile.roi_points;
    tray_name_ = profile.tray_name;
    teach_date_ = profile.teach_date;
    teach_joints_deg_ = profile.teach_joints_deg;
    has_teach_joints_ = profile.has_teach_joints;
    taught_edge_lengths_cm_ = profile.taught_edge_lengths_cm;
    taught_area_cm2_ = profile.taught_area_cm2;
    resetMotionTracking();

    if (
      recreate_interfaces &&
      (
        topics_changed || !overlay_pub_ || !tray_pose_pub_ || !tray_axis_overlay_pub_ || !tray_vector_pub_ ||
        !color_sub_ || !depth_sub_ || !camera_info_sub_))
    {
      createRosInterfaces();
    }
  }

  bool selectProfileByIndex(int index, bool persist_runtime = true)
  {
    if (index < 0 || index >= static_cast<int>(tray_profiles_.size()))
    {
      return false;
    }

    selected_profile_index_ = index;
    selected_profile_path_ = tray_profiles_[index].path;
    const bool interfaces_ready =
      static_cast<bool>(overlay_pub_) ||
      static_cast<bool>(tray_pose_pub_) ||
      static_cast<bool>(tray_axis_overlay_pub_) ||
      static_cast<bool>(tray_vector_pub_) ||
      static_cast<bool>(color_sub_) ||
      static_cast<bool>(depth_sub_) ||
      static_cast<bool>(camera_info_sub_);
    applyProfile(tray_profiles_[index], interfaces_ready);
    profile_status_message_ = "Loaded " + tray_profiles_[index].display_label;
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
    for (int i = 0; i < static_cast<int>(tray_profiles_.size()); ++i)
    {
      const std::filesystem::path candidate = tray_profiles_[i].path.lexically_normal();
      if (candidate == requested || candidate.filename() == requested.filename())
      {
        return selectProfileByIndex(i, persist_runtime);
      }
    }
    return false;
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
    if (runtime_settings_path_.empty() || !std::filesystem::exists(runtime_settings_path_))
    {
      return;
    }

    try
    {
      const YAML::Node root = YAML::LoadFile(runtime_settings_path_.string());
      if (!root || !root.IsMap())
      {
        return;
      }

      if (const YAML::Node selected_profile_path = root["selected_profile_path"];
        selected_profile_path && selected_profile_path.IsScalar())
      {
        const std::string profile_path_text = selected_profile_path.as<std::string>();
        if (!profile_path_text.empty())
        {
          selectProfileByPath(resolvePath(profile_path_text), false);
        }
      }
      if (const YAML::Node view_mode = root["view_mode"]; view_mode && view_mode.IsScalar())
      {
        applyRuntimeViewModeToken(view_mode.as<std::string>());
      }
      if (const YAML::Node overlay_enabled = root["overlay_enabled"]; overlay_enabled)
      {
        overlay_enabled_ = overlay_enabled.as<bool>();
      }
      if (const YAML::Node edge_tolerance = root["edge_tolerance_percent"]; edge_tolerance)
      {
        area_tolerance_percent_ = std::clamp(edge_tolerance.as<int>(), tolerance_slider_.min_value, tolerance_slider_.max_value);
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
      if (const YAML::Node seek_valid_frames = root["seek_valid_frames_confidence"]; seek_valid_frames)
      {
        seek_valid_confidence_frames_ = std::clamp(
          seek_valid_frames.as<int>(),
          seek_confidence_slider_.min_value,
          seek_confidence_slider_.max_value);
      }
      RCLCPP_INFO(
        get_logger(),
        "Loaded tray detect runtime UI settings from %s",
        runtime_settings_path_.c_str());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load tray detect runtime UI settings from %s: %s",
        runtime_settings_path_.c_str(),
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
      out << YAML::Key << "selected_profile_path" << YAML::Value <<
        ((selected_profile_index_ >= 0 && selected_profile_index_ < static_cast<int>(tray_profiles_.size()))
          ? tray_profiles_[selected_profile_index_].path.string()
          : selected_profile_path_.string());
      out << YAML::Key << "view_mode" << YAML::Value << runtimeViewModeToken();
      out << YAML::Key << "overlay_enabled" << YAML::Value << overlay_enabled_;
      out << YAML::Key << "edge_tolerance_percent" << YAML::Value << area_tolerance_percent_;
      out << YAML::Key << "seek_window_sec" << YAML::Value << seekWindowSeconds();
      out << YAML::Key << "seek_decay_sec" << YAML::Value << seekDecaySeconds();
      out << YAML::Key << "seek_valid_frames_confidence" << YAML::Value << seek_valid_confidence_frames_;
      out << YAML::EndMap;

      std::ofstream file(runtime_settings_path_);
      file << out.c_str();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to save tray detect runtime UI settings to %s: %s",
        runtime_settings_path_.c_str(),
        ex.what());
    }
  }

  bool canDeleteSelectedProfile() const
  {
    const bool has_selected_profile = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(tray_profiles_.size());
    if (!has_selected_profile)
    {
      return false;
    }
    return tray_profiles_[selected_profile_index_].path.filename() != "tray_teach_settings.yaml";
  }

  void requestDeleteSelectedProfile()
  {
    if (selected_profile_index_ < 0 || selected_profile_index_ >= static_cast<int>(tray_profiles_.size()))
    {
      profile_status_message_ = "No tray profile selected";
      return;
    }
    if (!canDeleteSelectedProfile())
    {
      profile_status_message_ = "Select a dated tray profile";
      return;
    }

    profile_dropdown_open_ = false;
    tolerance_slider_active_ = false;
    seek_window_slider_active_ = false;
    seek_decay_slider_active_ = false;
    seek_confidence_slider_active_ = false;
    delete_confirm_active_ = true;
    profile_status_message_ = "Confirm delete selected tray profile";
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
    if (selected_profile_index_ < 0 || selected_profile_index_ >= static_cast<int>(tray_profiles_.size()))
    {
      profile_status_message_ = "No tray profile selected";
      return false;
    }

    const TrayProfile deleted_profile = tray_profiles_[selected_profile_index_];
    const std::filesystem::path delete_path = deleted_profile.path;
    if (delete_path.filename() == "tray_teach_settings.yaml")
    {
      profile_status_message_ = "Select a dated tray profile";
      return false;
    }

    std::error_code fs_error;
    const bool removed = std::filesystem::remove(delete_path, fs_error);
    if (!removed || fs_error)
    {
      profile_status_message_ = "Delete failed";
      return false;
    }

    selected_profile_path_.clear();
    refreshTrayProfiles();
    syncLatestAliasProfileAfterDelete(deleted_profile);
    refreshTrayProfiles();

    if (!tray_profiles_.empty())
    {
      const int next_index = std::clamp(selected_profile_index_, 0, static_cast<int>(tray_profiles_.size()) - 1);
      selectProfileByIndex(next_index);
      profile_status_message_ = "Deleted " + delete_path.filename().string();
    }
    else
    {
      selected_profile_index_ = -1;
      profile_status_message_ = "Deleted " + delete_path.filename().string();
      saveRuntimeUiSettings();
    }

    return true;
  }

  std::string selectedProfileDisplayText() const
  {
    if (selected_profile_index_ >= 0 && selected_profile_index_ < static_cast<int>(tray_profiles_.size()))
    {
      return tray_profiles_[selected_profile_index_].display_label;
    }

    if (!tray_name_.empty())
    {
      return buildProfileLabel(tray_name_, teach_date_, selected_profile_path_);
    }

    return "Select tray profile";
  }

  bool canGoToTeach() const
  {
    const bool has_selected_profile = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(tray_profiles_.size());
    return has_selected_profile && has_teach_joints_ && !go_to_teach_in_progress_;
  }

  bool requestGoToTeach()
  {
    const bool has_selected_profile = selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(tray_profiles_.size());
    if (!has_selected_profile)
    {
      profile_status_message_ = "Go to Teach: select a tray profile";
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
    return kTopBarBaseHeight +
      (profile_dropdown_open_ ? static_cast<int>(profile_option_rects_.size()) * kDropdownRowHeight : 0);
  }

  void layoutTopBar(int width)
  {
    const int margin = 16;
    const int top_row_y = 14;
    const int top_row_height = 38;
    const int control_gap = 10;

    const int view_width = 150;
    const int overlay_width = 150;
    const int seek_width = 120;
    view_toggle_button_.rect = cv::Rect(margin, top_row_y, view_width, top_row_height);
    overlay_toggle_button_.rect = cv::Rect(
      view_toggle_button_.rect.x + view_toggle_button_.rect.width + control_gap,
      top_row_y,
      overlay_width,
      top_row_height);
    seek_toggle_button_.rect = cv::Rect(
      overlay_toggle_button_.rect.x + overlay_toggle_button_.rect.width + control_gap,
      top_row_y,
      seek_width,
      top_row_height);

    const int delete_width = 148;
    const int go_to_teach_width = 148;
    delete_button_.rect = cv::Rect(width - margin - delete_width, top_row_y, delete_width, top_row_height);
    go_to_teach_button_.rect = cv::Rect(
      delete_button_.rect.x - control_gap - go_to_teach_width,
      top_row_y,
      go_to_teach_width,
      top_row_height);

    const int dropdown_left = seek_toggle_button_.rect.x + seek_toggle_button_.rect.width + 14;
    const int dropdown_right = go_to_teach_button_.rect.x - 14;
    const int dropdown_width = std::max(160, dropdown_right - dropdown_left);
    profile_dropdown_rect_ = cv::Rect(
      dropdown_left,
      top_row_y,
      std::max(80, dropdown_width),
      top_row_height);

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
    seek_confidence_label_origin_ = cv::Point(
      quality_panel_rect_.x + panel_pad,
      quality_panel_rect_.y + 71);
    seek_confidence_slider_.track_rect = cv::Rect(
      quality_panel_rect_.x + panel_pad,
      quality_panel_rect_.y + 77,
      std::max(80, quality_panel_rect_.width - 2 * panel_pad),
      10);

    profile_option_rects_.clear();
    if (profile_dropdown_open_)
    {
      for (int i = 0; i < static_cast<int>(tray_profiles_.size()); ++i)
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

  cv::Rect seekConfidenceHitRect() const
  {
    return cv::Rect(
      seek_confidence_slider_.track_rect.x,
      seek_confidence_slider_.track_rect.y - kToleranceHitPadding,
      seek_confidence_slider_.track_rect.width,
      seek_confidence_slider_.track_rect.height + 2 * kToleranceHitPadding);
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
    area_tolerance_percent_ = static_cast<int>(std::round(
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

  void updateSeekConfidenceFromPoint(const cv::Point &point)
  {
    if (seek_confidence_slider_.track_rect.width <= 0)
    {
      return;
    }

    const int clamped_x = std::clamp(
      point.x,
      seek_confidence_slider_.track_rect.x,
      seek_confidence_slider_.track_rect.x + seek_confidence_slider_.track_rect.width);
    const double t = static_cast<double>(clamped_x - seek_confidence_slider_.track_rect.x) /
      std::max(1, seek_confidence_slider_.track_rect.width);
    const int updated_frames = static_cast<int>(std::round(
      seek_confidence_slider_.min_value + t * (seek_confidence_slider_.max_value - seek_confidence_slider_.min_value)));
    seek_valid_confidence_frames_ = std::clamp(
      updated_frames,
      seek_confidence_slider_.min_value,
      seek_confidence_slider_.max_value);
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

  void updateSeekVectorSummary(const SeekMotionData &motion)
  {
    seek_vector_summary_.has_value = true;
    seek_vector_summary_.delta_m = motion.delta_position_mm * (1.0 / kMetersToMillimeters);
    seek_vector_summary_.delta_t_sec = motion.dt_sec;
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

  bool publishSeekVectorData(
    const SeekCapture &last_capture,
    const SeekMotionData &motion,
    double decay_sec)
  {
    if (!tray_vector_pub_)
    {
      return false;
    }
    const cv::Vec3d position_mm = last_capture.pose.origin * kMetersToMillimeters;
    const cv::Vec3d rpy_deg = rotationToRpyDegrees(last_capture.pose.rotation);

    TrayVectorMsg msg;
    msg.header.stamp = toBuiltinTime(last_capture.stamp);
    msg.header.frame_id = !last_capture.frame_id.empty()
      ? last_capture.frame_id
      : (!camera_frame_id_.empty() ? camera_frame_id_ : std::string("camera_color_optical_frame"));
    msg.first_stamp = toBuiltinTime(seek_first_valid_capture_.has_value() ? seek_first_valid_capture_->stamp : last_capture.stamp);
    msg.last_stamp = toBuiltinTime(last_capture.stamp);
    msg.dt_sec = motion.dt_sec;
    msg.decay_sec = std::max(0.0, decay_sec);
    msg.position_mm.x = position_mm[0];
    msg.position_mm.y = position_mm[1];
    msg.position_mm.z = position_mm[2];
    msg.rpy_deg.x = rpy_deg[0];
    msg.rpy_deg.y = rpy_deg[1];
    msg.rpy_deg.z = rpy_deg[2];
    // Sterilize tray motion vector in Z: publish XY-only motion components.
    const cv::Vec3d planar_velocity_camera_mmps(
      motion.velocity_camera_mmps[0],
      motion.velocity_camera_mmps[1],
      0.0);
    const double planar_speed_mmps = cv::norm(planar_velocity_camera_mmps);
    const cv::Vec3d planar_direction_camera = planar_speed_mmps > 1e-9
      ? (planar_velocity_camera_mmps * (1.0 / planar_speed_mmps))
      : cv::Vec3d(0.0, 0.0, 0.0);

    msg.velocity_mmps.x = planar_velocity_camera_mmps[0];
    msg.velocity_mmps.y = planar_velocity_camera_mmps[1];
    msg.velocity_mmps.z = 0.0;
    msg.speed_mmps = planar_speed_mmps;
    msg.direction_unit.x = planar_direction_camera[0];
    msg.direction_unit.y = planar_direction_camera[1];
    msg.direction_unit.z = 0.0;
    tray_vector_pub_->publish(msg);
    return true;
  }

  void publishContinuousTrayPose(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<TrayPose3D> &detected_pose)
  {
    if (!tray_pose_pub_ || !detected_pose.has_value())
    {
      return;
    }

    PoseStampedMsg msg;
    msg.header.stamp = toBuiltinTime(stamp);
    msg.header.frame_id = frame_id.empty() ? "camera_color_optical_frame" : frame_id;
    msg.pose.position.x = detected_pose->origin[0];
    msg.pose.position.y = detected_pose->origin[1];
    msg.pose.position.z = detected_pose->origin[2];
    msg.pose.orientation = rotationToQuaternionMsg(detected_pose->rotation);
    tray_pose_pub_->publish(msg);
  }

  void publishTrayOverlayAxes(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<TrayEstimate> &estimate)
  {
    if (!tray_axis_overlay_pub_ || !estimate.has_value())
    {
      return;
    }

    const auto axes = computeTrayOverlayAxes(*estimate);
    if (!axes.has_value())
    {
      return;
    }

    PolygonStampedMsg msg;
    msg.header.stamp = toBuiltinTime(stamp);
    msg.header.frame_id = frame_id.empty() ? "tray_detect_image" : frame_id;
    msg.polygon.points.resize(3);
    msg.polygon.points[0].x = axes->origin.x;
    msg.polygon.points[0].y = axes->origin.y;
    msg.polygon.points[0].z = 1.0F;
    msg.polygon.points[1].x = axes->x_dir.x;
    msg.polygon.points[1].y = axes->x_dir.y;
    msg.polygon.points[1].z = 0.0F;
    msg.polygon.points[2].x = axes->y_dir.x;
    msg.polygon.points[2].y = axes->y_dir.y;
    msg.polygon.points[2].z = 0.0F;
    tray_axis_overlay_pub_->publish(msg);
  }

  std::optional<std::pair<double, double>> planarSizeMetersFromEdgeLengths(
    const std::array<double, 4> &edge_lengths_cm) const
  {
    if (!hasValidEdgeLengthsCm(edge_lengths_cm))
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

  std::optional<std::pair<double, double>> measuredTrayPlanarSizeMeters(
    const std::optional<TrayEstimate> &accepted_estimate) const
  {
    if (accepted_estimate.has_value() &&
        accepted_estimate->has_metric_estimate &&
        hasValidEdgeLengthsCm(accepted_estimate->edge_lengths_cm))
    {
      return planarSizeMetersFromEdgeLengths(accepted_estimate->edge_lengths_cm);
    }
    return std::nullopt;
  }

  std::optional<std::pair<double, double>> taughtTrayPlanarSizeMeters() const
  {
    if (hasValidEdgeLengthsCm(taught_edge_lengths_cm_))
    {
      return planarSizeMetersFromEdgeLengths(taught_edge_lengths_cm_);
    }
    return std::nullopt;
  }

  std::optional<std::pair<double, double>> trayPlanarSizeMeters(
    const std::optional<TrayEstimate> &accepted_estimate) const
  {
    if (const auto measured_size_m = measuredTrayPlanarSizeMeters(accepted_estimate);
        measured_size_m.has_value())
    {
      return measured_size_m;
    }
    return taughtTrayPlanarSizeMeters();
  }

  void publishTrayCubeMarker(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<TrayPose3D> &detected_pose,
    const std::optional<TrayEstimate> &accepted_estimate)
  {
    if (!publish_tray_cube_marker_ || !tray_cube_marker_pub_)
    {
      return;
    }

    MarkerMsg marker;
    marker.header.stamp = toBuiltinTime(stamp);
    marker.header.frame_id = frame_id.empty() ? "camera_color_optical_frame" : frame_id;
    marker.ns = "tray_detect";
    marker.id = 0;
    marker.type = MarkerMsg::CUBE;
    marker.frame_locked = false;

    const auto tray_size_m = trayPlanarSizeMeters(accepted_estimate);
    if (!detected_pose.has_value() || !tray_size_m.has_value())
    {
      marker.action = MarkerMsg::DELETE;
      tray_cube_marker_pub_->publish(marker);
      return;
    }

    const double thickness_m = std::max(1e-4, tray_thickness_mm_ / kMetersToMillimeters);
    const cv::Vec3d local_center(
      0.5 * tray_size_m->first,
      0.5 * tray_size_m->second,
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
    marker.scale.x = tray_size_m->first;
    marker.scale.y = tray_size_m->second;
    marker.scale.z = thickness_m;
    marker.color.r = 0.15f;
    marker.color.g = 0.85f;
    marker.color.b = 0.95f;
    marker.color.a = 0.65f;
    tray_cube_marker_pub_->publish(marker);
  }

  void handleGetTrayDimensions(
    const std::shared_ptr<GetTrayDimensionsSrv::Request> request,
    std::shared_ptr<GetTrayDimensionsSrv::Response> response)
  {
    (void)request;
    if (!response)
    {
      return;
    }

    std::optional<std::pair<double, double>> tray_size_m = latest_live_tray_size_m_;
    bool live_detection = tray_size_m.has_value();
    if (!tray_size_m.has_value())
    {
      tray_size_m = taughtTrayPlanarSizeMeters();
    }

    response->tray_name = tray_name_;
    response->live_detection = live_detection;
    if (!tray_size_m.has_value())
    {
      response->success = false;
      response->x_size_mm = 0.0;
      response->y_size_mm = 0.0;
      response->message = tray_name_.empty()
        ? "Tray dimensions are unavailable: no live detection and no taught profile size."
        : "Tray dimensions are unavailable for '" + tray_name_ +
            "': no live detection and no taught profile size.";
      return;
    }

    response->success = true;
    response->x_size_mm = tray_size_m->first * kMetersToMillimeters;
    response->y_size_mm = tray_size_m->second * kMetersToMillimeters;
    response->message = live_detection
      ? "Using live detected tray dimensions."
      : "Using taught tray profile dimensions.";
  }

  void resetSeekSessionState()
  {
    seek_first_valid_capture_.reset();
    seek_last_valid_capture_.reset();
    seek_valid_motion_samples_.clear();
    seek_valid_frame_count_ = 0;
    seek_window_started_ = false;
    seek_window_start_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void resetSeekEvidenceState()
  {
    seek_first_valid_capture_.reset();
    seek_last_valid_capture_.reset();
    seek_valid_motion_samples_.clear();
    seek_valid_frame_count_ = 0;
  }

  void toggleSeek()
  {
    if (seek_mode_active_ || seek_result_latched_)
    {
      seek_mode_active_ = false;
      seek_result_latched_ = false;
      resetSeekSessionState();
      profile_status_message_ = "Seek cancelled";
      return;
    }

    seek_mode_active_ = true;
    seek_result_latched_ = false;
    seek_vector_summary_.has_value = false;
    resetSeekSessionState();
    profile_status_message_ = cv::format(
      "Seek armed: %.1fs window, %.1fs decay, %d valid frames",
      seekWindowSeconds(),
      seekDecaySeconds(),
      seek_valid_confidence_frames_);
  }

  bool writeSeekPoseData(
    const std::filesystem::path &pose_path,
    const SeekCapture &first_capture,
    const SeekCapture &last_capture,
    const SeekMotionData &motion,
    std::size_t motion_sample_count,
    const std::filesystem::path &first_image_path,
    const std::filesystem::path &last_image_path,
    double effective_decay_sec) const
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "seek_window_sec" << YAML::Value << seekWindowSeconds();
    out << YAML::Key << "seek_decay_sec" << YAML::Value << seekDecaySeconds();
    out << YAML::Key << "effective_decay_sec" << YAML::Value << std::max(0.0, effective_decay_sec);
    out << YAML::Key << "valid_frames_confidence" << YAML::Value << seek_valid_confidence_frames_;
    out << YAML::Key << "motion_source" << YAML::Value << "average_all_valid_frames_linear_fit";
    out << YAML::Key << "motion_sample_count" << YAML::Value << static_cast<int>(motion_sample_count);
    out << YAML::Key << "first_frame_path" << YAML::Value << first_image_path.string();
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

    emit_capture("first_valid", first_capture);
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
    out << YAML::Key << "frame_id" << YAML::Value << tray_frame_id_;
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

  void saveSeekScreenshotsAndPoseData(
    const rclcpp::Time &fallback_stamp,
    bool bypass_decay_on_publish = false)
  {
    seek_mode_active_ = false;

    if (!seek_last_valid_capture_.has_value())
    {
      seek_vector_summary_.has_value = false;
      seek_result_latched_ = false;
      resetSeekSessionState();
      profile_status_message_ = cv::format("Seek done (%.1fs): no valid tray frame", seekWindowSeconds());
      return;
    }

    const auto &first_capture = seek_first_valid_capture_.has_value()
      ? *seek_first_valid_capture_
      : *seek_last_valid_capture_;
    const auto &last_capture = *seek_last_valid_capture_;
    const double effective_decay_sec = bypass_decay_on_publish ? 0.0 : seekDecaySeconds();
    SeekMotionData motion = seek_valid_motion_samples_.size() >= 2
      ? computeSeekMotionData(seek_valid_motion_samples_)
      : computeSeekMotionData(first_capture, last_capture);
    updateSeekVectorSummary(motion);
    const bool published_seek_vector = publishSeekVectorData(last_capture, motion, effective_decay_sec);
    const int64_t stamp_ns = (last_capture.stamp.nanoseconds() != 0)
      ? last_capture.stamp.nanoseconds()
      : fallback_stamp.nanoseconds();
    const std::filesystem::path first_path =
      std::filesystem::path(seek_snapshots_dir_) / ("seek_" + std::to_string(stamp_ns) + "_first.png");
    const std::filesystem::path last_path =
      std::filesystem::path(seek_snapshots_dir_) / ("seek_" + std::to_string(stamp_ns) + "_last.png");
    const std::filesystem::path pose_path =
      std::filesystem::path(seek_snapshots_dir_) / ("seek_" + std::to_string(stamp_ns) + "_pose.yaml");

    std::filesystem::path output_dir(seek_snapshots_dir_);
    std::error_code fs_error;
    std::filesystem::create_directories(output_dir, fs_error);
    if (fs_error)
    {
      profile_status_message_ = "Seek done: failed to prepare screenshot directory";
    }
    else
    {
      const bool wrote_first = !first_capture.frame.empty() && cv::imwrite(first_path.string(), first_capture.frame);
      const bool wrote_last = !last_capture.frame.empty() && cv::imwrite(last_path.string(), last_capture.frame);
      const bool wrote_pose = writeSeekPoseData(
        pose_path,
        first_capture,
        last_capture,
        motion,
        seek_valid_motion_samples_.size(),
        first_path,
        last_path,
        effective_decay_sec);

      if (wrote_first && wrote_last && wrote_pose)
      {
        profile_status_message_ = "Seek done: saved frames + pose data";
        RCLCPP_INFO(
          get_logger(),
          "Seek data saved:\n  first frame: %s\n  last frame:  %s\n  pose yaml:   %s",
          first_path.c_str(),
          last_path.c_str(),
          pose_path.c_str());
      }
      else
      {
        profile_status_message_ = "Seek done: partial save (check logs)";
        RCLCPP_WARN(
          get_logger(),
          "Seek save partial. first=%s last=%s pose=%s",
          wrote_first ? "ok" : "fail",
          wrote_last ? "ok" : "fail",
          wrote_pose ? "ok" : "fail");
      }
    }

    if (!published_seek_vector)
    {
      profile_status_message_ = "Seek done: tray vector publisher unavailable";
      seek_result_latched_ = false;
    }
    else
    {
      seek_result_latched_ = true;
      profile_status_message_ += " | waiting for tray intercept release";
    }
    resetSeekSessionState();
  }

  void updateSeekSession(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::optional<TrayPose3D> &detected_pose,
    const cv::Mat &output_frame)
  {
    if (!seek_mode_active_)
    {
      return;
    }

    if (!seek_window_started_)
    {
      seek_window_start_stamp_ = stamp;
      seek_window_started_ = true;
      profile_status_message_ = cv::format(
        "Seek running: %.1fs window, %.1fs decay, %d valid frames",
        seekWindowSeconds(),
        seekDecaySeconds(),
        seek_valid_confidence_frames_);
    }

    const double elapsed_sec = (stamp - seek_window_start_stamp_).seconds();
    bool confidence_ready = seek_valid_frame_count_ >= seek_valid_confidence_frames_;
    if (elapsed_sec <= seekWindowSeconds() + 1e-6 && detected_pose.has_value())
    {
      SeekCapture capture;
      capture.stamp = stamp;
      capture.pose = *detected_pose;
      capture.frame_id = frame_id;
      capture.frame = output_frame.clone();
      seek_valid_motion_samples_.push_back(SeekMotionSample{stamp, *detected_pose});
      if (!seek_first_valid_capture_.has_value())
      {
        seek_first_valid_capture_ = capture;
      }
      seek_last_valid_capture_ = std::move(capture);
      ++seek_valid_frame_count_;
      confidence_ready = seek_valid_frame_count_ >= seek_valid_confidence_frames_;
    }

    const bool window_elapsed = elapsed_sec >= seekWindowSeconds();
    if (!confidence_ready && !window_elapsed && seek_last_valid_capture_.has_value())
    {
      const double time_since_last_valid_sec = (stamp - seek_last_valid_capture_->stamp).seconds();
      if (time_since_last_valid_sec >= seekDecaySeconds())
      {
        const int dropped_frames = seek_valid_frame_count_;
        resetSeekEvidenceState();
        profile_status_message_ = cv::format(
          "Seek decay reset: dropped %d frame(s), waiting for %d continuous valid frames",
          dropped_frames,
          seek_valid_confidence_frames_);
        return;
      }
    }

    if (confidence_ready)
    {
      // Confidence-only publish path: seek vector publish is always immediate (decay=0).
      saveSeekScreenshotsAndPoseData(stamp, true);
      return;
    }

    if (window_elapsed)
    {
      seek_mode_active_ = false;
      seek_result_latched_ = false;
      seek_vector_summary_.has_value = false;
      profile_status_message_ = cv::format(
        "Seek done (%.1fs): confidence not reached (%d/%d)",
        seekWindowSeconds(),
        seek_valid_frame_count_,
        seek_valid_confidence_frames_);
      resetSeekSessionState();
    }
  }

  void resetMotionTracking()
  {
    clearPoseHistory();
    seek_mode_active_ = false;
    seek_result_latched_ = false;
    resetSeekSessionState();
  }

  void drawVelocityArrow(
    cv::Mat &image,
    const std::optional<TrayPose3D> &tray_pose,
    const CameraInfoMsg::ConstSharedPtr &info) const
  {
    (void)image;
    (void)tray_pose;
    (void)info;
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

    const rclcpp::Time stamp(msg->header.stamp);
    const std::string resolved_frame_id = resolvedCameraFrameId(msg->header, info);
    cv::Mat mask;
    if (detection_use_depth_)
    {
      if (depth_plane_model_.valid)
      {
        const cv::Mat normalized_depth = applyFixedDepthPlaneNormalization(depth_m, depth_plane_model_);
        mask = buildDepthMask(
          normalized_depth,
          depth_threshold_mm_,
          depth_plane_model_.reference_depth_m);
      }
      else if (!depth_m.empty() && depth_m.type() == CV_32FC1)
      {
        mask = cv::Mat::zeros(depth_m.size(), CV_8UC1);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 2000,
          "Depth mode enabled but fixed depth plane is missing; run tray teach depth-plane ROI and save profile.");
      }
    }
    else
    {
      mask = buildRgbMask(color_cv->image, red_threshold_, green_threshold_, blue_threshold_);
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
        depth_m,
        info,
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

    bool rejected_by_edge_lengths = false;
    bool rejected_by_missing_edge_metrics = false;
    bool rejected_by_area = false;
    double max_edge_error_percent = 0.0;
    std::optional<TrayEstimate> accepted_estimate = tray_estimate;
    std::optional<TrayPose3D> detected_tray_pose_3d;
    if (tray_estimate.has_value() && hasValidEdgeLengthsCm(taught_edge_lengths_cm_))
    {
      if (!tray_estimate->has_metric_estimate || !hasValidEdgeLengthsCm(tray_estimate->edge_lengths_cm))
      {
        accepted_estimate.reset();
        rejected_by_edge_lengths = true;
        rejected_by_missing_edge_metrics = true;
      }
      else if (!areEdgeLengthsWithinTaughtBand(
          tray_estimate->edge_lengths_cm,
          taught_edge_lengths_cm_,
          area_tolerance_percent_))
      {
        accepted_estimate.reset();
        rejected_by_edge_lengths = true;
        max_edge_error_percent = maxEdgeLengthDeviationPercent(
          tray_estimate->edge_lengths_cm,
          taught_edge_lengths_cm_);
      }
    }
    else if (tray_estimate.has_value() &&
             tray_estimate->has_metric_estimate &&
             tray_estimate->area_cm2 > 0.0 &&
             taught_area_cm2_ > 0.0 &&
             !isAreaWithinTaughtBand(tray_estimate->area_cm2, taught_area_cm2_, area_tolerance_percent_))
    {
      accepted_estimate.reset();
      rejected_by_area = true;
    }

    if (accepted_estimate.has_value())
    {
      const auto tray_pose_3d = estimateTrayPose3D(
        *accepted_estimate,
        depth_m,
        *info,
        depth_edge_offset_px_);
      if (tray_pose_3d.has_value())
      {
        detected_tray_pose_3d = tray_pose_3d;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Tray detected in image but natural 3D tray pose estimation failed.");
      }
    }
    latest_live_tray_size_m_ = measuredTrayPlanarSizeMeters(accepted_estimate);
    const std::optional<TrayPose3D> filtered_tray_pose_3d = filterTrayPoseOutliers(
      stamp,
      resolved_frame_id,
      detected_tray_pose_3d);

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
        cv::cvtColor(roi_ready ? detection_mask : mask, overlay, cv::COLOR_GRAY2BGR);
        break;
    }
    drawModeLabel(overlay, detection_use_depth_ ? "Depth" : "RGB");
    if (overlay_enabled_)
    {
      drawRoiOverlay(overlay, roi_points_);
      drawCenterCursor(overlay, roi_points_);
      drawTrayName(overlay, tray_name_);
      drawTrayEstimate(overlay, accepted_estimate, depth_edge_offset_px_);
      drawTrayAxes(overlay, accepted_estimate);
      drawVelocityArrow(overlay, filtered_tray_pose_3d, info);
      if (!roi_ready)
      {
        cv::putText(
          overlay,
          "Selected tray profile is missing ROI",
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
      if (rejected_by_edge_lengths && tray_estimate.has_value())
      {
        const std::string reject_text = rejected_by_missing_edge_metrics
          ? "Rejected: edge tolerance needs valid depth"
          : cv::format(
              "Rejected edge lengths, max error %.1f%% (+/-%d%%)",
              max_edge_error_percent,
              area_tolerance_percent_);
        cv::putText(
          overlay,
          reject_text,
          cv::Point(18, 68),
          cv::FONT_HERSHEY_SIMPLEX,
          0.72,
          cv::Scalar(0, 0, 255),
          2);
      }
      else if (rejected_by_area && tray_estimate.has_value() && tray_estimate->has_metric_estimate)
      {
        cv::putText(
          overlay,
          cv::format(
            "Rejected area %.0f mm2, teach %.0f mm2 (+/-%d%%)",
            tray_estimate->area_cm2 * kSquareCentimetersToSquareMillimeters,
            taught_area_cm2_ * kSquareCentimetersToSquareMillimeters,
            area_tolerance_percent_),
          cv::Point(18, 68),
          cv::FONT_HERSHEY_SIMPLEX,
          0.72,
          cv::Scalar(0, 0, 255),
          2);
      }
    }

    cv::Mat output = drawWindowFrame(overlay);
    updateSeekSession(stamp, resolved_frame_id, filtered_tray_pose_3d, output);
    publishContinuousTrayPose(stamp, resolved_frame_id, filtered_tray_pose_3d);
    publishTrayOverlayAxes(stamp, resolved_frame_id, accepted_estimate);
    publishTrayCubeMarker(stamp, resolved_frame_id, filtered_tray_pose_3d, accepted_estimate);

    if (publish_overlay_)
    {
      cv_bridge::CvImage overlay_image;
      overlay_image.header = msg->header;
      overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_image.image = output;
      overlay_pub_->publish(*overlay_image.toImageMsg());
    }

    const cv::Size window_size(output.cols, output.rows);
    if (window_size != rendered_window_size_)
    {
      cv::resizeWindow(kDetectWindowName, window_size.width, window_size.height);
      rendered_window_size_ = window_size;
    }
    cv::imshow(kDetectWindowName, output);
    cv::waitKey(1);
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
    cv::imshow(kDetectWindowName, output);
    cv::waitKey(1);
  }

  static void onMouseThunk(int event, int x, int y, int flags, void *userdata)
  {
    static_cast<TrayDetectNode *>(userdata)->onMouse(event, x, y, flags);
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
        seek_confidence_slider_active_ = false;
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
        if (!profile_dropdown_open_)
        {
          refreshTrayProfiles();
        }
        profile_dropdown_open_ = !profile_dropdown_open_;
        return;
      }

      if (profile_dropdown_open_)
      {
        const int clicked_profile_index = profileIndexAtPoint(point);
        if (clicked_profile_index >= 0)
        {
          selectProfileByIndex(clicked_profile_index);
          profile_dropdown_open_ = false;
          return;
        }
      }

      if (toleranceHitRect().contains(point))
      {
        tolerance_slider_active_ = true;
        seek_window_slider_active_ = false;
        seek_decay_slider_active_ = false;
        seek_confidence_slider_active_ = false;
        updateToleranceFromPoint(point);
        profile_dropdown_open_ = false;
        return;
      }

      if (seekWindowHitRect().contains(point))
      {
        seek_window_slider_active_ = true;
        tolerance_slider_active_ = false;
        seek_decay_slider_active_ = false;
        seek_confidence_slider_active_ = false;
        updateSeekWindowFromPoint(point);
        profile_dropdown_open_ = false;
        return;
      }

      if (seekDecayHitRect().contains(point))
      {
        seek_decay_slider_active_ = true;
        tolerance_slider_active_ = false;
        seek_window_slider_active_ = false;
        seek_confidence_slider_active_ = false;
        updateSeekDecayFromPoint(point);
        profile_dropdown_open_ = false;
        return;
      }

      if (seekConfidenceHitRect().contains(point))
      {
        seek_confidence_slider_active_ = true;
        tolerance_slider_active_ = false;
        seek_window_slider_active_ = false;
        seek_decay_slider_active_ = false;
        updateSeekConfidenceFromPoint(point);
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
    else if (event == cv::EVENT_MOUSEMOVE && seek_confidence_slider_active_)
    {
      updateSeekConfidenceFromPoint(point);
    }

    if (event == cv::EVENT_LBUTTONUP)
    {
      const bool any_slider_active = tolerance_slider_active_ || seek_window_slider_active_ ||
        seek_decay_slider_active_ || seek_confidence_slider_active_;
      tolerance_slider_active_ = false;
      seek_window_slider_active_ = false;
      seek_decay_slider_active_ = false;
      seek_confidence_slider_active_ = false;
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
      selected_profile_index_ < static_cast<int>(tray_profiles_.size()) &&
      tray_profiles_[selected_profile_index_].path.filename() != "tray_teach_settings.yaml";
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
      cv::putText(
        bar,
        text,
        cv::Point(button.rect.x + 12, button.rect.y + 26),
        cv::FONT_HERSHEY_DUPLEX,
        0.56,
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
    draw_button(go_to_teach_button_, go_to_teach_in_progress_ ? "Go Teach..." : "Go To Teach", can_go_to_teach, cv::Scalar(70, 140, 94), cv::Scalar(134, 232, 165));
    draw_button(delete_button_, "Delete Tray", can_delete, cv::Scalar(86, 76, 148), cv::Scalar(160, 146, 246));

    cv::rectangle(bar, profile_dropdown_rect_, cv::Scalar(61, 78, 96), cv::FILLED);
    cv::rectangle(bar, profile_dropdown_rect_, cv::Scalar(130, 166, 198), 2);
    const std::string selected_text = fitTextToWidth(
      selectedProfileDisplayText(),
      profile_dropdown_rect_.width - 34);
    cv::putText(
      bar,
      selected_text,
      cv::Point(profile_dropdown_rect_.x + 12, profile_dropdown_rect_.y + 26),
      cv::FONT_HERSHEY_DUPLEX,
      0.55,
      cv::Scalar(245, 245, 245),
      1,
      cv::LINE_AA);

    const int arrow_center_x = profile_dropdown_rect_.x + profile_dropdown_rect_.width - 16;
    const int arrow_center_y = profile_dropdown_rect_.y + 20;
    const std::vector<cv::Point> arrow = profile_dropdown_open_
      ? std::vector<cv::Point>{
          cv::Point(arrow_center_x - 5, arrow_center_y + 2),
          cv::Point(arrow_center_x + 5, arrow_center_y + 2),
          cv::Point(arrow_center_x, arrow_center_y - 4)}
      : std::vector<cv::Point>{
          cv::Point(arrow_center_x - 5, arrow_center_y - 2),
          cv::Point(arrow_center_x + 5, arrow_center_y - 2),
          cv::Point(arrow_center_x, arrow_center_y + 4)};
    cv::fillConvexPoly(bar, arrow, cv::Scalar(245, 245, 245));

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

    draw_panel(seek_vector_panel_rect_, "Motion Summary");
    draw_panel(seek_controls_panel_rect_, "Seek Controls");
    draw_panel(quality_panel_rect_, "Detection Quality");

    if (seek_vector_summary_.has_value)
    {
      const cv::Vec3d delta_mm = seek_vector_summary_.delta_m * kMetersToMillimeters;
      const double speed_mmps = seek_vector_summary_.delta_t_sec > 1e-6
        ? (cv::norm(delta_mm) / seek_vector_summary_.delta_t_sec)
        : 0.0;
      cv::putText(
        bar,
        cv::format(
          "dX %+0.1f  dY %+0.1f  dZ %+0.1f mm",
          delta_mm[0],
          delta_mm[1],
          delta_mm[2]),
        seek_vector_label_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.45,
        cv::Scalar(170, 238, 185),
        1,
        cv::LINE_AA);
      cv::putText(
        bar,
        cv::format("dt %.3fs   speed %.1f mm/s", seek_vector_summary_.delta_t_sec, speed_mmps),
        seek_vector_value_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.45,
        cv::Scalar(205, 212, 220),
        1,
        cv::LINE_AA);
      cv::putText(
        bar,
        "Source: averaged all valid seek frames",
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
        "No seek result yet",
        seek_vector_label_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.50,
        cv::Scalar(170, 174, 180),
        1,
        cv::LINE_AA);
      cv::putText(
        bar,
        "Run Seek to capture valid tray motion",
        seek_vector_value_origin_,
        cv::FONT_HERSHEY_DUPLEX,
        0.42,
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
      cv::format("Edge Tolerance +/-%d%%", area_tolerance_percent_),
      tolerance_label_origin_,
      cv::FONT_HERSHEY_DUPLEX,
      0.48,
      cv::Scalar(225, 230, 236),
      1,
      cv::LINE_AA);
    draw_slider(tolerance_slider_, area_tolerance_percent_, cv::Scalar(85, 225, 255));

    cv::putText(
      bar,
      cv::format("Valid frames confidence %d", seek_valid_confidence_frames_),
      seek_confidence_label_origin_,
      cv::FONT_HERSHEY_DUPLEX,
      0.48,
      cv::Scalar(225, 230, 236),
      1,
      cv::LINE_AA);
    draw_slider(seek_confidence_slider_, seek_valid_confidence_frames_, cv::Scalar(235, 191, 108));

    cv::rectangle(bar, status_panel_rect_, cv::Scalar(34, 36, 40), cv::FILLED);
    cv::rectangle(bar, status_panel_rect_, cv::Scalar(72, 77, 84), 1);
    const std::string default_status = selected_profile_index_ >= 0 &&
        selected_profile_index_ < static_cast<int>(tray_profiles_.size())
      ? "Ready | Profile: " + tray_profiles_[selected_profile_index_].path.filename().string()
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

    if (profile_dropdown_open_)
    {
      for (int i = 0; i < static_cast<int>(profile_option_rects_.size()); ++i)
      {
        const bool selected = i == selected_profile_index_;
        const cv::Scalar fill = selected ? cv::Scalar(72, 120, 72) : cv::Scalar(58, 58, 58);
        const cv::Scalar border = selected ? cv::Scalar(120, 255, 120) : cv::Scalar(110, 110, 110);
        cv::rectangle(bar, profile_option_rects_[i], fill, cv::FILLED);
        cv::rectangle(bar, profile_option_rects_[i], border, 1);
        cv::putText(
          bar,
          fitTextToWidth(tray_profiles_[i].display_label, profile_option_rects_[i].width - 20),
          cv::Point(profile_option_rects_[i].x + 10, profile_option_rects_[i].y + 22),
          cv::FONT_HERSHEY_SIMPLEX,
          0.55,
          cv::Scalar(255, 255, 255),
          1);
      }
    }

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

    const std::string title = "Confirm Tray Delete";
    const std::string target_name = (selected_profile_index_ >= 0 &&
      selected_profile_index_ < static_cast<int>(tray_profiles_.size()))
      ? tray_profiles_[selected_profile_index_].path.filename().string()
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
  std::string tray_pose_topic_;
  std::string tray_vector_topic_;
  std::string tray_cube_marker_topic_;
  std::string tray_dimensions_service_name_;
  std::string seek_service_name_;
  std::string seek_complete_service_name_;
  std::string seek_status_service_name_;
  std::string go_to_teach_service_name_;
  std::string movj_service_name_;
  std::string calibration_parent_frame_;
  std::string calibration_child_frame_;
  std::string calibration_dir_;
  std::string calibration_file_;
  std::filesystem::path runtime_settings_path_;
  std::string profiles_dir_;
  std::string teach_date_;
  std::string profile_status_message_;
  std::string camera_frame_id_;
  std::string tray_frame_id_;
  std::string tray_axis_overlay_topic_;
  std::string robot_base_frame_;
  bool use_calibration_ {true};
  bool publish_static_calibration_tf_ {true};
  bool auto_discover_calibration_ {true};
  bool publish_overlay_ {true};
  bool publish_tray_cube_marker_ {true};
  bool detection_use_depth_ {false};
  cv::Size rendered_window_size_ {};
  int red_threshold_ {120};
  int green_threshold_ {120};
  int blue_threshold_ {120};
  int depth_threshold_mm_ {10};
  DepthPlaneModel depth_plane_model_;
  int ray_step_px_ {3};
  int depth_edge_offset_px_ {4};
  int previous_color_percent_ {kDefaultPreviousColorPercent};
  int horizontal_ray_count_ {50};
  int vertical_ray_count_ {50};
  int outlier_sensitivity_ {50};
  int area_tolerance_percent_ {15};
  bool detect_black_to_white_ {true};
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
  bool seek_confidence_slider_active_ {false};
  std::string tray_name_ {"tray"};
  std::array<double, 6> teach_joints_deg_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool has_teach_joints_ {false};
  bool go_to_teach_in_progress_ {false};
  std::array<double, 4> taught_edge_lengths_cm_ {0.0, 0.0, 0.0, 0.0};
  double taught_area_cm2_ {0.0};
  double tray_thickness_mm_ {15.0};
  double motion_update_period_sec_ {0.1};
  double pose_filter_window_sec_ {0.8};
  double pose_outlier_position_mm_ {20.0};
  double pose_outlier_angle_deg_ {12.0};
  int pose_filter_min_samples_ {3};
  std::size_t pose_filter_max_samples_ {120};
  int seek_window_tenths_ {1};
  int seek_decay_tenths_ {1};
  int seek_valid_confidence_frames_ {21};
  int seek_valid_frame_count_ {0};
  std::optional<AxisAlignedRoiBounds> depth_plane_roi_bounds_;
  UiButton view_toggle_button_ {"View", cv::Rect(18, 14, 180, 42)};
  UiButton overlay_toggle_button_ {"Overlay", cv::Rect(18, 14, 150, 42)};
  UiButton seek_toggle_button_ {"Seek", cv::Rect(18, 14, 120, 42)};
  UiButton delete_button_ {"Delete Tray", cv::Rect(18, 14, 160, 42)};
  UiButton go_to_teach_button_ {"Go to Teach", cv::Rect(18, 14, 160, 42)};
  UiSlider tolerance_slider_ {"Tolerance", cv::Rect(), 1, 20};
  UiSlider seek_window_slider_ {"Seek Window", cv::Rect(), 1, 60};
  UiSlider seek_decay_slider_ {"Seek Decay", cv::Rect(), 1, 10};
  UiSlider seek_confidence_slider_ {"Seek Confidence", cv::Rect(), 2, 30};
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
  cv::Point seek_confidence_label_origin_;
  cv::Point seek_vector_label_origin_;
  cv::Point seek_vector_value_origin_;
  cv::Point seek_vector_time_origin_;
  std::vector<cv::Rect> profile_option_rects_;
  std::vector<TrayProfile> tray_profiles_;
  std::vector<cv::Point2f> roi_points_;
  int selected_profile_index_ {-1};
  std::filesystem::path selected_profile_path_;
  rclcpp::Time seek_window_start_stamp_ {0, 0, RCL_ROS_TIME};
  std::deque<TimedTrayPose3D> tray_pose_history_;
  std::string pose_history_frame_id_;
  std::optional<SeekCapture> seek_first_valid_capture_;
  std::optional<SeekCapture> seek_last_valid_capture_;
  std::deque<SeekMotionSample> seek_valid_motion_samples_;
  SeekVectorSummary seek_vector_summary_;
  std::optional<std::pair<double, double>> latest_live_tray_size_m_;
  std::string seek_snapshots_dir_;

  rclcpp::Publisher<ImageMsg>::SharedPtr overlay_pub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr tray_pose_pub_;
  rclcpp::Publisher<PolygonStampedMsg>::SharedPtr tray_axis_overlay_pub_;
  rclcpp::Publisher<TrayVectorMsg>::SharedPtr tray_vector_pub_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr tray_cube_marker_pub_;
  rclcpp::Service<GetTrayDimensionsSrv>::SharedPtr tray_dimensions_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr seek_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr seek_complete_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr seek_status_service_;
  rclcpp::Service<TriggerSrv>::SharedPtr go_to_teach_service_;
  rclcpp::TimerBase::SharedPtr camera_status_timer_;
  rclcpp::Client<MovJSrv>::SharedPtr movj_client_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;

  geometry_msgs::msg::Quaternion calibration_rotation_;
  geometry_msgs::msg::Vector3 calibration_translation_;

  std::mutex data_mutex_;
  cv::Mat latest_depth_;
  CameraInfoMsg::ConstSharedPtr latest_camera_info_;
  std::chrono::steady_clock::time_point last_camera_render_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrayDetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
