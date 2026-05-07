#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <QApplication>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPointF>
#include <QPushButton>
#include <QRect>
#include <QSizePolicy>
#include <QTextCursor>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <aruco_perception/msg/marker_detections.hpp>
#include <dobot_msgs_v4/msg/tool_vector_actual.hpp>
#include <dobot_msgs_v4/srv/mov_j.hpp>
#include <dobot_msgs_v4/srv/rel_mov_l_user.hpp>
#include <dobot_msgs_v4/srv/speed_factor.hpp>
#include <dobot_msgs_v4/srv/stop.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace
{
using MarkerDetectionsMsg = aruco_perception::msg::MarkerDetections;
using ToolVectorActualMsg = dobot_msgs_v4::msg::ToolVectorActual;
using MovJSrv = dobot_msgs_v4::srv::MovJ;
using RelMovLUserSrv = dobot_msgs_v4::srv::RelMovLUser;
using SpeedFactorSrv = dobot_msgs_v4::srv::SpeedFactor;
using StopSrv = dobot_msgs_v4::srv::Stop;
using ImageMsg = sensor_msgs::msg::Image;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

constexpr std::array<int, 4> kBinTeachAllowedMarkerIds{{1, 2, 3, 4}};
constexpr int kBinTeachRequiredMarkerCount = 4;

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;
using Pose7 = std::array<double, 7>;
using TcpPose = std::array<double, 6>;

struct GoalPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double rx{0.0};
  double ry{0.0};
  double rz{0.0};
};

struct CornerDot
{
  std::string role;
  double x{0.0};
  double y{0.0};
  int marker_id{0};
  int detection_index{0};
  int same_id_index{1};
};

struct MarkerObservation
{
  std::string key;
  int id{0};
  int detection_index{0};
  int same_id_index{1};
};

struct DepthPlaneSample
{
  double x_norm{0.0};
  double y_norm{0.0};
  double depth_m{0.0};
  int marker_id{0};
  int detection_index{0};
  int same_id_index{1};
};

struct DepthPlaneModel
{
  bool valid{false};
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double reference_depth_m{0.0};
  std::array<int, 4> roi{{0, 0, 0, 0}};
  std::string source{"aruco_marker_centers"};
  std::string source_frame;
  std::vector<DepthPlaneSample> samples;
};

struct MarkerData
{
  int detection_index{0};
  int id{0};
  int same_id_index{1};
  Vec3 position_m{{0.0, 0.0, 0.0}};
  Quat orientation{{0.0, 0.0, 0.0, 1.0}};
  std::optional<Vec2> pixel;
  std::vector<Vec2> corners;
};

struct DetectionFrame
{
  rclcpp::Time stamp;
  std::chrono::steady_clock::time_point received_monotonic;
  std::string frame_id;
  int image_width{0};
  int image_height{0};
  std::vector<MarkerData> markers;
};

struct CameraFrameInfo
{
  rclcpp::Time stamp;
  std::chrono::steady_clock::time_point received_monotonic;
  std::string frame_id;
  int width{0};
  int height{0};
};

struct Solution
{
  std::string bin_name;
  std::string frame_id;
  std::string parent_frame;
  bool platform_reference_enabled{false};
  std::string platform_name;
  std::string platform_calibration_file;
  std::string platform_parent_frame;
  std::string platform_frame;
  std::string marker_prefix;
  std::vector<int> allowed_marker_ids;
  int required_marker_count{kBinTeachRequiredMarkerCount};
  std::vector<int> marker_ids;
  std::string center_method{"marker_position_bounds_center"};
  Vec3 origin{{0.0, 0.0, 0.0}};
  Vec3 x_axis{{1.0, 0.0, 0.0}};
  Vec3 y_axis{{0.0, 1.0, 0.0}};
  Vec3 z_axis{{0.0, 0.0, 1.0}};
  Quat quaternion{{0.0, 0.0, 0.0, 1.0}};
  std::map<std::string, Vec3> marker_positions;
  std::vector<MarkerObservation> marker_observations;
  int image_width{0};
  int image_height{0};
  std::string dot_coordinate_frame;
  std::vector<int> roi_points;
  std::vector<double> roi_points_normalized;
  std::vector<CornerDot> dot_positions;
  DepthPlaneModel depth_plane;
};

struct CentroidPose
{
  std::map<std::string, Vec3> positions;
  std::vector<MarkerObservation> marker_observations;
  Vec3 center{{0.0, 0.0, 0.0}};
  Vec3 x_axis{{1.0, 0.0, 0.0}};
  Vec3 y_axis{{0.0, 1.0, 0.0}};
  Vec3 z_axis{{0.0, 0.0, 1.0}};
  Quat quaternion{{0.0, 0.0, 0.0, 1.0}};
};

struct DetectionSnapshot
{
  std::vector<MarkerData> visible;
  int missing_count{0};
  std::vector<std::string> details;
  std::optional<DetectionFrame> frame;
};

struct AlignPlan
{
  Solution solution;
  Solution target_solution;
  GoalPose goal;
  double span_mm{0.0};
};

std::chrono::steady_clock::time_point steadyNow()
{
  return std::chrono::steady_clock::now();
}

double secondsSince(const std::chrono::steady_clock::time_point &then)
{
  return std::chrono::duration<double>(steadyNow() - then).count();
}

std::string formatDouble(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string sanitizeName(const std::string &text)
{
  std::string token;
  bool previous_underscore = false;
  std::string trimmed = text;
  const auto first = trimmed.find_first_not_of(" \t\r\n");
  const auto last = trimmed.find_last_not_of(" \t\r\n");
  trimmed = (first == std::string::npos) ? std::string() : trimmed.substr(first, last - first + 1);
  for (char ch : trimmed)
  {
    const unsigned char uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) || ch == '_')
    {
      token.push_back(static_cast<char>(std::tolower(uch)));
      previous_underscore = false;
    }
    else if (!previous_underscore)
    {
      token.push_back('_');
      previous_underscore = true;
    }
  }
  while (!token.empty() && token.front() == '_')
  {
    token.erase(token.begin());
  }
  while (!token.empty() && token.back() == '_')
  {
    token.pop_back();
  }
  return token.empty() ? "bin" : token;
}

std::string formatMarkerIds(const std::vector<int> &marker_ids, const std::string &conjunction = "and")
{
  if (marker_ids.empty())
  {
    return std::string();
  }
  std::vector<std::string> parts;
  parts.reserve(marker_ids.size());
  for (const int marker_id : marker_ids)
  {
    parts.push_back(std::to_string(marker_id));
  }
  if (parts.size() == 1)
  {
    return parts.front();
  }
  if (parts.size() == 2)
  {
    return parts[0] + " " + conjunction + " " + parts[1];
  }
  std::ostringstream stream;
  for (size_t i = 0; i + 1 < parts.size(); ++i)
  {
    if (i > 0)
    {
      stream << ", ";
    }
    stream << parts[i];
  }
  stream << ", " << conjunction << " " << parts.back();
  return stream.str();
}

std::string formatMarkerIds(const std::array<int, 4> &marker_ids, const std::string &conjunction = "and")
{
  return formatMarkerIds(std::vector<int>(marker_ids.begin(), marker_ids.end()), conjunction);
}

double normalizedImageCoord(int value, int max_value)
{
  if (max_value <= 1)
  {
    return 0.0;
  }
  return static_cast<double>(value) / static_cast<double>(max_value - 1);
}

std::optional<std::array<int, 4>> roiBoundsFromFlatPoints(const std::vector<int> &roi_points)
{
  if (roi_points.size() < 8)
  {
    return std::nullopt;
  }

  int left = std::numeric_limits<int>::max();
  int top = std::numeric_limits<int>::max();
  int right = std::numeric_limits<int>::min();
  int bottom = std::numeric_limits<int>::min();
  for (size_t i = 0; i + 1 < roi_points.size(); i += 2)
  {
    const int x = roi_points[i];
    const int y = roi_points[i + 1];
    left = std::min(left, x);
    top = std::min(top, y);
    right = std::max(right, x);
    bottom = std::max(bottom, y);
  }
  if (left >= right || top >= bottom)
  {
    return std::nullopt;
  }
  return std::array<int, 4>{{left, top, right, bottom}};
}

bool solveLinear3x3(
  std::array<std::array<double, 3>, 3> matrix,
  std::array<double, 3> rhs,
  std::array<double, 3> &solution)
{
  for (int col = 0; col < 3; ++col)
  {
    int pivot = col;
    double pivot_abs = std::fabs(matrix[static_cast<size_t>(pivot)][static_cast<size_t>(col)]);
    for (int row = col + 1; row < 3; ++row)
    {
      const double candidate_abs = std::fabs(matrix[static_cast<size_t>(row)][static_cast<size_t>(col)]);
      if (candidate_abs > pivot_abs)
      {
        pivot = row;
        pivot_abs = candidate_abs;
      }
    }
    if (pivot_abs < 1e-12)
    {
      return false;
    }
    if (pivot != col)
    {
      std::swap(matrix[static_cast<size_t>(pivot)], matrix[static_cast<size_t>(col)]);
      std::swap(rhs[static_cast<size_t>(pivot)], rhs[static_cast<size_t>(col)]);
    }

    const double pivot_value = matrix[static_cast<size_t>(col)][static_cast<size_t>(col)];
    for (int item = col; item < 3; ++item)
    {
      matrix[static_cast<size_t>(col)][static_cast<size_t>(item)] /= pivot_value;
    }
    rhs[static_cast<size_t>(col)] /= pivot_value;

    for (int row = 0; row < 3; ++row)
    {
      if (row == col)
      {
        continue;
      }
      const double factor = matrix[static_cast<size_t>(row)][static_cast<size_t>(col)];
      for (int item = col; item < 3; ++item)
      {
        matrix[static_cast<size_t>(row)][static_cast<size_t>(item)] -=
          factor * matrix[static_cast<size_t>(col)][static_cast<size_t>(item)];
      }
      rhs[static_cast<size_t>(row)] -= factor * rhs[static_cast<size_t>(col)];
    }
  }

  solution = rhs;
  return
    std::isfinite(solution[0]) &&
    std::isfinite(solution[1]) &&
    std::isfinite(solution[2]);
}

std::optional<DepthPlaneModel> fitDepthPlaneFromMarkerCenters(
  const std::vector<MarkerData> &markers,
  const DetectionFrame &frame,
  const std::vector<int> &roi_points)
{
  if (frame.image_width <= 1 || frame.image_height <= 1)
  {
    return std::nullopt;
  }

  DepthPlaneModel plane;
  plane.source_frame = frame.frame_id;
  if (const auto roi = roiBoundsFromFlatPoints(roi_points); roi.has_value())
  {
    plane.roi = *roi;
  }
  else
  {
    plane.roi = std::array<int, 4>{{0, 0, frame.image_width - 1, frame.image_height - 1}};
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;
  double sum_xz = 0.0;
  double sum_yz = 0.0;

  for (const auto &marker : markers)
  {
    if (!marker.pixel)
    {
      continue;
    }
    const double depth_m = marker.position_m[2];
    if (!std::isfinite(depth_m) || depth_m <= 0.0)
    {
      continue;
    }
    const int px = std::clamp(
      static_cast<int>(std::lround((*marker.pixel)[0])),
      0,
      frame.image_width - 1);
    const int py = std::clamp(
      static_cast<int>(std::lround((*marker.pixel)[1])),
      0,
      frame.image_height - 1);
    const double x_norm = normalizedImageCoord(px, frame.image_width);
    const double y_norm = normalizedImageCoord(py, frame.image_height);
    if (!std::isfinite(x_norm) || !std::isfinite(y_norm))
    {
      continue;
    }

    plane.samples.push_back(
      DepthPlaneSample{
        x_norm,
        y_norm,
        depth_m,
        marker.id,
        marker.detection_index,
        marker.same_id_index});
    sum_x += x_norm;
    sum_y += y_norm;
    sum_z += depth_m;
    sum_xx += x_norm * x_norm;
    sum_xy += x_norm * y_norm;
    sum_yy += y_norm * y_norm;
    sum_xz += x_norm * depth_m;
    sum_yz += y_norm * depth_m;
  }

  const double count = static_cast<double>(plane.samples.size());
  if (plane.samples.size() < 3)
  {
    return std::nullopt;
  }

  const std::array<std::array<double, 3>, 3> normal_matrix{{
    std::array<double, 3>{{sum_xx, sum_xy, sum_x}},
    std::array<double, 3>{{sum_xy, sum_yy, sum_y}},
    std::array<double, 3>{{sum_x, sum_y, count}},
  }};
  const std::array<double, 3> normal_rhs{{sum_xz, sum_yz, sum_z}};
  std::array<double, 3> coefficients{{0.0, 0.0, 0.0}};
  if (!solveLinear3x3(normal_matrix, normal_rhs, coefficients))
  {
    return std::nullopt;
  }

  plane.a = coefficients[0];
  plane.b = coefficients[1];
  plane.c = coefficients[2];
  plane.reference_depth_m = sum_z / count;
  plane.valid =
    std::isfinite(plane.a) &&
    std::isfinite(plane.b) &&
    std::isfinite(plane.c) &&
    std::isfinite(plane.reference_depth_m) &&
    plane.reference_depth_m > 0.0;
  return plane.valid ? std::optional<DepthPlaneModel>(plane) : std::nullopt;
}

Vec3 vSub(const Vec3 &a, const Vec3 &b)
{
  return Vec3{{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}

double vDot(const Vec3 &a, const Vec3 &b)
{
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

Vec3 vCross(const Vec3 &a, const Vec3 &b)
{
  return Vec3{{
    (a[1] * b[2]) - (a[2] * b[1]),
    (a[2] * b[0]) - (a[0] * b[2]),
    (a[0] * b[1]) - (a[1] * b[0])}};
}

double vNorm(const Vec3 &a)
{
  return std::sqrt(std::max(0.0, vDot(a, a)));
}

std::optional<Vec3> vUnit(const Vec3 &a)
{
  const double norm = vNorm(a);
  if (norm < 1e-9)
  {
    return std::nullopt;
  }
  return Vec3{{a[0] / norm, a[1] / norm, a[2] / norm}};
}

Quat quaternionNormalize(const Quat &quat)
{
  const double qx = quat[0];
  const double qy = quat[1];
  const double qz = quat[2];
  const double qw = quat[3];
  const double norm = std::sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw));
  if (norm < 1e-12)
  {
    return Quat{{0.0, 0.0, 0.0, 1.0}};
  }
  const double inv = 1.0 / norm;
  return Quat{{qx * inv, qy * inv, qz * inv, qw * inv}};
}

Quat quaternionFromRotationMatrix(const std::array<std::array<double, 3>, 3> &m)
{
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  const double trace = m[0][0] + m[1][1] + m[2][2];
  if (trace > 0.0)
  {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (m[2][1] - m[1][2]) / s;
    qy = (m[0][2] - m[2][0]) / s;
    qz = (m[1][0] - m[0][1]) / s;
  }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
  {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    qw = (m[2][1] - m[1][2]) / s;
    qx = 0.25 * s;
    qy = (m[0][1] + m[1][0]) / s;
    qz = (m[0][2] + m[2][0]) / s;
  }
  else if (m[1][1] > m[2][2])
  {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    qw = (m[0][2] - m[2][0]) / s;
    qx = (m[0][1] + m[1][0]) / s;
    qy = 0.25 * s;
    qz = (m[1][2] + m[2][1]) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    qw = (m[1][0] - m[0][1]) / s;
    qx = (m[0][2] + m[2][0]) / s;
    qy = (m[1][2] + m[2][1]) / s;
    qz = 0.25 * s;
  }
  return quaternionNormalize(Quat{{qx, qy, qz, qw}});
}

double quaternionDot(const Quat &lhs, const Quat &rhs)
{
  return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2]) + (lhs[3] * rhs[3]);
}

Quat averageQuaternions(const std::vector<Quat> &quaternions)
{
  std::vector<Quat> clean;
  clean.reserve(quaternions.size());
  for (const auto &quat : quaternions)
  {
    clean.push_back(quaternionNormalize(quat));
  }
  if (clean.empty())
  {
    return Quat{{0.0, 0.0, 0.0, 1.0}};
  }

  const Quat ref = clean.front();
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  double sw = 0.0;
  for (auto quat : clean)
  {
    if (quaternionDot(ref, quat) < 0.0)
    {
      quat = Quat{{-quat[0], -quat[1], -quat[2], -quat[3]}};
    }
    sx += quat[0];
    sy += quat[1];
    sz += quat[2];
    sw += quat[3];
  }
  return quaternionNormalize(Quat{{sx, sy, sz, sw}});
}

Quat rpyDegToQuaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double deg_to_rad = M_PI / 180.0;
  const double roll = roll_deg * deg_to_rad;
  const double pitch = pitch_deg * deg_to_rad;
  const double yaw = yaw_deg * deg_to_rad;
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double qw = (cr * cp * cy) + (sr * sp * sy);
  const double qx = (sr * cp * cy) - (cr * sp * sy);
  const double qy = (cr * sp * cy) + (sr * cp * sy);
  const double qz = (cr * cp * sy) - (sr * sp * cy);
  return quaternionNormalize(Quat{{qx, qy, qz, qw}});
}

Quat quaternionMultiply(const Quat &lhs, const Quat &rhs)
{
  const double lx = lhs[0];
  const double ly = lhs[1];
  const double lz = lhs[2];
  const double lw = lhs[3];
  const double rx = rhs[0];
  const double ry = rhs[1];
  const double rz = rhs[2];
  const double rw = rhs[3];
  return quaternionNormalize(Quat{{
    (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
    (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
    (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
    (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz)}});
}

Quat quaternionConjugate(const Quat &quat)
{
  const Quat q = quaternionNormalize(quat);
  return Quat{{-q[0], -q[1], -q[2], q[3]}};
}

Vec3 quaternionRotateVector(const Quat &quat, const Vec3 &vec)
{
  const Quat q = quaternionNormalize(quat);
  const Vec3 q_vec{{q[0], q[1], q[2]}};
  const Vec3 t0 = vCross(q_vec, vec);
  const Vec3 t{{2.0 * t0[0], 2.0 * t0[1], 2.0 * t0[2]}};
  const Vec3 q_cross_t = vCross(q_vec, t);
  return Vec3{{
    vec[0] + (q[3] * t[0]) + q_cross_t[0],
    vec[1] + (q[3] * t[1]) + q_cross_t[1],
    vec[2] + (q[3] * t[2]) + q_cross_t[2]}};
}

std::array<double, 3> quaternionToRpyDeg(const Quat &quat)
{
  const Quat q = quaternionNormalize(quat);
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];
  const double sinr_cosp = 2.0 * ((w * x) + (y * z));
  const double cosr_cosp = 1.0 - (2.0 * ((x * x) + (y * y)));
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * ((w * y) - (z * x));
  double pitch = 0.0;
  if (std::fabs(sinp) >= 1.0)
  {
    pitch = std::copysign(M_PI / 2.0, sinp);
  }
  else
  {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)));
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  const double rad_to_deg = 180.0 / M_PI;
  return std::array<double, 3>{{roll * rad_to_deg, pitch * rad_to_deg, yaw * rad_to_deg}};
}

Pose7 transformPoseCompose(const Pose7 &parent_to_child, const Pose7 &child_to_grandchild)
{
  const Vec3 parent_t{{parent_to_child[0], parent_to_child[1], parent_to_child[2]}};
  const Quat parent_q{{parent_to_child[3], parent_to_child[4], parent_to_child[5], parent_to_child[6]}};
  const Vec3 child_t{{child_to_grandchild[0], child_to_grandchild[1], child_to_grandchild[2]}};
  const Quat child_q{{child_to_grandchild[3], child_to_grandchild[4], child_to_grandchild[5], child_to_grandchild[6]}};
  const Vec3 rotated_child = quaternionRotateVector(parent_q, child_t);
  const Quat q = quaternionMultiply(parent_q, child_q);
  return Pose7{{
    parent_t[0] + rotated_child[0],
    parent_t[1] + rotated_child[1],
    parent_t[2] + rotated_child[2],
    q[0], q[1], q[2], q[3]}};
}

Pose7 transformPoseInverse(const Pose7 &parent_to_child)
{
  const Vec3 t{{parent_to_child[0], parent_to_child[1], parent_to_child[2]}};
  const Quat q{{parent_to_child[3], parent_to_child[4], parent_to_child[5], parent_to_child[6]}};
  const Quat inv_q = quaternionConjugate(q);
  const Vec3 inv_t = quaternionRotateVector(inv_q, Vec3{{-t[0], -t[1], -t[2]}});
  return Pose7{{inv_t[0], inv_t[1], inv_t[2], inv_q[0], inv_q[1], inv_q[2], inv_q[3]}};
}

std::optional<GoalPose> buildAlignGoalPose(
  const Pose7 &target_pose,
  const Pose7 &gripper_to_camera_pose,
  double distance_mm,
  double min_base_z_mm)
{
  const double target_x = target_pose[0];
  const double target_y = target_pose[1];
  const double target_z = target_pose[2];
  const Quat target_quat{{target_pose[3], target_pose[4], target_pose[5], target_pose[6]}};
  if (distance_mm <= 0.0)
  {
    return std::nullopt;
  }

  const auto target_x_axis = vUnit(quaternionRotateVector(target_quat, Vec3{{1.0, 0.0, 0.0}}));
  const auto target_y_axis = vUnit(quaternionRotateVector(target_quat, Vec3{{0.0, 1.0, 0.0}}));
  const auto target_z_axis = vUnit(quaternionRotateVector(target_quat, Vec3{{0.0, 0.0, 1.0}}));
  if (!target_x_axis || !target_y_axis || !target_z_axis)
  {
    return std::nullopt;
  }

  const Vec3 camera_x_axis = *target_x_axis;
  const Vec3 camera_y_axis{{-(*target_y_axis)[0], -(*target_y_axis)[1], -(*target_y_axis)[2]}};
  const Vec3 camera_z_axis{{-(*target_z_axis)[0], -(*target_z_axis)[1], -(*target_z_axis)[2]}};
  const std::array<std::array<double, 3>, 3> camera_rotation{{
    std::array<double, 3>{{camera_x_axis[0], camera_y_axis[0], camera_z_axis[0]}},
    std::array<double, 3>{{camera_x_axis[1], camera_y_axis[1], camera_z_axis[1]}},
    std::array<double, 3>{{camera_x_axis[2], camera_y_axis[2], camera_z_axis[2]}}}};
  const Quat desired_camera_quat = quaternionFromRotationMatrix(camera_rotation);
  const Pose7 desired_camera_pose{{
    target_x - (camera_z_axis[0] * distance_mm),
    target_y - (camera_z_axis[1] * distance_mm),
    target_z - (camera_z_axis[2] * distance_mm),
    desired_camera_quat[0], desired_camera_quat[1], desired_camera_quat[2], desired_camera_quat[3]}};
  const Pose7 desired_gripper_pose = transformPoseCompose(
    desired_camera_pose,
    transformPoseInverse(gripper_to_camera_pose));
  if (desired_gripper_pose[2] < min_base_z_mm)
  {
    return std::nullopt;
  }

  const Quat q{{desired_gripper_pose[3], desired_gripper_pose[4], desired_gripper_pose[5], desired_gripper_pose[6]}};
  const auto rpy = quaternionToRpyDeg(q);
  return GoalPose{desired_gripper_pose[0], desired_gripper_pose[1], desired_gripper_pose[2], rpy[0], rpy[1], rpy[2]};
}

std::filesystem::path expandUserPath(const std::string &raw)
{
  if (raw.empty() || raw[0] != '~')
  {
    return std::filesystem::path(raw);
  }
  const char *home = std::getenv("HOME");
  if (home == nullptr)
  {
    return std::filesystem::path(raw);
  }
  if (raw.size() == 1)
  {
    return std::filesystem::path(home);
  }
  if (raw[1] == '/')
  {
    return std::filesystem::path(home) / raw.substr(2);
  }
  return std::filesystem::path(raw);
}

std::filesystem::path defaultPlatformCalibrationDir()
{
  const char *home = std::getenv("HOME");
  if (home == nullptr)
  {
    return std::filesystem::path("config") / "platform";
  }
  return std::filesystem::path(home) / "DOBOT_pickn_place" / "config" / "platform";
}

std::string exceptionMessage(const std::exception_ptr &eptr)
{
  if (!eptr)
  {
    return std::string();
  }
  try
  {
    std::rethrow_exception(eptr);
  }
  catch (const std::exception &ex)
  {
    return ex.what();
  }
  catch (...)
  {
    return "unknown exception";
  }
}

class BinTeachNode : public rclcpp::Node
{
public:
  explicit BinTeachNode()
  : rclcpp::Node("bin_teach"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, false),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*this)),
    static_platform_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this))
  {
    marker_prefix_ = declare_parameter<std::string>("marker_prefix", "aruco_marker");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "calibrated_camera_link");
    target_frame_ = declare_parameter<std::string>("target_frame", "bin_teach_target");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    gripper_frame_ = declare_parameter<std::string>("gripper_frame", "Link6");
    camera_frame_ = declare_parameter<std::string>("camera_frame", parent_frame_);
    use_platform_calibration_ = declare_parameter<bool>("use_platform_calibration", true);
    publish_static_platform_tf_ = declare_parameter<bool>("publish_static_platform_tf", true);
    auto_discover_platform_calibration_ = declare_parameter<bool>("auto_discover_platform_calibration", true);
    platform_calibration_dir_ = declare_parameter<std::string>(
      "platform_calibration_dir",
      defaultPlatformCalibrationDir().string());
    platform_calibration_file_ = declare_parameter<std::string>("platform_calibration_file", "");
    color_topic_ = declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    const std::string default_output_dir = (std::filesystem::path(std::getenv("HOME") ? std::getenv("HOME") : "") / "DOBOT_pickn_place" / "config" / "bin_teach").string();
    const std::string bin_teach_dir_alias = declare_parameter<std::string>("bin_teach_dir", "");
    const std::string output_dir_param = declare_parameter<std::string>(
      "output_dir",
      bin_teach_dir_alias.empty() ? default_output_dir : bin_teach_dir_alias);
    output_dir_ = expandUserPath(output_dir_param);
    default_bin_name_ = declare_parameter<std::string>("bin_name", "bin");
    bin_frame_prefix_ = declare_parameter<std::string>("bin_frame_prefix", "bin");
    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "/aruco_overlay");
    use_aruco_overlay_ = declare_parameter<bool>("use_aruco_overlay", false);
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/aruco_detections");
    motion_service_root_ = declare_parameter<std::string>("motion_service_root", "/dobot_bringup_ros2/srv");
    while (!motion_service_root_.empty() && motion_service_root_.back() == '/')
    {
      motion_service_root_.pop_back();
    }
    align_distance_mm_ = declare_parameter<double>("align_distance_mm", 300.0);
    align_pose_speed_percent_ = declare_parameter<int>("align_pose_speed_percent", 100);
    align_visible_max_age_sec_ = declare_parameter<double>("align_visible_max_age_sec", 0.75);
    align_initial_timeout_sec_ = declare_parameter<double>("align_initial_timeout_sec", 30.0);
    align_min_base_z_mm_ = declare_parameter<double>("align_min_base_z_mm", 200.0);
    align_goal_pos_tol_mm_ = declare_parameter<double>("align_goal_pos_tol_mm", 8.0);
    align_goal_rot_tol_deg_ = declare_parameter<double>("align_goal_rot_tol_deg", 3.0);
    align_up_max_distance_mm_ = declare_parameter<double>("align_up_max_distance_mm", 400.0);
    align_up_speed_factor_percent_ = declare_parameter<int>("align_up_speed_factor_percent", 5);
    align_up_timeout_sec_ = declare_parameter<double>("align_up_timeout_sec", 60.0);
    align_up_user_index_ = declare_parameter<int>("align_up_user_index", 0);
    align_restore_speed_factor_percent_ = declare_parameter<int>(
      "align_restore_speed_factor_percent",
      align_pose_speed_percent_);

    allowed_marker_ids_.assign(kBinTeachAllowedMarkerIds.begin(), kBinTeachAllowedMarkerIds.end());
    required_marker_count_ = kBinTeachRequiredMarkerCount;

    if (use_platform_calibration_)
    {
      if (platform_calibration_file_.empty() && auto_discover_platform_calibration_)
      {
        platform_calibration_file_ = findLatestPlatformCalibrationFile();
      }
      if (platform_calibration_file_.empty())
      {
        throw std::runtime_error(
          "use_platform_calibration=true but no platform calibration file is available. "
          "Run camera_calibration platform_teach first, or set use_platform_calibration:=false for legacy camera-relative bin teach.");
      }

      std::string reason;
      if (!loadPlatformCalibrationFromFile(platform_calibration_file_, reason))
      {
        throw std::runtime_error(
          "Failed to load platform calibration file '" + platform_calibration_file_ + "': " + reason);
      }

      parent_frame_ = platform_frame_;
      if (publish_static_platform_tf_)
      {
        publishPlatformTransform();
      }
      RCLCPP_INFO(
        get_logger(),
        "Platform calibration loaded from %s. Saving bin poses in %s. Static TF %s -> %s: %s",
        platform_calibration_file_.c_str(),
        platform_frame_.c_str(),
        platform_parent_frame_.c_str(),
        platform_frame_.c_str(),
        publish_static_platform_tf_ ? "enabled" : "disabled");
    }

    movj_client_ = create_client<MovJSrv>(motion_service_root_ + "/MovJ");
    relm_user_client_ = create_client<RelMovLUserSrv>(motion_service_root_ + "/RelMovLUser");
    speed_factor_client_ = create_client<SpeedFactorSrv>(motion_service_root_ + "/SpeedFactor");
    stop_client_ = create_client<StopSrv>(motion_service_root_ + "/Stop");

    tcp_sub_ = create_subscription<ToolVectorActualMsg>(
      "dobot_msgs_v4/msg/ToolVectorActual",
      10,
      [this](const ToolVectorActualMsg::SharedPtr msg) { tcpCallback(*msg); });
    detections_sub_ = create_subscription<MarkerDetectionsMsg>(
      detections_topic_,
      rclcpp::SensorDataQoS(),
      [this](const MarkerDetectionsMsg::SharedPtr msg) { detectionsCallback(*msg); });
    color_sub_ = create_subscription<ImageMsg>(
      color_topic_,
      rclcpp::SensorDataQoS(),
      [this](const ImageMsg::SharedPtr msg) { colorCallback(*msg); });
    if (use_aruco_overlay_)
    {
      overlay_sub_ = create_subscription<ImageMsg>(
        overlay_topic_,
        5,
        [this](const ImageMsg::SharedPtr msg) { overlayCallback(*msg); });
    }
  }

  const std::string &markerPrefix() const { return marker_prefix_; }
  const std::string &parentFrame() const { return parent_frame_; }
  const std::string &targetFrame() const { return target_frame_; }
  const std::string &baseFrame() const { return base_frame_; }
  const std::string &gripperFrame() const { return gripper_frame_; }
  const std::string &cameraFrame() const { return camera_frame_; }
  bool usePlatformCalibration() const { return use_platform_calibration_; }
  const std::string &platformName() const { return platform_name_; }
  const std::string &platformCalibrationFile() const { return platform_calibration_file_; }
  const std::string &colorTopic() const { return color_topic_; }
  const std::filesystem::path &outputDir() const { return output_dir_; }
  const std::string &defaultBinName() const { return default_bin_name_; }
  const std::string &overlayTopic() const { return overlay_topic_; }
  bool useArucoOverlay() const { return use_aruco_overlay_; }
  const std::string &detectionsTopic() const { return detections_topic_; }
  const std::string &motionServiceRoot() const { return motion_service_root_; }
  double alignDistanceMm() const { return align_distance_mm_; }
  int alignPoseSpeedPercent() const { return align_pose_speed_percent_; }
  double alignVisibleMaxAgeSec() const { return align_visible_max_age_sec_; }
  double alignInitialTimeoutSec() const { return align_initial_timeout_sec_; }
  double alignUpMaxDistanceMm() const { return align_up_max_distance_mm_; }
  int alignUpSpeedFactorPercent() const { return align_up_speed_factor_percent_; }
  double alignUpTimeoutSec() const { return align_up_timeout_sec_; }
  int alignRestoreSpeedFactorPercent() const { return align_restore_speed_factor_percent_; }
  const std::vector<int> &allowedMarkerIds() const { return allowed_marker_ids_; }
  int requiredMarkerCount() const { return required_marker_count_; }

  std::string markerFrame(int marker_id) const
  {
    return marker_prefix_ + "_" + std::to_string(marker_id);
  }

  Solution computeSolution(const std::string &bin_name)
  {
    const DetectionSnapshot snapshot = currentDetectionSnapshot();
    if (snapshot.missing_count > 0)
    {
      std::ostringstream message;
      message << "Need " << markerRequirementText() << " in one current detection frame.";
      for (const auto &detail : snapshot.details)
      {
        message << "\n  " << detail;
      }
      throw std::runtime_error(message.str());
    }
    std::vector<int> marker_ids;
    marker_ids.reserve(snapshot.visible.size());
    for (const auto &marker : snapshot.visible)
    {
      marker_ids.push_back(marker.id);
    }
    const std::string source_frame = snapshot.frame ? snapshot.frame->frame_id : parent_frame_;
    const auto teach_markers = markersInTeachFrame(snapshot.visible, source_frame);
    Solution solution = computeCentroidSolutionFromMarkers(
      bin_name,
      marker_ids,
      teach_markers.first,
      teach_markers.second,
      snapshot.frame);
    if (snapshot.frame && canFitDepthPlaneFromDetectionFrame(snapshot.frame->frame_id))
    {
      if (const auto plane = fitDepthPlaneFromMarkerCenters(snapshot.visible, *snapshot.frame, solution.roi_points);
          plane.has_value())
      {
        solution.depth_plane = *plane;
        latest_solution_ = solution;
        publishSolution(solution);
      }
    }
    return solution;
  }

  bool canFitDepthPlaneFromDetectionFrame(const std::string &frame_id) const
  {
    if (frame_id.empty())
    {
      return false;
    }
    if (frame_id == camera_frame_)
    {
      return true;
    }
    return !use_platform_calibration_ && frame_id == parent_frame_;
  }

  static std::optional<std::vector<CornerDot>> cornerDotsFromMarkers(const std::vector<MarkerData> &markers)
  {
    struct Candidate
    {
      int key{0};
      MarkerData marker;
      Vec2 center{{0.0, 0.0}};
      std::vector<Vec2> corners;
    };
    std::vector<Candidate> candidates;
    for (size_t order = 0; order < markers.size(); ++order)
    {
      const auto &marker = markers[order];
      if (!marker.pixel || marker.corners.size() < 4)
      {
        continue;
      }
      Candidate candidate;
      candidate.key = marker.detection_index;
      candidate.marker = marker;
      candidate.center = *marker.pixel;
      candidate.corners.assign(marker.corners.begin(), marker.corners.begin() + 4);
      candidates.push_back(candidate);
    }
    if (candidates.size() < 4)
    {
      return std::nullopt;
    }

    const auto upper_left_it = std::min_element(
      candidates.begin(), candidates.end(),
      [](const Candidate &a, const Candidate &b) { return (a.center[0] + a.center[1]) < (b.center[0] + b.center[1]); });
    const auto upper_right_it = std::max_element(
      candidates.begin(), candidates.end(),
      [](const Candidate &a, const Candidate &b) { return (a.center[0] - a.center[1]) < (b.center[0] - b.center[1]); });
    const auto lower_right_it = std::max_element(
      candidates.begin(), candidates.end(),
      [](const Candidate &a, const Candidate &b) { return (a.center[0] + a.center[1]) < (b.center[0] + b.center[1]); });
    const auto lower_left_it = std::max_element(
      candidates.begin(), candidates.end(),
      [](const Candidate &a, const Candidate &b) { return (a.center[1] - a.center[0]) < (b.center[1] - b.center[0]); });
    const std::array<const Candidate *, 4> ordered{{
      &(*upper_left_it), &(*upper_right_it), &(*lower_right_it), &(*lower_left_it)}};
    std::set<int> keys;
    for (const Candidate *item : ordered)
    {
      keys.insert(item->key);
    }
    if (keys.size() < 4)
    {
      return std::nullopt;
    }

    const std::array<std::string, 4> roles{{"upper_left", "upper_right", "lower_right", "lower_left"}};
    std::vector<CornerDot> dots;
    dots.reserve(4);
    for (size_t i = 0; i < ordered.size(); ++i)
    {
      const Candidate *item = ordered[i];
      auto corner_it = item->corners.begin();
      if (roles[i] == "upper_left")
      {
        corner_it = std::min_element(
          item->corners.begin(), item->corners.end(),
          [](const Vec2 &a, const Vec2 &b) { return (a[0] + a[1]) < (b[0] + b[1]); });
      }
      else if (roles[i] == "upper_right")
      {
        corner_it = std::max_element(
          item->corners.begin(), item->corners.end(),
          [](const Vec2 &a, const Vec2 &b) { return (a[0] - a[1]) < (b[0] - b[1]); });
      }
      else if (roles[i] == "lower_right")
      {
        corner_it = std::max_element(
          item->corners.begin(), item->corners.end(),
          [](const Vec2 &a, const Vec2 &b) { return (a[0] + a[1]) < (b[0] + b[1]); });
      }
      else
      {
        corner_it = std::max_element(
          item->corners.begin(), item->corners.end(),
          [](const Vec2 &a, const Vec2 &b) { return (a[1] - a[0]) < (b[1] - b[0]); });
      }
      CornerDot dot;
      dot.role = roles[i];
      dot.x = (*corner_it)[0];
      dot.y = (*corner_it)[1];
      dot.marker_id = item->marker.id;
      dot.detection_index = item->marker.detection_index;
      dot.same_id_index = item->marker.same_id_index;
      dots.push_back(dot);
    }
    return dots;
  }

  Solution addImageRoiToSolution(Solution solution, const std::vector<MarkerData> &markers, const DetectionFrame &frame)
  {
    const auto corner_dots = cornerDotsFromMarkers(markers);
    if (!corner_dots)
    {
      throw std::runtime_error("Need four marker pixel-corner detections to save bin ROI dots.");
    }

    solution.image_width = frame.image_width;
    solution.image_height = frame.image_height;
    solution.dot_coordinate_frame = "color_image_pixels";
    for (const auto &dot : *corner_dots)
    {
      solution.roi_points.push_back(static_cast<int>(std::lround(dot.x)));
      solution.roi_points.push_back(static_cast<int>(std::lround(dot.y)));
      if (solution.image_width > 0 && solution.image_height > 0)
      {
        solution.roi_points_normalized.push_back(dot.x / static_cast<double>(solution.image_width));
        solution.roi_points_normalized.push_back(dot.y / static_cast<double>(solution.image_height));
      }
    }
    solution.dot_positions = *corner_dots;
    return solution;
  }

  CentroidPose markerCentroidPose(const std::vector<MarkerData> &markers) const
  {
    CentroidPose output;
    std::vector<Quat> quaternions;
    for (size_t order = 0; order < markers.size(); ++order)
    {
      const auto &marker = markers[order];
      const int marker_id = marker.id;
      const std::string key = "marker_" + std::to_string(order + 1) + "_id_" + std::to_string(marker_id);
      output.positions[key] = marker.position_m;
      output.marker_observations.push_back(MarkerObservation{key, marker_id, marker.detection_index, marker.same_id_index});
      quaternions.push_back(marker.orientation);
    }

    if (output.positions.empty())
    {
      return output;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto &entry : output.positions)
    {
      const Vec3 &position = entry.second;
      min_x = std::min(min_x, position[0]);
      min_y = std::min(min_y, position[1]);
      min_z = std::min(min_z, position[2]);
      max_x = std::max(max_x, position[0]);
      max_y = std::max(max_y, position[1]);
      max_z = std::max(max_z, position[2]);
    }
    output.center = Vec3{{(min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5}};
    output.quaternion = averageQuaternions(quaternions);
    output.x_axis = vUnit(quaternionRotateVector(output.quaternion, Vec3{{1.0, 0.0, 0.0}})).value_or(Vec3{{1.0, 0.0, 0.0}});
    output.y_axis = vUnit(quaternionRotateVector(output.quaternion, Vec3{{0.0, 1.0, 0.0}})).value_or(Vec3{{0.0, 1.0, 0.0}});
    output.z_axis = vUnit(quaternionRotateVector(output.quaternion, Vec3{{0.0, 0.0, 1.0}})).value_or(Vec3{{0.0, 0.0, 1.0}});
    return output;
  }

  Solution computeCentroidSolutionFromMarkers(
    const std::string &bin_name,
    const std::vector<int> &marker_ids,
    const std::vector<MarkerData> &markers,
    const std::string &parent_frame,
    const std::optional<DetectionFrame> &frame = std::nullopt)
  {
    const CentroidPose pose = markerCentroidPose(markers);
    const std::string safe_name = sanitizeName(bin_name);
    Solution solution;
    solution.bin_name = safe_name;
    solution.frame_id = bin_frame_prefix_ + "_" + safe_name + "_frame";
    solution.parent_frame = parent_frame;
    solution.platform_reference_enabled = use_platform_calibration_;
    solution.platform_name = platform_name_;
    solution.platform_calibration_file = platform_calibration_file_;
    solution.platform_parent_frame = platform_parent_frame_;
    solution.platform_frame = platform_frame_;
    solution.marker_prefix = marker_prefix_;
    solution.allowed_marker_ids = allowed_marker_ids_;
    solution.required_marker_count = required_marker_count_;
    solution.marker_ids = marker_ids;
    solution.center_method = "marker_position_bounds_center";
    solution.origin = pose.center;
    solution.x_axis = pose.x_axis;
    solution.y_axis = pose.y_axis;
    solution.z_axis = pose.z_axis;
    solution.quaternion = pose.quaternion;
    solution.marker_positions = pose.positions;
    solution.marker_observations = pose.marker_observations;
    if (frame)
    {
      solution = addImageRoiToSolution(solution, markers, *frame);
    }
    latest_solution_ = solution;
    publishSolution(solution);
    return solution;
  }

  Solution computeCentroidTargetFromMarkers(
    const std::vector<int> &marker_ids,
    const std::vector<MarkerData> &markers,
    const std::string &parent_frame) const
  {
    const CentroidPose pose = markerCentroidPose(markers);
    Solution solution;
    solution.frame_id = target_frame_;
    solution.parent_frame = parent_frame;
    solution.origin = pose.center;
    solution.x_axis = pose.x_axis;
    solution.y_axis = pose.y_axis;
    solution.z_axis = pose.z_axis;
    solution.quaternion = pose.quaternion;
    solution.marker_ids = marker_ids;
    solution.center_method = "marker_position_bounds_center";
    solution.marker_positions = pose.positions;
    solution.marker_observations = pose.marker_observations;
    return solution;
  }

  void publishTargetFromDetectionFrame(const DetectionFrame &frame)
  {
    const auto visible = selectVisibleBinMarkers(frame.markers);
    if (static_cast<int>(visible.size()) < required_marker_count_)
    {
      return;
    }
    std::vector<int> marker_ids;
    marker_ids.reserve(visible.size());
    for (const auto &marker : visible)
    {
      marker_ids.push_back(marker.id);
    }
    try
    {
      const auto teach_markers = markersInTeachFrame(visible, frame.frame_id, false);
      Solution solution = computeCentroidTargetFromMarkers(marker_ids, teach_markers.first, teach_markers.second);
      latest_target_solution_ = solution;
      publishSolution(solution);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Cannot publish bin_teach target frame: %s",
        ex.what());
    }
  }

  void publishSolution(const Solution &solution)
  {
    TransformStampedMsg msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = solution.parent_frame;
    msg.child_frame_id = solution.frame_id;
    msg.transform.translation.x = solution.origin[0];
    msg.transform.translation.y = solution.origin[1];
    msg.transform.translation.z = solution.origin[2];
    msg.transform.rotation.x = solution.quaternion[0];
    msg.transform.rotation.y = solution.quaternion[1];
    msg.transform.rotation.z = solution.quaternion[2];
    msg.transform.rotation.w = solution.quaternion[3];
    tf_broadcaster_->sendTransform(msg);
  }

  void republishLatest()
  {
    if (latest_target_solution_)
    {
      publishSolution(*latest_target_solution_);
    }
    if (latest_solution_)
    {
      publishSolution(*latest_solution_);
    }
    if (latest_align_goal_)
    {
      publishAlignGoal(*latest_align_goal_);
    }
  }

  std::filesystem::path saveSolution(const Solution &solution)
  {
    std::filesystem::create_directories(output_dir_);
    const std::filesystem::path path = output_dir_ / (solution.bin_name + ".yaml");
    const std::time_t now_time = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &now_time);
#else
    localtime_r(&now_time, &tm);
#endif
    std::ostringstream stamp_stream;
    stamp_stream << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

    auto vec_line = [](const std::string &name, const Vec3 &value, const std::string &indent = "    ") {
      std::ostringstream stream;
      stream << indent << name << ": {x: " << formatDouble(value[0], 9)
             << ", y: " << formatDouble(value[1], 9)
             << ", z: " << formatDouble(value[2], 9) << "}\n";
      return stream.str();
    };

    const auto arm_pose = currentTcp();
    std::ostringstream text;
    text << "bin_teach:\n";
    text << "  bin_name: " << solution.bin_name << "\n";
    text << "  created_at: \"" << stamp_stream.str() << "\"\n";
    text << "  parent_frame: " << solution.parent_frame << "\n";
    text << "  bin_frame: " << solution.frame_id << "\n";
    text << "  platform_reference:\n";
    text << "    enabled: " << (solution.platform_reference_enabled ? "true" : "false") << "\n";
    if (solution.platform_reference_enabled)
    {
      text << "    platform_name: " << solution.platform_name << "\n";
      text << "    platform_frame: " << solution.platform_frame << "\n";
      text << "    platform_parent_frame: " << solution.platform_parent_frame << "\n";
      text << "    platform_calibration_file: " << solution.platform_calibration_file << "\n";
    }
    text << "  marker_prefix: " << solution.marker_prefix << "\n";
    text << "  allowed_marker_ids: [";
    for (size_t i = 0; i < solution.allowed_marker_ids.size(); ++i)
    {
      if (i > 0) { text << ", "; }
      text << solution.allowed_marker_ids[i];
    }
    text << "]\n";
    text << "  required_marker_count: " << solution.required_marker_count << "\n";
    text << "  marker_ids: [";
    for (size_t i = 0; i < solution.marker_ids.size(); ++i)
    {
      if (i > 0) { text << ", "; }
      text << solution.marker_ids[i];
    }
    text << "]\n";
    text << "  center_method: " << solution.center_method << "\n";
    text << "  transform:\n";
    text << "    translation: {x: " << formatDouble(solution.origin[0], 9)
         << ", y: " << formatDouble(solution.origin[1], 9)
         << ", z: " << formatDouble(solution.origin[2], 9) << "}\n";
    text << "    rotation: {x: " << formatDouble(solution.quaternion[0], 9)
         << ", y: " << formatDouble(solution.quaternion[1], 9)
         << ", z: " << formatDouble(solution.quaternion[2], 9)
         << ", w: " << formatDouble(solution.quaternion[3], 9) << "}\n";
    text << "  arm_pose_at_save:\n";
    text << "    source_topic: dobot_msgs_v4/msg/ToolVectorActual\n";
    text << "    base_frame: " << base_frame_ << "\n";
    text << "    gripper_frame: " << gripper_frame_ << "\n";
    text << "    position_units: mm\n";
    text << "    rotation_units: deg\n";
    if (!arm_pose)
    {
      text << "    valid: false\n";
    }
    else
    {
      text << "    valid: true\n";
      text << "    tcp: {x: " << formatDouble((*arm_pose)[0], 6)
           << ", y: " << formatDouble((*arm_pose)[1], 6)
           << ", z: " << formatDouble((*arm_pose)[2], 6)
           << ", rx: " << formatDouble((*arm_pose)[3], 6)
           << ", ry: " << formatDouble((*arm_pose)[4], 6)
           << ", rz: " << formatDouble((*arm_pose)[5], 6) << "}\n";
    }
    if (!solution.roi_points.empty())
    {
      text << "  image:\n";
      text << "    width: " << solution.image_width << "\n";
      text << "    height: " << solution.image_height << "\n";
      text << "    coordinate_frame: " << (solution.dot_coordinate_frame.empty() ? "color_image_pixels" : solution.dot_coordinate_frame) << "\n";
      text << "  roi_points: [";
      for (size_t i = 0; i < solution.roi_points.size(); ++i)
      {
        if (i > 0) { text << ", "; }
        text << solution.roi_points[i];
      }
      text << "]\n";
      if (!solution.roi_points_normalized.empty())
      {
        text << "  roi_points_normalized: [";
        for (size_t i = 0; i < solution.roi_points_normalized.size(); ++i)
        {
          if (i > 0) { text << ", "; }
          text << formatDouble(solution.roi_points_normalized[i], 9);
        }
        text << "]\n";
      }
      text << "  dot_positions:\n";
      for (const auto &dot : solution.dot_positions)
      {
        text << "    - role: " << dot.role << "\n";
        text << "      pixel: {x: " << formatDouble(dot.x, 3) << ", y: " << formatDouble(dot.y, 3) << "}\n";
        text << "      marker_id: " << dot.marker_id << "\n";
        text << "      detection_index: " << dot.detection_index << "\n";
        text << "      same_id_index: " << dot.same_id_index << "\n";
      }
    }
    text << "  depth_plane_enabled: " << (solution.depth_plane.valid ? "true" : "false") << "\n";
    text << "  depth_plane_a: " << formatDouble(solution.depth_plane.a, 12) << "\n";
    text << "  depth_plane_b: " << formatDouble(solution.depth_plane.b, 12) << "\n";
    text << "  depth_plane_c: " << formatDouble(solution.depth_plane.c, 12) << "\n";
    text << "  depth_plane_reference_depth_m: " << formatDouble(solution.depth_plane.reference_depth_m, 12) << "\n";
    text << "  depth_plane_roi: ["
         << solution.depth_plane.roi[0] << ", "
         << solution.depth_plane.roi[1] << ", "
         << solution.depth_plane.roi[2] << ", "
         << solution.depth_plane.roi[3] << "]\n";
    text << "  depth_plane_source: " << solution.depth_plane.source << "\n";
    text << "  depth_plane_source_frame: " << solution.depth_plane.source_frame << "\n";
    text << "  depth_plane_units: m\n";
    if (!solution.depth_plane.samples.empty())
    {
      text << "  depth_plane_samples:\n";
      for (const auto &sample : solution.depth_plane.samples)
      {
        text << "    - x_norm: " << formatDouble(sample.x_norm, 9) << "\n";
        text << "      y_norm: " << formatDouble(sample.y_norm, 9) << "\n";
        text << "      depth_m: " << formatDouble(sample.depth_m, 9) << "\n";
        text << "      marker_id: " << sample.marker_id << "\n";
        text << "      detection_index: " << sample.detection_index << "\n";
        text << "      same_id_index: " << sample.same_id_index << "\n";
      }
    }
    text << "  axes:\n";
    text << vec_line("x", solution.x_axis);
    text << vec_line("y", solution.y_axis);
    text << vec_line("z", solution.z_axis);
    text << "  marker_positions:\n";
    for (const auto &observation : solution.marker_observations)
    {
      text << vec_line(observation.key, solution.marker_positions.at(observation.key));
    }
    text << "  marker_observations:\n";
    for (const auto &observation : solution.marker_observations)
    {
      text << "    - key: " << observation.key << "\n";
      text << "      id: " << observation.id << "\n";
      text << "      detection_index: " << observation.detection_index << "\n";
      text << "      same_id_index: " << observation.same_id_index << "\n";
    }
    text << "  notes: \"transform origin is the center of the configured marker position bounds; marker positions are not averaged. Orientation normal is averaged from marker poses. roi_points are saved in item_teach-compatible color-image pixel order: upper-left, upper-right, lower-right, lower-left. depth_plane_* is fitted from ArUco marker center depths in normalized image coordinates and is the reference plane item_teach should inherit.\"\n";

    std::ofstream output(path);
    output << text.str();
    return path;
  }

  std::optional<TcpPose> currentTcp() const
  {
    return latest_tcp_;
  }

  Pose7 lookupPoseMm(const std::string &parent_frame, const std::string &child_frame, double timeout_sec = 0.15) const
  {
    if (parent_frame == child_frame)
    {
      return Pose7{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
    }
    const auto tf_msg = tf_buffer_.lookupTransform(
      parent_frame,
      child_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(std::max(0.01, timeout_sec)));
    const auto &t = tf_msg.transform.translation;
    const auto &q = tf_msg.transform.rotation;
    return Pose7{{
      static_cast<double>(t.x) * 1000.0,
      static_cast<double>(t.y) * 1000.0,
      static_cast<double>(t.z) * 1000.0,
      static_cast<double>(q.x),
      static_cast<double>(q.y),
      static_cast<double>(q.z),
      static_cast<double>(q.w)}};
  }

  Pose7 lookupPoseMmNoWait(const std::string &parent_frame, const std::string &child_frame) const
  {
    if (parent_frame == child_frame)
    {
      return Pose7{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
    }
    const auto tf_msg = tf_buffer_.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    const auto &t = tf_msg.transform.translation;
    const auto &q = tf_msg.transform.rotation;
    return Pose7{{
      static_cast<double>(t.x) * 1000.0,
      static_cast<double>(t.y) * 1000.0,
      static_cast<double>(t.z) * 1000.0,
      static_cast<double>(q.x),
      static_cast<double>(q.y),
      static_cast<double>(q.z),
      static_cast<double>(q.w)}};
  }

  Pose7 currentGripperPoseMm() const
  {
    const auto tcp = currentTcp();
    if (!tcp)
    {
      throw std::runtime_error("No live TCP pose yet on dobot_msgs_v4/msg/ToolVectorActual.");
    }
    const Quat q = rpyDegToQuaternion((*tcp)[3], (*tcp)[4], (*tcp)[5]);
    return Pose7{{(*tcp)[0], (*tcp)[1], (*tcp)[2], q[0], q[1], q[2], q[3]}};
  }

  Pose7 currentCameraPoseMm() const
  {
    const Pose7 gripper_pose = currentGripperPoseMm();
    const Pose7 gripper_to_camera = lookupPoseMm(gripper_frame_, camera_frame_);
    return transformPoseCompose(gripper_pose, gripper_to_camera);
  }

  Pose7 currentCameraPoseMmNoWait() const
  {
    const Pose7 gripper_pose = currentGripperPoseMm();
    const Pose7 gripper_to_camera = lookupPoseMmNoWait(gripper_frame_, camera_frame_);
    return transformPoseCompose(gripper_pose, gripper_to_camera);
  }

  Pose7 lookupBaseToFramePoseMm(const std::string &child_frame, double timeout_sec = 0.15) const
  {
    try
    {
      return lookupPoseMm(base_frame_, child_frame, timeout_sec);
    }
    catch (const tf2::TransformException &ex)
    {
      if (child_frame == gripper_frame_)
      {
        return currentGripperPoseMm();
      }
      if (child_frame == camera_frame_)
      {
        return currentCameraPoseMm();
      }
      try
      {
        const Pose7 base_to_camera = currentCameraPoseMm();
        const Pose7 camera_to_child = lookupPoseMm(camera_frame_, child_frame, timeout_sec);
        return transformPoseCompose(base_to_camera, camera_to_child);
      }
      catch (...)
      {
        throw;
      }
    }
  }

  Pose7 lookupBaseToFramePoseMmNoWait(const std::string &child_frame) const
  {
    try
    {
      return lookupPoseMmNoWait(base_frame_, child_frame);
    }
    catch (const tf2::TransformException &)
    {
      if (child_frame == gripper_frame_)
      {
        return currentGripperPoseMm();
      }
      if (child_frame == camera_frame_)
      {
        return currentCameraPoseMmNoWait();
      }
      const Pose7 base_to_camera = currentCameraPoseMmNoWait();
      const Pose7 camera_to_child = lookupPoseMmNoWait(camera_frame_, child_frame);
      return transformPoseCompose(base_to_camera, camera_to_child);
    }
  }

  static bool isPlatformCalibrationFilename(const std::filesystem::path &path)
  {
    const std::string filename = path.filename().string();
    return filename == "platform.yaml" ||
           (filename.rfind("platform_calibration_", 0) == 0 && path.extension() == ".yaml");
  }

  std::string findLatestPlatformCalibrationFile() const
  {
    try
    {
      const std::filesystem::path base = expandUserPath(platform_calibration_dir_);
      if (!std::filesystem::exists(base) || !std::filesystem::is_directory(base))
      {
        return {};
      }

      std::filesystem::path latest_path;
      std::filesystem::file_time_type latest_time;
      for (const auto &entry : std::filesystem::directory_iterator(base))
      {
        if (!entry.is_regular_file())
        {
          continue;
        }
        const auto &p = entry.path();
        if (!isPlatformCalibrationFilename(p))
        {
          continue;
        }
        if (std::filesystem::file_size(p) == 0)
        {
          continue;
        }
        if (latest_path.empty() || entry.last_write_time() > latest_time)
        {
          latest_path = p;
          latest_time = entry.last_write_time();
        }
      }
      return latest_path.string();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Failed to discover platform calibration files: %s", ex.what());
      return {};
    }
  }

  bool loadPlatformCalibrationFromFile(const std::string &file_path, std::string &reason)
  {
    const auto resolved_path = expandUserPath(file_path);
    try
    {
      if (!std::filesystem::exists(resolved_path))
      {
        reason = "File does not exist";
        return false;
      }
      if (std::filesystem::file_size(resolved_path) == 0)
      {
        reason = "Platform calibration file is empty";
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

    const auto calib = root["calibration_transform"];
    if (!calib)
    {
      reason = "Missing 'calibration_transform' key";
      return false;
    }
    const auto rot = calib["rotation"];
    const auto trans = calib["translation"];
    const auto metadata = root["metadata"];
    if (!rot || !trans || !metadata)
    {
      reason = "Missing rotation/translation/metadata keys";
      return false;
    }
    if (!metadata["transform_parent_frame"] || !metadata["transform_child_frame"])
    {
      reason = "Missing metadata transform_parent_frame/transform_child_frame";
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
      platform_pose_m_[0] = trans["x"].as<double>();
      platform_pose_m_[1] = trans["y"].as<double>();
      platform_pose_m_[2] = trans["z"].as<double>();
      platform_parent_frame_ = metadata["transform_parent_frame"].as<std::string>();
      platform_frame_ = metadata["transform_child_frame"].as<std::string>();
      platform_name_ = metadata["platform_name"] ? metadata["platform_name"].as<std::string>() : platform_frame_;
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Failed to parse platform calibration: ") + ex.what();
      return false;
    }

    const double norm = std::sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw));
    if (norm < 1e-9)
    {
      reason = "Invalid quaternion (zero norm)";
      return false;
    }
    const double inv = 1.0 / norm;
    platform_pose_m_[3] = qx * inv;
    platform_pose_m_[4] = qy * inv;
    platform_pose_m_[5] = qz * inv;
    platform_pose_m_[6] = qw * inv;
    platform_calibration_file_ = resolved_path.string();
    platform_calibration_loaded_ = true;
    if (platform_parent_frame_ != base_frame_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Platform calibration parent frame is %s, but bin_teach base_frame is %s. TF lookup will be used for frame composition.",
        platform_parent_frame_.c_str(),
        base_frame_.c_str());
    }
    return true;
  }

  void publishPlatformTransform()
  {
    if (!static_platform_tf_broadcaster_ || !platform_calibration_loaded_)
    {
      return;
    }

    TransformStampedMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = platform_parent_frame_;
    msg.child_frame_id = platform_frame_;
    msg.transform.translation.x = platform_pose_m_[0];
    msg.transform.translation.y = platform_pose_m_[1];
    msg.transform.translation.z = platform_pose_m_[2];
    msg.transform.rotation.x = platform_pose_m_[3];
    msg.transform.rotation.y = platform_pose_m_[4];
    msg.transform.rotation.z = platform_pose_m_[5];
    msg.transform.rotation.w = platform_pose_m_[6];
    static_platform_tf_broadcaster_->sendTransform(msg);
  }

  static MarkerData transformMarkerWithPose(const MarkerData &marker, const Pose7 &target_from_source_m)
  {
    const Pose7 source_to_marker_m{{
      marker.position_m[0],
      marker.position_m[1],
      marker.position_m[2],
      marker.orientation[0],
      marker.orientation[1],
      marker.orientation[2],
      marker.orientation[3]}};
    const Pose7 target_to_marker_m = transformPoseCompose(target_from_source_m, source_to_marker_m);
    MarkerData output = marker;
    output.position_m = Vec3{{target_to_marker_m[0], target_to_marker_m[1], target_to_marker_m[2]}};
    output.orientation = Quat{{target_to_marker_m[3], target_to_marker_m[4], target_to_marker_m[5], target_to_marker_m[6]}};
    return output;
  }

  std::pair<std::vector<MarkerData>, std::string> markersInTeachFrame(
    const std::vector<MarkerData> &markers,
    const std::string &source_frame,
    bool allow_tcp_fallback = true) const
  {
    if (!use_platform_calibration_)
    {
      return {markers, source_frame.empty() ? parent_frame_ : source_frame};
    }
    if (!platform_calibration_loaded_)
    {
      throw std::runtime_error("Platform calibration is enabled but no platform calibration is loaded.");
    }
    if (source_frame == platform_frame_)
    {
      return {markers, platform_frame_};
    }

    Pose7 platform_from_source_m;
    bool have_transform = false;
    try
    {
      const auto tf_msg = tf_buffer_.lookupTransform(
        platform_frame_,
        source_frame,
        tf2::TimePointZero);
      const auto &t = tf_msg.transform.translation;
      const auto &q = tf_msg.transform.rotation;
      platform_from_source_m = Pose7{{
        static_cast<double>(t.x),
        static_cast<double>(t.y),
        static_cast<double>(t.z),
        static_cast<double>(q.x),
        static_cast<double>(q.y),
        static_cast<double>(q.z),
        static_cast<double>(q.w)}};
      have_transform = true;
    }
    catch (const tf2::TransformException &tf_ex)
    {
      if (allow_tcp_fallback && platform_parent_frame_ == base_frame_ && source_frame == camera_frame_)
      {
        try
        {
          const Pose7 base_to_camera_mm = lookupBaseToFramePoseMmNoWait(camera_frame_);
          const Pose7 base_to_camera_m{{
            base_to_camera_mm[0] / 1000.0,
            base_to_camera_mm[1] / 1000.0,
            base_to_camera_mm[2] / 1000.0,
            base_to_camera_mm[3],
            base_to_camera_mm[4],
            base_to_camera_mm[5],
            base_to_camera_mm[6]}};
          platform_from_source_m = transformPoseCompose(transformPoseInverse(platform_pose_m_), base_to_camera_m);
          have_transform = true;
        }
        catch (const std::exception &fallback_ex)
        {
          throw std::runtime_error(
            "Cannot transform detections from " + source_frame + " into platform frame " + platform_frame_ +
            ". TF error: " + tf_ex.what() + ". TCP fallback error: " + fallback_ex.what());
        }
      }
      else
      {
        throw std::runtime_error(
          "Cannot transform detections from " + source_frame + " into platform frame " + platform_frame_ +
          ": " + tf_ex.what());
      }
    }

    if (!have_transform)
    {
      throw std::runtime_error("Cannot transform detections into platform frame " + platform_frame_ + ".");
    }

    std::vector<MarkerData> transformed;
    transformed.reserve(markers.size());
    for (const auto &marker : markers)
    {
      transformed.push_back(transformMarkerWithPose(marker, platform_from_source_m));
    }
    return {transformed, platform_frame_};
  }

  static double markerSpanMm(const Solution &solution)
  {
    std::vector<Vec3> positions;
    for (const auto &entry : solution.marker_positions)
    {
      positions.push_back(entry.second);
    }
    double span_m = 0.0;
    for (size_t i = 0; i < positions.size(); ++i)
    {
      for (size_t j = i + 1; j < positions.size(); ++j)
      {
        span_m = std::max(span_m, vNorm(vSub(positions[i], positions[j])));
      }
    }
    return span_m * 1000.0;
  }

  std::string markerRequirementText() const
  {
    return std::to_string(required_marker_count_) + " visible bin markers with IDs " + formatMarkerIds(allowed_marker_ids_, "or");
  }

  std::vector<MarkerData> selectVisibleBinMarkers(const std::vector<MarkerData> &markers) const
  {
    std::set<int> allowed(allowed_marker_ids_.begin(), allowed_marker_ids_.end());
    std::vector<MarkerData> selected;
    for (const auto &marker : markers)
    {
      if (allowed.count(marker.id) > 0)
      {
        selected.push_back(marker);
      }
    }
    std::sort(selected.begin(), selected.end(), [](const MarkerData &a, const MarkerData &b) {
      return a.detection_index < b.detection_index;
    });
    if (selected.size() > static_cast<size_t>(required_marker_count_))
    {
      selected.resize(required_marker_count_);
    }
    return selected;
  }

  DetectionSnapshot currentDetectionSnapshot(
    const std::optional<std::vector<int>> &marker_ids = std::nullopt,
    const std::optional<double> &max_age_sec = std::nullopt) const
  {
    const std::vector<int> allowed_ids = marker_ids.value_or(allowed_marker_ids_);
    DetectionSnapshot output;
    if (!latest_detection_frame_)
    {
      output.missing_count = 1;
      output.details.push_back("No current ArUco detection frame yet.");
      return output;
    }

    const double max_age = max_age_sec.value_or(align_visible_max_age_sec_);
    const double age_sec = secondsSince(latest_detection_frame_->received_monotonic);
    output.frame = latest_detection_frame_;
    if (age_sec > max_age)
    {
      output.missing_count = 1;
      output.details.push_back("Latest ArUco detection frame is stale: " + formatDouble(age_sec, 2) + "s.");
      return output;
    }

    const std::string frame_id = latest_detection_frame_->frame_id;
    if (frame_id != parent_frame_ && frame_id != camera_frame_)
    {
      output.missing_count = 1;
      output.details.push_back("Detection frame is " + frame_id + ", expected " + parent_frame_ + " or " + camera_frame_ + ".");
      return output;
    }

    output.details.push_back("Detection frame age " + formatDouble(age_sec, 2) + "s in " + frame_id + ".");
    std::set<int> allowed(allowed_ids.begin(), allowed_ids.end());
    for (const auto &marker : latest_detection_frame_->markers)
    {
      if (allowed.count(marker.id) > 0)
      {
        output.visible.push_back(marker);
      }
    }
    std::sort(output.visible.begin(), output.visible.end(), [](const MarkerData &a, const MarkerData &b) {
      return a.detection_index < b.detection_index;
    });
    std::vector<int> visible_ids;
    for (const auto &marker : output.visible)
    {
      visible_ids.push_back(marker.id);
    }
    output.missing_count = std::max(0, required_marker_count_ - static_cast<int>(output.visible.size()));
    if (output.missing_count > 0)
    {
      output.details.push_back(
        "Need " + markerRequirementText() + "; currently see " +
        std::to_string(output.visible.size()) + "/" + std::to_string(required_marker_count_) +
        " allowed marker(s).");
    }
    else if (static_cast<int>(output.visible.size()) > required_marker_count_)
    {
      output.visible.resize(required_marker_count_);
      visible_ids.clear();
      for (const auto &marker : output.visible)
      {
        visible_ids.push_back(marker.id);
      }
    }

    if (!output.visible.empty())
    {
      std::ostringstream stream;
      stream << "Visible allowed IDs: ";
      for (size_t i = 0; i < visible_ids.size(); ++i)
      {
        if (i > 0) { stream << ", "; }
        stream << visible_ids[i];
      }
      output.details.push_back(stream.str());
    }
    else
    {
      output.details.push_back("No visible markers with allowed IDs " + formatMarkerIds(allowed_ids, "or") + ".");
    }
    for (const auto &marker : output.visible)
    {
      std::ostringstream stream;
      stream << markerFrame(marker.id) << "#" << marker.same_id_index
             << ": x=" << formatDouble(marker.position_m[0], 3)
             << " y=" << formatDouble(marker.position_m[1], 3)
             << " z=" << formatDouble(marker.position_m[2], 3) << " m";
      output.details.push_back(stream.str());
    }
    return output;
  }

  std::pair<std::vector<Vec3>, std::vector<std::string>> markerVisibilitySnapshot(
    const std::optional<std::vector<int>> &marker_ids = std::nullopt,
    const std::optional<double> &max_age_sec = std::nullopt) const
  {
    const DetectionSnapshot snapshot = currentDetectionSnapshot(marker_ids, max_age_sec);
    std::vector<Vec3> visible_mm;
    visible_mm.reserve(snapshot.visible.size());
    for (const auto &marker : snapshot.visible)
    {
      visible_mm.push_back(Vec3{{
        marker.position_m[0] * 1000.0,
        marker.position_m[1] * 1000.0,
        marker.position_m[2] * 1000.0}});
    }
    return {visible_mm, snapshot.details};
  }

  DetectionSnapshot markerVisibilitySnapshotFull(
    const std::optional<std::vector<int>> &marker_ids = std::nullopt,
    const std::optional<double> &max_age_sec = std::nullopt) const
  {
    return currentDetectionSnapshot(marker_ids, max_age_sec);
  }

  AlignPlan planAlignToMarkers(const std::string &bin_name)
  {
    const DetectionSnapshot snapshot = currentDetectionSnapshot();
    if (snapshot.missing_count > 0)
    {
      std::ostringstream message;
      message << "Need fresh detections for " << markerRequirementText() << " before moving.";
      for (const auto &detail : snapshot.details)
      {
        message << "\n  " << detail;
      }
      throw std::runtime_error(message.str());
    }
    std::vector<int> marker_ids;
    marker_ids.reserve(snapshot.visible.size());
    for (const auto &marker : snapshot.visible)
    {
      marker_ids.push_back(marker.id);
    }

    const std::string parent_frame = snapshot.frame ? snapshot.frame->frame_id : parent_frame_;
    const auto teach_markers = markersInTeachFrame(snapshot.visible, parent_frame);
    Solution solution = computeCentroidSolutionFromMarkers(bin_name, marker_ids, teach_markers.first, teach_markers.second);
    Solution target_solution = computeCentroidTargetFromMarkers(marker_ids, teach_markers.first, teach_markers.second);
    latest_target_solution_ = target_solution;
    publishSolution(target_solution);

    if (!currentTcp())
    {
      throw std::runtime_error("No live TCP pose yet on dobot_msgs_v4/msg/ToolVectorActual.");
    }
    if (!movj_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
      throw std::runtime_error("Motion service is not ready: " + motion_service_root_ + "/MovJ");
    }

    const Pose7 parent_to_target_mm{{
      target_solution.origin[0] * 1000.0,
      target_solution.origin[1] * 1000.0,
      target_solution.origin[2] * 1000.0,
      target_solution.quaternion[0],
      target_solution.quaternion[1],
      target_solution.quaternion[2],
      target_solution.quaternion[3]}};
    const Pose7 base_to_parent_mm = lookupBaseToFramePoseMm(target_solution.parent_frame);
    const Pose7 target_pose = transformPoseCompose(base_to_parent_mm, parent_to_target_mm);
    const Pose7 gripper_to_camera_pose = lookupPoseMm(gripper_frame_, camera_frame_);
    const double span_mm = markerSpanMm(solution);
    const auto goal = buildAlignGoalPose(
      target_pose,
      gripper_to_camera_pose,
      std::max(1.0, align_distance_mm_),
      align_min_base_z_mm_);
    if (!goal)
    {
      throw std::runtime_error(
        "Cannot build a " + formatDouble(align_distance_mm_, 0) +
        " mm bin_teach align pose from current geometry.");
    }
    publishAlignGoal(*goal);
    return AlignPlan{solution, target_solution, *goal, span_mm};
  }

  template<typename CallbackT>
  bool sendMovJPose(const GoalPose &goal_pose, CallbackT &&callback)
  {
    if (!movj_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
      RCLCPP_WARN(get_logger(), "Motion service not available: %s/MovJ", motion_service_root_.c_str());
      return false;
    }
    const int speed = std::max(1, std::min(100, align_pose_speed_percent_));
    auto request = std::make_shared<MovJSrv::Request>();
    request->mode = false;
    request->a = static_cast<float>(goal_pose.x);
    request->b = static_cast<float>(goal_pose.y);
    request->c = static_cast<float>(goal_pose.z);
    request->d = static_cast<float>(goal_pose.rx);
    request->e = static_cast<float>(goal_pose.ry);
    request->f = static_cast<float>(goal_pose.rz);
    request->param_value = {"v=" + std::to_string(speed) + ",a=" + std::to_string(speed)};
    movj_client_->async_send_request(request, std::forward<CallbackT>(callback));
    return true;
  }

  static int clampPercent(int value)
  {
    return std::max(1, std::min(100, value));
  }

  template<typename CallbackT>
  bool sendSpeedFactor(int ratio, CallbackT &&callback)
  {
    if (!speed_factor_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
      RCLCPP_WARN(get_logger(), "Motion service not available: %s/SpeedFactor", motion_service_root_.c_str());
      return false;
    }
    auto request = std::make_shared<SpeedFactorSrv::Request>();
    request->ratio = clampPercent(ratio);
    speed_factor_client_->async_send_request(request, std::forward<CallbackT>(callback));
    return true;
  }

  template<typename CallbackT>
  bool sendRelativeUpMove(CallbackT &&callback)
  {
    if (!relm_user_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
      RCLCPP_WARN(get_logger(), "Motion service not available: %s/RelMovLUser", motion_service_root_.c_str());
      return false;
    }
    auto request = std::make_shared<RelMovLUserSrv::Request>();
    request->a = 0.0F;
    request->b = 0.0F;
    request->c = static_cast<float>(std::max(1.0, align_up_max_distance_mm_));
    request->d = 0.0F;
    request->e = 0.0F;
    request->f = 0.0F;
    request->param_value = {"user=" + std::to_string(align_up_user_index_)};
    relm_user_client_->async_send_request(request, std::forward<CallbackT>(callback));
    return true;
  }

  template<typename CallbackT>
  bool sendStopMotion(CallbackT &&callback)
  {
    if (!stop_client_->wait_for_service(std::chrono::milliseconds(200)))
    {
      RCLCPP_WARN(get_logger(), "Motion service not available: %s/Stop", motion_service_root_.c_str());
      return false;
    }
    auto request = std::make_shared<StopSrv::Request>();
    stop_client_->async_send_request(request, std::forward<CallbackT>(callback));
    return true;
  }

  static double angleErrorDeg(double current_deg, double target_deg)
  {
    const double wrapped = std::fmod(current_deg - target_deg + 180.0, 360.0);
    const double normalized = wrapped < 0.0 ? wrapped + 360.0 : wrapped;
    return std::fabs(normalized - 180.0);
  }

  bool goalReached(const std::optional<TcpPose> &current_tcp, const std::optional<GoalPose> &target_pose) const
  {
    if (!current_tcp || !target_pose)
    {
      return false;
    }
    if (std::fabs((*current_tcp)[0] - target_pose->x) > align_goal_pos_tol_mm_) { return false; }
    if (std::fabs((*current_tcp)[1] - target_pose->y) > align_goal_pos_tol_mm_) { return false; }
    if (std::fabs((*current_tcp)[2] - target_pose->z) > align_goal_pos_tol_mm_) { return false; }
    if (angleErrorDeg((*current_tcp)[3], target_pose->rx) > align_goal_rot_tol_deg_) { return false; }
    if (angleErrorDeg((*current_tcp)[4], target_pose->ry) > align_goal_rot_tol_deg_) { return false; }
    if (angleErrorDeg((*current_tcp)[5], target_pose->rz) > align_goal_rot_tol_deg_) { return false; }
    return true;
  }

  std::pair<bool, std::vector<std::string>> markersVisibleRecent(
    const std::optional<std::vector<int>> &marker_ids = std::nullopt,
    const std::optional<double> &max_age_sec = std::nullopt) const
  {
    const DetectionSnapshot snapshot = currentDetectionSnapshot(marker_ids, max_age_sec);
    return {snapshot.missing_count == 0 && static_cast<int>(snapshot.visible.size()) >= required_marker_count_, snapshot.details};
  }

  void publishAlignGoal(const GoalPose &goal_pose)
  {
    TransformStampedMsg msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = base_frame_;
    msg.child_frame_id = "bin_teach_align_goal";
    msg.transform.translation.x = goal_pose.x / 1000.0;
    msg.transform.translation.y = goal_pose.y / 1000.0;
    msg.transform.translation.z = goal_pose.z / 1000.0;
    const Quat q = rpyDegToQuaternion(goal_pose.rx, goal_pose.ry, goal_pose.rz);
    msg.transform.rotation.x = q[0];
    msg.transform.rotation.y = q[1];
    msg.transform.rotation.z = q[2];
    msg.transform.rotation.w = q[3];
    latest_align_goal_ = goal_pose;
    tf_broadcaster_->sendTransform(msg);
  }

  QImage latestVisualizationQImage(const std::vector<int> &marker_ids)
  {
    constexpr double kImageFreshMaxAgeSec = 3.0;
    const bool overlay_fresh =
      use_aruco_overlay_ &&
      !latest_overlay_qimage_.isNull() &&
      latest_overlay_received_monotonic_.has_value() &&
      secondsSince(*latest_overlay_received_monotonic_) <= kImageFreshMaxAgeSec;
    const bool camera_fresh =
      !latest_camera_qimage_.isNull() &&
      latest_camera_frame_.has_value() &&
      secondsSince(latest_camera_frame_->received_monotonic) <= kImageFreshMaxAgeSec;
    const bool using_overlay = overlay_fresh;
    const QImage base_image = using_overlay
      ? latest_overlay_qimage_
      : (camera_fresh ? latest_camera_qimage_ : QImage());
    if (base_image.isNull())
    {
      return QImage();
    }
    QImage canvas = base_image.convertToFormat(QImage::Format_RGB32);
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::Antialiasing, true);
    drawTeachVisualization(painter, canvas, marker_ids, using_overlay);
    painter.end();
    return canvas;
  }

private:
  void detectionsCallback(const MarkerDetectionsMsg &msg)
  {
    std::vector<MarkerData> markers;
    std::map<int, int> id_counts;
    const size_t pose_count = msg.poses.size();
    const size_t pixel_count = msg.pixel_centers.size();
    const size_t corner_count = msg.pixel_corners.size();
    for (size_t index = 0; index < msg.ids.size(); ++index)
    {
      if (index >= pose_count)
      {
        continue;
      }
      const int marker_id = static_cast<int>(msg.ids[index]);
      id_counts[marker_id] += 1;
      const auto &pose = msg.poses[index];
      MarkerData marker;
      marker.detection_index = static_cast<int>(index);
      marker.id = marker_id;
      marker.same_id_index = id_counts[marker_id];
      marker.position_m = Vec3{{
        static_cast<double>(pose.position.x),
        static_cast<double>(pose.position.y),
        static_cast<double>(pose.position.z)}};
      marker.orientation = Quat{{
        static_cast<double>(pose.orientation.x),
        static_cast<double>(pose.orientation.y),
        static_cast<double>(pose.orientation.z),
        static_cast<double>(pose.orientation.w)}};
      if (index < pixel_count)
      {
        const auto &center = msg.pixel_centers[index];
        marker.pixel = Vec2{{static_cast<double>(center.x), static_cast<double>(center.y)}};
      }
      if (index < corner_count)
      {
        const auto &points = msg.pixel_corners[index].points;
        if (points.size() >= 4)
        {
          for (size_t i = 0; i < 4; ++i)
          {
            marker.corners.push_back(Vec2{{static_cast<double>(points[i].x), static_cast<double>(points[i].y)}});
          }
        }
      }
      markers.push_back(marker);
    }

    DetectionFrame frame;
    frame.stamp = rclcpp::Time(msg.header.stamp);
    frame.received_monotonic = steadyNow();
    frame.frame_id = msg.header.frame_id;
    frame.image_width = static_cast<int>(msg.image_width);
    frame.image_height = static_cast<int>(msg.image_height);
    frame.markers = std::move(markers);
    latest_detection_frame_ = frame;
    publishTargetFromDetectionFrame(frame);
  }

  void colorCallback(const ImageMsg &msg)
  {
    const QImage image = imageMsgToQImage(msg, color_topic_);
    if (!image.isNull())
    {
      latest_camera_qimage_ = image;
      latest_camera_frame_ = CameraFrameInfo{
        rclcpp::Time(msg.header.stamp),
        steadyNow(),
        msg.header.frame_id,
        static_cast<int>(msg.width),
        static_cast<int>(msg.height)};
    }
  }

  void overlayCallback(const ImageMsg &msg)
  {
    const QImage image = imageMsgToQImage(msg, overlay_topic_);
    if (!image.isNull())
    {
      latest_overlay_qimage_ = image;
      latest_overlay_received_monotonic_ = steadyNow();
    }
  }

  void tcpCallback(const ToolVectorActualMsg &msg)
  {
    latest_tcp_ = TcpPose{{
      static_cast<double>(msg.x),
      static_cast<double>(msg.y),
      static_cast<double>(msg.z),
      static_cast<double>(msg.rx),
      static_cast<double>(msg.ry),
      static_cast<double>(msg.rz)}};
  }

  QImage imageMsgToQImage(const ImageMsg &msg, const std::string &source_name)
  {
    if (msg.width <= 0 || msg.height <= 0)
    {
      return QImage();
    }
    const size_t expected_size = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
    if (msg.data.size() < expected_size)
    {
      RCLCPP_WARN(get_logger(), "Received invalid %s image: data buffer too small.", source_name.c_str());
      return QImage();
    }
    std::string encoding = msg.encoding;
    std::transform(encoding.begin(), encoding.end(), encoding.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (encoding == "rgb8" || encoding == "8uc3")
    {
      QImage image(msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height), static_cast<int>(msg.step), QImage::Format_RGB888);
      return image.copy();
    }
    if (encoding == "bgr8")
    {
      QImage image(msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height), static_cast<int>(msg.step), QImage::Format_RGB888);
      return image.rgbSwapped().copy();
    }
    if (encoding == "mono8" || encoding == "8uc1")
    {
      QImage image(msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height), static_cast<int>(msg.step), QImage::Format_Grayscale8);
      return image.copy();
    }

    const std::string key = source_name + "|" + encoding;
    if (unsupported_image_encodings_.count(key) == 0)
    {
      unsupported_image_encodings_.insert(key);
      RCLCPP_WARN(
        get_logger(),
        "Unsupported %s encoding '%s'. Expected bgr8/rgb8/mono8.",
        source_name.c_str(),
        msg.encoding.c_str());
    }
    return QImage();
  }

  static int textWidth(const QFontMetrics &metrics, const QString &text)
  {
#if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
    return metrics.horizontalAdvance(text);
#else
    return metrics.width(text);
#endif
  }

  void drawStatusPanel(QPainter &painter, const QStringList &lines, int margin, const QFont &font)
  {
    if (lines.empty())
    {
      return;
    }
    painter.setFont(font);
    const QFontMetrics metrics(font);
    const int line_height = metrics.height() + 4;
    int box_width = 0;
    for (const auto &line : lines)
    {
      box_width = std::max(box_width, textWidth(metrics, line));
    }
    box_width += 22;
    const int box_height = (line_height * lines.size()) + 16;
    const QRect panel_rect(margin, margin, box_width, box_height);
    painter.fillRect(panel_rect, QColor(0, 0, 0, 150));
    painter.setPen(QPen(QColor(220, 230, 235), 1));
    painter.drawRect(panel_rect);
    int text_y = margin + 12 + metrics.ascent();
    for (const auto &line : lines)
    {
      painter.drawText(margin + 11, text_y, line);
      text_y += line_height;
    }
  }

  void drawTeachVisualization(QPainter &painter, const QImage &image, const std::vector<int> &marker_ids, bool using_overlay)
  {
    const int width = image.width();
    const int height = image.height();
    if (width <= 0 || height <= 0)
    {
      return;
    }
    const int min_side = std::max(1, std::min(width, height));
    const int margin = std::max(10, static_cast<int>(min_side * 0.018));
    const int line_width = std::max(2, static_cast<int>(min_side * 0.006));
    QFont font;
    font.setPointSize(std::max(10, std::min(20, height / 42)));

    const auto frame = latest_detection_frame_;
    std::optional<double> detection_age;
    bool detection_is_current = false;
    std::vector<MarkerData> display_markers;
    std::optional<double> depth_view_offset_y;
    int visible_count = 0;
    if (frame)
    {
      detection_age = secondsSince(frame->received_monotonic);
      detection_is_current = *detection_age <= std::max(0.1, align_visible_max_age_sec_);
      if (detection_is_current)
      {
        const int source_width = std::max(1, frame->image_width);
        const int source_height = std::max(1, frame->image_height);
        const bool stacked_overlay = using_overlay && height >= (source_height * 2 - 2);
        const double view_height = stacked_overlay ? (static_cast<double>(height) / 2.0) : static_cast<double>(height);
        const double scale_x = static_cast<double>(width) / static_cast<double>(source_width);
        const double scale_y = view_height / static_cast<double>(source_height);
        if (stacked_overlay)
        {
          depth_view_offset_y = view_height;
        }
        const std::set<int> watched_ids(marker_ids.begin(), marker_ids.end());
        for (const auto &marker : frame->markers)
        {
          if (watched_ids.count(marker.id) == 0 || !marker.pixel)
          {
            continue;
          }
          MarkerData display_marker = marker;
          display_marker.pixel = Vec2{{(*marker.pixel)[0] * scale_x, (*marker.pixel)[1] * scale_y}};
          if (marker.corners.size() >= 4)
          {
            display_marker.corners.clear();
            for (size_t i = 0; i < 4; ++i)
            {
              display_marker.corners.push_back(Vec2{{marker.corners[i][0] * scale_x, marker.corners[i][1] * scale_y}});
            }
          }
          display_markers.push_back(display_marker);
        }
        visible_count = static_cast<int>(frame->markers.size());
      }
    }

    if (detection_is_current)
    {
      const auto corner_dot_data = cornerDotsFromMarkers(display_markers);
      if (corner_dot_data)
      {
        std::vector<QPointF> corner_dots;
        for (const auto &dot : *corner_dot_data)
        {
          corner_dots.emplace_back(dot.x, dot.y);
        }
        const double dot_radius = std::max(6.0, min_side * 0.010);
        painter.setBrush(QBrush(QColor(255, 80, 220)));
        painter.setPen(QPen(QColor(20, 20, 20), std::max(2, line_width)));
        for (const auto &point : corner_dots)
        {
          painter.drawEllipse(point, dot_radius, dot_radius);
        }
        if (depth_view_offset_y)
        {
          for (const auto &point : corner_dots)
          {
            painter.drawEllipse(QPointF(point.x(), point.y() + *depth_view_offset_y), dot_radius, dot_radius);
          }
        }
        painter.setBrush(QBrush(Qt::NoBrush));
      }
    }

    const int required_count = required_marker_count_;
    const int watched_seen = static_cast<int>(display_markers.size());
    QStringList status_lines;
    if (!latest_camera_frame_)
    {
      status_lines << QString::fromStdString("Waiting for " + color_topic_);
    }
    else
    {
      const double camera_age = secondsSince(latest_camera_frame_->received_monotonic);
      status_lines << QString::fromStdString(color_topic_ + " | " + std::to_string(width) + "x" + std::to_string(height) + " | age " + formatDouble(camera_age, 2) + "s");
    }
    if (!frame)
    {
      status_lines << QString::fromStdString("No " + detections_topic_ + " frame yet");
    }
    else if (!detection_age)
    {
      status_lines << QString::fromStdString("Waiting for " + detections_topic_);
    }
    else if (detection_is_current)
    {
      status_lines << QString::fromStdString(
        "ArUco markers: " + std::to_string(visible_count) +
        " detected | bin_teach " + std::to_string(std::min(watched_seen, required_count)) +
        "/" + std::to_string(required_count));
    }
    else
    {
      status_lines << QString::fromStdString("ArUco detections stale: " + formatDouble(*detection_age, 2) + "s");
    }

    if (watched_seen < required_count && detection_is_current)
    {
      const int needed = required_count - watched_seen;
      status_lines << QString::fromStdString(
        "Need " + std::to_string(needed) + " more bin marker" + (needed == 1 ? "" : "s") +
        " with allowed IDs " + formatMarkerIds(marker_ids, "or"));
    }
    drawStatusPanel(painter, status_lines, margin, font);
  }

  std::string marker_prefix_;
  std::string parent_frame_;
  std::string target_frame_;
  std::string base_frame_;
  std::string gripper_frame_;
  std::string camera_frame_;
  bool use_platform_calibration_{true};
  bool publish_static_platform_tf_{true};
  bool auto_discover_platform_calibration_{true};
  std::string platform_calibration_dir_;
  std::string platform_calibration_file_;
  bool platform_calibration_loaded_{false};
  std::string platform_name_;
  std::string platform_parent_frame_;
  std::string platform_frame_;
  Pose7 platform_pose_m_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  std::string color_topic_;
  std::filesystem::path output_dir_;
  std::string default_bin_name_;
  std::string bin_frame_prefix_;
  std::string overlay_topic_;
  bool use_aruco_overlay_{false};
  std::string detections_topic_;
  std::string motion_service_root_;
  double align_distance_mm_{300.0};
  int align_pose_speed_percent_{100};
  double align_visible_max_age_sec_{0.75};
  double align_initial_timeout_sec_{30.0};
  double align_min_base_z_mm_{200.0};
  double align_goal_pos_tol_mm_{8.0};
  double align_goal_rot_tol_deg_{3.0};
  double align_up_max_distance_mm_{400.0};
  int align_up_speed_factor_percent_{5};
  double align_up_timeout_sec_{60.0};
  int align_up_user_index_{0};
  int align_restore_speed_factor_percent_{100};
  std::vector<int> allowed_marker_ids_;
  int required_marker_count_{kBinTeachRequiredMarkerCount};

  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_platform_tf_broadcaster_;
  std::optional<Solution> latest_solution_;
  std::optional<Solution> latest_target_solution_;
  std::optional<GoalPose> latest_align_goal_;
  std::optional<TcpPose> latest_tcp_;
  std::optional<DetectionFrame> latest_detection_frame_;
  QImage latest_camera_qimage_;
  std::optional<CameraFrameInfo> latest_camera_frame_;
  QImage latest_overlay_qimage_;
  std::optional<std::chrono::steady_clock::time_point> latest_overlay_received_monotonic_;
  std::set<std::string> unsupported_image_encodings_;

  rclcpp::Client<MovJSrv>::SharedPtr movj_client_;
  rclcpp::Client<RelMovLUserSrv>::SharedPtr relm_user_client_;
  rclcpp::Client<SpeedFactorSrv>::SharedPtr speed_factor_client_;
  rclcpp::Client<StopSrv>::SharedPtr stop_client_;
  rclcpp::Subscription<ToolVectorActualMsg>::SharedPtr tcp_sub_;
  rclcpp::Subscription<MarkerDetectionsMsg>::SharedPtr detections_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr overlay_sub_;
};

class BinTeachWindow : public QWidget
{
public:
  explicit BinTeachWindow(const std::shared_ptr<BinTeachNode> &node)
  : node_(node)
  {
    align_total_deadline_ = steadyNow();
    status_hold_until_ = steadyNow();
    align_deadline_ = steadyNow();

    setWindowTitle("bin_teach");
    setMinimumSize(1180, 720);

    bin_name_ = new QLineEdit(QString::fromStdString(node_->defaultBinName()), this);

    auto *form = new QFormLayout();
    form->addRow("Bin name", bin_name_);

    marker_ids_label_ = new QLabel(QString::fromStdString(formatMarkerIds(ids(), "or")), this);
    parent_label_ = new QLabel(QString::fromStdString(node_->parentFrame()), this);
    target_label_ = new QLabel(QString::fromStdString(node_->targetFrame()), this);
    output_label_ = new QLabel(QString::fromStdString(node_->outputDir().string()), this);
    output_label_->setWordWrap(true);
    form->addRow("Allowed marker IDs", marker_ids_label_);
    form->addRow(node_->usePlatformCalibration() ? "Platform frame" : "Marker parent frame", parent_label_);
    form->addRow("Target frame", target_label_);
    if (node_->usePlatformCalibration())
    {
      platform_file_label_ = new QLabel(QString::fromStdString(node_->platformCalibrationFile()), this);
      platform_file_label_->setWordWrap(true);
      form->addRow("Platform YAML", platform_file_label_);
    }
    form->addRow("Output directory", output_label_);

    align_button_ = new QPushButton("Align", this);
    align_button_->setToolTip("Move one full-orientation pose that faces the marker-bounds center target.");
    save_button_ = new QPushButton("Save bin_teach", this);
    save_button_->setEnabled(false);

    auto *button_row = new QHBoxLayout();
    button_row->addWidget(align_button_);
    button_row->addWidget(save_button_);

    status_ = new QPlainTextEdit(this);
    status_->setReadOnly(true);
    status_->setMinimumHeight(190);

    auto *controls_widget = new QWidget(this);
    controls_widget->setMinimumWidth(330);
    controls_widget->setMaximumWidth(390);
    controls_widget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    auto *controls_layout = new QVBoxLayout(controls_widget);
    controls_layout->addLayout(form);
    controls_layout->addLayout(button_row);
    controls_layout->addWidget(status_, 1);

    const std::string overlay_title = node_->useArucoOverlay()
      ? "ArUco Overlay (" + node_->overlayTopic() + ")"
      : "Camera View (" + node_->colorTopic() + ")";
    const std::string overlay_wait = node_->useArucoOverlay()
      ? "Waiting for " + node_->overlayTopic() + " ..."
      : "Waiting for " + node_->colorTopic() + " ...";
    auto *overlay_group = new QGroupBox(QString::fromStdString(overlay_title), this);
    overlay_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto *overlay_layout = new QVBoxLayout(overlay_group);
    overlay_label_ = new QLabel(QString::fromStdString(overlay_wait), this);
    overlay_label_->setAlignment(Qt::AlignCenter);
    overlay_label_->setMinimumSize(760, 540);
    overlay_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    overlay_label_->setStyleSheet("QLabel { background-color: #101010; color: #d0d0d0; border: 1px solid #444; }");
    overlay_layout->addWidget(overlay_label_);

    auto *root_layout = new QHBoxLayout(this);
    root_layout->addWidget(controls_widget, 0);
    root_layout->addWidget(overlay_group, 1);

    connect(align_button_, &QPushButton::clicked, this, [this]() { alignToMarkers(); });
    connect(save_button_, &QPushButton::clicked, this, [this]() { saveBinTeach(); });

    auto *ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, [this]() { spinRos(); });
    ros_timer->start(20);

    auto *overlay_timer = new QTimer(this);
    connect(overlay_timer, &QTimer::timeout, this, [this]() { refreshOverlay(); });
    overlay_timer->start(100);

    auto *check_timer = new QTimer(this);
    connect(check_timer, &QTimer::timeout, this, [this]() { autoCheckMarkers(); });
    check_timer->start(500);

    auto *align_watch_timer = new QTimer(this);
    connect(align_watch_timer, &QTimer::timeout, this, [this]() { runAlignStep(); });
    align_watch_timer->start(50);
  }

private:
  std::vector<int> ids() const
  {
    return node_->allowedMarkerIds();
  }

  void spinRos()
  {
    rclcpp::spin_some(node_);
    node_->republishLatest();
  }

  void refreshOverlay()
  {
    const QImage image = node_->latestVisualizationQImage(ids());
    if (image.isNull())
    {
      const std::string source = node_->useArucoOverlay()
        ? node_->overlayTopic()
        : node_->colorTopic();
      overlay_label_->clear();
      overlay_label_->setText(QString::fromStdString("no camera topics...\nWaiting for " + source + " ..."));
      return;
    }
    QPixmap pixmap = QPixmap::fromImage(image);
    if (pixmap.isNull())
    {
      return;
    }
    const QSize target_size = overlay_label_->size();
    if (target_size.width() > 1 && target_size.height() > 1)
    {
      pixmap = pixmap.scaled(target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }
    overlay_label_->setText("");
    overlay_label_->setPixmap(pixmap);
  }

  void log(const std::string &text, double hold_sec = 5.0)
  {
    const QString current = status_->toPlainText();
    const QString prefix = current.isEmpty() ? QString() : QString("\n");
    status_->setPlainText(current + prefix + QString::fromStdString(text));
    QTextCursor cursor = status_->textCursor();
    cursor.movePosition(QTextCursor::End);
    status_->setTextCursor(cursor);
    status_hold_until_ = std::max(status_hold_until_, steadyNow() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(hold_sec)));
  }

  std::string alignDistanceLabel() const
  {
    return formatDouble(node_->alignDistanceMm(), 0) + " mm";
  }

  void autoCheckMarkers()
  {
    if (align_in_flight_ || steadyNow() < status_hold_until_)
    {
      return;
    }
    checkMarkers();
  }

  void checkMarkers()
  {
    const std::vector<int> marker_ids = ids();
    const std::string requirement = node_->markerRequirementText();
    std::vector<std::string> lines;
    try
    {
      const auto visible_result = node_->markersVisibleRecent(marker_ids);
      if (!visible_result.first)
      {
        lines.push_back("Waiting for " + requirement + " in the current ArUco frame.");
        lines.insert(lines.end(), visible_result.second.begin(), visible_result.second.end());
        save_button_->setEnabled(false);
        status_->setPlainText(QString::fromStdString(joinLines(lines)));
        return;
      }
      const Solution solution = node_->computeSolution(bin_name_->text().toStdString());
      lines.push_back("Detected " + std::to_string(solution.marker_ids.size()) + " bin markers using allowed IDs.");
      lines.push_back("Preview TF: " + solution.parent_frame + " -> " + solution.frame_id);
      lines.push_back(
        "Origin xyz m: " + formatDouble(solution.origin[0], 4) + ", " +
        formatDouble(solution.origin[1], 4) + ", " + formatDouble(solution.origin[2], 4));
      lines.push_back("");
      lines.push_back("The saved transform origin is the center of the configured marker position bounds.");
      save_button_->setEnabled(true);
    }
    catch (const tf2::TransformException &ex)
    {
      lines.push_back("Waiting for marker transforms.");
      lines.push_back(ex.what());
      save_button_->setEnabled(false);
    }
    catch (const std::exception &ex)
    {
      lines.push_back("Cannot fit bin frame.");
      lines.push_back(ex.what());
      save_button_->setEnabled(false);
    }
    status_->setPlainText(QString::fromStdString(joinLines(lines)));
  }

  static std::string joinLines(const std::vector<std::string> &lines)
  {
    std::ostringstream stream;
    for (size_t i = 0; i < lines.size(); ++i)
    {
      if (i > 0) { stream << "\n"; }
      stream << lines[i];
    }
    return stream.str();
  }

  void alignToMarkers()
  {
    if (align_in_flight_)
    {
      log("[align] Align command is already in progress.", 4.0);
      return;
    }
    const std::vector<int> marker_ids = ids();
    const std::string requirement = node_->markerRequirementText();
    const DetectionSnapshot snapshot = node_->markerVisibilitySnapshotFull(marker_ids);
    if (static_cast<int>(snapshot.visible.size()) < node_->requiredMarkerCount() || snapshot.missing_count > 0)
    {
      status_hold_until_ = steadyNow() + std::chrono::seconds(8);
      status_->setPlainText(QString::fromStdString(
        "[align] Drag the robot until the camera sees " + requirement + ", then press Align.\n" +
        joinLines(snapshot.details)));
      return;
    }

    align_in_flight_ = true;
    align_button_->setEnabled(false);
    status_hold_until_ = steadyNow() + std::chrono::seconds(10);
    align_state_ = "planning_full_orientation_pose";
    align_goal_.reset();
    align_marker_ids_ = marker_ids;
    align_deadline_ = steadyNow();
    align_total_deadline_ = steadyNow() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(std::max(
      20.0,
      node_->alignInitialTimeoutSec() + node_->alignUpTimeoutSec() + 10.0)));
    save_button_->setEnabled(false);
    status_->setPlainText(QString::fromStdString(
      "[align] Required bin markers visible. Aligning full camera orientation, then moving up slowly until the camera sees " + requirement + " again."));
    planAndSendFullOrientationPose();
  }

  void planAndSendFullOrientationPose()
  {
    AlignPlan plan;
    try
    {
      plan = node_->planAlignToMarkers(bin_name_->text().toStdString());
    }
    catch (const std::exception &ex)
    {
      finishAlign();
      log("[align] Cannot compute the marker-bounds-center " + alignDistanceLabel() + " full-orientation pose: " + ex.what(), 8.0);
      return;
    }

    align_goal_ = plan.goal;
    save_button_->setEnabled(true);
    log(
      "[align] Marker span " + formatDouble(plan.span_mm, 0) + " mm. Target frame " +
      plan.target_solution.parent_frame + " -> " + plan.target_solution.frame_id +
      ". Moving camera to one " + formatDouble(node_->alignDistanceMm(), 0) +
      " mm full-orientation pose on " + node_->cameraFrame() + ".",
      10.0);
    log(
      "[align] MovJ pose -> x=" + formatDouble(plan.goal.x, 1) +
      " y=" + formatDouble(plan.goal.y, 1) +
      " z=" + formatDouble(plan.goal.z, 1) +
      " rx=" + formatDouble(plan.goal.rx, 1) +
      " ry=" + formatDouble(plan.goal.ry, 1) +
      " rz=" + formatDouble(plan.goal.rz, 1),
      10.0);

    const int speed_factor = BinTeachNode::clampPercent(node_->alignPoseSpeedPercent());
    const bool sent = node_->sendSpeedFactor(
      speed_factor,
      [this](rclcpp::Client<SpeedFactorSrv>::SharedFuture future) { onInitialSpeedFactorDone(future); });
    if (!sent)
    {
      finishAlign();
      log("[align] Failed to set initial SpeedFactor to " + std::to_string(speed_factor) + "%.", 8.0);
      return;
    }
    align_state_ = "waiting_initial_speed_factor";
    align_deadline_ = steadyNow() + std::chrono::seconds(3);
    log("[align] Setting initial SpeedFactor to " + std::to_string(speed_factor) + "% before full-orientation motion.", 10.0);
  }

  void onInitialSpeedFactorDone(rclcpp::Client<SpeedFactorSrv>::SharedFuture future)
  {
    if (!align_in_flight_ || align_state_ != "waiting_initial_speed_factor")
    {
      return;
    }
    try
    {
      const auto response = future.get();
      if (static_cast<int>(response->res) != 0)
      {
        finishAlign();
        log("[align] Initial SpeedFactor rejected: res=" + std::to_string(response->res), 8.0);
        return;
      }
    }
    catch (const std::exception &ex)
    {
      finishAlign();
      log("[align] Initial SpeedFactor call failed: " + std::string(ex.what()), 8.0);
      return;
    }

    if (!align_goal_)
    {
      finishAlign();
      log("[align] Failed to dispatch full-orientation motion command.", 8.0);
      return;
    }
    const bool sent = node_->sendMovJPose(
      *align_goal_,
      [this](rclcpp::Client<MovJSrv>::SharedFuture future) { onAlignPoseCommandDone(future); });
    if (!sent)
    {
      finishAlign();
      log("[align] Failed to dispatch full-orientation motion command.", 8.0);
      return;
    }
    align_state_ = "waiting_pose_command";
    align_deadline_ = steadyNow() + std::chrono::seconds(3);
  }

  void onAlignPoseCommandDone(rclcpp::Client<MovJSrv>::SharedFuture future)
  {
    try
    {
      const auto response = future.get();
      if (static_cast<int>(response->res) != 0)
      {
        finishAlign();
        log(
          "[align] " + alignDistanceLabel() + " pose rejected: res=" + std::to_string(response->res) +
          ", reply=" + response->robot_return,
          8.0);
        return;
      }
      align_state_ = "waiting_pose_reached";
      align_deadline_ = steadyNow() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(node_->alignInitialTimeoutSec()));
      log("[align] " + alignDistanceLabel() + " full-orientation pose accepted. Waiting for robot to reach it...", 10.0);
    }
    catch (const std::exception &ex)
    {
      finishAlign();
      log("[align] " + alignDistanceLabel() + " full-orientation command failed: " + ex.what(), 8.0);
    }
  }

  void runAlignStep()
  {
    if (!align_in_flight_)
    {
      return;
    }
    const auto now = steadyNow();
    if (now > align_total_deadline_)
    {
      if (align_state_ == "waiting_up_command" || align_state_ == "waiting_up_visibility")
      {
        requestStopForUpMotion("[align] Alignment timed out during upward motion. Sending Stop.", true);
        return;
      }
      if (align_state_ == "waiting_stop_command")
      {
        finishAlign();
        log("[align] Alignment timed out while waiting for Stop response.", 8.0);
        return;
      }
      finishAlign();
      log("[align] Marker-bounds-center full-orientation alignment timed out.", 8.0);
      return;
    }

    if (align_state_ == "waiting_initial_speed_factor")
    {
      if (now > align_deadline_)
      {
        finishAlign();
        log("[align] Timed out setting initial SpeedFactor.", 8.0);
      }
      return;
    }
    if (align_state_ == "waiting_pose_command")
    {
      if (now > align_deadline_)
      {
        finishAlign();
        log("[align] Timed out waiting for full-orientation motion command response.", 8.0);
      }
      return;
    }
    if (align_state_ == "waiting_pose_reached")
    {
      if (node_->goalReached(node_->currentTcp(), align_goal_))
      {
        log("[align] " + alignDistanceLabel() + " pose reached.", 10.0);
        startSlowUpMotion();
        return;
      }
      if (now > align_deadline_)
      {
        finishAlign();
        log("[align] Timed out waiting for the " + alignDistanceLabel() + " full-orientation pose.", 8.0);
      }
      return;
    }
    if (align_state_ == "waiting_up_speed_factor")
    {
      if (now > align_deadline_)
      {
        finishAlign();
        log("[align] Timed out setting the upward SpeedFactor.", 8.0);
      }
      return;
    }
    if (align_state_ == "waiting_up_command")
    {
      if (now > align_deadline_)
      {
        requestStopForUpMotion("[align] Timed out waiting for the upward motion command. Sending Stop.", true);
      }
      return;
    }
    if (align_state_ == "waiting_up_visibility")
    {
      const DetectionSnapshot snapshot = node_->markerVisibilitySnapshotFull(align_marker_ids_);
      const int marker_count = node_->requiredMarkerCount();
      if (static_cast<int>(snapshot.visible.size()) >= marker_count && snapshot.missing_count == 0)
      {
        requestStopForUpMotion("[align] All " + std::to_string(marker_count) + " markers visible during upward motion. Sending Stop.", true);
        try
        {
          node_->computeSolution(bin_name_->text().toStdString());
          save_button_->setEnabled(true);
          log(
            "[align] Stop sent. Camera full orientation is aligned and all " + std::to_string(marker_count) +
            " markers are visible. Ready to save bin_teach.",
            10.0);
        }
        catch (const std::exception &ex)
        {
          log("[align] Markers visible, but bin frame could not be fit: " + std::string(ex.what()), 8.0);
        }
        return;
      }
      if (now > align_deadline_)
      {
        requestStopForUpMotion(
          "[align] Timed out before all " + std::to_string(marker_count) +
          " markers became visible during upward motion. Sending Stop.",
          true);
      }
      return;
    }
    if (align_state_ == "waiting_stop_command")
    {
      if (now > align_deadline_)
      {
        finishAlign();
        log("[align] Timed out waiting for Stop response.", 8.0);
      }
      return;
    }
    if (align_state_ == "waiting_restore_speed_factor")
    {
      if (now > align_deadline_)
      {
        finishAlign();
        log("[align] Timed out restoring SpeedFactor after upward stop.", 8.0);
      }
      return;
    }
  }

  void startSlowUpMotion()
  {
    const int speed_factor = BinTeachNode::clampPercent(node_->alignUpSpeedFactorPercent());
    const int marker_count = node_->requiredMarkerCount();
    const bool sent = node_->sendSpeedFactor(
      speed_factor,
      [this](rclcpp::Client<SpeedFactorSrv>::SharedFuture future) { onUpSpeedFactorDone(future); });
    if (!sent)
    {
      finishAlign();
      log("[align] Cannot set upward SpeedFactor to " + std::to_string(speed_factor) + "%.", 8.0);
      return;
    }
    align_state_ = "waiting_up_speed_factor";
    align_deadline_ = steadyNow() + std::chrono::seconds(3);
    log(
      "[align] Setting upward SpeedFactor to " + std::to_string(speed_factor) +
      "%, then moving +Z until all " + std::to_string(marker_count) + " markers are visible.",
      10.0);
  }

  void onUpSpeedFactorDone(rclcpp::Client<SpeedFactorSrv>::SharedFuture future)
  {
    if (!align_in_flight_ || align_state_ != "waiting_up_speed_factor")
    {
      return;
    }
    try
    {
      const auto response = future.get();
      if (static_cast<int>(response->res) != 0)
      {
        finishAlign();
        log("[align] Upward SpeedFactor rejected: res=" + std::to_string(response->res), 8.0);
        return;
      }
    }
    catch (const std::exception &ex)
    {
      finishAlign();
      log("[align] Upward SpeedFactor call failed: " + std::string(ex.what()), 8.0);
      return;
    }

    const bool sent = node_->sendRelativeUpMove(
      [this](rclcpp::Client<RelMovLUserSrv>::SharedFuture future) { onUpMotionCommandDone(future); });
    if (!sent)
    {
      finishAlign();
      log("[align] Failed to dispatch upward RelMovLUser command.", 8.0);
      return;
    }
    align_state_ = "waiting_up_command";
    align_deadline_ = steadyNow() + std::chrono::seconds(3);
  }

  void onUpMotionCommandDone(rclcpp::Client<RelMovLUserSrv>::SharedFuture future)
  {
    if (!align_in_flight_ || align_state_ != "waiting_up_command")
    {
      return;
    }
    try
    {
      const auto response = future.get();
      if (static_cast<int>(response->res) != 0)
      {
        finishAlign();
        log("[align] Upward RelMovLUser command rejected: res=" + std::to_string(response->res), 8.0);
        return;
      }
    }
    catch (const std::exception &ex)
    {
      finishAlign();
      log("[align] Upward RelMovLUser command failed: " + std::string(ex.what()), 8.0);
      return;
    }

    align_state_ = "waiting_up_visibility";
    align_deadline_ = steadyNow() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(std::max(1.0, node_->alignUpTimeoutSec())));
    const int marker_count = node_->requiredMarkerCount();
    log(
      "[align] Moving up +Z up to " + formatDouble(node_->alignUpMaxDistanceMm(), 0) +
      " mm. Watching detections and will Stop as soon as all " + std::to_string(marker_count) + " markers are visible.",
      10.0);
  }

  bool requestStopForUpMotion(const std::string &message, bool finish_if_stop_unavailable)
  {
    const bool sent = node_->sendStopMotion(
      [this](rclcpp::Client<StopSrv>::SharedFuture future) { onUpStopDone(future); });
    if (!sent)
    {
      log("[align] Stop service is not ready; upward motion can only end at its commanded distance.", 8.0);
      if (finish_if_stop_unavailable)
      {
        finishAlign();
      }
      return false;
    }
    align_state_ = "waiting_stop_command";
    align_deadline_ = steadyNow() + std::chrono::seconds(3);
    log(message, 10.0);
    return true;
  }

  void onUpStopDone(rclcpp::Client<StopSrv>::SharedFuture future)
  {
    if (!align_in_flight_ || align_state_ != "waiting_stop_command")
    {
      return;
    }
    try
    {
      const auto response = future.get();
      if (static_cast<int>(response->res) != 0)
      {
        finishAlign();
        log("[align] Stop rejected after upward motion: res=" + std::to_string(response->res), 8.0);
        return;
      }
    }
    catch (const std::exception &ex)
    {
      finishAlign();
      log("[align] Stop call failed after upward motion: " + std::string(ex.what()), 8.0);
      return;
    }

    const int restore = node_->alignRestoreSpeedFactorPercent();
    if (restore <= 0)
    {
      finishAlign();
      log("[align] Stop accepted after upward motion.", 8.0);
      return;
    }

    const bool sent = node_->sendSpeedFactor(
      restore,
      [this](rclcpp::Client<SpeedFactorSrv>::SharedFuture future) { onRestoreSpeedFactorDone(future); });
    if (!sent)
    {
      finishAlign();
      log("[align] Stop accepted. Could not restore SpeedFactor.", 8.0);
      return;
    }
    align_state_ = "waiting_restore_speed_factor";
    align_deadline_ = steadyNow() + std::chrono::seconds(3);
  }

  void onRestoreSpeedFactorDone(rclcpp::Client<SpeedFactorSrv>::SharedFuture future)
  {
    if (!align_in_flight_ || align_state_ != "waiting_restore_speed_factor")
    {
      return;
    }
    try
    {
      const auto response = future.get();
      if (static_cast<int>(response->res) != 0)
      {
        log("[align] SpeedFactor restore rejected: res=" + std::to_string(response->res), 8.0);
      }
      else
      {
        log("[align] Stop accepted. SpeedFactor restored to " + std::to_string(node_->alignRestoreSpeedFactorPercent()) + "%.", 8.0);
      }
    }
    catch (const std::exception &ex)
    {
      log("[align] SpeedFactor restore failed: " + std::string(ex.what()), 8.0);
    }
    finishAlign();
  }

  void finishAlign()
  {
    align_in_flight_ = false;
    align_button_->setEnabled(true);
    align_state_ = "idle";
    align_goal_.reset();
    align_marker_ids_.reset();
    align_deadline_ = steadyNow();
    align_total_deadline_ = steadyNow();
  }

  void saveBinTeach()
  {
    try
    {
      const Solution solution = node_->computeSolution(bin_name_->text().toStdString());
      const std::filesystem::path path = node_->saveSolution(solution);
      status_->setPlainText(status_->toPlainText() + QString::fromStdString("\n\nSaved bin_teach:\n" + path.string()));
      status_hold_until_ = steadyNow() + std::chrono::seconds(5);
      QMessageBox::information(this, "bin_teach", QString::fromStdString("Saved:\n" + path.string()));
    }
    catch (const std::exception &ex)
    {
      QMessageBox::warning(this, "bin_teach", QString::fromStdString(ex.what()));
    }
  }

  std::shared_ptr<BinTeachNode> node_;
  bool align_in_flight_{false};
  std::string align_state_{"idle"};
  std::optional<GoalPose> align_goal_;
  std::optional<std::vector<int>> align_marker_ids_;
  std::chrono::steady_clock::time_point align_deadline_;
  std::chrono::steady_clock::time_point align_total_deadline_;
  std::chrono::steady_clock::time_point status_hold_until_;

  QLineEdit *bin_name_{nullptr};
  QLabel *marker_ids_label_{nullptr};
  QLabel *parent_label_{nullptr};
  QLabel *target_label_{nullptr};
  QLabel *platform_file_label_{nullptr};
  QLabel *output_label_{nullptr};
  QPushButton *align_button_{nullptr};
  QPushButton *save_button_{nullptr};
  QPlainTextEdit *status_{nullptr};
  QLabel *overlay_label_{nullptr};
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<BinTeachNode>();
  BinTeachWindow window(node);
  window.show();
  const int exit_code = app.exec();
  node.reset();
  rclcpp::shutdown();
  return exit_code;
}
