#include <array>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <aruco_perception/msg/marker_detections.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace camera_calibration
{
namespace
{
constexpr size_t kRequiredMarkerCount = 4;
constexpr int64_t kMinArucoId = 0;
constexpr int64_t kMaxArucoId = 49;  // DICT_5X5_50
constexpr double kAxisMinNorm = 1e-6;

struct BoardMarkerObservation
{
  int64_t id{0};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 4> corners{};
};

std::string formatMarkerIds(const std::vector<int64_t> &marker_ids)
{
  std::ostringstream stream;
  for (size_t i = 0; i < marker_ids.size(); ++i)
  {
    if (i > 0)
    {
      stream << ",";
    }
    stream << marker_ids[i];
  }
  return stream.str();
}

bool validateMarkerIds(const std::vector<int64_t> &marker_ids, std::string &reason)
{
  if (marker_ids.size() != kRequiredMarkerCount)
  {
    reason = "marker_ids must contain exactly 4 ArUco IDs in top-left,top-right,bottom-left,bottom-right order.";
    return false;
  }
  std::unordered_set<int64_t> seen;
  for (const auto id : marker_ids)
  {
    if (id < kMinArucoId || id > kMaxArucoId)
    {
      reason = "marker_ids must be in range 0..49 for DICT_5X5_50.";
      return false;
    }
    if (!seen.insert(id).second)
    {
      reason = "marker_ids must be unique for depth board pose fitting.";
      return false;
    }
  }
  reason.clear();
  return true;
}

bool isFiniteVector(const Eigen::Vector3d &value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point &point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d point32ToEigen(const geometry_msgs::msg::Point32 &point)
{
  return Eigen::Vector3d(
    static_cast<double>(point.x),
    static_cast<double>(point.y),
    static_cast<double>(point.z));
}

double relativeMismatch(double a, double b)
{
  const double denom = std::max(std::fabs(a), std::fabs(b));
  if (denom < kAxisMinNorm)
  {
    return std::numeric_limits<double>::infinity();
  }
  return std::fabs(a - b) / denom;
}
}  // namespace

class CalibrationPerception : public rclcpp::Node
{
public:
  CalibrationPerception()
  : Node("calibration_perception")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    marker_prefix_ = declare_parameter<std::string>("marker_prefix", "aruco_marker");
    output_frame_ = declare_parameter<std::string>("output_frame", "tag_frame");
    parent_frame_ = declare_parameter<std::string>(
      "parent_frame", "robot_camera_color_optical_frame");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/aruco_detections");
    marker_ids_ = declare_parameter<std::vector<int64_t>>(
      "marker_ids", std::vector<int64_t>{1, 2, 3, 4});
    std::string marker_ids_reason;
    if (!validateMarkerIds(marker_ids_, marker_ids_reason))
    {
      throw std::runtime_error(marker_ids_reason);
    }
    max_marker_age_sec_ = declare_parameter<double>("max_marker_age_sec", 1.5);
    max_board_plane_rmse_m_ = declare_parameter<double>("max_board_plane_rmse_m", 0.03);
    max_board_edge_mismatch_ratio_ = declare_parameter<double>(
      "max_board_edge_mismatch_ratio", 0.35);
    min_board_marker_spacing_m_ = declare_parameter<double>("min_board_marker_spacing_m", 0.03);
    (void)declare_parameter<double>("lookup_timeout", 0.05);
    (void)declare_parameter<double>("publish_rate", 20.0);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CalibrationPerception::handleParameterUpdate, this, std::placeholders::_1));

    detections_sub_ = create_subscription<aruco_perception::msg::MarkerDetections>(
      detections_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CalibrationPerception::detectionsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Fitting one depth board pose from %zu markers (%s_* layout TL,TR,BL,BR) "
                "into frame '%s' under parent '%s' (topic=%s, ids=%s, max age %.2fs, "
                "plane RMS <= %.3fm).",
                marker_ids_.size(), marker_prefix_.c_str(), output_frame_.c_str(),
                parent_frame_.c_str(), detections_topic_.c_str(), formatMarkerIds(marker_ids_).c_str(),
                max_marker_age_sec_, max_board_plane_rmse_m_);
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<aruco_perception::msg::MarkerDetections>::SharedPtr detections_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::string marker_prefix_;
  std::string output_frame_;
  std::string parent_frame_;
  std::string detections_topic_;
  std::vector<int64_t> marker_ids_;
  double max_marker_age_sec_{1.5};
  double max_board_plane_rmse_m_{0.03};
  double max_board_edge_mismatch_ratio_{0.35};
  double min_board_marker_spacing_m_{0.03};

  rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &parameter : parameters)
    {
      const auto &name = parameter.get_name();
      if (name != "parent_frame" && name != "max_marker_age_sec" && name != "marker_ids" &&
          name != "max_board_plane_rmse_m" && name != "max_board_edge_mismatch_ratio" &&
          name != "min_board_marker_spacing_m")
      {
        continue;
      }
      if (name == "marker_ids")
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
          result.successful = false;
          result.reason = "marker_ids must be an integer array.";
          return result;
        }
        const std::vector<int64_t> value = parameter.as_integer_array();
        std::string reason;
        if (!validateMarkerIds(value, reason))
        {
          result.successful = false;
          result.reason = reason;
          return result;
        }
        marker_ids_ = value;
        RCLCPP_INFO(
          get_logger(),
          "Calibration perception marker IDs updated: %s -> %s",
          formatMarkerIds(marker_ids_).c_str(), output_frame_.c_str());
        continue;
      }
      if (name == "max_marker_age_sec" || name == "max_board_plane_rmse_m" ||
          name == "max_board_edge_mismatch_ratio" || name == "min_board_marker_spacing_m")
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.successful = false;
          result.reason = name + " must be a number.";
          return result;
        }
        const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                               ? static_cast<double>(parameter.as_int())
                               : parameter.as_double();
        if (!std::isfinite(value) || value < 0.0)
        {
          result.successful = false;
          result.reason = name + " must be finite and non-negative.";
          return result;
        }
        if (name == "max_marker_age_sec")
        {
          max_marker_age_sec_ = value;
        }
        else if (name == "max_board_plane_rmse_m")
        {
          max_board_plane_rmse_m_ = value;
        }
        else if (name == "max_board_edge_mismatch_ratio")
        {
          max_board_edge_mismatch_ratio_ = value;
        }
        else
        {
          min_board_marker_spacing_m_ = value;
        }
        RCLCPP_INFO(
          get_logger(),
          "Calibration perception %s updated: %.4f",
          name.c_str(), value);
        continue;
      }
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
      {
        result.successful = false;
        result.reason = "parent_frame must be a string.";
        return result;
      }
      const std::string value = parameter.as_string();
      if (value.empty())
      {
        result.successful = false;
        result.reason = "parent_frame must be non-empty.";
        return result;
      }
      parent_frame_ = value.front() == '/' ? value.substr(1) : value;
      RCLCPP_INFO(
        get_logger(),
        "Calibration perception parent frame updated: %s",
        parent_frame_.c_str());
    }
    return result;
  }

  bool collectBoardObservations(
    const aruco_perception::msg::MarkerDetections::SharedPtr &msg,
    std::array<BoardMarkerObservation, kRequiredMarkerCount> &observations,
    std::string &reason) const
  {
    if (msg->ids.size() != msg->poses.size())
    {
      std::ostringstream stream;
      stream << "Detection message has " << msg->ids.size() << " ids but "
             << msg->poses.size() << " poses.";
      reason = stream.str();
      return false;
    }
    if (msg->camera_centers.size() != msg->ids.size() ||
        msg->camera_corners.size() != msg->ids.size())
    {
      std::ostringstream stream;
      stream << "Detection message has " << msg->ids.size() << " ids, "
             << msg->camera_centers.size() << " camera centers, and "
             << msg->camera_corners.size() << " camera corner groups.";
      reason = stream.str();
      return false;
    }

    const std::unordered_set<int64_t> required_ids(marker_ids_.begin(), marker_ids_.end());
    std::unordered_map<int64_t, size_t> detection_by_id;
    for (size_t i = 0; i < msg->ids.size(); ++i)
    {
      const int64_t id = static_cast<int64_t>(msg->ids[i]);
      if (required_ids.find(id) == required_ids.end())
      {
        continue;
      }
      if (!detection_by_id.emplace(id, i).second)
      {
        reason = "Current detection frame has duplicate required marker ID " + std::to_string(id) + ".";
        return false;
      }
    }

    std::vector<int64_t> missing_ids;
    for (size_t layout_index = 0; layout_index < marker_ids_.size(); ++layout_index)
    {
      const int64_t id = marker_ids_[layout_index];
      const auto found = detection_by_id.find(id);
      if (found == detection_by_id.end())
      {
        missing_ids.push_back(id);
        continue;
      }

      const size_t detection_index = found->second;
      const auto &corner_polygon = msg->camera_corners[detection_index];
      if (corner_polygon.points.size() != 4U)
      {
        reason = "Marker " + std::to_string(id) + " does not have 4 depth camera corners.";
        return false;
      }

      BoardMarkerObservation observation;
      observation.id = id;
      observation.center = pointToEigen(msg->camera_centers[detection_index]);
      if (!isFiniteVector(observation.center) || observation.center.z() <= 0.0)
      {
        reason = "Marker " + std::to_string(id) + " has invalid depth center.";
        return false;
      }

      for (size_t corner_index = 0; corner_index < observation.corners.size(); ++corner_index)
      {
        observation.corners[corner_index] = point32ToEigen(corner_polygon.points[corner_index]);
        if (!isFiniteVector(observation.corners[corner_index]) ||
            observation.corners[corner_index].z() <= 0.0)
        {
          reason = "Marker " + std::to_string(id) + " has invalid depth corner.";
          return false;
        }
      }
      observations[layout_index] = observation;
    }

    if (!missing_ids.empty())
    {
      reason = "Current detection frame is missing required marker IDs " + formatMarkerIds(missing_ids) + ".";
      return false;
    }
    return true;
  }

  bool estimateDepthBoardTransform(
    const std::array<BoardMarkerObservation, kRequiredMarkerCount> &markers,
    geometry_msgs::msg::TransformStamped &board_tf,
    std::string &reason)
  {
    const Eigen::Vector3d &top_left = markers[0].center;
    const Eigen::Vector3d &top_right = markers[1].center;
    const Eigen::Vector3d &bottom_left = markers[2].center;
    const Eigen::Vector3d &bottom_right = markers[3].center;

    std::vector<Eigen::Vector3d> plane_points;
    plane_points.reserve((markers.size() * 5U));
    for (const auto &marker : markers)
    {
      plane_points.push_back(marker.center);
      for (const auto &corner : marker.corners)
      {
        plane_points.push_back(corner);
      }
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &point : plane_points)
    {
      centroid += point;
    }
    centroid /= static_cast<double>(plane_points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : plane_points)
    {
      const Eigen::Vector3d delta = point - centroid;
      covariance += delta * delta.transpose();
    }
    covariance /= static_cast<double>(plane_points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
      reason = "Could not fit a plane to board depth points.";
      return false;
    }

    Eigen::Vector3d plane_normal = solver.eigenvectors().col(0);
    if (plane_normal.norm() < kAxisMinNorm)
    {
      reason = "Board plane normal is degenerate.";
      return false;
    }
    plane_normal.normalize();

    double squared_error_sum = 0.0;
    for (const auto &point : plane_points)
    {
      const double signed_distance = (point - centroid).dot(plane_normal);
      squared_error_sum += signed_distance * signed_distance;
    }
    const double plane_rmse = std::sqrt(squared_error_sum / static_cast<double>(plane_points.size()));
    if (max_board_plane_rmse_m_ > 0.0 && plane_rmse > max_board_plane_rmse_m_)
    {
      std::ostringstream stream;
      stream << "Board depth plane RMS " << std::fixed << std::setprecision(4)
             << plane_rmse << "m exceeds " << max_board_plane_rmse_m_ << "m.";
      reason = stream.str();
      return false;
    }

    const double width_top = (top_right - top_left).norm();
    const double width_bottom = (bottom_right - bottom_left).norm();
    const double height_left = (bottom_left - top_left).norm();
    const double height_right = (bottom_right - top_right).norm();
    const double min_spacing = std::min({width_top, width_bottom, height_left, height_right});
    if (min_spacing < min_board_marker_spacing_m_)
    {
      std::ostringstream stream;
      stream << "Board marker spacing is too small (" << std::fixed << std::setprecision(4)
             << min_spacing << "m).";
      reason = stream.str();
      return false;
    }

    const double width_mismatch = relativeMismatch(width_top, width_bottom);
    const double height_mismatch = relativeMismatch(height_left, height_right);
    if ((max_board_edge_mismatch_ratio_ > 0.0) &&
        (width_mismatch > max_board_edge_mismatch_ratio_ ||
         height_mismatch > max_board_edge_mismatch_ratio_))
    {
      std::ostringstream stream;
      stream << "Board edge mismatch is too high (width " << std::fixed << std::setprecision(3)
             << width_mismatch << ", height " << height_mismatch << ").";
      reason = stream.str();
      return false;
    }

    const Eigen::Vector3d board_center =
      (top_left + top_right + bottom_left + bottom_right) * 0.25;
    const Eigen::Vector3d x_hint =
      ((top_right - top_left) + (bottom_right - bottom_left)) * 0.5;
    const Eigen::Vector3d y_hint =
      ((bottom_left - top_left) + (bottom_right - top_right)) * 0.5;

    Eigen::Vector3d x_axis = x_hint - plane_normal * x_hint.dot(plane_normal);
    if (x_axis.norm() < kAxisMinNorm)
    {
      reason = "Board X axis is degenerate.";
      return false;
    }
    x_axis.normalize();

    Eigen::Vector3d y_axis = y_hint - plane_normal * y_hint.dot(plane_normal);
    y_axis = y_axis - x_axis * y_axis.dot(x_axis);
    if (y_axis.norm() < kAxisMinNorm)
    {
      reason = "Board Y axis is degenerate.";
      return false;
    }
    y_axis.normalize();

    Eigen::Vector3d z_axis = x_axis.cross(y_axis);
    if (z_axis.norm() < kAxisMinNorm)
    {
      reason = "Board Z axis is degenerate.";
      return false;
    }
    z_axis.normalize();

    const double plane_alignment = std::fabs(z_axis.dot(plane_normal));
    if (plane_alignment < 0.80)
    {
      std::ostringstream stream;
      stream << "Board axes disagree with fitted plane (alignment "
             << std::fixed << std::setprecision(3) << plane_alignment << ").";
      reason = stream.str();
      return false;
    }

    if (z_axis.dot(board_center) < 0.0)
    {
      y_axis = -y_axis;
      z_axis = -z_axis;
    }

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    Eigen::Quaterniond q(rotation);
    q.normalize();

    board_tf.header.stamp = this->get_clock()->now();
    board_tf.header.frame_id = parent_frame_;
    board_tf.child_frame_id = output_frame_;
    board_tf.transform.translation.x = board_center.x();
    board_tf.transform.translation.y = board_center.y();
    board_tf.transform.translation.z = board_center.z();
    board_tf.transform.rotation.x = q.x();
    board_tf.transform.rotation.y = q.y();
    board_tf.transform.rotation.z = q.z();
    board_tf.transform.rotation.w = q.w();
    return true;
  }

  void detectionsCallback(const aruco_perception::msg::MarkerDetections::SharedPtr msg)
  {
    if (marker_ids_.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "No marker IDs configured; cannot fit depth board pose.");
      return;
    }

    const std::string frame_id =
      msg->header.frame_id.empty() ? parent_frame_ : msg->header.frame_id;
    if (frame_id != parent_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Detection frame is %s, expected %s; not publishing %s.",
        frame_id.c_str(), parent_frame_.c_str(), output_frame_.c_str());
      return;
    }

    if (max_marker_age_sec_ > 0.0)
    {
      const rclcpp::Time stamp(msg->header.stamp);
      const double age_sec = (this->now() - stamp).seconds();
      if (stamp.nanoseconds() == 0 || !std::isfinite(age_sec) || age_sec > max_marker_age_sec_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "Detection message is stale (age %.3fs, max %.3fs); not publishing %s.",
          age_sec, max_marker_age_sec_, output_frame_.c_str());
        return;
      }
    }

    std::array<BoardMarkerObservation, kRequiredMarkerCount> board_markers;
    std::string reason;
    if (!collectBoardObservations(msg, board_markers, reason))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000,
        "Current board visibility check failed: %s Not publishing %s.",
        reason.c_str(), output_frame_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped board_tf;
    if (!estimateDepthBoardTransform(board_markers, board_tf, reason))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000,
        "Current depth board pose check failed: %s Not publishing %s.",
        reason.c_str(), output_frame_.c_str());
      return;
    }

    tf_broadcaster_->sendTransform(board_tf);
  }
};
}  // namespace camera_calibration

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_calibration::CalibrationPerception>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
