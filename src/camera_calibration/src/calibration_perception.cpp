#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace camera_calibration
{
class CalibrationPerception : public rclcpp::Node
{
public:
  CalibrationPerception()
  : Node("calibration_perception")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    marker_prefix_ = declare_parameter<std::string>("marker_prefix", "aruco_marker");
    output_frame_ = declare_parameter<std::string>("output_frame", "tag_frame");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "camera_link");
    marker_ids_ = declare_parameter<std::vector<int64_t>>(
      "marker_ids", std::vector<int64_t>{1, 2, 3, 4});
    lookup_timeout_sec_ = declare_parameter<double>("lookup_timeout", 0.05);
    max_marker_age_sec_ = declare_parameter<double>("max_marker_age_sec", 1.5);
    const double publish_rate_hz = declare_parameter<double>("publish_rate", 20.0);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CalibrationPerception::handleParameterUpdate, this, std::placeholders::_1));

    double period_ms = 1000.0 / std::max(1e-3, publish_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&CalibrationPerception::publishAverage, this));

    RCLCPP_INFO(get_logger(),
                "Averaging %zu fresh markers (%s_*) into frame '%s' under parent '%s' at %.2f Hz "
                "(max marker age %.2fs).",
                marker_ids_.size(), marker_prefix_.c_str(), output_frame_.c_str(),
                parent_frame_.c_str(), publish_rate_hz, max_marker_age_sec_);
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::string marker_prefix_;
  std::string output_frame_;
  std::string parent_frame_;
  std::vector<int64_t> marker_ids_;
  double lookup_timeout_sec_{0.05};
  double max_marker_age_sec_{1.5};

  rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &parameter : parameters)
    {
      const auto &name = parameter.get_name();
      if (name != "parent_frame" && name != "max_marker_age_sec")
      {
        continue;
      }
      if (name == "max_marker_age_sec")
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.successful = false;
          result.reason = "max_marker_age_sec must be a number.";
          return result;
        }
        const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                               ? static_cast<double>(parameter.as_int())
                               : parameter.as_double();
        if (!std::isfinite(value) || value < 0.0)
        {
          result.successful = false;
          result.reason = "max_marker_age_sec must be finite and non-negative.";
          return result;
        }
        max_marker_age_sec_ = value;
        RCLCPP_INFO(
          get_logger(),
          "Calibration perception max marker age updated: %.3fs",
          max_marker_age_sec_);
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

  bool isFreshMarkerTransform(
    const geometry_msgs::msg::TransformStamped &tf,
    const std::string &child_frame)
  {
    if (max_marker_age_sec_ <= 0.0)
    {
      return true;
    }

    const rclcpp::Time stamp(tf.header.stamp);
    if (stamp.nanoseconds() == 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Marker transform %s -> %s has a zero timestamp; not publishing %s.",
        parent_frame_.c_str(), child_frame.c_str(), output_frame_.c_str());
      return false;
    }

    const double age_sec = (this->get_clock()->now() - stamp).seconds();
    if (!std::isfinite(age_sec) || age_sec > max_marker_age_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Marker transform %s -> %s is stale (age %.3fs > %.3fs); all %zu markers must be visible.",
        parent_frame_.c_str(), child_frame.c_str(), age_sec, max_marker_age_sec_, marker_ids_.size());
      return false;
    }

    return true;
  }

  void publishAverage()
  {
    if (marker_ids_.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "No marker IDs configured; nothing to average.");
      return;
    }

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(marker_ids_.size());

    for (const auto id : marker_ids_)
    {
      const std::string child_frame = marker_prefix_ + "_" + std::to_string(id);
      try
      {
        auto tf = tf_buffer_->lookupTransform(
          parent_frame_, child_frame, tf2::TimePointZero, tf2::durationFromSec(lookup_timeout_sec_));
        if (!isFreshMarkerTransform(tf, child_frame))
        {
          return;
        }
        transforms.push_back(tf);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 2000,
          "Missing transform %s -> %s: %s", parent_frame_.c_str(), child_frame.c_str(), ex.what());
        return;
      }
    }

    if (transforms.size() != marker_ids_.size())
    {
      return;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    double sum_qx = 0.0, sum_qy = 0.0, sum_qz = 0.0, sum_qw = 0.0;

    tf2::Quaternion ref_q;
    tf2::fromMsg(transforms.front().transform.rotation, ref_q);
    ref_q.normalize();

    for (const auto &tf : transforms)
    {
      sum_x += tf.transform.translation.x;
      sum_y += tf.transform.translation.y;
      sum_z += tf.transform.translation.z;

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      q.normalize();
      if (ref_q.dot(q) < 0.0)
      {
        q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
      }
      sum_qx += q.x();
      sum_qy += q.y();
      sum_qz += q.z();
      sum_qw += q.w();
    }

    const double inv_count = 1.0 / static_cast<double>(transforms.size());
    geometry_msgs::msg::TransformStamped avg;
    avg.header.stamp = this->get_clock()->now();
    avg.header.frame_id = parent_frame_;
    avg.child_frame_id = output_frame_;
    avg.transform.translation.x = sum_x * inv_count;
    avg.transform.translation.y = sum_y * inv_count;
    avg.transform.translation.z = sum_z * inv_count;

    tf2::Quaternion q_avg(sum_qx * inv_count, sum_qy * inv_count, sum_qz * inv_count, sum_qw * inv_count);
    q_avg.normalize();
    avg.transform.rotation = tf2::toMsg(q_avg);

    tf_broadcaster_->sendTransform(avg);
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
