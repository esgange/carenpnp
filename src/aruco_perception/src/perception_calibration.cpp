#include <filesystem>
#include <string>
#include <iostream>
#include <cstdlib>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <dobot_common/workspace_paths.hpp>

namespace aruco_perception
{
class PerceptionCalibration : public rclcpp::Node
{
public:
  PerceptionCalibration()
  : Node("perception_calibration"), static_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this))
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "Link6");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "calibrated_camera_link");
    calibration_dir_ = this->declare_parameter<std::string>(
      "calibration_dir", defaultCalibrationDir());
    calibration_file_ = this->declare_parameter<std::string>("calibration_file", "");
    auto_discover_ = this->declare_parameter<bool>("auto_discover", true);
    // Only broadcast when a calibration file is found; otherwise prompt and skip publishing.

    std::string path_used;
    Eigen::Quaterniond q;
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    bool loaded = false;
    std::string reason;

    if (calibration_file_.empty() && auto_discover_)
    {
      calibration_file_ = findLatestCalibration();
    }

    if (!calibration_file_.empty())
    {
      loaded = loadCalibration(calibration_file_, q, t, reason);
      if (!loaded)
      {
        RCLCPP_WARN(get_logger(), "Failed to load calibration '%s': %s",
                    calibration_file_.c_str(), reason.c_str());
      }
      else
      {
        path_used = calibration_file_;
      }
    }

    if (!loaded)
    {
      RCLCPP_WARN(get_logger(),
                  "Calibration not available; no transform will be published between %s and %s.",
                  parent_frame_.c_str(), child_frame_.c_str());
      RCLCPP_WARN(get_logger(),
                  "Press Enter to continue WITHOUT calibration (markers will stay in the camera/raw frame), "
                  "or Ctrl+C to abort and provide a calibration file.");
      std::string line;
      std::getline(std::cin, line);
      RCLCPP_INFO(get_logger(),
                  "Continuing without calibration; %s will not be broadcast. "
                  "Set downstream camera_frame to %s or the camera's raw frame.",
                  child_frame_.c_str(), parent_frame_.c_str());
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Loaded calibration from '%s'. Broadcasting static TF %s -> %s.",
                  path_used.c_str(), parent_frame_.c_str(), child_frame_.c_str());
    }

    publishTransform(q, t);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::string parent_frame_;
  std::string child_frame_;
  std::string calibration_dir_;
  std::string calibration_file_;
  bool auto_discover_{true};

  void publishTransform(const Eigen::Quaterniond &q_in, const Eigen::Vector3d &t_in)
  {
    Eigen::Quaterniond q = q_in;
    if (q.norm() < 1e-9)
    {
      q = Eigen::Quaterniond::Identity();
    }
    q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = t_in.x();
    tf_msg.transform.translation.y = t_in.y();
    tf_msg.transform.translation.z = t_in.z();
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    tf_msg.transform.rotation = tf2::toMsg(tf_q);

    static_broadcaster_->sendTransform(tf_msg);
  }

  std::string defaultCalibrationDir() const
  {
    return dobot_common::paths::workspacePath({"calibration"}, __FILE__).string();
  }

  std::string findLatestCalibration() const
  {
    try
    {
      const auto calib_dir = std::filesystem::path(calibration_dir_);
      if (!std::filesystem::exists(calib_dir) || !std::filesystem::is_directory(calib_dir))
      {
        return {};
      }

      std::filesystem::path latest_path;
      std::filesystem::file_time_type latest_time;
      for (const auto &entry : std::filesystem::directory_iterator(calib_dir))
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
      RCLCPP_WARN(this->get_logger(), "Failed to discover calibration files: %s", ex.what());
      return {};
    }
  }

  bool loadCalibration(const std::string &file_path, Eigen::Quaterniond &q,
                       Eigen::Vector3d &t, std::string &reason) const
  {
    try
    {
      if (!std::filesystem::exists(file_path))
      {
        reason = "File does not exist";
        return false;
      }
      if (std::filesystem::file_size(file_path) == 0)
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
      root = YAML::LoadFile(file_path);
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Could not read YAML: ") + ex.what();
      return false;
    }

    auto calib = root["calibration_transform"];
    if (!calib)
    {
      reason = "Missing 'calibration_transform' key";
      return false;
    }
    auto rot = calib["rotation"];
    auto trans = calib["translation"];
    if (!rot || !trans)
    {
      reason = "Missing rotation/translation keys";
      return false;
    }
    try
    {
      double w = rot["w"].as<double>();
      double x = rot["x"].as<double>();
      double y = rot["y"].as<double>();
      double z = rot["z"].as<double>();
      q = Eigen::Quaterniond(w, x, y, z);
      t = Eigen::Vector3d(trans["x"].as<double>(), trans["y"].as<double>(), trans["z"].as<double>());
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Failed to parse rotation/translation: ") + ex.what();
      return false;
    }

    if (q.norm() < 1e-9)
    {
      reason = "Invalid quaternion (zero norm)";
      return false;
    }
    q.normalize();
    return true;
  }
};
}  // namespace aruco_perception

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_perception::PerceptionCalibration>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
