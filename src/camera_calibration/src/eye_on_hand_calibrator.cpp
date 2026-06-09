#include "camera_calibration/eye_on_hand_calibrator.hpp"

#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <cctype>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rmw/qos_profiles.h>

#include <dobot_common/robot_identity.hpp>
#include <dobot_common/workspace_paths.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
std::filesystem::path defaultCameraCalibrationDir()
{
  return dobot_common::paths::workspacePath({"calibration"}, __FILE__);
}

std::string currentDateStamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &now_time);
#else
  localtime_r(&now_time, &tm);
#endif
  std::ostringstream stream;
  stream << std::put_time(&tm, "%d%m%Y");
  return stream.str();
}

std::string calibrationModeFilenameToken(const std::string &mode)
{
  return mode == "eye_to_hand" ? "eyetohand" : "eyeonhand";
}

std::string trimCopy(const std::string &value)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  const auto first = std::find_if(value.begin(), value.end(), not_space);
  const auto last = std::find_if(value.rbegin(), value.rend(), not_space).base();
  if (first >= last)
  {
    return {};
  }
  return std::string(first, last);
}

std::string unquoteConfigValue(std::string value)
{
  value = trimCopy(value);
  if (value.size() >= 2 && value.front() == value.back() &&
      (value.front() == '"' || value.front() == '\''))
  {
    value = value.substr(1, value.size() - 2);
  }
  return trimCopy(value);
}

std::string readRobotIpFromStationConfig()
{
  const auto station_config_path = dobot_common::paths::workspacePath({"station_config"}, __FILE__);
  std::ifstream stream(station_config_path);
  if (!stream.good())
  {
    return {};
  }

  std::string robot_ip_address;
  std::string ip_address;
  std::string raw_line;
  while (std::getline(stream, raw_line))
  {
    std::string line = trimCopy(raw_line);
    if (line.empty() || line.front() == '#')
    {
      continue;
    }
    const std::string export_prefix = "export ";
    if (line.rfind(export_prefix, 0) == 0)
    {
      line = trimCopy(line.substr(export_prefix.size()));
    }
    const auto equals = line.find('=');
    if (equals == std::string::npos)
    {
      continue;
    }
    const std::string key = trimCopy(line.substr(0, equals));
    const std::string value = unquoteConfigValue(line.substr(equals + 1));
    if (key == "ROBOT_IP_ADDRESS" && !value.empty())
    {
      robot_ip_address = value;
    }
    else if (key == "ip_address" && !value.empty())
    {
      ip_address = value;
    }
  }

  return robot_ip_address.empty() ? ip_address : robot_ip_address;
}

std::string resolveRobotIpAddress(const std::string &requested)
{
  const std::string requested_ip = trimCopy(requested);
  if (!requested_ip.empty())
  {
    return requested_ip;
  }
  if (const char *env_ip = std::getenv("ROBOT_IP_ADDRESS"); env_ip != nullptr && *env_ip != '\0')
  {
    return trimCopy(env_ip);
  }
  return readRobotIpFromStationConfig();
}

std::string sanitizeFilenameToken(const std::string &value)
{
  std::string token;
  bool previous_was_underscore = false;
  for (unsigned char c : value)
  {
    if (std::isalnum(c) || c == '.' || c == '-' || c == '_')
    {
      token.push_back(static_cast<char>(c));
      previous_was_underscore = false;
    }
    else if (!previous_was_underscore)
    {
      token.push_back('_');
      previous_was_underscore = true;
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
  return token;
}

std::string calibrationFilenameForMode(const std::string &mode, const std::string &robot_ip_address)
{
  std::string filename = "axab_calibration_" + calibrationModeFilenameToken(mode) +
                         "_" + currentDateStamp();
  const std::string ip_token = sanitizeFilenameToken(robot_ip_address);
  if (!ip_token.empty())
  {
    filename += "_" + ip_token;
  }
  filename += ".yaml";
  return filename;
}

std::filesystem::path defaultCameraCalibrationPath(const std::string &mode,
                                                   const std::string &robot_ip_address)
{
  return defaultCameraCalibrationDir() / calibrationFilenameForMode(mode, robot_ip_address);
}

bool shouldReplaceCameraCalibrationYaml(const std::filesystem::path &path, const std::string &mode)
{
  const std::string filename = path.filename().string();
  if (path.extension() != ".yaml")
  {
    return false;
  }
  const std::string prefix = "axab_calibration_" + calibrationModeFilenameToken(mode) + "_";
  return filename.rfind(prefix, 0) == 0;
}

bool isStandardCalibrationFilenameForMode(const std::string &filename, const std::string &mode)
{
  const std::string prefix = "axab_calibration_" + calibrationModeFilenameToken(mode) + "_";
  return filename.rfind(prefix, 0) == 0 && std::filesystem::path(filename).extension() == ".yaml";
}

bool shouldNormalizeOutputPath(const std::string &path_text, const std::string &mode)
{
  const std::string filename = std::filesystem::path(path_text).filename().string();
  return filename.empty() || !isStandardCalibrationFilenameForMode(filename, mode);
}

std::string formatTransformYaml(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
  Eigen::Quaterniond q(rotation);
  q.normalize();

  std::ostringstream out;
  out << std::fixed << std::setprecision(9);
  out << "transform:\n";
  out << "  translation:\n";
  out << "    x: " << translation.x() << "\n";
  out << "    y: " << translation.y() << "\n";
  out << "    z: " << translation.z() << "\n";
  out << "  rotation:\n";
  out << "    x: " << q.x() << "\n";
  out << "    y: " << q.y() << "\n";
  out << "    z: " << q.z() << "\n";
  out << "    w: " << q.w();
  return out.str();
}
}  // namespace

namespace camera_calibration
{
EyeOnHandCalibrator::EyeOnHandCalibrator()
: rclcpp::Node("eye_on_hand_calibrator")
{
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  gripper_frame_ = this->declare_parameter<std::string>("gripper_frame", "Link6");
  camera_frame_ = this->declare_parameter<std::string>(
    "camera_frame", "robot_camera_color_optical_frame");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "tag_frame");
  max_target_age_sec_ = this->declare_parameter<double>("max_target_age_sec", 1.5);
  calibration_name_ = this->declare_parameter<std::string>("calibration_name", "cr10_orbbec335");
  tracking_base_frame_ = this->declare_parameter<std::string>(
    "tracking_base_frame", "robot_camera_color_optical_frame");
  tracking_marker_frame_ = this->declare_parameter<std::string>(
    "tracking_marker_frame", "charuco_target");
  freehand_robot_movement_ = this->declare_parameter<bool>("freehand_robot_movement", true);
  move_group_namespace_ = this->declare_parameter<std::string>("move_group_namespace", "/");
  move_group_ = this->declare_parameter<std::string>("move_group", "manipulator");
  min_samples_ = this->declare_parameter<int>("min_samples", 8);
  const std::string requested_mode =
    this->declare_parameter<std::string>("calibration_mode", "eye_on_hand");
  calibration_mode_ = normalizeCalibrationMode(requested_mode);
  std::string requested_mode_lower = requested_mode;
  std::transform(requested_mode_lower.begin(), requested_mode_lower.end(), requested_mode_lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (requested_mode_lower != calibration_mode_)
  {
    RCLCPP_WARN(get_logger(),
                "Unsupported calibration_mode '%s'; falling back to '%s'.",
                requested_mode.c_str(), calibration_mode_.c_str());
  }
  robot_ip_address_ = resolveRobotIpAddress(
    this->declare_parameter<std::string>("robot_ip_address", ""));
  calibrated_camera_frame_ = this->declare_parameter<std::string>(
    "calibrated_camera_frame", defaultCalibratedCameraFrame());
  std::string default_output = calibrationFilenameForMode(calibration_mode_, robot_ip_address_);
  try
  {
    const auto calib_dir = defaultCameraCalibrationDir();
    std::filesystem::create_directories(calib_dir);
    default_output = defaultCameraCalibrationPath(calibration_mode_, robot_ip_address_).string();
  }
  catch (const std::exception &ex)
  {
    RCLCPP_WARN(get_logger(),
                "Could not resolve calibration directory in HOME, defaulting output to %s (%s)",
                default_output.c_str(), ex.what());
  }

  output_path_ = this->declare_parameter<std::string>("output_path", default_output);
  if (shouldNormalizeOutputPath(output_path_, calibration_mode_))
  {
    output_path_ = default_output;
  }

  RCLCPP_INFO(get_logger(), "Writing calibration output to: %s", output_path_.c_str());
  if (robot_ip_address_.empty())
  {
    RCLCPP_WARN(get_logger(),
                "Robot IP address was not resolved; calibration filename will not include an IP suffix.");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Robot IP address for calibration filename: %s", robot_ip_address_.c_str());
  }
  RCLCPP_INFO(get_logger(),
              "Using mode=%s frames base=%s, gripper=%s, camera=%s, calibrated=%s, target=%s "
              "(min_samples=%d, max_target_age=%.2fs)",
              calibration_mode_.c_str(),
              base_frame_.c_str(), gripper_frame_.c_str(),
              camera_frame_.c_str(), calibrated_camera_frame_.c_str(),
              target_frame_.c_str(), min_samples_, max_target_age_sec_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  live_tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EyeOnHandCalibrator::broadcastLiveTransform, this));

  service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  add_sample_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "add_sample",
    std::bind(&EyeOnHandCalibrator::handleAddSample, this, _1, _2),
    rmw_qos_profile_services_default,
    service_group_);
  preview_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "preview_calibration",
    std::bind(&EyeOnHandCalibrator::handlePreview, this, _1, _2),
    rmw_qos_profile_services_default,
    service_group_);
  compute_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "compute_calibration",
    std::bind(&EyeOnHandCalibrator::handleCompute, this, _1, _2),
    rmw_qos_profile_services_default,
    service_group_);
  save_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "save_calibration",
    std::bind(&EyeOnHandCalibrator::handleSave, this, _1, _2),
    rmw_qos_profile_services_default,
    service_group_);
  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_samples",
    std::bind(&EyeOnHandCalibrator::handleReset, this, _1, _2),
    rmw_qos_profile_services_default,
    service_group_);
  remove_last_sample_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "remove_last_sample",
    std::bind(&EyeOnHandCalibrator::handleRemoveLastSample, this, _1, _2),
    rmw_qos_profile_services_default,
    service_group_);

  RCLCPP_INFO(get_logger(),
              "Calibration node ready (%s). Call /add_sample, /preview_calibration, "
              "/compute_calibration, /remove_last_sample, or /reset_samples.",
              calibration_mode_.c_str());
}

std::string EyeOnHandCalibrator::normalizeCalibrationMode(const std::string &mode)
{
  std::string normalized = mode;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (normalized == "eye_to_hand")
  {
    return "eye_to_hand";
  }
  return "eye_on_hand";
}

bool EyeOnHandCalibrator::isEyeToHandMode() const
{
  return calibration_mode_ == "eye_to_hand";
}

std::string EyeOnHandCalibrator::defaultCalibratedCameraFrame() const
{
  return isEyeToHandMode() ? "bin_calibrated_camera_link" : "arm_calibrated_camera_link";
}

void EyeOnHandCalibrator::handleAddSample(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mutex_);
  PoseSample sample;
  std::string reason;
  if (!fetchTransforms(sample, reason))
  {
    RCLCPP_WARN(get_logger(), "Failed to record sample: %s", reason.c_str());
    res->success = false;
    res->message = reason;
    return;
  }

  samples_.push_back(sample);
  has_solution_ = false;
  res->success = true;
  res->message = "Sample recorded. Total: " + std::to_string(samples_.size());
  RCLCPP_INFO(get_logger(), "Sample %zu recorded using frames (base=%s, gripper=%s, camera=%s, target=%s)",
              samples_.size(), base_frame_.c_str(), gripper_frame_.c_str(),
              camera_frame_.c_str(), target_frame_.c_str());
}

void EyeOnHandCalibrator::handleReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mutex_);
  samples_.clear();
  has_solution_ = false;
  has_live_transform_ = false;
  res->success = true;
  res->message = "Samples cleared.";
}

void EyeOnHandCalibrator::handleRemoveLastSample(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (samples_.empty())
  {
    res->success = false;
    res->message = "No samples to remove. Total: 0";
    return;
  }

  samples_.pop_back();
  has_solution_ = false;
  has_live_transform_ = false;
  last_used_samples_ = 0;
  live_used_samples_ = 0;
  res->success = true;
  res->message = "Last sample removed. Total: " + std::to_string(samples_.size());
  RCLCPP_INFO(get_logger(), "Last calibration sample removed. Total: %zu", samples_.size());
}

bool EyeOnHandCalibrator::fetchTransforms(PoseSample &sample, std::string &reason)
{
  geometry_msgs::msg::TransformStamped t_base_gripper;
  geometry_msgs::msg::TransformStamped t_camera_target;
  try
  {
    t_base_gripper = tf_buffer_->lookupTransform(
      base_frame_, gripper_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    reason = "TF lookup failed for base->gripper (" + base_frame_ + " -> " + gripper_frame_ +
             "): " + ex.what();
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
    return false;
  }
  try
  {
    t_camera_target = tf_buffer_->lookupTransform(
      camera_frame_, target_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    reason = "TF lookup failed for camera->target (" + camera_frame_ + " -> " + target_frame_ +
             "): " + ex.what();
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
    return false;
  }

  if (max_target_age_sec_ > 0.0)
  {
    const rclcpp::Time target_stamp(t_camera_target.header.stamp);
    const double target_age_sec = (this->now() - target_stamp).seconds();
    if (target_stamp.nanoseconds() == 0 || !std::isfinite(target_age_sec) ||
        target_age_sec > max_target_age_sec_)
    {
      std::ostringstream msg;
      msg << "Target TF is stale for camera->target (" << camera_frame_ << " -> " << target_frame_
          << "). Age " << std::fixed << std::setprecision(3) << target_age_sec
          << "s exceeds " << max_target_age_sec_
          << "s. Keep all 4 calibration markers visible before taking a sample.";
      reason = msg.str();
      RCLCPP_WARN(get_logger(), "%s", reason.c_str());
      return false;
    }
  }

  geometry_msgs::msg::PoseStamped ee_pose;
  ee_pose.header = t_base_gripper.header;
  ee_pose.pose = tf2::toMsg(tf2::transformToEigen(t_base_gripper));

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header = t_camera_target.header;
  target_pose.pose = tf2::toMsg(tf2::transformToEigen(t_camera_target));

  sample.end_effector = ee_pose;
  sample.target = target_pose;
  return true;
}

Eigen::Isometry3d EyeOnHandCalibrator::poseToIsometry(const geometry_msgs::msg::Pose &pose)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                       pose.orientation.z);
  q.normalize();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return iso;
}

void EyeOnHandCalibrator::eigenToCv(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, cv::Mat &R_out,
                                    cv::Mat &t_out)
{
  R_out = cv::Mat(3, 3, CV_64FC1);
  t_out = cv::Mat(3, 1, CV_64FC1);
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      R_out.at<double>(r, c) = R(r, c);
    }
    t_out.at<double>(r, 0) = t(r);
  }
}

bool EyeOnHandCalibrator::runCalibration(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation,
                                         std::string &camera_frame,
                                         std::string &transform_parent_frame,
                                         size_t &used_samples)
{
  return runCalibrationWithMinimumSamples(
    min_samples_, rotation, translation, camera_frame, transform_parent_frame, used_samples, nullptr);
}

bool EyeOnHandCalibrator::runCalibrationWithMinimumSamples(
  int required_samples, Eigen::Matrix3d &rotation, Eigen::Vector3d &translation,
  std::string &camera_frame, std::string &transform_parent_frame, size_t &used_samples,
  std::string *failure_reason)
{
  std::vector<PoseSample> snapshot;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    snapshot = samples_;
  }

  required_samples = std::max(1, required_samples);
  if (static_cast<int>(snapshot.size()) < required_samples)
  {
    std::ostringstream msg;
    msg << "Need at least " << required_samples << " samples, have " << snapshot.size();
    if (failure_reason != nullptr)
    {
      *failure_reason = msg.str();
    }
    RCLCPP_WARN(get_logger(), "%s", msg.str().c_str());
    return false;
  }

  std::vector<cv::Mat> R_gripper2base;
  std::vector<cv::Mat> t_gripper2base;
  std::vector<cv::Mat> R_target2cam;
  std::vector<cv::Mat> t_target2cam;
  R_gripper2base.reserve(snapshot.size());
  t_gripper2base.reserve(snapshot.size());
  R_target2cam.reserve(snapshot.size());
  t_target2cam.reserve(snapshot.size());

  for (const auto &sample : snapshot)
  {
    const auto T_gripper_to_base = poseToIsometry(sample.end_effector.pose);
    const auto T_target_to_camera = poseToIsometry(sample.target.pose);
    const auto T_robot = isEyeToHandMode() ? T_gripper_to_base.inverse() : T_gripper_to_base;

    cv::Mat Rb, tb, Rt, tt;
    eigenToCv(T_robot.rotation(), T_robot.translation(), Rb, tb);
    eigenToCv(T_target_to_camera.rotation(), T_target_to_camera.translation(), Rt, tt);

    R_gripper2base.push_back(Rb);
    t_gripper2base.push_back(tb);
    R_target2cam.push_back(Rt);
    t_target2cam.push_back(tt);
  }

  cv::Mat R_cam2gripper, t_cam2gripper;
  try
  {
    cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                         R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
  }
  catch (const cv::Exception &ex)
  {
    if (failure_reason != nullptr)
    {
      *failure_reason = ex.what();
    }
    RCLCPP_ERROR(get_logger(), "OpenCV calibrateHandEye failed: %s", ex.what());
    return false;
  }

  rotation = Eigen::Matrix3d::Identity();
  translation = Eigen::Vector3d::Zero();
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      rotation(r, c) = R_cam2gripper.at<double>(r, c);
    }
    translation(r) = t_cam2gripper.at<double>(r, 0);
  }

  camera_frame = camera_frame_;
  transform_parent_frame = isEyeToHandMode() ? base_frame_ : gripper_frame_;
  used_samples = snapshot.size();
  return true;
}

bool EyeOnHandCalibrator::writeResultYAML(const Eigen::Matrix3d &rotation,
                                          const Eigen::Vector3d &translation,
                                          const std::string &camera_frame,
                                          const std::string &transform_parent_frame,
                                          size_t sample_count) const
{
  const std::filesystem::path output_path(output_path_);
  try
  {
    const auto parent = output_path.parent_path();
    if (!parent.empty())
    {
      std::filesystem::create_directories(parent);
      for (const auto &entry : std::filesystem::directory_iterator(parent))
      {
        if (!entry.is_regular_file())
        {
          continue;
        }
        const auto candidate = entry.path();
        if (candidate == output_path || !shouldReplaceCameraCalibrationYaml(candidate, calibration_mode_) ||
            !dobot_common::robot_identity::filenameMatchesExactRobot(candidate, robot_ip_address_))
        {
          continue;
        }
        std::error_code ec;
        std::filesystem::remove(candidate, ec);
        if (ec)
        {
          RCLCPP_WARN(get_logger(),
                      "Failed to delete old camera calibration file %s: %s",
                      candidate.string().c_str(), ec.message().c_str());
        }
      }
    }
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(get_logger(), "Failed to prepare output directory: %s", ex.what());
    return false;
  }

  std::ofstream out(output_path);
  if (!out.good())
  {
    RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", output_path_.c_str());
    return false;
  }

  Eigen::Quaterniond q(rotation);
  q.normalize();
  (void)transform_parent_frame;
  (void)sample_count;
  const std::string calibration_type = isEyeToHandMode() ? "eye_on_base" : "eye_in_hand";
  const std::string tracking_base_frame =
    tracking_base_frame_.empty() ? camera_frame : tracking_base_frame_;
  const std::string tracking_marker_frame =
    tracking_marker_frame_.empty() ? target_frame_ : tracking_marker_frame_;

  out << std::fixed << std::setprecision(12);
  out << "parameters:\n";
  out << "  name: " << calibration_name_ << "\n";
  out << "  calibration_type: " << calibration_type << "\n";
  out << "  robot_base_frame: " << base_frame_ << "\n";
  out << "  robot_effector_frame: " << gripper_frame_ << "\n";
  out << "  transform_parent_frame: " << transform_parent_frame << "\n";
  out << "  transform_child_frame: " << calibrated_camera_frame_ << "\n";
  out << "  tracking_base_frame: " << tracking_base_frame << "\n";
  out << "  tracking_marker_frame: " << tracking_marker_frame << "\n";
  out << "  freehand_robot_movement: " << (freehand_robot_movement_ ? "true" : "false") << "\n";
  out << "  move_group_namespace: " << move_group_namespace_ << "\n";
  out << "  move_group: " << move_group_ << "\n";
  out << "transform:\n";
  out << "  translation:\n";
  out << "    x: " << translation.x() << "\n";
  out << "    y: " << translation.y() << "\n";
  out << "    z: " << translation.z() << "\n";
  out << "  rotation:\n";
  out << "    x: " << q.x() << "\n";
  out << "    y: " << q.y() << "\n";
  out << "    z: " << q.z() << "\n";
  out << "    w: " << q.w() << "\n";
  return true;
}

void EyeOnHandCalibrator::publishCalibratedTransform(
  const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = isEyeToHandMode() ? base_frame_ : gripper_frame_;
  tf_msg.child_frame_id = calibrated_camera_frame_;
  tf_msg.transform.translation.x = translation.x();
  tf_msg.transform.translation.y = translation.y();
  tf_msg.transform.translation.z = translation.z();
  Eigen::Quaterniond q(rotation);
  q.normalize();
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf_msg.transform.rotation = tf2::toMsg(tf_q);
  static_broadcaster_->sendTransform(tf_msg);
}

void EyeOnHandCalibrator::publishDynamicCalibratedTransform(
  const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation,
  const std::string &transform_parent_frame)
{
  if (!dynamic_broadcaster_)
  {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = transform_parent_frame;
  tf_msg.child_frame_id = calibrated_camera_frame_;
  tf_msg.transform.translation.x = translation.x();
  tf_msg.transform.translation.y = translation.y();
  tf_msg.transform.translation.z = translation.z();
  Eigen::Quaterniond q(rotation);
  q.normalize();
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf_msg.transform.rotation = tf2::toMsg(tf_q);
  dynamic_broadcaster_->sendTransform(tf_msg);
}

void EyeOnHandCalibrator::broadcastLiveTransform()
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  std::string parent_frame;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_live_transform_)
    {
      return;
    }
    rotation = live_rotation_;
    translation = live_translation_;
    parent_frame = live_parent_frame_;
  }

  publishDynamicCalibratedTransform(rotation, translation, parent_frame);
}

void EyeOnHandCalibrator::handlePreview(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::string camera_frame;
  std::string parent_frame;
  size_t used_samples = 0;
  std::string failure_reason;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (samples_.size() < 3U)
    {
      res->success = false;
      res->message =
        "Preview calibration waiting: OpenCV hand-eye solve needs at least 3 samples.";
      return;
    }
  }
  if (!runCalibrationWithMinimumSamples(
      3, R, t, camera_frame, parent_frame, used_samples, &failure_reason))
  {
    res->success = false;
    res->message = "Preview calibration unavailable: " + failure_reason;
    return;
  }

  {
    std::lock_guard<std::mutex> lk(mutex_);
    live_rotation_ = R;
    live_translation_ = t;
    live_parent_frame_ = parent_frame;
    live_used_samples_ = used_samples;
    has_live_transform_ = true;
  }
  publishDynamicCalibratedTransform(R, t, parent_frame);

  res->success = true;
  std::ostringstream oss;
  oss << "Preview calibration using " << used_samples << " samples. Live TF "
      << parent_frame << " -> " << calibrated_camera_frame_
      << " is rebroadcast while the calibrator is running"
      << " (input camera=" << camera_frame << ").\n"
      << formatTransformYaml(R, t);
  res->message = oss.str();
}

void EyeOnHandCalibrator::handleCompute(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::string camera_frame;
  std::string parent_frame;
  size_t used_samples = 0;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (samples_.size() < static_cast<size_t>(min_samples_))
    {
      res->success = false;
      res->message = "Not enough samples yet. Need at least " + std::to_string(min_samples_);
      return;
    }
  }
  if (!runCalibration(R, t, camera_frame, parent_frame, used_samples))
  {
    res->success = false;
    res->message = "Calibration failed or insufficient data.";
    return;
  }

  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_rotation_ = R;
    last_translation_ = t;
    last_camera_frame_ = camera_frame;
    last_parent_frame_ = parent_frame;
    last_used_samples_ = used_samples;
    live_rotation_ = R;
    live_translation_ = t;
    live_parent_frame_ = parent_frame;
    live_used_samples_ = used_samples;
    has_solution_ = true;
    has_live_transform_ = true;
  }
  publishCalibratedTransform(R, t);
  publishDynamicCalibratedTransform(R, t, parent_frame);

  res->success = true;
  std::ostringstream oss;
  oss << "Calibration computed (not saved). Translation: [" << t.transpose()
      << "] Rotation matrix first row: [" << R.row(0)
      << "]. Broadcasted static TF " << parent_frame << " -> " << calibrated_camera_frame_ << " "
      << "(mode=" << calibration_mode_ << "). "
      << "Use save_calibration to write YAML.\n"
      << formatTransformYaml(R, t);
  res->message = oss.str();
}

void EyeOnHandCalibrator::handleSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::string camera_frame;
  std::string parent_frame;
  size_t used_samples = 0;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_solution_)
    {
      res->success = false;
      res->message = "No computed solution cached. Run compute_calibration first.";
      return;
    }
    R = last_rotation_;
    t = last_translation_;
    camera_frame = last_camera_frame_;
    parent_frame = last_parent_frame_;
    used_samples = last_used_samples_;
  }

  if (!writeResultYAML(R, t, camera_frame, parent_frame, used_samples))
  {
    res->success = false;
    res->message = "Cached solution present, but writing YAML failed.";
    return;
  }

  publishCalibratedTransform(R, t);
  {
    std::lock_guard<std::mutex> lk(mutex_);
    live_rotation_ = R;
    live_translation_ = t;
    live_parent_frame_ = parent_frame;
    live_used_samples_ = used_samples;
    has_live_transform_ = true;
  }
  publishDynamicCalibratedTransform(R, t, parent_frame);

  res->success = true;
  std::ostringstream oss;
  oss << "Saved cached calibration to " << output_path_ << " using " << used_samples
      << " samples. Broadcasted static TF " << parent_frame << " -> " << calibrated_camera_frame_ << " "
      << "(mode=" << calibration_mode_ << ").";
  res->message = oss.str();
}
}  // namespace camera_calibration
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_calibration::EyeOnHandCalibrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
