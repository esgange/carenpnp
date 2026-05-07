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

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
std::filesystem::path defaultCameraCalibrationDir()
{
  const char *home = std::getenv("HOME");
  if (home == nullptr)
  {
    return std::filesystem::path("calibration");
  }
  return std::filesystem::path(home) / "DOBOT_pickn_place" / "calibration";
}

std::filesystem::path defaultCameraCalibrationPath()
{
  return defaultCameraCalibrationDir() / "axab_calibration.yaml";
}

bool isCameraCalibrationYaml(const std::filesystem::path &path)
{
  const std::string filename = path.filename().string();
  return path.extension() == ".yaml" &&
         (filename == "axab_calibration.yaml" || filename.rfind("axab_calibration_", 0) == 0);
}
}  // namespace

namespace camera_calibration
{
EyeOnHandCalibrator::EyeOnHandCalibrator()
: rclcpp::Node("eye_on_hand_calibrator")
{
  std::string default_output = "axab_calibration.yaml";
  try
  {
    const auto calib_dir = defaultCameraCalibrationDir();
    std::filesystem::create_directories(calib_dir);
    default_output = defaultCameraCalibrationPath().string();
  }
  catch (const std::exception &ex)
  {
    RCLCPP_WARN(get_logger(),
                "Could not resolve calibration directory in HOME, defaulting output to %s (%s)",
                default_output.c_str(), ex.what());
  }

  output_path_ = this->declare_parameter<std::string>("output_path", default_output);
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  gripper_frame_ = this->declare_parameter<std::string>("gripper_frame", "Link6");
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "tag_frame");
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

  RCLCPP_INFO(get_logger(), "Writing calibration output to: %s", output_path_.c_str());
  RCLCPP_INFO(get_logger(),
              "Using mode=%s frames base=%s, gripper=%s, camera=%s, target=%s (min_samples=%d)",
              calibration_mode_.c_str(),
              base_frame_.c_str(), gripper_frame_.c_str(),
              camera_frame_.c_str(), target_frame_.c_str(), min_samples_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  add_sample_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "add_sample",
    std::bind(&EyeOnHandCalibrator::handleAddSample, this, _1, _2),
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

  RCLCPP_INFO(get_logger(),
              "Calibration node ready (%s). Call /add_sample, /compute_calibration, "
              "or /reset_samples.",
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
  res->success = true;
  res->message = "Samples cleared.";
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
  std::vector<PoseSample> snapshot;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    snapshot = samples_;
  }

  if (static_cast<int>(snapshot.size()) < min_samples_)
  {
    RCLCPP_WARN(get_logger(), "Need at least %d samples, have %zu", min_samples_, snapshot.size());
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
        if (candidate == output_path || !isCameraCalibrationYaml(candidate))
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
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  double roll_rad, pitch_rad, yaw_rad;
  tf2::Matrix3x3(tf_q).getRPY(roll_rad, pitch_rad, yaw_rad);
  constexpr double RAD_TO_DEG = 180.0 / M_PI;
  const double roll_deg = roll_rad * RAD_TO_DEG;
  const double pitch_deg = pitch_rad * RAD_TO_DEG;
  const double yaw_deg = yaw_rad * RAD_TO_DEG;

  const auto now = this->now();
  int64_t nanoseconds = now.nanoseconds();
  int64_t seconds = nanoseconds / 1000000000LL;
  int64_t nanorem = nanoseconds % 1000000000LL;
  if (nanorem < 0)
  {
    nanorem += 1000000000LL;
    --seconds;
  }
  std::time_t time_sec = static_cast<std::time_t>(seconds);
  std::tm tm = *std::gmtime(&time_sec);
  std::stringstream timestamp_ss;
  timestamp_ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
               << "." << std::setw(6) << std::setfill('0') << (nanorem / 1000);

  out << std::fixed << std::setprecision(6);
  out << "calibration_transform:\n";
  out << "  rotation:\n";
  out << "    w: " << q.w() << "\n";
  out << "    x: " << q.x() << "\n";
  out << "    y: " << q.y() << "\n";
  out << "    z: " << q.z() << "\n";
  out << "  rotation_degrees:\n";
  out << "    roll: " << roll_deg << "\n";
  out << "    pitch: " << pitch_deg << "\n";
  out << "    yaw: " << yaw_deg << "\n";
  out << "  translation:\n";
  out << "    x: " << translation.x() << "\n";
  out << "    y: " << translation.y() << "\n";
  out << "    z: " << translation.z() << "\n";
  out << "metadata:\n";
  out << "  calibration_mode: " << calibration_mode_ << "\n";
  out << "  transform_parent_frame: " << transform_parent_frame << "\n";
  out << "  transform_child_frame: calibrated_camera_link\n";
  out << "  transform_type: " << (isEyeToHandMode() ? "base_to_camera" : "gripper_to_camera") << "\n";
  out << "  sample_count: " << sample_count << "\n";
  out << "  base_frame: " << base_frame_ << "\n";
  out << "  gripper_frame: " << gripper_frame_ << "\n";
  out << "  camera_frame: " << camera_frame << "\n";
  out << "  target_frame: " << target_frame_ << "\n";
  out << "  units:\n";
  out << "    rotation: quaternion/degrees\n";
  out << "    translation: meter\n";
  out << "timestamp: '" << timestamp_ss.str() << "'\n";
  return true;
}

void EyeOnHandCalibrator::publishCalibratedTransform(
  const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = isEyeToHandMode() ? base_frame_ : gripper_frame_;
  tf_msg.child_frame_id = "calibrated_camera_link";
  tf_msg.transform.translation.x = translation.x();
  tf_msg.transform.translation.y = translation.y();
  tf_msg.transform.translation.z = translation.z();
  Eigen::Quaterniond q(rotation);
  q.normalize();
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf_msg.transform.rotation = tf2::toMsg(tf_q);
  static_broadcaster_->sendTransform(tf_msg);
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
    has_solution_ = true;
  }
  publishCalibratedTransform(R, t);

  res->success = true;
  std::ostringstream oss;
  oss << "Calibration computed (not saved). Translation: [" << t.transpose()
      << "] Rotation matrix first row: [" << R.row(0)
      << "]. Broadcasted static TF " << parent_frame << " -> calibrated_camera_link "
      << "(mode=" << calibration_mode_ << "). "
      << "Use save_calibration to write YAML.";
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

  res->success = true;
  std::ostringstream oss;
  oss << "Saved cached calibration to " << output_path_ << " using " << used_samples
      << " samples. Broadcasted static TF " << parent_frame << " -> calibrated_camera_link "
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
