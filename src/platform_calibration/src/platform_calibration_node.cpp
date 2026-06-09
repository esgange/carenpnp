#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <QApplication>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QTextCursor>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <dobot_common/robot_identity.hpp>
#include <dobot_common/workspace_paths.hpp>

namespace
{
using ImageMsg = sensor_msgs::msg::Image;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

constexpr size_t kRequiredMarkerCount = 4;
constexpr int64_t kMinArucoId = 0;
constexpr int64_t kMaxArucoId = 49;  // DICT_5X5_50

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
  return token.empty() ? "platform_reference" : token;
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
  if (raw == "~")
  {
    return std::filesystem::path(home);
  }
  if (raw.rfind("~/", 0) == 0)
  {
    return std::filesystem::path(home) / raw.substr(2);
  }
  return std::filesystem::path(raw);
}

std::filesystem::path defaultPlatformCalibrationDir()
{
  return dobot_common::paths::workspacePath({"calibration"}, __FILE__);
}

std::string formatMarkerIds(const std::vector<int64_t> &marker_ids)
{
  std::ostringstream stream;
  for (size_t i = 0; i < marker_ids.size(); ++i)
  {
    if (i > 0)
    {
      stream << ", ";
    }
    stream << marker_ids[i];
  }
  return stream.str();
}

bool validateMarkerIds(const std::vector<int64_t> &marker_ids, std::string &reason)
{
  if (marker_ids.size() != kRequiredMarkerCount)
  {
    reason = "marker_ids must contain exactly 4 ArUco IDs.";
    return false;
  }
  for (const auto id : marker_ids)
  {
    if (id < kMinArucoId || id > kMaxArucoId)
    {
      reason = "marker_ids must be in range 0..49 for DICT_5X5_50.";
      return false;
    }
  }
  reason.clear();
  return true;
}

std::string joinLines(const std::vector<std::string> &lines)
{
  std::ostringstream stream;
  for (size_t i = 0; i < lines.size(); ++i)
  {
    if (i > 0)
    {
      stream << "\n";
    }
    stream << lines[i];
  }
  return stream.str();
}

std::string joinStrings(const std::vector<std::string> &items, const std::string &separator)
{
  std::ostringstream stream;
  for (size_t i = 0; i < items.size(); ++i)
  {
    if (i > 0)
    {
      stream << separator;
    }
    stream << items[i];
  }
  return stream.str();
}

std::string timestampUtc(const rclcpp::Time &time)
{
  int64_t nanoseconds = time.nanoseconds();
  int64_t seconds = nanoseconds / 1000000000LL;
  int64_t nanorem = nanoseconds % 1000000000LL;
  if (nanorem < 0)
  {
    nanorem += 1000000000LL;
    --seconds;
  }
  std::time_t time_sec = static_cast<std::time_t>(seconds);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &time_sec);
#else
  gmtime_r(&time_sec, &tm);
#endif
  std::ostringstream stream;
  stream << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
         << "." << std::setw(6) << std::setfill('0') << (nanorem / 1000);
  return stream.str();
}

double quaternionAngularDistanceRad(
  const geometry_msgs::msg::Quaternion &a_msg,
  const geometry_msgs::msg::Quaternion &b_msg)
{
  tf2::Quaternion a;
  tf2::Quaternion b;
  tf2::fromMsg(a_msg, a);
  tf2::fromMsg(b_msg, b);
  if (a.length2() < 1e-12 || b.length2() < 1e-12)
  {
    return M_PI;
  }
  a.normalize();
  b.normalize();
  const double dot = std::clamp(std::fabs(a.dot(b)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

struct PlatformCalibration
{
  std::filesystem::path path;
  std::string platform_name;
  std::string parent_frame;
  std::string platform_frame;
  TransformStampedMsg transform;
};

struct BoardPoseSample
{
  std::chrono::steady_clock::time_point received;
  rclcpp::Time stamp;
  TransformStampedMsg transform;
};

struct StabilityStatus
{
  bool stable {false};
  double span_sec {0.0};
  double max_translation_m {0.0};
  double max_rotation_deg {0.0};
  std::string message;
};

struct BoardLookup
{
  TransformStampedMsg transform;
  int visible_marker_count {0};
  std::vector<int64_t> missing_marker_ids;
  StabilityStatus stability;
};

class PlatformCalibrationNode : public rclcpp::Node
{
public:
  PlatformCalibrationNode()
  : Node("platform_calibration"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, true),
    static_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
    dynamic_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    platform_name_ = declare_parameter<std::string>("platform_name", "Platform reference");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "bin_calibrated_camera_link");
    observed_board_frame_ = declare_parameter<std::string>("observed_board_frame", "platform_board_observed");
    marker_prefix_ = declare_parameter<std::string>("marker_prefix", "aruco_marker");
    marker_ids_ = declare_parameter<std::vector<int64_t>>(
      "marker_ids", std::vector<int64_t>{1, 2, 3, 4});
    std::string marker_ids_reason;
    if (!validateMarkerIds(marker_ids_, marker_ids_reason))
    {
      throw std::runtime_error(marker_ids_reason);
    }
    color_topic_ = declare_parameter<std::string>("color_topic", "/bin_camera/color/image_raw");
    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "/aruco_overlay");
    use_aruco_overlay_ = declare_parameter<bool>("use_aruco_overlay", true);
    lookup_timeout_sec_ = declare_parameter<double>("lookup_timeout", 0.15);
    max_marker_age_sec_ = declare_parameter<double>("max_marker_age_sec", 1.5);
    stability_window_sec_ = declare_parameter<double>("stability_window_sec", 1.0);
    stability_translation_tolerance_m_ = declare_parameter<double>(
      "stability_translation_tolerance_m", 0.001);
    stability_rotation_tolerance_deg_ = declare_parameter<double>(
      "stability_rotation_tolerance_deg", 1.0);
    delete_existing_on_save_ = declare_parameter<bool>("delete_existing_on_save", true);
    max_marker_age_sec_ = std::max(0.0, max_marker_age_sec_);
    stability_window_sec_ = std::max(0.1, stability_window_sec_);
    stability_translation_tolerance_m_ = std::max(0.0001, stability_translation_tolerance_m_);
    stability_rotation_tolerance_deg_ = std::max(0.1, stability_rotation_tolerance_deg_);
    const std::string default_dir = defaultPlatformCalibrationDir().string();
    robot_ip_address_ = dobot_common::robot_identity::resolveRobotIpAddress(
      declare_parameter<std::string>("robot_ip_address", ""), __FILE__);
    platform_calibration_dir_ = expandUserPath(
      declare_parameter<std::string>("platform_calibration_dir", default_dir));
    platform_calibration_file_ = declare_parameter<std::string>("platform_calibration_file", "");

    color_sub_ = create_subscription<ImageMsg>(
      color_topic_,
      rclcpp::SensorDataQoS(),
      [this](const ImageMsg::SharedPtr msg) { colorCallback(*msg); });
    overlay_sub_ = create_subscription<ImageMsg>(
      overlay_topic_,
      rclcpp::QoS(5),
      [this](const ImageMsg::SharedPtr msg) { overlayCallback(*msg); });
    loadExistingCalibrationIfAvailable();

    RCLCPP_INFO(
      get_logger(),
      "platform_calibration ready. Platform=%s base=%s observed_board=%s output_dir=%s stability=%.2fs %.1fmm %.1fdeg",
      platform_name_.c_str(),
      base_frame_.c_str(),
      observed_board_frame_.c_str(),
      platform_calibration_dir_.string().c_str(),
      stability_window_sec_,
      stability_translation_tolerance_m_ * 1000.0,
      stability_rotation_tolerance_deg_);
    if (robot_ip_address_.empty())
    {
      RCLCPP_WARN(
        get_logger(),
        "Robot IP address was not resolved; platform calibration filenames will not include an IP suffix.");
    }
    else
    {
      RCLCPP_INFO(
        get_logger(),
        "Robot IP address for platform calibration filename/load filtering: %s",
        robot_ip_address_.c_str());
    }
  }

  const std::string &platformName() const { return platform_name_; }
  const std::string &baseFrame() const { return base_frame_; }
  const std::string &cameraFrame() const { return camera_frame_; }
  const std::string &observedBoardFrame() const { return observed_board_frame_; }
  const std::string &colorTopic() const { return color_topic_; }
  const std::string &overlayTopic() const { return overlay_topic_; }
  const std::filesystem::path &platformCalibrationDir() const { return platform_calibration_dir_; }
  bool useArucoOverlay() const { return use_aruco_overlay_; }

  std::filesystem::path outputPathForName(const std::string &requested_name) const
  {
    if (!platform_calibration_file_.empty())
    {
      return expandUserPath(platform_calibration_file_);
    }
    std::string filename =
      "platform_calibration_" + sanitizeName(requested_name) + "_" +
      dobot_common::robot_identity::currentDateStamp();
    const std::string robot_ip_token =
      dobot_common::robot_identity::sanitizeFilenameToken(robot_ip_address_);
    if (!robot_ip_token.empty())
    {
      filename += "_" + robot_ip_token;
    }
    filename += ".yaml";
    return platform_calibration_dir_ / filename;
  }

  std::vector<std::string> runningCameraCalibrationNodes() const
  {
    static const std::set<std::string> kBlockingNodeNames{
      "camera_calibration_gui",
      "calibration_perception",
      "eye_on_hand_calibrator"};

    std::set<std::string> matches;
    for (const auto &full_name : get_node_names())
    {
      const auto slash = full_name.find_last_of('/');
      const std::string name = slash == std::string::npos ? full_name : full_name.substr(slash + 1);
      if (kBlockingNodeNames.count(name) == 0)
      {
        continue;
      }
      matches.insert(full_name);
    }
    return std::vector<std::string>(matches.begin(), matches.end());
  }

  bool cameraCalibrationConflictActive(std::string &reason) const
  {
    const std::vector<std::string> nodes = runningCameraCalibrationNodes();
    if (nodes.empty())
    {
      reason.clear();
      return false;
    }

    reason =
      "Camera calibration is running (" + joinStrings(nodes, ", ") +
      "). Close camera_calibration before using platform_calibration, then press Refresh.";
    return true;
  }

  bool observedBoardReady(std::string &reason)
  {
    if (cameraCalibrationConflictActive(reason))
    {
      return false;
    }
    BoardLookup lookup;
    return lookupObservedBoard(lookup, reason);
  }

  std::vector<std::string> statusLines(const std::string &requested_name)
  {
    const std::string safe_name = sanitizeName(requested_name);
    std::vector<std::string> lines;
    lines.push_back("Platform name: " + safe_name);
    lines.push_back("Output YAML: " + outputPathForName(safe_name).string());
    if (loaded_calibration_)
    {
      lines.push_back(
        "Loaded current platform: " + loaded_calibration_->parent_frame + " -> " +
        loaded_calibration_->platform_frame);
      lines.push_back("Loaded YAML: " + loaded_calibration_->path.string());
    }

    std::string conflict_reason;
    if (cameraCalibrationConflictActive(conflict_reason))
    {
      lines.push_back(
        "Board markers: 0/" + std::to_string(marker_ids_.size()) +
        " visible (" + formatMarkerIds(marker_ids_) + ")");
      lines.push_back("Teach status: close camera calibration first");
      lines.push_back(conflict_reason);
      return lines;
    }

    BoardLookup lookup;
    std::string reason;
    if (!lookupObservedBoard(lookup, reason))
    {
      lines.push_back(
        "Board markers: " + std::to_string(lookup.visible_marker_count) + "/" +
        std::to_string(marker_ids_.size()) + " visible (" + formatMarkerIds(marker_ids_) + ")");
      lines.push_back("Teach status: waiting for stable board pose");
      lines.push_back(reason);
      return lines;
    }

    const auto &observed = lookup.transform;
    const auto &t = observed.transform.translation;
    tf2::Quaternion q;
    tf2::fromMsg(observed.transform.rotation, q);
    q.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    constexpr double kRadToDeg = 180.0 / M_PI;
    lines.push_back(
      "Board markers: " + std::to_string(lookup.visible_marker_count) + "/" +
      std::to_string(marker_ids_.size()) + " visible (" + formatMarkerIds(marker_ids_) + ")");
    lines.push_back("Teach status: stable, ready to save");
    lines.push_back(lookup.stability.message);
    lines.push_back(
      "Live TF: " + base_frame_ + " -> " + observed_board_frame_ +
      " xyz m " + formatDouble(t.x, 4) + ", " + formatDouble(t.y, 4) + ", " + formatDouble(t.z, 4));
    lines.push_back(
      "RPY deg: " + formatDouble(roll * kRadToDeg, 2) + ", " +
      formatDouble(pitch * kRadToDeg, 2) + ", " + formatDouble(yaw * kRadToDeg, 2));
    return lines;
  }

  std::filesystem::path saveCalibration(const std::string &requested_name)
  {
    const std::string safe_name = sanitizeName(requested_name);
    std::string conflict_reason;
    if (cameraCalibrationConflictActive(conflict_reason))
    {
      throw std::runtime_error("Cannot save platform calibration: " + conflict_reason);
    }

    BoardLookup lookup;
    std::string reason;
    if (!lookupObservedBoard(lookup, reason))
    {
      throw std::runtime_error("Cannot save platform calibration: " + reason);
    }
    TransformStampedMsg observed = lookup.transform;

    const std::filesystem::path path = outputPathForName(safe_name);
    const auto parent_dir = path.parent_path();
    if (!parent_dir.empty())
    {
      std::filesystem::create_directories(parent_dir);
    }
    if (delete_existing_on_save_ && !parent_dir.empty())
    {
      deleteExistingPlatformFiles(parent_dir);
    }

    tf2::Quaternion q;
    tf2::fromMsg(observed.transform.rotation, q);
    if (q.length2() < 1e-12)
    {
      q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    q.normalize();

    const auto &t = observed.transform.translation;
    std::ofstream out(path);
    if (!out.good())
    {
      throw std::runtime_error("Failed to open output file: " + path.string());
    }

    out << std::fixed << std::setprecision(9);
    out << "transform:\n";
    out << "  translation:\n";
    out << "    x: " << t.x << "\n";
    out << "    y: " << t.y << "\n";
    out << "    z: " << t.z << "\n";
    out << "  rotation:\n";
    out << "    x: " << q.x() << "\n";
    out << "    y: " << q.y() << "\n";
    out << "    z: " << q.z() << "\n";
    out << "    w: " << q.w() << "\n";
    out << "metadata:\n";
    out << "  calibration_type: platform_reference\n";
    out << "  platform_name: " << safe_name << "\n";
    if (!robot_ip_address_.empty())
    {
      out << "  robot_ip_address: " << robot_ip_address_ << "\n";
    }
    out << "  transform_parent_frame: " << base_frame_ << "\n";
    out << "  transform_child_frame: " << safe_name << "\n";
    out << "  transform_type: base_to_platform\n";
    out << "  observed_board_frame: " << observed_board_frame_ << "\n";
    out << "  camera_frame: " << camera_frame_ << "\n";
    out << "  marker_prefix: " << marker_prefix_ << "\n";
    out << "  marker_ids: [";
    for (size_t i = 0; i < marker_ids_.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << marker_ids_[i];
    }
    out << "]\n";
    out << "  units:\n";
    out << "    rotation: quaternion\n";
    out << "    translation: meter\n";
    out << "timestamp: '" << timestampUtc(now()) << "'\n";
    out << "notes: 'Platform calibration is the saved pose of the calibration board in the robot base frame. Bin teach transforms are saved relative to this platform frame.'\n";
    out.close();
    if (!out.good())
    {
      throw std::runtime_error("Failed while writing output file: " + path.string());
    }

    PlatformCalibration calibration;
    calibration.path = path;
    calibration.platform_name = safe_name;
    calibration.parent_frame = base_frame_;
    calibration.platform_frame = safe_name;
    calibration.transform = observed;
    calibration.transform.header.frame_id = base_frame_;
    calibration.transform.child_frame_id = safe_name;
    loaded_calibration_ = calibration;
    platform_name_ = safe_name;
    publishPlatformTransform(calibration);

    RCLCPP_INFO(
      get_logger(),
      "Saved platform calibration %s -> %s to %s",
      base_frame_.c_str(),
      safe_name.c_str(),
      path.string().c_str());
    return path;
  }

  QImage latestVisualizationQImage() const
  {
    constexpr double kImageFreshMaxAgeSec = 3.0;
    const bool overlay_fresh =
      use_aruco_overlay_ &&
      !latest_overlay_qimage_.isNull() &&
      latest_overlay_received_monotonic_.has_value() &&
      secondsSince(*latest_overlay_received_monotonic_) <= kImageFreshMaxAgeSec;
    const bool camera_fresh =
      !latest_camera_qimage_.isNull() &&
      latest_camera_received_monotonic_.has_value() &&
      secondsSince(*latest_camera_received_monotonic_) <= kImageFreshMaxAgeSec;
    const QImage base_image = overlay_fresh
      ? latest_overlay_qimage_
      : (camera_fresh ? latest_camera_qimage_ : QImage());
    if (base_image.isNull())
    {
      return QImage();
    }
    return base_image.convertToFormat(QImage::Format_RGB32);
  }

private:
  std::string platform_name_;
  std::string base_frame_;
  std::string camera_frame_;
  std::string observed_board_frame_;
  std::string marker_prefix_;
  std::vector<int64_t> marker_ids_;
  std::string color_topic_;
  std::string overlay_topic_;
  bool use_aruco_overlay_{true};
  double lookup_timeout_sec_{0.15};
  double max_marker_age_sec_{1.5};
  double stability_window_sec_{1.0};
  double stability_translation_tolerance_m_{0.001};
  double stability_rotation_tolerance_deg_{1.0};
  bool delete_existing_on_save_{true};
  std::filesystem::path platform_calibration_dir_;
  std::string platform_calibration_file_;
  std::string robot_ip_address_;

  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::optional<PlatformCalibration> loaded_calibration_;
  QImage latest_camera_qimage_;
  QImage latest_overlay_qimage_;
  std::optional<std::chrono::steady_clock::time_point> latest_camera_received_monotonic_;
  std::optional<std::chrono::steady_clock::time_point> latest_overlay_received_monotonic_;
  std::set<std::string> unsupported_image_encodings_;
  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr overlay_sub_;
  std::deque<BoardPoseSample> board_pose_history_;
  std::optional<rclcpp::Time> last_observed_stamp_;
  std::optional<StabilityStatus> latest_stability_;

  void resetStabilityHistory()
  {
    board_pose_history_.clear();
    last_observed_stamp_.reset();
    latest_stability_.reset();
  }

  StabilityStatus computeStabilityStatus() const
  {
    StabilityStatus status;
    if (board_pose_history_.empty())
    {
      status.message = "Stability: waiting for marker samples.";
      return status;
    }

    const auto &newest = board_pose_history_.back();
    status.span_sec = std::chrono::duration<double>(
      newest.received - board_pose_history_.front().received).count();
    if (status.span_sec < stability_window_sec_)
    {
      status.message =
        "Stability: collecting " + formatDouble(status.span_sec, 2) + "/" +
        formatDouble(stability_window_sec_, 2) + "s of marker pose.";
      return status;
    }

    const auto &ref_t = newest.transform.transform.translation;
    const auto &ref_q = newest.transform.transform.rotation;
    double max_translation = 0.0;
    double max_rotation_rad = 0.0;
    for (const auto &sample : board_pose_history_)
    {
      const auto &t = sample.transform.transform.translation;
      const double axis_delta = std::max({
        std::fabs(t.x - ref_t.x),
        std::fabs(t.y - ref_t.y),
        std::fabs(t.z - ref_t.z)});
      max_translation = std::max(max_translation, axis_delta);
      max_rotation_rad = std::max(
        max_rotation_rad,
        quaternionAngularDistanceRad(sample.transform.transform.rotation, ref_q));
    }

    constexpr double kRadToDeg = 180.0 / M_PI;
    status.max_translation_m = max_translation;
    status.max_rotation_deg = max_rotation_rad * kRadToDeg;
    status.stable =
      max_translation <= stability_translation_tolerance_m_ &&
      status.max_rotation_deg <= stability_rotation_tolerance_deg_;
    status.message =
      "Stability: " + formatDouble(status.max_translation_m * 1000.0, 2) +
      "mm / " + formatDouble(status.max_rotation_deg, 2) + "deg over " +
      formatDouble(status.span_sec, 2) + "s (limits " +
      formatDouble(stability_translation_tolerance_m_ * 1000.0, 1) +
      "mm / " + formatDouble(stability_rotation_tolerance_deg_, 1) + "deg).";
    return status;
  }

  StabilityStatus updateStabilityStatus(const TransformStampedMsg &observed)
  {
    const rclcpp::Time stamp(observed.header.stamp);
    if (stamp.nanoseconds() != 0 &&
        last_observed_stamp_.has_value() &&
        stamp.nanoseconds() == last_observed_stamp_->nanoseconds())
    {
      return latest_stability_.value_or(computeStabilityStatus());
    }

    const auto received = steadyNow();
    board_pose_history_.push_back(BoardPoseSample{received, stamp, observed});
    if (stamp.nanoseconds() != 0)
    {
      last_observed_stamp_ = stamp;
    }

    const double retention_sec = std::max(stability_window_sec_ * 2.0, stability_window_sec_ + 0.5);
    while (!board_pose_history_.empty() &&
           std::chrono::duration<double>(received - board_pose_history_.front().received).count() > retention_sec)
    {
      board_pose_history_.pop_front();
    }

    latest_stability_ = computeStabilityStatus();
    return *latest_stability_;
  }

  std::vector<std::string> markerFrameNames() const
  {
    std::unordered_map<int64_t, size_t> id_counts;
    for (const auto id : marker_ids_)
    {
      ++id_counts[id];
    }

    std::unordered_map<int64_t, size_t> id_occurrences;
    std::vector<std::string> frame_names;
    frame_names.reserve(marker_ids_.size());
    for (const auto id : marker_ids_)
    {
      std::string frame_name = marker_prefix_ + "_" + std::to_string(id);
      if (id_counts[id] > 1)
      {
        const size_t occurrence = ++id_occurrences[id];
        frame_name += "_" + std::to_string(occurrence);
      }
      frame_names.push_back(frame_name);
    }
    return frame_names;
  }

  bool isFreshMarkerTransform(
    const TransformStampedMsg &tf,
    const std::string &child_frame,
    std::string &freshness_reason) const
  {
    if (max_marker_age_sec_ <= 0.0)
    {
      freshness_reason.clear();
      return true;
    }

    const rclcpp::Time stamp(tf.header.stamp);
    if (stamp.nanoseconds() == 0)
    {
      freshness_reason =
        "Marker TF " + base_frame_ + " -> " + child_frame +
        " has a zero timestamp.";
      return false;
    }

    const double age_sec = (now() - stamp).seconds();
    if (!std::isfinite(age_sec) || age_sec > max_marker_age_sec_)
    {
      freshness_reason =
        "Marker TF " + base_frame_ + " -> " + child_frame +
        " is stale: " + formatDouble(std::max(0.0, age_sec), 2) + "s > " +
        formatDouble(max_marker_age_sec_, 1) + "s.";
      return false;
    }

    freshness_reason.clear();
    return true;
  }

  bool lookupObservedBoard(BoardLookup &lookup, std::string &reason)
  {
    return lookupObservedBoardFromMarkerTf(lookup, reason);
  }

  bool lookupObservedBoardFromMarkerTf(BoardLookup &lookup, std::string &reason)
  {
    if (marker_ids_.empty())
    {
      resetStabilityHistory();
      reason = "No marker IDs configured for platform_calibration.";
      return false;
    }

    const std::vector<std::string> child_frames = markerFrameNames();
    std::vector<TransformStampedMsg> transforms;
    transforms.reserve(child_frames.size());
    lookup.missing_marker_ids.clear();
    std::string last_error;

    for (size_t index = 0; index < child_frames.size(); ++index)
    {
      const std::string &child_frame = child_frames[index];
      try
      {
        auto tf = tf_buffer_.lookupTransform(
          base_frame_,
          child_frame,
          tf2::TimePointZero,
          tf2::durationFromSec(std::max(0.0, lookup_timeout_sec_)));

        std::string freshness_reason;
        if (!isFreshMarkerTransform(tf, child_frame, freshness_reason))
        {
          lookup.missing_marker_ids.push_back(marker_ids_[index]);
          last_error = freshness_reason;
          continue;
        }
        transforms.push_back(tf);
      }
      catch (const tf2::TransformException &ex)
      {
        lookup.missing_marker_ids.push_back(marker_ids_[index]);
        last_error = "Missing TF " + base_frame_ + " -> " + child_frame + ": " + ex.what();
      }
    }

    lookup.visible_marker_count = static_cast<int>(transforms.size());
    if (transforms.size() != child_frames.size())
    {
      resetStabilityHistory();
      reason =
        "Waiting for marker TFs in " + base_frame_ + ": visible " +
        std::to_string(lookup.visible_marker_count) + "/" +
        std::to_string(marker_ids_.size());
      if (!lookup.missing_marker_ids.empty())
      {
        reason += ", missing " + formatMarkerIds(lookup.missing_marker_ids);
      }
      reason += ".";
      if (!last_error.empty())
      {
        reason += " " + last_error;
      }
      return false;
    }

    tf2::Quaternion ref_q;
    bool have_ref_q = false;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_qx = 0.0;
    double sum_qy = 0.0;
    double sum_qz = 0.0;
    double sum_qw = 0.0;

    for (const auto &tf : transforms)
    {
      const auto &translation = tf.transform.translation;
      sum_x += translation.x;
      sum_y += translation.y;
      sum_z += translation.z;

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      if (q.length2() < 1e-12)
      {
        q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
      }
      q.normalize();
      if (!have_ref_q)
      {
        ref_q = q;
        have_ref_q = true;
      }
      else if (ref_q.dot(q) < 0.0)
      {
        q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
      }

      sum_qx += q.x();
      sum_qy += q.y();
      sum_qz += q.z();
      sum_qw += q.w();
    }

    const double inv_count = 1.0 / static_cast<double>(transforms.size());
    tf2::Quaternion avg_q(sum_qx * inv_count, sum_qy * inv_count, sum_qz * inv_count, sum_qw * inv_count);
    if (avg_q.length2() < 1e-12)
    {
      avg_q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    avg_q.normalize();

    lookup.transform.header.stamp = now();
    lookup.transform.header.frame_id = base_frame_;
    lookup.transform.child_frame_id = observed_board_frame_;
    lookup.transform.transform.translation.x = sum_x * inv_count;
    lookup.transform.transform.translation.y = sum_y * inv_count;
    lookup.transform.transform.translation.z = sum_z * inv_count;
    lookup.transform.transform.rotation = tf2::toMsg(avg_q);
    if (dynamic_broadcaster_)
    {
      dynamic_broadcaster_->sendTransform(lookup.transform);
    }

    lookup.stability = updateStabilityStatus(lookup.transform);
    if (!lookup.stability.stable)
    {
      reason = lookup.stability.message;
      return false;
    }
    return true;
  }

  static bool isPlatformCalibrationFilename(const std::filesystem::path &path)
  {
    const std::string filename = path.filename().string();
    return filename == "platform.yaml" ||
           (filename.rfind("platform_calibration_", 0) == 0 && path.extension() == ".yaml");
  }

  void deleteExistingPlatformFiles(const std::filesystem::path &dir) const
  {
    if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir))
    {
      return;
    }
    for (const auto &entry : std::filesystem::directory_iterator(dir))
    {
      if (!entry.is_regular_file())
      {
        continue;
      }
      const auto path = entry.path();
      if (!isPlatformCalibrationFilename(path))
      {
        continue;
      }
      if (!dobot_common::robot_identity::filenameMatchesExactRobot(path, robot_ip_address_))
      {
        continue;
      }
      std::error_code ec;
      std::filesystem::remove(path, ec);
      if (ec)
      {
        RCLCPP_WARN(
          get_logger(),
          "Failed to delete old platform calibration %s: %s",
          path.string().c_str(),
          ec.message().c_str());
      }
    }
  }

  std::filesystem::path findLatestPlatformCalibrationFile() const
  {
    try
    {
      if (!std::filesystem::exists(platform_calibration_dir_) ||
          !std::filesystem::is_directory(platform_calibration_dir_))
      {
        return {};
      }
      dobot_common::robot_identity::LatestRobotFileSelection selection;
      for (const auto &entry : std::filesystem::directory_iterator(platform_calibration_dir_))
      {
        if (!entry.is_regular_file())
        {
          continue;
        }
        const auto path = entry.path();
        if (!isPlatformCalibrationFilename(path) || std::filesystem::file_size(path) == 0)
        {
          continue;
        }
        selection.consider(path, entry.last_write_time(), robot_ip_address_);
      }
      return selection.selected();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(), "Failed to discover platform calibration files: %s", ex.what());
      return {};
    }
  }

  bool loadCalibrationFromFile(const std::filesystem::path &path, PlatformCalibration &calibration, std::string &reason) const
  {
    try
    {
      if (!std::filesystem::exists(path))
      {
        reason = "File does not exist";
        return false;
      }
      if (std::filesystem::file_size(path) == 0)
      {
        reason = "File is empty";
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
      root = YAML::LoadFile(path.string());
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Could not read YAML: ") + ex.what();
      return false;
    }

    const auto calib = root["transform"] ? root["transform"] : root["calibration_transform"];
    if (!calib || !calib["rotation"] || !calib["translation"])
    {
      reason = "Missing transform rotation/translation";
      return false;
    }
    const auto metadata = root["metadata"];
    if (!metadata || !metadata["transform_parent_frame"] || !metadata["transform_child_frame"])
    {
      reason = "Missing metadata transform_parent_frame/transform_child_frame";
      return false;
    }

    try
    {
      const auto rot = calib["rotation"];
      const auto trans = calib["translation"];
      tf2::Quaternion q(
        rot["x"].as<double>(),
        rot["y"].as<double>(),
        rot["z"].as<double>(),
        rot["w"].as<double>());
      if (q.length2() < 1e-12)
      {
        reason = "Invalid quaternion";
        return false;
      }
      q.normalize();
      calibration.path = path;
      calibration.parent_frame = metadata["transform_parent_frame"].as<std::string>();
      calibration.platform_frame = metadata["transform_child_frame"].as<std::string>();
      calibration.platform_name = metadata["platform_name"] ? metadata["platform_name"].as<std::string>() : calibration.platform_frame;
      calibration.transform.header.frame_id = calibration.parent_frame;
      calibration.transform.child_frame_id = calibration.platform_frame;
      calibration.transform.transform.translation.x = trans["x"].as<double>();
      calibration.transform.transform.translation.y = trans["y"].as<double>();
      calibration.transform.transform.translation.z = trans["z"].as<double>();
      calibration.transform.transform.rotation = tf2::toMsg(q);
    }
    catch (const std::exception &ex)
    {
      reason = std::string("Failed to parse platform calibration: ") + ex.what();
      return false;
    }
    return true;
  }

  void loadExistingCalibrationIfAvailable()
  {
    const std::filesystem::path path = platform_calibration_file_.empty()
      ? findLatestPlatformCalibrationFile()
      : expandUserPath(platform_calibration_file_);
    if (path.empty())
    {
      return;
    }

    PlatformCalibration calibration;
    std::string reason;
    if (!loadCalibrationFromFile(path, calibration, reason))
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load existing platform calibration %s: %s",
        path.string().c_str(),
        reason.c_str());
      return;
    }

    loaded_calibration_ = calibration;
    if (platform_name_ == "robot_platform_1" && !calibration.platform_name.empty())
    {
      platform_name_ = calibration.platform_name;
    }
    publishPlatformTransform(calibration);
    RCLCPP_INFO(
      get_logger(),
      "Loaded platform calibration from %s. Broadcasting static TF %s -> %s.",
      path.string().c_str(),
      calibration.parent_frame.c_str(),
      calibration.platform_frame.c_str());
  }

  void publishPlatformTransform(const PlatformCalibration &calibration) const
  {
    if (!static_broadcaster_)
    {
      return;
    }
    TransformStampedMsg msg = calibration.transform;
    msg.header.stamp = now();
    static_broadcaster_->sendTransform(msg);
  }

  void colorCallback(const ImageMsg &msg)
  {
    const QImage image = imageMsgToQImage(msg, color_topic_);
    if (!image.isNull())
    {
      latest_camera_qimage_ = image;
      latest_camera_received_monotonic_ = steadyNow();
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
    std::transform(
      encoding.begin(),
      encoding.end(),
      encoding.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (encoding == "rgb8" || encoding == "8uc3")
    {
      QImage image(
        msg.data.data(),
        static_cast<int>(msg.width),
        static_cast<int>(msg.height),
        static_cast<int>(msg.step),
        QImage::Format_RGB888);
      return image.copy();
    }
    if (encoding == "bgr8")
    {
      QImage image(
        msg.data.data(),
        static_cast<int>(msg.width),
        static_cast<int>(msg.height),
        static_cast<int>(msg.step),
        QImage::Format_RGB888);
      return image.rgbSwapped().copy();
    }
    if (encoding == "mono8" || encoding == "8uc1")
    {
      QImage image(
        msg.data.data(),
        static_cast<int>(msg.width),
        static_cast<int>(msg.height),
        static_cast<int>(msg.step),
        QImage::Format_Grayscale8);
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
};

class PlatformCalibrationWindow : public QWidget
{
public:
  explicit PlatformCalibrationWindow(const std::shared_ptr<PlatformCalibrationNode> &node)
  : node_(node)
  {
    status_hold_until_ = steadyNow();
    setWindowTitle("platform_calibration");
    setMinimumSize(1040, 680);

    platform_name_ = new QLineEdit(QString::fromStdString(node_->platformName()), this);
    base_label_ = new QLabel(QString::fromStdString(node_->baseFrame()), this);
    observed_label_ = new QLabel(QString::fromStdString(node_->observedBoardFrame()), this);
    camera_label_ = new QLabel(QString::fromStdString(node_->cameraFrame()), this);
    output_label_ = new QLabel(QString::fromStdString(node_->outputPathForName(node_->platformName()).string()), this);
    output_label_->setWordWrap(true);

    auto *form = new QFormLayout();
    form->addRow("Platform name", platform_name_);
    form->addRow("Base frame", base_label_);
    form->addRow("Observed board frame", observed_label_);
    form->addRow("Camera frame", camera_label_);
    form->addRow("Output YAML", output_label_);

    refresh_button_ = new QPushButton("Refresh", this);
    save_button_ = new QPushButton("Save platform", this);
    auto *button_row = new QHBoxLayout();
    button_row->addWidget(refresh_button_);
    button_row->addWidget(save_button_);

    status_ = new QPlainTextEdit(this);
    status_->setReadOnly(true);
    status_->setMinimumHeight(210);

    auto *controls_widget = new QWidget(this);
    controls_widget->setMinimumWidth(340);
    controls_widget->setMaximumWidth(410);
    controls_widget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    auto *controls_layout = new QVBoxLayout(controls_widget);
    controls_layout->addLayout(form);
    controls_layout->addLayout(button_row);
    controls_layout->addWidget(status_, 1);

    const std::string image_title = node_->useArucoOverlay()
      ? "ArUco Overlay (" + node_->overlayTopic() + ")"
      : "Camera View (" + node_->colorTopic() + ")";
    auto *image_group = new QGroupBox(QString::fromStdString(image_title), this);
    image_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto *image_layout = new QVBoxLayout(image_group);
    image_label_ = new QLabel("Waiting for camera image ...", this);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setMinimumSize(620, 460);
    image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    image_label_->setStyleSheet("QLabel { background-color: #101010; color: #d0d0d0; border: 1px solid #444; }");
    image_layout->addWidget(image_label_);

    auto *root_layout = new QHBoxLayout(this);
    root_layout->addWidget(controls_widget, 0);
    root_layout->addWidget(image_group, 1);

    connect(refresh_button_, &QPushButton::clicked, this, [this]() {
      status_hold_until_ = steadyNow();
      refreshStatus();
    });
    connect(save_button_, &QPushButton::clicked, this, [this]() { savePlatform(); });
    connect(platform_name_, &QLineEdit::textChanged, this, [this]() {
      output_label_->setText(QString::fromStdString(node_->outputPathForName(platform_name_->text().toStdString()).string()));
      refreshStatus();
    });

    auto *ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, [this]() {
      if (!rclcpp::ok())
      {
        QApplication::quit();
        return;
      }
      rclcpp::spin_some(node_);
    });
    ros_timer->start(20);

    auto *status_timer = new QTimer(this);
    connect(status_timer, &QTimer::timeout, this, [this]() { refreshStatus(); });
    status_timer->start(500);

    auto *image_timer = new QTimer(this);
    connect(image_timer, &QTimer::timeout, this, [this]() { refreshImage(); });
    image_timer->start(100);
  }

private:
  std::shared_ptr<PlatformCalibrationNode> node_;
  std::chrono::steady_clock::time_point status_hold_until_;
  bool calibration_conflict_warning_shown_{false};

  QLineEdit *platform_name_{nullptr};
  QLabel *base_label_{nullptr};
  QLabel *observed_label_{nullptr};
  QLabel *camera_label_{nullptr};
  QLabel *output_label_{nullptr};
  QPushButton *refresh_button_{nullptr};
  QPushButton *save_button_{nullptr};
  QPlainTextEdit *status_{nullptr};
  QLabel *image_label_{nullptr};

  void refreshStatus()
  {
    std::string conflict_reason;
    const bool calibration_conflict = node_->cameraCalibrationConflictActive(conflict_reason);
    if (calibration_conflict && !calibration_conflict_warning_shown_)
    {
      calibration_conflict_warning_shown_ = true;
      QMessageBox::warning(
        this,
        "platform_calibration",
        QString::fromStdString(
          conflict_reason +
          "\n\nClose the camera calibration launch/window, then click Refresh."));
    }
    else if (!calibration_conflict)
    {
      calibration_conflict_warning_shown_ = false;
    }

    std::string reason;
    const bool ready = node_->observedBoardReady(reason);
    save_button_->setEnabled(ready);
    if (steadyNow() < status_hold_until_)
    {
      return;
    }
    const auto lines = node_->statusLines(platform_name_->text().toStdString());
    status_->setPlainText(QString::fromStdString(joinLines(lines)));
  }

  void refreshImage()
  {
    const QImage image = node_->latestVisualizationQImage();
    if (image.isNull())
    {
      const std::string source = node_->useArucoOverlay()
        ? node_->overlayTopic()
        : node_->colorTopic();
      image_label_->clear();
      image_label_->setText(QString::fromStdString("no camera topics...\nWaiting for " + source + " ..."));
      return;
    }
    QPixmap pixmap = QPixmap::fromImage(image);
    if (pixmap.isNull())
    {
      return;
    }
    const QSize target_size = image_label_->size();
    if (target_size.width() > 1 && target_size.height() > 1)
    {
      pixmap = pixmap.scaled(target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }
    image_label_->setText("");
    image_label_->setPixmap(pixmap);
  }

  void savePlatform()
  {
    try
    {
      const auto path = node_->saveCalibration(platform_name_->text().toStdString());
      status_hold_until_ = steadyNow() + std::chrono::seconds(6);
      status_->setPlainText(QString::fromStdString("Saved platform calibration:\n" + path.string()));
      QTextCursor cursor = status_->textCursor();
      cursor.movePosition(QTextCursor::End);
      status_->setTextCursor(cursor);
      QMessageBox::information(this, "platform_calibration", QString::fromStdString("Saved:\n" + path.string()));
    }
    catch (const std::exception &ex)
    {
      QMessageBox::warning(this, "platform_calibration", QString::fromStdString(ex.what()));
    }
  }
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<PlatformCalibrationNode>();
  PlatformCalibrationWindow window(node);
  window.show();
  const int exit_code = app.exec();
  node.reset();
  rclcpp::shutdown();
  return exit_code;
}
