#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <aruco_perception/msg/marker_detections.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace aruco_perception
{
class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  struct MarkerPose
  {
    int id;
    std::array<Eigen::Vector3d, 4> cam_corners;
    Eigen::Vector3d center_cam;
    Eigen::Isometry3d pose;
    std::vector<cv::Point2f> image_corners;
  };

  void colorCallback(const ImageMsg::ConstSharedPtr msg);
  void depthCallback(const ImageMsg::ConstSharedPtr msg);
  void cameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg);
  void tryProcessFrame();
  void processFrame(const ImageMsg::ConstSharedPtr &color,
                    const ImageMsg::ConstSharedPtr &depth,
                    const CameraInfoMsg::ConstSharedPtr &info);
  void publishDetections(const rclcpp::Time &image_stamp,
                         const ImageMsg &color,
                         const std::vector<MarkerPose> &markers);
  void publishOverlay(const rclcpp::Time &stamp, const cv::Mat &color_bgr,
                      const cv::Mat &depth_image, const CameraInfoMsg &info,
                      const std::vector<MarkerPose> &markers);
  void renderNoCameraTopicsOverlay();
  static void onMouseThunk(int event, int x, int y, int flags, void *userdata);
  void onMouse(int event, int x, int y, int flags);
  void processPendingReset();
  void resetDetectorState();

  std::optional<double> depthAt(const cv::Mat &depth, int u, int v) const;
  std::optional<Eigen::Vector3d> centerPointFromDepth(
    const std::vector<cv::Point2f> &corners, const cv::Mat &depth,
    const CameraInfoMsg &info) const;
  std::optional<std::array<Eigen::Vector3d, 4>> cornersToPoints(
    const std::vector<cv::Point2f> &corners, const cv::Mat &depth,
    const CameraInfoMsg &info) const;
  Eigen::Vector3d projectPixel(double u, double v, double depth,
                               const CameraInfoMsg &info) const;
  std::optional<Eigen::Isometry3d> estimatePoseFrom3D(
    const std::array<Eigen::Vector3d, 4> &cam_points,
    const std::optional<Eigen::Vector3d> &center_override) const;
  geometry_msgs::msg::PoseStamped toPoseMsg(const Eigen::Isometry3d &pose,
                                            const rclcpp::Time &stamp) const;
  cv::Point2f projectPointToPixel(const Eigen::Vector3d &pt,
                                  const CameraInfoMsg &info) const;
  cv::Mat filterDepth(const cv::Mat &depth_image_raw);
  bool loadCalibrationFromFile(const std::string &path, Eigen::Quaterniond &q,
                               Eigen::Vector3d &t, std::string &reason) const;
  void publishCalibrationTransform();

  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<aruco_perception::msg::MarkerDetections>::SharedPtr detections_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  bool has_info_{false};
  cv::Mat camera_matrix_;

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string marker_frame_prefix_;
  std::string camera_frame_id_;
  bool use_calibration_{true};
  bool publish_static_calibration_tf_{true};
  std::string calibration_file_;
  std::string calibration_parent_frame_;
  std::string calibration_child_frame_;
  Eigen::Quaterniond calibration_rotation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d calibration_translation_{Eigen::Vector3d::Zero()};
  double sync_tolerance_sec_;
  int target_marker_id_;
  int depth_average_kernel_;

  bool publish_viz_{true};
  bool publish_overlay_{true};
  bool show_overlay_window_{true};
  double depth_colormap_max_{1.5};
  std::string overlay_topic_;
  std::string detections_topic_;
  double overlay_rate_hz_{10.0};
  rclcpp::Time last_overlay_stamp_{0, 0, RCL_ROS_TIME};

  // Tunable visualization parameters
  double overlay_axis_scale_{0.45};     // fraction of marker edge length
  double overlay_axis_min_len_{0.015};  // meters

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::TimerBase::SharedPtr camera_status_timer_;
  std::chrono::steady_clock::time_point last_overlay_render_time_;

  mutable std::mutex data_mutex_;
  ImageMsg::ConstSharedPtr last_color_;
  ImageMsg::ConstSharedPtr last_depth_;
  CameraInfoMsg::ConstSharedPtr last_info_;
  cv::Rect reset_button_rect_{12, 12, 170, 40};
  bool reset_button_pressed_{false};
  bool reset_requested_{false};
};
}  // namespace aruco_perception
