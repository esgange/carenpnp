#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <unordered_set>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace obstacle_perception
{

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    const std::size_t h1 = std::hash<int>{}(key.x);
    const std::size_t h2 = std::hash<int>{}(key.y);
    const std::size_t h3 = std::hash<int>{}(key.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

struct VoxelAccumulator
{
  double sum_x{0.0};
  double sum_y{0.0};
  double sum_z{0.0};
  int count{0};
};

class ObstaclePerceptionNode : public rclcpp::Node
{
public:
  explicit ObstaclePerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void depthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  bool depthToMeters(
    const cv::Mat & depth,
    const std::string & encoding,
    int u,
    int v,
    float & depth_m) const;

  void publishPointCloud(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    const std::vector<std::array<float, 3>> & points,
    const std::vector<std::array<uint8_t, 3>> & colors,
    bool has_color);

  void publishMarkers(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    const std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> & voxels);
  void cameraStatusCallback();

  image_geometry::PinholeCameraModel cam_model_;

  const std::string output_frame_{"calibrated_camera_link"};
  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  double voxel_size_{0.05};
  double min_range_{0.15};
  double max_range_{2.5};
  int pixel_stride_{4};
  int min_points_per_voxel_{3};
  double marker_lifetime_{0.4};
  bool publish_pointcloud_{true};
  bool publish_markers_{true};
  bool use_color_{true};
  bool filter_floating_{true};
  int neighbor_radius_voxels_{1};
  int min_neighbor_voxels_{2};

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr camera_status_timer_;
  std::chrono::steady_clock::time_point last_camera_frame_time_;
};

}  // namespace obstacle_perception
