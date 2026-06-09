#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace obstacle_perception
{

struct MemoryVoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const MemoryVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct MemoryVoxelHash
{
  std::size_t operator()(const MemoryVoxelKey & key) const noexcept
  {
    const std::size_t h1 = std::hash<int>{}(key.x);
    const std::size_t h2 = std::hash<int>{}(key.y);
    const std::size_t h3 = std::hash<int>{}(key.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

struct MemoryVoxel
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double sum_r{0.0};
  double sum_g{0.0};
  double sum_b{0.0};
  int count{0};
  bool has_color{false};
  rclcpp::Time last_update;
};

class ObstacleMemoryNode : public rclcpp::Node
{
public:
  explicit ObstacleMemoryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishMemory();
  void prune(double now_sec);

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string frame_id_override_;
  double voxel_size_{0.05};
  double decay_seconds_{0.0};
  int max_voxels_{200000};
  double publish_rate_{5.0};
  int min_hits_{5};
  std::string target_frame_;
  uint8_t color_r_{0};
  uint8_t color_g_{100};
  uint8_t color_b_{255};
  bool skip_if_live_{true};
  double blue_tint_{0.3};
  bool frustum_enable_{true};
  std::string frustum_frame_{"arm_calibrated_camera_link"};
  double frustum_near_{0.1};
  double frustum_far_{3.0};
  double frustum_hfov_deg_{70.0};
  double frustum_vfov_deg_{55.0};

  std::unordered_map<MemoryVoxelKey, MemoryVoxel, MemoryVoxelHash> voxels_;
  std::string last_frame_id_;
  std::unordered_set<MemoryVoxelKey, MemoryVoxelHash> live_voxels_;
  tf2::Vector3 live_cam_origin_{0.0, 0.0, 0.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace obstacle_perception
