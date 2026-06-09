#include "obstacle_perception/obstacle_perception_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <rmw/qos_profiles.h>

namespace obstacle_perception
{

ObstaclePerceptionNode::ObstaclePerceptionNode(const rclcpp::NodeOptions & options)
: Node("obstacle_perception", options)
{
  // Tunable parameters
  voxel_size_ = declare_parameter<double>("voxel_size", 0.01);
  min_range_ = declare_parameter<double>("min_range", 0.2);
  max_range_ = declare_parameter<double>("max_range", 2.5);
  pixel_stride_ = declare_parameter<int>("pixel_stride", 2);
  min_points_per_voxel_ = declare_parameter<int>("min_points_per_voxel", 1);
  marker_lifetime_ = declare_parameter<double>("marker_lifetime", 20.0);
  publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
  publish_markers_ = declare_parameter<bool>("publish_markers", true);
  use_color_ = declare_parameter<bool>("use_color", true);
  filter_floating_ = declare_parameter<bool>("filter_floating", true);
  neighbor_radius_voxels_ = declare_parameter<int>("neighbor_radius_voxels", 1);
  min_neighbor_voxels_ = declare_parameter<int>("min_neighbor_voxels", 10);
  color_topic_ =
    declare_parameter<std::string>("color_topic", "/robot_camera/color/image_raw");
  depth_topic_ =
    declare_parameter<std::string>("depth_topic", "/robot_camera/depth/image_raw");
  camera_info_topic_ =
    declare_parameter<std::string>("camera_info_topic", "/robot_camera/color/camera_info");

  if (voxel_size_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "voxel_size must be positive, defaulting to 0.05 m");
    voxel_size_ = 0.05;
  }
  pixel_stride_ = std::max(1, pixel_stride_);
  min_points_per_voxel_ = std::max(1, min_points_per_voxel_);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacles/points", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacles/markers", 10);

  color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
  info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), color_sub_, depth_sub_, info_sub_);
  sync_->registerCallback(std::bind(
    &ObstaclePerceptionNode::depthCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  camera_status_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ObstaclePerceptionNode::cameraStatusCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "Obstacle perception listening to %s, %s, %s; publishing in frame [%s]",
    color_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(), output_frame_.c_str());
}

void ObstaclePerceptionNode::depthCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  last_camera_frame_time_ = std::chrono::steady_clock::now();

  if (depth_msg->height == 0 || depth_msg->width == 0) {
    return;
  }

  cam_model_.fromCameraInfo(info_msg);
  if (cam_model_.fx() <= 0.0 || cam_model_.fy() <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Camera intrinsics invalid (fx or fy is zero), skipping frame");
    return;
  }
  const std::string frame_id = output_frame_;

  cv_bridge::CvImageConstPtr cv_ptr;
  cv_bridge::CvImageConstPtr cv_color;
  try {
    cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cv_bridge failed to convert depth image: %s", e.what());
    return;
  }
  if (use_color_) {
    try {
      cv_color = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cv_bridge failed to convert color image: %s", e.what());
      return;
    }
  }

  const cv::Mat & depth = cv_ptr->image;
  const int height = depth.rows;
  const int width = depth.cols;

  std::vector<std::array<float, 3>> points;
  points.reserve((height / pixel_stride_) * (width / pixel_stride_));
  std::vector<std::array<uint8_t, 3>> colors;
  const bool color_ready = use_color_ && cv_color;
  if (color_ready) {
    colors.reserve(points.capacity());
  }

  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
  voxels.reserve(points.capacity());

  for (int v = 0; v < height; v += pixel_stride_) {
    for (int u = 0; u < width; u += pixel_stride_) {
      float depth_m = 0.0f;
      if (!depthToMeters(depth, depth_msg->encoding, u, v, depth_m)) {
        continue;
      }

      if (depth_m < min_range_ || depth_m > max_range_) {
        continue;
      }

      const double x = (static_cast<double>(u) - cam_model_.cx()) * depth_m / cam_model_.fx();
      const double y = (static_cast<double>(v) - cam_model_.cy()) * depth_m / cam_model_.fy();
      const double z = depth_m;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.0) {
        continue;
      }

      points.push_back(
        {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});

      if (color_ready) {
        const cv::Vec3b bgr = cv_color->image.at<cv::Vec3b>(v, u);
        colors.push_back({bgr[2], bgr[1], bgr[0]});  // store as RGB
      }

      VoxelKey key{
        static_cast<int>(std::floor(x / voxel_size_)),
        static_cast<int>(std::floor(y / voxel_size_)),
        static_cast<int>(std::floor(z / voxel_size_))};

      auto & acc = voxels[key];
      acc.sum_x += x;
      acc.sum_y += y;
      acc.sum_z += z;
      acc.count += 1;
    }
  }

  const rclcpp::Time stamp = depth_msg->header.stamp;
  std::unordered_set<VoxelKey, VoxelKeyHash> valid_voxels;
  if (!filter_floating_ || voxels.empty()) {
    for (const auto & kv : voxels) {
      valid_voxels.insert(kv.first);
    }
  } else {
    const int r = std::max(1, neighbor_radius_voxels_);
    for (const auto & kv : voxels) {
      int neighbors = 0;
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dz = -r; dz <= r; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            VoxelKey nk{
              kv.first.x + dx,
              kv.first.y + dy,
              kv.first.z + dz};
            auto it = voxels.find(nk);
            if (it != voxels.end() && it->second.count >= min_points_per_voxel_) {
              neighbors++;
              if (neighbors >= min_neighbor_voxels_) {
                break;
              }
            }
          }
          if (neighbors >= min_neighbor_voxels_) {
            break;
          }
        }
        if (neighbors >= min_neighbor_voxels_) {
          break;
        }
      }
      if (neighbors >= min_neighbor_voxels_) {
        valid_voxels.insert(kv.first);
      }
    }
  }

  std::vector<std::array<float, 3>> filtered_points;
  std::vector<std::array<uint8_t, 3>> filtered_colors;
  filtered_points.reserve(points.size());
  if (color_ready) {
    filtered_colors.reserve(colors.size());
  }
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i];
    VoxelKey key{
      static_cast<int>(std::floor(p[0] / voxel_size_)),
      static_cast<int>(std::floor(p[1] / voxel_size_)),
      static_cast<int>(std::floor(p[2] / voxel_size_))};
    if (valid_voxels.find(key) == valid_voxels.end()) {
      continue;
    }
    filtered_points.push_back(p);
    if (color_ready && i < colors.size()) {
      filtered_colors.push_back(colors[i]);
    }
  }

  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> filtered_voxels;
  filtered_voxels.reserve(valid_voxels.size());
  for (const auto & kv : voxels) {
    if (valid_voxels.find(kv.first) != valid_voxels.end()) {
      filtered_voxels.emplace(kv.first, kv.second);
    }
  }

  publishPointCloud(stamp, frame_id, filtered_points, filtered_colors, color_ready);
  publishMarkers(stamp, frame_id, filtered_voxels);
}

bool ObstaclePerceptionNode::depthToMeters(
  const cv::Mat & depth,
  const std::string & encoding,
  int u,
  int v,
  float & depth_m) const
{
  if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    depth_m = depth.at<float>(v, u);
  } else if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    depth_m = static_cast<float>(depth.at<uint16_t>(v, u)) * 0.001f;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported depth encoding: %s", encoding.c_str());
    return false;
  }

  if (!std::isfinite(depth_m) || depth_m <= 0.0f) {
    return false;
  }

  return true;
}

void ObstaclePerceptionNode::cameraStatusCallback()
{
  const auto now_steady = std::chrono::steady_clock::now();
  if (last_camera_frame_time_.time_since_epoch().count() != 0 &&
    now_steady - last_camera_frame_time_ < std::chrono::seconds(2))
  {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "no camera topics... color=%s publishers=%zu depth=%s publishers=%zu info=%s publishers=%zu",
    color_topic_.c_str(), count_publishers(color_topic_),
    depth_topic_.c_str(), count_publishers(depth_topic_),
    camera_info_topic_.c_str(), count_publishers(camera_info_topic_));

  publishPointCloud(now(), output_frame_, {}, {}, true);

  if (!publish_markers_ || !marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker status;
  status.header.frame_id = output_frame_;
  status.header.stamp = now();
  status.ns = "camera_status";
  status.id = 0;
  status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status.action = visualization_msgs::msg::Marker::ADD;
  status.pose.position.x = 0.0;
  status.pose.position.y = 0.0;
  status.pose.position.z = 0.35;
  status.pose.orientation.w = 1.0;
  status.scale.z = 0.08;
  status.color.r = 1.0f;
  status.color.g = 0.82f;
  status.color.b = 0.0f;
  status.color.a = 1.0f;
  status.text = "no camera topics...";
  status.lifetime = rclcpp::Duration::from_seconds(1.5);
  array.markers.push_back(status);
  marker_pub_->publish(array);
}

void ObstaclePerceptionNode::publishPointCloud(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::vector<std::array<float, 3>> & points,
  const std::vector<std::array<uint8_t, 3>> & colors,
  bool has_color)
{
  if (!publish_pointcloud_ || !cloud_pub_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  if (has_color) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i];
    *iter_x = p[0];
    *iter_y = p[1];
    *iter_z = p[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
    if (has_color && i < colors.size()) {
      *iter_r = colors[i][0];
      *iter_g = colors[i][1];
      *iter_b = colors[i][2];
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }

  cloud.width = static_cast<uint32_t>(points.size());
  cloud_pub_->publish(cloud);
}

void ObstaclePerceptionNode::publishMarkers(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> & voxels)
{
  if (!publish_markers_ || !marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  int id = 0;
  for (const auto & entry : voxels) {
    const auto & acc = entry.second;
    if (acc.count < min_points_per_voxel_) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "obstacle_voxels";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    const double inv = 1.0 / static_cast<double>(acc.count);
    marker.pose.position.x = acc.sum_x * inv;
    marker.pose.position.y = acc.sum_y * inv;
    marker.pose.position.z = acc.sum_z * inv;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = voxel_size_;
    marker.scale.y = voxel_size_;
    marker.scale.z = voxel_size_;

    marker.color.r = 1.0f;
    marker.color.g = 0.35f;
    marker.color.b = 0.15f;
    marker.color.a = 0.6f;

    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    array.markers.push_back(marker);
  }

  marker_pub_->publish(array);
}

}  // namespace obstacle_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
  auto node = std::make_shared<obstacle_perception::ObstaclePerceptionNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
