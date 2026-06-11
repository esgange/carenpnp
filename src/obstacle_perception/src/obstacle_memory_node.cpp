#include "obstacle_perception/obstacle_memory_node.hpp"

#include <algorithm>
#include <cmath>

namespace obstacle_perception
{

ObstacleMemoryNode::ObstacleMemoryNode(const rclcpp::NodeOptions & options)
: Node("obstacle_memory", options)
{
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/obstacles/points");
  output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/obstacles/memory_points");
  frame_id_override_ = declare_parameter<std::string>("frame_id_override", "");
  voxel_size_ = declare_parameter<double>("voxel_size", 0.03);
  decay_seconds_ = declare_parameter<double>("decay_seconds", 0.0);
  max_voxels_ = declare_parameter<int>("max_voxels", 400000);
  publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
  min_hits_ = declare_parameter<int>("min_hits", 30);
  target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  skip_if_live_ = declare_parameter<bool>("skip_if_live", true);
  blue_tint_ = declare_parameter<double>("blue_tint", 0.02);
  frustum_enable_ = declare_parameter<bool>("frustum_enable", true);
  frustum_frame_ = declare_parameter<std::string>("frustum_frame", "arm_calibrated_camera_link");
  frustum_near_ = declare_parameter<double>("frustum_near", 0.1);
  frustum_far_ = declare_parameter<double>("frustum_far", 3.0);
  frustum_hfov_deg_ = declare_parameter<double>("frustum_hfov_deg", 65.0);
  frustum_vfov_deg_ = declare_parameter<double>("frustum_vfov_deg", 50.0);
  color_r_ = static_cast<uint8_t>(std::clamp<int>(declare_parameter<int>("color_r", 0), 0, 255));
  color_g_ = static_cast<uint8_t>(std::clamp<int>(declare_parameter<int>("color_g", 100), 0, 255));
  color_b_ = static_cast<uint8_t>(std::clamp<int>(declare_parameter<int>("color_b", 255), 0, 255));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleMemoryNode::cloudCallback, this, std::placeholders::_1));

  auto period_ms = static_cast<int>(1000.0 / std::max(1e-3, publish_rate_));
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&ObstacleMemoryNode::publishMemory, this));

  RCLCPP_INFO(
    get_logger(),
    "Obstacle memory listening to %s, publishing %s; voxel_size=%.3f decay=%.1fs max_voxels=%d min_hits=%d target_frame=%s",
    input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), voxel_size_, decay_seconds_, max_voxels_, min_hits_, target_frame_.c_str());
}

void ObstacleMemoryNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto now = msg->header.stamp;
  const std::string source_frame = msg->header.frame_id;
  const std::string target_frame = frame_id_override_.empty() ? target_frame_ : frame_id_override_;
  geometry_msgs::msg::TransformStamped tf_msg;
  tf2::Transform tf_transform;
  bool have_tf = false;
  live_voxels_.clear();
  if (target_frame.empty() || target_frame == source_frame) {
    have_tf = true;
    tf_transform.setIdentity();
    last_frame_id_ = source_frame;
    live_cam_origin_.setValue(0.0, 0.0, 0.0);
  } else {
    try {
      tf_msg = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      tf_transform.setOrigin(tf2::Vector3(
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z));
      tf_transform.setRotation(tf2::Quaternion(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w));
      have_tf = true;
      last_frame_id_ = target_frame;
      live_cam_origin_.setValue(
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF lookup failed %s->%s: %s", target_frame.c_str(), source_frame.c_str(), ex.what());
      return;
    }
  }

  if (!have_tf) {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  bool has_rgb = false;
  for (const auto & field : msg->fields) {
    if (field.name == "rgb") {
      has_rgb = true;
      break;
    }
  }
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_r;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_g;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_b;
  if (has_rgb) {
    iter_r = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*msg, "r");
    iter_g = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*msg, "g");
    iter_b = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*msg, "b");
  }

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    tf2::Vector3 p(*iter_x, *iter_y, *iter_z);
    uint8_t r = color_r_;
    uint8_t g = color_g_;
    uint8_t b = color_b_;
    if (has_rgb) {
      r = **iter_r;
      g = **iter_g;
      b = **iter_b;
    }
    const tf2::Vector3 pt = tf_transform * p;
    const double x = pt.x();
    const double y = pt.y();
    const double z = pt.z();

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    MemoryVoxelKey key{
      static_cast<int>(std::floor(x / voxel_size_)),
      static_cast<int>(std::floor(y / voxel_size_)),
      static_cast<int>(std::floor(z / voxel_size_))};

    live_voxels_.insert(key);
    auto & voxel = voxels_[key];
    if (voxel.count == 0) {
      voxel.x = x;
      voxel.y = y;
      voxel.z = z;
      voxel.count = 1;
      voxel.sum_r = static_cast<double>(r);
      voxel.sum_g = static_cast<double>(g);
      voxel.sum_b = static_cast<double>(b);
      voxel.has_color = has_rgb;
    } else {
      const double inv = 1.0 / static_cast<double>(voxel.count + 1);
      voxel.x = (voxel.x * voxel.count + x) * inv;
      voxel.y = (voxel.y * voxel.count + y) * inv;
      voxel.z = (voxel.z * voxel.count + z) * inv;
      voxel.count += 1;
      voxel.sum_r += static_cast<double>(r);
      voxel.sum_g += static_cast<double>(g);
      voxel.sum_b += static_cast<double>(b);
      voxel.has_color = voxel.has_color || has_rgb;
    }
    voxel.last_update = now;
    if (has_rgb) {
      ++(*iter_r);
      ++(*iter_g);
      ++(*iter_b);
    }

  }

  if (static_cast<int>(voxels_.size()) > max_voxels_) {
    // Drop oldest entries when over capacity.
    std::vector<std::pair<MemoryVoxelKey, rclcpp::Time>> entries;
    entries.reserve(voxels_.size());
    for (const auto & kv : voxels_) {
      entries.emplace_back(kv.first, kv.second.last_update);
    }
    std::sort(entries.begin(), entries.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;});
    const int to_remove = static_cast<int>(voxels_.size()) - max_voxels_;
    for (int i = 0; i < to_remove && i < static_cast<int>(entries.size()); ++i) {
      voxels_.erase(entries[i].first);
    }
  }
}

void ObstacleMemoryNode::prune(double now_sec)
{
  if (decay_seconds_ <= 0.0) {
    return;
  }
  std::vector<MemoryVoxelKey> to_erase;
  to_erase.reserve(voxels_.size() / 4);
  for (const auto & kv : voxels_) {
    const double age = now_sec - rclcpp::Time(kv.second.last_update).seconds();
    if (age > decay_seconds_) {
      to_erase.push_back(kv.first);
    }
  }
  for (const auto & key : to_erase) {
    voxels_.erase(key);
  }
}

void ObstacleMemoryNode::publishMemory()
{
  if (voxels_.empty() || !cloud_pub_) {
    return;
  }

  const double now_sec = this->now().seconds();
  prune(now_sec);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = this->now();
  cloud.header.frame_id = last_frame_id_.empty() ? "arm_calibrated_camera_link" : last_frame_id_;
  cloud.height = 1;
  cloud.is_dense = false;

  const size_t n = std::count_if(
    voxels_.begin(), voxels_.end(),
    [this](const auto & kv) {return kv.second.count >= min_hits_;});
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(n);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

  tf2::Transform frustum_tf;
  bool have_frustum_tf = false;
  if (frustum_enable_ && !frustum_frame_.empty()) {
    try {
      auto tf = tf_buffer_->lookupTransform(frustum_frame_, last_frame_id_, tf2::TimePointZero);
      frustum_tf.setOrigin(tf2::Vector3(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z));
      frustum_tf.setRotation(tf2::Quaternion(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w));
      have_frustum_tf = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Frustum TF lookup failed %s->%s: %s", frustum_frame_.c_str(), last_frame_id_.c_str(), ex.what());
    }
  }
  const double tan_h = std::tan(frustum_hfov_deg_ * M_PI / 360.0);
  const double tan_v = std::tan(frustum_vfov_deg_ * M_PI / 360.0);

  for (const auto & kv : voxels_) {
    if (kv.second.count < min_hits_) {
      continue;
    }
    if (skip_if_live_ && live_voxels_.find(kv.first) != live_voxels_.end()) {
      continue;
    }
    if (frustum_enable_ && have_frustum_tf) {
      tf2::Vector3 p(kv.second.x, kv.second.y, kv.second.z);
      const tf2::Vector3 pf = frustum_tf * p;
      const double z = pf.z();
      if (z > frustum_near_ && z < frustum_far_) {
        const double ax = std::abs(pf.x());
        const double ay = std::abs(pf.y());
        if (ax <= z * tan_h && ay <= z * tan_v) {
          continue;
        }
      }
    }
    const auto & v = kv.second;
    *iter_x = static_cast<float>(v.x);
    *iter_y = static_cast<float>(v.y);
    *iter_z = static_cast<float>(v.z);
    uint8_t r_out = color_r_;
    uint8_t g_out = color_g_;
    uint8_t b_out = color_b_;
    if (v.has_color && v.count > 0) {
      const double inv = 1.0 / static_cast<double>(v.count);
      r_out = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(v.sum_r * inv), 0, 255));
      g_out = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(v.sum_g * inv), 0, 255));
      b_out = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(v.sum_b * inv), 0, 255));
    }
    // Blend with a blue tint
    const double tint = std::clamp(blue_tint_, 0.0, 1.0);
    r_out = static_cast<uint8_t>(std::clamp<int>(
      static_cast<int>(r_out * (1.0 - tint) + 0.0 * tint), 0, 255));
    g_out = static_cast<uint8_t>(std::clamp<int>(
      static_cast<int>(g_out * (1.0 - tint) + 0.0 * tint), 0, 255));
    b_out = static_cast<uint8_t>(std::clamp<int>(
      static_cast<int>(b_out * (1.0 - tint) + 255.0 * tint), 0, 255));

    *iter_r = r_out;
    *iter_g = g_out;
    *iter_b = b_out;
    ++iter_x; ++iter_y; ++iter_z;
    ++iter_r; ++iter_g; ++iter_b;
  }

  cloud.width = static_cast<uint32_t>(n);
  cloud_pub_->publish(cloud);
}

}  // namespace obstacle_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<obstacle_perception::ObstacleMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
