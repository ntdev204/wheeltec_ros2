













#include "nav2_collision_monitor/scan.hpp"

#include <cmath>
#include <functional>

namespace nav2_collision_monitor
{

Scan::Scan(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;


  getCommonParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

void Scan::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{


  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }



  tf2::Transform tf_transform;
  if (!getTransform(data_->header.frame_id, data_->header.stamp, curr_time, tf_transform)) {
    return;
  }


  float angle = data_->angle_min;
  for (size_t i = 0; i < data_->ranges.size(); i++) {
    if (data_->ranges[i] >= data_->range_min && data_->ranges[i] <= data_->range_max) {

      tf2::Vector3 p_v3_s(
        data_->ranges[i] * std::cos(angle),
        data_->ranges[i] * std::sin(angle),
        0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;


      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += data_->angle_increment;
  }
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  data_ = msg;
}

}
