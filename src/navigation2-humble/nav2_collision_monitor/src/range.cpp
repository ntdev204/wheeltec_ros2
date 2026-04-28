













#include "nav2_collision_monitor/range.hpp"

#include <math.h>
#include <cmath>
#include <functional>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Range::Range(
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
  RCLCPP_INFO(logger_, "[%s]: Creating Range", source_name_.c_str());
}

Range::~Range()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Range", source_name_.c_str());
  data_sub_.reset();
}

void Range::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS range_qos = rclcpp::SensorDataQoS();
  data_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
    source_topic, range_qos,
    std::bind(&Range::dataCallback, this, std::placeholders::_1));
}

void Range::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{


  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }


  if (data_->range < data_->min_range || data_->range > data_->max_range) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Data range %fm is out of {%f..%f} sensor span. Ignoring...",
      source_name_.c_str(), data_->range, data_->min_range, data_->max_range);
    return;
  }



  tf2::Transform tf_transform;
  if (!getTransform(data_->header.frame_id, data_->header.stamp, curr_time, tf_transform)) {
    return;
  }


  float angle;
  for (
    angle = -data_->field_of_view / 2;
    angle < data_->field_of_view / 2;
    angle += obstacles_angle_)
  {

    tf2::Vector3 p_v3_s(
      data_->range * std::cos(angle),
      data_->range * std::sin(angle),
      0.0);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;


    data.push_back({p_v3_b.x(), p_v3_b.y()});
  }


  angle = data_->field_of_view / 2;


  tf2::Vector3 p_v3_s(
    data_->range * std::cos(angle),
    data_->range * std::sin(angle),
    0.0);
  tf2::Vector3 p_v3_b = tf_transform * p_v3_s;


  data.push_back({p_v3_b.x(), p_v3_b.y()});
}

void Range::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 180));
  obstacles_angle_ = node->get_parameter(source_name_ + ".obstacles_angle").as_double();
}

void Range::dataCallback(sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  data_ = msg;
}

}
