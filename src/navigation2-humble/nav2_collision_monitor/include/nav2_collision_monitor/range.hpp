













#ifndef NAV2_COLLISION_MONITOR__RANGE_HPP_
#define NAV2_COLLISION_MONITOR__RANGE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "sensor_msgs/msg/range.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{



class Range : public Source
{
public:
  

  Range(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  

  ~Range();

  

  void configure();

  

  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  

  void getParameters(std::string & source_topic);

  

  void dataCallback(sensor_msgs::msg::Range::ConstSharedPtr msg);




  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr data_sub_;


  double obstacles_angle_;


  sensor_msgs::msg::Range::ConstSharedPtr data_;
};

}

#endif
