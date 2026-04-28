













#ifndef NAV2_COLLISION_MONITOR__SCAN_HPP_
#define NAV2_COLLISION_MONITOR__SCAN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{



class Scan : public Source
{
public:
  

  Scan(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  

  ~Scan();

  

  void configure();

  

  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  

  void dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);




  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr data_sub_;


  sensor_msgs::msg::LaserScan::ConstSharedPtr data_;
};

}

#endif
