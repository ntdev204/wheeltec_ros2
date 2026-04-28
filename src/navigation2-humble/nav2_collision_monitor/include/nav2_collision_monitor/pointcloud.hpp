













#ifndef NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_
#define NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_

#include <memory>
#include <vector>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{



class PointCloud : public Source
{
public:
  

  PointCloud(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  

  ~PointCloud();

  

  void configure();

  

  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  

  void getParameters(std::string & source_topic);

  

  void dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);




  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr data_sub_;


  double min_height_, max_height_;


  sensor_msgs::msg::PointCloud2::ConstSharedPtr data_;
};

}

#endif
