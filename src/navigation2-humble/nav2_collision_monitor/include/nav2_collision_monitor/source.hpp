













#ifndef NAV2_COLLISION_MONITOR__SOURCE_HPP_
#define NAV2_COLLISION_MONITOR__SOURCE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{



class Source
{
public:
  

  Source(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  

  virtual ~Source();

  

  virtual void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const = 0;

protected:
  

  void getCommonParameters(std::string & source_topic);

  

  bool sourceValid(
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time) const;

  

  bool getTransform(
    const std::string & source_frame_id,
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time,
    tf2::Transform & tf_transform) const;




  nav2_util::LifecycleNode::WeakPtr node_;

  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};



  std::string source_name_;



  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string base_frame_id_;

  std::string global_frame_id_;

  tf2::Duration transform_tolerance_;

  rclcpp::Duration source_timeout_;
};

}

#endif
