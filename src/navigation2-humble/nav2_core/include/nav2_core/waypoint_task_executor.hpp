














#ifndef NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_core
{


class WaypointTaskExecutor
{
public:
  

  WaypointTaskExecutor() {}

  

  virtual ~WaypointTaskExecutor() {}

  

  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  

  virtual bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) = 0;
};
}
#endif
