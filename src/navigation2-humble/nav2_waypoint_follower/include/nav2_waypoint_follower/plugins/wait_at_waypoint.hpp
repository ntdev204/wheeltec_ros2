













#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{



class WaitAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:


  WaitAtWaypoint();

  

  ~WaitAtWaypoint();

  

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:

  int waypoint_pause_duration_;
  bool is_enabled_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
};

}
#endif
