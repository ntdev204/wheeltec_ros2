












#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{



class InputAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:


  InputAtWaypoint();

  

  ~InputAtWaypoint();

  

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  

  void Cb(const std_msgs::msg::Empty::SharedPtr msg);

  bool input_received_;
  bool is_enabled_;
  rclcpp::Duration timeout_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};

}

#endif
