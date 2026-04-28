













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_behavior_tree
{



class IsStuckCondition : public BT::ConditionNode
{
public:
  

  IsStuckCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsStuckCondition() = delete;

  

  ~IsStuckCondition() override;

  

  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg);

  

  BT::NodeStatus tick() override;

  

  void logStuck(const std::string & msg) const;

  

  void updateStates();

  

  bool isStuck();

  

  static BT::PortsList providedPorts() {return {};}

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::atomic<bool> is_stuck_;


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;


  double current_accel_;


  double brake_accel_limit_;
};

}

#endif
