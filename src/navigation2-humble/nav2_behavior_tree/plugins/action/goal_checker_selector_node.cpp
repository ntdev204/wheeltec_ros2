














#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/goal_checker_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

GoalCheckerSelector::GoalCheckerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic_name", topic_name_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  goal_checker_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_, qos, std::bind(&GoalCheckerSelector::callbackGoalCheckerSelect, this, _1));
}

BT::NodeStatus GoalCheckerSelector::tick()
{
  rclcpp::spin_some(node_);






  if (last_selected_goal_checker_.empty()) {
    std::string default_goal_checker;
    getInput("default_goal_checker", default_goal_checker);
    if (default_goal_checker.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_goal_checker_ = default_goal_checker;
    }
  }

  setOutput("selected_goal_checker", last_selected_goal_checker_);

  return BT::NodeStatus::SUCCESS;
}

void
GoalCheckerSelector::callbackGoalCheckerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_goal_checker_ = msg->data;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalCheckerSelector>("GoalCheckerSelector");
}
