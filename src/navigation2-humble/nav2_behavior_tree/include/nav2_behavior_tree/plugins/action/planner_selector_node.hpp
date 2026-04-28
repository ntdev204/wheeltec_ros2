














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{



class PlannerSelector : public BT::SyncActionNode
{
public:
  

  PlannerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_planner",
        "the default planner to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "planner_selector",
        "the input topic name to select the planner"),

      BT::OutputPort<std::string>(
        "selected_planner",
        "Selected planner by subscription")
    };
  }

private:
  

  BT::NodeStatus tick() override;

  

  void callbackPlannerSelect(const std_msgs::msg::String::SharedPtr msg);


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_selector_sub_;

  std::string last_selected_planner_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}

#endif
