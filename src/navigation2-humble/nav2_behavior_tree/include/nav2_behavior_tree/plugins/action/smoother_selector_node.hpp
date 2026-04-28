














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTHER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTHER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{



class SmootherSelector : public BT::SyncActionNode
{
public:
  

  SmootherSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_smoother",
        "the default smoother to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "smoother_selector",
        "the input topic name to select the smoother"),

      BT::OutputPort<std::string>(
        "selected_smoother",
        "Selected smoother by subscription")
    };
  }

private:
  

  BT::NodeStatus tick() override;

  

  void callbackSmootherSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr smoother_selector_sub_;

  std::string last_selected_smoother_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}

#endif
