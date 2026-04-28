














#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{



class ControllerSelector : public BT::SyncActionNode
{
public:
  

  ControllerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_controller",
        "the default controller to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "controller_selector",
        "the input topic name to select the controller"),

      BT::OutputPort<std::string>(
        "selected_controller",
        "Selected controller by subscription")
    };
  }

private:
  

  BT::NodeStatus tick() override;

  

  void callbackControllerSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_selector_sub_;

  std::string last_selected_controller_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}

#endif
