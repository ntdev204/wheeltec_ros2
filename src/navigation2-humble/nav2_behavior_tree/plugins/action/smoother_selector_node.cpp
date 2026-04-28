















#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/smoother_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

SmootherSelector::SmootherSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("topic_name", topic_name_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  smoother_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&SmootherSelector::callbackSmootherSelect, this, _1),
    sub_option);
}

BT::NodeStatus SmootherSelector::tick()
{
  callback_group_executor_.spin_some();






  if (last_selected_smoother_.empty()) {
    std::string default_smoother;
    getInput("default_smoother", default_smoother);
    if (default_smoother.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_smoother_ = default_smoother;
    }
  }

  setOutput("selected_smoother", last_selected_smoother_);

  return BT::NodeStatus::SUCCESS;
}

void
SmootherSelector::callbackSmootherSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_smoother_ = msg->data;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SmootherSelector>("SmootherSelector");
}
