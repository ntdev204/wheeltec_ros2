













#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/assisted_teleop_action.hpp"

namespace nav2_behavior_tree
{

AssistedTeleopAction::AssistedTeleopAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf)
{
  double time_allowance;
  getInput("time_allowance", time_allowance);
  getInput("is_recovery", is_recovery_);


  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void AssistedTeleopAction::on_tick()
{
  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus AssistedTeleopAction::on_aborted()
{
  return is_recovery_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::AssistedTeleopAction>(
        name, "assisted_teleop", config);
    };

  factory.registerBuilder<nav2_behavior_tree::AssistedTeleopAction>("AssistedTeleop", builder);
}
