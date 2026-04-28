













#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/spin_action.hpp"

namespace nav2_behavior_tree
{

SpinAction::SpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("spin_dist", dist);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.target_yaw = dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  getInput("is_recovery", is_recovery_);
}

void SpinAction::on_tick()
{
  if (is_recovery_) {
    increment_recovery_count();
  }
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin", builder);
}
