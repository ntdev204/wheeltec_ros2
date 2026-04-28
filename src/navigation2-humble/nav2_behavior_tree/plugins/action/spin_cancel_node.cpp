













#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/spin_cancel_node.hpp"

namespace nav2_behavior_tree
{

SpinCancel::SpinCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SpinCancel>(
        name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinCancel>(
    "CancelSpin", builder);
}
