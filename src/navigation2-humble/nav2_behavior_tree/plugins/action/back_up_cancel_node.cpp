













#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/back_up_cancel_node.hpp"

namespace nav2_behavior_tree
{

BackUpCancel::BackUpCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::BackUpCancel>(
        name, "backup", config);
    };

  factory.registerBuilder<nav2_behavior_tree::BackUpCancel>(
    "CancelBackUp", builder);
}
