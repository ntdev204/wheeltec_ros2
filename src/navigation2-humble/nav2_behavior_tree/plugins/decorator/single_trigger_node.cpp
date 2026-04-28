













#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/single_trigger_node.hpp"

namespace nav2_behavior_tree
{

SingleTrigger::SingleTrigger(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(true)
{
}

BT::NodeStatus SingleTrigger::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  if (first_time_) {
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        first_time_ = false;
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
        first_time_ = false;
        return BT::NodeStatus::FAILURE;

      default:
        first_time_ = false;
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SingleTrigger>("SingleTrigger");
}
