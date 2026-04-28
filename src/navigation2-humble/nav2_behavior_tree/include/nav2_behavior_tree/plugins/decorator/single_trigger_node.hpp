













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SINGLE_TRIGGER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SINGLE_TRIGGER_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{



class SingleTrigger : public BT::DecoratorNode
{
public:
  

  SingleTrigger(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  

  BT::NodeStatus tick() override;

  bool first_time_;
};

}

#endif
