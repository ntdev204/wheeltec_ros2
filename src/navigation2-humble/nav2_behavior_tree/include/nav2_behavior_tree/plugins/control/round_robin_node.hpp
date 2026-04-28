













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{



class RoundRobinNode : public BT::ControlNode
{
public:
  

  explicit RoundRobinNode(const std::string & name);

  

  RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config);

  

  BT::NodeStatus tick() override;

  

  void halt() override;

  

  static BT::PortsList providedPorts() {return {};}

private:
  unsigned int current_child_idx_{0};
  unsigned int num_failed_children_{0};
};

}

#endif
