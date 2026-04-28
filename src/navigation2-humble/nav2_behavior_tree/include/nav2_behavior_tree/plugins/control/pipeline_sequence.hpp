













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{



class PipelineSequence : public BT::ControlNode
{
public:
  

  explicit PipelineSequence(const std::string & name);

  

  PipelineSequence(const std::string & name, const BT::NodeConfiguration & config);

  

  void halt() override;

  

  static BT::PortsList providedPorts() {return {};}

protected:
  

  BT::NodeStatus tick() override;

  std::size_t last_child_ticked_ = 0;
};
}

#endif
