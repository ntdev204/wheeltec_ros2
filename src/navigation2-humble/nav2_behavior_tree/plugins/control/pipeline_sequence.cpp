













#include <stdexcept>
#include <sstream>
#include <string>

#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"

namespace nav2_behavior_tree
{

PipelineSequence::PipelineSequence(const std::string & name)
: BT::ControlNode(name, {})
{
}

PipelineSequence::PipelineSequence(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus PipelineSequence::tick()
{
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
        ControlNode::haltChildren();
        last_child_ticked_ = 0;
        return status;
      case BT::NodeStatus::SUCCESS:


        break;
      case BT::NodeStatus::RUNNING:
        if (i >= last_child_ticked_) {
          last_child_ticked_ = i;
          return status;
        }

        break;
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status <<
          "from child " << children_nodes_[i]->name();
        throw std::runtime_error(error_msg.str());
    }
  }

  ControlNode::haltChildren();
  last_child_ticked_ = 0;
  return BT::NodeStatus::SUCCESS;
}

void PipelineSequence::halt()
{
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
}
