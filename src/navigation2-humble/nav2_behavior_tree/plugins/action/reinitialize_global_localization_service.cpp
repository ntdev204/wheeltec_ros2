













#include <string>
#include "nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.hpp"

namespace nav2_behavior_tree
{

ReinitializeGlobalLocalizationService::ReinitializeGlobalLocalizationService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<std_srvs::srv::Empty>(service_node_name, conf)
{}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ReinitializeGlobalLocalizationService>(
    "ReinitializeGlobalLocalization");
}
