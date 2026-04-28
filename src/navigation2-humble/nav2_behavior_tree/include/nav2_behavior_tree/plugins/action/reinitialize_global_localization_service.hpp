













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REINITIALIZE_GLOBAL_LOCALIZATION_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REINITIALIZE_GLOBAL_LOCALIZATION_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_behavior_tree
{



class ReinitializeGlobalLocalizationService : public BtServiceNode<std_srvs::srv::Empty>
{
public:
  

  ReinitializeGlobalLocalizationService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);
};

}

#endif
