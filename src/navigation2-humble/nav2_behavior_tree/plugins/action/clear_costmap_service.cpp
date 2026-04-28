













#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

namespace nav2_behavior_tree
{

ClearEntireCostmapService::ClearEntireCostmapService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name, conf)
{
}

void ClearEntireCostmapService::on_tick()
{
  increment_recovery_count();
}

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>(service_node_name, conf)
{
}

void ClearCostmapExceptRegionService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>(service_node_name, conf)
{
}

void ClearCostmapAroundRobotService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
    "ClearCostmapExceptRegion");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapAroundRobotService>(
    "ClearCostmapAroundRobot");
}
