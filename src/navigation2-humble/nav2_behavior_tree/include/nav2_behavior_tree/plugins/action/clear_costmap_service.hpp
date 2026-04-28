













#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"

namespace nav2_behavior_tree
{



class ClearEntireCostmapService : public BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  

  ClearEntireCostmapService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  

  void on_tick() override;
};



class ClearCostmapExceptRegionService
  : public BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  

  ClearCostmapExceptRegionService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  

  void on_tick() override;

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>(
          "reset_distance", 1,
          "Distance from the robot above which obstacles are cleared")
      });
  }
};



class ClearCostmapAroundRobotService : public BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>
{
public:
  

  ClearCostmapAroundRobotService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  

  void on_tick() override;

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>(
          "reset_distance", 1,
          "Distance from the robot under which obstacles are cleared")
      });
  }
};

}

#endif
