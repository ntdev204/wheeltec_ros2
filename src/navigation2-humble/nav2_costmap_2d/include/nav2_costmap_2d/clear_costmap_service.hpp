













#ifndef NAV2_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_
#define NAV2_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{

class Costmap2DROS;



class ClearCostmapService
{
public:
  

  ClearCostmapService(const nav2_util::LifecycleNode::WeakPtr & parent, Costmap2DROS & costmap);

  

  ClearCostmapService() = delete;

  

  void clearRegion(double reset_distance, bool invert);

  

  void clearEntirely();

private:

  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};


  Costmap2DROS & costmap_;


  unsigned char reset_value_;
  std::vector<std::string> clearable_layers_;


  rclcpp::Service<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr clear_except_service_;
  

  void clearExceptRegionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_service_;
  

  void clearAroundRobotCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_service_;
  

  void clearEntireCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response);

  

  void clearLayerRegion(
    std::shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
    bool invert);

  

  bool getPosition(double & x, double & y) const;
};

}

#endif
