















#ifndef NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
#define NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_costmap_2d
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;



template<typename CostmapT>
class FootprintCollisionChecker
{
public:
  

  FootprintCollisionChecker();
  

  explicit FootprintCollisionChecker(CostmapT costmap);
  

  double footprintCost(const Footprint footprint);
  

  double footprintCostAtPose(double x, double y, double theta, const Footprint footprint);
  

  double lineCost(int x0, int x1, int y0, int y1) const;
  

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  

  double pointCost(int x, int y) const;
  

  void setCostmap(CostmapT costmap);
  

  CostmapT getCostmap()
  {
    return costmap_;
  }

protected:
  CostmapT costmap_;
};

}

#endif
