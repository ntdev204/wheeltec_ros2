












#include <vector>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_smac_planner/constants.hpp"

#ifndef NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_

namespace nav2_smac_planner
{



class GridCollisionChecker
  : public nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
{
public:
  

  GridCollisionChecker(
    nav2_costmap_2d::Costmap2D * costmap,
    unsigned int num_quantizations);

  





  

  void setFootprint(
    const nav2_costmap_2d::Footprint & footprint,
    const bool & radius,
    const double & possible_inscribed_cost);

  

  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown);

  

  bool inCollision(
    const unsigned int & i,
    const bool & traverse_unknown);

  

  float getCost();

  

  std::vector<float> & getPrecomputedAngles()
  {
    return angles_;
  }

private:
  

  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::vector<nav2_costmap_2d::Footprint> oriented_footprints_;
  nav2_costmap_2d::Footprint unoriented_footprint_;
  double footprint_cost_;
  bool footprint_is_radius_;
  std::vector<float> angles_;
  double possible_inscribed_cost_{-1};
};

}

#endif
