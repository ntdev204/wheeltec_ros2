

#ifndef DWB_CRITICS__GOAL_DIST_HPP_
#define DWB_CRITICS__GOAL_DIST_HPP_

#include <vector>
#include "dwb_critics/map_grid.hpp"

namespace dwb_critics
{


class GoalDistCritic : public MapGridCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;

protected:
  bool getLastPoseOnCostmap(
    const nav_2d_msgs::msg::Path2D & global_plan, unsigned int & x,
    unsigned int & y);
};

}
#endif
