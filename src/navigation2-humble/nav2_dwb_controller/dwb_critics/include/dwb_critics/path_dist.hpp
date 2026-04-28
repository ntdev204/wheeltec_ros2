

#ifndef DWB_CRITICS__PATH_DIST_HPP_
#define DWB_CRITICS__PATH_DIST_HPP_

#include "dwb_critics/map_grid.hpp"

namespace dwb_critics
{


class PathDistCritic : public MapGridCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
};

}
#endif
