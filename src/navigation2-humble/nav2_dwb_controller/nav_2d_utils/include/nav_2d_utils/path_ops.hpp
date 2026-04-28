


#ifndef NAV_2D_UTILS__PATH_OPS_HPP_
#define NAV_2D_UTILS__PATH_OPS_HPP_

#include "nav_2d_msgs/msg/path2_d.hpp"

namespace nav_2d_utils
{


nav_2d_msgs::msg::Path2D adjustPlanResolution(
  const nav_2d_msgs::msg::Path2D & global_plan_in,
  double resolution);
}

#endif
