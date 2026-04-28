













#ifndef NAV2_COLLISION_MONITOR__KINEMATICS_HPP_
#define NAV2_COLLISION_MONITOR__KINEMATICS_HPP_

#include <vector>

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{



void transformPoints(const Pose & pose, std::vector<Point> & points);



void projectState(const double & dt, Pose & pose, Velocity & velocity);

}

#endif
