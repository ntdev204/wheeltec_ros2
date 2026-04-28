


#ifndef DWB_CRITICS__ALIGNMENT_UTIL_HPP_
#define DWB_CRITICS__ALIGNMENT_UTIL_HPP_

#include "geometry_msgs/msg/pose2_d.hpp"

namespace dwb_critics
{


geometry_msgs::msg::Pose2D getForwardPose(const geometry_msgs::msg::Pose2D & pose, double distance);

}

#endif
