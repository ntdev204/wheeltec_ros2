


#include "dwb_critics/alignment_util.hpp"
#include <cmath>

using std::cos;
using std::sin;

namespace dwb_critics
{
geometry_msgs::msg::Pose2D getForwardPose(const geometry_msgs::msg::Pose2D & pose, double distance)
{
  geometry_msgs::msg::Pose2D forward_pose;
  forward_pose.x = pose.x + distance * cos(pose.theta);
  forward_pose.y = pose.y + distance * sin(pose.theta);
  forward_pose.theta = pose.theta;
  return forward_pose;
}
}
