













#include "nav2_collision_monitor/kinematics.hpp"

#include <cmath>

namespace nav2_collision_monitor
{

void transformPoints(const Pose & pose, std::vector<Point> & points)
{
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);

  for (Point & point : points) {









    const double mul_x = point.x - pose.x;
    const double mul_y = point.y - pose.y;
    point.x = mul_x * cos_theta + mul_y * sin_theta;
    point.y = -mul_x * sin_theta + mul_y * cos_theta;
  }
}

void projectState(const double & dt, Pose & pose, Velocity & velocity)
{
  const double theta = velocity.tw * dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);





  pose.x = pose.x + velocity.x * dt;
  pose.y = pose.y + velocity.y * dt;

  pose.theta = pose.theta + theta;






  const double velocity_upd_x = velocity.x * cos_theta - velocity.y * sin_theta;
  const double velocity_upd_y = velocity.x * sin_theta + velocity.y * cos_theta;
  velocity.x = velocity_upd_x;
  velocity.y = velocity_upd_y;
}

}
