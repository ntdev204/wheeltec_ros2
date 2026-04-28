


#ifndef DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_
#define DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_

#include <memory>
#include <string>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{


class LimitedAccelGenerator : public StandardTrajectoryGenerator
{
public:
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;

protected:
  

  nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & cmd_vel,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const double dt) override;
  double acceleration_time_;
  std::string plugin_name_;
};
}

#endif
