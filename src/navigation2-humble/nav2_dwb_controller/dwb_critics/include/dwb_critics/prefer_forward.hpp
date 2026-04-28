


#ifndef DWB_CRITICS__PREFER_FORWARD_HPP_
#define DWB_CRITICS__PREFER_FORWARD_HPP_

#include <string>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{



class PreferForwardCritic : public dwb_core::TrajectoryCritic
{
public:
  PreferForwardCritic()
  : penalty_(1.0), strafe_x_(0.1), strafe_theta_(0.2), theta_scale_(10.0) {}
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  double penalty_, strafe_x_, strafe_theta_, theta_scale_;
};

}
#endif
