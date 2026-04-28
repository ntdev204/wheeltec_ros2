


#ifndef DWB_CRITICS__TWIRLING_HPP_
#define DWB_CRITICS__TWIRLING_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{


class TwirlingCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
};
}

#endif
