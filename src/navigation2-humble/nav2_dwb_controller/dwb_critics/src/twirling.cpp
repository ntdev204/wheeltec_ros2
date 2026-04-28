


#include "dwb_critics/twirling.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dwb_critics
{
void TwirlingCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
}

double TwirlingCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  return fabs(traj.velocity.theta);
}
}

PLUGINLIB_EXPORT_CLASS(dwb_critics::TwirlingCritic, dwb_core::TrajectoryCritic)
