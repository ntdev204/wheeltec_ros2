













#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);

  getParam(threshold_to_consider_, "threshold_to_consider", 0.5);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, and %f "
    "angular threshold.",
    power_, weight_, threshold_to_consider_);
}

void GoalAngleCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  if (!utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  data.costs += xt::pow(
    xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
    weight_, power_);
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)
