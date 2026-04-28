













#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/motion_models.hpp"


namespace mppi
{



struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;

  xt::xtensor<float, 1> & costs;
  float & model_dt;

  bool fail_flag;
  nav2_core::GoalChecker * goal_checker;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;
};

}

#endif
