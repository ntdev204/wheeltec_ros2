













#include "nav2_mppi_controller/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

using namespace xt::placeholders;
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0);

  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  getParam(use_path_orientations_, "use_path_orientations", false);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{

  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }


  utils::setPathFurthestPointIfNotSet(data);
  if (*data.furthest_reached_path_point < offset_from_furthest_) {
    return;
  }


  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

  const auto & T_x = data.trajectories.x;
  const auto & T_y = data.trajectories.y;
  const auto & T_yaw = data.trajectories.yaws;

  const auto P_x = xt::view(data.path.x, xt::range(_, -1));
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));

  const size_t batch_size = T_x.shape(0);
  const size_t time_steps = T_x.shape(1);
  const size_t traj_pts_eval = floor(time_steps / trajectory_point_step_);
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  if (path_segments_count < 1) {
    return;
  }

  float dist_sq = 0, dx = 0, dy = 0, dyaw = 0, summed_dist = 0;
  double min_dist_sq = std::numeric_limits<float>::max();
  size_t min_s = 0;

  for (size_t t = 0; t < batch_size; ++t) {
    summed_dist = 0;
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      min_dist_sq = std::numeric_limits<float>::max();
      min_s = 0;


      for (size_t s = 0; s < path_segments_count - 1; s++) {
        xt::xtensor_fixed<float, xt::xshape<2>> P;
        dx = P_x(s) - T_x(t, p);
        dy = P_y(s) - T_y(t, p);
        if (use_path_orientations_) {
          dyaw = angles::shortest_angular_distance(P_yaw(s), T_yaw(t, p));
          dist_sq = dx * dx + dy * dy + dyaw * dyaw;
        } else {
          dist_sq = dx * dx + dy * dy;
        }
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          min_s = s;
        }
      }



      if (min_s != 0 && (*data.path_pts_valid)[min_s]) {
        summed_dist += std::sqrt(min_dist_sq);
      }
    }

    cost[t] = summed_dist / traj_pts_eval;
  }

  data.costs += xt::pow(std::move(cost) * weight_, power_);
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
