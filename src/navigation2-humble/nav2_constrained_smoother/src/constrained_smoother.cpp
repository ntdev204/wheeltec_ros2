















#include <algorithm>
#include <string>
#include <memory>
#include <utility>
#include <vector>

#include "nav2_constrained_smoother/constrained_smoother.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;

namespace nav2_constrained_smoother
{

void ConstrainedSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_sub_ = costmap_sub;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  smoother_ = std::make_unique<nav2_constrained_smoother::Smoother>();
  optimizer_params_.get(node.get(), name);
  smoother_params_.get(node.get(), name);
  smoother_->initialize(optimizer_params_);
}

void ConstrainedSmoother::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up smoother: %s of type"
    " nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

void ConstrainedSmoother::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating smoother: %s of type "
    "nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

void ConstrainedSmoother::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating smoother: %s of type "
    "nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

bool ConstrainedSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 2) {
    return true;
  }


  std::vector<Eigen::Vector3d> path_world;
  path_world.reserve(path.poses.size());


  Eigen::Vector2d start_dir;
  Eigen::Vector2d end_dir;
  for (size_t i = 0; i < path.poses.size(); i++) {
    auto & pose = path.poses[i].pose;
    double angle = tf2::getYaw(pose.orientation);
    Eigen::Vector2d orientation(cos(angle), sin(angle));
    if (i == path.poses.size() - 1) {



      path_world.emplace_back(pose.position.x, pose.position.y, path_world.back()[2]);
      end_dir = orientation;
    } else {
      auto & pos_next = path.poses[i + 1].pose.position;
      Eigen::Vector2d mvmt(pos_next.x - pose.position.x, pos_next.y - pose.position.y);


      bool reversing = smoother_params_.reversing_enabled && orientation.dot(mvmt) < 0;


      path_world.emplace_back(pose.position.x, pose.position.y, reversing ? -1 : 1);
      if (i == 0) {
        start_dir = orientation;
      } else if (i == 1 && !smoother_params_.keep_start_orientation) {


        path_world[0][2] = path_world.back()[2];
      }
    }
  }

  smoother_params_.max_time = max_time.seconds();


  auto costmap = costmap_sub_->getCostmap();
  if (!smoother_->smooth(path_world, start_dir, end_dir, costmap.get(), smoother_params_)) {
    RCLCPP_WARN(
      logger_,
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      plugin_name_.c_str());
    throw new nav2_core::PlannerException(
            "Failed to smooth plan, Ceres could not find a usable solution.");
  }


  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.poses.front().header;
  path.poses.clear();
  path.poses.reserve(path_world.size());
  for (auto & pw : path_world) {
    pose.pose.position.x = pw[0];
    pose.pose.position.y = pw[1];
    pose.pose.orientation.z = sin(pw[2] / 2);
    pose.pose.orientation.w = cos(pw[2] / 2);

    path.poses.push_back(pose);
  }

  return true;
}

}


PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_smoother::ConstrainedSmoother,
  nav2_core::Smoother)
