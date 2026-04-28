













#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_smac_planner
{

class SmacPlanner2D : public nav2_core::GlobalPlanner
{
public:
  

  SmacPlanner2D();

  

  ~SmacPlanner2D();

  

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  

  void cleanup() override;

  

  void activate() override;

  

  void deactivate() override;

  

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlanner2D")};
  std::string _global_frame, _name;
  float _tolerance;
  int _downsampling_factor;
  bool _downsample_costmap;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  double _max_planning_time;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  bool _use_final_approach_orientation;
  SearchInfo _search_info;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}

#endif
