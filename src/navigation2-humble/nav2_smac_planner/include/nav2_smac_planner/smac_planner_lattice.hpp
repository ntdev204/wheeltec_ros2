













#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace nav2_smac_planner
{

class SmacPlannerLattice : public nav2_core::GlobalPlanner
{
public:
  

  SmacPlannerLattice();

  

  ~SmacPlannerLattice();

  

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

  std::unique_ptr<AStarAlgorithm<NodeLattice>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerLattice")};
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  MotionModel _motion_model;
  LatticeMetadata _metadata;
  std::string _global_frame, _name;
  SearchInfo _search_info;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  float _tolerance;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  double _max_planning_time;
  double _lookup_table_size;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}

#endif
