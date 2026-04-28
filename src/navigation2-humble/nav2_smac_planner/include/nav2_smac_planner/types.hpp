













#ifndef NAV2_SMAC_PLANNER__TYPES_HPP_
#define NAV2_SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_smac_planner
{

typedef std::pair<float, unsigned int> NodeHeuristicPair;



struct SearchInfo
{
  float minimum_turning_radius;
  float non_straight_penalty;
  float change_penalty;
  float reverse_penalty;
  float cost_penalty;
  float retrospective_penalty;
  float rotation_penalty;
  float analytic_expansion_ratio;
  float analytic_expansion_max_length;
  std::string lattice_filepath;
  bool cache_obstacle_heuristic;
  bool allow_reverse_expansion;
};



struct SmootherParams
{
  

  SmootherParams()
  : holonomic_(false)
  {
  }

  

  void get(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.");


    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "tolerance", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "tolerance", tolerance_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_iterations", rclcpp::ParameterValue(1000));
    node->get_parameter(local_name + "max_iterations", max_its_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_data", rclcpp::ParameterValue(0.2));
    node->get_parameter(local_name + "w_data", w_data_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(0.3));
    node->get_parameter(local_name + "w_smooth", w_smooth_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "do_refinement", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "do_refinement", do_refinement_);
  }

  double tolerance_;
  int max_its_;
  double w_data_;
  double w_smooth_;
  bool holonomic_;
  bool do_refinement_;
};



struct MotionPose
{
  

  MotionPose() {}

  

  MotionPose(const float & x, const float & y, const float & theta)
  : _x(x), _y(y), _theta(theta)
  {}

  MotionPose operator-(const MotionPose & p2)
  {
    return MotionPose(this->_x - p2._x, this->_y - p2._y, this->_theta - p2._theta);
  }

  float _x;
  float _y;
  float _theta;
};

typedef std::vector<MotionPose> MotionPoses;



struct LatticeMetadata
{
  float min_turning_radius;
  float grid_resolution;
  unsigned int number_of_headings;
  std::vector<float> heading_angles;
  unsigned int number_of_trajectories;
  std::string motion_model;
};



struct MotionPrimitive
{
  unsigned int trajectory_id;
  float start_angle;
  float end_angle;
  float turning_radius;
  float trajectory_length;
  float arc_length;
  float straight_length;
  bool left_turn;
  MotionPoses poses;
};

typedef std::vector<MotionPrimitive> MotionPrimitives;
typedef std::vector<MotionPrimitive *> MotionPrimitivePtrs;

}

#endif
