













#ifndef DEPRECATED__OPTIONS_HPP_
#define DEPRECATED__OPTIONS_HPP_

#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_smac_planner
{



struct SmootherParams
{
  

  SmootherParams()
  {
  }

  

  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.smoother.");


    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_curve", rclcpp::ParameterValue(1.5));
    node->get_parameter(local_name + "w_curve", curvature_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_cost", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_cost", costmap_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_dist", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_dist", distance_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(15000.0));
    node->get_parameter(local_name + "w_smooth", smooth_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "cost_scaling_factor", rclcpp::ParameterValue(10.0));
    node->get_parameter(local_name + "cost_scaling_factor", costmap_factor);
  }

  double smooth_weight{0.0};
  double costmap_weight{0.0};
  double distance_weight{0.0};
  double curvature_weight{0.0};
  double max_curvature{0.0};
  double costmap_factor{0.0};
  double max_time;
};



struct OptimizerParams
{
  OptimizerParams()
  : debug(false),
    max_iterations(50),
    max_time(1e4),
    param_tol(1e-8),
    fn_tol(1e-6),
    gradient_tol(1e-10)
  {
  }

  

  struct AdvancedParams
  {
    AdvancedParams()
    : min_line_search_step_size(1e-9),
      max_num_line_search_step_size_iterations(20),
      line_search_sufficient_function_decrease(1e-4),
      max_num_line_search_direction_restarts(20),
      max_line_search_step_contraction(1e-3),
      min_line_search_step_contraction(0.6),
      line_search_sufficient_curvature_decrease(0.9),
      max_line_search_step_expansion(10)
    {
    }

    

    void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
    {
      std::string local_name = name + std::string(".smoother.optimizer.advanced.");


      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "min_line_search_step_size",
        rclcpp::ParameterValue(1e-20));
      node->get_parameter(
        local_name + "min_line_search_step_size",
        min_line_search_step_size);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_num_line_search_step_size_iterations",
        rclcpp::ParameterValue(50));
      node->get_parameter(
        local_name + "max_num_line_search_step_size_iterations",
        max_num_line_search_step_size_iterations);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "line_search_sufficient_function_decrease",
        rclcpp::ParameterValue(1e-20));
      node->get_parameter(
        local_name + "line_search_sufficient_function_decrease",
        line_search_sufficient_function_decrease);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_num_line_search_direction_restarts",
        rclcpp::ParameterValue(10));
      node->get_parameter(
        local_name + "max_num_line_search_direction_restarts",
        max_num_line_search_direction_restarts);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_line_search_step_expansion",
        rclcpp::ParameterValue(50));
      node->get_parameter(
        local_name + "max_line_search_step_expansion",
        max_line_search_step_expansion);
    }


    double min_line_search_step_size;
    int max_num_line_search_step_size_iterations;
    double line_search_sufficient_function_decrease;
    int max_num_line_search_direction_restarts;

    double max_line_search_step_contraction;
    double min_line_search_step_contraction;
    double line_search_sufficient_curvature_decrease;
    int max_line_search_step_expansion;
  };

  

  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.optimizer.");


    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
    node->get_parameter(local_name + "param_tol", param_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
    node->get_parameter(local_name + "fn_tol", fn_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "gradient_tol", gradient_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_iterations", rclcpp::ParameterValue(500));
    node->get_parameter(local_name + "max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_time", rclcpp::ParameterValue(0.100));
    node->get_parameter(local_name + "max_time", max_time);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
    node->get_parameter(local_name + "debug_optimizer", debug);

    advanced.get(node, name);
  }

  bool debug;
  int max_iterations;
  double max_time;

  double param_tol;
  double fn_tol;
  double gradient_tol;

  AdvancedParams advanced;
};

}

#endif
