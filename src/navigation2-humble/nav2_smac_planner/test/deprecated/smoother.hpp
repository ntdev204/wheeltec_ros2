













#ifndef DEPRECATED__SMOOTHER_HPP_
#define DEPRECATED__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/smoother_cost_function.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_smac_planner
{



class Smoother
{
public:
  

  Smoother() {}

  

  ~Smoother() {}

  

  void initialize(const OptimizerParams params)
  {
    _debug = params.debug;




    _options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    _options.line_search_type = ceres::WOLFE;
    _options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    _options.line_search_interpolation_type = ceres::CUBIC;

    _options.max_num_iterations = params.max_iterations;
    _options.max_solver_time_in_seconds = params.max_time;

    _options.function_tolerance = params.fn_tol;
    _options.gradient_tolerance = params.gradient_tol;
    _options.parameter_tolerance = params.param_tol;

    _options.min_line_search_step_size = params.advanced.min_line_search_step_size;
    _options.max_num_line_search_step_size_iterations =
      params.advanced.max_num_line_search_step_size_iterations;
    _options.line_search_sufficient_function_decrease =
      params.advanced.line_search_sufficient_function_decrease;
    _options.max_line_search_step_contraction = params.advanced.max_line_search_step_contraction;
    _options.min_line_search_step_contraction = params.advanced.min_line_search_step_contraction;
    _options.max_num_line_search_direction_restarts =
      params.advanced.max_num_line_search_direction_restarts;
    _options.line_search_sufficient_curvature_decrease =
      params.advanced.line_search_sufficient_curvature_decrease;
    _options.max_line_search_step_expansion = params.advanced.max_line_search_step_expansion;

    if (_debug) {
      _options.minimizer_progress_to_stdout = true;
    } else {
      _options.logging_type = ceres::SILENT;
    }
  }

  

  bool smooth(
    std::vector<Eigen::Vector2d> & path,
    nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params)
  {
    _options.max_solver_time_in_seconds = params.max_time;

#ifdef _MSC_VER
    std::vector<double> parameters_vec(path.size() * 2);
    double * parameters = parameters_vec.data();
#else
    double parameters[path.size() * 2];
#endif
    for (unsigned int i = 0; i != path.size(); i++) {
      parameters[2 * i] = path[i][0];
      parameters[2 * i + 1] = path[i][1];
    }

    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UnconstrainedSmootherCostFunction(&path, costmap, params));
    ceres::Solve(_options, problem, parameters, &summary);

    if (_debug) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost <= 0.0) {
      return false;
    }

    for (unsigned int i = 0; i != path.size(); i++) {
      path[i][0] = parameters[2 * i];
      path[i][1] = parameters[2 * i + 1];
    }

    return true;
  }

private:
  bool _debug;
  ceres::GradientProblemSolver::Options _options;
};

}

#endif
