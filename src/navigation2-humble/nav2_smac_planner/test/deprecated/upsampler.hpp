













#ifndef DEPRECATED__UPSAMPLER_HPP_
#define DEPRECATED__UPSAMPLER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <algorithm>
#include <utility>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/upsampler_cost_function.hpp"
#include "nav2_smac_planner/upsampler_cost_function_nlls.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_smac_planner
{



class Upsampler
{
public:
  

  Upsampler() {}

  

  ~Upsampler() {}

  

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

  

  bool upsample(
    std::vector<Eigen::Vector2d> & path,
    const SmootherParams & params,
    const int & upsample_ratio)
  {
    _options.max_solver_time_in_seconds = params.max_time;

    if (upsample_ratio != 2 && upsample_ratio != 4) {

      return false;
    }

    const int param_ratio = upsample_ratio * 2.0;
    const int total_size = 2 * (path.size() * upsample_ratio - upsample_ratio + 1);
    double parameters[total_size];




    unsigned int next_pt;
    Eigen::Vector2d interpolated;
    std::vector<Eigen::Vector2d> temp_path;
    for (unsigned int pt = 0; pt != path.size() - 1; pt++) {
      next_pt = pt + 1;
      interpolated = (path[next_pt] + path[pt]) / 2.0;

      parameters[param_ratio * pt] = path[pt][0];
      parameters[param_ratio * pt + 1] = path[pt][1];
      temp_path.push_back(path[pt]);

      parameters[param_ratio * pt + 2] = interpolated[0];
      parameters[param_ratio * pt + 3] = interpolated[1];
      temp_path.push_back(interpolated);
    }

    parameters[total_size - 2] = path.back()[0];
    parameters[total_size - 1] = path.back()[1];
    temp_path.push_back(path.back());


    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UpsamplerCostFunction(temp_path, params, upsample_ratio));
    ceres::Solve(_options, problem, parameters, &summary);


    path.resize(total_size / 2);
    for (int i = 0; i != total_size / 2; i++) {
      path[i][0] = parameters[2 * i];
      path[i][1] = parameters[2 * i + 1];
    }











































    if (_debug) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost <= 0.0) {
      return false;
    }

    return true;
  }

private:
  bool _debug;
  ceres::GradientProblemSolver::Options _options;
};

}

#endif
