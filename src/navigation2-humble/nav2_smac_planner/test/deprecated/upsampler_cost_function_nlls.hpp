













#ifndef DEPRECATED__UPSAMPLER_COST_FUNCTION_NLLS_HPP_
#define DEPRECATED__UPSAMPLER_COST_FUNCTION_NLLS_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "Eigen/Core"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/options.hpp"

#define EPSILON 0.0001

namespace nav2_smac_planner
{


class UpsamplerConstrainedCostFunction : public ceres::SizedCostFunction<1, 1, 1>
{
public:
  

  UpsamplerConstrainedCostFunction(
    const std::vector<Eigen::Vector2d> & path,
    const SmootherParams & params,
    const int & upsample_ratio,
    const int & i)
  : _path(path),
    _params(params),
    _upsample_ratio(upsample_ratio),
    index(i)
  {
  }

  

  struct CurvatureComputations
  {
    

    CurvatureComputations()
    {
      valid = true;
    }

    bool valid;
    

    bool isValid()
    {
      return valid;
    }

    Eigen::Vector2d delta_xi{0, 0};
    Eigen::Vector2d delta_xi_p{0, 0};
    double delta_xi_norm{0};
    double delta_xi_p_norm{0};
    double delta_phi_i{0};
    double turning_rad{0};
    double ki_minus_kmax{0};
  };

  


  bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const override
  {
    Eigen::Vector2d xi = Eigen::Vector2d(parameters[0][0], parameters[1][0]);
    Eigen::Vector2d xi_p1 = _path.at(index + 1);
    Eigen::Vector2d xi_m1 = _path.at(index - 1);
    CurvatureComputations curvature_params;
    double grad_x_raw = 0, grad_y_raw = 0, cost_raw = 0;


    addSmoothingResidual(15000, xi, xi_p1, xi_m1, cost_raw);
    addCurvatureResidual(60.0, xi, xi_p1, xi_m1, curvature_params, cost_raw);

    residuals[0] = 0;
    residuals[0] = cost_raw;

    if (jacobians != NULL && jacobians[0] != NULL) {
      addSmoothingJacobian(15000, xi, xi_p1, xi_m1, grad_x_raw, grad_y_raw);
      addCurvatureJacobian(60.0, xi, xi_p1, xi_m1, curvature_params, grad_x_raw, grad_y_raw);

      jacobians[0][0] = 0;
      jacobians[1][0] = 0;
      jacobians[0][0] = grad_x_raw;
      jacobians[1][0] = grad_y_raw;
      jacobians[0][1] = 0.0;
      jacobians[1][1] = 0.0;
    }

    return true;
  }

protected:
  

  inline void addSmoothingResidual(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    double & r) const
  {
    r += weight * (
      pt_p.dot(pt_p) -
      4 * pt_p.dot(pt) +
      2 * pt_p.dot(pt_m) +
      4 * pt.dot(pt) -
      4 * pt.dot(pt_m) +
      pt_m.dot(pt_m));
  }

  

  inline void addSmoothingJacobian(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    double & j0,
    double & j1) const
  {
    j0 += weight *
      (-4 * pt_m[0] + 8 * pt[0] - 4 * pt_p[0]);
    j1 += weight *
      (-4 * pt_m[1] + 8 * pt[1] - 4 * pt_p[1]);
  }

  

  inline void getCurvatureParams(
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    CurvatureComputations & curvature_params) const
  {
    curvature_params.valid = true;
    curvature_params.delta_xi = Eigen::Vector2d(pt[0] - pt_m[0], pt[1] - pt_m[1]);
    curvature_params.delta_xi_p = Eigen::Vector2d(pt_p[0] - pt[0], pt_p[1] - pt[1]);
    curvature_params.delta_xi_norm = curvature_params.delta_xi.norm();
    curvature_params.delta_xi_p_norm = curvature_params.delta_xi_p.norm();

    if (curvature_params.delta_xi_norm < EPSILON || curvature_params.delta_xi_p_norm < EPSILON ||
      std::isnan(curvature_params.delta_xi_p_norm) || std::isnan(curvature_params.delta_xi_norm) ||
      std::isinf(curvature_params.delta_xi_p_norm) || std::isinf(curvature_params.delta_xi_norm))
    {

      curvature_params.valid = false;
      return;
    }

    const double & delta_xi_by_xi_p =
      curvature_params.delta_xi_norm * curvature_params.delta_xi_p_norm;
    double projection =
      curvature_params.delta_xi.dot(curvature_params.delta_xi_p) / delta_xi_by_xi_p;
    if (fabs(1 - projection) < EPSILON || fabs(projection + 1) < EPSILON) {
      projection = 1.0;
    }

    curvature_params.delta_phi_i = std::acos(projection);
    curvature_params.turning_rad = curvature_params.delta_phi_i / curvature_params.delta_xi_norm;

    curvature_params.ki_minus_kmax = curvature_params.turning_rad - _upsample_ratio *
      _params.max_curvature;

    if (curvature_params.ki_minus_kmax <= EPSILON) {

      curvature_params.valid = false;
    }
  }

  

  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    CurvatureComputations & curvature_params,
    double & r) const
  {
    getCurvatureParams(pt, pt_p, pt_m, curvature_params);

    if (!curvature_params.isValid()) {
      return;
    }


    r += weight *
      curvature_params.ki_minus_kmax * curvature_params.ki_minus_kmax;
  }

  

  inline void addCurvatureJacobian(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & ,
    CurvatureComputations & curvature_params,
    double & j0,
    double & j1) const
  {
    if (!curvature_params.isValid()) {
      return;
    }

    const double & partial_delta_phi_i_wrt_cost_delta_phi_i =
      -1 / std::sqrt(1 - std::pow(std::cos(curvature_params.delta_phi_i), 2));

    auto neg_pt_plus = -1 * pt_p;
    Eigen::Vector2d p1 = normalizedOrthogonalComplement(
      pt, neg_pt_plus, curvature_params.delta_xi_norm, curvature_params.delta_xi_p_norm);
    Eigen::Vector2d p2 = normalizedOrthogonalComplement(
      neg_pt_plus, pt, curvature_params.delta_xi_p_norm, curvature_params.delta_xi_norm);

    const double & u = 2 * curvature_params.ki_minus_kmax;
    const double & common_prefix =
      (1 / curvature_params.delta_xi_norm) * partial_delta_phi_i_wrt_cost_delta_phi_i;
    const double & common_suffix = curvature_params.delta_phi_i /
      (curvature_params.delta_xi_norm * curvature_params.delta_xi_norm);
    const Eigen::Vector2d & d_delta_xi_d_xi = curvature_params.delta_xi /
      curvature_params.delta_xi_norm;

    const Eigen::Vector2d jacobian = u *
      (common_prefix * (-p1 - p2) - (common_suffix * d_delta_xi_d_xi));
    const Eigen::Vector2d jacobian_im1 = u *
      (common_prefix * p2 + (common_suffix * d_delta_xi_d_xi));
    const Eigen::Vector2d jacobian_ip1 = u * (common_prefix * p1);
    j0 += weight * jacobian[0];
    j1 += weight * jacobian[1];




  }
  

  inline Eigen::Vector2d normalizedOrthogonalComplement(
    const Eigen::Vector2d & a,
    const Eigen::Vector2d & b,
    const double & a_norm,
    const double & b_norm) const
  {
    return (a - (a.dot(b) * b / b.squaredNorm())) / (a_norm * b_norm);
  }

  std::vector<Eigen::Vector2d> _path;
  SmootherParams _params;
  int _upsample_ratio;
  int index;
};

}

#endif
