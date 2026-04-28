













#ifndef DEPRECATED__UPSAMPLER_COST_FUNCTION_HPP_
#define DEPRECATED__UPSAMPLER_COST_FUNCTION_HPP_

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


class UpsamplerCostFunction : public ceres::FirstOrderFunction
{
public:
  

  UpsamplerCostFunction(
    const std::vector<Eigen::Vector2d> & path,
    const SmootherParams & params,
    const int & upsample_ratio)
  : _num_params(2 * path.size()),
    _params(params),
    _upsample_ratio(upsample_ratio),
    _path(path)
  {
  }


  

  struct CurvatureComputations
  {
    

    CurvatureComputations()
    {
      valid = false;
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

  

  virtual bool Evaluate(
    const double * parameters,
    double * cost,
    double * gradient) const
  {
    Eigen::Vector2d xi;
    Eigen::Vector2d xi_p1;
    Eigen::Vector2d xi_m1;
    uint x_index, y_index;
    cost[0] = 0.0;
    double cost_raw = 0.0;
    double grad_x_raw = 0.0;
    double grad_y_raw = 0.0;


    CurvatureComputations curvature_params;

    for (int i = 0; i != NumParameters() / 2; i++) {
      x_index = 2 * i;
      y_index = 2 * i + 1;
      gradient[x_index] = 0.0;
      gradient[y_index] = 0.0;
      if (i < 1 || i >= NumParameters() / 2 - 1) {
        continue;
      }


      if (i % _upsample_ratio == 1) {
        continue;
      }

      xi = Eigen::Vector2d(parameters[x_index], parameters[y_index]);


      xi_p1 = _path.at(i + 1);
      xi_m1 = _path.at(i - 1);




      addSmoothingResidual(15000, xi, xi_p1, xi_m1, cost_raw);
      addCurvatureResidual(60.0, xi, xi_p1, xi_m1, curvature_params, cost_raw);

      if (gradient != NULL) {

        addSmoothingJacobian(15000, xi, xi_p1, xi_m1, grad_x_raw, grad_y_raw);
        addCurvatureJacobian(60.0, xi, xi_p1, xi_m1, curvature_params, grad_x_raw, grad_y_raw);

        gradient[x_index] = grad_x_raw;
        gradient[y_index] = grad_y_raw;
      }
    }

    cost[0] = cost_raw;
    return true;
  }

  

  virtual int NumParameters() const {return _num_params;}

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

  int _num_params;
  SmootherParams _params;
  int _upsample_ratio;
  std::vector<Eigen::Vector2d> _path;
};

}

#endif
