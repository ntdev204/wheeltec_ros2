













#ifndef DEPRECATED__SMOOTHER_COST_FUNCTION_HPP_
#define DEPRECATED__SMOOTHER_COST_FUNCTION_HPP_

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
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/options.hpp"

#define EPSILON 0.0001

namespace nav2_smac_planner
{



class UnconstrainedSmootherCostFunction : public ceres::FirstOrderFunction
{
public:
  

  UnconstrainedSmootherCostFunction(
    std::vector<Eigen::Vector2d> * original_path,
    nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params)
  : _original_path(original_path),
    _num_params(2 * original_path->size()),
    _costmap(costmap),
    _params(params)
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

    Eigen::Vector2d delta_xi{0.0, 0.0};
    Eigen::Vector2d delta_xi_p{0.0, 0.0};
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
    unsigned int x_index, y_index;
    cost[0] = 0.0;
    double cost_raw = 0.0;
    double grad_x_raw = 0.0;
    double grad_y_raw = 0.0;
    unsigned int mx, my;
    bool valid_coords = true;
    double costmap_cost = 0.0;


    CurvatureComputations curvature_params;

    for (int i = 0; i != NumParameters() / 2; i++) {
      x_index = 2 * i;
      y_index = 2 * i + 1;
      gradient[x_index] = 0.0;
      gradient[y_index] = 0.0;
      if (i < 1 || i >= NumParameters() / 2 - 1) {
        continue;
      }

      xi = Eigen::Vector2d(parameters[x_index], parameters[y_index]);
      xi_p1 = Eigen::Vector2d(parameters[x_index + 2], parameters[y_index + 2]);
      xi_m1 = Eigen::Vector2d(parameters[x_index - 2], parameters[y_index - 2]);


      addSmoothingResidual(_params.smooth_weight, xi, xi_p1, xi_m1, cost_raw);
      addCurvatureResidual(_params.curvature_weight, xi, xi_p1, xi_m1, curvature_params, cost_raw);
      addDistanceResidual(_params.distance_weight, xi, _original_path->at(i), cost_raw);

      if (valid_coords = _costmap->worldToMap(xi[0], xi[1], mx, my)) {
        costmap_cost = _costmap->getCost(mx, my);
        addCostResidual(_params.costmap_weight, costmap_cost, cost_raw, xi);
      }

      if (gradient != NULL) {

        gradient[x_index] = 0.0;
        gradient[y_index] = 0.0;
        addSmoothingJacobian(_params.smooth_weight, xi, xi_p1, xi_m1, grad_x_raw, grad_y_raw);
        addCurvatureJacobian(
          _params.curvature_weight, xi, xi_p1, xi_m1, curvature_params,
          grad_x_raw, grad_y_raw);
        addDistanceJacobian(
          _params.distance_weight, xi, _original_path->at(
            i), grad_x_raw, grad_y_raw);

        if (valid_coords) {
          addCostJacobian(_params.costmap_weight, mx, my, costmap_cost, grad_x_raw, grad_y_raw);
        }

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

  

  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    CurvatureComputations & curvature_params,
    double & r) const
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

    curvature_params.ki_minus_kmax = curvature_params.turning_rad - _params.max_curvature;

    if (curvature_params.ki_minus_kmax <= EPSILON) {

      curvature_params.valid = false;
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

  

  inline void addDistanceResidual(
    const double & weight,
    const Eigen::Vector2d & xi,
    const Eigen::Vector2d & xi_original,
    double & r) const
  {
    r += weight * (xi - xi_original).dot(xi - xi_original);
  }

  

  inline void addDistanceJacobian(
    const double & weight,
    const Eigen::Vector2d & xi,
    const Eigen::Vector2d & xi_original,
    double & j0,
    double & j1) const
  {
    j0 += weight * 2 * (xi[0] - xi_original[0]);
    j1 += weight * 2 * (xi[1] - xi_original[1]);
  }


  

  inline void addCostResidual(
    const double & weight,
    const double & value,
    double & r,
    Eigen::Vector2d & xi) const
  {
    if (value == FREE) {
      return;
    }

    r += weight * value * value;









  }

  

  inline void addCostJacobian(
    const double & weight,
    const unsigned int & mx,
    const unsigned int & my,
    const double & value,
    double & j0,
    double & j1) const
  {
    if (value == FREE) {
      return;
    }

    const Eigen::Vector2d grad = getCostmapGradient(mx, my);
    const double common_prefix = -2.0 * _params.costmap_factor * weight * value * value;

    j0 += common_prefix * grad[0];
    j1 += common_prefix * grad[1];
  }

  

  inline Eigen::Vector2d getCostmapGradient(
    const unsigned int mx,
    const unsigned int my) const
  {


    Eigen::Vector2d gradient;

    double l_1 = 0.0;
    double l_2 = 0.0;
    double l_3 = 0.0;
    double r_1 = 0.0;
    double r_2 = 0.0;
    double r_3 = 0.0;

    if (mx < _costmap->getSizeInCellsX()) {
      r_1 = static_cast<double>(_costmap->getCost(mx + 1, my));
    }
    if (mx + 1 < _costmap->getSizeInCellsX()) {
      r_2 = static_cast<double>(_costmap->getCost(mx + 2, my));
    }
    if (mx + 2 < _costmap->getSizeInCellsX()) {
      r_3 = static_cast<double>(_costmap->getCost(mx + 3, my));
    }

    if (mx > 0) {
      l_1 = static_cast<double>(_costmap->getCost(mx - 1, my));
    }
    if (mx - 1 > 0) {
      l_2 = static_cast<double>(_costmap->getCost(mx - 2, my));
    }
    if (mx - 2 > 0) {
      l_3 = static_cast<double>(_costmap->getCost(mx - 3, my));
    }

    gradient[1] = (45 * r_1 - 9 * r_2 + r_3 - 45 * l_1 + 9 * l_2 - l_3) / 60;

    if (my < _costmap->getSizeInCellsY()) {
      r_1 = static_cast<double>(_costmap->getCost(mx, my + 1));
    }
    if (my + 1 < _costmap->getSizeInCellsY()) {
      r_2 = static_cast<double>(_costmap->getCost(mx, my + 2));
    }
    if (my + 2 < _costmap->getSizeInCellsY()) {
      r_3 = static_cast<double>(_costmap->getCost(mx, my + 3));
    }

    if (my > 0) {
      l_1 = static_cast<double>(_costmap->getCost(mx, my - 1));
    }
    if (my - 1 > 0) {
      l_2 = static_cast<double>(_costmap->getCost(mx, my - 2));
    }
    if (my - 2 > 0) {
      l_3 = static_cast<double>(_costmap->getCost(mx, my - 3));
    }

    gradient[0] = (45 * r_1 - 9 * r_2 + r_3 - 45 * l_1 + 9 * l_2 - l_3) / 60;

    gradient.normalize();
    return gradient;
  }

  

  inline Eigen::Vector2d normalizedOrthogonalComplement(
    const Eigen::Vector2d & a,
    const Eigen::Vector2d & b,
    const double & a_norm,
    const double & b_norm) const
  {
    return (a - (a.dot(b) * b / b.squaredNorm())) / (a_norm * b_norm);
  }

  std::vector<Eigen::Vector2d> * _original_path{nullptr};
  int _num_params;
  nav2_costmap_2d::Costmap2D * _costmap{nullptr};
  SmootherParams _params;

};

}

#endif
