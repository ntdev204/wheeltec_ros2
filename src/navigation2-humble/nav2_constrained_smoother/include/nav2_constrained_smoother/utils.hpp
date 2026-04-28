













#ifndef NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_

#include <limits>
#include "Eigen/Core"

#define EPSILON 0.0001

namespace nav2_constrained_smoother
{



template<typename T>
inline Eigen::Matrix<T, 2, 1> arcCenter(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
  Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

  if (is_cusp) {
    d2 = -d2;
    pt_next = pt + d2;
  }

  T det = d1[0] * d2[1] - d1[1] * d2[0];
  if (ceres::abs(det) < (T)1e-4) {
    return Eigen::Matrix<T, 2, 1>(
      (T)std::numeric_limits<double>::infinity(), (T)std::numeric_limits<double>::infinity());
  }





  Eigen::Matrix<T, 2, 1> mid1 = (pt_prev + pt) / (T)2;
  Eigen::Matrix<T, 2, 1> mid2 = (pt + pt_next) / (T)2;
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  T det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0];
  T det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0];
  Eigen::Matrix<T, 2, 1> center((det1 * n2[0] - det2 * n1[0]) / det,
    (det1 * n2[1] - det2 * n1[1]) / det);
  return center;
}



template<typename T>
inline Eigen::Matrix<T, 2, 1> tangentDir(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> center = arcCenter(pt_prev, pt, pt_next, is_cusp);
  if (ceres::isinf(center[0])) {
    Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
    Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

    if (is_cusp) {
      d2 = -d2;
      pt_next = pt + d2;
    }

    Eigen::Matrix<T, 2, 1> result(pt_next[0] - pt_prev[0], pt_next[1] - pt_prev[1]);
    if (result[0] == 0.0 && result[1] == 0.0) {
      return Eigen::Matrix<T, 2, 1>(d1[1], -d1[0]);
    }
    return result;
  }



  return Eigen::Matrix<T, 2, 1>(center[1] - pt[1], pt[0] - center[0]);
}

}

#endif
