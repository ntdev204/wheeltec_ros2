















#ifndef NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_

#include <string>
#include <memory>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"

namespace nav2_amcl
{



class MotionModel
{
public:
  virtual ~MotionModel() = default;

  

  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5) = 0;

  

  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) = 0;
};
}

#endif
