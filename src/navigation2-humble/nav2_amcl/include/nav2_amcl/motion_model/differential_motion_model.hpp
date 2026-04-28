


#ifndef NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_

#include <sys/types.h>
#include <math.h>
#include <algorithm>
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/angleutils.hpp"


namespace nav2_amcl
{

class DifferentialMotionModel : public nav2_amcl::MotionModel
{
public:
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5);
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};
}
#endif
