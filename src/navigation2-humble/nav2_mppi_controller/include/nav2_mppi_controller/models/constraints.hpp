













#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{



struct ControlConstraints
{
  double vx_max;
  double vx_min;
  double vy;
  double wz;
};



struct SamplingStd
{
  double vx;
  double vy;
  double wz;
};

}

#endif
