













#ifndef NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{



struct Path
{
  xt::xtensor<float, 1> x;
  xt::xtensor<float, 1> y;
  xt::xtensor<float, 1> yaws;

  

  void reset(unsigned int size)
  {
    x = xt::zeros<float>({size});
    y = xt::zeros<float>({size});
    yaws = xt::zeros<float>({size});
  }
};

}

#endif
