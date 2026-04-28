













#ifndef NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

namespace mppi::models
{



struct Trajectories
{
  xt::xtensor<float, 2> x;
  xt::xtensor<float, 2> y;
  xt::xtensor<float, 2> yaws;

  

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = xt::zeros<float>({batch_size, time_steps});
    y = xt::zeros<float>({batch_size, time_steps});
    yaws = xt::zeros<float>({batch_size, time_steps});
  }
};

}

#endif
