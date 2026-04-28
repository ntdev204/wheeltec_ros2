













#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{



class MotionModel
{
public:
  

  MotionModel() = default;

  

  virtual ~MotionModel() = default;

  

  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
      xt::view(state.cwz, xt::all(), xt::range(0, -1));

    if (isHolonomic()) {
      xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
        xt::view(state.cvy, xt::all(), xt::range(0, -1));
    }
  }

  

  virtual bool isHolonomic() = 0;

  

  virtual void applyConstraints(models::ControlSequence & ) {}
};



class AckermannMotionModel : public MotionModel
{
public:
  

  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {
    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  

  bool isHolonomic() override
  {
    return false;
  }

  

  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) < min_turning_r_);
    view = xt::sign(wz) * vx / min_turning_r_;
  }

  

  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};



class DiffDriveMotionModel : public MotionModel
{
public:
  

  DiffDriveMotionModel() = default;

  

  bool isHolonomic() override
  {
    return false;
  }
};



class OmniMotionModel : public MotionModel
{
public:
  

  OmniMotionModel() = default;

  

  bool isHolonomic() override
  {
    return true;
  }
};

}

#endif
