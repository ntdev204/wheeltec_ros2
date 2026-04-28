













#ifndef NAV2_COSTMAP_2D__ORDER_LAYER_HPP_
#define NAV2_COSTMAP_2D__ORDER_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"

namespace nav2_costmap_2d
{

class OrderLayer : public nav2_costmap_2d::Layer
{
public:
  OrderLayer();

  virtual void activate();
  virtual void deactivate();

  virtual void reset() {}
  virtual bool isClearable() {return false;}

  virtual void updateBounds(
    double, double, double, double *, double *, double *, double *);

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D &, int, int, int, int);

private:
  bool activated_;
};

}

#endif
