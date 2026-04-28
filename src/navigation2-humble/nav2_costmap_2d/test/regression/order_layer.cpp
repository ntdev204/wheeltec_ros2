













#include "order_layer.hpp"

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

OrderLayer::OrderLayer()
: activated_(false)
{
}

void OrderLayer::activate()
{
  std::this_thread::sleep_for(100ms);
  activated_ = true;
}

void OrderLayer::deactivate()
{
  activated_ = false;
}

void OrderLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  if (!activated_) {
    throw std::runtime_error("update before activated");
  }
}

void OrderLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &, int, int, int, int)
{
  if (!activated_) {
    throw std::runtime_error("update before activated");
  }
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::OrderLayer, nav2_costmap_2d::Layer)
