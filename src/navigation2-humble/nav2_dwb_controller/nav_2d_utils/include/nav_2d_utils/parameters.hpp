


#ifndef NAV_2D_UTILS__PARAMETERS_HPP_
#define NAV_2D_UTILS__PARAMETERS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
namespace nav_2d_utils
{



template<class param_t>
param_t searchAndGetParam(
  const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & param_name,
  const param_t & default_value)
{
  param_t value;
  nav2_util::declare_parameter_if_not_declared(
    nh, param_name,
    rclcpp::ParameterValue(default_value));
  nh->get_parameter(param_name, value);
  return value;
}

}
#pragma GCC diagnostic pop

#endif
