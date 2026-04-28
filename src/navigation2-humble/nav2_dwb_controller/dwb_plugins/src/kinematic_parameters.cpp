


#include "dwb_plugins/kinematic_parameters.hpp"

#include <memory>
#include <string>
#include <vector>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace dwb_plugins
{

KinematicsHandler::KinematicsHandler()
{
  kinematics_.store(new KinematicParameters);
}

KinematicsHandler::~KinematicsHandler()
{
  delete kinematics_.load();
}

void KinematicsHandler::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;

  declare_parameter_if_not_declared(nh, plugin_name + ".min_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".min_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".max_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".max_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".max_vel_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".min_speed_xy",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".max_speed_xy",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".min_speed_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".acc_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".acc_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".acc_lim_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".decel_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".decel_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".decel_lim_theta",
    rclcpp::ParameterValue(0.0));

  KinematicParameters kinematics;

  nh->get_parameter(plugin_name + ".min_vel_x", kinematics.min_vel_x_);
  nh->get_parameter(plugin_name + ".min_vel_y", kinematics.min_vel_y_);
  nh->get_parameter(plugin_name + ".max_vel_x", kinematics.max_vel_x_);
  nh->get_parameter(plugin_name + ".max_vel_y", kinematics.max_vel_y_);
  nh->get_parameter(plugin_name + ".max_vel_theta", kinematics.max_vel_theta_);
  nh->get_parameter(plugin_name + ".min_speed_xy", kinematics.min_speed_xy_);
  nh->get_parameter(plugin_name + ".max_speed_xy", kinematics.max_speed_xy_);
  nh->get_parameter(plugin_name + ".min_speed_theta", kinematics.min_speed_theta_);
  nh->get_parameter(plugin_name + ".acc_lim_x", kinematics.acc_lim_x_);
  nh->get_parameter(plugin_name + ".acc_lim_y", kinematics.acc_lim_y_);
  nh->get_parameter(plugin_name + ".acc_lim_theta", kinematics.acc_lim_theta_);
  nh->get_parameter(plugin_name + ".decel_lim_x", kinematics.decel_lim_x_);
  nh->get_parameter(plugin_name + ".decel_lim_y", kinematics.decel_lim_y_);
  nh->get_parameter(plugin_name + ".decel_lim_theta", kinematics.decel_lim_theta_);

  kinematics.base_max_vel_x_ = kinematics.max_vel_x_;
  kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
  kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
  kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;


  dyn_params_handler_ = nh->add_on_set_parameters_callback(
    std::bind(&KinematicsHandler::dynamicParametersCallback, this, _1));

  kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);
}

void KinematicsHandler::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  KinematicParameters kinematics(*kinematics_.load());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {

    kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_;
    kinematics.max_vel_x_ = kinematics.base_max_vel_x_;
    kinematics.max_vel_y_ = kinematics.base_max_vel_y_;
    kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_;
  } else {
    if (percentage) {

      kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_ * speed_limit / 100.0;
      kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * speed_limit / 100.0;
      kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * speed_limit / 100.0;
      kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * speed_limit / 100.0;
    } else {

      if (speed_limit < kinematics.base_max_speed_xy_) {
        kinematics.max_speed_xy_ = speed_limit;




        const double ratio = speed_limit / kinematics.base_max_speed_xy_;
        kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * ratio;
        kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * ratio;
        kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * ratio;
      }
    }
  }


  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);
}

rcl_interfaces::msg::SetParametersResult
KinematicsHandler::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  KinematicParameters kinematics(*kinematics_.load());

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".min_vel_x") {
        kinematics.min_vel_x_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_vel_y") {
        kinematics.min_vel_y_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_vel_x") {
        kinematics.max_vel_x_ = parameter.as_double();
        kinematics.base_max_vel_x_ = kinematics.max_vel_x_;
      } else if (name == plugin_name_ + ".max_vel_y") {
        kinematics.max_vel_y_ = parameter.as_double();
        kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
      } else if (name == plugin_name_ + ".max_vel_theta") {
        kinematics.max_vel_theta_ = parameter.as_double();
        kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;
      } else if (name == plugin_name_ + ".min_speed_xy") {
        kinematics.min_speed_xy_ = parameter.as_double();
        kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
      } else if (name == plugin_name_ + ".max_speed_xy") {
        kinematics.max_speed_xy_ = parameter.as_double();
        kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
      } else if (name == plugin_name_ + ".min_speed_theta") {
        kinematics.min_speed_theta_ = parameter.as_double();
        kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;
      } else if (name == plugin_name_ + ".acc_lim_x") {
        kinematics.acc_lim_x_ = parameter.as_double();
      } else if (name == plugin_name_ + ".acc_lim_y") {
        kinematics.acc_lim_y_ = parameter.as_double();
      } else if (name == plugin_name_ + ".acc_lim_theta") {
        kinematics.acc_lim_theta_ = parameter.as_double();
      } else if (name == plugin_name_ + ".decel_lim_x") {
        kinematics.decel_lim_x_ = parameter.as_double();
      } else if (name == plugin_name_ + ".decel_lim_y") {
        kinematics.decel_lim_y_ = parameter.as_double();
      } else if (name == plugin_name_ + ".decel_lim_theta") {
        kinematics.decel_lim_theta_ = parameter.as_double();
      }
    }
  }
  update_kinematics(kinematics);
  result.successful = true;
  return result;
}

void KinematicsHandler::update_kinematics(KinematicParameters kinematics)
{
  delete kinematics_.load();
  kinematics_.store(new KinematicParameters(kinematics));
}

}
