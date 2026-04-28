













#ifndef NAV2_UTIL__LIFECYCLE_NODE_HPP_
#define NAV2_UTIL__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "nav2_util/node_thread.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"

namespace nav2_util
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;



class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  

  LifecycleNode(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

  typedef struct
  {
    double from_value;
    double to_value;
    double step;
  } floating_point_range;

  typedef struct
  {
    int from_value;
    int to_value;
    int step;
  } integer_range;

  

  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  

  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const floating_point_range fp_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  

  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const integer_range int_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  

  std::shared_ptr<nav2_util::LifecycleNode> shared_from_this()
  {
    return std::static_pointer_cast<nav2_util::LifecycleNode>(
      rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  

  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & )
  {
    RCLCPP_FATAL(
      get_logger(),
      "Lifecycle node %s does not have error state implemented", get_name());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  

  void createBond();

  

  void destroyBond();

protected:
  

  void printLifecycleNodeNotification();


  std::unique_ptr<bond::Bond> bond_{nullptr};
};

}

#endif
