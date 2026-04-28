


#include "nav2_costmap_2d/layer.hpp"

#include <string>
#include <vector>
#include "nav2_util/node_utils.hpp"

namespace nav2_costmap_2d
{

Layer::Layer()
: layered_costmap_(nullptr),
  name_(),
  tf_(nullptr),
  current_(false),
  enabled_(false)
{}

void
Layer::initialize(
  LayeredCostmap * parent,
  std::string name,
  tf2_ros::Buffer * tf,
  const nav2_util::LifecycleNode::WeakPtr & node,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  layered_costmap_ = parent;
  name_ = name;
  tf_ = tf;
  node_ = node;
  callback_group_ = callback_group;
  {
    auto node_shared_ptr = node_.lock();
    logger_ = node_shared_ptr->get_logger();
    clock_ = node_shared_ptr->get_clock();
  }

  onInitialize();
}

const std::vector<geometry_msgs::msg::Point> &
Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

void
Layer::declareParameter(
  const std::string & param_name,
  const rclcpp::ParameterValue & value)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  nav2_util::declare_parameter_if_not_declared(
    node, getFullName(param_name), value);
}

void
Layer::declareParameter(
  const std::string & param_name,
  const rclcpp::ParameterType & param_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  nav2_util::declare_parameter_if_not_declared(
    node, getFullName(param_name), param_type);
}

bool
Layer::hasParameter(const std::string & param_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  return node->has_parameter(getFullName(param_name));
}

std::string
Layer::getFullName(const std::string & param_name)
{
  return std::string(name_ + "." + param_name);
}

}
