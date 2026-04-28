














#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/speed_controller.hpp"

namespace nav2_behavior_tree
{

SpeedController::SpeedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_tick_(false),
  period_(1.0),
  min_rate_(0.1),
  max_rate_(1.0),
  min_speed_(0.0),
  max_speed_(0.5)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
    std::string err_msg = "SpeedController node cannot have rate <= 0.0";
    RCLCPP_FATAL(node_->get_logger(), err_msg.c_str());
    throw BT::BehaviorTreeException(err_msg);
  }

  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  std::string odom_topic;
  node_->get_parameter_or("odom_topic", odom_topic, std::string("odom"));
  odom_smoother_ = config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>(
    "odom_smoother");
}

inline BT::NodeStatus SpeedController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {


    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
  }

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {

    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
    goal_ = current_goal;
    goals_ = current_goals;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto elapsed = node_->now() - start_;




  if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    elapsed.seconds() >= period_)
  {
    first_tick_ = false;


    if (elapsed.seconds() >= period_) {
      updatePeriod();
      start_ = node_->now();
    }

    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpeedController>("SpeedController");
}
