













#ifndef NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
#define NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/behavior.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_behaviors
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;



template<typename ActionT>
class TimedBehavior : public nav2_core::Behavior
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  

  TimedBehavior()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false),
    transform_tolerance_(0.0)
  {
  }

  virtual ~TimedBehavior() = default;




  virtual Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;







  virtual Status onCycleUpdate() = 0;



  virtual void onConfigure()
  {
  }



  virtual void onCleanup()
  {
  }


  virtual void onActionCompletion()
  {
  }


  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    behavior_name_ = name;
    tf_ = tf;

    node->get_parameter("cycle_frequency", cycle_frequency_);
    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    node->get_parameter("transform_tolerance", transform_tolerance_);

    action_server_ = std::make_shared<ActionServer>(
      node, behavior_name_,
      std::bind(&TimedBehavior::execute, this));

    collision_checker_ = collision_checker;

    vel_pub_ = node->template create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    onConfigure();
  }


  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
    onCleanup();
  }


  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", behavior_name_.c_str());

    vel_pub_->on_activate();
    action_server_->activate();
    enabled_ = true;
  }


  void deactivate() override
  {
    vel_pub_->on_deactivate();
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string behavior_name_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double cycle_frequency_;
  double enabled_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  rclcpp::Duration elasped_time_{0, 0};


  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};


  rclcpp::Logger logger_{rclcpp::get_logger("nav2_behaviors")};



  void execute()
  {
    RCLCPP_INFO(logger_, "Running %s", behavior_name_.c_str());

    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Called while inactive, ignoring request.");
      return;
    }

    if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
      RCLCPP_INFO(
        logger_,
        "Initial checks failed for %s", behavior_name_.c_str());
      action_server_->terminate_current();
      return;
    }

    auto start_time = steady_clock_.now();


    auto result = std::make_shared<typename ActionT::Result>();

    rclcpp::WallRate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      elasped_time_ = steady_clock_.now() - start_time;
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(logger_, "Canceling %s", behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = elasped_time_;
        action_server_->terminate_all(result);
        onActionCompletion();
        return;
      }


      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          logger_, "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_current(result);
        onActionCompletion();
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          RCLCPP_INFO(
            logger_,
            "%s completed successfully", behavior_name_.c_str());
          result->total_elapsed_time = steady_clock_.now() - start_time;
          action_server_->succeeded_current(result);
          onActionCompletion();
          return;

        case Status::FAILED:
          RCLCPP_WARN(logger_, "%s failed", behavior_name_.c_str());
          result->total_elapsed_time = steady_clock_.now() - start_time;
          action_server_->terminate_current(result);
          onActionCompletion();
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }


  void stopRobot()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;

    vel_pub_->publish(std::move(cmd_vel));
  }
};

}

#endif
