













#ifndef NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace nav2_bt_navigator
{



struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};



class NavigatorMuxer
{
public:
  

  NavigatorMuxer()
  : current_navigator_(std::string("")) {}

  

  bool isNavigating()
  {
    std::scoped_lock l(mutex_);
    return !current_navigator_.empty();
  }

  

  void startNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation requested while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    }
    current_navigator_ = navigator_name;
  }

  

  void stopNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation stopped while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};



template<class ActionT>
class Navigator
{
public:
  using Ptr = std::shared_ptr<nav2_bt_navigator::Navigator<ActionT>>;

  

  Navigator()
  {
    plugin_muxer_ = nullptr;
  }

  

  virtual ~Navigator() = default;

  

  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    nav2_bt_navigator::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;


    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);


    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      default_bt_xml_filename,
      std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&Navigator::onLoop, this),
      std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
      std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);
    blackboard->set<bool>("initial_pose_received", false);
    blackboard->set<int>("number_recoveries", 0);
    blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);

    return configure(parent_node, odom_smoother) && ok;
  }

  

  bool on_activate()
  {
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return activate() && ok;
  }

  

  bool on_deactivate()
  {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return deactivate() && ok;
  }

  

  bool on_cleanup()
  {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return cleanup() && ok;
  }

  

  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  

  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "Requested navigation from %s while another navigator is processing,"
        " rejecting request.", getName().c_str());
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  

  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  

  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  

  virtual void onLoop() = 0;

  

  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  

  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  

  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr ,
    std::shared_ptr<nav2_util::OdomSmoother>)
  {
    return true;
  }

  

  virtual bool cleanup() {return true;}

  

  virtual bool activate() {return true;}

  

  virtual bool deactivate() {return true;}

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;
};

}

#endif
