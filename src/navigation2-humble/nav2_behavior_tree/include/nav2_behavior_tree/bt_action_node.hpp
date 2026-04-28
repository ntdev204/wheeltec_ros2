













#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;



template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  

  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());


    bt_loop_duration_ =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);


    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);


    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  

  void createActionClient(const std::string & action_name)
  {

    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);


    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for 1 s",
        action_name.c_str());
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
  }

  

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }




  

  virtual void on_tick()
  {
  }

  

  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>)
  {
  }

  

  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  

  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  

  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  

  BT::NodeStatus tick() override
  {

    if (status() == BT::NodeStatus::IDLE) {

      setStatus(BT::NodeStatus::RUNNING);


      on_tick();

      send_new_goal();
    }

    try {


      if (future_goal_handle_) {
        auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {

          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }

          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }


      if (rclcpp::ok() && !goal_result_available_) {

        on_wait_for_result(feedback_);


        feedback_.reset();

        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        callback_group_executor_.spin_some();


        if (!goal_result_available_) {

          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      if (e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server"))
      {

        return BT::NodeStatus::FAILURE;
      } else {

        throw e;
      }
    }

    BT::NodeStatus status;
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        break;

      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();
        break;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    goal_handle_.reset();
    return status;
  }

  

  void halt() override
  {
    if (should_cancel_goal()) {
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }
    }

    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  

  bool should_cancel_goal()
  {

    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }


    if (!goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();


    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  

  void send_new_goal()
  {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        if (future_goal_handle_) {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "Goal result for %s available, but it hasn't received the goal response yet. "
            "It's probably a goal result for the last goal request", action_name_.c_str());
          return;
        }




        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        feedback_ = feedback;
      };

    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    time_goal_sent_ = node_->now();
  }

  

  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;


    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    elapsed += timeout;

    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    return false;
  }

  

  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;


  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;


  std::shared_ptr<const typename ActionT::Feedback> feedback_;


  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;



  std::chrono::milliseconds server_timeout_;


  std::chrono::milliseconds bt_loop_duration_;


  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  future_goal_handle_;
  rclcpp::Time time_goal_sent_;
};

}

#endif
