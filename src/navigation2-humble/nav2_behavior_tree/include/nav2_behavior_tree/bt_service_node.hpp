













#ifndef NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;



template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  

  BtServiceNode(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(service_node_name, conf), service_node_name_(service_node_name)
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


    getInput("service_name", service_name_);
    service_client_ = node_->create_client<ServiceT>(
      service_name_,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);


    request_ = std::make_shared<typename ServiceT::Request>();


    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for \"%s\" service",
      service_name_.c_str());
    if (!service_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" service server not available after waiting for 1 s",
        service_node_name.c_str());
      throw std::runtime_error(
              std::string(
                "Service server %s not available",
                service_node_name.c_str()));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "\"%s\" BtServiceNode initialized",
      service_node_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode()
  {
  }

  

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  

  BT::NodeStatus tick() override
  {
    if (!request_sent_) {
      on_tick();
      future_result_ = service_client_->async_send_request(request_).share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }
    return check_future();
  }

  

  void halt() override
  {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  

  virtual void on_tick()
  {
  }

  

  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response>)
  {
    return BT::NodeStatus::SUCCESS;
  }

  

  virtual BT::NodeStatus check_future()
  {
    auto elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  

  virtual void on_wait_for_result()
  {
  }

protected:
  

  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);
  }

  std::string service_name_, service_node_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;


  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;



  std::chrono::milliseconds server_timeout_;


  std::chrono::milliseconds bt_loop_duration_;


  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;
};

}

#endif
