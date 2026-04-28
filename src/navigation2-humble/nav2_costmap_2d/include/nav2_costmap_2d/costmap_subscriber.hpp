













#ifndef NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{


class CostmapSubscriber
{
public:
  

  CostmapSubscriber(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name);

  

  CostmapSubscriber(
    const rclcpp::Node::WeakPtr & parent,
    const std::string & topic_name);

  

  ~CostmapSubscriber() {}

  

  std::shared_ptr<Costmap2D> getCostmap();

  

  void toCostmap2D();
  

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

protected:
  std::shared_ptr<Costmap2D> costmap_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
};

}

#endif
