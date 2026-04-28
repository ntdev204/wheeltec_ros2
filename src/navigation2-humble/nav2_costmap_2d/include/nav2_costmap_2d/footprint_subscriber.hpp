













#ifndef NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_
#define NAV2_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_costmap_2d
{


class FootprintSubscriber
{
public:
  

  FootprintSubscriber(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name,
    tf2_ros::Buffer & tf,
    std::string robot_base_frame = "base_link",
    double transform_tolerance = 0.1);

  

  FootprintSubscriber(
    const rclcpp::Node::WeakPtr & parent,
    const std::string & topic_name,
    tf2_ros::Buffer & tf,
    std::string robot_base_frame = "base_link",
    double transform_tolerance = 0.1);

  

  ~FootprintSubscriber() {}

  

  bool getFootprintRaw(
    std::vector<geometry_msgs::msg::Point> & footprint,
    std_msgs::msg::Header & footprint_header);

  

  bool getFootprintInRobotFrame(
    std::vector<geometry_msgs::msg::Point> & footprint,
    std_msgs::msg::Header & footprint_header);

protected:
  

  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  tf2_ros::Buffer & tf_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  bool footprint_received_{false};
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};

}

#endif
