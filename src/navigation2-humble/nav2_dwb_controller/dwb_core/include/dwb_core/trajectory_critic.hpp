


#ifndef DWB_CORE__TRAJECTORY_CRITIC_HPP_
#define DWB_CORE__TRAJECTORY_CRITIC_HPP_

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_core
{


class TrajectoryCritic
{
public:
  using Ptr = std::shared_ptr<dwb_core::TrajectoryCritic>;

  virtual ~TrajectoryCritic() {}

  

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & name,
    const std::string & ns,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = nh;
    name_ = name;
    costmap_ros_ = costmap_ros;
    dwb_plugin_name_ = ns;
    if (!nh->has_parameter(dwb_plugin_name_ + "." + name_ + ".scale")) {
      nh->declare_parameter(
        dwb_plugin_name_ + "." + name_ + ".scale",
        rclcpp::ParameterValue(1.0));
    }
    nh->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
    onInit();
  }
  virtual void onInit() {}

  

  virtual void reset() {}

  

  virtual bool prepare(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &,
    const nav_2d_msgs::msg::Path2D &)
  {
    return true;
  }

  

  virtual double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) = 0;

  

  virtual void debrief(const nav_2d_msgs::msg::Twist2D &) {}

  

  virtual void addCriticVisualization(std::vector<std::pair<std::string, std::vector<float>>> &) {}

  std::string getName()
  {
    return name_;
  }

  virtual double getScale() const {return scale_;}
  void setScale(const double scale) {scale_ = scale;}

protected:
  std::string name_;
  std::string dwb_plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  double scale_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
};

}

#endif
