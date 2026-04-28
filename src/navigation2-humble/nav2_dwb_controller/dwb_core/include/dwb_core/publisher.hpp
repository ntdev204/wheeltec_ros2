


#ifndef DWB_CORE__PUBLISHER_HPP_
#define DWB_CORE__PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_msgs/msg/local_plan_evaluation.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using rclcpp_lifecycle::LifecyclePublisher;

namespace dwb_core
{



class DWBPublisher
{
public:
  explicit DWBPublisher(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  nav2_util::CallbackReturn on_configure();
  nav2_util::CallbackReturn on_activate();
  nav2_util::CallbackReturn on_deactivate();
  nav2_util::CallbackReturn on_cleanup();

  

  bool shouldRecordEvaluation() {return publish_evaluation_ || publish_trajectories_;}

  

  void publishEvaluation(std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results);
  void publishLocalPlan(
    const std_msgs::msg::Header & header,
    const dwb_msgs::msg::Trajectory2D & traj);
  void publishCostGrid(
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const std::vector<TrajectoryCritic::Ptr> critics);
  void publishGlobalPlan(const nav_2d_msgs::msg::Path2D plan);
  void publishTransformedPlan(const nav_2d_msgs::msg::Path2D plan);
  void publishLocalPlan(const nav_2d_msgs::msg::Path2D plan);

protected:
  void publishTrajectories(const dwb_msgs::msg::LocalPlanEvaluation & results);


  void publishGenericPlan(
    const nav_2d_msgs::msg::Path2D plan,
    rclcpp::Publisher<nav_msgs::msg::Path> & pub, bool flag);


  bool publish_evaluation_;
  bool publish_global_plan_;
  bool publish_transformed_;
  bool publish_local_plan_;
  bool publish_trajectories_;
  bool publish_cost_grid_pc_;
  bool publish_input_params_;


  builtin_interfaces::msg::Duration marker_lifetime_;


  std::shared_ptr<LifecyclePublisher<dwb_msgs::msg::LocalPlanEvaluation>> eval_pub_;
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> transformed_pub_;
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> local_pub_;
  std::shared_ptr<LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
  std::shared_ptr<LifecyclePublisher<sensor_msgs::msg::PointCloud2>> cost_grid_pc_pub_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
};

}

#endif
