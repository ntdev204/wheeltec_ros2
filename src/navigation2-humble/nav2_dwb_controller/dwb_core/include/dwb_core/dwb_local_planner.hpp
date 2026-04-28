


#ifndef DWB_CORE__DWB_LOCAL_PLANNER_HPP_
#define DWB_CORE__DWB_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "dwb_core/publisher.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dwb_core
{



class DWBLocalPlanner : public nav2_core::Controller
{
public:
  

  DWBLocalPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  virtual ~DWBLocalPlanner() {}

  

  void activate() override;

  

  void deactivate() override;

  

  void cleanup() override;

  

  void setPlan(const nav_msgs::msg::Path & path) override;

  

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * ) override;

  

  virtual dwb_msgs::msg::TrajectoryScore scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & traj,
    double best_score = -1);

  

  virtual nav_2d_msgs::msg::Twist2DStamped computeVelocityCommands(
    const nav_2d_msgs::msg::Pose2DStamped & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results);

  

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    if (traj_generator_) {
      traj_generator_->setSpeedLimit(speed_limit, percentage);
    }
  }

protected:
  

  void prepareGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
    nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan = true);

  

  virtual dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results);

  

  virtual nav_2d_msgs::msg::Path2D transformGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose);
  nav_2d_msgs::msg::Path2D global_plan_;
  bool prune_plan_;
  double prune_distance_;
  bool debug_trajectory_details_;
  rclcpp::Duration transform_tolerance_{0, 0};
  bool shorten_transformed_plan_;

  

  std::string resolveCriticClassName(std::string base_name);

  

  virtual void loadCritics();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DWBLocalPlanner")};

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<DWBPublisher> pub_;
  std::vector<std::string> default_critic_namespaces_;


  pluginlib::ClassLoader<TrajectoryGenerator> traj_gen_loader_;
  TrajectoryGenerator::Ptr traj_generator_;

  pluginlib::ClassLoader<TrajectoryCritic> critic_loader_;
  std::vector<TrajectoryCritic::Ptr> critics_;

  std::string dwb_plugin_name_;

  bool short_circuit_trajectory_evaluation_;
};

}

#endif
