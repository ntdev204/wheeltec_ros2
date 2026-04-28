















#ifndef NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_
#define NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_navfn_planner
{

class NavfnPlanner : public nav2_core::GlobalPlanner
{
public:
  

  NavfnPlanner();

  

  ~NavfnPlanner();

  

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  

  void cleanup() override;

  

  void activate() override;

  

  void deactivate() override;


  

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  

  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav_msgs::msg::Path & plan);

  

  bool computePotential(const geometry_msgs::msg::Point & world_point);

  

  bool getPlanFromPotential(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  

  void smoothApproachToGoal(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  

  double getPointPotential(const geometry_msgs::msg::Point & world_point);







  

  inline double squared_distance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2)
  {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    return dx * dx + dy * dy;
  }

  

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  

  void mapToWorld(double mx, double my, double & wx, double & wy);

  

  void clearRobotCell(unsigned int mx, unsigned int my);

  

  bool isPlannerOutOfDate();


  std::unique_ptr<NavFn> planner_;


  std::shared_ptr<tf2_ros::Buffer> tf_;


  rclcpp::Clock::SharedPtr clock_;


  rclcpp::Logger logger_{rclcpp::get_logger("NavfnPlanner")};


  nav2_costmap_2d::Costmap2D * costmap_;


  std::string global_frame_, name_;


  bool allow_unknown_, use_final_approach_orientation_;



  double tolerance_;


  bool use_astar_;


  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}

#endif
