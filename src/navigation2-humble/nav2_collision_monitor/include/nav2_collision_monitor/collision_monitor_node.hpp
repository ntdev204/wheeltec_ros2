













#ifndef NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"
#include "nav2_collision_monitor/source.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"

namespace nav2_collision_monitor
{



class CollisionMonitor : public nav2_util::LifecycleNode
{
public:
  

  explicit CollisionMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  

  ~CollisionMonitor();

protected:
  

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  

  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  

  void publishVelocity(const Action & robot_action);

  

  bool getParameters(
    std::string & cmd_vel_in_topic,
    std::string & cmd_vel_out_topic);
  

  bool configurePolygons(
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  

  bool configureSources(
    const std::string & base_frame_id,
    const std::string & odom_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);

  

  void process(const Velocity & cmd_vel_in);

  

  bool processStopSlowdown(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  

  bool processApproach(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  

  void printAction(
    const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  

  void publishPolygons() const;




  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  std::vector<std::shared_ptr<Polygon>> polygons_;


  std::vector<std::shared_ptr<Source>> sources_;



  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;


  bool process_active_;


  Action robot_action_prev_;

  rclcpp::Time stop_stamp_;

  rclcpp::Duration stop_pub_timeout_;
};

}

#endif
