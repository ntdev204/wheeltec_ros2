













#ifndef NAV2_COLLISION_MONITOR__POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{



class Polygon
{
public:
  

  Polygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  

  virtual ~Polygon();

  

  bool configure();
  

  void activate();
  

  void deactivate();

  

  std::string getName() const;
  

  ActionType getActionType() const;
  

  int getMaxPoints() const;
  

  double getSlowdownRatio() const;
  

  double getTimeBeforeCollision() const;

  

  virtual void getPolygon(std::vector<Point> & poly) const;

  

  void updatePolygon();

  

  virtual int getPointsInside(const std::vector<Point> & points) const;

  

  double getCollisionTime(
    const std::vector<Point> & collision_points,
    const Velocity & velocity) const;

  

  void publish() const;

protected:
  

  bool getCommonParameters(std::string & polygon_pub_topic);

  

  virtual bool getParameters(std::string & polygon_pub_topic, std::string & footprint_topic);

  

  bool isPointInside(const Point & point) const;




  nav2_util::LifecycleNode::WeakPtr node_;

  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};



  std::string polygon_name_;

  ActionType action_type_;

  int max_points_;

  double slowdown_ratio_;

  double time_before_collision_;

  double simulation_time_step_;

  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;



  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string base_frame_id_;

  tf2::Duration transform_tolerance_;



  bool visualize_;

  geometry_msgs::msg::Polygon polygon_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;


  std::vector<Point> poly_;
};

}

#endif
