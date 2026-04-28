













#include "nav2_collision_monitor/polygon.hpp"

#include <exception>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING),
  slowdown_ratio_(0.0), footprint_sub_(nullptr), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), transform_tolerance_(transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Polygon", polygon_name_.c_str());
}

Polygon::~Polygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Polygon", polygon_name_.c_str());
  poly_.clear();
}

bool Polygon::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string polygon_pub_topic, footprint_topic;

  if (!getParameters(polygon_pub_topic, footprint_topic)) {
    return false;
  }

  if (!footprint_topic.empty()) {
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
      node, footprint_topic, *tf_buffer_,
      base_frame_id_, tf2::durationToSec(transform_tolerance_));
  }

  if (visualize_) {

    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point & p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;

      polygon_.points.push_back(p_s);
    }

    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      polygon_pub_topic, polygon_qos);
  }

  return true;
}

void Polygon::activate()
{
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

void Polygon::deactivate()
{
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

std::string Polygon::getName() const
{
  return polygon_name_;
}

ActionType Polygon::getActionType() const
{
  return action_type_;
}

int Polygon::getMaxPoints() const
{
  return max_points_;
}

double Polygon::getSlowdownRatio() const
{
  return slowdown_ratio_;
}

double Polygon::getTimeBeforeCollision() const
{
  return time_before_collision_;
}

void Polygon::getPolygon(std::vector<Point> & poly) const
{
  poly = poly_;
}

void Polygon::updatePolygon()
{
  if (footprint_sub_ != nullptr) {

    std::vector<geometry_msgs::msg::Point> footprint_vec;
    std_msgs::msg::Header footprint_header;
    footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header);

    std::size_t new_size = footprint_vec.size();
    poly_.resize(new_size);
    polygon_.points.resize(new_size);

    geometry_msgs::msg::Point32 p_s;
    for (std::size_t i = 0; i < new_size; i++) {
      poly_[i] = {footprint_vec[i].x, footprint_vec[i].y};
      p_s.x = footprint_vec[i].x;
      p_s.y = footprint_vec[i].y;
      polygon_.points[i] = p_s;
    }
  }
}

int Polygon::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

double Polygon::getCollisionTime(
  const std::vector<Point> & collision_points,
  const Velocity & velocity) const
{

  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;


  std::vector<Point> points_transformed;


  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {


    projectState(simulation_time_step_, pose, vel);

    points_transformed = collision_points;
    transformPoints(pose, points_transformed);


    if (getPointsInside(points_transformed) > max_points_) {
      return time;
    }
  }


  return -1.0;
}

void Polygon::publish() const
{
  if (!visualize_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }


  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = node->now();
  poly_s->header.frame_id = base_frame_id_;
  poly_s->polygon = polygon_;


  polygon_pub_->publish(std::move(poly_s));
}

bool Polygon::getCommonParameters(std::string & polygon_pub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {


    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".action_type", rclcpp::PARAMETER_STRING);
    const std::string at_str =
      node->get_parameter(polygon_name_ + ".action_type").as_string();
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else if (at_str == "approach") {
      action_type_ = APPROACH;
    } else {
      RCLCPP_ERROR(logger_, "[%s]: Unknown action type: %s", polygon_name_.c_str(), at_str.c_str());
      return false;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();

    if (action_type_ == SLOWDOWN) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
      slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
    }

    if (action_type_ == APPROACH) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".time_before_collision", rclcpp::ParameterValue(2.0));
      time_before_collision_ =
        node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".simulation_time_step", rclcpp::ParameterValue(0.1));
      simulation_time_step_ =
        node->get_parameter(polygon_name_ + ".simulation_time_step").as_double();
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".visualize", rclcpp::ParameterValue(false));
    visualize_ = node->get_parameter(polygon_name_ + ".visualize").as_bool();
    if (visualize_) {

      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name_));
      polygon_pub_topic = node->get_parameter(polygon_name_ + ".polygon_pub_topic").as_string();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting common polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool Polygon::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  try {
    if (action_type_ == APPROACH) {

      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".footprint_topic",
        rclcpp::ParameterValue("local_costmap/published_footprint"));
      footprint_topic =
        node->get_parameter(polygon_name_ + ".footprint_topic").as_string();



      return true;
    } else {

      footprint_topic.clear();
    }


    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row =
      node->get_parameter(polygon_name_ + ".points").as_double_array();

    if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Polygon has incorrect points description",
        polygon_name_.c_str());
      return false;
    }


    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

inline bool Polygon::isPointInside(const Point & point) const
{





  const int poly_size = poly_.size();
  int i, j;
  bool res = false;


  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {



    if ((point.y <= poly_[i].y) == (point.y > poly_[j].y)) {

      const double x_inter = poly_[i].x +
        (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) /
        (poly_[j].y - poly_[i].y);

      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

}
