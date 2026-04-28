


#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include <exception>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace nav2_costmap_2d
{

CostmapFilter::CostmapFilter()
: filter_info_topic_(""), mask_topic_("")
{
  access_ = new mutex_t();
}

CostmapFilter::~CostmapFilter()
{
  delete access_;
}

void CostmapFilter::onInitialize()
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("filter_info_topic", rclcpp::PARAMETER_STRING);
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));


    node->get_parameter(name_ + "." + "enabled", enabled_);
    filter_info_topic_ = node->get_parameter(name_ + "." + "filter_info_topic").as_string();
    double transform_tolerance;
    node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance);
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);


    enable_service_ = node->create_service<std_srvs::srv::SetBool>(
      name_ + "/toggle_filter",
      std::bind(
        &CostmapFilter::enableCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Parameter problem: %s", ex.what());
    throw ex;
  }
}

void CostmapFilter::activate()
{
  initializeFilter(filter_info_topic_);
}

void CostmapFilter::deactivate()
{
  resetFilter();
}

void CostmapFilter::reset()
{
  resetFilter();
  initializeFilter(filter_info_topic_);
  current_ = false;
}

void CostmapFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * , double * , double * , double * )
{
  if (!enabled_) {
    return;
  }

  latest_pose_.x = robot_x;
  latest_pose_.y = robot_y;
  latest_pose_.theta = robot_yaw;
}

void CostmapFilter::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
  current_ = true;
}

void CostmapFilter::enableCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  response->success = true;
  if (enabled_) {
    response->message = "Enabled";
  } else {
    response->message = "Disabled";
  }
}

bool CostmapFilter::transformPose(
  const std::string global_frame,
  const geometry_msgs::msg::Pose2D & global_pose,
  const std::string mask_frame,
  geometry_msgs::msg::Pose2D & mask_pose) const
{
  if (mask_frame != global_frame) {



    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::PointStamped in, out;
    in.header.stamp = clock_->now();
    in.header.frame_id = global_frame;
    in.point.x = global_pose.x;
    in.point.y = global_pose.y;
    in.point.z = 0;

    try {
      tf_->transform(in, out, mask_frame, transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "CostmapFilter: failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame.c_str(), mask_frame.c_str(), ex.what());
      return false;
    }
    mask_pose.x = out.point.x;
    mask_pose.y = out.point.y;
  } else {


    mask_pose = global_pose;
  }

  return true;
}

bool CostmapFilter::worldToMask(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  double origin_x = filter_mask->info.origin.position.x;
  double origin_y = filter_mask->info.origin.position.y;
  double resolution = filter_mask->info.resolution;
  unsigned int size_x = filter_mask->info.width;
  unsigned int size_y = filter_mask->info.height;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = std::round((wx - origin_x) / resolution);
  my = std::round((wy - origin_y) / resolution);
  if (mx >= size_x || my >= size_y) {
    return false;
  }

  return true;
}

}
