













#include <vector>
#include <memory>
#include "nav2_smoother/savitzky_golay_smoother.hpp"

namespace nav2_smoother
{

using namespace smoother_utils;
using namespace nav2_util::geometry_utils;
using namespace std::chrono;
using nav2_util::declare_parameter_if_not_declared;

void SavitzkyGolaySmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(
    node, name + ".do_refinement", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name + ".refinement_num", rclcpp::ParameterValue(2));
  node->get_parameter(name + ".do_refinement", do_refinement_);
  node->get_parameter(name + ".refinement_num", refinement_num_);
}

bool SavitzkyGolaySmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 9) {

      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));


      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();

      if (time_remaining <= 0.0) {
        RCLCPP_WARN(
          logger_,
          "Smoothing time exceeded allowed duration of %0.2f.", max_time.seconds());
        return false;
      }


      success = success && smoothImpl(curr_path_segment, reversing_segment);


      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

bool SavitzkyGolaySmoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{

  const unsigned int & path_size = path.poses.size();


  const std::array<double, 7> filter = {
    -2.0 / 21.0,
    3.0 / 21.0,
    6.0 / 21.0,
    7.0 / 21.0,
    6.0 / 21.0,
    3.0 / 21.0,
    -2.0 / 21.0};

  auto applyFilter = [&](const std::vector<geometry_msgs::msg::Point> & data)
    -> geometry_msgs::msg::Point
    {
      geometry_msgs::msg::Point val;
      for (unsigned int i = 0; i != filter.size(); i++) {
        val.x += filter[i] * data[i].x;
        val.y += filter[i] * data[i].y;
      }
      return val;
    };

  auto applyFilterOverAxes =
    [&](std::vector<geometry_msgs::msg::PoseStamped> & plan_pts) -> void
    {

      unsigned int idx = 1;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});


      for (idx = 3; idx < path_size - 4; ++idx) {
        plan_pts[idx].pose.position = applyFilter(
        {
          plan_pts[idx - 3].pose.position,
          plan_pts[idx - 2].pose.position,
          plan_pts[idx - 1].pose.position,
          plan_pts[idx].pose.position,
          plan_pts[idx + 1].pose.position,
          plan_pts[idx + 2].pose.position,
          plan_pts[idx + 3].pose.position});
      }


      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 2].pose.position});

      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position});
    };

  applyFilterOverAxes(path.poses);


  if (do_refinement_) {
    for (int i = 0; i < refinement_num_; i++) {
      applyFilterOverAxes(path.poses);
    }
  }

  updateApproximatePathOrientations(path, reversing_segment);
  return true;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smoother::SavitzkyGolaySmoother, nav2_core::Smoother)
