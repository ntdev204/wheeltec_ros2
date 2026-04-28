













#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_



#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM


#include <experimental/filesystem>
#include <mutex>
#include <string>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"


namespace nav2_waypoint_follower
{

class PhotoAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  

  PhotoAtWaypoint();

  

  ~PhotoAtWaypoint();

  

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

  

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  

  static void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & mat);

protected:

  std::mutex global_mutex_;

  std::experimental::filesystem::path save_dir_;

  std::string image_format_;

  std::string image_topic_;

  bool is_enabled_;

  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_;

  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscriber_;
};
}

#endif
