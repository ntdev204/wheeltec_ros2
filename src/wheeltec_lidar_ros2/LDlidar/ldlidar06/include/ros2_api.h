

#ifndef __ROS_API_H__
#define __ROS_API_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

struct LaserScanSetting
{
  std::string frame_id;
  bool laser_scan_dir;
  bool enable_angle_crop_func;
  double angle_crop_min;
  double angle_crop_max;
};

#endif


