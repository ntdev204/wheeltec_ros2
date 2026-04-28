



#ifndef RPLIDAR_NODE_HPP_
#define RPLIDAR_NODE_HPP_

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <rplidar.h>
#include <visibility.h>

#include <chrono>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define M_PI 3.1415926535897932384626433832795

namespace
{
using LaserScan = sensor_msgs::msg::LaserScan;
using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
using StartMotorService = rclcpp::Service<std_srvs::srv::Empty>::SharedPtr;
using StopMotorService = rclcpp::Service<std_srvs::srv::Empty>::SharedPtr;
using RPlidarDriver = rp::standalone::rplidar::RPlidarDriver;
using RplidarScanMode = rp::standalone::rplidar::RplidarScanMode;
using Clock = rclcpp::Clock::SharedPtr;
using ResponseNodeArray = std::unique_ptr<rplidar_response_measurement_node_hq_t[]>;
using EmptyRequest = std::shared_ptr<std_srvs::srv::Empty::Request>;
using EmptyResponse = std::shared_ptr<std_srvs::srv::Empty::Response>;
using Timer = rclcpp::TimerBase::SharedPtr;
using namespace std::chrono_literals;
}

namespace rplidar_ros
{

constexpr double deg_2_rad(double x)
{
  return x * M_PI / 180.0;
}

static float getAngle(const rplidar_response_measurement_node_hq_t & node)
{
  return node.angle_z_q14 * 90.f / 16384.f;
}

class RPLIDAR_ROS_PUBLIC rplidar_node : public rclcpp::Node
{
public:
  explicit rplidar_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~rplidar_node();

  void publish_scan(const double scan_time, ResponseNodeArray nodes, size_t node_count);

  
  void stop_motor(const EmptyRequest req, EmptyResponse res);
  void start_motor(const EmptyRequest req, EmptyResponse res);

private:
  bool getRPLIDARDeviceInfo() const;
  bool checkRPLIDARHealth() const;
  bool set_scan_mode();
  void publish_loop();
  void start();
  void stop();

  
  std::string channel_type_;
  std::string tcp_ip_;
  std::string serial_port_;
  std::string topic_name_;
  int tcp_port_;
  int serial_baudrate_;
  std::string frame_id_;
  bool inverted_;
  bool angle_compensate_;
  bool flip_x_axis_;
  int m_angle_compensate_multiple;
  std::string scan_mode_;
  bool auto_standby_;
  
  LaserScanPub m_publisher;
  
  StopMotorService m_stop_motor_service;
  StartMotorService m_start_motor_service;
  
  RPlidarDriver * m_drv = nullptr;
  
  Timer m_timer;
  
  size_t m_scan_count = 0;
  double max_distance = 8.0f;
  double angle_min = deg_2_rad(0);
  double angle_max = deg_2_rad(359);
  const float min_distance = 0.15f;
  
  bool m_running = false;
};

}

#endif
