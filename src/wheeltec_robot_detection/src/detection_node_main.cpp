"""Main entry point for C++ detection node."""

#include "wheeltec_robot_detection/detection_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<wheeltec_robot_detection::DetectionNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
