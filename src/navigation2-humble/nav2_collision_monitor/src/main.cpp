













#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav2_collision_monitor/collision_monitor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_collision_monitor::CollisionMonitor>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
