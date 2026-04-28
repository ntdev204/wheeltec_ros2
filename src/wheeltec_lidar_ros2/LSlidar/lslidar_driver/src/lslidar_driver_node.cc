


#include "rclcpp/rclcpp.hpp"
#include "lslidar_driver/lslidar_driver.h"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lslidar_driver::LslidarDriver>();
  
  while (rclcpp::ok() && node->polling()) {
        rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
