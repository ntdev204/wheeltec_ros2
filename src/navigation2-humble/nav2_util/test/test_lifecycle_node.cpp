













#include <memory>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;





TEST(LifecycleNode, RclcppNodeExitsCleanly)
{

  auto node1 = std::make_shared<nav2_util::LifecycleNode>("test_node", "");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  SUCCEED();
}

TEST(LifecycleNode, MultipleRclcppNodesExitCleanly)
{

  auto node1 = std::make_shared<nav2_util::LifecycleNode>("test_node1", "");
  auto node2 = std::make_shared<nav2_util::LifecycleNode>("test_node2", "");

  std::this_thread::sleep_for(std::chrono::seconds(1));
  SUCCEED();
}
