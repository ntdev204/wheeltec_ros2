













#ifndef NAV2_UTIL__TEST__TEST_LIFECYCLE_CLI_NODE_HPP_
#define NAV2_UTIL__TEST__TEST_LIFECYCLE_CLI_NODE_HPP_

#include <cstdlib>
#include <memory>
#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/lifecycle_utils.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

class DummyNode : public nav2_util::LifecycleNode
{
public:
  DummyNode()
  : nav2_util::LifecycleNode("nav2_test_cli", "")
  {
    activated = false;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & )
  {
    activated = true;
    return nav2_util::CallbackReturn::SUCCESS;
  }

  bool activated;
};

class Handle
{
public:
  Handle()
  {
    node = std::make_shared<DummyNode>();
    thread = std::make_shared<nav2_util::NodeThread>(node->get_node_base_interface());
  }
  ~Handle()
  {
    thread.reset();
    node.reset();
  }

  std::shared_ptr<nav2_util::NodeThread> thread;
  std::shared_ptr<DummyNode> node;
};

class RclCppFixture
{
public:
  RclCppFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclCppFixture()
  {
    rclcpp::shutdown();
  }
};

RclCppFixture g_rclcppfixture;

TEST(LifeycleCLI, fails_no_node_name)
{
  Handle handle;
  auto rc = system("ros2 run nav2_util lifecycle_bringup");
  (void)rc;
#ifdef _WIN32
  Sleep(1000);
#else
  sleep(1);
#endif

  EXPECT_EQ(handle.node->activated, false);
  SUCCEED();
}

TEST(LifeycleCLI, succeeds_node_name)
{
  Handle handle;
  auto rc = system("ros2 run nav2_util lifecycle_bringup nav2_test_cli");
#ifdef _WIN32
  Sleep(3000);
#else
  sleep(3);
#endif

  (void)rc;
  EXPECT_EQ(handle.node->activated, true);
  SUCCEED();
}

#endif
