













#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

using nav2_util::startup_lifecycle_nodes;
using nav2_util::reset_lifecycle_nodes;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

void SpinNodesUntilDone(
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes,
  std::atomic<bool> * test_done)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  for (const auto & node : nodes) {
    exec.add_node(node->get_node_base_interface());
  }
  while (rclcpp::ok() && !(*test_done)) {
    exec.spin_some();
  }
}

TEST(Lifecycle, interface)
{
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes;
  nodes.push_back(rclcpp_lifecycle::LifecycleNode::make_shared("foo"));
  nodes.push_back(rclcpp_lifecycle::LifecycleNode::make_shared("bar"));

  std::atomic<bool> done(false);
  std::thread node_thread(SpinNodesUntilDone, nodes, &done);
  startup_lifecycle_nodes("/foo:/bar");
  reset_lifecycle_nodes("/foo:/bar");
  done = true;
  node_thread.join();
  SUCCEED();
}
