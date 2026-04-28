













#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  TestLifecycleNode(bool bond, std::string name)
  : nav2_util::LifecycleNode(name)
  {
    state = "";
    enable_bond = bond;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & ) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Configured!");
    state = "configured";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & ) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Activated!");
    state = "activated";
    if (enable_bond) {
      createBond();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & ) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Deactivated!");
    state = "deactivated";
    if (enable_bond) {
      destroyBond();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & ) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Cleanup!");
    state = "cleaned up";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & ) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Shutdown!");
    state = "shut down";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & ) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is encountered an error!");
    state = "errored";
    return CallbackReturn::SUCCESS;
  }

  bool bondAllocated()
  {
    return bond_ ? true : false;
  }

  void breakBond()
  {
    bond_->breakBond();
  }

  std::string getState()
  {
    return state;
  }

  bool isBondEnabled()
  {
    return enable_bond;
  }

  bool isBondConnected()
  {
    return bondAllocated() ? !bond_->isBroken() : false;
  }

  std::string state;
  bool enable_bond;
};

class TestFixture
{
public:
  TestFixture(bool bond, std::string node_name)
  {
    lf_node_ = std::make_shared<TestLifecycleNode>(bond, node_name);
    lf_thread_ = std::make_unique<nav2_util::NodeThread>(lf_node_->get_node_base_interface());
  }

  std::shared_ptr<TestLifecycleNode> lf_node_;
  std::unique_ptr<nav2_util::NodeThread> lf_thread_;
};

TEST(LifecycleBondTest, POSITIVE)
{

  rclcpp::Rate(1).sleep();

  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);


  auto fixture = TestFixture(true, "bond_tester");
  auto bond_tester = fixture.lf_node_;

  EXPECT_TRUE(client.startup());


  rclcpp::Rate(5).sleep();
  EXPECT_TRUE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "activated");

  bond_tester->breakBond();


  rclcpp::Rate(5).sleep();
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));
  EXPECT_FALSE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "cleaned up");


  EXPECT_TRUE(client.startup());
  EXPECT_EQ(bond_tester->getState(), "activated");
  EXPECT_TRUE(bond_tester->isBondConnected());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::ACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));


  EXPECT_TRUE(client.reset());
  EXPECT_FALSE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "cleaned up");
}

TEST(LifecycleBondTest, NEGATIVE)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);


  auto fixture = TestFixture(false, "bond_tester");
  auto bond_tester = fixture.lf_node_;
  EXPECT_FALSE(client.startup());
  EXPECT_FALSE(bond_tester->isBondEnabled());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
