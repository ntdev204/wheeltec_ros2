














#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"

class TestNode : public BT::SyncActionNode
{
public:
  TestNode(const std::string & name, const BT::NodeConfiguration & config)
  : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick()
  {
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }
};

class InitialPoseReceivedConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    test_node_ = std::make_shared<TestNode>("TestNode", *config_);
  }

  void TearDown()
  {
    test_node_.reset();
  }

protected:
  static std::shared_ptr<TestNode> test_node_;
};

std::shared_ptr<TestNode> InitialPoseReceivedConditionTestFixture::test_node_ = nullptr;

TEST_F(InitialPoseReceivedConditionTestFixture, test_behavior)
{
  EXPECT_EQ(nav2_behavior_tree::initialPoseReceived(*test_node_), BT::NodeStatus::FAILURE);
  config_->blackboard->set("initial_pose_received", true);
  EXPECT_EQ(nav2_behavior_tree::initialPoseReceived(*test_node_), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
