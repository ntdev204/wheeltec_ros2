














#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/goal_updated_controller.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class GoalUpdatedControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {

    geometry_msgs::msg::PoseStamped goal1;
    goal1.header.stamp = node_->now();
    std::vector<geometry_msgs::msg::PoseStamped> poses1;
    poses1.push_back(goal1);
    config_->blackboard->set("goal", goal1);
    config_->blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("goals", poses1);

    bt_node_ = std::make_shared<nav2_behavior_tree::GoalUpdatedController>(
      "goal_updated_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::GoalUpdatedController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::GoalUpdatedController>
GoalUpdatedControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
GoalUpdatedControllerTestFixture::dummy_node_ = nullptr;

TEST_F(GoalUpdatedControllerTestFixture, test_behavior)
{

  geometry_msgs::msg::PoseStamped goal2;
  goal2.header.stamp = node_->now();
  std::vector<geometry_msgs::msg::PoseStamped> poses2;
  poses2.push_back(goal2);


  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);


  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);


  config_->blackboard->set("goal", goal2);
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);


  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);


  config_->blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("goals", poses2);
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
