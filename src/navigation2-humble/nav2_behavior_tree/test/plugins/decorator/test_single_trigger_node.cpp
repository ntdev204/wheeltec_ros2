














#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/single_trigger_node.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class SingleTriggerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    bt_node_ = std::make_shared<nav2_behavior_tree::SingleTrigger>(
      "single_trigger", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::SingleTrigger> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::SingleTrigger>
SingleTriggerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
SingleTriggerTestFixture::dummy_node_ = nullptr;

TEST_F(SingleTriggerTestFixture, test_behavior)
{

  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);


  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);


  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);


  dummy_node_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);



  bt_node_->halt();
  dummy_node_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
