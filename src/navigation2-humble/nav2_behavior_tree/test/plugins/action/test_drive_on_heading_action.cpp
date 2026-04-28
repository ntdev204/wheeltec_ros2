














#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/drive_on_heading_action.hpp"

class DriveOnHeadingActionServer : public TestActionServer<nav2_msgs::action::DriveOnHeading>
{
public:
  DriveOnHeadingActionServer()
  : TestActionServer("drive_on_heading")
  {}

protected:
  void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle
    <nav2_msgs::action::DriveOnHeading>>
    goal_handle)
  override
  {
    nav2_msgs::action::DriveOnHeading::Result::SharedPtr result =
      std::make_shared<nav2_msgs::action::DriveOnHeading::Result>();
    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

class DriveOnHeadingActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("drive_on_heading_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();


    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::DriveOnHeadingAction>(
          name, "drive_on_heading", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::DriveOnHeadingAction>("DriveOnHeading", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<DriveOnHeadingActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr DriveOnHeadingActionTestFixture::node_ = nullptr;
std::shared_ptr<DriveOnHeadingActionServer>
DriveOnHeadingActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * DriveOnHeadingActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> DriveOnHeadingActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> DriveOnHeadingActionTestFixture::tree_ = nullptr;

TEST_F(DriveOnHeadingActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <DriveOnHeading />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("dist_to_travel"), 0.15);
  EXPECT_EQ(tree_->rootNode()->getInput<double>("speed"), 0.025);
  EXPECT_EQ(tree_->rootNode()->getInput<double>("time_allowance"), 10.0);

  xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <DriveOnHeading dist_to_travel="2" speed="0.26" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("dist_to_travel"), 2.0);
  EXPECT_EQ(tree_->rootNode()->getInput<double>("speed"), 0.26);
}

TEST_F(DriveOnHeadingActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <DriveOnHeading dist_to_travel="2" speed="0.26" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  auto goal = action_server_->getCurrentGoal();
  EXPECT_EQ(goal->target.x, 2.0);
  EXPECT_EQ(goal->speed, 0.26f);
}

TEST_F(DriveOnHeadingActionTestFixture, test_failure)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <DriveOnHeading dist_to_travel="2" speed="0.26" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  action_server_->setReturnSuccess(false);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);

  auto goal = action_server_->getCurrentGoal();
  EXPECT_EQ(goal->target.x, 2.0);
  EXPECT_EQ(goal->speed, 0.26f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);


  DriveOnHeadingActionTestFixture::action_server_ = std::make_shared<DriveOnHeadingActionServer>();
  std::thread server_thread([]() {
      rclcpp::spin(DriveOnHeadingActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
