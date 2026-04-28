














#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

class NavigateToPoseActionServer : public TestActionServer<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseActionServer()
  : TestActionServer("navigate_to_pose")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    goal_handle->succeed(result);
  }
};

class NavigateToPoseActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("navigate_to_pose_action_test_fixture");
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
        return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
          name, "navigate_to_pose", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::NavigateToPoseAction>(
      "NavigateToPose", builder);
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

  static std::shared_ptr<NavigateToPoseActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr NavigateToPoseActionTestFixture::node_ = nullptr;
std::shared_ptr<NavigateToPoseActionServer>
NavigateToPoseActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * NavigateToPoseActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> NavigateToPoseActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> NavigateToPoseActionTestFixture::tree_ = nullptr;

TEST_F(NavigateToPoseActionTestFixture, test_tick)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <NavigateToPose goal="{goal}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;


  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }


  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->pose, pose);


  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);


  pose.pose.position.x = -2.5;
  pose.pose.orientation.x = 1.0;
  config_->blackboard->set<geometry_msgs::msg::PoseStamped>("goal", pose);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(action_server_->getCurrentGoal()->pose, pose);
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);


  NavigateToPoseActionTestFixture::action_server_ =
    std::make_shared<NavigateToPoseActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(NavigateToPoseActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
