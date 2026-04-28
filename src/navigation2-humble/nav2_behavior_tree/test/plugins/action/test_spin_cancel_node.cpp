













#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/spin_cancel_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class CancelSpinServer : public TestActionServer<nav2_msgs::action::Spin>
{
public:
  CancelSpinServer()
  : TestActionServer("spin")
  {}

protected:
  void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::Spin>>
    goal_handle)
  {
    while (!goal_handle->is_canceling()) {

      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  }
};

class CancelSpinActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("cancel_spin_action_test_fixture");
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
    client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(
      node_, "spin");

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::SpinCancel>(
          name, "spin", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::SpinCancel>("CancelSpin", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    client_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<CancelSpinServer> action_server_;
  static std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::Spin>> client_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr CancelSpinActionTestFixture::node_ = nullptr;
std::shared_ptr<CancelSpinServer>
CancelSpinActionTestFixture::action_server_ = nullptr;
std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::Spin>>
CancelSpinActionTestFixture::client_ = nullptr;

BT::NodeConfiguration * CancelSpinActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
CancelSpinActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> CancelSpinActionTestFixture::tree_ = nullptr;

TEST_F(CancelSpinActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
             <CancelSpin name="SpinCancel"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();


  auto goal_msg = nav2_msgs::action::Spin::Goal();


  goal_msg.target_yaw = 1.57;


  client_->wait_for_action_server();
  client_->async_send_goal(goal_msg, send_goal_options);


  std::this_thread::sleep_for(std::chrono::milliseconds(15));


  tree_->rootNode()->executeTick();


  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);


  EXPECT_EQ(action_server_->isGoalCancelled(), true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);


  CancelSpinActionTestFixture::action_server_ = std::make_shared<CancelSpinServer>();
  std::thread server_thread([]() {
      rclcpp::spin(CancelSpinActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
