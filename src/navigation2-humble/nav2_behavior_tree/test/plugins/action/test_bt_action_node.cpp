














#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class FibonacciActionServer : public rclcpp::Node
{
public:
  FibonacciActionServer()
  : rclcpp::Node("fibonacci_node", rclcpp::NodeOptions()),
    sleep_duration_(0ms)
  {
    this->action_server_ = rclcpp_action::create_server<test_msgs::action::Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

  void setHandleGoalSleepDuration(std::chrono::milliseconds sleep_duration)
  {
    sleep_duration_ = sleep_duration;
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal>)
  {
    if (sleep_duration_ > 0ms) {
      std::this_thread::sleep_for(sleep_duration_);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> handle)
  {

    if (handle) {
      const auto goal = handle->get_goal();
      auto result = std::make_shared<test_msgs::action::Fibonacci::Result>();

      if (goal->order < 0) {
        handle->abort(result);
        return;
      }

      auto & sequence = result->sequence;
      sequence.push_back(0);
      sequence.push_back(1);

      for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
        sequence.push_back(sequence[i] + sequence[i - 1]);
      }

      handle->succeed(result);
    }
  }

protected:
  rclcpp_action::Server<test_msgs::action::Fibonacci>::SharedPtr action_server_;
  std::chrono::milliseconds sleep_duration_;
};

class FibonacciAction : public nav2_behavior_tree::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  FibonacciAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<test_msgs::action::Fibonacci>(xml_tag_name, "fibonacci", conf)
  {}

  void on_tick() override
  {
    getInput("order", goal_.order);
  }

  BT::NodeStatus on_success() override
  {
    config().blackboard->set<std::vector<int>>("sequence", result_.result->sequence);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("order", "Fibonacci order")});
  }
};

class BTActionNodeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_action_node_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();


    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
    config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<FibonacciAction>(name, config);
      };

    factory_->registerBuilder<FibonacciAction>("Fibonacci", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {

    action_server_ = std::make_shared<FibonacciActionServer>();
    server_thread_ = std::make_shared<std::thread>(
      []() {
        while (rclcpp::ok() && BTActionNodeTestFixture::action_server_ != nullptr) {
          rclcpp::spin_some(BTActionNodeTestFixture::action_server_);
          std::this_thread::sleep_for(100ns);
        }
      });
  }

  void TearDown() override
  {
    action_server_.reset();
    tree_.reset();
    server_thread_->join();
    server_thread_.reset();
  }

  static std::shared_ptr<FibonacciActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<std::thread> server_thread_;
};

rclcpp::Node::SharedPtr BTActionNodeTestFixture::node_ = nullptr;
std::shared_ptr<FibonacciActionServer> BTActionNodeTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * BTActionNodeTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> BTActionNodeTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> BTActionNodeTestFixture::tree_ = nullptr;
std::shared_ptr<std::thread> BTActionNodeTestFixture::server_thread_ = nullptr;

TEST_F(BTActionNodeTestFixture, test_server_timeout_success)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="5" />
        </BehaviorTree>
      </root>)";


  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


  action_server_->setHandleGoalSleepDuration(2ms);


  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;


  rclcpp::WallRate loopRate(10ms);


  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }


  auto sequence = config_->blackboard->get<std::vector<int>>("sequence");


  std::vector<int> expected = {0, 1, 1, 2, 3, 5};



  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);


  EXPECT_EQ(sequence.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(sequence[i], expected[i]);
  }





  tree_->haltTree();


  action_server_->setHandleGoalSleepDuration(100ms);


  ticks = 0;
  result = BT::NodeStatus::RUNNING;


  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }



  EXPECT_EQ(result, BT::NodeStatus::FAILURE);


  EXPECT_EQ(ticks, 2);
}

TEST_F(BTActionNodeTestFixture, test_server_timeout_failure)
{

  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="2" />
        </BehaviorTree>
      </root>)";



  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 90ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


  action_server_->setHandleGoalSleepDuration(100ms);


  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;


  rclcpp::WallRate loopRate(10ms);


  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }



  EXPECT_EQ(result, BT::NodeStatus::FAILURE);


  EXPECT_EQ(ticks, 9);





  tree_->haltTree();


  action_server_->setHandleGoalSleepDuration(25ms);


  ticks = 0;
  result = BT::NodeStatus::RUNNING;


  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }



  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
