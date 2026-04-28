













#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "test_msgs/action/fibonacci.hpp"
#include "std_msgs/msg/empty.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

using std::placeholders::_1;
using namespace std::chrono_literals;

class FibonacciServerNode : public rclcpp::Node
{
public:
  FibonacciServerNode()
  : rclcpp::Node("fibonacci_server_node")
  {
  }

  ~FibonacciServerNode()
  {
  }

  void on_init()
  {
    action_server_ = std::make_shared<nav2_util::SimpleActionServer<Fibonacci>>(
      shared_from_this(),
      "fibonacci",
      std::bind(&FibonacciServerNode::execute, this));

    deactivate_subs_ = create_subscription<std_msgs::msg::Empty>(
      "deactivate_server",
      1,
      [this](std_msgs::msg::Empty::UniquePtr ) {
        RCLCPP_INFO(this->get_logger(), "Deactivating");
        action_server_->deactivate();
      });

    activate_subs_ = create_subscription<std_msgs::msg::Empty>(
      "activate_server",
      1,
      [this](std_msgs::msg::Empty::UniquePtr ) {
        RCLCPP_INFO(this->get_logger(), "Activating");
        action_server_->activate();
      });

    omit_preempt_subs_ = create_subscription<std_msgs::msg::Empty>(
      "omit_preemption",
      1,
      [this](std_msgs::msg::Empty::UniquePtr ) {
        RCLCPP_INFO(this->get_logger(), "Ignoring preemptions");
        do_premptions_ = false;
      });
  }

  void on_term()
  {







    action_server_.reset();
  }

  void execute()
  {
    rclcpp::Rate loop_rate(10);

preempted:

    auto goal = action_server_->get_current_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();


    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {


      if (action_server_->is_cancel_requested() || !action_server_->is_server_active()) {
        result->sequence = sequence;
        return;
      }


      if (do_premptions_ && action_server_->is_preempt_requested()) {
        action_server_->accept_pending_goal();
        goto preempted;
      }


      sequence.push_back(sequence[i] + sequence[i - 1]);


      action_server_->publish_feedback(feedback);
      loop_rate.sleep();
    }


    if (rclcpp::ok()) {
      result->sequence = sequence;
      action_server_->succeeded_current(result);
    }
  }

private:
  std::shared_ptr<nav2_util::SimpleActionServer<Fibonacci>> action_server_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr deactivate_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr activate_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr omit_preempt_subs_;

  bool do_premptions_{true};
};

class RclCppFixture
{
public:
  RclCppFixture()
  {
  }

  void Setup()
  {
    server_thread_ =
      std::make_shared<std::thread>(std::bind(&RclCppFixture::server_thread_func, this));
  }

  ~RclCppFixture()
  {
    server_thread_->join();
  }

  void server_thread_func()
  {
    auto node = std::make_shared<FibonacciServerNode>();
    node->on_init();
    rclcpp::spin(node->get_node_base_interface());
    node->on_term();
    node.reset();
  }

  std::shared_ptr<std::thread> server_thread_;
};

RclCppFixture g_rclcppfixture;

class ActionTestNode : public rclcpp::Node
{
public:
  ActionTestNode()
  : rclcpp::Node(nav2_util::generate_internal_node_name("action_test_node"))
  {
  }

  void on_init()
  {
    action_client_ = rclcpp_action::create_client<Fibonacci>(shared_from_this(), "fibonacci");
    action_client_->wait_for_action_server();

    deactivate_pub_ = this->create_publisher<std_msgs::msg::Empty>("deactivate_server", 1);
    activate_pub_ = this->create_publisher<std_msgs::msg::Empty>("activate_server", 1);
    omit_prempt_pub_ = this->create_publisher<std_msgs::msg::Empty>("omit_preemption", 1);
  }

  void on_term()
  {
    action_client_.reset();
  }

  void deactivate_server()
  {
    deactivate_pub_->publish(std_msgs::msg::Empty());
  }

  void activate_server()
  {
    activate_pub_->publish(std_msgs::msg::Empty());
  }

  void omit_server_preemptions()
  {
    omit_prempt_pub_->publish(std_msgs::msg::Empty());
  }

  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr deactivate_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr activate_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr omit_prempt_pub_;
};

class ActionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<ActionTestNode>();
    node_->on_init();
  }

  void TearDown() override
  {
    std::cout << " Teardown" << std::endl;
    node_->on_term();
    std::cout << " Teardown..." << std::endl;
    node_.reset();
    std::cout << " Teardown complete" << std::endl;
  }

  std::shared_ptr<ActionTestNode> node_;
};

TEST_F(ActionTest, test_simple_action)
{
  node_->activate_server();


  auto goal = Fibonacci::Goal();
  goal.order = 12;


  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = future_goal_handle.get();


  auto future_result = node_->action_client_->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, future_result),
    rclcpp::FutureReturnCode::SUCCESS);


  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);


  int sum = 0;
  for (auto number : result.result->sequence) {
    sum += number;
  }

  EXPECT_EQ(sum, 376);
  SUCCEED();
}

TEST_F(ActionTest, test_simple_action_with_feedback)
{
  int feedback_sum = 0;


  auto feedback_callback = [&feedback_sum](
    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr ,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      feedback_sum += feedback->sequence.back();
    };


  auto goal = Fibonacci::Goal();
  goal.order = 10;

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  send_goal_options.feedback_callback = feedback_callback;


  auto future_goal_handle = node_->action_client_->async_send_goal(goal, send_goal_options);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = future_goal_handle.get();


  auto future_result = node_->action_client_->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_result), rclcpp::FutureReturnCode::SUCCESS);


  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);


  int sum = 0;
  for (auto number : result.result->sequence) {
    sum += number;
  }

  EXPECT_EQ(sum, 143);
  EXPECT_GE(feedback_sum, 0);
  SUCCEED();
}

TEST_F(ActionTest, test_simple_action_activation_cycling)
{

  auto goal = Fibonacci::Goal();


  goal.order = 12'000'000;




  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);


  node_->deactivate_server();

  auto goal_handle = future_goal_handle.get();


  auto future_result = node_->action_client_->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, future_result),
    rclcpp::FutureReturnCode::SUCCESS);


  EXPECT_EQ(future_result.get().code, rclcpp_action::ResultCode::ABORTED);


  node_->activate_server();

  goal.order = 12;


  future_goal_handle = node_->action_client_->async_send_goal(goal);
  std::cout << "Sent goal, spinning til complete..." << std::endl;
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  goal_handle = future_goal_handle.get();


  future_result = node_->action_client_->async_get_result(goal_handle);
  std::cout << "Getting result, spinning til complete..." << std::endl;
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, future_result),
    rclcpp::FutureReturnCode::SUCCESS);


  EXPECT_EQ(future_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  SUCCEED();
}

TEST_F(ActionTest, test_simple_action_preemption)
{

  auto goal = Fibonacci::Goal();


  goal.order = 12'000'000;


  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  std::cout << "Sent goal, spinning til complete..." << std::endl;
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);


  auto preemption_goal = Fibonacci::Goal();
  preemption_goal.order = 1;


  future_goal_handle = node_->action_client_->async_send_goal(preemption_goal);
  std::cout << "Sent goal, spinning til complete..." << std::endl;
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = future_goal_handle.get();


  auto future_result = node_->action_client_->async_get_result(goal_handle);
  std::cout << "Getting result, spinning til complete..." << std::endl;
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, future_result),
    rclcpp::FutureReturnCode::SUCCESS);


  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);


  int sum = 0;
  for (auto number : result.result->sequence) {
    sum += number;
  }

  EXPECT_EQ(sum, 1);
  SUCCEED();
}

TEST_F(ActionTest, test_simple_action_preemption_after_succeeded)
{

  auto goal = Fibonacci::Goal();
  goal.order = 20;

  auto preemption = Fibonacci::Goal();
  preemption.order = 1;


  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  node_->omit_server_preemptions();

  auto future_preempt_handle = node_->action_client_->async_send_goal(preemption);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);


  auto goal_handle = future_goal_handle.get();


  auto future_result = node_->action_client_->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, future_result),
    rclcpp::FutureReturnCode::SUCCESS);


  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);


  int sum = 0;
  for (auto number : result.result->sequence) {
    sum += number;
  }

  EXPECT_EQ(sum, 17710);


  goal_handle = future_preempt_handle.get();


  future_result = node_->action_client_->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node_, future_result),
    rclcpp::FutureReturnCode::SUCCESS);


  result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);


  sum = 0;
  for (auto number : result.result->sequence) {
    sum += number;
  }

  EXPECT_EQ(sum, 1);
  SUCCEED();
}

TEST_F(ActionTest, test_handle_goal_deactivated)
{
  node_->deactivate_server();
  auto goal = Fibonacci::Goal();
  goal.order = 12;


  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);

  node_->activate_server();

  SUCCEED();
}

TEST_F(ActionTest, test_handle_cancel)
{
  auto goal = Fibonacci::Goal();
  goal.order = 14000000;


  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      future_goal_handle), rclcpp::FutureReturnCode::SUCCESS);


  auto cancel_response = node_->action_client_->async_cancel_goal(future_goal_handle.get());
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node_,
      cancel_response), rclcpp::FutureReturnCode::SUCCESS);


  EXPECT_EQ(future_goal_handle.get()->get_status(), rclcpp_action::GoalStatus::STATUS_CANCELING);

  SUCCEED();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_rclcppfixture.Setup();
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  rclcpp::Rate(1).sleep();
  return result;
}
