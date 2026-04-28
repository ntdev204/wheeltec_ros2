













#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"

using nav2_behaviors::TimedBehavior;
using nav2_behaviors::Status;
using BehaviorAction = nav2_msgs::action::DummyBehavior;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<BehaviorAction>;

using namespace std::chrono_literals;



class DummyBehavior : public TimedBehavior<BehaviorAction>
{
public:
  DummyBehavior()
  : TimedBehavior<BehaviorAction>(),
    initialized_(false) {}

  ~DummyBehavior() = default;

  Status onRun(const std::shared_ptr<const BehaviorAction::Goal> goal) override
  {

    initialized_ = false;
    command_ = goal->command.data;
    start_time_ = std::chrono::system_clock::now();



    if (command_ == "Testing success" || command_ == "Testing failure on run") {
      initialized_ = true;
      return Status::SUCCEEDED;
    }

    return Status::FAILED;
  }

  Status onCycleUpdate() override
  {




    if (command_ != "Testing success" || !initialized_) {
      return Status::FAILED;
    }



    auto current_time = std::chrono::system_clock::now();
    auto motion_duration = 1s;

    if (current_time - start_time_ >= motion_duration) {

      return Status::SUCCEEDED;
    }

    return Status::RUNNING;
  }

private:
  bool initialized_;
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};



class BehaviorTest : public ::testing::Test
{
protected:
  BehaviorTest() {SetUp();}
  ~BehaviorTest() = default;

  void SetUp() override
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleBehaviorTestNode", rclcpp::NodeOptions());
    node_lifecycle_->declare_parameter(
      "costmap_topic",
      rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
    node_lifecycle_->declare_parameter(
      "footprint_topic",
      rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string costmap_topic, footprint_topic;
    node_lifecycle_->get_parameter("costmap_topic", costmap_topic);
    node_lifecycle_->get_parameter("footprint_topic", footprint_topic);
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_lifecycle_, costmap_topic);
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_lifecycle_, footprint_topic, *tf_buffer_);
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_,
      node_lifecycle_->get_name());

    behavior_ = std::make_shared<DummyBehavior>();
    behavior_->configure(node_lifecycle_, "Behavior", tf_buffer_, collision_checker_);
    behavior_->activate();

    client_ = rclcpp_action::create_client<BehaviorAction>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_graph_interface(),
      node_lifecycle_->get_node_logging_interface(),
      node_lifecycle_->get_node_waitables_interface(), "Behavior");
    std::cout << "Setup complete." << std::endl;
  }

  void TearDown() override {}

  bool sendCommand(const std::string & command)
  {
    if (!client_->wait_for_action_server(4s)) {
      std::cout << "Server not up" << std::endl;
      return false;
    }

    auto goal = BehaviorAction::Goal();
    goal.command.data = command;
    auto future_goal = client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(node_lifecycle_, future_goal) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "failed sending goal" << std::endl;

      return false;
    }

    goal_handle_ = future_goal.get();

    if (!goal_handle_) {
      std::cout << "goal was rejected" << std::endl;

      return false;
    }

    return true;
  }

  Status getOutcome()
  {
    if (getResult().code == rclcpp_action::ResultCode::SUCCEEDED) {
      return Status::SUCCEEDED;
    }

    return Status::FAILED;
  }

  ClientGoalHandle::WrappedResult getResult()
  {
    std::cout << "Getting async result..." << std::endl;
    auto future_result = client_->async_get_result(goal_handle_);
    std::cout << "Waiting on future..." << std::endl;
    rclcpp::spin_until_future_complete(node_lifecycle_, future_result);
    std::cout << "future received!" << std::endl;
    return future_result.get();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<DummyBehavior> behavior_;
  std::shared_ptr<rclcpp_action::Client<BehaviorAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<BehaviorAction>> goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};



TEST_F(BehaviorTest, testingSuccess)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingSequentialFailures)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getOutcome(), Status::FAILED);
  SUCCEED();
}

TEST_F(BehaviorTest, testingTotalElapsedTimeIsGratherThanZeroIfStarted)
{
  ASSERT_TRUE(sendCommand("Testing success"));
  EXPECT_GT(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

TEST_F(BehaviorTest, testingTotalElapsedTimeIsZeroIfFailureOnInit)
{
  ASSERT_TRUE(sendCommand("Testing failure on init"));
  EXPECT_EQ(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

TEST_F(BehaviorTest, testingTotalElapsedTimeIsZeroIfFailureOnRun)
{
  ASSERT_TRUE(sendCommand("Testing failure on run"));
  EXPECT_EQ(getResult().result->total_elapsed_time.sec, 0.0);
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();


  rclcpp::shutdown();

  return all_successful;
}
