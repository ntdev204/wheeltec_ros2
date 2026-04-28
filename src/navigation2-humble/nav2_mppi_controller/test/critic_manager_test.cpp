













#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/critic_manager.hpp"



class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;
using namespace mppi::critics;

class DummyCritic : public CriticFunction
{
public:
  virtual void initialize() {initialized_ = true;}
  virtual void score(CriticData & ) {scored_ = true;}
  bool initialized_{false}, scored_{false};
};

class CriticManagerWrapper : public CriticManager
{
public:
  CriticManagerWrapper()
  : CriticManager() {}

  virtual void loadCritics()
  {
    critics_.clear();
    auto instance = std::unique_ptr<critics::CriticFunction>(new DummyCritic);
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + "DummyCritic", costmap_ros_,
      parameters_handler_);
  }

  std::string getFullNameWrapper(const std::string & name)
  {
    return getFullName(name);
  }

  bool getDummyCriticInitialized()
  {
    return dynamic_cast<DummyCritic *>(critics_[0].get())->initialized_;
  }

  bool getDummyCriticScored()
  {
    return dynamic_cast<DummyCritic *>(critics_[0].get())->scored_;
  }
};

class CriticManagerWrapperEnum : public CriticManager
{
public:
  CriticManagerWrapperEnum()
  : CriticManager() {}

  unsigned int getCriticNum()
  {
    return critics_.size();
  }
};

TEST(CriticManagerTests, BasicCriticOperations)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);


  CriticManagerWrapper critic_manager;
  critic_manager.on_configure(node, "critic_manager", costmap_ros, &param_handler);
  EXPECT_TRUE(critic_manager.getDummyCriticInitialized());


  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs;
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};

  data.fail_flag = true;
  EXPECT_FALSE(critic_manager.getDummyCriticScored());
  data.fail_flag = false;
  critic_manager.evalTrajectoriesScores(data);
  EXPECT_TRUE(critic_manager.getDummyCriticScored());


  EXPECT_EQ(critic_manager.getFullNameWrapper("name"), std::string("mppi::critics::name"));
}

TEST(CriticManagerTests, CriticLoadingTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter(
    "critic_manager.critics",
    rclcpp::ParameterValue(std::vector<std::string>{"ConstraintCritic", "PreferForwardCritic"}));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);


  CriticManagerWrapperEnum critic_manager;
  critic_manager.on_configure(node, "critic_manager", costmap_ros, &param_handler);
  EXPECT_EQ(critic_manager.getCriticNum(), 2u);
}
