













#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/noise_generator.hpp"
#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"



class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;

TEST(NoiseGeneratorTest, NoiseGeneratorLifecycle)
{

  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;

  generator.initialize(settings, false);
  generator.shutdown();
}

TEST(NoiseGeneratorTest, NoiseGeneratorMain)
{

  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;


  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(25);
  for (unsigned int i = 0; i != control_sequence.vx.shape(0); i++) {
    control_sequence.vx(i) = i;
    control_sequence.vy(i) = i;
    control_sequence.wz(i) = i;
  }

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);


  generator.initialize(settings, false);
  generator.reset(settings, false);
  generator.setNoisedControls(state, control_sequence);
  EXPECT_EQ(state.cvx(0), 0);
  EXPECT_EQ(state.cvy(0), 0);
  EXPECT_EQ(state.cwz(0), 0);
  EXPECT_EQ(state.cvx(9), 9);
  EXPECT_EQ(state.cvy(9), 9);
  EXPECT_EQ(state.cwz(9), 9);


  generator.generateNextNoises();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);
  EXPECT_NE(state.cvx(0), 0);
  EXPECT_EQ(state.cvy(0), 0);
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(9), 9);
  EXPECT_EQ(state.cvy(9), 9);
  EXPECT_NE(state.cwz(9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cvy(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(9), 9, 0.3);
  EXPECT_NEAR(state.cvy(9), 9, 0.3);
  EXPECT_NEAR(state.cwz(9), 9, 0.3);


  generator.reset(settings, true);
  generator.generateNextNoises();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);
  EXPECT_NE(state.cvx(0), 0);
  EXPECT_NE(state.cvy(0), 0);
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(9), 9);
  EXPECT_NE(state.cvy(9), 9);
  EXPECT_NE(state.cwz(9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cvy(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(9), 9, 0.3);
  EXPECT_NEAR(state.cvy(9), 9, 0.3);
  EXPECT_NEAR(state.cwz(9), 9, 0.3);

  generator.shutdown();
}
