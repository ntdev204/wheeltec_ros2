













#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
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

TEST(MotionModelTests, DiffDriveTest)
{
  models::ControlSequence control_sequence;
  models::State state;
  int batches = 1000;
  int timesteps = 50;
  control_sequence.reset(timesteps);
  state.reset(batches, timesteps);
  std::unique_ptr<DiffDriveMotionModel> model =
    std::make_unique<DiffDriveMotionModel>();


  state.cvx = 10 * xt::ones<float>({batches, timesteps});
  state.cvy = 5 * xt::ones<float>({batches, timesteps});
  state.cwz = 1 * xt::ones<float>({batches, timesteps});


  xt::view(state.vx, xt::all(), 0) = 10;
  xt::view(state.wz, xt::all(), 0) = 1;

  model->predict(state);

  EXPECT_EQ(state.vx, state.cvx);
  EXPECT_EQ(state.vy, xt::zeros<float>({batches, timesteps}));
  EXPECT_EQ(state.wz, state.cwz);


  for (unsigned int i = 0; i != control_sequence.vx.shape(0); i++) {
    control_sequence.vx(i) = i * i * i;
    control_sequence.wz(i) = i * i * i;
  }

  models::ControlSequence initial_control_sequence = control_sequence;
  model->applyConstraints(control_sequence);
  EXPECT_EQ(initial_control_sequence.vx, control_sequence.vx);
  EXPECT_EQ(initial_control_sequence.vy, control_sequence.vy);
  EXPECT_EQ(initial_control_sequence.wz, control_sequence.wz);


  EXPECT_EQ(model->isHolonomic(), false);


  model.reset();
}

TEST(MotionModelTests, OmniTest)
{
  models::ControlSequence control_sequence;
  models::State state;
  int batches = 1000;
  int timesteps = 50;
  control_sequence.reset(timesteps);
  state.reset(batches, timesteps);
  std::unique_ptr<OmniMotionModel> model =
    std::make_unique<OmniMotionModel>();


  state.cvx = 10 * xt::ones<float>({batches, timesteps});
  state.cvy = 5 * xt::ones<float>({batches, timesteps});
  state.cwz = 1 * xt::ones<float>({batches, timesteps});


  xt::view(state.vx, xt::all(), 0) = 10;
  xt::view(state.vy, xt::all(), 0) = 5;
  xt::view(state.wz, xt::all(), 0) = 1;

  model->predict(state);

  EXPECT_EQ(state.vx, state.cvx);
  EXPECT_EQ(state.vy, state.cvy);
  EXPECT_EQ(state.wz, state.cwz);


  for (unsigned int i = 0; i != control_sequence.vx.shape(0); i++) {
    control_sequence.vx(i) = i * i * i;
    control_sequence.vy(i) = i * i * i;
    control_sequence.wz(i) = i * i * i;
  }

  models::ControlSequence initial_control_sequence = control_sequence;
  model->applyConstraints(control_sequence);
  EXPECT_EQ(initial_control_sequence.vx, control_sequence.vx);
  EXPECT_EQ(initial_control_sequence.vy, control_sequence.vy);
  EXPECT_EQ(initial_control_sequence.wz, control_sequence.wz);


  EXPECT_EQ(model->isHolonomic(), true);


  model.reset();
}

TEST(MotionModelTests, AckermannTest)
{
  models::ControlSequence control_sequence;
  models::State state;
  int batches = 1000;
  int timesteps = 50;
  control_sequence.reset(timesteps);
  state.reset(batches, timesteps);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  ParametersHandler param_handler(node);
  std::unique_ptr<AckermannMotionModel> model =
    std::make_unique<AckermannMotionModel>(&param_handler);


  state.cvx = 10 * xt::ones<float>({batches, timesteps});
  state.cvy = 5 * xt::ones<float>({batches, timesteps});
  state.cwz = 1 * xt::ones<float>({batches, timesteps});


  xt::view(state.vx, xt::all(), 0) = 10;
  xt::view(state.wz, xt::all(), 0) = 1;

  model->predict(state);

  EXPECT_EQ(state.vx, state.cvx);
  EXPECT_EQ(state.vy, xt::zeros<float>({batches, timesteps}));
  EXPECT_EQ(state.wz, state.cwz);


  for (unsigned int i = 0; i != control_sequence.vx.shape(0); i++) {
    control_sequence.vx(i) = i * i * i;
    control_sequence.wz(i) = i * i * i * i;
  }

  models::ControlSequence initial_control_sequence = control_sequence;
  model->applyConstraints(control_sequence);

  EXPECT_EQ(initial_control_sequence.vx, control_sequence.vx);
  EXPECT_NE(initial_control_sequence.wz, control_sequence.wz);


  EXPECT_NEAR(model->getMinTurningRadius(), 0.2, 1e-6);
  for (unsigned int i = 1; i != control_sequence.vx.shape(0); i++) {
    EXPECT_TRUE(fabs(control_sequence.vx(i)) / fabs(control_sequence.wz(i)) >= 0.2);
  }


  EXPECT_EQ(model->isHolonomic(), false);


  model.reset();
}
