#ifndef WHEELTEC_PID_CONTROLLER__VELOCITY_SMOOTHER_HPP_
#define WHEELTEC_PID_CONTROLLER__VELOCITY_SMOOTHER_HPP_

#include <algorithm>
#include <cmath>

namespace wheeltec_smoother
{

/**
 * Single-axis velocity smoother: low-pass filter + rate limiter.
 * NO feedback loop — does not fight with MPPI.
 */
class AxisSmoother
{
public:
  AxisSmoother() = default;

  void configure(double max_vel, double max_accel, double deadband)
  {
    max_vel_ = max_vel;
    max_accel_ = max_accel;
    deadband_ = deadband;
    current_ = 0.0;
  }

  void reset() { current_ = 0.0; }

  /**
   * @brief Smooth a velocity command
   * @param target  raw velocity from MPPI
   * @param dt      time step in seconds
   * @return smoothed velocity
   */
  double smooth(double target, double dt)
  {
    if (dt <= 0.0) return current_;

    // Deadband: zero small commands
    if (std::fabs(target) < deadband_) {
      target = 0.0;
    }

    // Velocity clamp
    target = std::clamp(target, -max_vel_, max_vel_);

    // Rate limiting (acceleration/deceleration)
    double max_change = max_accel_ * dt;
    double diff = target - current_;

    if (std::fabs(diff) > max_change) {
      current_ += std::copysign(max_change, diff);
    } else {
      current_ = target;
    }

    return current_;
  }

  double get_current() const { return current_; }

private:
  double max_vel_ = 1.0;
  double max_accel_ = 1.0;
  double deadband_ = 0.01;
  double current_ = 0.0;
};

}  // namespace wheeltec_smoother

#endif  // WHEELTEC_PID_CONTROLLER__VELOCITY_SMOOTHER_HPP_
