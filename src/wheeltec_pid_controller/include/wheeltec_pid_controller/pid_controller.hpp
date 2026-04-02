#ifndef WHEELTEC_PID_CONTROLLER__PID_CONTROLLER_HPP_
#define WHEELTEC_PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace wheeltec_pid
{

struct PidGains
{
  double kp = 1.0;
  double ki = 0.1;
  double kd = 0.05;
  double max_integral = 0.3;
  double deadband = 0.01;
};

class PidAxis
{
public:
  PidAxis() = default;

  void set_gains(const PidGains & g)
  {
    gains_ = g;
    reset();
  }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_update_ = true;
  }

  /**
   * @brief Compute PID output
   * @param setpoint  desired velocity (from Nav2 cmd_vel_raw)
   * @param feedback  actual velocity (from odom)
   * @param dt        time step in seconds
   * @return corrected velocity command
   */
  double compute(double setpoint, double feedback, double dt)
  {
    if (dt <= 0.0) {
      return setpoint;
    }

    double error = setpoint - feedback;

    // Deadband: ignore tiny errors
    if (std::fabs(error) < gains_.deadband) {
      error = 0.0;
    }

    // Proportional
    double p_term = gains_.kp * error;

    // Integral with anti-windup clamping
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -gains_.max_integral, gains_.max_integral);
    double i_term = gains_.ki * integral_;

    // Derivative (skip first update to avoid spike)
    double d_term = 0.0;
    if (!first_update_) {
      double derivative = (error - prev_error_) / dt;
      d_term = gains_.kd * derivative;
    }
    first_update_ = false;
    prev_error_ = error;

    // PID output = setpoint (feedforward) + correction
    return setpoint + p_term + i_term + d_term;
  }

private:
  PidGains gains_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  bool first_update_ = true;
};

}  // namespace wheeltec_pid

#endif  // WHEELTEC_PID_CONTROLLER__PID_CONTROLLER_HPP_
