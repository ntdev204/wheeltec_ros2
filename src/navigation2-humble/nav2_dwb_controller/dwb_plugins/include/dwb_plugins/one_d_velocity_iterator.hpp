


#ifndef DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_
#define DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_

#include <algorithm>
#include <cmath>

namespace dwb_plugins
{

const double EPSILON = 1E-5;



inline double projectVelocity(double v0, double accel, double decel, double dt, double target)
{
  double v1;
  if (v0 < target) {
    v1 = v0 + accel * dt;
    return std::min(target, v1);
  } else {
    v1 = v0 + decel * dt;
    return std::max(target, v1);
  }
}



class OneDVelocityIterator
{
public:
  

  OneDVelocityIterator(
    double current, double min, double max, double acc_limit, double decel_limit, double acc_time,
    int num_samples)
  {
    if (current < min) {
      current = min;
    } else if (current > max) {
      current = max;
    }
    max_vel_ = projectVelocity(current, acc_limit, decel_limit, acc_time, max);
    min_vel_ = projectVelocity(current, acc_limit, decel_limit, acc_time, min);
    reset();

    if (fabs(min_vel_ - max_vel_) < EPSILON) {
      increment_ = 1.0;
      return;
    }
    num_samples = std::max(2, num_samples);


    increment_ = (max_vel_ - min_vel_) / std::max(1, (num_samples - 1));
  }

  

  double getVelocity() const
  {
    if (return_zero_now_) {return 0.0;}
    return current_;
  }

  

  OneDVelocityIterator & operator++()
  {
    if (return_zero_ && current_ < 0.0 && current_ + increment_ > 0.0 &&
      current_ + increment_ <= max_vel_ + EPSILON)
    {
      return_zero_now_ = true;
      return_zero_ = false;
    } else {
      current_ += increment_;
      return_zero_now_ = false;
    }
    return *this;
  }

  

  void reset()
  {
    current_ = min_vel_;
    return_zero_ = true;
    return_zero_now_ = false;
  }

  

  bool isFinished() const
  {
    return current_ > max_vel_ + EPSILON;
  }

private:
  bool return_zero_, return_zero_now_;
  double min_vel_, max_vel_;
  double current_;
  double increment_;
};
}

#endif
