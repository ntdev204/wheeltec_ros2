













#ifndef NAV2_COLLISION_MONITOR__TYPES_HPP_
#define NAV2_COLLISION_MONITOR__TYPES_HPP_

namespace nav2_collision_monitor
{


struct Velocity
{
  double x;
  double y;
  double tw;

  inline bool operator<(const Velocity & second) const
  {
    const double first_vel = x * x + y * y + tw * tw;
    const double second_vel = second.x * second.x + second.y * second.y + second.tw * second.tw;

    return first_vel < second_vel;
  }

  inline Velocity operator*(const double & mul) const
  {
    return {x * mul, y * mul, tw * mul};
  }

  inline bool isZero() const
  {
    return x == 0.0 && y == 0.0 && tw == 0.0;
  }
};


struct Point
{
  double x;
  double y;
};


struct Pose
{
  double x;
  double y;
  double theta;
};


enum ActionType
{
  DO_NOTHING = 0,
  STOP = 1,
  SLOWDOWN = 2,
  APPROACH = 3
};


struct Action
{
  ActionType action_type;
  Velocity req_vel;
};

}

#endif
