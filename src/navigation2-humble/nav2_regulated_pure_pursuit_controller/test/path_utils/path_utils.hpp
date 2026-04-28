















#ifndef PATH_UTILS__PATH_UTILS_HPP_
#define PATH_UTILS__PATH_UTILS_HPP_

#include <cmath>
#include <initializer_list>
#include <memory>

#include "nav_msgs/msg/path.hpp"

namespace path_utils
{



class PathSegment
{
public:
  virtual void append(nav_msgs::msg::Path & path, double spacing) const = 0;
  virtual ~PathSegment() {}
};

class Arc : public PathSegment
{
public:
  explicit Arc(double radius, double radians)
  : radius_(radius), radians_(radians) {}
  void append(nav_msgs::msg::Path & path, double spacing) const override;

private:
  double radius_;
  double radians_;
};

class Straight : public PathSegment
{
public:
  explicit Straight(double length)
  : length_(length) {}
  void append(nav_msgs::msg::Path & path, double spacing) const override;

private:
  double length_;
};

class LeftTurn : public Arc
{
public:
  explicit LeftTurn(double radius)
  : Arc(radius, M_PI_2) {}
};

class RightTurn : public Arc
{
public:
  explicit RightTurn(double radius)
  : Arc(radius, -M_PI_2) {}
};

class LeftTurnAround : public Arc
{
public:
  explicit LeftTurnAround(double radius)
  : Arc(radius, M_PI) {}
};

class RightTurnAround : public Arc
{
public:
  explicit RightTurnAround(double radius)
  : Arc(radius, -M_PI) {}
};

class LeftCircle : public Arc
{
public:
  explicit LeftCircle(double radius)
  : Arc(radius, 2.0 * M_PI) {}
};

class RightCircle : public Arc
{
public:
  explicit RightCircle(double radius)
  : Arc(radius, -2.0 * M_PI) {}
};

nav_msgs::msg::Path generate_path(
  geometry_msgs::msg::PoseStamped start,
  double spacing,
  std::initializer_list<std::unique_ptr<PathSegment>> segments);

}

#endif
