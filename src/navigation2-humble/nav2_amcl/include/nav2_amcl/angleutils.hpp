


#ifndef NAV2_AMCL__ANGLEUTILS_HPP_
#define NAV2_AMCL__ANGLEUTILS_HPP_

#include <math.h>

namespace nav2_amcl
{



class angleutils
{
public:
  

  static double normalize(double z);

  

  static double angle_diff(double a, double b);
};

inline double
angleutils::normalize(double z)
{
  return atan2(sin(z), cos(z));
}

inline double
angleutils::angle_diff(double a, double b)
{
  a = normalize(a);
  b = normalize(b);
  double d1 = a - b;
  double d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) {
    d2 *= -1.0;
  }
  if (fabs(d1) < fabs(d2)) {
    return d1;
  } else {
    return d2;
  }
}

}

#endif
