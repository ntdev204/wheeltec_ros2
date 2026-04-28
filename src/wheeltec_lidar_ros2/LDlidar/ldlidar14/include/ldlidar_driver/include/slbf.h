

#ifndef __SLBF_H_
#define __SLBF_H_

#include <math.h>

#include <algorithm>

#include "pointdata.h"

namespace ldlidar {

class Slbf {
 private:
  const int kConfidenceHigh = 200;
  const int kConfidenceMiddle = 150;
  const int kConfidenceLow = 92;
  const int kScanFre = 2300;

  double curr_speed_;
  bool enable_strict_policy_;



  Slbf() = delete;
  Slbf(const Slbf &) = delete;
  Slbf &operator=(const Slbf &) = delete;

 public:
  Slbf(int speed, bool strict_policy = true);
  Points2D NearFilter(const Points2D &tmp) const;
  void EnableStrictPolicy(bool enable);
  ~Slbf();
};

}

#endif

