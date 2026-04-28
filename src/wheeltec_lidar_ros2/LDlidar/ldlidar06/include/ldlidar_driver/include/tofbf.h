


#ifndef __TOFBF_H_
#define __TOFBF_H_

#include <math.h>

#include <algorithm>

#include "pointdata.h"

namespace ldlidar {

class Tofbf {
 private:
  const int kIntensityLow = 15;
  const int kIntensitySingle = 220;
  const int kScanFrequency = 4500;

  double curr_speed_;
  Tofbf() = delete;
  Tofbf(const Tofbf &) = delete;
  Tofbf &operator=(const Tofbf &) = delete;

 public:
  Tofbf(int speed);
  std::vector<PointData> NearFilter(const std::vector<PointData> &tmp) const;
  ~Tofbf();
};

}

#endif


