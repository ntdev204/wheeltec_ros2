

#ifndef __TRANSFORM_H
#define __TRANSFORM_H

#include <math.h>

#include <algorithm>

#include "pointdata.h"

namespace ldlidar {

class SlTransform {
 private:
  bool to_right_hand_ = true;
  double offset_x_;
  double offset_y_;
  LDType version_;

 public:
  SlTransform(LDType version, bool to_right_hand_ = false);
  Points2D Transform(const Points2D &data);
  ~SlTransform();
};

}

#endif

