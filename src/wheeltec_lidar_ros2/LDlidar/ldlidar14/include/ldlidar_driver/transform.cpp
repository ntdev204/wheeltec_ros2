


#include "transform.h"

namespace ldlidar {



SlTransform::SlTransform(LDType version, bool to_right_hand_) {
  switch (version) {
    case LDType::LD_14:
      offset_x_ = 5.9;
      offset_y_ = -20.14;
      break;
    default:
      break;
  }
  this->to_right_hand_ = to_right_hand_;
  version_ = version;
}

Points2D SlTransform::Transform(const Points2D &data) {
  Points2D tmp2;
  static double last_shift_delta = 0;
  for (auto n : data) {



    double angle;
    if (n.distance > 0) {
      double x = n.distance + offset_x_;
      double y = n.distance * 0.11923 + offset_y_;
      double shift = atan(y / x) * 180.f / 3.14159;

      if (to_right_hand_) {
        float right_hand = (360.f - n.angle);
        angle = right_hand + shift;
      } else {
        angle = n.angle - shift;
      }
      last_shift_delta = shift;
    } else {
      if (to_right_hand_) {
        float right_hand = (360.f - n.angle);
        angle = right_hand + last_shift_delta;
      } else {
        angle = n.angle - last_shift_delta;
      }
    }
    
    if (angle > 360) {
      angle -= 360;
    }
    if (angle < 0) {
      angle += 360;
    }
    
    switch (version_) {
      case LDType::LD_14:
        if (n.distance == 0) {
          tmp2.push_back(PointData(angle, n.distance, 0));
        } else {
          tmp2.push_back(PointData(angle, n.distance, n.intensity));
        }
        break;
      default:
        break;
    }
  }

  return tmp2;
}

SlTransform::~SlTransform() {}

}

