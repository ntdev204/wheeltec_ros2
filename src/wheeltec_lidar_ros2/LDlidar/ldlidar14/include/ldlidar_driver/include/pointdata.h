

#ifndef _POINT_DATA_H_
#define _POINT_DATA_H_

#include <stdint.h>

#include <iostream>
#include <vector>

namespace ldlidar {

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)
#define RADIAN_TO_ANGLE(angle) ((angle)*180000 / 3141.59)

enum class LDType {
  LD_14,
  NO_VER
};

struct PointData {

  float angle;
  uint16_t distance;
  uint8_t intensity;

  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t intensity, double x = 0,
            double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->x = x;
    this->y = y;
  }
  PointData() {}
  friend std::ostream &operator<<(std::ostream &os, const PointData &data) {
    os << data.angle << " " << data.distance << " " << (int)data.intensity
       << " " << data.x << " " << data.y;
    return os;
  }
};

typedef std::vector<PointData> Points2D;

}

#endif


