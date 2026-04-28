


#include "tofbf.h"

namespace ldlidar {



Tofbf::Tofbf(int speed) {
  curr_speed_ = speed; 
}

Tofbf::~Tofbf() {

}



std::vector<PointData> Tofbf::NearFilter(
    const std::vector<PointData> &tmp) const {
  std::vector<PointData> normal, pending, item;
  std::vector<std::vector<PointData>> group;


  for (auto n : tmp) {
    if (n.distance < 5000) {
      pending.push_back(n);
    } else {
      normal.push_back(n);
    }
  }

  if (tmp.empty()) return normal;

  double angle_delta_up_limit = curr_speed_ / kScanFrequency * 2;


  std::sort(pending.begin(), pending.end(), [](PointData a, PointData b) { return a.angle < b.angle; });

  PointData last(-10, 0, 0);

  for (auto n : pending) {
    if (abs(n.angle - last.angle) > angle_delta_up_limit ||
        abs(n.distance - last.distance) > last.distance * 0.03) {
      if (item.empty() == false) {
        group.push_back(item);
        item.clear();
      }
    }
    item.push_back(n);
    last = n;
  }

  if (item.empty() == false) group.push_back(item);

  if (group.empty()) return normal;


  auto first_item = group.front().front();
  auto last_item = group.back().back();
  if (fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&
      abs(first_item.distance - last_item.distance) < last.distance * 0.03) {
    group.front().insert(group.front().begin(), group.back().begin(), group.back().end());
    group.erase(group.end() - 1);
  }

  for (auto n : group) {
    if (n.size() == 0) continue;

    if (n.size() > 15) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }


    if (n.size() < 3) {
      int c = 0;
      for (auto m : n) {
        c += m.intensity;
      }
      c /= n.size();
      if (c < kIntensitySingle){

        for (auto& point: n) {
          point.distance = 0;
          point.intensity = 0;
        }
      } 
    } else {

      double confidence_avg = 0;
      double dis_avg = 0;
      for (auto m : n) {
        confidence_avg += m.intensity;
        dis_avg += m.distance;
      }
      confidence_avg /= n.size();
      dis_avg /= n.size();


      if (confidence_avg > kIntensityLow) {


      } else {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }

      }
    }

    normal.insert(normal.end(), n.begin(), n.end());
  }

  return normal;
}

}


