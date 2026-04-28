


#include <sys/types.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

Laser::Laser(size_t max_beams, map_t * map)
: max_samples_(0), max_obs_(0), temp_obs_(NULL)
{
  max_beams_ = max_beams;
  map_ = map;
}

Laser::~Laser()
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
}

void
Laser::reallocTempData(int new_max_samples, int new_max_obs)
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
  max_obs_ = new_max_obs;
  max_samples_ = fmax(max_samples_, new_max_samples);

  temp_obs_ = new double *[max_samples_]();
  for (int k = 0; k < max_samples_; k++) {
    temp_obs_[k] = new double[max_obs_]();
  }
}

void
Laser::SetLaserPose(pf_vector_t & laser_pose)
{
  laser_pose_ = laser_pose;
}

}
