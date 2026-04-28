


#include "nav2_amcl/motion_model/omni_motion_model.hpp"

namespace nav2_amcl
{

void
OmniMotionModel::initialize(
  double alpha1, double alpha2, double alpha3, double alpha4,
  double alpha5)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

void
OmniMotionModel::odometryUpdate(
  pf_t * pf, const pf_vector_t & pose,
  const pf_vector_t & delta)
{

  pf_sample_set_t * set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  double delta_trans, delta_rot, delta_bearing;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

  delta_trans = sqrt(
    delta.v[0] * delta.v[0] +
    delta.v[1] * delta.v[1]);
  delta_rot = delta.v[2];


  double trans_hat_stddev = sqrt(
    alpha3_ * (delta_trans * delta_trans) +
    alpha4_ * (delta_rot * delta_rot) );
  double rot_hat_stddev = sqrt(
    alpha1_ * (delta_rot * delta_rot) +
    alpha2_ * (delta_trans * delta_trans) );
  double strafe_hat_stddev = sqrt(
    alpha4_ * (delta_rot * delta_rot) +
    alpha5_ * (delta_trans * delta_trans) );

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t * sample = set->samples + i;

    delta_bearing = angleutils::angle_diff(
      atan2(delta.v[1], delta.v[0]),
      old_pose.v[2]) + sample->pose.v[2];
    double cs_bearing = cos(delta_bearing);
    double sn_bearing = sin(delta_bearing);


    delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
    delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
    delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);

    sample->pose.v[0] += (delta_trans_hat * cs_bearing +
      delta_strafe_hat * sn_bearing);
    sample->pose.v[1] += (delta_trans_hat * sn_bearing -
      delta_strafe_hat * cs_bearing);
    sample->pose.v[2] += delta_rot_hat;
  }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_amcl::OmniMotionModel, nav2_amcl::MotionModel)
