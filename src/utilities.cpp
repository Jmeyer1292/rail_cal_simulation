#include "rail_cal_simulation/utilities.h"
#include <iostream>
#include <random>

Eigen::Affine3d perturbPose(const Eigen::Affine3d& pose, double spatial_noise, double angle_noise)
{
  std::random_device dev;
  std::default_random_engine eng (dev());

  std::uniform_real_distribution<double> spatial_dist (-spatial_noise, spatial_noise);
  std::uniform_real_distribution<double> angle_dist (-angle_noise, angle_noise);
  std::uniform_real_distribution<double> one_to_one (-1, 1);

  Eigen::Vector3d translation (spatial_dist(eng), spatial_dist(eng), spatial_dist(eng));
  Eigen::Vector3d rot_axis (one_to_one(eng), one_to_one(eng), one_to_one(eng));
  rot_axis.normalize();

  double angle = angle_dist(eng);

  std::cout << "Applied positional noise of " << translation.transpose() << "\n";
  std::cout << "Applied angular noise of " << (angle * 180.0/M_PI) << " degrees about " << rot_axis.transpose() << "\n";
  Eigen::Affine3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}
