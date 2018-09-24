#include <rail_cal_simulation/random.h>

PinholeCamera randomizeCamera(const PinholeCamera& input, const double focal_length_variance,
                              const double center_point_variance, const double k1_k2_variance, const double k3_variance,
                              const double tang_dist_variance, std::shared_ptr<std::default_random_engine> rng)
{
  std::normal_distribution<double> fx_dist (input.intrinsics.data()[0], focal_length_variance);
  std::normal_distribution<double> fy_dist (input.intrinsics.data()[1], focal_length_variance);

  std::normal_distribution<double> cx_dist (input.intrinsics.data()[2], center_point_variance);
  std::normal_distribution<double> cy_dist (input.intrinsics.data()[3], center_point_variance);

  std::normal_distribution<double> k1_dist (input.intrinsics.data()[4], k1_k2_variance);
  std::normal_distribution<double> k2_dist (input.intrinsics.data()[5], k1_k2_variance);
  std::normal_distribution<double> k3_dist (input.intrinsics.data()[6], k3_variance);

  std::normal_distribution<double> p1_dist (input.intrinsics.data()[7], tang_dist_variance);
  std::normal_distribution<double> p2_dist (input.intrinsics.data()[8], tang_dist_variance);

  PinholeCamera output = input;
  output.intrinsics.data()[0] = fx_dist(*rng);
  output.intrinsics.data()[1] = fy_dist(*rng);
  output.intrinsics.data()[2] = cx_dist(*rng);
  output.intrinsics.data()[3] = cy_dist(*rng);

  output.intrinsics.data()[4] = k1_dist(*rng);
  output.intrinsics.data()[5] = k2_dist(*rng);
  output.intrinsics.data()[6] = k3_dist(*rng);

  output.intrinsics.data()[7] = p1_dist(*rng);
  output.intrinsics.data()[8] = p2_dist(*rng);

  return output;
}

Eigen::Vector3d perturbOrientation(const Eigen::Vector3d& seed, double x_variance, double y_variance,
                                   std::shared_ptr<std::default_random_engine> rng)
{
  std::normal_distribution<double> x_dist (0.0, x_variance);
  std::normal_distribution<double> y_dist (0.0, y_variance);

  Eigen::Vector3d r (x_dist(*rng), y_dist(*rng), 0.0);
  Eigen::Vector3d new_vec = (seed + r).normalized();

  Eigen::Quaterniond q;
  q.setFromTwoVectors(seed, new_vec);

  Eigen::AngleAxisd aa;
  aa = q;
  std::cout << "Random " << (aa.angle() * 180 / M_PI) << " degrees at " << aa.axis().transpose() << "\n";
  std::cout << "New vec: " << new_vec.transpose() << "\n";
  return new_vec;
}

Eigen::Affine3d perturbPose(const Eigen::Affine3d& pose, double spatial_noise, double angle_noise,
                            std::shared_ptr<std::default_random_engine> rng)
{

  std::uniform_real_distribution<double> spatial_dist (-spatial_noise, spatial_noise);
  std::uniform_real_distribution<double> angle_dist (-angle_noise, angle_noise);
  std::uniform_real_distribution<double> one_to_one (-1, 1);

  Eigen::Vector3d translation (spatial_dist(*rng), spatial_dist(*rng), spatial_dist(*rng));
  Eigen::Vector3d rot_axis (one_to_one(*rng), one_to_one(*rng), one_to_one(*rng));
  rot_axis.normalize();

  double angle = angle_dist(*rng);

  std::cout << "Applied positional noise of " << translation.transpose() << "\n";
  std::cout << "Applied angular noise of " << (angle * 180.0/M_PI) << " degrees about " << rot_axis.transpose() << "\n";
  Eigen::Affine3d new_pose = pose * Eigen::Translation3d(translation) * Eigen::AngleAxisd(angle, rot_axis);

  return new_pose;
}

