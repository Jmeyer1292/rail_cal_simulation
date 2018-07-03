#include <rail_cal_simulation/random.h>

PinholeCamera randomizeCamera(const PinholeCamera& input, const double focal_length_variance,
                              const double center_point_variance, const double radial_dist_variance,
                              const double tang_dist_variance, std::shared_ptr<std::default_random_engine> rng)
{
  std::normal_distribution<double> fx_dist (input.intrinsics.data()[0], focal_length_variance);
  std::normal_distribution<double> fy_dist (input.intrinsics.data()[1], focal_length_variance);

  std::normal_distribution<double> cx_dist (input.intrinsics.data()[2], center_point_variance);
  std::normal_distribution<double> cy_dist (input.intrinsics.data()[3], center_point_variance);

  std::normal_distribution<double> k1_dist (input.intrinsics.data()[4], radial_dist_variance);
  std::normal_distribution<double> k2_dist (input.intrinsics.data()[5], radial_dist_variance);
  std::normal_distribution<double> k3_dist (input.intrinsics.data()[6], radial_dist_variance);

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
