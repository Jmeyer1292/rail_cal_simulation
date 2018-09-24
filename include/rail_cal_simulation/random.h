#ifndef RAIL_CAL_SIM_RANDOM_H
#define RAIL_CAL_SIM_RANDOM_H

#include <Eigen/Dense>
#include <memory>
#include <random>
#include <rail_cal_simulation/camera_model.h>

PinholeCamera randomizeCamera(const PinholeCamera& input,
                              const double focal_length_variance,
                              const double center_point_variance,
                              const double k1_k2_variance,
                              const double k3_variance,
                              const double tang_dist_variance,
                              std::shared_ptr<std::default_random_engine> rng);

Eigen::Affine3d perturbPose(const Eigen::Affine3d& pose, double spatial_noise, double angle_noise,
                            std::shared_ptr<std::default_random_engine> rng);

Eigen::Vector3d perturbOrientation(const Eigen::Vector3d& seed, double x_variance, double y_variance,
                                   std::shared_ptr<std::default_random_engine> rng);



#endif // RAIL_CAL_SIM_RANDOM_H
