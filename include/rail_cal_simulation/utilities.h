#ifndef RAIL_CAL_SIM_UTILITIES_H
#define RAIL_CAL_SIM_UTILITIES_H

#include <Eigen/Dense>

Eigen::Affine3d perturbPose(const Eigen::Affine3d& pose, double spatial_noise, double angle_noise);

#endif // RAIL_CAL_SIM_UTILITIES_H
