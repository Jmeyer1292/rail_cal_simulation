#ifndef PNP_H
#define PNP_H

#include <rail_cal_simulation/camera_model.h>
#include <rail_cal_simulation/dot_grid.h>

struct PnPProbem {
  PinholeCamera camera;
  DotGrid grid;
  Eigen::Affine3d guess;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> image;
};

bool computeTargetPose(const PnPProbem& prob, Eigen::Affine3d& out);

#endif // PNP_H
