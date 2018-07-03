#include "rail_cal_simulation/observation_creator.h"
#include "rail_cal_simulation/ceres_helpers.h"

EigenSTLVector<Eigen::Vector2d> projectGrid(const Eigen::Affine3d &camera_to_target, const PinholeCamera &camera,
                                            const DotGrid &grid)
{
  // Helpful aliasing
  double fx = camera.intrinsics.data()[0];
  double fy = camera.intrinsics.data()[1];
  double cx = camera.intrinsics.data()[2];
  double cy = camera.intrinsics.data()[3];
  double k1 = camera.intrinsics.data()[4];
  double k2 = camera.intrinsics.data()[5];
  double k3 = camera.intrinsics.data()[6];
  double p1 = camera.intrinsics.data()[7];
  double p2 = camera.intrinsics.data()[8];

  EigenSTLVector<Eigen::Vector2d> out;
  out.reserve(grid.points().size());

  for (const auto& point : grid.points())
  {
    Eigen::Vector3d dot_in_target = camera_to_target * point;
    Eigen::Vector2d in_image;
    projectPointDist(dot_in_target.data(), k1, k2, k3, p1, p2, fx, fy, cx, cy, in_image.data());
    out.push_back(in_image);
  }
  return out;
}

bool inSensorBounds(const int width, const int height, const EigenSTLVector<Eigen::Vector2d> &pts_in_image)
{
  for (const auto& pt : pts_in_image)
  {
    if (pt.x() < 0.0 || pt.x() > width || pt.y() < 0.0 || pt.y() > height) return false;
  }
  return true;
}
