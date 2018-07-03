#ifndef RAIL_CAL_SIM_COST_FUNCTION_H
#define RAIL_CAL_SIM_COST_FUNCTION_H

#include <ceres/autodiff_cost_function.h>
#include "rail_cal_simulation/ceres_helpers.h"

class RailICal
{
public:
  RailICal(double ob_x, double ob_y, double rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {
  }

  template <typename T>
  bool operator()(const T* const c_p1, /**intrinsics [9] */
                  const T* const c_p2, /**target_pose [6] */
                  T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;  // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T* target_aa(&c_p2[0]);  // extract target's angle axis
    const T* target_tx(&c_p2[3]);  // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    camera_point[2] = camera_point[2] + T(rail_position_);  // transform to camera's location along rail

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double rail_position, Eigen::Vector3d point)
  {
    Point3d p3d;
    p3d.x = point.x();
    p3d.y = point.y();
    p3d.z = point.z();
    return (new ceres::AutoDiffCostFunction<RailICal, 2, 9, 6>(new RailICal(o_x, o_y, rail_position, p3d)));
  }
  double ox_;            /** observed x location of object in image */
  double oy_;            /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
  Point3d point_;        /** point expressed in target coordinates */
};

#endif
