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

class RailICal3
{
public:
  RailICal3(double ob_x, double ob_y, Point3d rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {}

  template<typename T>
  bool operator()(const T* const c_p1, const T* const c_p2, T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    camera_point[0] = camera_point[0] + T(rail_position_.x); // transform to camera's location along rail
    camera_point[1] = camera_point[1] + T(rail_position_.y); // transform to camera's location along rail
    camera_point[2] = camera_point[2] + T(rail_position_.z); // transform to camera's location along rail

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy,  residual);

    return true;
  }

  static ceres::CostFunction* Create(const double o_x, const double o_y, Eigen::Vector3d rail_position, Eigen::Vector3d point)
  {
    Point3d p3d;
    p3d.x = point.x();
    p3d.y = point.y();
    p3d.z = point.z();

    Point3d rail;
    rail.x = rail_position.x();
    rail.y = rail_position.y();
    rail.z = rail_position.z();

    return new ceres::AutoDiffCostFunction<RailICal3, 2, 9, 6>(new RailICal3(o_x, o_y, rail, p3d));
  }

  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  Point3d rail_position_; /** location of camera along rail */
  Point3d point_; /** point expressed in target coordinates */
};

// Estimate the axis of motion as part of the optimization
class RailICal4
{
public:
  RailICal4(double ob_x, double ob_y, double rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {}

  template<typename T>
  bool operator()(const T* const c_p1, // Intrinsics (9 params)
                  const T* const c_p2, // Target origin (6 params)
                  const T* const c_p3, // Camera skew (2 params)
                  T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    // Now let's move the camera point by the rail travel
    // This involves two steps:
    // 1. Estimating the axis of motion (relative to camera z)
    // 2. Moving the camera back by that distance

    T nominal_axis[3]; // Nominally we move back...
    nominal_axis[0] = T(0.0);
    nominal_axis[1] = T(0.0);
    nominal_axis[2] = T(1.0);

    T rotation_axis[3];
    rotation_axis[0] = c_p3[0];
    rotation_axis[1] = c_p3[1];
    rotation_axis[2] = T(0.0);

    T motion_axis[3];
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis, motion_axis);

    camera_point[0] = camera_point[0] + T(rail_position_) * motion_axis[0];
    camera_point[1] = camera_point[1] + T(rail_position_) * motion_axis[1];
    camera_point[2] = camera_point[2] + T(rail_position_) * motion_axis[2];

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  }

  static ceres::CostFunction* Create(const double o_x, const double o_y, double rail_position, Eigen::Vector3d point)
  {
    Point3d p3d;
    p3d.x = point.x();
    p3d.y = point.y();
    p3d.z = point.z();

    return new ceres::AutoDiffCostFunction<RailICal4, 2, 9, 6, 2>(new RailICal4(o_x, o_y, rail_position, p3d));
  }

  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
  Point3d point_; /** point expressed in target coordinates */
};

// RailICal4 with no distortion
class RailICal5
{
public:
  RailICal5(double ob_x, double ob_y, double rail_position, Point3d point)
    : ox_(ob_x), oy_(ob_y), rail_position_(rail_position), point_(point)
  {}

  template<typename T>
  bool operator()(const T* const c_p1, // Intrinsics (9 params)
                  const T* const c_p2, // Target origin (6 params)
                  const T* const c_p3, // Camera skew (2 params)
                  T* residual) const
  {
    T fx, fy, cx, cy, k1, k2, k3, p1, p2;      // extract intrinsics
    extractCameraIntrinsics(c_p1, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    const T *target_aa(& c_p2[0]); // extract target's angle axis
    const T *target_tx(& c_p2[3]); // extract target's position

    /** transform point into camera frame */
    T camera_point[3]; /** point in camera coordinates */
    transformPoint3d(target_aa, target_tx, point_, camera_point);
    // Now let's move the camera point by the rail travel
    // This involves two steps:
    // 1. Estimating the axis of motion (relative to camera z)
    // 2. Moving the camera back by that distance

    T nominal_axis[3]; // Nominally we move back...
    nominal_axis[0] = T(0.0);
    nominal_axis[1] = T(0.0);
    nominal_axis[2] = T(1.0);

    T rotation_axis[3];
    rotation_axis[0] = c_p3[0];
    rotation_axis[1] = c_p3[1];
    rotation_axis[2] = T(0.0);

    T motion_axis[3];
    ceres::AngleAxisRotatePoint(rotation_axis, nominal_axis, motion_axis);

    camera_point[0] = camera_point[0] + T(rail_position_) * motion_axis[0];
    camera_point[1] = camera_point[1] + T(rail_position_) * motion_axis[1];
    camera_point[2] = camera_point[2] + T(rail_position_) * motion_axis[2];

    /** compute project point into image plane and compute residual */
    T ox = T(ox_);
    T oy = T(oy_);
    T zero = T(0.0);
    cameraPntResidualDist(camera_point, zero, zero, zero, zero, zero, fx, fy, cx, cy, ox, oy, residual);

    return true;
  }

  static ceres::CostFunction* Create(const double o_x, const double o_y, double rail_position, Eigen::Vector3d point)
  {
    Point3d p3d;
    p3d.x = point.x();
    p3d.y = point.y();
    p3d.z = point.z();

    return new ceres::AutoDiffCostFunction<RailICal5, 2, 9, 6, 2>(new RailICal5(o_x, o_y, rail_position, p3d));
  }

  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double rail_position_; /** location of camera along rail */
  Point3d point_; /** point expressed in target coordinates */
};



#endif
