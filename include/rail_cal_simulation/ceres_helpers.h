#ifndef RAIL_CAL_SIM_CERES_HELPERS_H
#define RAIL_CAL_SIM_CERES_HELPERS_H

#include <ceres/rotation.h>
#include <rail_cal_simulation/point3d.h>

template <typename T>
inline void extractCameraIntrinsics(const T intrinsics[9], T& fx, T& fy, T& cx, T& cy, T& k1, T& k2, T& k3, T& p1,
                                    T& p2)
{
  fx = intrinsics[0]; /** focal length x */
  fy = intrinsics[1]; /** focal length y */
  cx = intrinsics[2]; /** central point x */
  cy = intrinsics[3]; /** central point y */
  k1 = intrinsics[4]; /** distortion k1  */
  k2 = intrinsics[5]; /** distortion k2  */
  k3 = intrinsics[6]; /** distortion k3  */
  p1 = intrinsics[7]; /** distortion p1  */
  p2 = intrinsics[8]; /** distortion p2  */
}


/*! \brief ceres compliant function to apply an angle-axis and translation to transform a point in Point3d form
 *  @param angle_axis, ax, ay, and az
 *  @param tx translation tx, ty and tz
 *  @param point the original point in a Point3d form
 *  @param t_point the transformed point
 */
template <typename T>
inline void transformPoint3d(const T angle_axis[3], const T tx[3], const Point3d& point, T t_point[3])
{
  T point_[3];
  point_[0] = T(point.x);
  point_[1] = T(point.y);
  point_[2] = T(point.z);
  ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);
  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template <typename T>
inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
{
  T point_[3];
  point_[0] = point[0];
  point_[1] = point[1];
  point_[2] = point[2];
  ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);
  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}


template <typename T>
inline void projectPointDist(T point[3], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx, T& fy, T& cx, T& cy, T pt_in_image[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  /** scale into the image plane by distance away from camera */
  T xp;
  T yp;
  if (zp1 == T(0))
  {  // avoid divide by zero
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  /* temporary variables for distortion model */
  T xp2 = xp * xp;  /* x^2 */
  T yp2 = yp * yp;  /* y^2 */
  T r2 = xp2 + yp2; /* r^2 radius squared */
  T r4 = r2 * r2;   /* r^4 */
  T r6 = r2 * r4;   /* r^6 */

  /* apply the distortion coefficients to refine pixel location */
  T xpp = xp + k1 * r2 * xp           // 2nd order term
          + k2 * r4 * xp              // 4th order term
          + k3 * r6 * xp              // 6th order term
          + p2 * (r2 + T(2.0) * xp2)  // tangential
          + p1 * xp * yp * T(2.0);    // other tangential term
  T ypp = yp + k1 * r2 * yp           // 2nd order term
          + k2 * r4 * yp              // 4th order term
          + k3 * r6 * yp              // 6th order term
          + p1 * (r2 + T(2.0) * yp2)  // tangential term
          + p2 * xp * yp * T(2.0);    // other tangential term

  /** perform projection using focal length and camera center into image plane */
  pt_in_image[0] = fx * xpp + cx;
  pt_in_image[1] = fy * ypp + cy;
}

template <typename T>
inline void cameraPntResidualDist(T point[3], T& k1, T& k2, T& k3, T& p1, T& p2, T& fx, T& fy, T& cx, T& cy, T& ox,
                                  T& oy, T residual[2])
{
  T pt_in_image[2];
  projectPointDist(point, k1, k2, k3, p1, p2, fx, fy, cx, cy, pt_in_image);

  residual[0] = pt_in_image[0] - ox;
  residual[1] = pt_in_image[1] - oy;
}


#endif // RAIL_CAL_SIM_CERES_HELPERS_H
