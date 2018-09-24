#ifndef MUTABLE_MODEL_COSTS_H
#define MUTABLE_MODEL_COSTS_H

#include <rail_cal_simulation/ceres_helpers.h>
#include <ceres/ceres.h>

/**
 * @brief The FullCameraModel struct
 */
template<class T>
struct FullCameraModel
{
  FullCameraModel(const T* data) : data_(data) {}

  constexpr static int numParams() { return 9; }

  void project(const T* point_in_camera, T* point_in_image) const
  {
    // First alias the variables...
    const T& fx = data_[0]; /** focal length x */
    const T& fy = data_[1]; /** focal length y */
    const T& cx = data_[2]; /** central point x */
    const T& cy = data_[3]; /** central point y */
    const T& k1 = data_[4]; /** distortion k1  */
    const T& k2 = data_[5]; /** distortion k2  */
    const T& k3 = data_[6]; /** distortion k3  */
    const T& p1 = data_[7]; /** distortion p1  */
    const T& p2 = data_[8]; /** distortion p2  */

    // Now for the math
    T xp1 = point_in_camera[0];
    T yp1 = point_in_camera[1];
    T zp1 = point_in_camera[2];

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
    point_in_image[0] = fx * xpp + cx;
    point_in_image[1] = fy * ypp + cy;
  }

  const T* data_;
};

template<typename T>
struct ReducedCameraModel
{
  ReducedCameraModel(const T* data) : data_(data) {}

  constexpr static int numParams() { return 5; }

  void project(const T* point_in_camera, T* point_in_image) const
  {
    // First alias the variables...
    const T& f = data_[0]; /** focal length x */
//    const T& fy = data_[1]; /** focal length y */
    const T& cx = data_[1]; /** central point x */
    const T& cy = data_[2]; /** central point y */
    const T& k1 = data_[3]; /** distortion k1  */
    const T& k2 = data_[4]; /** distortion k2  */
//    const T& k3 = data_[6]; /** distortion k3  */
//    const T& p1 = data_[7]; /** distortion p1  */
//    const T& p2 = data_[8]; /** distortion p2  */

    // Now for the math
    T xp1 = point_in_camera[0];
    T yp1 = point_in_camera[1];
    T zp1 = point_in_camera[2];

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
//    T r6 = r2 * r4;   /* r^6 */

    /* apply the distortion coefficients to refine pixel location */
    T xpp = xp + k1 * r2 * xp           // 2nd order term
            + k2 * r4 * xp;              // 4th order term
//            + k3 * r6 * xp              // 6th order term
//            + p2 * (r2 + T(2.0) * xp2)  // tangential
//            + p1 * xp * yp * T(2.0);    // other tangential term
    T ypp = yp + k1 * r2 * yp           // 2nd order term
            + k2 * r4 * yp;              // 4th order term
//            + k3 * r6 * yp              // 6th order term
//            + p1 * (r2 + T(2.0) * yp2)  // tangential term
//            + p2 * xp * yp * T(2.0);    // other tangential term

    /** perform projection using focal length and camera center into image plane */
    point_in_image[0] = f * xpp + cx;
    point_in_image[1] = f * ypp + cy;
  }

  const T* data_;
};

struct ReducedCameraModelMaker {
  template<class T> static ReducedCameraModel<T> make(const T* data) { return ReducedCameraModel<T>(data); }
  constexpr static int numParams() { return 5; }
};

struct FullCameraModelMaker {
  template<class T> static FullCameraModel<T> make(const T* data) { return FullCameraModel<T>(data); }
  constexpr static int numParams() { return 9; }
};

/**
 *
 */

template<class Model>
class IntrFunctor
{
public:
  IntrFunctor(const Eigen::Vector3d& in_target, const Eigen::Vector2d& in_image)
    : in_target_(in_target), in_image_(in_image)
  {}

  template<typename T>
  bool operator()(const T* const target_pose, const T* const camera_intr, T* const residual) const
  {
    auto model = Model::template make(camera_intr);

    const T* target_angle_axis = target_pose + 0;
    const T* target_position = target_pose + 3;

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(in_target_(0));
    target_pt[1] = T(in_target_(1));
    target_pt[2] = T(in_target_(2));

    T camera_point[3];  // Point in camera coordinates
    transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    /** compute project point into image plane and compute residual */
    T ox = T(in_image_.x());
    T oy = T(in_image_.y());

    T pt_in_image[2];
    model.project(camera_point, pt_in_image);
    residual[0] = pt_in_image[0] - ox;
    residual[1] = pt_in_image[1] - oy;

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& in_target, const Eigen::Vector2d& in_image)
  {
    constexpr int numModelParams = Model::numParams();
    return new ceres::AutoDiffCostFunction<IntrFunctor<Model>, 2, 6, numModelParams>(new IntrFunctor<Model>(in_target, in_image));
  }


private:
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

#endif // MUTABLE_MODEL_COSTS_H
