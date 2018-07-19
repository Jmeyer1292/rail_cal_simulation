#include "rail_cal_simulation/pnp.h"
#include "rail_cal_simulation/ceres_helpers.h"
#include <ceres/ceres.h>

struct SolvePnPCostFunc
{
public:
  SolvePnPCostFunc(const PinholeCamera& intr, const Eigen::Vector3d& pt_in_target,
                   const Eigen::Vector2d& pt_in_image)
    : intr_(intr), in_target_(pt_in_target), in_image_(pt_in_image)
  {}

  template<typename T>
  bool operator()(const T* const target_pose, T* const residual) const
  {
    const T* target_angle_axis = target_pose + 0;
    const T* target_position = target_pose + 3;

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(in_target_(0));
    target_pt[1] = T(in_target_(1));
    target_pt[2] = T(in_target_(2));

    T camera_point[3];  // Point in camera coordinates
    transformPoint(target_angle_axis, target_position, target_pt, camera_point);

    // Hack
    T camera_model[9];
    for (int i = 0; i < 9; ++i)
      camera_model[i] = T(intr_.intrinsics.data()[i]);

    T fx, fy, cx, cy, k1, k2, k3, p1, p2;  // extract intrinsics
    extractCameraIntrinsics(camera_model, fx, fy, cx, cy, k1, k2, k3, p1, p2);

    /** compute project point into image plane and compute residual */
    T ox = T(in_image_.x());
    T oy = T(in_image_.y());
    cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, ox, oy, residual);

    return true;
  }

  PinholeCamera intr_;
  Eigen::Vector3d in_target_;
  Eigen::Vector2d in_image_;
};

bool computeTargetPose(const PnPProbem &prob, Eigen::Affine3d &out)
{
  double guess[6] = {M_PI, 0, 0, 0, 0, 0.5};
  ceres::Problem problem;

  for (std::size_t i = 0; i < prob.image.size(); ++i) // For each 3D point seen in the 2D image
  {
    // Define
    const auto& img_obs = prob.image[i];
    const auto& point_in_target = prob.grid.points()[i];

    // Allocate Ceres data structures - ownership is taken by the ceres
    // Problem data structure
    auto* cost_fn = new SolvePnPCostFunc(prob.camera, point_in_target, img_obs);

    auto* cost_block = new ceres::AutoDiffCostFunction<SolvePnPCostFunc, 2, 6>(cost_fn);

    problem.AddResidualBlock(cost_block, NULL, guess);
  }


  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  std::cout << "pnp converged: " << (summary.termination_type == ceres::CONVERGENCE) << "\n";
//  std::cout << "pnp init: " << summary.initial_cost / summary.num_residuals << "\n";
//  std::cout << "pnp final: " << summary.final_cost / summary.num_residuals << "\n";
//  std::cout << "guess: " << guess[0] << " " << guess[1] << " " << guess[2] << " " << guess[3] << " " << guess[4] << " " << guess[5] << "\n";

  //
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  p.translation().x() = guess[3];
  p.translation().y() = guess[4];
  p.translation().z() = guess[5];

  const Eigen::Vector3d rr(guess[0], guess[1], guess[2]);
  const double dot_product = rr.dot(rr);

  // Check for 0-rotation
  if (dot_product > std::numeric_limits<double>::epsilon())
  {
    Eigen::AngleAxisd aa(rr.norm(), rr.normalized());
    p.linear() = aa.toRotationMatrix();
  }
  else
  {
    p.linear()(0, 0) = 1.0; // Logic taken from Ceres' own aaxis to rot matrix conversion
    p.linear()(1, 0) = rr[2];
    p.linear()(2, 0) = -rr[1];
    p.linear()(0, 1) = -rr[2];
    p.linear()(1, 1) = 1.0;
    p.linear()(2, 1) =  rr[0];
    p.linear()(0, 2) =  rr[1];
    p.linear()(1, 2) = -rr[0];
    p.linear()(2, 2) = 1.0;
  }

  out = p;

  return summary.termination_type == ceres::CONVERGENCE;
}
