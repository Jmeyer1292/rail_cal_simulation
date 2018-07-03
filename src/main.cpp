#include <ceres/ceres.h>
#include <iostream>

#include <rail_cal_simulation/camera_model.h>
#include <rail_cal_simulation/cost_function.h>
#include <rail_cal_simulation/dot_grid.h>
#include <rail_cal_simulation/observation_creator.h>
#include <Eigen/Dense>

static PinholeCamera makeCamera()
{
  PinholeCamera camera;

  // Sensor parameters
  camera.width = 1900;
  camera.height = 1200;
  // Pinhole parameters
  camera.intrinsics.data()[0] = 1200.0; // fx
  camera.intrinsics.data()[1] = 1200.0; // fy
  camera.intrinsics.data()[2] = camera.width / 2; // cx
  camera.intrinsics.data()[3] = camera.height / 2; // cy
  // Distortion Parameters
  camera.intrinsics.data()[4] = 0.0; // k1
  camera.intrinsics.data()[5] = 0.0; // k2
  camera.intrinsics.data()[6] = 0.0; // k3
  camera.intrinsics.data()[7] = 0.0; // p1
  camera.intrinsics.data()[8] = 0.0; // p2

  return camera;
}

static DotGrid makeTarget()
{
  return DotGrid(19, 12, 0.01);
}

std::vector<EigenSTLVector<Eigen::Vector2d>>
takePictures(const PinholeCamera& camera, const DotGrid& target, const Eigen::Affine3d& target_pose,
             const Eigen::Affine3d& camera_start, const double min_distance, const double max_distance,
             const double increment)
{
  // Vector of vector of dots in image
  std::vector<EigenSTLVector<Eigen::Vector2d>> images;

  for (double s = min_distance; s <= max_distance; s += increment)
  {
    Eigen::Affine3d camera_pose = camera_start * Eigen::Translation3d(0, 0, -s);
    Eigen::Affine3d target_in_camera = camera_pose.inverse() * target_pose;

    EigenSTLVector<Eigen::Vector2d> grid_in_image = projectGrid(target_in_camera, camera, target);
    images.push_back(grid_in_image);
  }

  return images;
}

void runExperiment1()
{
  // Define camera & target
  PinholeCamera true_camera = makeCamera();
  DotGrid true_target = makeTarget();
  std::cout << true_target << "\n";

  // First let us define the "ground truth"
  // The target will be located at Origin
  Eigen::Affine3d true_target_pose = Eigen::Affine3d::Identity();

  // The path is defined by a linear path that NOMINALLY starts in the middle of the
  // target and proceeds backwards along -Z in camera space
  Eigen::Affine3d ideal_camera_start_point = Eigen::Affine3d::Identity();
  // correct for target center
  ideal_camera_start_point.translation() = Eigen::Vector3d(true_target.physicalWidth() / 2,
                                                           true_target.physicalHeight() / 2, 0);
  // set orientation (camera looking down at the target, camera x pointing along target x)
  ideal_camera_start_point.linear() << 1,  0,  0,
                                       0, -1,  0,
                                       0,  0, -1;

  // Take a picture every 20 cm from 0.5 meters to 1.5 meters
  const double min_distance = 0.5;
  const double max_distance = 1.5;
  const double increment = 0.2;

  std::vector<EigenSTLVector<Eigen::Vector2d>> raw_images = takePictures(true_camera, true_target, true_target_pose,
                                                                         ideal_camera_start_point,
                                                                         min_distance, max_distance,
                                                                         increment);

  // Setup optimization
  // Guesses
  PinholeCamera guess_camera = true_camera;
  guess_camera.intrinsics.data()[0] = 1000.0; // fx
  double target_pose[6];
  target_pose[0] = M_PI;  // rx
  target_pose[1] = 0.0;   // ry
  target_pose[2] = 0.0;   // rz
  target_pose[3] = 0.0;   // x
  target_pose[4] = 0.0;   // y
  target_pose[5] = 0.5;   // z

  ceres::Problem problem;

  // Build costs
  for (std::size_t i = 0; i < raw_images.size(); ++i)
  {
    const double rail_position = 0.0 + i * increment;

    for (std::size_t j = 0; j < raw_images[i].size(); ++j)
    {
      Eigen::Vector2d in_image = raw_images[i][j];
      Eigen::Vector3d in_target = true_target.points()[j];

      problem.AddResidualBlock(RailICal::Create(in_image.x(), in_image.y(), rail_position, in_target),
                               NULL, guess_camera.intrinsics.data(), target_pose);

    } // for each circle
  } // for each image


  // Solve
  ceres::Solver::Options options;
  options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Analyze results
  std::cout << summary.FullReport() << "\n";

  // Results:
  std::cout << "---After minimization---\n";
  std::cout << guess_camera << "\n";
  std::cout << "Target Pose:\n";
  std::cout << target_pose[0] << " " << target_pose[1] << " " << target_pose[2] << " "
            << target_pose[3] << " " << target_pose[4] << " " << target_pose[5] << "\n";
}

int main()
{
  runExperiment1();
}
