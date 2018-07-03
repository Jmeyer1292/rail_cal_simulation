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

struct PhysicalSetup
{
  PinholeCamera camera;
  DotGrid target;

  Eigen::Affine3d target_pose;
  Eigen::Affine3d camera_origin_pose; // Camera pose when its focal point intersects the target
};

struct ExperimentalSetup
{
  double min_distance = 0.5;    // Take pictures from this distance
  double max_distance = 2.0;    // To this distance
  double increment = 0.25;      // At this increment

  bool filter_images = true;    // If true, images with points outside image bounds are removed
};

struct NoiseModels
{
  // TODO
};

struct ExperimentalData
{
  std::vector<double> rail_position; // Relative to start of rail movement
  std::vector<EigenSTLVector<Eigen::Vector2d>> images;
};

ExperimentalData
takePictures(const PhysicalSetup& cell, const ExperimentalSetup& experiment, const NoiseModels& noise)
{
  (void)noise;
  // Vector of vector of dots in image
  ExperimentalData data;

  for (double s = experiment.min_distance; s <= experiment.max_distance; s += experiment.increment)
  {
    Eigen::Affine3d camera_pose = cell.camera_origin_pose * Eigen::Translation3d(0, 0, -s);
    Eigen::Affine3d target_in_camera = camera_pose.inverse() * cell.target_pose;

    EigenSTLVector<Eigen::Vector2d> grid_in_image = projectGrid(target_in_camera, cell.camera, cell.target);
    data.images.push_back(grid_in_image);
    data.rail_position.push_back(s - experiment.min_distance);
  }

  return data;
}

PhysicalSetup makeGroundTruth()
{
  PhysicalSetup cell;
  cell.camera = makeCamera();
  cell.target = makeTarget();
  cell.target_pose.setIdentity();

  cell.camera_origin_pose.setIdentity();
  cell.camera_origin_pose.translation() = Eigen::Vector3d(cell.target.physicalWidth() / 2,
                                                          cell.target.physicalHeight() / 2,
                                                          0.0);

  // set orientation (camera looking down at the target, camera x pointing along target x)
  cell.camera_origin_pose.linear() << 1,  0,  0,
                                      0, -1,  0,
                                      0,  0, -1;
  return cell;
}

ExperimentalData runExperiment(const PhysicalSetup& setup, const ExperimentalSetup& experiment,
                               const NoiseModels& noise_models)
{
  (void)noise_models; // TODO: Implement this for observation noise, rail imprecision, etc.

  ExperimentalData raw_data = takePictures(setup, experiment, noise_models);

  ExperimentalData filtered_data;
  if (experiment.filter_images)
  {
    for (std::size_t i = 0; i < raw_data.images.size(); ++i)
    {
      if (inSensorBounds(setup.camera.width, setup.camera.height, raw_data.images[i]))
      {
        filtered_data.images.push_back(raw_data.images[i]);
        filtered_data.rail_position.push_back(raw_data.rail_position[i]);
      }
      else
      {
        std::cerr << "Image at " << i << ", rail = " << raw_data.rail_position[i] << " was not in image bounds\n";
      }
    }
  }
  else
  {
    filtered_data = raw_data;
  }

  return filtered_data;
}

void runExperiment1()
{
  PhysicalSetup cell = makeGroundTruth();

  // Define the experiment
  ExperimentalSetup experiment;
  experiment.min_distance = 0.5;
  experiment.max_distance = 2.0;
  experiment.increment = 0.25;
  experiment.filter_images = true;

  // Run data collection
  ExperimentalData data = runExperiment(cell, experiment, NoiseModels());

  // Setup optimization
  // Guesses
  PinholeCamera guess_camera = cell.camera;
  double target_pose[6];
  target_pose[0] = M_PI;  // rx
  target_pose[1] = 0.0;   // ry
  target_pose[2] = 0.0;   // rz
  target_pose[3] = 0.0;   // x
  target_pose[4] = 0.0;   // y
  target_pose[5] = 0.5;   // z

  ceres::Problem problem;

  // Build costs
  for (std::size_t i = 0; i < data.images.size(); ++i)
  {
    const double rail_position = data.rail_position[i];

    for (std::size_t j = 0; j < data.images[i].size(); ++j)
    {
      Eigen::Vector2d in_image = data.images[i][j];
      Eigen::Vector3d in_target = cell.target.points()[j];

      problem.AddResidualBlock(RailICal::Create(in_image.x(), in_image.y(), rail_position, in_target),
                               NULL, guess_camera.intrinsics.data(), target_pose);

    } // for each circle
  } // for each image


  // Solve
  ceres::Solver::Options options;
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
