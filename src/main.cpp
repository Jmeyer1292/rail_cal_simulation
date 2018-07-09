#include <ceres/ceres.h>
#include <iostream>
#include <ros/ros.h>

#include <rail_cal_simulation/camera_model.h>
#include <rail_cal_simulation/cost_function.h>
#include <rail_cal_simulation/dot_grid.h>
#include <rail_cal_simulation/observation_creator.h>
#include <rail_cal_simulation/random.h>
#include <rail_cal_simulation/utilities.h>
#include <Eigen/Dense>

static PinholeCamera makeCamera(bool randomize, std::shared_ptr<std::default_random_engine> rng)
{
  PinholeCamera camera;

  // Sensor parameters
  camera.width = 1600;
  camera.height = 1200;
  // Pinhole parameters
  camera.intrinsics.data()[0] = 800.0; // fx
  camera.intrinsics.data()[1] = 800.0; // fy
  camera.intrinsics.data()[2] = camera.width / 2; // cx
  camera.intrinsics.data()[3] = camera.height / 2; // cy
  // Distortion Parameters
  camera.intrinsics.data()[4] = -0.03; // k1
  camera.intrinsics.data()[5] = 0.05; // k2
  camera.intrinsics.data()[6] = 0.0; // k3
  camera.intrinsics.data()[7] = 0.002; // p1
  camera.intrinsics.data()[8] = 0.0; // p2

  if (randomize)
  {
    const static double focal_length_variance = 30.0;
    const static double center_point_variance = 10.0;
    const static double radial_dist_variance = 0.01;
    const static double tang_dist_variance = 0.01;
    camera = randomizeCamera(camera, focal_length_variance, center_point_variance, radial_dist_variance,
                             tang_dist_variance, rng);
  }

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
  Eigen::Vector3d rail_travel_in_camera; // Direction of rail motion in camera frame; nominally 0, 0, -1
                                         // but we can't be perfect.
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
    // First move the camera back the minimum distance along the origin vector (e.g. it's pointing at the right spot to start)
    Eigen::Affine3d camera_pose = cell.camera_origin_pose * Eigen::Translation3d(0, 0, -experiment.min_distance);
    // Then move back along the (possibly skewed) camera motion axis, simulating rail motion
    camera_pose = camera_pose * Eigen::Translation3d((s - experiment.min_distance) * cell.rail_travel_in_camera);

    Eigen::Affine3d target_in_camera = camera_pose.inverse() * cell.target_pose;

    EigenSTLVector<Eigen::Vector2d> grid_in_image = projectGrid(target_in_camera, cell.camera, cell.target);
    data.images.push_back(grid_in_image);
    data.rail_position.push_back(s - experiment.min_distance);
  }

  return data;
}

PhysicalSetup makeGroundTruth(std::shared_ptr<std::default_random_engine> rng)
{
  PhysicalSetup cell;
  cell.camera = makeCamera(true, rng);
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

  // Now perturb the camera/target intersection
  const double spatial_noise = 0.01; // +/-
  const double angular_noise = 30.0 * M_PI / 180.0; // +/- deg

  // TODO: Take a seed - currently you get a new random pertubation each time
  cell.camera_origin_pose = perturbPose(cell.camera_origin_pose, spatial_noise, angular_noise, rng);

  // Set the axis of travel & then perturb it a little
  cell.rail_travel_in_camera = Eigen::Vector3d(0, 0, -1);
  const static double x_var = 0.05;
  const static double y_var = 0.05;
  cell.rail_travel_in_camera = perturbOrientation(cell.rail_travel_in_camera, x_var, y_var, rng);

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

bool runExperiment(std::shared_ptr<std::default_random_engine> rng)
{
  PhysicalSetup cell = makeGroundTruth(rng);

  std::cout << "--- Ground Truth ---\n";
  std::cout << cell.camera << "\n";

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
  PinholeCamera guess_camera = makeCamera(false, rng); // generate a "perfect", gaussian centered camera

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
//  std::cout << summary.FullReport() << "\n";
  std::cout << "Init avg residual: " << summary.initial_cost / summary.num_residuals << "\n";
  std::cout << "Final avg residual: " << summary.final_cost / summary.num_residuals << "\n";

  std::cout << "---After minimization---\n";
  std::cout << guess_camera << "\n";


  std::cout << "---Target Pose---\n";
  std::cout << target_pose[0] << " " << target_pose[1] << " " << target_pose[2] << " "
            << target_pose[3] << " " << target_pose[4] << " " << target_pose[5] << "\n";

  std::cout << "---Camera Errors---\n";
  std::array<double, 9> diff = difference(cell.camera, guess_camera);
  bool answer_found = true;
  for (std::size_t i = 0; i < diff.size(); ++i)
  {
    if (abs(diff[i]) > 1e-3) answer_found = false;
    std::cout << "   diff(" << i << "): " << diff[i] << "\n";
  }
  return answer_found;
}

// Uses railcal3 to estimate the axis of motion
bool runExperiment2(std::shared_ptr<std::default_random_engine> rng)
{
  PhysicalSetup cell = makeGroundTruth(rng);

  std::cout << "--- Ground Truth ---\n";
  std::cout << cell.camera << "\n";

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
  PinholeCamera guess_camera = makeCamera(false, rng); // generate a "perfect", gaussian centered camera

  double target_pose[6];
  target_pose[0] = M_PI;  // rx
  target_pose[1] = 0.0;   // ry
  target_pose[2] = 0.0;   // rz
  target_pose[3] = 0.0;   // x
  target_pose[4] = 0.0;   // y
  target_pose[5] = 0.5;   // z

  Eigen::Vector3d rail_travel_axis = Eigen::Vector3d(0,0,1).normalized();
  rail_travel_axis = -1.0 * cell.rail_travel_in_camera;

  ceres::Problem problem;

  // Build costs
  for (std::size_t i = 0; i < data.images.size(); ++i)
  {
    const double rail_displacement = data.rail_position[i];
    const Eigen::Vector3d rail_position = rail_displacement * rail_travel_axis;

    for (std::size_t j = 0; j < data.images[i].size(); ++j)
    {
      Eigen::Vector2d in_image = data.images[i][j];
      Eigen::Vector3d in_target = cell.target.points()[j];

      problem.AddResidualBlock(RailICal3::Create(in_image.x(), in_image.y(), rail_position, in_target),
                               NULL, guess_camera.intrinsics.data(), target_pose);

    } // for each circle
  } // for each image


  // Solve
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Analyze results
//  std::cout << summary.FullReport() << "\n";
  std::cout << "Init avg residual: " << summary.initial_cost / summary.num_residuals << "\n";
  std::cout << "Final avg residual: " << summary.final_cost / summary.num_residuals << "\n";

  std::cout << "---After minimization---\n";
  std::cout << guess_camera << "\n";


  std::cout << "---Target Pose---\n";
  std::cout << target_pose[0] << " " << target_pose[1] << " " << target_pose[2] << " "
            << target_pose[3] << " " << target_pose[4] << " " << target_pose[5] << "\n";

  std::cout << "---Camera Errors---\n";
  std::array<double, 9> diff = difference(cell.camera, guess_camera);
  bool answer_found = true;
  for (std::size_t i = 0; i < diff.size(); ++i)
  {
    if (abs(diff[i]) > 1e-3) answer_found = false;
    std::cout << "   diff(" << i << "): " << diff[i] << "\n";
  }
  return answer_found;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rail_cal_sim");
  ros::NodeHandle pnh ("~");

  int seed = pnh.param<int>("seed", 0);
  int n_trials = pnh.param<int>("n_trials", 1);

  std::shared_ptr<std::default_random_engine> rng (new std::default_random_engine(seed));
  for (int i = 0; i < n_trials; ++i)
  {
    bool r = runExperiment2(rng);
    if (!r) std::cout << "Experiment " << i << " failed to converge to correct answer (seed = " << seed << ")\n";
  }
}
