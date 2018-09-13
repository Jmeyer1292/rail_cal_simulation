#include <ros/ros.h>
#include <rail_cal_simulation/camera_model.h>
#include <rail_cal_simulation/cost_function.h>
#include <rail_cal_simulation/dot_grid.h>
#include <rail_cal_simulation/observation_creator.h>
#include <rail_cal_simulation/random.h>
#include <rail_cal_simulation/utilities.h>

#include <ceres/ceres.h>

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

static ThreeDimensionalGrid makeTarget()
{
  return ThreeDimensionalGrid(Eigen::Vector3i(100, 100, 100), 0.01);
}

struct PhysicalSetup
{
  PinholeCamera camera;
  ThreeDimensionalGrid target;
  Eigen::Affine3d target_pose;
};

struct ExperimentalData
{
  using ObsPair = std::pair<Eigen::Vector3d, Eigen::Vector2d>; // <in_target, in_image>
  EigenSTLVector<ObsPair> image_points;
};

ExperimentalData takePictures(const PhysicalSetup& cell)
{
  // Loop through the points of the grid...
//  const Eigen::Vector3i dims = cell.target.

  return {};
}

PhysicalSetup makeGroundTruth(std::shared_ptr<std::default_random_engine> rng)
{
  PhysicalSetup cell;
  cell.camera = makeCamera(true, rng);
  cell.target = makeTarget();
  cell.target_pose.setIdentity();

  return cell;
}

bool runExperiment(std::shared_ptr<std::default_random_engine> rng)
{
  PhysicalSetup cell = makeGroundTruth(rng);

  std::cout << "--- Ground Truth ---\n";
  std::cout << cell.camera << "\n";

  // Run data collection
  ExperimentalData data = takePictures(cell);

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

  for (const auto& p : data.image_points)
  {
    const Eigen::Vector3d& pt_in_target = p.first;
    const Eigen::Vector2d& pt_in_image = p.second;

    problem.AddResidualBlock(IntrCostFunctor::Create(pt_in_target, pt_in_image), NULL, guess_camera.intrinsics.data(),
                             target_pose);
  }

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


  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rail_cal_sim");
  ros::NodeHandle pnh ("~");

  int seed = pnh.param<int>("seed", 0);
  int n_trials = pnh.param<int>("n_trials", 1);
}
