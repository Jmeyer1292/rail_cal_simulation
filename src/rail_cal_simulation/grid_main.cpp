#include <ros/ros.h>
#include <rail_cal_simulation/camera_model.h>
#include <rail_cal_simulation/cost_function.h>
#include <rail_cal_simulation/dot_grid.h>
#include <rail_cal_simulation/observation_creator.h>
#include <rail_cal_simulation/random.h>
#include <rail_cal_simulation/utilities.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rail_cal_simulation/mutable_model_costs.h>

#include <ceres/ceres.h>

static Eigen::Vector2d pixelNoise(std::normal_distribution<double>& dist,
                                  std::default_random_engine& engine)
{
  return Eigen::Vector2d(dist(engine), dist(engine));
}

static PinholeCamera makeCamera(bool randomize, std::shared_ptr<std::default_random_engine> rng)
{
  PinholeCamera camera;

  // Sensor parameters
  camera.width = 1600;
  camera.height = 1200;
  // Pinhole parameters
  camera.intrinsics.data()[0] = 500.0; // fx
  camera.intrinsics.data()[1] = 500.0; // fy
  camera.intrinsics.data()[2] = camera.width / 2; // cx
  camera.intrinsics.data()[3] = camera.height / 2; // cy
  // Distortion Parameters
  camera.intrinsics.data()[4] = 0.0; // k1
  camera.intrinsics.data()[5] = 0.0; // k2
  camera.intrinsics.data()[6] = 0.0; // k3
  camera.intrinsics.data()[7] = 0.0; // p1
  camera.intrinsics.data()[8] = 0.0; // p2

  if (randomize)
  {
    const static double focal_length_variance = 30.0;
    const static double center_point_variance = 10.0;
    const static double k1_k2_dist_variance = 0.001;
    const static double k3_dist_variance = 0.001;
    const static double tang_dist_variance = 0.00;
    camera = randomizeCamera(camera, focal_length_variance, center_point_variance, k1_k2_dist_variance, k3_dist_variance,
                             tang_dist_variance, rng);
    camera.intrinsics.data()[1] = camera.intrinsics.data()[0];
  }

  return camera;
}

static ThreeDimensionalGrid makeTarget()
{
//  return ThreeDimensionalGrid(Eigen::Vector3i(200, 100, 100), 0.1);
  return ThreeDimensionalGrid(Eigen::Vector3i(50, 50, 50), 0.4);
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

void addNoise(ExperimentalData& data, std::shared_ptr<std::default_random_engine> engine)
{
  return;
//  std::normal_distribution<double> pixel_noise (0.0, 0.1);

//  for (auto& sample : data.image_points)
//  {
//    auto& pt_in_image = sample.second;
//    pt_in_image += pixelNoise(pixel_noise, *engine);
//  }
}

ExperimentalData takePictures(const PhysicalSetup& cell)
{
  // Loop through the points of the grid...
  const Eigen::Vector3i dims = cell.target.dimensions();
  ExperimentalData data;

  for (int x = 0; x < dims.x(); ++x)
  {
    for (int y = 0; y  < dims.y(); ++y)
    {
      for (int z = 0; z < dims.z(); ++z)
      {
        // Compute location of point
        const Eigen::Vector3d point_in_target = cell.target.positionOf(Eigen::Vector3i(x, y, z));
        const Eigen::Vector3d point_in_camera = cell.target_pose * point_in_target;

        // Test to see if point is "visible"
        if (point_in_camera.z() <= 0.0) // Not visible if behind camera
          continue;

        Eigen::Vector2d point_in_image;
        if (!projectAndTest(cell.camera, point_in_camera, point_in_image))
        {
          continue;
        }
//        const auto point_in_image = projectPoint(cell.camera, point_in_camera);

        // Test if the point was projected onto the image plane (dx)
        if (point_in_image.x() < 0.0 || point_in_image.x() > cell.camera.width)
          continue;

        // Test if the point was projected onto the image plane (dy)
        if (point_in_image.y() < 0.0 || point_in_image.y() > cell.camera.height)
          continue;

        // Point was seen, so add it
        data.image_points.emplace_back(point_in_target, point_in_image);
      }
    }
  }

  return data;
}

static void printExtents(const ExperimentalData& data)
{
  Eigen::Vector2d min (std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  Eigen::Vector2d max (-1., -1.);

  for (const auto& pt : data.image_points)
  {
    min = min.array().min(pt.second.array());
    max = max.array().max(pt.second.array());
  }

  std::cout << "MIN " << min.transpose() << "\n";
  std::cout << "MAX " << max.transpose() << "\n";
}

void makePicture(PinholeCamera &C, ExperimentalData& data)
{
  cv::Mat Obs_Image(C.height, C.width, CV_8UC1, cv::Scalar::all(0)); // black and white image is fine
  for (const auto& pt : data.image_points)
  {
    int row = static_cast<int>(pt.second.y() + 0.5);
    int col = static_cast<int>(pt.second.x() + 0.5);

    if (row >= 0 && row < C.height && col >= 0 && col < C.width)
      Obs_Image.at<unsigned char>(row,col) = 255;
  }
  imshow("Observation Image", Obs_Image);
  cv::waitKey(0);
}

PhysicalSetup makeGroundTruth(std::shared_ptr<std::default_random_engine> rng)
{
  PhysicalSetup cell;
  cell.camera = makeCamera(true, rng);
  cell.target = makeTarget();

  cell.target_pose.setIdentity();
  cell.target_pose.matrix().col(0).head<3>() = Eigen::Vector3d(0, 0, 1);
  cell.target_pose.matrix().col(1).head<3>() = Eigen::Vector3d(-1., 0, 0);
  cell.target_pose.matrix().col(2).head<3>() = Eigen::Vector3d(0, -1, 0);
  cell.target_pose.rotate(Eigen::AngleAxisd(.10, Eigen::Vector3d(.1,.2,.1).normalized()));
//  cell.target_pose.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));

  const auto grid_size = cell.target.positionOf(cell.target.dimensions());
  std::cout << "GRID SIZE: " << grid_size.transpose() << "\n";
  cell.target_pose.translation() = Eigen::Vector3d(grid_size.y() / 2., grid_size.z() / 2., grid_size.x() / 5.0);

  std::cout << "Grid pose:\n" << cell.target_pose.matrix() << "\n\n";

  return cell;
}

bool computeCovariance2(const std::string& filename, const double* camera_params, const std::size_t param_size, ceres::Problem& problem)
{
  // Parameter setup
  ceres::Covariance::Options cov_options;
  cov_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covariance(cov_options);

  // We want to compute the variance of the camera matrix with itself
  std::vector<std::pair<const double*, const double*>> cov_pairs = {std::make_pair(camera_params, camera_params)};

  if (!covariance.Compute(cov_pairs, &problem))
  {
    return false;
  }

  // Allocate buffer to copy the covariance into
  std::vector<double> cov_in_in (param_size * param_size, 0.0);
  std::vector<double> ef (param_size, 0.0);
  if (!covariance.GetCovarianceBlock(camera_params, camera_params, cov_in_in.data()))
  {
    return false;
  }

  FILE* fp = std::fopen(filename.c_str(), "w");
  // Write to file
  if (!fp)
  {
    return false;
  }

  // Loop over
  for (std::size_t i = 0; i < param_size; ++i)
  {
    double sigma_i = std::sqrt(std::abs(cov_in_in[i * param_size + i]));
    for (std::size_t j = 0; j < param_size; ++j)
    {
      double sigma_j = std::sqrt(std::abs(cov_in_in[j * param_size + j]));
      double value;
      if (i == j)
      {
        value = sigma_i;
        ef[i] = sigma_i;
      }
      else
      {
        if (sigma_i == 0.0) sigma_i = 1.0;
        if (sigma_j == 0.0) sigma_j = 1.0;
        value = cov_in_in[i * param_size + j] / (sigma_i * sigma_j);
      }
      // Print
      std::fprintf(fp, "%16.5f ", value);
    }
    std::fprintf(fp, "\n");
  } // end of cov computation / normalization

  std::fclose(fp);
  return true;
}

bool computeCovariance(std::string& covariance_file_name, PinholeCamera &C, ceres::Problem &P )
{
  FILE* fp;
  if ((fp = fopen(covariance_file_name.c_str(), "w")) != NULL)
  {
    ceres::Covariance::Options covariance_options;
    covariance_options.algorithm_type = ceres::DENSE_SVD;
    ceres::Covariance covariance(covariance_options);

    double * pblock = & (C.intrinsics.data()[0]);

    std::vector<std::pair<const double*, const double*> > covariance_pairs;
    covariance_pairs.push_back(std::make_pair(pblock, pblock));

    if(covariance.Compute(covariance_pairs, &P))
    {
      fprintf(fp, "covariance blocks:\n");
      double cov_in_in[9*9];
      double ef[9];
      if(covariance.GetCovarianceBlock(pblock, pblock, cov_in_in)){
        fprintf(fp, "cov_in_in is 9x9\n");
        for(int i=0;i<9;i++){
          double sigma_i = sqrt(fabs(cov_in_in[i*9+i]));
          for(int j=0;j<9;j++){
            double sigma_j = sqrt(fabs(cov_in_in[j*9+j]));
            double value;
            if(i==j){
              value = sigma_i;
              ef[i] = sigma_i;
            }
            else{
              if(sigma_i==0) sigma_i = 1;
              if(sigma_j==0) sigma_j = 1;
              value = cov_in_in[i * 9 + j]/(sigma_i*sigma_j);
            }
            fprintf(fp, "%16.5f ", value);
          }  // end of j loop
          fprintf(fp, "\n");
        }  // end of i loop
      }// end if success getting covariance
      // compute the effect of the variance of each term in pixels
      fprintf(fp,"effect of variance at R = image_width/4\n");

      fprintf(fp,"fx fy: %16.5f %16.5f\n", ef[0], ef[1]);
      fprintf(fp,"cx cy: %16.5f %16.5f\n", ef[2], ef[3]);
      double R = (C.width + C.height)/6;
      double RR = R*R;
      fprintf(fp,"k1 k2 k3: %16.5f %16.5f %16.5f\n", ef[4]*RR, ef[5]*RR*RR, ef[6]*RR*RR*RR);
      fprintf(fp,"p1 p2: %16.5f %16.5f\n", ef[7]*3*RR, ef[8]*3*RR);
      fclose(fp);
      return(true);
    }// end if covariances could be computed
    else{
      ROS_ERROR("could not compute covariance");
    }// end could not compute covariance
  }// end if file opens
  ROS_ERROR("could not open covariance file %s", covariance_file_name.c_str());
  return (false);
};  // end computeCovariance()


bool runExperiment(std::shared_ptr<std::default_random_engine> rng)
{
  std::cout << "***New Trial***\n";
  PhysicalSetup cell = makeGroundTruth(rng);

  std::cout << "--- Ground Truth ---\n";
  std::cout << cell.camera << "\n";

  // Run data collection
  ExperimentalData data = takePictures(cell);
  printExtents(data);
  addNoise(data, rng);

  makePicture(cell.camera, data);

  // Setup optimization
  // Guesses
  PinholeCamera guess_camera = makeCamera(false, rng); // generate a "perfect", gaussian centered camera

  Eigen::AngleAxisd aaxis (cell.target_pose.linear());
  Eigen::Vector3d axis = aaxis.axis().normalized() * aaxis.angle();

  double target_pose[6];
  target_pose[0] = axis(0);  // rx
  target_pose[1] = axis(1);   // ry
  target_pose[2] = axis(2);   // rz
  target_pose[3] = 0.25;   // x
  target_pose[4] = 0.0;   // y
  target_pose[5] = 0.25;   // z

  ceres::Problem problem;

  // Build the reduced camera matrix to optimize over
  double camera_intr[5];
  camera_intr[0] = guess_camera.intrinsics.fx();
  camera_intr[1] = guess_camera.intrinsics.cx();
  camera_intr[2] = guess_camera.intrinsics.cy();
  camera_intr[3] = guess_camera.intrinsics.k1();
  camera_intr[4] = guess_camera.intrinsics.k2();

  for (const auto& p : data.image_points)
  {
    const Eigen::Vector3d& pt_in_target = p.first;
    const Eigen::Vector2d& pt_in_image = p.second;

    problem.AddResidualBlock(IntrFunctor<ReducedCameraModelMaker>::Create(pt_in_target, pt_in_image), NULL, target_pose, camera_intr);
//    problem.AddResidualBlock(IntrCostFunctor::Create(pt_in_target, pt_in_image), NULL, target_pose, guess_camera.intrinsics.data());
  }

  // Solve
  ceres::Solver::Options options;
  options.num_threads = 32;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Analyze results
  std::cout << "Init avg residual: " << std::sqrt(2.0 * summary.initial_cost / summary.num_residuals) << "\n";
  std::cout << "Final avg residual: " << std::sqrt(2.0 * summary.final_cost / summary.num_residuals) << "\n";

  // print the covariance block
  std::string cov_file_name("Jon_is_here.txt");
//  computeCovariance(cov_file_name, guess_camera, problem );
  computeCovariance2(cov_file_name, camera_intr, 5, problem);
  std::cout << "Covariance computation finished\n";

  return true;
}

static void drawLoop(std::shared_ptr<std::default_random_engine> rng)
{
  std::cout << "***New Trial***\n";
  PhysicalSetup cell = makeGroundTruth(rng);

  std::cout << "--- Ground Truth ---\n";
  std::cout << cell.camera << "\n";

  while (true)
  {
    ExperimentalData data = takePictures(cell);
    makePicture(cell.camera, data);

    int c = cv::waitKey(33);
    if (c == 'w') cell.target_pose.translation().x() += 0.1;
    if (c == 's') cell.target_pose.translation().x() -= 0.1;
    if (c == 'a') cell.target_pose.translation().y() += 0.1;
    if (c == 'd') cell.target_pose.translation().y() -= 0.1;
    if (c == 'q') cell.target_pose.translation().z() += 0.1;
    if (c == 'e') cell.target_pose.translation().z() -= 0.1;
    if (c == 'x') break;
  }
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
    bool r = runExperiment(rng);
    if (!r) ROS_WARN_STREAM("Experiment " << i << " failed to converge to correct answer (seed = " << seed << ")\n");
  }

//  drawLoop(rng);
}
