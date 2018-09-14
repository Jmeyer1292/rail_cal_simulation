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
  return ThreeDimensionalGrid(Eigen::Vector3i(150, 100, 50), 0.1);
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

        const auto point_in_image = projectPoint(cell.camera, point_in_camera);

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
  cv::Mat Obs_Image(C.width, C.height, CV_8UC1, cv::Scalar::all(0)); // black and white image is fine
  for (const auto& pt : data.image_points)
  {
    int row = (int) pt.second.y();
    int col = (int) pt.second.x();
    Obs_Image.at<unsigned int>(row,col) = 255;
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
  cell.target_pose.rotate(Eigen::AngleAxisd(.17,Eigen::Vector3d(.1,.2,.1).normalized()));

  cell.target_pose.translation() = Eigen::Vector3d(0.25, 0.25, 0.25);

  return cell;
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
  PhysicalSetup cell = makeGroundTruth(rng);

  std::cout << "--- Ground Truth ---\n";
  std::cout << cell.camera << "\n";

  // Run data collection
  ExperimentalData data = takePictures(cell);
  printExtents(data);

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


  for (const auto& p : data.image_points)
  {
    const Eigen::Vector3d& pt_in_target = p.first;
    const Eigen::Vector2d& pt_in_image = p.second;

    problem.AddResidualBlock(IntrCostFunctor::Create(pt_in_target, pt_in_image), NULL, target_pose, guess_camera.intrinsics.data());
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
  std::cout << "\t" << target_pose[0] << " " << target_pose[1] << " " << target_pose[2] << "\n"
            << "\t" << target_pose[3] << " " << target_pose[4] << " " << target_pose[5] << "\n";

  std::cout << "---Camera Errors---\n";
  std::array<double, 9> diff = difference(cell.camera, guess_camera);
  bool answer_found = true;
  for (std::size_t i = 0; i < diff.size(); ++i)
  {
    if (abs(diff[i]) > 1e-3) answer_found = false;
    std::cout << "   diff(" << i << "): " << diff[i] << "\n";
  }

  // print the covariance block
  std::string cov_file_name("Jon_is_here.txt");
  computeCovariance(cov_file_name, guess_camera, problem );

  return true;
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
}
