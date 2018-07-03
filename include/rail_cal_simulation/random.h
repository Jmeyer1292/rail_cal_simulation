#ifndef RAIL_CAL_SIM_RANDOM_H
#define RAIL_CAL_SIM_RANDOM_H

#include <memory>
#include <random>
#include <rail_cal_simulation/camera_model.h>

PinholeCamera randomizeCamera(const PinholeCamera& input,
                              const double focal_length_variance,
                              const double center_point_variance,
                              const double radial_dist_variance,
                              const double tang_dist_variance,
                              std::shared_ptr<std::default_random_engine> rng);



#endif // RAIL_CAL_SIM_RANDOM_H
