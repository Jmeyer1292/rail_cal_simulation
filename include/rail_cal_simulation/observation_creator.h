#ifndef RAIL_CAL_SIM_OBS_CREATOR_H
#define RAIL_CAL_SIM_OBS_CREATOR_H

#include <rail_cal_simulation/camera_model.h>
#include <rail_cal_simulation/dot_grid.h>
#include <Eigen/Dense>

template <typename T>
using EigenSTLVector = std::vector<T, Eigen::aligned_allocator<T>>;

EigenSTLVector<Eigen::Vector2d> projectGrid(const Eigen::Affine3d& camera_to_target,
                                            const PinholeCamera& camera, const DotGrid& grid);

// Checks to see if ALL projected points are inside the sensor bounds
bool inSensorBounds(const int width, const int height, const EigenSTLVector<Eigen::Vector2d>& pts_in_image);



#endif // RAIL_CAL_SIM_OBS_CREATOR_H
