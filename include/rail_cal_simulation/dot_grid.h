#ifndef DOT_GRID_H
#define DOT_GRID_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>


class DotGrid
{
public:
  using PointVector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

  DotGrid(int rows, int cols, double spacing);

  PointVector& points() { return target_points_; }
  const PointVector& points() const { return target_points_; }

  double physicalWidth() const;
  double physicalHeight() const;
  int rows() const { return rows_; }
  int cols() const { return cols_; }

private:
  int rows_, cols_;
  double spacing_;
  PointVector target_points_;
};

std::ostream& operator<<(std::ostream& os, const DotGrid& grid);

#endif // DOT_GRID_H
