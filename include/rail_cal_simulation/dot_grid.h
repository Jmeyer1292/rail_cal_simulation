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
  DotGrid() = default;

  PointVector& points() { return target_points_; }
  const PointVector& points() const { return target_points_; }

  double physicalWidth() const;
  double physicalHeight() const;
  int rows() const { return rows_; }
  int cols() const { return cols_; }

public:
  int rows_, cols_;
  double spacing_;
  PointVector target_points_;
};

class ThreeDimensionalGrid
{
public:
  ThreeDimensionalGrid(const Eigen::Vector3i& grid_dimensions, double spacing)
    : grid_dimensions_(grid_dimensions)
    , spacing_(spacing)
  {
  }

  ThreeDimensionalGrid()
    : grid_dimensions_(Eigen::Vector3i::Zero())
    , spacing_(0.0)
  {}

  const Eigen::Vector3i& dimensions() const { return grid_dimensions_; }
  double spacing() const { return spacing_; }

  Eigen::Vector3d positionOf(const Eigen::Vector3i& index) const
  {
    return spacing_ * index.cast<double>();
  }

private:
  Eigen::Vector3i grid_dimensions_;
  double spacing_;
};

std::ostream& operator<<(std::ostream& os, const DotGrid& grid);

#endif // DOT_GRID_H
