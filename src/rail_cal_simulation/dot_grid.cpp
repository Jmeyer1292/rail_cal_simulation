#include "rail_cal_simulation/dot_grid.h"
#include <cassert>

static void makePoints(int rows, int cols, double spacing, DotGrid::PointVector& points)
{
  points.reserve(rows * cols);

  for (int i = 1; i < (rows + 1); i++)
  {
    double y = (rows - i) * spacing;
    for (int j = 0; j < cols; j++)
    {
      double x = j * spacing;
      Eigen::Vector3d point (x, y, 0.0);
      points.push_back(point);
    }
  }
}

DotGrid::DotGrid(int rows, int cols, double spacing)
  : rows_(rows), cols_(cols), spacing_(spacing)
{
  assert(rows > 0);
  assert(cols > 0);
  assert(spacing > 0.0);

  // Fill out target_points_
  makePoints(rows, cols, spacing, target_points_);
}

double DotGrid::physicalWidth() const
{
  return (cols_ - 1) * spacing_;
}

double DotGrid::physicalHeight() const
{
  return (rows_ - 1) * spacing_;
}

std::ostream &operator<<(std::ostream &os, const DotGrid &grid)
{
  os << "---Dot Grid---\n";
  os << "   rows(y): " << grid.rows() << "\n";
  os << "   cols(x): " << grid.cols() << "\n";
  os << "   height(meters, y): " << grid.physicalHeight() << "\n";
  os << "   width(meters, x): " << grid.physicalWidth() << "\n";
  os << "---\n";
  return os;
}
