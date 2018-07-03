#include "rail_cal_simulation/camera_model.h"

std::ostream &operator<<(std::ostream &os, const PinholeCamera &camera)
{
  os << "---Pinhole Camera---\n";
  os << "   width: " << camera.width << "\n";
  os << "   height: " << camera.height << "\n";
  os << "   fx: " << camera.intrinsics.data()[0] << "\n";
  os << "   fy: " << camera.intrinsics.data()[1] << "\n";
  os << "   cx: " << camera.intrinsics.data()[2] << "\n";
  os << "   cy: " << camera.intrinsics.data()[3] << "\n";
  os << "   k1: " << camera.intrinsics.data()[4] << "\n";
  os << "   k2: " << camera.intrinsics.data()[5] << "\n";
  os << "   k3: " << camera.intrinsics.data()[6] << "\n";
  os << "   p1: " << camera.intrinsics.data()[7] << "\n";
  os << "   p2: " << camera.intrinsics.data()[8] << "\n";
  os << "---\n";
  return os;
}
