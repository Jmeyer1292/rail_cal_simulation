#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <array>
#include <iostream>

class Intrinsics
{
public:
  double* data() { return values.data(); }
  const double* data() const { return values.data(); }

private:
  std::array<double, 9> values;
};

struct PinholeCamera
{
  // Sensor Parameters
  Intrinsics intrinsics;

  // Camera Intrinsics
  int width;
  int height;
};

std::ostream& operator<<(std::ostream& os, const PinholeCamera& camera);

#endif // CAMERA_MODEL_H
