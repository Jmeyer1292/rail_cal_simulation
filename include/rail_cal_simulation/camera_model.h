#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <array>
#include <iostream>

class Intrinsics
{
public:
  double* data() { return values.data(); }
  const double* data() const { return values.data(); }

  const double& fx() const { return values[0]; }
  const double& fy() const { return values[1]; }
  const double& cx() const { return values[2]; }
  const double& cy() const { return values[3]; }
  const double& k1() const { return values[4]; }
  const double& k2() const { return values[5]; }
  const double& k3() const { return values[6]; }
  const double& p1() const { return values[7]; }
  const double& p2() const { return values[8]; }

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

std::array<double, 9> difference(const PinholeCamera& c1, const PinholeCamera& c2);

#endif // CAMERA_MODEL_H
