#pragma once

#include "Types.h"

class Kinematics {
 public:
  // Solves one leg in its local frame: +X outward, +Y tangential, -Z downward.
  static bool solveLeg(const Vec3 &foot, JointAngles &angles);
};

