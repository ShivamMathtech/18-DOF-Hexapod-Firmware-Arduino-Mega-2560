#include "Kinematics.h"

#include <math.h>

#include "Config.h"

namespace {
float clampUnit(float value) {
  if (value < -1.0f) return -1.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float radiansToDegrees(float radians) {
  return radians * 180.0f / PI;
}
}

bool Kinematics::solveLeg(const Vec3 &foot, JointAngles &angles) {
  const float horizontal = sqrtf(foot.x * foot.x + foot.y * foot.y);
  const float planarReach = horizontal - config::COXA_LENGTH_MM;
  const float vertical = -foot.z;
  const float distance = sqrtf(planarReach * planarReach + vertical * vertical);

  const float maxReach = config::FEMUR_LENGTH_MM + config::TIBIA_LENGTH_MM;
  const float minReach = fabsf(config::FEMUR_LENGTH_MM - config::TIBIA_LENGTH_MM);
  if (planarReach <= 1.0f || distance > maxReach - 0.5f || distance < minReach + 0.5f) {
    return false;
  }

  const float coxaYaw = atan2f(foot.y, foot.x);
  const float femurCos =
      (config::FEMUR_LENGTH_MM * config::FEMUR_LENGTH_MM + distance * distance -
       config::TIBIA_LENGTH_MM * config::TIBIA_LENGTH_MM) /
      (2.0f * config::FEMUR_LENGTH_MM * distance);
  const float kneeCos =
      (config::FEMUR_LENGTH_MM * config::FEMUR_LENGTH_MM +
       config::TIBIA_LENGTH_MM * config::TIBIA_LENGTH_MM - distance * distance) /
      (2.0f * config::FEMUR_LENGTH_MM * config::TIBIA_LENGTH_MM);

  const float lineAngle = atan2f(vertical, planarReach);
  const float femurPitch = lineAngle + acosf(clampUnit(femurCos));
  const float kneeInterior = acosf(clampUnit(kneeCos));

  angles.coxa = radiansToDegrees(coxaYaw);
  angles.femur = radiansToDegrees(femurPitch);
  angles.tibia = 180.0f - radiansToDegrees(kneeInterior);

  return angles.coxa >= -60.0f && angles.coxa <= 60.0f &&
         angles.femur >= 15.0f && angles.femur <= 155.0f &&
         angles.tibia >= 10.0f && angles.tibia <= 165.0f;
}

