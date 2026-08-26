#pragma once

#include "Config.h"
#include "PCA9685Driver.h"
#include "Types.h"

class HexapodController {
 public:
  HexapodController();

  bool begin();
  bool applyFootTargets(const Vec3 targets[config::LEG_COUNT]);
  bool moveToFootTargets(const Vec3 targets[config::LEG_COUNT], uint16_t durationMs);
  bool manualJoint(uint8_t leg, Joint joint, float logicalAngleDeg);
  bool neutralPose();
  void relax();

  bool setTrim(uint8_t leg, Joint joint, float trimDeg);
  void setRuntimeTrims(const float trims[config::SERVO_COUNT]);
  void getRuntimeTrims(float trims[config::SERVO_COUNT]) const;
  void resetRuntimeTrims();

  bool hasFootPose() const { return hasFootPose_; }
  const Vec3 *currentFootTargets() const { return currentFootTargets_; }
  uint8_t lastIKErrorMask() const { return lastIKErrorMask_; }
  bool driversReady() const { return driverA_.ready() && driverB_.ready(); }

 private:
  bool writeLegAngles(uint8_t leg, const JointAngles &angles);
  bool writeJoint(uint8_t leg, Joint joint, float logicalAngleDeg);
  uint16_t angleToPulse(uint8_t index, float logicalAngleDeg) const;
  PCA9685Driver &driverFor(uint8_t driver);

  PCA9685Driver driverA_;
  PCA9685Driver driverB_;
  float runtimeTrimDeg_[config::SERVO_COUNT];
  JointAngles currentAngles_[config::LEG_COUNT];
  Vec3 currentFootTargets_[config::LEG_COUNT];
  bool hasFootPose_;
  uint8_t lastIKErrorMask_;
};

