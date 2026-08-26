#include "HexapodController.h"

#include <math.h>

#include "Kinematics.h"
#include "ServoConfig.h"

HexapodController::HexapodController()
    : driverA_(config::PCA9685_ADDRESS_A),
      driverB_(config::PCA9685_ADDRESS_B),
      hasFootPose_(false),
      lastIKErrorMask_(0) {
  resetRuntimeTrims();
  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    currentAngles_[leg] = {0.0f, 90.0f, 90.0f};
    currentFootTargets_[leg] = {config::DEFAULT_REACH_MM, 0.0f, -config::DEFAULT_BODY_HEIGHT_MM};
  }
}

bool HexapodController::begin() {
  const bool aReady = driverA_.begin(Wire);
  const bool bReady = driverB_.begin(Wire);
  if (!aReady || !bReady) return false;
  if (!driverA_.setPWMFrequency(config::SERVO_FREQUENCY_HZ)) return false;
  if (!driverB_.setPWMFrequency(config::SERVO_FREQUENCY_HZ)) return false;
  relax();
  return true;
}

bool HexapodController::applyFootTargets(const Vec3 targets[config::LEG_COUNT]) {
  JointAngles solved[config::LEG_COUNT];
  lastIKErrorMask_ = 0;

  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    if (!Kinematics::solveLeg(targets[leg], solved[leg])) {
      lastIKErrorMask_ |= static_cast<uint8_t>(1U << leg);
    }
  }
  if (lastIKErrorMask_ != 0) return false;

  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    if (!writeLegAngles(leg, solved[leg])) return false;
  }
  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    currentFootTargets_[leg] = targets[leg];
  }
  hasFootPose_ = true;
  return true;
}

bool HexapodController::moveToFootTargets(const Vec3 targets[config::LEG_COUNT], uint16_t durationMs) {
  if (!hasFootPose_ || durationMs < config::CONTROL_INTERVAL_MS) {
    return applyFootTargets(targets);
  }

  Vec3 start[config::LEG_COUNT];
  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) start[leg] = currentFootTargets_[leg];

  const uint16_t steps = static_cast<uint16_t>(durationMs / config::CONTROL_INTERVAL_MS);
  for (uint16_t step = 1; step <= steps; ++step) {
    const float t = static_cast<float>(step) / static_cast<float>(steps);
    const float smooth = t * t * (3.0f - 2.0f * t);
    Vec3 intermediate[config::LEG_COUNT];
    for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
      intermediate[leg].x = start[leg].x + (targets[leg].x - start[leg].x) * smooth;
      intermediate[leg].y = start[leg].y + (targets[leg].y - start[leg].y) * smooth;
      intermediate[leg].z = start[leg].z + (targets[leg].z - start[leg].z) * smooth;
    }
    if (!applyFootTargets(intermediate)) return false;
    delay(config::CONTROL_INTERVAL_MS);
  }
  return true;
}

bool HexapodController::manualJoint(uint8_t leg, Joint joint, float logicalAngleDeg) {
  if (leg >= config::LEG_COUNT) return false;
  hasFootPose_ = false;
  return writeJoint(leg, joint, logicalAngleDeg);
}

bool HexapodController::neutralPose() {
  hasFootPose_ = false;
  bool ok = true;
  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    ok = writeLegAngles(leg, {0.0f, 90.0f, 90.0f}) && ok;
  }
  return ok;
}

void HexapodController::relax() {
  driverA_.setAllChannelsOff();
  driverB_.setAllChannelsOff();
  hasFootPose_ = false;
}

bool HexapodController::setTrim(uint8_t leg, Joint joint, float trimDeg) {
  if (leg >= config::LEG_COUNT || !isfinite(trimDeg) || fabsf(trimDeg) > config::MAX_TRIM_DEG) {
    return false;
  }
  runtimeTrimDeg_[servoIndex(leg, joint)] = trimDeg;
  return true;
}

void HexapodController::setRuntimeTrims(const float trims[config::SERVO_COUNT]) {
  for (uint8_t i = 0; i < config::SERVO_COUNT; ++i) {
    runtimeTrimDeg_[i] = (isfinite(trims[i]) && fabsf(trims[i]) <= config::MAX_TRIM_DEG) ? trims[i] : 0.0f;
  }
}

void HexapodController::getRuntimeTrims(float trims[config::SERVO_COUNT]) const {
  for (uint8_t i = 0; i < config::SERVO_COUNT; ++i) trims[i] = runtimeTrimDeg_[i];
}

void HexapodController::resetRuntimeTrims() {
  for (uint8_t i = 0; i < config::SERVO_COUNT; ++i) runtimeTrimDeg_[i] = 0.0f;
}

bool HexapodController::writeLegAngles(uint8_t leg, const JointAngles &angles) {
  return writeJoint(leg, Joint::COXA, angles.coxa) &&
         writeJoint(leg, Joint::FEMUR, angles.femur) &&
         writeJoint(leg, Joint::TIBIA, angles.tibia);
}

bool HexapodController::writeJoint(uint8_t leg, Joint joint, float logicalAngleDeg) {
  if (leg >= config::LEG_COUNT || !isfinite(logicalAngleDeg)) return false;
  const uint8_t index = servoIndex(leg, joint);
  const ServoCalibration &cal = SERVO_CALIBRATIONS[index];
  const uint16_t pulse = angleToPulse(index, logicalAngleDeg);
  if (!driverFor(cal.driver).writeMicroseconds(cal.channel, pulse)) return false;
  setJointValue(currentAngles_[leg], joint, logicalAngleDeg);
  return true;
}

uint16_t HexapodController::angleToPulse(uint8_t index, float logicalAngleDeg) const {
  const ServoCalibration &cal = SERVO_CALIBRATIONS[index];
  const float offset = logicalAngleDeg - cal.neutralLogicalDeg +
                       cal.defaultTrimDeg + runtimeTrimDeg_[index];
  float pulse = cal.centerPulseUs + cal.direction * offset * cal.microsecondsPerDegree;
  if (pulse < cal.minPulseUs) pulse = cal.minPulseUs;
  if (pulse > cal.maxPulseUs) pulse = cal.maxPulseUs;
  return static_cast<uint16_t>(lroundf(pulse));
}

PCA9685Driver &HexapodController::driverFor(uint8_t driver) {
  return driver == 0 ? driverA_ : driverB_;
}

