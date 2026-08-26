#pragma once

#include "Config.h"
#include "HexapodController.h"
#include "Types.h"

class GaitEngine {
 public:
  GaitEngine();

  void begin(HexapodController &controller);
  bool stand();
  bool sit();
  bool stop();
  void relax();
  void emergencyStop();
  void clearEmergency();
  bool setVelocity(float forward, float lateral, float turn);
  bool update(uint32_t nowMs);

  void setSpeedPercent(uint8_t percent);
  void setStrideLength(float millimetres);
  void setStepHeight(float millimetres);
  bool setBodyHeight(float millimetres);

  MotionState state() const { return state_; }
  const char *stateName() const;
  bool isWalking() const { return state_ == MotionState::WALKING; }
  bool emergencyLatched() const { return state_ == MotionState::EMERGENCY_STOP; }
  bool lastIKFault() const { return lastIKFault_; }
  float bodyHeight() const { return bodyHeightMm_; }
  float strideLength() const { return strideLengthMm_; }
  float stepHeight() const { return stepHeightMm_; }
  uint8_t speedPercent() const { return speedPercent_; }

 private:
  void buildNeutralTargets(float bodyHeight, Vec3 targets[config::LEG_COUNT]) const;
  static float clampNormalized(float value);
  static float smoothStep(float value);

  HexapodController *controller_;
  MotionState state_;
  float forward_;
  float lateral_;
  float turn_;
  float bodyHeightMm_;
  float strideLengthMm_;
  float stepHeightMm_;
  uint8_t speedPercent_;
  uint32_t phaseStartMs_;
  uint32_t lastUpdateMs_;
  bool lastIKFault_;
};

