#include "GaitEngine.h"

#include <math.h>

GaitEngine::GaitEngine()
    : controller_(nullptr),
      state_(MotionState::RELAXED),
      forward_(0.0f),
      lateral_(0.0f),
      turn_(0.0f),
      bodyHeightMm_(config::DEFAULT_BODY_HEIGHT_MM),
      strideLengthMm_(config::DEFAULT_STRIDE_MM),
      stepHeightMm_(config::DEFAULT_STEP_HEIGHT_MM),
      speedPercent_(50),
      phaseStartMs_(0),
      lastUpdateMs_(0),
      lastIKFault_(false) {}

void GaitEngine::begin(HexapodController &controller) {
  controller_ = &controller;
  state_ = MotionState::RELAXED;
}

bool GaitEngine::stand() {
  if (!controller_ || emergencyLatched()) return false;
  Vec3 target[config::LEG_COUNT];
  if (!controller_->hasFootPose()) {
    buildNeutralTargets(config::MIN_BODY_HEIGHT_MM, target);
    if (!controller_->applyFootTargets(target)) return false;
    delay(300);
  }
  buildNeutralTargets(bodyHeightMm_, target);
  if (!controller_->moveToFootTargets(target, 700)) return false;
  state_ = MotionState::STANDING;
  lastIKFault_ = false;
  return true;
}

bool GaitEngine::sit() {
  if (!controller_ || emergencyLatched()) return false;
  if (!controller_->hasFootPose() && !stand()) return false;
  Vec3 target[config::LEG_COUNT];
  buildNeutralTargets(config::MIN_BODY_HEIGHT_MM, target);
  if (!controller_->moveToFootTargets(target, 600)) return false;
  state_ = MotionState::SITTING;
  return true;
}

bool GaitEngine::stop() {
  if (!controller_ || emergencyLatched()) return false;
  forward_ = lateral_ = turn_ = 0.0f;
  if (state_ == MotionState::WALKING) {
    Vec3 target[config::LEG_COUNT];
    buildNeutralTargets(bodyHeightMm_, target);
    if (!controller_->moveToFootTargets(target, 300)) return false;
  }
  state_ = MotionState::STANDING;
  return true;
}

void GaitEngine::relax() {
  if (controller_) controller_->relax();
  forward_ = lateral_ = turn_ = 0.0f;
  state_ = MotionState::RELAXED;
}

void GaitEngine::emergencyStop() {
  if (controller_) controller_->relax();
  forward_ = lateral_ = turn_ = 0.0f;
  state_ = MotionState::EMERGENCY_STOP;
}

void GaitEngine::clearEmergency() {
  if (state_ == MotionState::EMERGENCY_STOP) state_ = MotionState::RELAXED;
}

bool GaitEngine::setVelocity(float forward, float lateral, float turn) {
  if (!controller_ || emergencyLatched()) return false;
  forward = clampNormalized(forward);
  lateral = clampNormalized(lateral);
  turn = clampNormalized(turn);
  if (fabsf(forward) < 0.01f && fabsf(lateral) < 0.01f && fabsf(turn) < 0.01f) {
    return stop();
  }
  if (state_ != MotionState::WALKING) {
    if (state_ != MotionState::STANDING && !stand()) return false;
    phaseStartMs_ = millis();
    lastUpdateMs_ = 0;
  }
  forward_ = forward;
  lateral_ = lateral;
  turn_ = turn;
  state_ = MotionState::WALKING;
  return true;
}

bool GaitEngine::update(uint32_t nowMs) {
  if (!controller_ || state_ != MotionState::WALKING) return true;
  if (nowMs - lastUpdateMs_ < config::CONTROL_INTERVAL_MS) return true;
  lastUpdateMs_ = nowMs;

  const uint32_t periodMs = static_cast<uint32_t>(config::DEFAULT_GAIT_PERIOD_MS) * 50UL /
                            static_cast<uint32_t>(speedPercent_);
  const float globalPhase = fmodf(static_cast<float>(nowMs - phaseStartMs_) /
                                  static_cast<float>(periodMs), 1.0f);
  const float amplitude = fmaxf(fabsf(forward_), fmaxf(fabsf(lateral_), fabsf(turn_)));
  Vec3 targets[config::LEG_COUNT];

  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    const bool tripodA = leg == 0 || leg == 2 || leg == 4;
    float phase = globalPhase + (tripodA ? 0.0f : 0.5f);
    if (phase >= 1.0f) phase -= 1.0f;

    float travel;
    float lift = 0.0f;
    if (phase < 0.5f) {
      const float u = phase * 2.0f;
      travel = -0.5f + smoothStep(u);
      lift = stepHeightMm_ * sinf(PI * u) * amplitude;
    } else {
      const float u = (phase - 0.5f) * 2.0f;
      travel = 0.5f - u;
    }

    const float mount = config::LEG_MOUNT_ANGLE_DEG[leg] * PI / 180.0f;
    float bodyX = travel * strideLengthMm_ * forward_;
    float bodyY = travel * strideLengthMm_ * lateral_;
    bodyX += -sinf(mount) * travel * config::TURN_STRIDE_MM * turn_;
    bodyY +=  cosf(mount) * travel * config::TURN_STRIDE_MM * turn_;

    targets[leg].x = config::DEFAULT_REACH_MM + cosf(mount) * bodyX + sinf(mount) * bodyY;
    targets[leg].y = -sinf(mount) * bodyX + cosf(mount) * bodyY;
    targets[leg].z = -bodyHeightMm_ + lift;
  }

  if (!controller_->applyFootTargets(targets)) {
    forward_ = lateral_ = turn_ = 0.0f;
    state_ = MotionState::STANDING;
    lastIKFault_ = true;
    return false;
  }
  lastIKFault_ = false;
  return true;
}

void GaitEngine::setSpeedPercent(uint8_t percent) {
  if (percent < 10) percent = 10;
  if (percent > 100) percent = 100;
  speedPercent_ = percent;
}

void GaitEngine::setStrideLength(float millimetres) {
  if (millimetres < config::MIN_STRIDE_MM) millimetres = config::MIN_STRIDE_MM;
  if (millimetres > config::MAX_STRIDE_MM) millimetres = config::MAX_STRIDE_MM;
  strideLengthMm_ = millimetres;
}

void GaitEngine::setStepHeight(float millimetres) {
  if (millimetres < config::MIN_STEP_HEIGHT_MM) millimetres = config::MIN_STEP_HEIGHT_MM;
  if (millimetres > config::MAX_STEP_HEIGHT_MM) millimetres = config::MAX_STEP_HEIGHT_MM;
  stepHeightMm_ = millimetres;
}

bool GaitEngine::setBodyHeight(float millimetres) {
  if (millimetres < config::MIN_BODY_HEIGHT_MM || millimetres > config::MAX_BODY_HEIGHT_MM) {
    return false;
  }
  bodyHeightMm_ = millimetres;
  if (state_ == MotionState::STANDING) return stand();
  return true;
}

const char *GaitEngine::stateName() const {
  switch (state_) {
    case MotionState::RELAXED: return "RELAXED";
    case MotionState::STANDING: return "STANDING";
    case MotionState::WALKING: return "WALKING";
    case MotionState::SITTING: return "SITTING";
    case MotionState::EMERGENCY_STOP: return "EMERGENCY_STOP";
    default: return "UNKNOWN";
  }
}

void GaitEngine::buildNeutralTargets(float bodyHeight, Vec3 targets[config::LEG_COUNT]) const {
  for (uint8_t leg = 0; leg < config::LEG_COUNT; ++leg) {
    targets[leg] = {config::DEFAULT_REACH_MM, 0.0f, -bodyHeight};
  }
}

float GaitEngine::clampNormalized(float value) {
  if (value < -1.0f) return -1.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float GaitEngine::smoothStep(float value) {
  return value * value * (3.0f - 2.0f * value);
}

