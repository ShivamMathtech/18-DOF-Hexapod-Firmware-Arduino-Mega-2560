#pragma once

#include <Arduino.h>

enum class Joint : uint8_t {
  COXA = 0,
  FEMUR = 1,
  TIBIA = 2
};

enum class MotionState : uint8_t {
  RELAXED,
  STANDING,
  WALKING,
  SITTING,
  EMERGENCY_STOP
};

struct Vec3 {
  float x;
  float y;
  float z;
};

struct JointAngles {
  float coxa;
  float femur;
  float tibia;
};

inline float jointValue(const JointAngles &angles, Joint joint) {
  switch (joint) {
    case Joint::COXA: return angles.coxa;
    case Joint::FEMUR: return angles.femur;
    default: return angles.tibia;
  }
}

inline void setJointValue(JointAngles &angles, Joint joint, float value) {
  switch (joint) {
    case Joint::COXA: angles.coxa = value; break;
    case Joint::FEMUR: angles.femur = value; break;
    case Joint::TIBIA: angles.tibia = value; break;
  }
}

