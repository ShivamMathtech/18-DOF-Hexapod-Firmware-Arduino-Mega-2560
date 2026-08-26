#include "ServoConfig.h"

// Channel map matches the wiring diagram:
// Driver A CH0-8 = legs 1-3; Driver B CH0-8 = legs 4-6.
// Default mirrored directions are a starting point only. Verify every joint
// while the robot is supported and change signs to match your assembly.
const ServoCalibration SERVO_CALIBRATIONS[config::SERVO_COUNT] = {
  // driver, ch, dir, center, min, max, neutral, trim, us/degree
  {0, 0, +1, 1500, 600, 2400, 0.0f,  0.0f, 10.0f},  // L1 coxa
  {0, 1, +1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L1 femur
  {0, 2, -1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L1 tibia
  {0, 3, +1, 1500, 600, 2400, 0.0f,  0.0f, 10.0f},  // L2 coxa
  {0, 4, +1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L2 femur
  {0, 5, -1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L2 tibia
  {0, 6, +1, 1500, 600, 2400, 0.0f,  0.0f, 10.0f},  // L3 coxa
  {0, 7, +1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L3 femur
  {0, 8, -1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L3 tibia

  {1, 0, -1, 1500, 600, 2400, 0.0f,  0.0f, 10.0f},  // L4 coxa
  {1, 1, -1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L4 femur
  {1, 2, +1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L4 tibia
  {1, 3, -1, 1500, 600, 2400, 0.0f,  0.0f, 10.0f},  // L5 coxa
  {1, 4, -1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L5 femur
  {1, 5, +1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L5 tibia
  {1, 6, -1, 1500, 600, 2400, 0.0f,  0.0f, 10.0f},  // L6 coxa
  {1, 7, -1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f},  // L6 femur
  {1, 8, +1, 1500, 600, 2400, 90.0f, 0.0f, 10.0f}   // L6 tibia
};

uint8_t servoIndex(uint8_t leg, Joint joint) {
  return static_cast<uint8_t>(leg * config::JOINTS_PER_LEG + static_cast<uint8_t>(joint));
}

const char *jointName(Joint joint) {
  switch (joint) {
    case Joint::COXA: return "COXA";
    case Joint::FEMUR: return "FEMUR";
    default: return "TIBIA";
  }
}

const char *legName(uint8_t leg) {
  static const char *const names[config::LEG_COUNT] = {
    "LEG 1", "LEG 2", "LEG 3", "LEG 4", "LEG 5", "LEG 6"
  };
  return leg < config::LEG_COUNT ? names[leg] : "INVALID LEG";
}

