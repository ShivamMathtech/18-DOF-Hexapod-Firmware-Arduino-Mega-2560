#pragma once

#include "Config.h"
#include "Types.h"

struct ServoCalibration {
  uint8_t driver;              // 0 = PCA9685 A, 1 = PCA9685 B
  uint8_t channel;
  int8_t direction;            // +1 or -1 for mirrored installation
  uint16_t centerPulseUs;
  uint16_t minPulseUs;
  uint16_t maxPulseUs;
  float neutralLogicalDeg;
  float defaultTrimDeg;
  float microsecondsPerDegree;
};

extern const ServoCalibration SERVO_CALIBRATIONS[config::SERVO_COUNT];

uint8_t servoIndex(uint8_t leg, Joint joint);
const char *jointName(Joint joint);
const char *legName(uint8_t leg);

