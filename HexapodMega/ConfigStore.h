#pragma once

#include <Arduino.h>

#include "Config.h"

class ConfigStore {
 public:
  bool load(float trims[config::SERVO_COUNT]);
  bool save(const float trims[config::SERVO_COUNT]);
  void defaults(float trims[config::SERVO_COUNT]) const;

 private:
  struct CalibrationRecord {
    uint32_t magic;
    uint16_t version;
    float trims[config::SERVO_COUNT];
    uint32_t checksum;
  };

  uint32_t checksum(const CalibrationRecord &record) const;
};

