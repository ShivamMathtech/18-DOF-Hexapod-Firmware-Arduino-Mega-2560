#include "ConfigStore.h"

#include <EEPROM.h>
#include <math.h>
#include <stddef.h>

uint32_t ConfigStore::checksum(const CalibrationRecord &record) const {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  const size_t length = offsetof(CalibrationRecord, checksum);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool ConfigStore::load(float trims[config::SERVO_COUNT]) {
  if (EEPROM.length() < static_cast<int>(sizeof(CalibrationRecord))) {
    defaults(trims);
    return false;
  }

  CalibrationRecord record;
  EEPROM.get(0, record);
  bool valid = record.magic == config::CALIBRATION_MAGIC &&
               record.version == config::CALIBRATION_VERSION &&
               record.checksum == checksum(record);

  for (uint8_t i = 0; valid && i < config::SERVO_COUNT; ++i) {
    valid = isfinite(record.trims[i]) && fabsf(record.trims[i]) <= config::MAX_TRIM_DEG;
  }

  if (!valid) {
    defaults(trims);
    return false;
  }

  for (uint8_t i = 0; i < config::SERVO_COUNT; ++i) trims[i] = record.trims[i];
  return true;
}

bool ConfigStore::save(const float trims[config::SERVO_COUNT]) {
  if (EEPROM.length() < static_cast<int>(sizeof(CalibrationRecord))) return false;

  CalibrationRecord record;
  record.magic = config::CALIBRATION_MAGIC;
  record.version = config::CALIBRATION_VERSION;
  for (uint8_t i = 0; i < config::SERVO_COUNT; ++i) {
    if (!isfinite(trims[i]) || fabsf(trims[i]) > config::MAX_TRIM_DEG) return false;
    record.trims[i] = trims[i];
  }
  record.checksum = checksum(record);
  EEPROM.put(0, record);
  return true;
}

void ConfigStore::defaults(float trims[config::SERVO_COUNT]) const {
  for (uint8_t i = 0; i < config::SERVO_COUNT; ++i) trims[i] = 0.0f;
}

