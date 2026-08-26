#pragma once

#include <cstring>

#include "Arduino.h"

class EEPROMClass {
 public:
  EEPROMClass() { std::memset(data_, 0xFF, sizeof(data_)); }
  int length() const { return static_cast<int>(sizeof(data_)); }

  template <typename T> T &get(int address, T &value) {
    std::memcpy(&value, data_ + address, sizeof(T));
    return value;
  }

  template <typename T> const T &put(int address, const T &value) {
    std::memcpy(data_ + address, &value, sizeof(T));
    return value;
  }

 private:
  uint8_t data_[4096];
};

extern EEPROMClass EEPROM;

