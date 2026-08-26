#pragma once

#include "Arduino.h"

class TwoWire {
 public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(int, int quantity) { return static_cast<uint8_t>(quantity); }
  int available() { return 1; }
  int read() { return 0; }
};

extern TwoWire Wire;

