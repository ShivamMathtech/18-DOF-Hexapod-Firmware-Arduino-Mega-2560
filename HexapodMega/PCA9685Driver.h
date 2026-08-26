#pragma once

#include <Arduino.h>
#include <Wire.h>

class PCA9685Driver {
 public:
  explicit PCA9685Driver(uint8_t address);

  bool begin(TwoWire &wire = Wire);
  bool setPWMFrequency(float frequencyHz);
  bool setPWM(uint8_t channel, uint16_t onTick, uint16_t offTick);
  bool writeMicroseconds(uint8_t channel, uint16_t pulseUs);
  bool setChannelOff(uint8_t channel);
  bool setAllChannelsOff();

  uint8_t address() const { return address_; }
  bool ready() const { return ready_; }

 private:
  bool write8(uint8_t reg, uint8_t value);
  bool read8(uint8_t reg, uint8_t &value);

  uint8_t address_;
  TwoWire *wire_;
  float frequencyHz_;
  bool ready_;
};

