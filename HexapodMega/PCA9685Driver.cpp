#include "PCA9685Driver.h"

#include <math.h>

namespace {
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t ALL_LED_ON_L = 0xFA;
constexpr uint8_t PRESCALE = 0xFE;
constexpr float OSCILLATOR_HZ = 25000000.0f;
}

PCA9685Driver::PCA9685Driver(uint8_t address)
    : address_(address), wire_(nullptr), frequencyHz_(50.0f), ready_(false) {}

bool PCA9685Driver::begin(TwoWire &wire) {
  wire_ = &wire;
  if (!write8(MODE1, 0x00)) return false;
  if (!write8(MODE2, 0x04)) return false;  // Totem-pole output.
  delay(5);
  ready_ = true;
  return setPWMFrequency(frequencyHz_) && setAllChannelsOff();
}

bool PCA9685Driver::setPWMFrequency(float frequencyHz) {
  if (!wire_ || frequencyHz < 24.0f || frequencyHz > 1526.0f) return false;

  const float prescaleValue = OSCILLATOR_HZ / (4096.0f * frequencyHz) - 1.0f;
  uint8_t prescale = static_cast<uint8_t>(floorf(prescaleValue + 0.5f));
  uint8_t oldMode = 0;
  if (!read8(MODE1, oldMode)) return false;

  const uint8_t sleepMode = static_cast<uint8_t>((oldMode & 0x7F) | 0x10);
  if (!write8(MODE1, sleepMode)) return false;
  if (!write8(PRESCALE, prescale)) return false;
  if (!write8(MODE1, oldMode)) return false;
  delay(5);
  if (!write8(MODE1, static_cast<uint8_t>(oldMode | 0xA1))) return false;

  frequencyHz_ = frequencyHz;
  return true;
}

bool PCA9685Driver::setPWM(uint8_t channel, uint16_t onTick, uint16_t offTick) {
  if (!wire_ || channel > 15 || onTick > 4095 || offTick > 4095) return false;
  const uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(static_cast<uint8_t>(onTick & 0xFF));
  wire_->write(static_cast<uint8_t>((onTick >> 8) & 0x0F));
  wire_->write(static_cast<uint8_t>(offTick & 0xFF));
  wire_->write(static_cast<uint8_t>((offTick >> 8) & 0x0F));
  return wire_->endTransmission() == 0;
}

bool PCA9685Driver::writeMicroseconds(uint8_t channel, uint16_t pulseUs) {
  if (frequencyHz_ <= 0.0f) return false;
  float ticksFloat = pulseUs * frequencyHz_ * 4096.0f / 1000000.0f;
  if (ticksFloat < 1.0f) ticksFloat = 1.0f;
  if (ticksFloat > 4095.0f) ticksFloat = 4095.0f;
  return setPWM(channel, 0, static_cast<uint16_t>(lroundf(ticksFloat)));
}

bool PCA9685Driver::setChannelOff(uint8_t channel) {
  if (!wire_ || channel > 15) return false;
  const uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(static_cast<uint8_t>(0));
  wire_->write(static_cast<uint8_t>(0));
  wire_->write(static_cast<uint8_t>(0));
  wire_->write(static_cast<uint8_t>(0x10));  // Full-off bit.
  return wire_->endTransmission() == 0;
}

bool PCA9685Driver::setAllChannelsOff() {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
  wire_->write(ALL_LED_ON_L);
  wire_->write(static_cast<uint8_t>(0));
  wire_->write(static_cast<uint8_t>(0));
  wire_->write(static_cast<uint8_t>(0));
  wire_->write(static_cast<uint8_t>(0x10));
  return wire_->endTransmission() == 0;
}

bool PCA9685Driver::write8(uint8_t reg, uint8_t value) {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool PCA9685Driver::read8(uint8_t reg, uint8_t &value) {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(static_cast<int>(address_), 1) != 1) return false;
  if (!wire_->available()) return false;
  value = static_cast<uint8_t>(wire_->read());
  return true;
}

