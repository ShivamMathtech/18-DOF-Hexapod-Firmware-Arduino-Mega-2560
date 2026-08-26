#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

using byte = uint8_t;

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int INPUT_PULLUP = 2;
constexpr int BIN = 2;
constexpr int HEX = 16;

#ifndef PI
#define PI 3.14159265358979323846
#endif
#define F(value) value

class Stream {
 public:
  virtual ~Stream() = default;
  virtual int available() { return 0; }
  virtual int read() { return -1; }

  template <typename T> size_t print(const T &) { return 0; }
  template <typename T> size_t println(const T &) { return 0; }
  template <typename T> size_t print(const T &, int) { return 0; }
  template <typename T> size_t println(const T &, int) { return 0; }
  size_t println() { return 0; }
};

class HardwareSerial : public Stream {
 public:
  void begin(uint32_t) {}
};

extern HardwareSerial Serial;

uint32_t millis();
void delay(uint32_t milliseconds);
void pinMode(uint8_t, uint8_t);
int digitalRead(uint8_t);

