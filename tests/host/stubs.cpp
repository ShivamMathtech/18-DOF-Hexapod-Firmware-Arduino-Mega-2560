#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

HardwareSerial Serial;
TwoWire Wire;
EEPROMClass EEPROM;

namespace {
uint32_t fakeMillis = 0;
}

uint32_t millis() { return fakeMillis; }
void delay(uint32_t milliseconds) { fakeMillis += milliseconds; }
void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return HIGH; }

