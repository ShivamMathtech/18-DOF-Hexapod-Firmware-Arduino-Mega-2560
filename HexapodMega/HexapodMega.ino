#include <Arduino.h>
#include <Wire.h>

#include "CommandProcessor.h"
#include "Config.h"
#include "ConfigStore.h"
#include "GaitEngine.h"
#include "HexapodController.h"

HexapodController robot;
GaitEngine gait;
ConfigStore calibrationStore;
CommandProcessor commands(robot, gait, calibrationStore);

bool physicalEStopLatched = false;
bool watchdogReported = false;

void setup() {
  pinMode(config::EMERGENCY_STOP_PIN, INPUT_PULLUP);
  Serial.begin(config::SERIAL_BAUD);
  delay(500);

  Serial.println(F("\n18-DOF HEXAPOD / ARDUINO MEGA 2560"));
  Serial.println(F("Firmware starting; servo outputs remain disabled."));

  Wire.begin();
  Wire.setClock(config::I2C_CLOCK_HZ);

  if (!robot.begin()) {
    Serial.println(F("FATAL: PCA9685 initialization failed. Check 0x40/0x41 and I2C wiring."));
    gait.begin(robot);
    gait.emergencyStop();
  } else {
    float trims[config::SERVO_COUNT];
    if (calibrationStore.load(trims)) Serial.println(F("Calibration loaded from EEPROM."));
    else Serial.println(F("No valid EEPROM calibration; using zero trims."));
    robot.setRuntimeTrims(trims);
    gait.begin(robot);
    Serial.println(F("Drivers ready. Support the robot, then type: neutral"));
  }

  commands.begin(Serial);
}

void loop() {
  const uint32_t now = millis();

  if (digitalRead(config::EMERGENCY_STOP_PIN) == LOW) {
    if (!physicalEStopLatched) {
      gait.emergencyStop();
      physicalEStopLatched = true;
      Serial.println(F("PHYSICAL EMERGENCY STOP: outputs disabled"));
    }
  } else {
    physicalEStopLatched = false;
  }

  commands.poll();

  if (gait.isWalking() && config::MOTION_COMMAND_TIMEOUT_MS > 0 &&
      now - commands.lastMotionCommandMs() > config::MOTION_COMMAND_TIMEOUT_MS) {
    gait.stop();
    if (!watchdogReported) {
      Serial.println(F("WATCHDOG: motion command timed out; robot stopped"));
      watchdogReported = true;
    }
  } else if (gait.isWalking()) {
    watchdogReported = false;
  }

  if (!gait.update(now) && gait.lastIKFault()) {
    Serial.print(F("IK FAULT: unreachable leg mask 0b"));
    Serial.println(robot.lastIKErrorMask(), BIN);
  }
}

