#include "CommandProcessor.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <Wire.h>

#include "ServoConfig.h"

CommandProcessor::CommandProcessor(HexapodController &controller, GaitEngine &gait, ConfigStore &store)
    : controller_(controller), gait_(gait), store_(store), stream_(nullptr),
      length_(0), lastMotionCommandMs_(0) {
  buffer_[0] = '\0';
}

void CommandProcessor::begin(Stream &stream) {
  stream_ = &stream;
  printHelp();
}

void CommandProcessor::poll() {
  if (!stream_) return;
  while (stream_->available()) {
    const char incoming = static_cast<char>(stream_->read());
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      buffer_[length_] = '\0';
      if (length_ > 0) handleLine(buffer_);
      length_ = 0;
      buffer_[0] = '\0';
    } else if (length_ < sizeof(buffer_) - 1) {
      buffer_[length_++] = incoming;
    } else {
      length_ = 0;
      stream_->println(F("ERR command too long"));
    }
  }
}

void CommandProcessor::handleLine(char *line) {
  lowerCase(line);
  char *args[6] = {nullptr};
  uint8_t count = 0;
  char *save = nullptr;
  for (char *token = strtok_r(line, " \t", &save); token && count < 6;
       token = strtok_r(nullptr, " \t", &save)) {
    args[count++] = token;
  }
  if (count == 0) return;

  if (!strcmp(args[0], "help")) {
    printHelp();
  } else if (!strcmp(args[0], "status")) {
    printStatus();
  } else if (!strcmp(args[0], "stand")) {
    stream_->println(gait_.stand() ? F("OK standing") : F("ERR unable to stand"));
  } else if (!strcmp(args[0], "sit")) {
    stream_->println(gait_.sit() ? F("OK sitting") : F("ERR unable to sit"));
  } else if (!strcmp(args[0], "stop")) {
    stream_->println(gait_.stop() ? F("OK stopped") : F("ERR stop rejected"));
    lastMotionCommandMs_ = millis();
  } else if (!strcmp(args[0], "relax")) {
    gait_.relax();
    stream_->println(F("OK servo outputs disabled"));
  } else if (!strcmp(args[0], "estop")) {
    gait_.emergencyStop();
    stream_->println(F("EMERGENCY STOP LATCHED"));
  } else if (!strcmp(args[0], "clear")) {
    gait_.clearEmergency();
    stream_->println(F("OK emergency latch cleared; servos remain relaxed"));
  } else if (!strcmp(args[0], "walk") && count >= 4) {
    const float forward = percentArgument(args[1], 0.0f);
    const float lateral = percentArgument(args[2], 0.0f);
    const float turn = percentArgument(args[3], 0.0f);
    if (gait_.setVelocity(forward, lateral, turn)) {
      lastMotionCommandMs_ = millis();
      stream_->println(F("OK velocity accepted"));
    } else stream_->println(F("ERR velocity rejected"));
  } else if (!strcmp(args[0], "forward") || !strcmp(args[0], "back") ||
             !strcmp(args[0], "left") || !strcmp(args[0], "right") ||
             !strcmp(args[0], "turnl") || !strcmp(args[0], "turnr")) {
    const float amount = percentArgument(count >= 2 ? args[1] : nullptr, 40.0f);
    float forward = 0.0f, lateral = 0.0f, turn = 0.0f;
    if (!strcmp(args[0], "forward")) forward = fabsf(amount);
    if (!strcmp(args[0], "back")) forward = -fabsf(amount);
    if (!strcmp(args[0], "left")) lateral = fabsf(amount);
    if (!strcmp(args[0], "right")) lateral = -fabsf(amount);
    if (!strcmp(args[0], "turnl")) turn = fabsf(amount);
    if (!strcmp(args[0], "turnr")) turn = -fabsf(amount);
    if (gait_.setVelocity(forward, lateral, turn)) {
      lastMotionCommandMs_ = millis();
      stream_->println(F("OK motion started"));
    } else stream_->println(F("ERR motion rejected"));
  } else if (!strcmp(args[0], "speed") && count >= 2) {
    int speed = atoi(args[1]);
    if (speed < 10) speed = 10;
    if (speed > 100) speed = 100;
    gait_.setSpeedPercent(static_cast<uint8_t>(speed));
    stream_->println(F("OK speed updated"));
  } else if (!strcmp(args[0], "stride") && count >= 2) {
    gait_.setStrideLength(static_cast<float>(atof(args[1])));
    stream_->println(F("OK stride updated"));
  } else if (!strcmp(args[0], "step") && count >= 2) {
    gait_.setStepHeight(static_cast<float>(atof(args[1])));
    stream_->println(F("OK step height updated"));
  } else if (!strcmp(args[0], "body") && count >= 2) {
    stream_->println(gait_.setBodyHeight(static_cast<float>(atof(args[1]))) ?
                     F("OK body height updated") : F("ERR body height out of range"));
  } else if (!strcmp(args[0], "neutral")) {
    if (gait_.isWalking()) gait_.stop();
    stream_->println(controller_.neutralPose() ? F("OK neutral calibration pose") : F("ERR neutral pose failed"));
  } else if (!strcmp(args[0], "servo") && count >= 4) {
    Joint joint;
    const int leg = atoi(args[1]);
    if (leg < 1 || leg > 6 || !parseJoint(args[2], joint)) {
      stream_->println(F("ERR usage: servo <1-6> <coxa|femur|tibia> <angle>"));
    } else if (gait_.isWalking()) {
      stream_->println(F("ERR stop walking before manual servo control"));
    } else {
      stream_->println(controller_.manualJoint(static_cast<uint8_t>(leg - 1), joint,
                                                static_cast<float>(atof(args[3]))) ?
                       F("OK servo moved") : F("ERR servo command failed"));
    }
  } else if (!strcmp(args[0], "trim") && count >= 4) {
    Joint joint;
    const int leg = atoi(args[1]);
    const float trim = static_cast<float>(atof(args[3]));
    if (leg < 1 || leg > 6 || !parseJoint(args[2], joint) ||
        !controller_.setTrim(static_cast<uint8_t>(leg - 1), joint, trim)) {
      stream_->println(F("ERR usage: trim <1-6> <coxa|femur|tibia> <-30..30>"));
    } else stream_->println(F("OK trim set in RAM; use save to store"));
  } else if (!strcmp(args[0], "save")) {
    float trims[config::SERVO_COUNT];
    controller_.getRuntimeTrims(trims);
    stream_->println(store_.save(trims) ? F("OK calibration saved to EEPROM") : F("ERR EEPROM save failed"));
  } else if (!strcmp(args[0], "load")) {
    float trims[config::SERVO_COUNT];
    const bool loaded = store_.load(trims);
    controller_.setRuntimeTrims(trims);
    stream_->println(loaded ? F("OK calibration loaded") : F("WARN no valid record; defaults loaded"));
  } else if (!strcmp(args[0], "defaults")) {
    controller_.resetRuntimeTrims();
    stream_->println(F("OK trim defaults loaded in RAM; use save to store"));
  } else if (!strcmp(args[0], "scan")) {
    scanI2C();
  } else {
    stream_->println(F("ERR unknown command; type help"));
  }
}

void CommandProcessor::printHelp() {
  if (!stream_) return;
  stream_->println(F("\n18-DOF HEXAPOD COMMANDS"));
  stream_->println(F("help | status | scan"));
  stream_->println(F("stand | sit | stop | relax | estop | clear"));
  stream_->println(F("walk <forward%> <left%> <turn-left%>   values -100..100"));
  stream_->println(F("forward|back|left|right|turnl|turnr [10..100]"));
  stream_->println(F("speed <10..100> | stride <20..85> | step <10..60> | body <60..125>"));
  stream_->println(F("neutral | servo <leg> <joint> <angle>"));
  stream_->println(F("trim <leg> <joint> <-30..30> | save | load | defaults"));
}

void CommandProcessor::printStatus() {
  stream_->print(F("state=")); stream_->println(gait_.stateName());
  stream_->print(F("drivers=")); stream_->println(controller_.driversReady() ? F("READY") : F("FAULT"));
  stream_->print(F("speed%=")); stream_->println(gait_.speedPercent());
  stream_->print(F("stride_mm=")); stream_->println(gait_.strideLength(), 1);
  stream_->print(F("step_mm=")); stream_->println(gait_.stepHeight(), 1);
  stream_->print(F("body_height_mm=")); stream_->println(gait_.bodyHeight(), 1);
  stream_->print(F("ik_error_mask=0b")); stream_->println(controller_.lastIKErrorMask(), BIN);
}

void CommandProcessor::scanI2C() {
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      stream_->print(F("I2C device at 0x"));
      if (address < 16) stream_->print('0');
      stream_->println(address, HEX);
      ++found;
    }
  }
  stream_->print(F("devices=")); stream_->println(found);
}

bool CommandProcessor::parseJoint(const char *token, Joint &joint) const {
  if (!strcmp(token, "coxa") || !strcmp(token, "0")) joint = Joint::COXA;
  else if (!strcmp(token, "femur") || !strcmp(token, "1")) joint = Joint::FEMUR;
  else if (!strcmp(token, "tibia") || !strcmp(token, "2")) joint = Joint::TIBIA;
  else return false;
  return true;
}

void CommandProcessor::lowerCase(char *text) {
  for (; *text; ++text) *text = static_cast<char>(tolower(static_cast<unsigned char>(*text)));
}

float CommandProcessor::percentArgument(const char *token, float defaultPercent) {
  float value = token ? static_cast<float>(atof(token)) : defaultPercent;
  if (value < -100.0f) value = -100.0f;
  if (value > 100.0f) value = 100.0f;
  return value / 100.0f;
}
