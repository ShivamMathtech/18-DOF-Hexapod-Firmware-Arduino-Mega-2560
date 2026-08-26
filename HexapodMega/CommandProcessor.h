#pragma once

#include <Arduino.h>

#include "ConfigStore.h"
#include "GaitEngine.h"
#include "HexapodController.h"

class CommandProcessor {
 public:
  CommandProcessor(HexapodController &controller, GaitEngine &gait, ConfigStore &store);

  void begin(Stream &stream);
  void poll();
  uint32_t lastMotionCommandMs() const { return lastMotionCommandMs_; }

 private:
  void handleLine(char *line);
  void printHelp();
  void printStatus();
  void scanI2C();
  bool parseJoint(const char *token, Joint &joint) const;
  static void lowerCase(char *text);
  static float percentArgument(const char *token, float defaultPercent);

  HexapodController &controller_;
  GaitEngine &gait_;
  ConfigStore &store_;
  Stream *stream_;
  char buffer_[96];
  uint8_t length_;
  uint32_t lastMotionCommandMs_;
};

