#pragma once

#include <Arduino.h>

namespace config {

constexpr uint8_t LEG_COUNT = 6;
constexpr uint8_t JOINTS_PER_LEG = 3;
constexpr uint8_t SERVO_COUNT = LEG_COUNT * JOINTS_PER_LEG;

constexpr uint8_t PCA9685_ADDRESS_A = 0x40;
constexpr uint8_t PCA9685_ADDRESS_B = 0x41;
constexpr float SERVO_FREQUENCY_HZ = 50.0f;
constexpr uint32_t I2C_CLOCK_HZ = 400000UL;

// Optional normally-open emergency-stop button from pin 2 to GND.
constexpr uint8_t EMERGENCY_STOP_PIN = 2;

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint32_t CONTROL_INTERVAL_MS = 20UL;
constexpr uint32_t MOTION_COMMAND_TIMEOUT_MS = 3000UL;

// Robot geometry in millimetres. Measure and replace these values for your frame.
constexpr float COXA_LENGTH_MM = 45.0f;
constexpr float FEMUR_LENGTH_MM = 75.0f;
constexpr float TIBIA_LENGTH_MM = 110.0f;
constexpr float DEFAULT_REACH_MM = 140.0f;
constexpr float DEFAULT_BODY_HEIGHT_MM = 95.0f;
constexpr float MIN_BODY_HEIGHT_MM = 60.0f;
constexpr float MAX_BODY_HEIGHT_MM = 125.0f;

constexpr float DEFAULT_STRIDE_MM = 55.0f;
constexpr float MIN_STRIDE_MM = 20.0f;
constexpr float MAX_STRIDE_MM = 85.0f;
constexpr float DEFAULT_STEP_HEIGHT_MM = 35.0f;
constexpr float MIN_STEP_HEIGHT_MM = 10.0f;
constexpr float MAX_STEP_HEIGHT_MM = 60.0f;
constexpr float TURN_STRIDE_MM = 45.0f;
constexpr uint16_t DEFAULT_GAIT_PERIOD_MS = 1000;

// Body-frame convention: +X forward, +Y left, +Z upward.
// Leg order: front-left, middle-left, rear-left, front-right, middle-right, rear-right.
static const float LEG_MOUNT_ANGLE_DEG[LEG_COUNT] = {
  45.0f, 90.0f, 135.0f, -45.0f, -90.0f, -135.0f
};

constexpr uint32_t CALIBRATION_MAGIC = 0x48455841UL;  // "HEXA"
constexpr uint16_t CALIBRATION_VERSION = 1;
constexpr float MAX_TRIM_DEG = 30.0f;

}  // namespace config

