# Changelog

## 1.0.0 — 2026-08-26

- Initial Arduino Mega 2560 firmware for an 18-DOF hexapod.
- Self-contained dual-PCA9685 driver at addresses `0x40` and `0x41`.
- Analytical three-joint leg inverse kinematics.
- Non-blocking tripod gait with forward, strafe, and turn commands.
- Per-servo channel, direction, pulse, neutral, and trim configuration.
- Checksummed EEPROM calibration storage.
- Serial command interface, motion watchdog, and optional physical E-stop.
- Wiring, calibration, command, architecture, and host-test documentation.

