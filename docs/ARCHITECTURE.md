# Firmware Architecture

The main loop is cooperative and does not allocate dynamic strings. Serial parsing uses a fixed 96-byte buffer, and gait updates run every 20 ms.

## Data flow

1. `CommandProcessor` converts serial input into pose, velocity, calibration, or safety commands.
2. `GaitEngine` creates six local-frame foot targets using a two-tripod phase generator.
3. `Kinematics` solves coxa yaw, femur pitch, and tibia bend for each target.
4. `HexapodController` applies per-servo direction, neutral, trim, and pulse limits.
5. `PCA9685Driver` writes 50 Hz pulse widths to the correct driver and channel.
6. `ConfigStore` validates and persists 18 trim values with a versioned checksum.

All six leg solutions are checked before a gait frame is written. If any target is unreachable, the new frame is rejected, walking is stopped, and the affected legs are reported as a bit mask.

## Tripod gait

- Tripod A: legs 1, 3, 5
- Tripod B: legs 2, 4, 6

Each group spends half the cycle in swing and half in stance. A cubic smooth-step curve moves the swing foot from rear to front, while a sine curve raises the foot. The stance foot moves from front to rear. Forward, lateral, and yaw contributions are combined in the body frame and rotated into each leg’s local frame before IK.

## Extension points

- Add IMU stabilization by adjusting body-frame foot targets before local-frame conversion.
- Add RC, Bluetooth, or ROS transport by calling `GaitEngine::setVelocity` and repeating updates before the watchdog expires.
- Add current or battery monitoring in the main loop and call `emergencyStop()` when thresholds are exceeded.
- Add body roll, pitch, and yaw with a body-transform layer before `Kinematics::solveLeg`.

