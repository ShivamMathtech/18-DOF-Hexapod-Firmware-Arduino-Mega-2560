# Servo Calibration Procedure

Calibrate with the robot supported above the floor and the servo power switch within immediate reach.

## 1. Verify I²C without servo power

Upload the firmware, open the 115200-baud Serial Monitor, and run:

```text
scan
```

Expected addresses are `0x40` and `0x41`.

## 2. Check servo direction and horn position

Turn servo power on and run `neutral`. The logical neutral angles are:

- Coxa: 0°
- Femur: 90°
- Tibia: 90°

If a joint drives toward a hard stop, switch servo power off immediately. Reverse that servo’s `direction` field (`+1` to `-1`, or the opposite) in `ServoConfig.cpp`, upload again, and repeat.

Remove and refit the servo horn as close as possible to mechanical neutral before applying electronic trim.

## 3. Fine trim

Use a small value and adjust one joint at a time:

```text
trim 1 coxa -2.5
trim 1 femur 3.0
trim 1 tibia -1.5
neutral
```

Valid trim is −30° to +30°. If more correction is needed, reposition the horn instead of forcing a large electronic offset.

Run `save` to write all trims to EEPROM. `load` restores the saved record. `defaults` resets RAM trims to zero but does not overwrite EEPROM until `save` is issued.

## 4. Set pulse limits

The starting range is 600–2400 µs with 1500 µs center. These values may exceed the safe travel of some servos. For each entry in `ServoConfig.cpp`:

1. Decrease the range.
2. Test small manual movements with `servo <leg> <joint> <angle>`.
3. Expand only until the required mechanical range is reached.
4. Keep clearance from every hard stop.

## 5. Measure geometry

Measure pivot-center to pivot-center and update `Config.h`:

- `COXA_LENGTH_MM`
- `FEMUR_LENGTH_MM`
- `TIBIA_LENGTH_MM`
- `DEFAULT_REACH_MM`

Incorrect geometry causes poor foot placement and unreachable IK targets.

## 6. Loaded test

Start with the robot supported:

```text
stand
forward 20
stop
sit
relax
```

Reduce stride, step height, and speed if the motion approaches linkage or servo limits.

