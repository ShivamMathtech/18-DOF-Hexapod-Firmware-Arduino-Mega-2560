# Serial Command Reference

Serial settings: 115200 baud, 8-N-1, newline-terminated ASCII commands.

## Motion

| Syntax | Range | Description |
|---|---:|---|
| `stand` | — | Move to neutral standing pose |
| `sit` | — | Lower to minimum configured body height |
| `relax` | — | Disable all PWM outputs |
| `walk F L T` | −100…100 | Forward, left-strafe, and left-turn percentages |
| `forward N` | 10…100 | Forward motion |
| `back N` | 10…100 | Backward motion |
| `left N` / `right N` | 10…100 | Lateral motion |
| `turnl N` / `turnr N` | 10…100 | Turn in place |
| `stop` | — | Finish walking and return to stand |

Repeat a motion command within the configured timeout to keep walking. This makes loss of a serial controller fail safe.

## Gait tuning

| Syntax | Range | Unit |
|---|---:|---|
| `speed N` | 10…100 | percent |
| `stride N` | 20…85 | mm |
| `step N` | 10…60 | mm |
| `body N` | 60…125 | mm |

## Calibration and diagnostics

| Syntax | Description |
|---|---|
| `scan` | Print responding I²C addresses |
| `status` | Print controller status |
| `neutral` | Send all servos to logical calibration neutral |
| `servo 1 coxa 10` | Manually command one joint |
| `trim 1 coxa -2.5` | Change one servo trim in RAM |
| `save` | Store all trims in checksummed EEPROM record |
| `load` | Reload the EEPROM record |
| `defaults` | Set all RAM trims to zero |
| `estop` | Disable outputs and latch emergency state |
| `clear` | Clear emergency state; outputs remain disabled |

