# Wiring Reference

## I²C logic bus

```text
Mega 20 SDA  ->  Driver A SDA  ->  Driver B SDA
Mega 21 SCL  ->  Driver A SCL  ->  Driver B SCL
Mega 5V      ->  Driver A VCC  ->  Driver B VCC
Mega GND     ->  Driver A GND  ->  Driver B GND
```

The Mega uses pins 20 and 21 for hardware I²C. The firmware configures the bus for 400 kHz. If long wires or noise cause errors, lower `I2C_CLOCK_HZ` in `Config.h` to `100000UL`.

## Board addresses

- Driver A: all address jumpers open, address `0x40`.
- Driver B: close only `A0`, address `0x41`.

After wiring, run `scan`. Both addresses must be reported.

## Servo power bus

```text
Regulated supply +5–6 V -> 30 A fuse -> master switch -> A V+ and B V+
Regulated supply negative -----------------------------> common GND
```

The external supply negative must connect to Arduino GND. Do not connect the external positive rail to Arduino 5 V or to PCA9685 `VCC`.

Add a 4700 µF capacitor across `V+` and GND beside each driver. Use short, heavy power wires and star distribution where practical. Size the supply, fuse, switch, and conductors from measured servo stall current, not only nominal current.

## Servo connector orientation

| Servo wire | PCA9685 row |
|---|---|
| Signal: orange, yellow, or white | PWM signal |
| Positive: red | `V+` |
| Ground: brown or black | GND |

Some clones reverse the header-row order. Follow the silkscreen on the actual board and verify polarity with a meter.

## Optional emergency stop

Connect a normally-open pushbutton between Mega digital pin 2 and GND. The firmware enables the internal pull-up. Pressing the button disables both PCA9685 outputs and latches the emergency state. Release the button, correct the problem, then issue `clear`. Servo output remains relaxed until another pose command.

