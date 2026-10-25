# Antenna Positioner

MSc project: stepper-motor-driven antenna positioner.

## Hardware

- ESP32 Dev Module (CP2102 USB-UART)
- CNC Shield V3
- DRV8825 stepper driver (X axis), full step (no microstep jumpers)
- NEMA 17 stepper motor
- 12–24 V motor supply on the shield's blue terminal

## Wiring (ESP32 → CNC Shield V3)

| ESP32   | CNC Shield | Function                     |
|---------|------------|------------------------------|
| 3V3     | 5V         | Logic supply for the shield  |
| GND     | GND        | Common ground                |
| GPIO 26 | D2         | X.STEP                       |
| GPIO 27 | D5         | X.DIR                        |
| GPIO 25 | D8         | EN (LOW = driver enabled)    |

Notes:
- The shield's 5V pin must be powered, or the driver stays disabled and Vref reads 0 V.
  Use the ESP32's **3V3**, not 5V, because the shield's pull-ups connect to ESP32 GPIOs (3.3 V max).
- DRV8825 current: I ≈ 2 × Vref. Measure Vref between the potentiometer and GND.

## Software

PlatformIO, Arduino framework (`esp32dev`). Serial monitor at 115200 baud.

Speed is set by `STEP_DELAY_US` in `src/main.cpp`: RPM ≈ 150000 / STEP_DELAY_US (full step).
