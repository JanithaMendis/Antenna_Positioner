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

Firmware commands: `f <cm>`, `b <cm>`, `move <cm>` (absolute, 0–20 cm), `stop`, `pos`, `zero`, `home`, `env`, `status`, `reset`.
`status` replies with one line for the PC app: `STATUS pos=5.00 busy=0 killed=0 t=24.3 rh=56.1`.

## PC app (`pc_app/`)

NanoVNA + antenna positioner GUI, based on the Version2_SE_System draft and adapted to this firmware.

```
cd pc_app
pip install -r requirements.txt
python se_app.py           # real hardware (close the Serial Monitor first)
python se_app.py --demo    # VNA and antenna simulated
```

- **Antenna tab:** live position / T / RH, one command box for the firmware commands, STOP button.
- **Measure tab:** 1–3 stops (max 20 cm). Each run moves to every stop, sweeps the VNA, logs T/RH, then returns home.
- **Results:** SE per stop plus the average (AVG) over the stops, plots, CSV export. Sessions are saved in `pc_app/sessions/`.

Speed is set by `STEP_DELAY_US` in `src/main.cpp`: RPM ≈ 150000 / STEP_DELAY_US (full step).
