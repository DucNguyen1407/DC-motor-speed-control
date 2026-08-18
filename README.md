# DC Motor Speed Control

A closed-loop DC motor speed control system built on **Arduino**, using **quadrature encoder feedback** and a **PID controller** to track a target RPM, with a **C# HMI** for real-time setpoint/tuning input and telemetry, and a **Proteus** project for circuit simulation.

---

## Overview

```
┌─────────────────┐        Serial (9600 baud)        ┌──────────────────────┐
│    C# HMI       │  ◄───────────────────────────►   │   Arduino Firmware   │
│ (Setpoint + PID)│     "target;Kp;Ki;Kd\n"          │   PID + Encoder ISR  │
└─────────────────┘        "setpoint,PV\n"           └──────────────────────┘
                                                                 │
                                                       ┌─────────┴─────────┐
                                                       │   L298 Driver +   │
                                                       │  DC Motor+Encoder │
                                                       └───────────────────┘
```


---

## Features

- Real-time closed-loop RPM control using a quadrature encoder for speed feedback
- PID control with derivative-on-measurement (no derivative kick) and integral clamping anti-windup
- Soft-start setpoint ramping (trajectory generation) to limit acceleration
- Low-pass filtering of the measured speed signal to reduce encoder noise
- Bidirectional motor drive (forward/reverse) via an L298 H-bridge driver
- Serial communication protocol for live setpoint and PID gain updates from a PC
- C# HMI application for setpoint control, live PID tuning, and telemetry plotting
- Proteus simulation project for circuit validation

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Arduino (ATmega-based board) |
| Motor driver | L298 H-bridge |
| Motor | DC motor with quadrature encoder |
| Feedback | Incremental encoder (360 pulses/revolution) |

### Pin Configuration

| Signal | Pin |
|--------|-----|
| Encoder A (interrupt) | D2 |
| Encoder B | D4 |
| Motor driver IN1 | A1 |
| Motor driver IN2 | A0 |
| Motor driver ENA (PWM) | D3 |

> Pin assignments are defined at the top of the firmware and can be changed freely.

---

## Firmware

### Requirements

- Arduino IDE
- `LiquidCrystal` library (for optional LCD display support)

### Upload

1. Open the `.ino` sketch in the Arduino IDE.
2. Select the correct board and COM port.
3. Compile and upload.

### Serial Protocol

**PC → Arduino** (setpoint + gains, newline-terminated):

```
targetSetpoint;Kp;Ki;Kd
```

**Arduino → PC** (telemetry, sent every control cycle):

```
currentSetpoint,PV
```

Baud rate: **9600**.

### Control Loop Design

```
setup()
  ├── configure motor + encoder pins
  └── attachInterrupt(ENCODER_PIN_A, countPulse, RISING)

loop()  — every 20 ms:
  ├── A. Ramp current setpoint toward target setpoint
  ├── B. Read encoder pulse count → compute raw RPM → low-pass filter
  ├── C. PID computation (P + clamped-anti-windup I + derivative-on-measurement D)
  ├── D. Drive motor via L298 (direction + PWM magnitude)
  └── E. Send telemetry over Serial

countPulse() (ISR, on encoder edge)
  └── increments/decrements pulse count based on encoder phase
```

### PID Tuning

Default gains (overridable live from the HMI):

| Gain | Role |
|------|------|
| `Kp` | Proportional |
| `Ki` | Integral |
| `Kd` | Derivative |

Anti-windup: the integral term only accumulates when the controller output is not saturated at the PWM limits, preventing integral windup during large setpoint changes.

---

## HMI (C# Application)

A Windows desktop application used to:
- Connect to the Arduino over serial
- Send target RPM and PID gains
- Plot real-time setpoint vs. measured speed (PV)

---

## Simulation

`dktocdocdc.pdsprj` — a **Proteus** project containing the schematic (microcontroller, L298 driver, DC motor, and encoder) for simulating and validating the control circuit before deploying to real hardware.

---

## Known Limitations

- Encoder resolution (360 pulses/rev) limits low-speed measurement accuracy
- Serial telemetry rate is tied to the 20 ms control loop period
- PID gains sent over serial take effect immediately with no bumpless-transfer handling

## Contribution
*Contributions are welcome! Please feel free to submit issues or pull requests.*
