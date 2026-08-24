# DC Motor Speed Control

A closed-loop DC motor speed control system built on **Arduino**, using **quadrature encoder feedback** and a **PID controller** to track a target RPM, with a **C# HMI** for real-time setpoint/tuning input and telemetry, and a **Proteus** project for circuit simulation.

---

## 1. System Architecture & Block Diagram

```
┌─────────────────┐        Serial (9600 baud)        ┌──────────────────────┐
│    C# HMI       │  ◄───────────────────────────►   │   Arduino Firmware   │
│ (Setpoint + PID)│     "target;Kp;Ki;Kd\n"          │   PID + Encoder ISR  │
└─────────────────┘        "setpoint,PV\n"           └──────────────────────┘
                                                                 │
                                                       ┌─────────┴─────────┐
                                                       │   L298 Driver +   │
                                                       │  DC Motor+Encoder │
                                                       │  + 24V Power Rail │
                                                       └───────────────────┘
```

### Closed-Loop Control Block Diagram
<!-- Image 1: System Block Diagram -->
<p align="center">
  <img width="2930" height="1210" alt="image" src="https://github.com/user-attachments/assets/186d56f9-7fa2-497f-a96f-1c48ce154873" />
  <br>
  <em>Figure 1: Closed-loop control block diagram (Setpoint, Arduino PID Controller, L298 Actuator, DC Motor Plant, and Quadrature Encoder Feedback).</em>
</p>

---

## 2. Features

- **Real-time Closed-Loop Speed Control:** Precise RPM tracking using a 360 PPR incremental encoder.
- **Enhanced PID Controller:**
  - *Derivative-on-Measurement:* Prevents sudden derivative kicks upon setpoint steps.
  - *Clamping Anti-Windup:* Prevents integral saturation during large transitions or high loads.
- **Soft-Start Setpoint Ramping:** Trajectory generation limits instantaneous acceleration (100 RPM/s).
- **Low-Pass Filtering (LPF):** Digital filtering on the raw feedback signal reduces encoder quantization noise.
- **Bidirectional Control:** Forward and reverse motor drive using the L298 H-Bridge driver.
- **Dynamic Serial Communication:** Update Setpoint, $K_p$, $K_i$, and $K_d$ parameters on-the-fly without firmware re-flashing.
- **C# Telemetry & HMI Application:** Real-time parameter tuning and dynamic response plotting.
- **Proteus Circuit Simulation:** Full circuit schematic validation before hardware deployment.

---

## 3. Hardware & Circuit Schematic

| Component | Specification |
|-----------|---------------|
| **MCU** | Arduino Uno (ATmega328P) |
| **Actuator (Driver)** | L298N Dual H-Bridge Motor Driver |
| **Plant (Motor)** | 24V DC Geared Motor |
| **Sensor (Feedback)** | Incremental Quadrature Optical/Magnetic Encoder (360 pulses/rev) |
| **Power Supply** | 24V DC (Motor Power) & 5V DC (Logic Power) |

### Pin Configuration

| Signal Name | Arduino Pin | Description |
|:---|:---:|:---|
| **Encoder Channel A** | `D2` | External Hardware Interrupt (`INT0`, RISING edge) |
| **Encoder Channel B** | `D4` | Direction phase detection |
| **Driver IN1** | `A1` | Direction logic pin 1 |
| **Driver IN2** | `A0` | Direction logic pin 2 |
| **Driver ENA** | `D3` | Motor speed control via Timer PWM output |
| **Serial RX / TX** | `D0` / `D1` | Serial UART communication with PC / COMPIM |

### Proteus Circuit Simulation
<!-- Image 2: Proteus Hardware Schematic -->
<p align="center">
  <img width="1960" height="1378" alt="image" src="https://github.com/user-attachments/assets/9c40dc9e-7a35-41e0-80a8-f24f9bfcfc92" />
  <br>
  <em>Figure 2: Complete hardware schematic simulated in Proteus (Arduino UNO, COMPIM module, L298 Driver, and DC Motor with Encoder).</em>
</p>

---

## 4. Firmware & Control Loop

### Uploading Firmware
1. Open [`codetest.ino`](codetest.ino) in the Arduino IDE.
2. Select **Arduino Uno** and the corresponding COM port.
3. Compile and upload to the microcontroller board.

### Serial Protocol (9600 Baud)

* **PC (C# HMI) $\rightarrow$ Arduino** (Setpoint & PID parameters, newline-terminated):
  ```text
  targetSetpoint;Kp;Ki;Kd\n
  ```
* **Arduino $\rightarrow$ PC (C# HMI)** (Telemetry data sent every 20 ms cycle):
  ```text
  currentSetpoint,PV\n
  ```

### Control Loop Structure

```
setup()
  ├── Configure GPIOs (IN1, IN2, ENA, Encoder pins)
  ├── Initialize L298 in STOP mode
  └── attachInterrupt(ENCODER_PIN_A, countPulse, RISING)

loop()  — Executes every T = 20 ms:
  ├── 1. Read & parse any incoming Serial commands (targetSetpoint, Kp, Ki, Kd)
  ├── 2. Trajectory generator: ramp currentSetpoint towards targetSetpoint
  ├── 3. Safely read & reset encoder pulses (atomic access)
  ├── 4. Calculate raw RPM and apply Low-Pass Filter (PV)
  ├── 5. Compute PID output with Anti-Windup and Derivative-on-Measurement
  ├── 6. Set L298 bridge direction (IN1, IN2) and PWM level (ENA)
  └── 7. Transmit currentSetpoint and PV over Serial for real-time plotting
```

---

## 5. Experimental Results & Telemetry (C# HMI)

Real-time speed response curves and telemetry captured directly from the C# HMI application:

<!-- Image 3: C# Response 1 -->
<p align="center">
  <img width="2422" height="1481" alt="image" src="https://github.com/user-attachments/assets/84203583-1539-40d7-a286-da5b9b622d52" />
  <br>
  <em>Figure 3: Real-time motor speed response captured from the C# HMI application (Sample 1).</em>
</p>

<!-- Image 4: C# Response 2 -->
<p align="center">
  <img width="2422" height="1481" alt="image" src="https://github.com/user-attachments/assets/50c6e5d6-cc88-4024-b89d-0cb01cfd661e" />
  <br>
  <em>Figure 4: Real-time motor speed response captured from the C# HMI application (Sample 2).</em>
</p>

<!-- Image 5: C# Response 3 -->
<p align="center">
  <img width="2436" height="1480" alt="image" src="https://github.com/user-attachments/assets/16bde618-c9ac-4e23-8925-8978a517ab44" />
  <br>
  <em>Figure 5: Real-time motor speed response captured from the C# HMI application (Sample 3).</em>
</p>

---

## 6. Project Structure

```text
├── codetest.ino         # Arduino PID firmware source code
├── dktocdocdc.pdsprj    # Proteus circuit simulation design file
├── README.md            # Project documentation and specifications
├── hmi/                 # C# Windows Forms HMI application project
└── images/              # Images for documentation
    ├── system_block_diagram.png
    ├── proteus_schematic.png
    ├── hmi_response_1.png
    ├── hmi_response_2.png
    └── hmi_response_3.png
```

---

## 7. Known Limitations

- **Encoder Resolution:** At very low RPM (< 50 RPM), the 360 PPR resolution may introduce slight speed calculation quantization error.
- **Telemetry Coupling:** Telemetry transmission frequency is tied to the 20 ms control loop execution period.
- **Bumpless Parameter Transfer:** In-flight gain changes take effect immediately without integral state re-initialization.

---

## 8. License & Contribution

Contributions, issues, and feature requests are welcome! Feel free to submit a pull request or file an issue.
