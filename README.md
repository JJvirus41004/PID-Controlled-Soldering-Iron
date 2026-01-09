# PID Controlled Soldering Iron using ESP32 🔥

A real-world implementation of a **PID-based closed-loop temperature control system** for a soldering iron
using an **ESP32**, **NTC thermistor**, and **relay-based time proportional control**.

This project focuses on applying **control theory in a non-ideal, noisy, real hardware environment**.
This project was built and tuned on real hardware — no simulation-only assumptions.

## 📌 Quick Access

- 🔧 **Firmware (ESP32 Code)**  
  👉 [View main firmware source](./firmware/pid_soldering_iron.ino)

- 📄 **Project Documentation**  
  👉 [View detailed documentation](./docs/PID_Soldering_Iron_Report.pdf)

## 🚀 Project Overview

- **Controller:** ESP32  
- **Sensor:** 10k NTC Thermistor  
- **Actuator:** Optocoupler-based Relay  
- **Control Algorithm:** PID + AutoTune  
- **Target Temperature:** ~100°C  

Unlike simulations, this project deals with **thermal inertia, relay limitations, sensor noise, and real-world delays**.

## 🎯 Why This Project Matters

This is not a “PID library demo”.

It demonstrates:
- Translating **continuous PID output → discrete relay control**
- Handling **thermal lag and overshoot**
- Understanding **why theory breaks in practice**
- Applying **PID AutoTune correctly**
- Designing a **complete feedback loop**

Sensor → Algorithm → Actuator → Real World → Sensor

## ⚙️ Hardware Setup

| Component | Description |
|---------|------------|
| MCU | ESP32 |
| Thermistor | 10k NTC (Voltage Divider) |
| Relay | Optocoupler-based ON/OFF relay |
| ADC Pin | GPIO 34 |
| Relay Control | GPIO 4 |
| AutoTune Button | GPIO 2 |

## 🧠 Control Strategy

### PID Control
- **P:** Reacts to present temperature error
- **I:** Eliminates steady-state offset
- **D:** Predicts and reduces overshoot

### Time Proportional Control
Since a relay is **binary**, PID output is converted into ON/OFF time windows.

Example:
- PID Output = 60%
- Relay ON = 1200 ms
- Relay OFF = 800 ms (within a 2s window)

## 🔄 PID AutoTune

AutoTune intentionally oscillates the system to learn:
- Rise time
- Delay
- System inertia

The tuned values are then applied automatically and fine-tuned manually for stability.

## 🧪 Testing & Validation

The system was tested directly on hardware under repeated heating and cooling cycles.

Validation steps included:
- Observing temperature response during cold start and steady-state operation
- Monitoring overshoot and settling time after PID tuning
- Verifying relay switching behavior within defined time windows
- Comparing expected PID response with actual thermal behavior

Serial logs were used to monitor temperature, PID output, and relay state in real time, helping identify instability, overshoot, and sensor-related issues.

## 📄 Documentation

A detailed project report is included in the `docs/` directory, covering:
- Design decisions and hardware setup
- PID theory and practical implementation
- Tuning attempts, failures, and improvements
- Observations from real-world testing

Maintaining documentation throughout the project made it easier to analyze behavior, revisit tuning decisions, and validate improvements over time.

## 🧪 Results

- Stable temperature regulation near **100°C**
- Reduced overshoot and oscillations after tuning
- Predictable and repeatable closed-loop behavior under thermal lag

## 🧠 Key Learnings

- PID control behaves very differently on real hardware compared to theory or simulations
- Sensor noise, placement, and thermal delay directly affected tuning and stability
- Using a relay required converting PID output into time-based control windows
- Stable behavior came from observing trends over time, not just changing gain values
- Proper documentation helped track tuning attempts, failures, and improvements
- Testing and validating changes on hardware was essential before trusting results

## 📂 Repository Structure

//

## 👤 Author

**Jayant Kumar**  
Embedded Systems | Firmware | Control Systems  

## ⭐ If you like this project

Star ⭐ the repository — it helps others discover it!
