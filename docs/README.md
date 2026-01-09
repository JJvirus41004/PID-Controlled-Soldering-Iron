# Project Documentation

This directory contains detailed documentation related to the development,
testing, and validation of the PID-controlled soldering iron project.
The documents here go beyond final results and focus on the reasoning,
experiments, and learning that occurred while working with real hardware.

📘 **Full Technical Report**  
👉 [Open full technical report](./PID_Soldering_Iron_Report.pdf)

## 📘 PID_Soldering_Iron_Report.pdf

This report documents the complete technical journey of building a
PID-controlled soldering iron using an ESP32 and an NTC thermistor.

Rather than presenting only final results, the report captures:
- Theoretical understanding of PID control (P, I, and D terms)
- Practical challenges faced when applying PID to a real thermal system
- Effects of thermal inertia, sensor lag, and ADC noise
- Limitations of relay-based actuation and the need for time-proportional control
- Manual PID tuning attempts and the transition to PID AutoTune
- Observations made while testing and stabilizing the system on hardware

The intent of this document is to reflect how control theory behaves
when exposed to real-world constraints, and how iterative testing and
observation were required to achieve stable temperature regulation.

This report also serves as a personal engineering reference for future
embedded and control-system projects.
