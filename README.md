# SewerSafe AI 👷‍♂️🚨

A smart helmet prototype engineered to detect hazardous gas environments in sewer systems and prevent workplace fatalities. 

This repository contains the firmware and hardware architecture for the SewerSafe system, built around an ESP32 microcontroller and a multi-sensor array.

## System Architecture

The hardware leverages an ESP32 to process real-time environmental data. The sensor array is designed to detect critical threshold breaches for Methane, H2S, and CO:
*   **MICS6814:** Used for broad multi-gas detection.
*   **Winsen MP-4:** Calibrated specifically for combustible gases. 
*   **BME680:** Tracks environmental baselines (temperature, humidity, pressure) to correlate with gas density.

## Engineering Challenges & Optimizations

*   **Latency Reduction:** Optimized the firmware logic loop to handle 5+ concurrent test cases, reducing the system's response delay by 13% and achieving a critical detection alert within **6.2 seconds**.
*   **Physical Housing:** The sensor enclosures and helmet modifications were structurally engineered and 3D modelled using CATIA V5 and V6 for optimized manufacturability.

## Current Roadblocks (WIP)

While the logic and detection thresholds are operational, the system is currently facing two major hardware bottlenecks before it can be shipped:
1.  **Sensor Drift:** Maintaining calibration accuracy over extended periods in high-humidity environments.
2.  **Power Consumption:** The continuous analog reads draw significant power, requiring a more optimized sleep-cycle logic for the ESP32 to make the battery life viable for a full shift.
