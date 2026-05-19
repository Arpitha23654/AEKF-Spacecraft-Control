# Real-Time Spacecraft Attitude Determination (AEKF)

![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![STM32](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)

A Hardware-in-the-Loop (HITL) aerospace telemetry system that calculates real-time 3D orientation (quaternions) using a bare-metal ARM Cortex-M4 microcontroller and an Adaptive Extended Kalman Filter (AEKF).

## 🚀 Project Overview
This project bridges the gap between complex orbital math and physical hardware. It reads raw physical physics data (acceleration and gyroscopic rates) from an MPU-6050 sensor, processes it through a highly optimized C++ math engine, and streams the optimal quaternions over UART to a Python-based 3D visualizer.

### Key Features
*   **Math Engine:** 4-state Quaternion-based Adaptive Extended Kalman Filter built using the `Eigen` C++ library.
*   **Hardware Integration:** Bare-metal C implementation on an STM32 Nucleo-F446RE, featuring custom I2C drivers for the MPU-6050.
*   **Real-Time Telemetry:** Fast 10Hz serial transmission of state vectors.
*   **3D Visualization:** A dynamic Python UI using `vpython` and `pyserial` to render the satellite's orientation perfectly in sync with physical hardware movements.

## 🛠️ Architecture Pipeline

1. **Hardware Layer (I2C):** MPU-6050 captures raw 16-bit physical rotation and gravity data.
2. **C-to-C++ Bridge:** The raw data is passed from the C-based STM32 HAL drivers into the C++ `AdaptiveEKF` class via an `extern "C"` wrapper.
3. **Filter Layer (AEKF):** 
    *   *Predict:* Uses Gyroscope rates ($rad/s$) to update the state transition matrix.
    *   *Update:* Uses Accelerometer gravity vectors ($g$) to correct state drift.
4. **Telemetry Layer (UART):** The optimal filtered quaternions are transmitted at 115200 baud rate to the host PC.
5. **Visualization Layer (Python):** Parses the serial stream, converts quaternions to Forward/Up vectors, and renders the 3D physics engine.

## 💻 Running the Visualizer

1. Flash the `main.c` firmware to the STM32 Nucleo.
2. Ensure no serial monitors (like PuTTY) are currently open.
3. Install the required Python graphics libraries:
```bash
pip install pyserial vpython
