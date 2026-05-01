# Adaptive Extended Kalman Filter (AEKF) for Spacecraft Attitude Determination 🚀

Welcome to the **AEKF-Spacecraft-Control** repository! 

This project implements an Adaptive Extended Kalman Filter (AEKF) designed specifically for robust, real-time spacecraft attitude determination. The mathematics, control theory, and embedded systems logic contained here are structured for hardware-in-the-loop (HIL) simulations and deployment on embedded microcontrollers like the STM32.

---

## 🎯 Project Objectives
* **Optimal State Estimation:** Utilize AEKF to process noisy sensor data (e.g., gyroscopes, star trackers, sun sensors) to estimate the spacecraft's true orientation (quaternions).
* **Adaptability:** Implement adaptive covariance matrices to dynamically handle varying sensor noise and external disturbances in a space environment.
* **Embedded Optimization:** Write highly efficient C/C++ matrix and quaternion math libraries capable of running within the strict real-time constraints of an RTOS-based STM32 microcontroller.
* **Simulation & Validation:** Validate the control laws and filter accuracy using rigorous MATLAB simulations before hardware deployment.

## 📁 Repository Structure
To maintain a professional standard, the code is structured as follows:

* `/src` - Core flight software (C/C++), including quaternion math and AEKF updates.
* `/simulations` - MATLAB/Python scripts for system modeling and performance evaluation.
* `/docs` - Mathematical derivations, state space models, and hardware schematics.

## 🛠️ Technology Stack
* **Language:** C, C++, MATLAB
* **Hardware:** STM32 Microcontrollers
* **Concepts:** Extended Kalman Filters, Quaternions, Linear Algebra, Real-Time Control Systems, Sensor Data Fusion.

---

*This repository is a central part of my professional progression into Space Engineering, focusing on advanced control systems and power electronics.*
