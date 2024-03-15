# Vehicle Electronic Control Unit (ECU) and Battery Management System (BMS) Design Report

This repository contains the design documentation and codebase for the Embedded Systems of ORI23e.
![SunDay-Wellness-Tracker](images/image2.png)

## ECU Overview

The ECU serves as the central processing unit for the vehicle, managing signal processing and control tasks. It handles functions such as pedal control, indicator control, and motor torque control.

### Microcontroller Selection

The NI myRIO 1900 was selected as the main microcontroller for the ECU due to its superior packaging, stability, and FPGA access. It offers ample processing power with a clock frequency of 667MHz.

### Coding Architecture

The ECU is programmed using National Instruments' LabVIEW software, providing a graphical flow-based design for hardware integration and algorithm development. The codebase is structured with modular sub-VIs for different functionalities.

### Functionality Overview

- **APPS Control**: Implements plausibility checks and signal processing for throttle input from acceleration pedal position sensors.
![SunDay-Wellness-Tracker](images/image2.png)
- **Ready to Drive (RTD) Sequence**: Controls the transition of the vehicle into an operable state, sending enable signals to the motor controller.
![SunDay-Wellness-Tracker](images/image2.png)
- **Pump Control**: Manages the cooling system of the motor to prevent overheating during operation.
![SunDay-Wellness-Tracker](images/image2.png)

### ECU Extension PCB Design

An ECU Extension PCB was designed to interface the NI myRIO with other vehicle systems. It includes MOSFET-based switching logic for actuating signals and facilitates connections using berg strips and Amphenol connectors.

### Wireless Telemetry System

Utilizes a NodeMCU ESP8266 for wireless telemetry, allowing real-time monitoring of vehicle data over the internet. The telemetry data is transmitted to the Google Cloud Platform for storage and visualization using InfluxDB.

## BMS Overview

The BMS is responsible for monitoring cell voltages, temperatures, and fault conditions of the battery pack.

### BMS Topology Selection

The Modular topology was chosen for its advantages in wiring harnesses, assembly, space efficiency, and scalability.

### Slave IC Selection

The LTC6813 was selected as the BMS slave monitoring IC due to its ability to monitor up to 18 cells, integrated isoSPI interface, and support for temperature sensor inputs.

### Master IC Selection

The AtMega 2560 MCU from the AVR family was chosen for the BMS Master due to its ease of programming, library support, and stackable architecture.

### Data Flow through the BMS

The BMS Master continuously monitors cell parameters and transmits data to the data logger via the CAN bus. This data facilitates diagnostics and performance analysis.

### BMS Logic Flow Diagram

The BMS Logic Flow Diagram illustrates the sequence of operations performed by the BMS Master, including cell monitoring, fault detection, and data transmission.

### Model and SoC Estimation

A battery model was developed using Simulink to estimate cell performance and assist in State of Charge (SoC) estimation. SoC estimation algorithms, including Kalman Filter and Luenberger Observer, are being evaluated for implementation.

For detailed documentation and codebase, refer to the corresponding directories in this repository.
