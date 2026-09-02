# Project Overview

### Mission Statement
Provide a unified, maintainable, and extensible software stack for the ILS OSR V3 Project, 
enabling reliable integration between high-level autonomy, teleopation, and low-level hardware control.

### Real-World Significance
This project addresses key challenges of robotics system integration in competitive and research environments, ensuring
robust communication, standardized interfaces, and streamlined development for current and future teams.

### Key Features 
- **Unified Architecture:** Bridges ROS2-based autonomy and teleoperation with embedded microcontroller firmware.
- **Standardized Interfaces:** Defines consistent communication protocols and APIs for seamless subsystem integration.
- **Centralized Codebase:** Houses all CDH-realted code, documentation, and tools in a single repository.
- **Testing & Simulation Support:** Comprehensive documentation and modular design for easy onboarding and future development.

### Intended Users
- Robotics Researchers and team developers seeking a reference CDH Stack
- Industry partners and sponsors aiming to implement reliable & autonomous robotics
- Open-source robotics community (via planned public releases)

### Scope
The repository includes:
- ROS2 packages for the Onboard Computer (OBC)
- ESP-IDF Firmware for the ESP32 microcontroller
- Shared libraries, documentation, and development tools

# Getting Started
### 1. Read [Onboarding Documentation](./documentation/ONBOARDING.md) And [Contribution Documentation](./documentation/CONTRIBUTING.md)

### 2.  Prerequisites
This project requires two distinct platforms for development:
* For ROS2 development in `/ros2_ws`:  
  -  Ubuntu 22.04.05 LTS (Desktop, Server, WSL), with ROS2 Humble and C++ Build Tools installed
* For ESP-IDF development in `/firmware`:
  - VS-Code with the ESP-IDF extension enabled and working.
  - Documented incompatibilies on Windows 11 Hosts with MSVC.
  - It is reccomended to use `C:\Lunabotics` on Windows for development.

* Alternatively for pure C++ `/common`:
  - Ubuntu 22.04.05 LTS (Desktop, Server, WSL), with C++ Build Tools installed.
  - The `.vscode` for `/common`is installation agnostic and has been tested to work with the VSCode CMake extension on multiple developer workstations.


Generally, it is recommended to use a Windows 11 based system for development, with ESP-IDF installed on the Host Operating System and ROS2 installed on an Ubuntu 22.04.05LTS WSL Virtual Machine.

### Setup Instructions
> **Note:** Setup Instructions are planned *but generally, these will not be created until this project exceeds the subteam level, or on an as-needed basis*.

# Visibility & Open-Source Policy

> This repository is source available, and will have an appropriate open source license applied upon project completion.


## Project Contributors

This project is actively developed and maintained by the Engineering ILS Foundation:

* **Aden Mann** - [@Aden-M](https://github.com/Aden-M) - President

