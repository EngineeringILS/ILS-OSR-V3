# Project Overview

### Mission Statement
Provide a unfiied, maintainable, and extensible Comamnd & Data Handling (CDH) software stack for the TAMU Lunabotics robot, 
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
- TAMU Lunabotics team members (present and future)
- Robotics Researchers and team developers seeking a reference CDH Stack
- Industry partners and sponsors aiming to implement reliable & autonomous robotics
- Open-source robotics community (via planned public releases)

### Scope
The repository includes:
- ROS2 packages for the Onboard Computer (OBC)
- ESP-IDF Firmware for the ESP32 microcontroller
- Shared libraries, documentation, and development tools

# Getting Started
### Prerequisites
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
TODO













# Visibility & Open-Source Policy

This repository is **private** and intended for internal use by
TAMU SEDS Lunabotics.

We plan maintaining a separate **public repository** that contains periodic,
cleaned snapshots of this codebase for open-source release.

- Internal repo (this one): experimental branches, competition configs,
  and non-public tooling.
- Public repo: stable snapshots intended for external users, released
  under an open-source license.

Before pushing anything to the (planned) public repo, verify that:
- No credentials or secrets are present.
- No internal-only documents or data are included.
- The code builds and runs in a reasonable default configuration.


## Project Contributors

This project is actively developed and maintained by the TAMU SEDS Lunabotics CDH Subteam:

### Core Team
* **Rylee Hunt** - [@Rhunt24](https://github.com/Rhunt24) - Chief Engineer
* **Aden Mann** - [@Aden-M](https://github.com/Aden-M) - Subteam Lead
* **Alex Stevens** - [@alexnstevens06](https://github.com/alexnstevens) - Subteam Member - Embedded Systems C++ / ESP-IDF
* **Enrique Venegas** - [@ev-17](https://github.com/ev-17) - Subteam Member -  ROS2 Teleoperation Control Systems
* **Miguel Vargas** - [@miguellvargas](https://github.com/miguellvargas) - Subteam Member - ROS2 
* **Noah Stovall** - [][] - Subteam Member - ROS2 Motor Control 
* **Timothy Guan** - [][] - Subteam Shadow - Python User Interface