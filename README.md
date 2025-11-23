# CDH Software Stack - ROS2 & ESP-IDF

This repository hosts all Command & Data Handling (CDH) software for the TAMU SEDS Lunabotics robot, including:

- **ROS2** packages running on the Onboard Computer (OBC)

- **ESP-IDF** firmware for the ESP32 Onboard Microcontroller handling telemetry and OOB management, complementing the OBC.


## Project Purpose

The goal of this prioject is to provide a **cohesive, well-structured software stack** for the CDH subsystem that:

- **Bridges high-level autonomy and teleoperative control and low-level hardware**  ROS2 nodes on the OBC coordinate with:
    - Ground Station for teleoperative control and telemetry output 
    - MCU motor control
    - OBC Sensor Input

    To expose a clean API for GNC/Autonomy.

- **Standardizes interfaces across the robot** Defines consistent topics, messages, and communication patterns (Wi-Fi/serial/etc.) so every subsystem (GNC, Electrical, Mechanical, Structural) can integrate against a predictable CDH layer.

- **Centralizes all CDH code in one place**   
Keeps ROS2 packages, embedded firmware, and shared libraries in a single repo, with a consistent layout to ensure code reusability for future teams.

- **Supports testing and simulation**  
Enables bench tests and hardware-in-the-loop setups, and simulation environments to verify code before deployment.

- **Improves maintainability and handoff**  
Documents architecture, conventions,a dn workflows so future Lunabotics teams can extend and adapt the CDH stack without reverse-engineering legacy code.  

---

In summary: this repository provides a **single source of truth** for all CDH efforts as they relate to the Robot CDH Subsystem, as well as the other Robot subsystems.

## Visibility & Open-Source Policy

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