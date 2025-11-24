# CDH ESP-IDF firmware 
### Firmware Architecture
- `/firmware` is the ESP-IDF based firmware for the ESP32 in the CDH System Architecture.
- Special configuration is included in `/common/CMakeLists.txt` to enable proper linking and compilation for embedded projects.
- Platform Specific Drivers are developed by building upon base-classes provided in `/common`

### System Integration:
The ESP-IDF firmware aims to implement the following functionality:
- OBC telemetry and Out-of-Band Management:
    - Controls the OBC telemetry and management pins to provide: Shutdown, Restart, and Start commands to the OBC.
    - Listens to the OBC over JTAG for telemetry and status.
    - Transmits node-specific commands to ROS2 nodes running on the OBC.
- Ground Station Command and Control:
    - Listens to the Ground Station over Wi-Fi for: Shutdown, Restart, and Start commands.
    - Listens to the Ground Station over Wi-Fi for commands to be administered to the OBC ROS2 Nodes.
    - Reports OBC status and system telemetry to the Ground Station.
- Always-On Low Power Mode:
    - Maintains readiness for Robot startup independent of other robot systems using battery power.

### Risk Management:
The ESP-IDF firmware addresses the following Risk Factors:
- OBC Failure: If the OBC fails, this firmware allows for the Ground Station to attempt restarts.
- Autonomy Failure: If autonomy fails, this firmware allows for manual override to teleoperation mode.
- State Storage: If the OBC loses power, the ESP32 maintains a register of the last known state for motor encoders to allow for safe resumption of autonomous control.
- Communication Failure: If both the OBC and ESP32 detect a Wi-Fi failure, both systems stop all operation until connection is restored for safe recovery.

### Software Integration:
The ESP-IDF firmware maintains a registry of all enabled sensors and motors, regardless of whether they are running on the ESP32 or OBC.
- The ESP-IDF firmware contains local extensions of `/common` defined motors and sensors in `/firmware`, and monitors their status on the OBC, while removing the overhead of specifically defined functionality in `/ros2_ws`.
- The general theory is that the ESP32 will be much more reliable than the OBC due to reduced complexity in both hardware, software, and operating system.