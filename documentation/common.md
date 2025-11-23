# Project Common (Shared Drivers):
1. The philosophy behind the shared driver system is that current and future CDH/GNC efforts can easily and reliably encapsulate new hardware through driver abstraction to be used for the Lunabotics Robot.  
2. Every driver should provide at a base level, methods to access data or control components, standardized state information, and standardized state methods.   
   This structured approach allows for full compliance with our system design requirements and project TPMS.
3. Additionally, the shared drivers should be theoretically compatible with both ESP32 and Debian/ROS2-based platforms, so long as both systems understand what buses are avaliable:
-  (e.g. I2C, SPI, OBC might not have I2C, etc...)

# Current Shared Drivers:
- **Update this list** with shared drivers as they are introduced. 
-  `SensorInterface.hpp` is the C++ base class which is to be used to wrap all sensor drivers.

# Planned Shared Drivers:
-  `MotorInterface.hpp` is a planned C++ base class which is to be used to wrap all motor drivers.

# Example Shared Drivers:
- `FakeIMU.hpp` and `FakeIMU.cpp` provide a wrapped `SensorInterface` class to provide square wave data to mimic the behavior of an actual implemented IMU, and demonstrates the system requirements for a proper driver.