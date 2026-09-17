# Project Common (Shared Drivers):
1. The shared driver system encapsulates hardware through portable interfaces for the Lunabotics Robot.
2. Every driver should provide at a base level, methods to access data or control components, standardized state information, and standardized state methods.   
   This structured approach allows for full compliance with our system design requirements and project TPMS.
3. Additionally, the shared drivers should be theoretically compatible with both ESP32 and Debian/ROS2-based platforms, so long as both systems understand what buses are avaliable:
-  (e.g. I2C, SPI, OBC might not have I2C, etc...)

# Current Shared Drivers:
-  `SensorInterface.hpp` is the C++ base class which is to be used to wrap all sensor drivers.
-  `PeripheralInterface.hpp` provides initialization and state reporting for output peripherals.
-  `InterfaceProtocols.hpp` defines portable bus and GPIO configurations.
-  `DataTypes.hpp` and `Units.hpp` provide unit-aware measurements and timestamps.
-  Concrete ESP-IDF drivers belong in `firmware/components`; `common` must remain platform independent.

# Planned Shared Drivers:
-  `MotorInterface.hpp` is a planned C++ base class which is to be used to wrap all motor drivers.

# Example Shared Drivers:
- `FakeIMU.hpp` and `FakeIMU.cpp` provide sinusoidal acceleration through `SensorInterface` for standalone testing.

### Driver Documentation
1. Use Doxygen comments for classes and public methods, including parameters, return values, and hardware constraints.
2. Group constructors, lifecycle methods, device methods, and private members with short section comments.
3. Explain register conversions, failure handling, and resource ownership in implementation comments.
4. Keep TODOs specific and documentation consistent with implemented behavior.
