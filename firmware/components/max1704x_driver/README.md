### Max1704x Fuel Gauge Library
- This library wraps a standard SensorInterface for managing the lifecycle and data collection methods for the MAX1704x fuel gauge IC.
- This class must be completed in order to comply with Project Design Principles.
- [ ] Code Implemented to Meet Project Design Principles and provide full Battery Management
    - [ ] Properly Initializes the Max1704x Fuel Gauge IC
    - [ ] Properly Provides upstream information and methods
    - [ ] Handles Battery Disconnection Events 
    - [ ] Handles Low Battery Deep Sleep (stretch goal)
    - [ ] Handles Severe Hardware Failures (IC Failure or Disconnection for unknown reason)
Functionality
- [ ] Code Functionally Complete 
    - [ ] Compilation
    - [ ] Unit Tests
    - [ ] Hardware Tests
    - [x] Derivative of the `SensorInterface` 
    - [ ] Compliance with `Units.hpp` and `DataTypes.hpp`
    - [ ] Platform specific I2C options

> Note: This class will be the first properly implemented hardware sensor and is expected to require significantly more development time due to the required constraints.