### INA3221 Power Monitor Library
- Wraps the three-channel INA3221 as a `SensorInterface` and `I2CDevice`.
- Requires an initialized I2C bus and positive shunt resistance for each channel (default: 0.05 ohms).
- `init()` checks device identification and enables continuous measurements.
- `read()` collects bus voltage, shunt voltage, current, power, status flags, and a timestamp.
- `getData()` copies the last complete sample; failed reads preserve that sample and may be retried.

### Functionality
- [x] Shared interface and unit-aware data integration
- [x] Three-channel measurement and serial test harness
- [x] Device identification and shunt resistance checks
- [ ] Alert threshold configuration
- [ ] Hardware regression testing after maintenance changes

> Note: The I2C bus must outlive the driver. Check `getState()`, `getErr()`, and `getRXErr()` when an operation fails.
