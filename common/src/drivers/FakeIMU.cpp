#include <common/drivers/FakeIMU.hpp>
#define _USE_MATH_DEFINES
#include <cmath>

namespace Lunabotics {
namespace Common {
namespace Sensors{

FakeIMU::FakeIMU(HostController host) :
    SensorInterface(InterfaceType::OTHER, host),
    _readCounter(0)
    {}

FakeIMU::~FakeIMU() {
    // No Dynamic Memory Objects Exist in this class, therefore no action is needed in the destructor.
    // No Hardware Driver Initializations Exist in this class, therefore no action is needed in the destructor.
}

// init() implementation:
bool FakeIMU::init() {
    _State = SensorState::CONNECTED;

    // Reset Sensor Data Values:
    _IMUAcceleration = {Units::ZERO, Units::ZERO, Units::ZERO};
    _readCounter = 0;
    _startTime = DataTypes::LocalClock::now();
    // Indicate Success:
    return true;
}

// read() implementation:
bool FakeIMU::read() {
    // This is the core logic for the square wave output.

    // Check if the sensor is in a bad state:
    if (_State == SensorState::FAILED || _State == SensorState::UNINITIALIZED) {
        return false; // Can't read from a failed or uninitialized sensor.
    }

    _readCounter++; // Update the read counter
    auto now = DataTypes::LocalClock::now(); // Update the current time
    std::chrono::duration<float> elapsed = now - _startTime;


    // Use standard SeaLevelGravity Constant from Units:
    const auto amplitude = 1.0f * Units::Gravity::SeaLevel;
    // Not using units for the simple math operations:
    const float period_s = 4.0f; // 4-second period for one full wave.
    const float radians = (elapsed.count() / period_s) * (2.0f * M_PI);

    const float sinWave = sin(radians);
    const float cosWave = cos(radians);

    // Update sensor data with simulated "sensor" data:
    // Units become inherited because they are expressed inside of amplitude, type safety is achieved:
    _IMUAcceleration.a_x = amplitude * sinWave;
    _IMUAcceleration.a_y = amplitude * cosWave;
    _IMUAcceleration.a_z = Units::Gravity::SeaLevel;
    _IMUAcceleration.timestamp = DataTypes::LocalClock::now();

    _State= SensorState::CONNECTED;
    return true;
}

void FakeIMU::getData(DataTypes::LinearAcceleration& data) const {
    data = _IMUAcceleration;
}

} // namespace Sensors
} // namespace Common
} // namespace Lunabotics