#include <common/drivers/FakeIMU.hpp>
#define _USE_MATH_DEFINES
#include <cmath>

namespace Lunabotics {
namespace Common {
namespace Sensors{

FakeIMU::FakeIMU(HostController host) :
    SensorInterface(InterfaceType::OTHER, host),
    m_read_counter(0)
    {}

FakeIMU::~FakeIMU() {
    // No Dynamic Memory Objects Exist in this class, therefore no action is needed in the destructor.
    // No Hardware Driver Initializations Exist in this class, therefore no action is needed in the destructor.
}

// init() implementation:
bool FakeIMU::init() {
    m_state = SensorState::CONNECTED;

    // Reset Sensor Data Values:
    m_data = {0.0f, 0.0f, 0.0f};
    m_read_counter = 0;
    m_start_time = std::chrono::system_clock::now();
    // Indicate Success:
    return true;
}

// read() implementation:
bool FakeIMU::read() {
    // This is the core logic for the square wave output.

    // Check if the sensor is in a bad state:
    if (m_state == SensorState::FAILED || m_state == SensorState::UNINITIALIZED) {
        return false; // Can't read from a failed or uninitialized sensor.
    }

    m_read_counter++; // Update the read counter
    auto now = std::chrono::system_clock::now(); // Update the current time
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start_time).count();

    const float amplitude = 1.0f; // G Force multiple.
    const float period_ms = 4000.0f; // 4-second period for one full wave.
    const float radians = (elapsed_ms / period_ms) * (2.0f * M_PI);

    const float sinWave = sin(radians);
    const float cosWave = cos(radians);

    // Update sensor data with simulated "sensor" data:
    m_data.ax = amplitude * sinWave;
    m_data.ay = amplitude * cosWave;
    m_data.az = 9.81f;

    m_state = SensorState::CONNECTED;
    return true;
}

void FakeIMU::getData(IMUData& data) const {
    data = m_data;
}

} // namespace Sensors
} // namespace Common
} // namespace Lunabotics