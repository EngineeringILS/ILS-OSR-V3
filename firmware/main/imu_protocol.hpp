#pragma once

#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <cinttypes>
#include <cstdio>
#include <limits>
#include <string>

namespace ImuProtocol {
enum class Action { Start, Stop, Read, Init, Invalid };
struct Command {
    uint32_t id = 0;
    Action action = Action::Invalid;
};

inline Command parse_command(const std::string& line) {
    if (line.compare(0, 4, "CMD,") != 0) return {};
    const auto separator = line.find(',', 4);
    if (separator == std::string::npos || separator == 4 ||
        line.find(',', separator + 1) != std::string::npos) return {};
    uint32_t id = 0;
    for (size_t i = 4; i < separator; ++i) {
        const char c = line[i];
        if (c < '0' || c > '9') return {};
        const uint32_t digit = c - '0';
        if (id > (std::numeric_limits<uint32_t>::max() - digit) / 10) return {};
        id = id * 10 + digit;
    }
    if (id == 0) return {};
    const auto action = line.substr(separator + 1);
    if (action == "START") return {id, Action::Start};
    if (action == "STOP") return {id, Action::Stop};
    if (action == "READ") return {id, Action::Read};
    if (action == "INIT") return {id, Action::Init};
    return {id, Action::Invalid};
}

inline std::string ack(uint32_t id, const std::string& action) {
    return "ACK," + std::to_string(id) + "," + action + "\n";
}
inline std::string error(uint32_t id, const std::string& code) {
    return "ERR," + std::to_string(id) + "," + code + "\n";
}

inline std::string format_sample(const Lunabotics::Common::DataTypes::LSM9DS1Data& data,
                                 uint32_t id, uint64_t sequence, int64_t timestamp_us) {
    namespace Units = Lunabotics::Common::Units;
    const auto acceleration = Units::meters / Units::squared(Units::seconds);
    const auto angular_rate = Units::radians / Units::seconds;
    // Ten %.9g measurements and three integer fields fit well below 512 bytes.
    char line[512];
    const int size = std::snprintf(line, sizeof(line),
        "IMU,1,%" PRIu32 ",%" PRIu64 ",%" PRId64
        ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        id, sequence, timestamp_us,
        data.acceleration.a_x.in(acceleration),
        data.acceleration.a_y.in(acceleration),
        data.acceleration.a_z.in(acceleration),
        data.angular_velocity.x.in(angular_rate),
        data.angular_velocity.y.in(angular_rate),
        data.angular_velocity.z.in(angular_rate),
        data.magnetic_field.x.in(Units::tesla) * 1e6,
        data.magnetic_field.y.in(Units::tesla) * 1e6,
        data.magnetic_field.z.in(Units::tesla) * 1e6,
        data.temperature.in(Units::kelvins) - 273.15);
    if (size < 0 || static_cast<size_t>(size) >= sizeof(line)) return error(id, "FORMAT_ERROR");
    return std::string(line, static_cast<size_t>(size));
}
} // namespace ImuProtocol
