#include <SerialIO.hpp>
#include <imu_protocol.hpp>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << "Failed at line " << __LINE__ << ": " << #condition << '\n'; \
    return 1; } } while (false)

int main(int argc, char**) {
    using namespace Lunabotics::Common;
    using namespace ImuProtocol;
    DataTypes::LSM9DS1Data data{};
    data.acceleration.a_x = Units::make_quantity<Units::Acceleration>(1.25);
    data.angular_velocity.z = Units::make_quantity<Units::AngularVelocity>(-0.5);
    data.magnetic_field.y = Units::tesla(42e-6);
    data.temperature = Units::kelvins(298.15);
    const auto formatted = format_sample(data, 7, 4294967296ULL, 9007199254740993LL);
    // Allows the Python interoperability test to consume actual C++ output.
    if (argc > 1) { std::cout << formatted; return 0; }
    std::istringstream stream(formatted);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    CHECK(fields.size() == 15);
    CHECK(fields[0] == "IMU" && fields[1] == "1" && fields[2] == "7");
    CHECK(fields[3] == "4294967296" && fields[4] == "9007199254740993");
    CHECK(std::stod(fields[5]) == 1.25 && std::stod(fields[10]) == -0.5);
    CHECK(std::abs(std::stod(fields[12]) - 42) < 1e-8 && std::stod(fields[14]) == 25);
    CHECK(formatted.back() == '\n');
    CHECK(ack(123, "STOP") == "ACK,123,STOP\n");
    CHECK(error(0, "BAD_COMMAND") == "ERR,0,BAD_COMMAND\n");

    CHECK(parse_command("CMD,1,START").action == Action::Start);
    CHECK(parse_command("CMD,2,STOP").action == Action::Stop);
    CHECK(parse_command("CMD,3,READ").action == Action::Read);
    CHECK(parse_command("CMD,4294967295,INIT").id == UINT32_MAX);
    CHECK(parse_command("CMD,4,unknown").id == 4);
    CHECK(parse_command("CMD,4,unknown").action == Action::Invalid);
    for (const auto* invalid : {"", "CMD,0,READ", "CMD,-1,READ", "CMD,+1,READ",
         "CMD,4294967296,READ", "CMD,9999999999999999999999,READ", "CMD,1x,READ",
         "CMD, 1,READ", "CMD,,READ", "CMD,1", "CMD,1,READ,", "CMD,1,READ,STOP"}) {
        const auto command = parse_command(invalid);
        CHECK(command.id == 0 && command.action == Action::Invalid);
    }
    {
        SerialIO serial(1024, 16);
        CHECK(serial.init() == ESP_OK);
        std::string line = "old";
        TestSerial::output.clear();
        TestSerial::input = "CMD,1,ST";
        CHECK(serial.poll_line(line) == LineResult::None && line.empty());
        TestSerial::input = "OP\r\nnext\n\n";
        CHECK(serial.poll_line(line) == LineResult::Line && line == "CMD,1,STOP");
        CHECK(serial.poll_line(line) == LineResult::Line && line == "next");
        CHECK(serial.poll_line(line) == LineResult::Line && line.empty());
        CHECK(TestSerial::output.empty());
        TestSerial::input = std::string(15, 'a') + "\n";
        CHECK(serial.poll_line(line) == LineResult::Line && line.size() == 15);
        TestSerial::input = std::string(100, 'a') + "\nvalid\n";
        CHECK(serial.poll_line(line) == LineResult::None);
        CHECK(TestSerial::input.size() == 107 - 16);
        LineResult result;
        do { result = serial.poll_line(line); } while (result == LineResult::None);
        CHECK(result == LineResult::Overflow && line.empty());
        CHECK(serial.poll_line(line) == LineResult::Line && line == "valid");
        for (const char invalid : {'\0', '\b', '\t', static_cast<char>(0xff)}) {
            TestSerial::input = std::string("bad") + invalid + "STOP\nOK\n";
            CHECK(serial.poll_line(line) == LineResult::Overflow);
            CHECK(serial.poll_line(line) == LineResult::Line && line == "OK");
        }
        TestSerial::input = "partial";
        CHECK(serial.poll_line(line) == LineResult::None);
        serial.deinit();
        CHECK(serial.init() == ESP_OK);
        TestSerial::input = "new\n";
        CHECK(serial.poll_line(line) == LineResult::Line && line == "new");
    }
    CHECK(TestSerial::installations == 0);
}
