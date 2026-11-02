#include "lsm9ds1.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <tuple>
#include <vector>

struct FakeBus {
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> regs;
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> writes;
    int reads_before_failure = -1;
    int writes_before_failure = -1;
    uint64_t now = 123456789;
};

static int fake_read(void *opaque, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    auto &bus = *static_cast<FakeBus *>(opaque);
    if (bus.reads_before_failure == 0) return -1;
    if (bus.reads_before_failure > 0) --bus.reads_before_failure;
    const bool magnetic = addr == 0x1c || addr == 0x1e;
    const uint8_t base = magnetic ? static_cast<uint8_t>(reg & 0x7f) : reg;
    const bool increment = magnetic ? (reg & 0x80) != 0
                                   : (bus.regs[{addr, 0x22}] & 0x04) != 0;
    for (size_t i = 0; i < len; ++i) {
        const auto offset = increment ? i : 0u;
        data[i] = bus.regs[{addr, static_cast<uint8_t>(base + offset)}];
    }
    return 0;
}

static int fake_write(void *opaque, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
    auto &bus = *static_cast<FakeBus *>(opaque);
    if (bus.writes_before_failure == 0) return -1;
    if (bus.writes_before_failure > 0) --bus.writes_before_failure;
    for (size_t i = 0; i < len; ++i) {
        const auto r = static_cast<uint8_t>(reg + i);
        bus.regs[{addr, r}] = data[i];
        bus.writes.emplace_back(addr, r, data[i]);
    }
    return 0;
}

static int fake_clock(void *opaque, uint64_t *ns) {
    *ns = static_cast<FakeBus *>(opaque)->now;
    return 0;
}

static void expect(bool condition, const char *message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

static void near(double actual, double expected, double epsilon, const char *message) {
    expect(std::fabs(actual - expected) <= epsilon, message);
}

static lsm9ds1_device make_device(FakeBus &bus, uint8_t xg = 0x6b, uint8_t m = 0x1e) {
    bus.regs[{xg, 0x0f}] = 0x68;
    bus.regs[{m, 0x0f}] = 0x3d;
    lsm9ds1_io io{&bus, fake_read, fake_write, fake_clock};
    lsm9ds1_config cfg{xg, m};
    lsm9ds1_device dev{};
    expect(lsm9ds1_open(&dev, &io, &cfg) == LSM9DS1_OK, "open succeeds");
    return dev;
}

static void put16(FakeBus &bus, uint8_t addr, uint8_t reg, int16_t value) {
    const uint16_t bits = static_cast<uint16_t>(value);
    bus.regs[{addr, reg}] = static_cast<uint8_t>(bits & 0xffu);
    bus.regs[{addr, static_cast<uint8_t>(reg + 1)}] = static_cast<uint8_t>(bits >> 8);
}

static void test_identity_rejection_and_probe_has_no_writes() {
    FakeBus bus;
    auto dev = make_device(bus);
    uint8_t xg = 0, m = 0;
    expect(lsm9ds1_probe(&dev, &xg, &m) == LSM9DS1_OK, "valid identity accepted");
    expect(xg == 0x68 && m == 0x3d, "identities returned");
    expect(bus.writes.empty(), "probe performs no writes");
    bus.regs[{0x6b, 0x0f}] = 0x00;
    expect(lsm9ds1_probe(&dev, nullptr, nullptr) == LSM9DS1_ERROR_IDENTITY,
           "bad identity rejected");
}

static void test_alternate_addresses_and_configuration() {
    FakeBus bus;
    auto dev = make_device(bus, 0x6a, 0x1c);
    expect(lsm9ds1_initialize(&dev) == LSM9DS1_OK, "alternate addresses initialize");
    const std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> expected{
        {0x6a, 0x22, 0x44}, {0x6a, 0x1e, 0x38}, {0x6a, 0x1f, 0x38},
        {0x6a, 0x10, 0x60}, {0x6a, 0x20, 0x60},
        {0x1c, 0x20, 0xf4}, {0x1c, 0x21, 0x00}, {0x1c, 0x22, 0x00},
        {0x1c, 0x23, 0x0c}, {0x1c, 0x24, 0x40}};
    expect(bus.writes == expected, "documented configuration bytes written");
}

static void test_signed_conversions_and_scales() {
    FakeBus bus;
    auto dev = make_device(bus);
    expect(lsm9ds1_initialize(&dev) == LSM9DS1_OK, "initialize succeeds");
    put16(bus, 0x6b, 0x15, 16);
    put16(bus, 0x6b, 0x18, -1000); put16(bus, 0x6b, 0x1a, 1000); put16(bus, 0x6b, 0x1c, -1);
    put16(bus, 0x6b, 0x28, 1000); put16(bus, 0x6b, 0x2a, -1000); put16(bus, 0x6b, 0x2c, 32767);
    put16(bus, 0x1e, 0x28, -1000); put16(bus, 0x1e, 0x2a, 1000); put16(bus, 0x1e, 0x2c, -1);
    lsm9ds1_sample sample{};
    expect(lsm9ds1_read_sample(&dev, &sample) == LSM9DS1_OK, "sample read succeeds");
    near(sample.die_temperature_c, 26.0, 1e-12, "die temperature conversion");
    near(sample.acceleration_m_s2[0], 1000.0 * 0.061e-3 * 9.80665, 1e-12, "accel scale");
    near(sample.acceleration_m_s2[1], -1000.0 * 0.061e-3 * 9.80665, 1e-12, "signed accel");
    near(sample.angular_rate_rad_s[0], -1000.0 * 8.75e-3 * 3.14159265358979323846 / 180.0, 1e-12, "gyro scale");
    near(sample.magnetic_field_t[0], -1000.0 * 0.14e-7, 1e-12, "magnetic scale");
    near(sample.magnetic_field_t[1], 1000.0 * 0.14e-7, 1e-12, "magnetic burst increments Y registers");
    near(sample.magnetic_field_t[2], -1.0 * 0.14e-7, 1e-12, "magnetic burst increments Z registers");
    expect(sample.monotonic_timestamp_ns == bus.now && sample.status == LSM9DS1_OK,
           "timestamp and status reported");
}

static void test_error_propagation_and_last_complete_preservation() {
    FakeBus bus;
    auto dev = make_device(bus);
    expect(lsm9ds1_initialize(&dev) == LSM9DS1_OK, "initialize succeeds");
    put16(bus, 0x6b, 0x15, 0); put16(bus, 0x6b, 0x18, 1); put16(bus, 0x6b, 0x1a, 2); put16(bus, 0x6b, 0x1c, 3);
    put16(bus, 0x6b, 0x28, 4); put16(bus, 0x6b, 0x2a, 5); put16(bus, 0x6b, 0x2c, 6);
    put16(bus, 0x1e, 0x28, 7); put16(bus, 0x1e, 0x2a, 8); put16(bus, 0x1e, 0x2c, 9);
    lsm9ds1_sample first{};
    expect(lsm9ds1_read_sample(&dev, &first) == LSM9DS1_OK, "first complete sample");
    bus.now += 100;
    bus.reads_before_failure = 2;
    put16(bus, 0x6b, 0x18, 1234);
    lsm9ds1_sample failed{};
    expect(lsm9ds1_read_sample(&dev, &failed) == LSM9DS1_ERROR_IO, "partial failure propagated");
    expect(failed.status == LSM9DS1_ERROR_IO, "sample carries error status");
    expect(failed.monotonic_timestamp_ns == first.monotonic_timestamp_ns,
           "last complete timestamp preserved");
    near(failed.angular_rate_rad_s[0], first.angular_rate_rad_s[0], 0.0,
         "partial gyro data not published");
    near(failed.magnetic_field_t[2], first.magnetic_field_t[2], 0.0,
         "last complete sample retained");
}

static void test_failed_reinitialize_clears_initialized_state() {
    FakeBus bus;
    auto dev = make_device(bus);
    expect(lsm9ds1_initialize(&dev) == LSM9DS1_OK, "initial initialize succeeds");
    bus.writes_before_failure = 0;
    expect(lsm9ds1_initialize(&dev) == LSM9DS1_ERROR_IO,
           "failed reinitialize propagates write error");
    lsm9ds1_sample sample{};
    expect(lsm9ds1_read_sample(&dev, &sample) == LSM9DS1_ERROR_NOT_INITIALIZED,
           "failed reinitialize leaves device uninitialized");
}

int main() {
    test_identity_rejection_and_probe_has_no_writes();
    test_alternate_addresses_and_configuration();
    test_signed_conversions_and_scales();
    test_error_propagation_and_last_complete_preservation();
    test_failed_reinitialize_clears_initialized_state();
    std::puts("All LSM9DS1 core tests passed");
    return 0;
}
