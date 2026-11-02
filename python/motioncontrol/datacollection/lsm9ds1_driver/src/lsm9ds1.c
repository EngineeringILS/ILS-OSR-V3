#include "lsm9ds1.h"

#include <math.h>
#include <string.h>

#define REG_WHO_AM_I 0x0fu
#define WHO_AM_I_XG 0x68u
#define WHO_AM_I_M 0x3du

static int valid_addresses(const lsm9ds1_config *config) {
    return (config->xg_address == LSM9DS1_XG_ADDRESS_DEFAULT ||
            config->xg_address == LSM9DS1_XG_ADDRESS_ALT) &&
           (config->m_address == LSM9DS1_M_ADDRESS_DEFAULT ||
            config->m_address == LSM9DS1_M_ADDRESS_ALT);
}

static lsm9ds1_status read_reg(lsm9ds1_device *device, uint8_t address,
                               uint8_t reg, uint8_t *data, size_t length) {
    return device->io.read(device->io.user, address, reg, data, length) == 0
               ? LSM9DS1_OK : LSM9DS1_ERROR_IO;
}

static lsm9ds1_status write_byte(lsm9ds1_device *device, uint8_t address,
                                 uint8_t reg, uint8_t value) {
    return device->io.write(device->io.user, address, reg, &value, 1u) == 0
               ? LSM9DS1_OK : LSM9DS1_ERROR_IO;
}

static int16_t signed_le(const uint8_t *bytes) {
    uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8u);
    return (int16_t)value;
}

lsm9ds1_config lsm9ds1_default_config(void) {
    lsm9ds1_config config = {LSM9DS1_XG_ADDRESS_DEFAULT,
                             LSM9DS1_M_ADDRESS_DEFAULT};
    return config;
}

lsm9ds1_status lsm9ds1_open(lsm9ds1_device *device,
                            const lsm9ds1_io *io,
                            const lsm9ds1_config *config) {
    lsm9ds1_config selected;
    if (device == NULL || io == NULL || io->read == NULL || io->write == NULL ||
        io->monotonic_clock == NULL) {
        return LSM9DS1_ERROR_ARGUMENT;
    }
    selected = config == NULL ? lsm9ds1_default_config() : *config;
    if (!valid_addresses(&selected)) {
        return LSM9DS1_ERROR_ARGUMENT;
    }
    memset(device, 0, sizeof(*device));
    device->io = *io;
    device->config = selected;
    return LSM9DS1_OK;
}

lsm9ds1_status lsm9ds1_probe(lsm9ds1_device *device,
                             uint8_t *xg_identity,
                             uint8_t *m_identity) {
    uint8_t xg = 0u;
    uint8_t m = 0u;
    lsm9ds1_status status;
    if (device == NULL || device->io.read == NULL) return LSM9DS1_ERROR_ARGUMENT;
    status = read_reg(device, device->config.xg_address, REG_WHO_AM_I, &xg, 1u);
    if (status != LSM9DS1_OK) return status;
    status = read_reg(device, device->config.m_address, REG_WHO_AM_I, &m, 1u);
    if (status != LSM9DS1_OK) return status;
    if (xg_identity != NULL) *xg_identity = xg;
    if (m_identity != NULL) *m_identity = m;
    return (xg == WHO_AM_I_XG && m == WHO_AM_I_M)
               ? LSM9DS1_OK : LSM9DS1_ERROR_IDENTITY;
}

lsm9ds1_status lsm9ds1_initialize(lsm9ds1_device *device) {
    static const struct { uint8_t target; uint8_t reg; uint8_t value; } settings[] = {
        {0u, 0x22u, 0x44u}, /* XG BDU, register-address auto increment, little endian */
        {0u, 0x1eu, 0x38u}, /* gyro XYZ enabled */
        {0u, 0x1fu, 0x38u}, /* accelerometer XYZ enabled */
        {0u, 0x10u, 0x60u}, /* gyro 119 Hz, +/-245 dps */
        {0u, 0x20u, 0x60u}, /* accelerometer 119 Hz, +/-2 g */
        {1u, 0x20u, 0xf4u}, /* temp compensation, ultra-high XY, 20 Hz */
        {1u, 0x21u, 0x00u}, /* magnetometer +/-4 gauss */
        {1u, 0x22u, 0x00u}, /* continuous conversion, I2C enabled */
        {1u, 0x23u, 0x0cu}, /* ultra-high Z, little endian */
        {1u, 0x24u, 0x40u}  /* block data update */
    };
    size_t i;
    lsm9ds1_status status;
    if (device == NULL) return LSM9DS1_ERROR_ARGUMENT;
    device->initialized = 0;
    status = lsm9ds1_probe(device, NULL, NULL);
    if (status != LSM9DS1_OK) return status;
    for (i = 0u; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        uint8_t address = settings[i].target == 0u ? device->config.xg_address
                                                   : device->config.m_address;
        status = write_byte(device, address, settings[i].reg, settings[i].value);
        if (status != LSM9DS1_OK) return status;
    }
    device->initialized = 1;
    return LSM9DS1_OK;
}

lsm9ds1_status lsm9ds1_read_sample(lsm9ds1_device *device,
                                   lsm9ds1_sample *sample) {
    uint8_t temperature[2];
    uint8_t gyro[6];
    uint8_t accel[6];
    uint8_t mag[6];
    lsm9ds1_sample candidate;
    lsm9ds1_status status;
    int axis;
    const double pi = 3.14159265358979323846;
    if (device == NULL || sample == NULL) return LSM9DS1_ERROR_ARGUMENT;
    if (!device->initialized) {
        memset(sample, 0, sizeof(*sample));
        sample->status = LSM9DS1_ERROR_NOT_INITIALIZED;
        return sample->status;
    }
#define READ_OR_FAIL(addr, reg, buffer) do { \
    status = read_reg(device, (addr), (reg), (buffer), sizeof(buffer)); \
    if (status != LSM9DS1_OK) goto failed; \
} while (0)
    READ_OR_FAIL(device->config.xg_address, 0x15u, temperature);
    READ_OR_FAIL(device->config.xg_address, 0x18u, gyro);
    READ_OR_FAIL(device->config.xg_address, 0x28u, accel);
    /* Magnetometer I2C bursts require SUB[7], unlike XG IF_ADD_INC. */
    READ_OR_FAIL(device->config.m_address, 0xa8u, mag);
#undef READ_OR_FAIL
    memset(&candidate, 0, sizeof(candidate));
    candidate.die_temperature_c = (double)signed_le(temperature) / 16.0 + 25.0;
    for (axis = 0; axis < 3; ++axis) {
        candidate.angular_rate_rad_s[axis] =
            (double)signed_le(&gyro[axis * 2]) * 8.75e-3 * pi / 180.0;
        candidate.acceleration_m_s2[axis] =
            (double)signed_le(&accel[axis * 2]) * 0.061e-3 * 9.80665;
        candidate.magnetic_field_t[axis] =
            (double)signed_le(&mag[axis * 2]) * 0.14e-7;
    }
    if (device->io.monotonic_clock(device->io.user,
                                    &candidate.monotonic_timestamp_ns) != 0) {
        status = LSM9DS1_ERROR_TIME;
        goto failed;
    }
    candidate.status = LSM9DS1_OK;
    device->last_complete = candidate;
    device->has_complete_sample = 1;
    *sample = candidate;
    return LSM9DS1_OK;

failed:
    if (device->has_complete_sample) *sample = device->last_complete;
    else memset(sample, 0, sizeof(*sample));
    sample->status = status;
    return status;
}

const char *lsm9ds1_status_string(lsm9ds1_status status) {
    switch (status) {
        case LSM9DS1_OK: return "ok";
        case LSM9DS1_ERROR_ARGUMENT: return "invalid argument or address";
        case LSM9DS1_ERROR_IO: return "I2C input/output error";
        case LSM9DS1_ERROR_IDENTITY: return "unexpected WHO_AM_I value";
        case LSM9DS1_ERROR_NOT_INITIALIZED: return "sensor is not initialized";
        case LSM9DS1_ERROR_TIME: return "monotonic clock error";
        default: return "unknown error";
    }
}
