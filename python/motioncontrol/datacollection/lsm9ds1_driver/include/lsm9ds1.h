#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LSM9DS1_XG_ADDRESS_DEFAULT 0x6bu
#define LSM9DS1_M_ADDRESS_DEFAULT  0x1eu
#define LSM9DS1_XG_ADDRESS_ALT     0x6au
#define LSM9DS1_M_ADDRESS_ALT      0x1cu

typedef enum {
    LSM9DS1_OK = 0,
    LSM9DS1_ERROR_ARGUMENT = -1,
    LSM9DS1_ERROR_IO = -2,
    LSM9DS1_ERROR_IDENTITY = -3,
    LSM9DS1_ERROR_NOT_INITIALIZED = -4,
    LSM9DS1_ERROR_TIME = -5
} lsm9ds1_status;

typedef int (*lsm9ds1_read_fn)(void *user, uint8_t address, uint8_t reg,
                               uint8_t *data, size_t length);
typedef int (*lsm9ds1_write_fn)(void *user, uint8_t address, uint8_t reg,
                                const uint8_t *data, size_t length);
typedef int (*lsm9ds1_clock_fn)(void *user, uint64_t *nanoseconds);

typedef struct {
    void *user;
    lsm9ds1_read_fn read;
    lsm9ds1_write_fn write;
    lsm9ds1_clock_fn monotonic_clock;
} lsm9ds1_io;

typedef struct {
    uint8_t xg_address;
    uint8_t m_address;
} lsm9ds1_config;

typedef struct {
    double acceleration_m_s2[3];
    double angular_rate_rad_s[3];
    double magnetic_field_t[3];
    double die_temperature_c;
    uint64_t monotonic_timestamp_ns;
    lsm9ds1_status status;
} lsm9ds1_sample;

typedef struct {
    lsm9ds1_io io;
    lsm9ds1_config config;
    lsm9ds1_sample last_complete;
    int initialized;
    int has_complete_sample;
} lsm9ds1_device;

lsm9ds1_config lsm9ds1_default_config(void);
lsm9ds1_status lsm9ds1_open(lsm9ds1_device *device,
                            const lsm9ds1_io *io,
                            const lsm9ds1_config *config);
lsm9ds1_status lsm9ds1_probe(lsm9ds1_device *device,
                             uint8_t *xg_identity,
                             uint8_t *m_identity);
lsm9ds1_status lsm9ds1_initialize(lsm9ds1_device *device);
lsm9ds1_status lsm9ds1_read_sample(lsm9ds1_device *device,
                                   lsm9ds1_sample *sample);
const char *lsm9ds1_status_string(lsm9ds1_status status);

#ifdef __cplusplus
}
#endif
#endif
