#define _POSIX_C_SOURCE 200809L
#include "lsm9ds1.h"
#include "lsm9ds1_linux_i2c.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void usage(FILE *stream, const char *program) {
    fprintf(stream,
        "Usage: %s [--bus /dev/i2c-N] [--xg-address 0x6a|0x6b]\n"
        "          [--mag-address 0x1c|0x1e] --probe\n"
        "       %s [bus/address options] --init --samples N\n\n"
        "  --probe       Read and verify both WHO_AM_I registers only; no writes.\n"
        "  --init        Explicitly allow configuration writes before sampling.\n"
        "  --samples N   Read a bounded number of complete samples (N > 0).\n"
        "  --bus PATH    Select /dev/i2c-N explicitly; otherwise discover the single\n"
        "                sysfs adapter whose name contains 'i2c-tiny-usb'.\n"
        "  --help        Show this help.\n\n"
        "Defaults: XG 0x6b, magnetometer 0x1e; alternate addresses are 0x6a/0x1c.\n"
        "Configured mode: accel +/-2 g at 119 Hz, gyro +/-245 dps at 119 Hz,\n"
        "magnetometer +/-4 gauss at 20 Hz. Wiring and logic levels must be 3.3 V.\n",
        program, program);
}

static int parse_address(const char *text, unsigned long first, unsigned long second,
                         uint8_t *result) {
    char *end = NULL;
    unsigned long value;
    errno = 0;
    value = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || (value != first && value != second))
        return -1;
    *result = (uint8_t)value;
    return 0;
}

static int parse_bus_path(const char *text) {
    const char *number;
    if (text == NULL || strncmp(text, "/dev/i2c-", 9u) != 0) return -1;
    number = text + 9;
    if (*number == '\0') return -1;
    while (*number != '\0') {
        if (!isdigit((unsigned char)*number)) return -1;
        ++number;
    }
    return 0;
}

static int parse_samples(const char *text, unsigned long *result) {
    char *end = NULL;
    errno = 0;
    *result = strtoul(text, &end, 10);
    return errno == 0 && end != text && *end == '\0' && *result > 0u ? 0 : -1;
}

int main(int argc, char **argv) {
    const char *bus_argument = NULL;
    char discovered_bus[64];
    char error[256];
    lsm9ds1_config config = lsm9ds1_default_config();
    lsm9ds1_linux_i2c bus = {-1};
    lsm9ds1_io io;
    lsm9ds1_device device;
    int probe = 0;
    int initialize = 0;
    int result = 0;
    unsigned long count = 0u;
    int i;
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) { usage(stdout, argv[0]); return 0; }
        if (strcmp(argv[i], "--probe") == 0) { probe = 1; continue; }
        if (strcmp(argv[i], "--init") == 0) { initialize = 1; continue; }
        if (strcmp(argv[i], "--bus") == 0 && i + 1 < argc) {
            bus_argument = argv[++i];
            if (parse_bus_path(bus_argument) != 0) { fprintf(stderr, "Invalid --bus path; expected /dev/i2c-N\n"); return 2; }
            continue;
        }
        if (strcmp(argv[i], "--samples") == 0 && i + 1 < argc && parse_samples(argv[++i], &count) == 0) continue;
        if (strcmp(argv[i], "--xg-address") == 0 && i + 1 < argc &&
            parse_address(argv[++i], 0x6au, 0x6bu, &config.xg_address) == 0) continue;
        if (strcmp(argv[i], "--mag-address") == 0 && i + 1 < argc &&
            parse_address(argv[++i], 0x1cu, 0x1eu, &config.m_address) == 0) continue;
        fprintf(stderr, "Invalid or incomplete argument: %s\n", argv[i]);
        usage(stderr, argv[0]);
        return 2;
    }
    if (probe == initialize || (initialize && count == 0u)) {
        fprintf(stderr, "Choose exactly --probe, or --init --samples N. Sampling writes are never implicit.\n");
        return 2;
    }
    if (bus_argument == NULL) {
        if (lsm9ds1_linux_discover(discovered_bus, sizeof(discovered_bus), error, sizeof(error)) != 0) {
            fprintf(stderr, "Adapter discovery failed: %s\n", error);
            return 1;
        }
        bus_argument = discovered_bus;
    }
    if (lsm9ds1_linux_open(&bus, bus_argument, error, sizeof(error)) != 0) {
        fprintf(stderr, "%s\n", error);
        return 1;
    }
    io = lsm9ds1_linux_io(&bus);
    if (lsm9ds1_open(&device, &io, &config) != LSM9DS1_OK) {
        fprintf(stderr, "Invalid sensor configuration.\n");
        lsm9ds1_linux_close(&bus);
        return 1;
    }
    if (probe) {
        uint8_t xg = 0u, mag = 0u;
        lsm9ds1_status status = lsm9ds1_probe(&device, &xg, &mag);
        if (status != LSM9DS1_OK) {
            fprintf(stderr, "Probe failed at XG 0x%02x / MAG 0x%02x: %s (read 0x%02x / 0x%02x)\n",
                    config.xg_address, config.m_address, lsm9ds1_status_string(status), xg, mag);
            lsm9ds1_linux_close(&bus);
            return 1;
        }
        printf("LSM9DS1 identified on %s: XG 0x%02x WHO_AM_I=0x%02x; MAG 0x%02x WHO_AM_I=0x%02x\n",
               bus_argument, config.xg_address, xg, config.m_address, mag);
    } else {
        unsigned long sample_number;
        lsm9ds1_status status = lsm9ds1_initialize(&device);
        if (status != LSM9DS1_OK) {
            fprintf(stderr, "Initialization failed: %s\n", lsm9ds1_status_string(status));
            lsm9ds1_linux_close(&bus);
            return 1;
        }
        puts("timestamp_ns,status,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,mag_x_t,mag_y_t,mag_z_t,die_temp_c");
        for (sample_number = 0u; sample_number < count; ++sample_number) {
            lsm9ds1_sample sample;
            struct timespec delay = {0, 50000000L};
            status = lsm9ds1_read_sample(&device, &sample);
            printf("%" PRIu64 ",%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.6g\n",
                   sample.monotonic_timestamp_ns, lsm9ds1_status_string(sample.status),
                   sample.acceleration_m_s2[0], sample.acceleration_m_s2[1], sample.acceleration_m_s2[2],
                   sample.angular_rate_rad_s[0], sample.angular_rate_rad_s[1], sample.angular_rate_rad_s[2],
                   sample.magnetic_field_t[0], sample.magnetic_field_t[1], sample.magnetic_field_t[2],
                   sample.die_temperature_c);
            if (status != LSM9DS1_OK) {
                fprintf(stderr, "Sample %lu failed; CSV row retains the last complete values: %s\n",
                        sample_number + 1u, lsm9ds1_status_string(status));
                result = 1;
            }
            if (sample_number + 1u < count) (void)nanosleep(&delay, NULL);
        }
    }
    lsm9ds1_linux_close(&bus);
    return result;
}
