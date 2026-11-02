#ifndef LSM9DS1_LINUX_I2C_H
#define LSM9DS1_LINUX_I2C_H

#include "lsm9ds1.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int fd; } lsm9ds1_linux_i2c;

int lsm9ds1_linux_discover(char *device_path, size_t path_size,
                           char *error, size_t error_size);
int lsm9ds1_linux_open(lsm9ds1_linux_i2c *bus, const char *device_path,
                       char *error, size_t error_size);
void lsm9ds1_linux_close(lsm9ds1_linux_i2c *bus);
lsm9ds1_io lsm9ds1_linux_io(lsm9ds1_linux_i2c *bus);

#ifdef __cplusplus
}
#endif
#endif
