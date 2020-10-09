#ifndef I2C_FUNCTIONS_H
#define I2C_FUNCTIONS_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include <linux/i2c-dev.h>


#define I2C_NODE "/dev/i2c-"

int i2c_open(int bus);

void i2c_close(int fd);

// Write to an I2C slave device's register:
int i2c_write(int fd, unsigned char slave_addr, unsigned char reg, unsigned char data);

// Read the given I2C slave device's register and return the read value in `*result`:
int i2c_read(int fd, unsigned char slave_addr, unsigned char reg, unsigned char *result);


#endif
