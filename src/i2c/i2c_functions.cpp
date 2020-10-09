#include "i2c_functions.h"

// Returns a new file descriptor for communicating with the I2C bus:
int i2c_open(int bus) {
    int file;
    char filename[64];

    snprintf(filename, 64, "%s%d", I2C_NODE, bus);

    if ((file = open(filename, O_RDWR)) < 0)
        return -1;

    return file;

    // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
    // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
    // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
    // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
    // for reading device registers which requires a write first before reading the response.
}

void i2c_close(int fd) {
    close(fd);
}

// Write to an I2C slave device's register:
int i2c_write(int fd, unsigned char slave_addr, unsigned char reg, unsigned char data) {
    unsigned char outbuf[2];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    outbuf[0] = reg;
    outbuf[1] = data;

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }
    return 0;
}

// Read the given I2C slave device's register and return the read value in `*result`:
int i2c_read(int fd, unsigned char slave_addr, unsigned char reg, unsigned char *result) {
    unsigned char outbuf[1], inbuf[1];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = slave_addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = 1;
    msgs[1].buf = inbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    outbuf[0] = reg;

    inbuf[0] = 0;

    *result = 0;
    if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }

    *result = inbuf[0];
    return 0;
}
