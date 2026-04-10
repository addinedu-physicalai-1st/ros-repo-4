#include "IMU.hpp"
#include <cstdio>
#include <cstring>

#define BNO055_ADDRESS 0x28
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08

IMU::IMU(int i2c_bus, const std::string& mode) {
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", i2c_bus);
    i2c_fd = open(filename, O_RDWR);
    if (i2c_fd < 0) {
        throw std::runtime_error("Failed to open I2C bus: " + std::string(filename));
    }
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        close(i2c_fd);
        throw std::runtime_error("Failed to connect to BNO055 sensor at address 0x28");
    }

    // Set OPR_MODE to NDOF (0x0C) or other requested mode
    uint8_t mode_val = 0x0C; // Default to NDOF
    if (mode == "ndof") mode_val = 0x0C;
    else if (mode == "imu") mode_val = 0x08;
    
    writeReg(BNO055_OPR_MODE_ADDR, mode_val);
    usleep(30000); // Wait for mode switch

    // Check Chip ID
    uint8_t chip_id = readReg(0x00);
    if (chip_id != 0xA0) {
        char err_msg[64];
        sprintf(err_msg, "Unexpected Chip ID: 0x%02X (Expected 0xA0)", chip_id);
        throw std::runtime_error(err_msg);
    }
}

IMU::~IMU() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }
}

void IMU::writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(i2c_fd, buf, 2) != 2) {
        std::cerr << "Warning: Failed to write to register 0x" << std::hex << (int)reg << std::dec << std::endl;
    }
}

uint8_t IMU::readReg(uint8_t reg) {
    if (write(i2c_fd, &reg, 1) != 1) return 0;
    uint8_t val;
    if (read(i2c_fd, &val, 1) != 1) return 0;
    return val;
}

void IMU::readDataRegs(uint8_t reg, uint8_t* data, int len) {
    if (write(i2c_fd, &reg, 1) != 1) {
        std::cerr << "Error: Failed to set I2C register address 0x" << std::hex << (int)reg << std::dec << std::endl;
        return;
    }
    int bytes_read = read(i2c_fd, data, len);
    if (bytes_read != len) {
        std::cerr << "Error: Failed to read " << len << " bytes from register 0x" << std::hex << (int)reg 
                  << std::dec << ". Read: " << bytes_read << std::endl;
    }
}

IMU::Data IMU::readData() {
    Data d;
    uint8_t buf[6];
    memset(buf, 0, 6); // Initialize to zero

    // Read Euler angles (3 * 16-bit = 6 bytes)
    readDataRegs(BNO055_EULER_H_LSB_ADDR, buf, 6);
    int16_t x = ((int16_t)buf[1] << 8) | buf[0];
    int16_t y = ((int16_t)buf[3] << 8) | buf[2];
    int16_t z = ((int16_t)buf[5] << 8) | buf[4];
    d.euler[0] = (float)x / 16.0f;
    d.euler[1] = (float)y / 16.0f;
    d.euler[2] = (float)z / 16.0f;

    memset(buf, 0, 6); // Reset for next read
    // Read Linear Acceleration (3 * 16-bit = 6 bytes)
    readDataRegs(BNO055_ACCEL_DATA_X_LSB_ADDR, buf, 6);
    int16_t ax = ((int16_t)buf[1] << 8) | buf[0];
    int16_t ay = ((int16_t)buf[3] << 8) | buf[2];
    int16_t az = ((int16_t)buf[5] << 8) | buf[4];
    d.acc[0] = (float)ax / 100.0f;
    d.acc[1] = (float)ay / 100.0f;
    d.acc[2] = (float)az / 100.0f;

    return d;
}
