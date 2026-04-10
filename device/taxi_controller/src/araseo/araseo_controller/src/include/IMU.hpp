#ifndef IMU_HPP
#define IMU_HPP

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cstdint>

class IMU {
public:
    struct Data {
        float euler[3]; // Roll, Pitch, Yaw
        float acc[3];   // X, Y, Z
    };

    IMU(int i2c_bus, const std::string& mode);
    ~IMU();

    Data readData();

private:
    int i2c_fd;
    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void readDataRegs(uint8_t reg, uint8_t* data, int len);
};

#endif
