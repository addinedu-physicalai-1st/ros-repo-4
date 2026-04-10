#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <math.h>

class Battery {
private:
    int i2c_fd;
    int i2c_bus;
    int i2c_address;

    const uint8_t REG_BATTERY = 0xF8;

    int read_adc_channel(uint8_t reg);

public:
    Battery(int bus = 1, int address = 0x08);
    ~Battery();

    bool init();
    float get_voltage();
    float get_percentage();
    void close_battery();
};

#endif // BATTERY_HPP
