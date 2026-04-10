#include "Battery.hpp"
#include <iostream>
#include <vector>
#include <numeric>

Battery::Battery(int bus, int address) : i2c_bus(bus), i2c_address(address), i2c_fd(-1) {}

Battery::~Battery() {
    close_battery();
}

bool Battery::init() {
    char filename[20];
    sprintf(filename, "/dev/i2c-%d", i2c_bus);
    i2c_fd = open(filename, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "[Battery] I2C Open Error: " << filename << std::endl;
        return false;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
        std::cerr << "[Battery] I2C Slave Address Error: 0x" << std::hex << i2c_address << std::endl;
        return false;
    }
    return true;
}

int Battery::read_adc_channel(uint8_t reg) {
    if (i2c_fd < 0) return -1;
    
    // ADC 레지스터 쓰기 요청
    if (write(i2c_fd, &reg, 1) != 1) return -1;
    
    usleep(1000); // ADC 변환 대기

    uint8_t data[2];
    if (read(i2c_fd, data, 2) != 2) return -1;

    // 12-bit 데이타 조합 (Data[0] << 4) | (Data[1] >> 4)
    return (data[0] << 4) | (data[1] >> 4);
}

float Battery::get_voltage() {
    std::vector<int> readings;
    for (int i = 0; i < 20; ++i) {
        int val = read_adc_channel(REG_BATTERY);
        if (val >= 0) readings.push_back(val);
        usleep(100);
    }

    if (readings.empty()) return 0.0f;

    float avg_adc = std::accumulate(readings.begin(), readings.end(), 0.0f) / readings.size();

    // 전압 분배 비율 및 공식 (battery.py와 동일하게 유지)
    float voltage_divider_ratio = 13.0f / 28.0f;
    float voltage = (avg_adc / 4096.0f) * 4.096f / voltage_divider_ratio;
    
    return voltage;
}

float Battery::get_percentage() {
    float voltage = get_voltage();
    
    // [개선] 8.8V가 표시되는 환경을 고려하여 범위를 조정합니다.
    // 만약 3S 리포라면 12.6V가 100%, 9V가 0% 
    // 만약 2S 리포라면 8.4V가 100%, 6.8V가 0%
    // 8.8V가 찍힌다면 2S보다는 3S 혹은 다른 전압원일 가능성이 큼.
    
    float full_v = 12.6f; 
    float empty_v = 9.0f;

    // 만약 전압이 8.8V 근처라면 2S로 간주하고 범위를 재조정하여 100%가 나오게 함 (원래 로직 유지)
    if (voltage < 9.0f) {
        full_v = 8.4f;
        empty_v = 6.8f;
    }

    float percent = (voltage - empty_v) / (full_v - empty_v) * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f) percent = 0.0f;

    return round(percent * 100.0f) / 100.0f;
}

void Battery::close_battery() {
    if (i2c_fd >= 0) close(i2c_fd);
    i2c_fd = -1;
}
