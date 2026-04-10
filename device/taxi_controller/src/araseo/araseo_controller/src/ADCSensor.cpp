#include "ADCSensor.hpp"
#include <cstring>
#include <chrono>
#include <thread>

ADCSensor::ADCSensor(int i2c_bus, uint8_t i2c_addr) : i2c_fd(-1), address(i2c_addr) {
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", i2c_bus);
    i2c_fd = open(filename, O_RDWR);
    if (i2c_fd < 0) {
        throw std::runtime_error("Failed to open I2C bus: " + std::string(filename));
    }
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        close(i2c_fd);
        throw std::runtime_error("Failed to connect to ADC at address 0x08");
    }
}

ADCSensor::~ADCSensor() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }
}

uint16_t ADCSensor::readChannel(uint8_t reg) {
    if (write(i2c_fd, &reg, 1) != 1) return 0;
    
    // Wait for conversion (Pinky reference used 6ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    
    uint8_t data[2] = {0, 0};
    if (read(i2c_fd, data, 2) != 2) return 0;
    
    // 12-bit ADC result: (data[0] << 4) | (data[1] >> 4)
    return (uint16_t(data[0]) << 4) | (uint16_t(data[1]) >> 4);
}

ADCSensor::Data ADCSensor::readData() {
    Data d;
    // Map from pinky_pro:
    // 0x98: IR1 (adc[2] in pinky_pro)
    // 0xC8: IR2 (adc[1] in pinky_pro)
    // 0x88: IR3 (adc[0] in pinky_pro)
    // 0xD8: Ultrasonic (adc[3])
    // 0xF8: Battery (adc[4])
    
    // Raw 데이터 읽기
    float ir0_raw = (float)readChannel(0x98);
    float ir1_raw = (float)readChannel(0xC8);
    float ir2_raw = (float)readChannel(0x88);
    float ultra_raw = (float)readChannel(0xD8);
    float batt_raw = (float)readChannel(0xF8);
    
    // 가공 (Converter)
    float ultra_conv = (ultra_raw / 4096.0f) * 1.0f - 0.03f;
    float batt_conv = (batt_raw / 4096.0f) * 4.096f / (13.0f / 28.0f);
    
    // Kalman 필터 적용 (IR 센서용)
    if (is_first_read) {
        kf_ir[0].init(ir0_raw);
        kf_ir[1].init(ir1_raw);
        kf_ir[2].init(ir2_raw);
        last_ultrasonic = ultra_conv;
        last_battery = batt_conv;
        is_first_read = false;
    } else {
        kf_ir[0].update(ir0_raw);
        kf_ir[1].update(ir1_raw);
        kf_ir[2].update(ir2_raw);
        
        // 초음파 및 배터리는 기존 EMA 필터 유지
        last_ultrasonic = (ultra_conv * ALPHA) + (last_ultrasonic * (1.0f - ALPHA));
        last_battery = (batt_conv * ALPHA) + (last_battery * (1.0f - ALPHA));
    }
    
    // 필터링된 값 대입
    d.ir[0] = kf_ir[0].x;
    d.ir[1] = kf_ir[1].x;
    d.ir[2] = kf_ir[2].x;
    d.ultrasonic = last_ultrasonic;
    d.battery = last_battery;
    
    return d;
}
