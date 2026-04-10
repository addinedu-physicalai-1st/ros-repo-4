#ifndef ADCSENSOR_HPP
#define ADCSENSOR_HPP

#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <iostream>
#include <cstdint>

class ADCSensor {
public:
    struct Data {
        float ir[3];
        float ultrasonic;
        float battery;
    };

    ADCSensor(int i2c_bus = 1, uint8_t i2c_addr = 0x08);
    ~ADCSensor();

    Data readData();

    struct KalmanFilter1D {
        float x = 0.0f; // Estmated state
        float P = 1.0f; // Error covariance
        float Q = 0.0001f; // Process noise
        float R = 0.01f;   // Measurement noise

        void init(float val) {
            x = val;
            P = 1.0f;
        }

        float update(float measurement) {
            P = P + Q;
            float K = P / (P + R);
            x = x + K * (measurement - x);
            P = (1.0f - K) * P;
            return x;
        }
    };

    int i2c_fd;
    uint8_t address;

    // Kalman 필터 (IR 센서용)
    KalmanFilter1D kf_ir[3];

    // EMA 필터 관련 변수 (초음파, 배터리용)
    float last_ultrasonic = 0.0f;
    float last_battery = 0.0f;
    bool is_first_read = true;
    const float ALPHA = 0.2f; // 필터 강도 (작을수록 부드럽지만 반응은 느려짐)

    uint16_t readChannel(uint8_t reg);
};

#endif // ADCSENSOR_HPP
