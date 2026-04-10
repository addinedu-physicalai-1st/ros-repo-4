#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <iostream>
#include <cstdint>
#ifdef PLATFORM_RASPI
#include <dynamixel_sdk/dynamixel_sdk.h>
#else
namespace dynamixel {
class PortHandler;
class PacketHandler;
}
#endif

// ===== 다이나믹셀 제어 테이블 주소 (XM/XH 시리즈 기준) =====
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PRESENT_VELOCITY   128
#define ADDR_OPERATING_MODE     11

// ===== 프로토콜 버전 =====
#define PROTOCOL_VERSION        2.0

// ===== 기본 설정 =====
#define LEFT_MOTOR_ID           1
#define RIGHT_MOTOR_ID          2
#define BAUDRATE                1000000
#define DEVICE_NAME             "/dev/ttyAMA4"

// 토크 및 속도 값
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define VELOCITY_CONTROL_MODE   1

class Motor {
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    bool left_motor_dir;
    bool right_motor_dir;
    int max_velocity_value;

    void set_operating_mode(uint8_t motor_id, uint8_t mode);
    int speed_to_dxl_velocity(float speed);

public:
    Motor();
    ~Motor();

    bool init();
    void enable_motor(int motor_id = -1);
    void disable_motor(int motor_id = -1);

    void set_left_motor_dir(bool value) { left_motor_dir = value; }
    void set_right_motor_dir(bool value) { right_motor_dir = value; }

    void set_left_motor(float L_speed);
    void set_right_motor(float R_speed);

    void move(float L_speed, float R_speed);
    void stop();
    void close();
};

#endif // MOTOR_HPP
