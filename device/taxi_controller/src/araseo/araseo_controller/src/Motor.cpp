#include "Motor.hpp"
#include <algorithm>

Motor::Motor() : portHandler(nullptr), packetHandler(nullptr), 
                 left_motor_dir(false), right_motor_dir(false), 
                 max_velocity_value(41) {
}

Motor::~Motor() {
    close();
}

bool Motor::init() {
#ifdef PLATFORM_PC
    std::cout << "[System] Motor stub enabled on PC build." << std::endl;
    return true;
#else
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        std::cerr << "[Error] Failed to open port: " << DEVICE_NAME << std::endl;
        return false;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "[Error] Failed to set baudrate: " << BAUDRATE << std::endl;
        return false;
    }

    set_operating_mode(LEFT_MOTOR_ID, VELOCITY_CONTROL_MODE);
    set_operating_mode(RIGHT_MOTOR_ID, VELOCITY_CONTROL_MODE);

    std::cout << "[System] Motor Initialization Successful." << std::endl;
    return true;
#endif
}

void Motor::set_operating_mode(uint8_t motor_id, uint8_t mode) {
#ifdef PLATFORM_PC
    (void)motor_id;
    (void)mode;
#else
    disable_motor(motor_id);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, mode, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "[Motor #" << (int)motor_id << "] " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cerr << "[Motor #" << (int)motor_id << "] " << packetHandler->getRxPacketError(dxl_error) << std::endl;
    } else {
        std::cout << "[Motor #" << (int)motor_id << "] Success: Velocity Control Mode." << std::endl;
    }
    enable_motor(motor_id);
#endif
}

int Motor::speed_to_dxl_velocity(float speed) {
    speed = std::max(-100.0f, std::min(100.0f, speed));
    return (int)((speed / 100.0f) * max_velocity_value);
}

void Motor::enable_motor(int motor_id) {
#ifdef PLATFORM_PC
    (void)motor_id;
#else
    uint8_t dxl_error = 0;
    if (motor_id == -1) {
        packetHandler->write1ByteTxRx(portHandler, LEFT_MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, RIGHT_MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    } else {
        packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    }
#endif
}

void Motor::disable_motor(int motor_id) {
#ifdef PLATFORM_PC
    (void)motor_id;
#else
    uint8_t dxl_error = 0;
    if (motor_id == -1) {
        packetHandler->write1ByteTxRx(portHandler, LEFT_MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, RIGHT_MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    } else {
        packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }
#endif
}

void Motor::set_left_motor(float L_speed) {
#ifdef PLATFORM_PC
    (void)L_speed;
#else
    if (left_motor_dir) L_speed = -L_speed;
    int velocity = speed_to_dxl_velocity(L_speed);
    uint8_t dxl_error = 0;
    packetHandler->write4ByteTxRx(portHandler, LEFT_MOTOR_ID, ADDR_GOAL_VELOCITY, (uint32_t)velocity, &dxl_error);
#endif
}

void Motor::set_right_motor(float R_speed) {
#ifdef PLATFORM_PC
    (void)R_speed;
#else
    if (right_motor_dir) R_speed = -R_speed;
    // Python 코드 참고: 우측 모터는 방향 특성상 반전(-R_speed)
    int velocity = speed_to_dxl_velocity(-R_speed);
    uint8_t dxl_error = 0;
    packetHandler->write4ByteTxRx(portHandler, RIGHT_MOTOR_ID, ADDR_GOAL_VELOCITY, (uint32_t)velocity, &dxl_error);
#endif
}

void Motor::move(float L_speed, float R_speed) {
    set_left_motor(L_speed);
    set_right_motor(R_speed);
}

void Motor::stop() {
    move(0, 0);
}

void Motor::close() {
#ifdef PLATFORM_PC
    return;
#else
    stop();
    disable_motor();
    if (portHandler) portHandler->closePort();
#endif
}
