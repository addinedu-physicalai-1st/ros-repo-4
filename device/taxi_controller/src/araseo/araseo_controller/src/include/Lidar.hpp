#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#ifdef PLATFORM_RASPI
#include <sl_lidar.h>
#include <sl_lidar_driver.h>
#else
struct sl_lidar_response_measurement_node_hq_t {
    uint16_t angle_z_q14 = 0;
    uint32_t dist_mm_q2 = 0;
};
namespace sl {
class ILidarDriver;
class IChannel;
}
#endif

#define LIDAR_DEVICE_PORT   "/dev/ttyAMA0"
#define LIDAR_BAUDRATE      460800  // RPLiDAR C1 460800, A1/A2 115200
#define SAFE_STOP_DIST      0.6f    // 60cm 이내면 정지 (안정성 강화)

class Lidar {
private:
    sl::ILidarDriver* driver;
    sl::IChannel* channel;
    std::thread scan_thread;
    std::atomic<bool> is_running;
    std::mutex data_mutex;

    float front_distance; // 전방 장애물 최소 거리 (m)
    float front_left_distance;  // 전방 좌측 거리
    float front_right_distance; // 전방 우측 거리
    float stop_distance;  // 정지 임계 거리 (m)
    float detection_angle = 30.0f; // [신규] 감지 허용 각도 (+/- deg)
    bool obstacle_detected;
    std::vector<sl_lidar_response_measurement_node_hq_t> scan_data;
    
    bool checkHealth();
    void scan_loop();

public:
    Lidar();
    ~Lidar();

    bool init();
    void start_scan();
    void stop_scan();

    float get_front_distance() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return front_distance;
    }

    float get_left_distance() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return front_left_distance;
    }

    float get_right_distance() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return front_right_distance;
    }

    std::vector<sl_lidar_response_measurement_node_hq_t> get_scan_data() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return scan_data;
    }

    bool is_obstacle_detected() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return obstacle_detected;
    }

    void set_stop_distance(float dist) {
        std::lock_guard<std::mutex> lock(data_mutex);
        stop_distance = dist;
    }

    void set_detection_angle(float angle) {
        std::lock_guard<std::mutex> lock(data_mutex);
        detection_angle = angle;
    }

    float get_detection_angle() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return detection_angle;
    }

    void close();
};

#endif // LIDAR_HPP
