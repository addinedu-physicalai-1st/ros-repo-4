#include "Lidar.hpp"
#include <iomanip>
#include <algorithm>

Lidar::Lidar() : driver(nullptr), channel(nullptr), is_running(false), 
                 front_distance(10.0f), front_left_distance(10.0f), front_right_distance(10.0f),
                 stop_distance(SAFE_STOP_DIST), obstacle_detected(false) {
}

Lidar::~Lidar() {
    close();
}

bool Lidar::init() {
#ifdef PLATFORM_PC
    std::cout << "[Lidar] Stub enabled on PC build." << std::endl;
    return true;
#else
    auto driver_res = sl::createLidarDriver();
    if (!driver_res) {
        std::cerr << "[Lidar] Failed to create driver." << std::endl;
        return false;
    }
    driver = *driver_res;

    auto channel_res = sl::createSerialPortChannel(LIDAR_DEVICE_PORT, LIDAR_BAUDRATE);
    if (!channel_res) {
        std::cerr << "[Lidar] Failed to create channel." << std::endl;
        return false;
    }
    channel = *channel_res;

    // Retry connection up to 3 times
    int retry_count = 3;
    bool connected = false;
    while (retry_count-- > 0) {
        if (SL_IS_OK(driver->connect(channel))) {
            connected = true;
            break;
        }
        std::cerr << "[Lidar] Connection attempt failed. Retrying... (" << retry_count << " left)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if (!connected) {
        std::cerr << "[Lidar] Error, cannot bind to the specified serial port " << LIDAR_DEVICE_PORT << std::endl;
        return false;
    }
    
    // Give some time for the connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!checkHealth()) {
        std::cerr << "[Lidar] Health check failed. Attempting deep reset..." << std::endl;
        driver->reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        if (!checkHealth()) {
            std::cerr << "[Lidar] Critical: Health check still FAILED after deep reset." << std::endl;
        } else {
            std::cout << "[Lidar] Recovered from error status via deep reset." << std::endl;
        }
    }

    std::cout << "[Lidar] Successfully connected to SLAMTEC Lidar." << std::endl;
    return true;
#endif
}

bool Lidar::checkHealth() {
#ifdef PLATFORM_PC
    return true;
#else
    if (!driver) return false;
    sl_lidar_response_device_health_t health;
    sl_result res = driver->getHealth(health);
    if (SL_IS_OK(res)) {
        std::cout << "[Lidar] SLAMTEC Lidar health status : " << (int)health.status << std::endl;
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            std::cerr << "[Lidar] Error, internal error detected. Code: " << health.error_code << std::endl;
            return false;
        }
        return true;
    } else {
        std::cerr << "[Lidar] Error, cannot retrieve health info." << std::endl;
        return false;
    }
#endif
}

void Lidar::start_scan() {
#ifdef PLATFORM_PC
    is_running = true;
    std::cout << "[Lidar] Stub scan started." << std::endl;
    return;
#else
    if (!driver) return;
    
    is_running = true;
    driver->setMotorSpeed(600); // RPLiDAR 모터 회전 시작
    driver->startScan(false, true); // 0: force, 1: useTypical
    scan_thread = std::thread(&Lidar::scan_loop, this);
    std::cout << "[Lidar] Scanning Started." << std::endl;
#endif
}

void Lidar::scan_loop() {
#ifdef PLATFORM_PC
    while (is_running) {
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            front_distance = 10.0f;
            front_left_distance = 10.0f;
            front_right_distance = 10.0f;
            obstacle_detected = false;
            scan_data.clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
#else
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 8192;

    while (is_running) {
        count = 8192;
        sl_result ans = driver->grabScanDataHq(nodes, count);
        
        if (SL_IS_OK(ans)) {
            driver->ascendScanData(nodes, count);
            
            float min_dist = 10.0f;
            float left_min = 10.0f;
            float right_min = 10.0f;
            bool found = false;
            int valid_count = 0;

            for (size_t i = 0; i < count; i++) {
                if (nodes[i].dist_mm_q2 == 0) continue; // Invalid data
                valid_count++;

                float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
                float dist = nodes[i].dist_mm_q2 / 4.0f / 1000.0f; // mm to m

                // [수정] 전방 필터 (detection_angle 적용) - 비상 정지용
                if (angle > (360.0f - detection_angle) || angle < detection_angle) {
                    if (dist < min_dist) {
                        min_dist = dist;
                        found = true;
                    }
                }

                // 회피용 섹터 구분 (detection_angle 기준으로 확장)
                if (angle >= (360.0f - detection_angle*1.5f) && angle <= 360.0f) { // Front-Left
                    if (dist < left_min) left_min = dist;
                } else if (angle >= 0.0f && angle <= (detection_angle*1.5f)) { // Front-Right
                    if (dist < right_min) right_min = dist;
                }
            }

            std::lock_guard<std::mutex> lock(data_mutex);
            front_distance = min_dist;
            front_left_distance = left_min;
            front_right_distance = right_min;
            obstacle_detected = (found && min_dist < stop_distance);
            
            // 시각화를 위해 360도 전체 데이터 저장
            scan_data.assign(nodes, nodes + count);
            
            // 디버그 로그 (주기적으로 출력)
            static int ok_cnt = 0;
            if (ok_cnt++ % 50 == 0) {
                // std::cout << "[Lidar] Grabbed: " << count << " points (Valid: " << valid_count << "), MinDist: " << min_dist << "m" << std::endl;
            }
        } else if (ans == SL_RESULT_OPERATION_TIMEOUT) {
            // 타임아웃 발생 시 모터 재확인 및 잠시 대기
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } else {
            static int err_cnt = 0;
            if (err_cnt++ % 50 == 0) {
                std::cerr << "[Lidar] grabScanDataHq failed, result: " << std::hex << ans << std::dec << std::endl;
                // 에러 지속 시 리셋 시도 검토 가능 (현재는 로그만 출력)
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#endif
}

void Lidar::stop_scan() {
    is_running = false;
    if (scan_thread.joinable()) {
        scan_thread.join();
    }
#ifdef PLATFORM_RASPI
    if (driver) {
        driver->stop();
    }
#endif
}

void Lidar::close() {
    stop_scan();
#ifdef PLATFORM_RASPI
    if (driver) {
        driver->disconnect();
        delete driver;
        driver = nullptr;
    }
    if (channel) {
        delete channel;
        channel = nullptr;
    }
#endif
}
