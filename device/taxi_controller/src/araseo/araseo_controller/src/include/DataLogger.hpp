#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <sys/stat.h>

class DataLogger {
public:
    DataLogger(const std::string& base_dir = "training_data") : base_dir_(base_dir), is_logging_(false) {}

    bool start() {
        // Create directory
        mkdir(base_dir_.c_str(), 0777);
        std::string img_dir = base_dir_ + "/images";
        mkdir(img_dir.c_str(), 0777);

        csv_file_.open(base_dir_ + "/data_log.csv", std::ios::app);
        if (!csv_file_.is_open()) return false;
        
        csv_file_ << "timestamp,image_path,steering,accelerator,brake,blinker\n";
        is_logging_ = true;
        return true;
    }

    void stop() {
        if (csv_file_.is_open()) csv_file_.close();
        is_logging_ = false;
    }

    void log(const cv::Mat& frame, float steer, float gas, float brake, int blinker) {
        if (!is_logging_) return;

        auto now = std::chrono::system_clock::now();
        auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        
        std::string img_name = "frame_" + std::to_string(ts) + ".jpg";
        std::string img_path = base_dir_ + "/images/" + img_name;

        // Save image
        cv::imwrite(img_path, frame);

        // Save CSV record
        csv_file_ << ts << "," << img_name << "," << steer << "," << gas << "," << brake << "," << blinker << "\n";
    }

private:
    std::string base_dir_;
    std::ofstream csv_file_;
    bool is_logging_;
};

#endif // DATA_LOGGER_HPP
