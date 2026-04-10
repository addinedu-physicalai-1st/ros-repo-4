#include "JoystickManager.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <mutex>

JoystickManager::JoystickManager(const std::string& device_path)
    : device_path_(device_path), fd_(-1) {}

JoystickManager::~JoystickManager() {
    close();
}

bool JoystickManager::open() {
    fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ == -1) {
        std::cerr << "Could not open joystick: " << device_path_ << " (" << std::strerror(errno) << ")" << std::endl;
        return false;
    }
    return true;
}

void JoystickManager::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool JoystickManager::is_open() const {
    return fd_ != -1;
}

JoystickManager::State JoystickManager::getState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

bool JoystickManager::readEvent(js_event& event) {
    if (fd_ == -1) return false;

    ssize_t bytes = ::read(fd_, &event, sizeof(event));
    if (bytes == sizeof(event)) {
        // Update latest state
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (event.type & JS_EVENT_AXIS) {
            latest_state_.axes[event.number] = event.value;
        } else if (event.type & JS_EVENT_BUTTON) {
            latest_state_.buttons[event.number] = (event.value != 0);
        }
        return true;
    }
    return false;
}
