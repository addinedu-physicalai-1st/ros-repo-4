#ifndef JOYSTICKMANAGER_HPP
#define JOYSTICKMANAGER_HPP

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <linux/joystick.h>

class JoystickManager {
public:
    JoystickManager(const std::string& device_path = "/dev/input/js0");
    ~JoystickManager();

    bool open();
    void close();
    bool is_open() const;

    // Returns a snapshot of current axis values and button states
    struct State {
        std::map<int, int> axes;    // axis index -> value (-32767 to 32767)
        std::map<int, bool> buttons; // button index -> pressed (true/false)
    };

    State getState();
    bool readEvent(js_event& event);

private:
    std::string device_path_;
    int fd_;
    State latest_state_;
    mutable std::mutex state_mutex_;
};

#endif // JOYSTICKMANAGER_HPP
