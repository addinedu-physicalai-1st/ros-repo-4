#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "JoystickManager.hpp"
#include "SensorDataPublisher.h"
#include "SensorData.hpp"

// Sony Racing Wheel T60 - Axis Mapping (Assuming standard Linux Joydev)
#define AXIS_STEERING    0
#define AXIS_ROI_H       5 // ROI Height (w/s)
#define AXIS_ROI_G       4 // ROI Gap (a/d)
#define BTN_MODE_TOGGLE  1 // O Button (Manual/Auto)
#define BTN_CAMERA       0 // X Button
#define BTN_BEV          2 // Square Button
#define BTN_LIDAR        3 // Triangle Button
#define BTN_BRAKE        6 // User reported 6:on
#define BTN_GAS          7 // User reported 7:on
#define BTN_BLINKER_L    4 // L1
#define BTN_BLINKER_R    5 // R1
#define BTN_SPEED_UP     10 // Speed Up (i)
#define BTN_SPEED_DOWN   11 // Speed Down (k)
#define BTN_STOP         12 // Emergency Stop (Space)
#define BTN_SELECT       8  // Lidar Dist Down (Select)
#define BTN_START        9  // Lidar Dist Up (Start)

int main(int argc, char** argv) {
    std::string device = "/dev/input/js0";
    if (argc > 1) device = argv[1];

    JoystickManager joy(device);
    if (!joy.open()) {
        std::cerr << "Failed to open wheel at " << device << std::endl;
        return -1;
    }

    SensorDataPublisher ddsPub(99); // ID 99 for Control
    if (!ddsPub.init()) {
        std::cerr << "Failed to initialize DDS Publisher." << std::endl;
        return -1;
    }

    std::cout << "Sony T60 JoyPublisher Started (Button Pedal Mode)." << std::endl;
    std::cout << "Watching for inputs on " << device << "..." << std::endl;

    bool manual_mode = true;
    int blinker_state = 0; // 0: None, 1: L, 2: R, 3: Hazard

    while (true) {
        js_event event;
        while (joy.readEvent(event)) {
            // Check for buttons
            if (event.type == JS_EVENT_BUTTON && event.value == 1) {
                if (event.number == BTN_MODE_TOGGLE) {
                    manual_mode = !manual_mode;
                    std::cout << "Mode Switched: " << (manual_mode ? "MANUAL" : "AUTO") << std::endl;
                }
                if (event.number == BTN_BLINKER_L) blinker_state = (blinker_state == 1) ? 0 : 1;
                if (event.number == BTN_BLINKER_R) blinker_state = (blinker_state == 2) ? 0 : 2;
            }
        }

        auto state = joy.getState();
        float steer = (float)state.axes[AXIS_STEERING] / 32767.0f;
        
        // Sony T60 Digital Pedals (Buttons 6 and 7)
        float gas = state.buttons[BTN_GAS] ? 1.0f : 0.0f;
        float brake = state.buttons[BTN_BRAKE] ? 1.0f : 0.0f;

        // Publish to DDS
        SensorData st;
        st.sensor_name("VEHICLE_CONTROL");
        
        // 0:Steer, 1:Gas, 2:Brake, 3:Blinker, 4:ManualMode, 5:Axis5(H), 6:Axis4(G), 7:Btn3(L), 8:Btn0(C), 9:Btn2(B), 10:Btn1(M)
        std::vector<float> data = { 
            steer, 
            gas, 
            brake, 
            (float)blinker_state, 
            (float)manual_mode,
            (float)state.axes[AXIS_ROI_H],
            (float)state.axes[AXIS_ROI_G],
            (float)(state.buttons[BTN_LIDAR] ? 1.0f : 0.0f),
            (float)(state.buttons[BTN_CAMERA] ? 1.0f : 0.0f),
            (float)(state.buttons[BTN_BEV] ? 1.0f : 0.0f),
            (float)(state.buttons[BTN_MODE_TOGGLE] ? 1.0f : 0.0f),
            (float)(state.buttons[BTN_SPEED_UP] ? 1.0f : 0.0f),   // 11
            (float)(state.buttons[BTN_SPEED_DOWN] ? 1.0f : 0.0f), // 12
            (float)(state.buttons[BTN_STOP] ? 1.0f : 0.0f),         // 13
            (float)(state.buttons[BTN_SELECT] ? 1.0f : 0.0f),       // 14
            (float)(state.buttons[BTN_START] ? 1.0f : 0.0f),         // 15
            (float)(state.buttons[BTN_BLINKER_L] ? 1.0f : 0.0f),      // 16
            (float)(state.buttons[BTN_BLINKER_R] ? 1.0f : 0.0f)       // 17
        };
        st.data(data);
        ddsPub.publish(st);

        std::cout << "\r[CTRL] Steer: " << std::fixed << std::setprecision(2) << steer 
                  << " | Gas: " << (gas > 0 ? "ON " : "OFF") 
                  << " | Mode: " << (manual_mode ? "MAN" : "AUT")
                  << " | ROI: " << (int)state.axes[AXIS_ROI_H] << "/" << (int)state.axes[AXIS_ROI_G] << "   " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }

    return 0;
}
