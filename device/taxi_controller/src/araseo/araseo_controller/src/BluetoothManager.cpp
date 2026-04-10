#include "BluetoothManager.hpp"
#include <iostream>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>
#include <sstream>
#include <regex>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <sys/time.h>
#include <sys/select.h>
#include <iomanip>

// Manually define necessary constants and structs if bluetooth headers are missing
#ifndef AF_BLUETOOTH
#define AF_BLUETOOTH 31
#endif

#ifndef BTPROTO_HCI
#define BTPROTO_HCI 1
#endif

#ifndef SOL_HCI
#define SOL_HCI 0
#endif

#ifndef HCI_FILTER
#define HCI_FILTER 2
#endif

#define HCI_EVENT_PKT 0x04
#define EVT_LE_META_EVENT 0x3E
#define EVT_LE_ADVERTISING_REPORT 0x02
#define HCI_MAX_EVENT_SIZE 260
#define HCI_EVENT_HDR_SIZE 2

// Opcodes and constants for LE scan
#define OCF_LE_SET_SCAN_PARAMETERS 0x000B
#define OCF_LE_SET_SCAN_ENABLE 0x000C
#define OGF_LE_CTL 0x08

struct hci_filter {
    uint32_t type_mask;
    uint32_t event_mask[2];
    uint16_t opcode;
};

struct evt_le_meta_event {
    uint8_t subevent;
    uint8_t data[0];
} __attribute__ ((packed));

struct le_advertising_info {
    uint8_t evt_type;
    uint8_t bdaddr_type;
    uint8_t bdaddr[6]; // bdaddr_t equivalent
    uint8_t length;
    uint8_t data[0];
} __attribute__ ((packed));

// Helper for address to string conversion
void ba2str_local(const uint8_t *ba, char *str) {
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            ba[5], ba[4], ba[3], ba[2], ba[1], ba[0]);
}

BluetoothManager::BluetoothManager() : dev_id(-1), sock(-1), high_speed_running(false) {}
BluetoothManager::~BluetoothManager() {
    stopHighSpeedScan();
}

bool BluetoothManager::init() {
    // Note: This implementation still uses bluetoothctl for general info 
    // but can be extended for raw HCI if needed.
    // For now, let's stick to the user's raw socket logic for high-speed scan.
    return true; 
}

// Keep the previous bluetoothctl-based scan for general inquiry
std::vector<BluetoothDeviceInfo> BluetoothManager::scan(int duration_sec) {
    std::vector<BluetoothDeviceInfo> devices;
    
    // 1. Pre-seed names from known devices
    {
        std::array<char, 128> buffer;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("bluetoothctl devices", "r"), pclose);
        if (pipe) {
            std::regex dev_list_regex("Device ([0-9A-F:]{17}) (.*)");
            while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
                std::string line(buffer.data());
                std::smatch match;
                if (std::regex_search(line, match, dev_list_regex)) {
                    BluetoothDeviceInfo dev;
                    dev.address = match[1].str();
                    dev.name = match[2].str();
                    dev.rssi = 0;
                    dev.distance = -1.0f;
                    devices.push_back(dev);
                }
            }
        }
    }
    
    // 2. Try High-speed raw scan (requires sudo)
    sock = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
    if (sock >= 0) {
        struct hci_filter nf;
        memset(&nf, 0, sizeof(nf));
        nf.type_mask = (1 << HCI_EVENT_PKT);
        nf.event_mask[0] = (1 << (EVT_LE_META_EVENT & 31));
        if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) == 0) {
            std::cout << "Using High-Speed Raw HCI Scan..." << std::endl;
            system("bluetoothctl --timeout 2 scan on > /dev/null 2>&1 &");
            
            uint8_t buf[HCI_MAX_EVENT_SIZE];
            auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < duration_sec) {
                struct timeval tv;
                tv.tv_sec = 1; tv.tv_usec = 0;
                setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
                int len = read(sock, buf, sizeof(buf));
                if (len <= 0) continue;
                if (len < (1 + 2 + 1)) continue;
                evt_le_meta_event *meta = (evt_le_meta_event *)(buf + 3);
                if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;
                le_advertising_info *info = (le_advertising_info *)(meta->data + 1);
                char addr_str[19]; ba2str_local(info->bdaddr, addr_str);
                int8_t rssi = (int8_t)buf[len - 1];

                updateDeviceList(devices, addr_str, "", (int)rssi);
            }
            close(sock);
            return devices;
        }
        close(sock);
    }

    // 3. Fallback to bluetoothctl streaming (No-library, No-sudo requirement)
    std::cout << "Using bluetoothctl Streaming Scan (Backup)..." << std::endl;
    std::string cmd = "bluetoothctl --timeout " + std::to_string(duration_sec) + " scan on";
    std::array<char, 256> buffer;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return devices;

    std::regex rssi_regex("Device ([0-9A-F:]{17}) RSSI: .* \\((-?[0-9]+)\\)");
    std::regex name_new_regex("\\[NEW\\] Device ([0-9A-F:]{17}) (.*)");
    std::regex name_chg_regex("\\[CHG\\] Device ([0-9A-F:]{17}) Name: (.*)");

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        std::string line(buffer.data());
        std::smatch match;
        if (std::regex_search(line, match, rssi_regex)) {
            updateDeviceList(devices, match[1].str(), "", std::stoi(match[2].str()));
        } else if (std::regex_search(line, match, name_new_regex)) {
            updateDeviceList(devices, match[1].str(), match[2].str(), 0);
        } else if (std::regex_search(line, match, name_chg_regex)) {
            updateDeviceList(devices, match[1].str(), match[2].str(), 0);
        }
    }

    return devices;
}

void BluetoothManager::updateDeviceList(std::vector<BluetoothDeviceInfo>& devices, const std::string& addr, const std::string& name, int rssi) {
    auto it = std::find_if(devices.begin(), devices.end(), [&](const BluetoothDeviceInfo& d) {
        return d.address == addr;
    });

    if (it != devices.end()) {
        if (rssi != 0) {
            it->rssi = rssi;
            it->distance = estimateDistance(rssi);
            std::cout << "\r[BLE] " << it->name << " (" << addr << ") | RSSI: " << rssi << " | Dist: " << std::fixed << std::setprecision(2) << it->distance << "m    " << std::flush;
        }
        if (!name.empty()) it->name = name;
    } else {
        BluetoothDeviceInfo dev;
        dev.address = addr;
        dev.name = name.empty() ? "[Unknown]" : name;
        dev.rssi = rssi;
        dev.distance = rssi != 0 ? estimateDistance(rssi) : -1.0f;
        devices.push_back(dev);
    }
}

float BluetoothManager::estimateDistance(int rssi) {
    if (rssi == 0 || rssi == 127) return -1.0f;
    return std::pow(10.0f, (TX_POWER_1M - (float)rssi) / (10.0f * PATH_LOSS_EXPONENT));
}

void BluetoothManager::startHighSpeedScan() {}
void BluetoothManager::stopHighSpeedScan() {}
std::vector<BluetoothDeviceInfo> BluetoothManager::getLatestData() { return {}; }
