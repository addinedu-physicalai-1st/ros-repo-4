#ifndef BLUETOOTHMANAGER_HPP
#define BLUETOOTHMANAGER_HPP

#include <string>
#include <vector>

struct BluetoothDeviceInfo {
    std::string name;
    std::string address;
    int rssi;
    float distance;
};

class BluetoothManager {
public:
    BluetoothManager();
    ~BluetoothManager();

    bool init();
    std::vector<BluetoothDeviceInfo> scan(int duration_sec = 8);
    float estimateDistance(int rssi);

    // New methods for high-speed scan
    void startHighSpeedScan();
    void stopHighSpeedScan();
    std::vector<BluetoothDeviceInfo> getLatestData();

private:
    void updateDeviceList(std::vector<BluetoothDeviceInfo>& devices, const std::string& addr, const std::string& name, int rssi);
    int dev_id;
    int sock;
    bool high_speed_running;
    
    const float TX_POWER_1M = -60.0f; 
    const float PATH_LOSS_EXPONENT = 2.5f; 

    // Helper for command implementation if needed
    // bool hci_le_scan_enable(int sock, uint8_t enable, uint8_t filter_duplicates);
};

#endif // BLUETOOTHMANAGER_HPP
