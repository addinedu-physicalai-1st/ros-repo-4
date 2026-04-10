#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <map>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include "SensorDataPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

// Global Shared State for the JSON writer thread
struct SharedBuffer {
    std::mutex mtx;
    std::string latest_image = "";
    float steering = 0, throttle = 0, brake = 0;
    int blinker = 0; 
    float roll = 0, pitch = 0, yaw = 0, accz = 9.8f; // [신규] IMU 데이터
    bool auto_mode = true;
    int samples = 0;
    bool received_any = false;
} g_buffer;

class DataBridgeListener : public DataReaderListener {
public:
    void on_data_available(DataReader* reader) override {
        SensorData st;
        SampleInfo info;
        while (reader->take_next_sample(&st, &info) == ReturnCode_t::RETCODE_OK) {
            if (info.valid_data) {
                std::lock_guard<std::mutex> lock(g_buffer.mtx);
                if (st.sensor_name() == "CAMERA_LANE") {
                    g_buffer.latest_image = st.status();
                    std::cout << "[DDS Bridge] Received CAMERA_LANE Image! Size: " << st.status().length() << std::endl;
                } else if (st.sensor_name() == "VEHICLE_CONTROL") {
                    if (st.data().size() >= 3) {
                        g_buffer.steering = st.data()[0];
                        g_buffer.throttle = st.data()[1];
                        g_buffer.brake = st.data()[2];
                        if (st.data().size() >= 4) {
                            g_buffer.blinker = (int)st.data()[3];
                        }
                        g_buffer.samples++;
                        if (g_buffer.samples % 10 == 0) {
                            std::cout << "[DDS Bridge] VEHICLE_CONTROL -> Steer: " << g_buffer.steering 
                                      << ", Gas: " << g_buffer.throttle << ", Samples: " << g_buffer.samples << std::endl;
                        }
                    } else {
                        std::cout << "[DDS Bridge] Received VEHICLE_CONTROL but data size is too small: " << st.data().size() << std::endl;
                    }
                } else if (st.sensor_name() == "IMU" && st.data().size() >= 4) {
                    g_buffer.roll = st.data()[0];
                    g_buffer.pitch = st.data()[1];
                    g_buffer.yaw = st.data()[2];
                    g_buffer.accz = st.data()[3];
                } else if (st.sensor_name() == "VEHICLE_STATUS" && st.data().size() >= 3) {
                    g_buffer.auto_mode = (st.data()[0] != 0.0f);
                }
                g_buffer.received_any = true;
                
                // Catch-all monitor for debugging
                static std::map<std::string, int> sensor_counts;
                sensor_counts[st.sensor_name()]++;
                if (sensor_counts[st.sensor_name()] % 30 == 0) {
                   std::cout << "[DDS Bridge] Received: " << st.sensor_name() 
                             << " (Total: " << sensor_counts[st.sensor_name()] << ")" << std::endl;
                }
            }
        }
    }

    void on_subscription_matched(DataReader*, const SubscriptionMatchedStatus& info) override {
        if (info.current_count_change == 1) std::cout << "[DDS Bridge] Publisher Matched!" << std::endl;
        else if (info.current_count_change == -1) std::cout << "[DDS Bridge] Publisher Unmatched." << std::endl;
    }
};

void json_writer_task() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz limit
        
        std::string img;
        float s, t, b;
        float s_roll, s_pitch, s_yaw, s_accz;
        int blink;
        int count;
        bool any;
        bool amode;

        {
            std::lock_guard<std::mutex> lock(g_buffer.mtx);
            img = g_buffer.latest_image;
            s = g_buffer.steering;
            t = g_buffer.throttle;
            b = g_buffer.brake;
            blink = g_buffer.blinker;
            count = g_buffer.samples;
            any = g_buffer.received_any;
            s_roll = g_buffer.roll;
            s_pitch = g_buffer.pitch;
            s_yaw = g_buffer.yaw;
            s_accz = g_buffer.accz;
            amode = g_buffer.auto_mode;
        }

        if (!any) continue;

        // Atomic write to data.json
        std::ofstream file("data.json.tmp");
        file << "{"
             << "\"joy_steering\":" << s << ","
             << "\"joy_throttle\":" << t << ","
             << "\"joy_brake\":" << b << ","
             << "\"joy_blinker\":" << blink << ","
             << "\"samples\":" << count << ","
             << "\"img\":\"" << img << "\","
             << "\"roll\":" << s_roll << ","
             << "\"pitch\":" << s_pitch << ","
             << "\"yaw\":" << s_yaw << ","
             << "\"accz\":" << s_accz << ","
             << "\"auto_mode\":" << (amode ? "true" : "false")
             << "}" << std::endl;
        file.close();
        
        // Atomic rename to avoid dashboard reading partially written file
        std::rename("data.json.tmp", "data.json");
    }
}

int main() {
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("DataBridge_Participant");
    
    // Disable SHM and enforce UDPv4 for Raspberry Pi stability
    pqos.transport().use_builtin_transports = false;
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    // Increase socket buffers for large image frames
    udp_transport->sendBufferSize = 1048576; // 1MB
    udp_transport->receiveBufferSize = 1048576; // 1MB
    pqos.transport().user_transports.push_back(udp_transport);

    uint32_t domain_id = getenv("ROS_DOMAIN_ID") ? std::atoi(getenv("ROS_DOMAIN_ID")) : 204;
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);
    if (!participant) return 1;

    TypeSupport type(new SensorDataPubSubType());
    type.register_type(participant);

    Topic* topic = participant->create_topic("rt/SensorDataTopic", type.get_type_name(), TOPIC_QOS_DEFAULT);
    Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    
    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
    rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
    rqos.history().kind = KEEP_LAST_HISTORY_QOS;
    rqos.history().depth = 2; // Low depth for low latency and memory efficiency

    DataBridgeListener listener;
    DataReader* reader = subscriber->create_datareader(topic, rqos, &listener);

    std::cout << "Data Bridge (DDS -> Web) is active on Domain 0" << std::endl;

    std::thread writer(json_writer_task);
    writer.join();

    return 0;
}
