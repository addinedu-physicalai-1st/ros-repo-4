#ifndef SENSORDATASUBSCRIBER_H
#define SENSORDATASUBSCRIBER_H

#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>

#include "SensorDataPubSubTypes.hpp"

class SensorDataSubscriber
{
public:
    SensorDataSubscriber();
    virtual ~SensorDataSubscriber();

    bool init();
    void run();

    struct ControlState {
        float steering = 0;
        float accelerator = 0;
        float brake = 0;
        int blinker = 0;
        bool manual_mode = true;
        
        // [신규] 휠 하드웨어 입력
        float roi_h_axis = 0;
        float roi_g_axis = 0;
        bool lidar_btn = false;
        bool camera_btn = false;
        bool bev_btn = false;
        bool mode_btn = false;
        bool speed_up_btn = false;
        bool speed_down_btn = false;
        bool stop_btn = false;
        bool lidar_dist_up_btn = false;
        bool lidar_dist_down_btn = false;
        bool l1_btn = false;
        bool r1_btn = false;
        
        // [신규] 통합 센서 상태
        float imu[3] = {0,0,0}; // R, P, Y
        float ir[3] = {0,0,0};
        float ultra = 0;
        float batt = 0;

        // [신규] 관제 카메라(천장) Ground Truth
        float gt_x = 0;
        float gt_y = 0;
        float gt_yaw = 0;
        bool gt_updated = false;

        std::string latest_image_base64 = "";

        uint32_t samples = 0;
    };

    ControlState getControlState();

private:
    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:
        SubListener() : matched(0), samples(0) {}
        virtual ~SubListener() override {}

        virtual void on_subscription_matched(
                eprosima::fastdds::dds::DataReader* reader,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

        virtual void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override;

        void print_hex_and_ascii(const uint8_t* data, size_t length);

        int matched;
        uint32_t samples;
        SensorDataSubscriber* parent;
    };

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataReader* reader_;
    eprosima::fastdds::dds::TypeSupport type_;
    
    ControlState latest_control_;
    std::mutex control_mutex_;

    SubListener listener_;
};

#endif // SENSORDATASUBSCRIBER_H
