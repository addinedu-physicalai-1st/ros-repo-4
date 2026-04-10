#include "CameraPublisher.hpp"
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include "SensorData.hpp"
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <iostream>
#include <atomic>

extern std::atomic<bool> g_enable_sensor_log;


using namespace eprosima::fastdds::dds;

CameraPublisher::CameraPublisher()
    : participant_(nullptr)
    , publisher_(nullptr)
    , topic_(nullptr)
    , writer_(nullptr)
    , type_(new SensorDataPubSubType())
{
}

CameraPublisher::~CameraPublisher()
{
    if (writer_ != nullptr) {
        publisher_->delete_datawriter(writer_);
    }
    if (publisher_ != nullptr) {
        participant_->delete_publisher(publisher_);
    }
    if (topic_ != nullptr) {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

bool CameraPublisher::init()
{
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("CameraPublisher_Participant");

    // Disable Shared Memory transport and force UDPv4
    pqos.transport().use_builtin_transports = false;
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    // Increase socket buffers for large image frames
    udp_transport->sendBufferSize = 1048576;    // 1MB
    udp_transport->receiveBufferSize = 1048576; // 1MB
    pqos.transport().user_transports.push_back(udp_transport);

    uint32_t domain_id = getenv("ROS_DOMAIN_ID") ? std::atoi(getenv("ROS_DOMAIN_ID")) : 204;
    participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);

    if (participant_ == nullptr)
    {
        return false;
    }
    type_.register_type(participant_);

    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (!publisher_) return false;

    topic_ = participant_->create_topic("rt/SensorDataTopic", type_.get_type_name(), TOPIC_QOS_DEFAULT);
    if (!topic_) return false;

    DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
    wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
    wqos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
    wqos.history().depth = 20;
    
    // Set memory policy to handle variable string sizes reliably
    wqos.endpoint().history_memory_policy = eprosima::fastrtps::rtps::DYNAMIC_RESERVE_MEMORY_MODE;

    writer_ = publisher_->create_datawriter(topic_, wqos, &listener_);
    if (!writer_) return false;

    return true;
}

void CameraPublisher::publish(const std::string& base64_image)
{
    SensorData st;
    st.sensor_name("CAMERA_LANE");
    st.status(base64_image); // 이미지를 status 필드에 담아 전송
    
    ReturnCode_t ret = writer_->write(&st);
    if (ret != ReturnCode_t::RETCODE_OK) {
        if (g_enable_sensor_log.load()) {
            std::cerr << "[DDS] Error: Camera publish failed (Code: " << ret() << ")" << std::endl;
        }
    } else {
        if (g_enable_sensor_log.load()) {
            std::cout << "[DDS] Successfully wrote to DataWriter. Size: " << base64_image.length() << std::endl;
        }
    }
}

void CameraPublisher::publishIMU(float roll, float pitch, float yaw, float accz)
{
    SensorData st;
    st.sensor_name("IMU");
    st.data({roll, pitch, yaw, accz});
    
    ReturnCode_t ret = writer_->write(&st);
    if (ret != ReturnCode_t::RETCODE_OK) {
        // Warning if publish fails
    }
}

void CameraPublisher::publishADC(const ADCSensor::Data& data)
{
    SensorData st;
    st.sensor_name("ADC_SENSORS");
    // data order: [IR1, IR2, IR3, Ultrasonic, Battery]
    st.data({data.ir[0], data.ir[1], data.ir[2], data.ultrasonic, data.battery});
    
    ReturnCode_t ret = writer_->write(&st);
    if (ret != ReturnCode_t::RETCODE_OK) {
        // Warning if publish fails
    }
}

void CameraPublisher::publishStatus(bool auto_mode, int speed, float dist)
{
    SensorData st;
    st.sensor_name("VEHICLE_STATUS");
    st.data({(float)auto_mode, (float)speed, dist});
    
    ReturnCode_t ret = writer_->write(&st);
    if (ret != ReturnCode_t::RETCODE_OK) {
        // Warning if publish fails
    }
}

void CameraPublisher::PubListener::on_publication_matched(
    DataWriter*, const PublicationMatchedStatus& info)
{
    if (info.current_count_change == 1) {
        matched = info.total_count;
        if (g_enable_sensor_log.load()) {
            std::cout << "\n[DDS] Camera Stream matched with a reader." << std::endl;
        }
    } else if (info.current_count_change == -1) {
        matched = info.total_count;
        if (g_enable_sensor_log.load()) {
            std::cout << "\n[DDS] Camera Stream unmatched." << std::endl;
        }
    }
}
