#ifndef CAMERA_PUBLISHER_HPP
#define CAMERA_PUBLISHER_HPP

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include "SensorDataPubSubTypes.h"
#include "ADCSensor.hpp"
#include <string>

class CameraPublisher {
public:
    CameraPublisher();
    virtual ~CameraPublisher();

    bool init();
    void publish(const std::string& base64_image);
    void publishIMU(float roll, float pitch, float yaw, float accz);
    void publishADC(const ADCSensor::Data& data);
    void publishStatus(bool auto_mode, int speed, float dist);

private:
    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataWriter* writer_;
    eprosima::fastdds::dds::TypeSupport type_;

    class PubListener : public eprosima::fastdds::dds::DataWriterListener {
    public:
        PubListener() : matched(0) {}
        ~PubListener() override {}
        void on_publication_matched(eprosima::fastdds::dds::DataWriter* writer,
                                   const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;
        int matched;
    } listener_;
};

#endif // CAMERA_PUBLISHER_HPP
