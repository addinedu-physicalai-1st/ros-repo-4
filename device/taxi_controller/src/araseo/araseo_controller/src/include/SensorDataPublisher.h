#ifndef SENSORDATAPUBLISHER_H
#define SENSORDATAPUBLISHER_H

#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>

#include "SensorDataPubSubTypes.hpp"


class SensorDataPublisher
{
public:
    SensorDataPublisher(int id);
    virtual ~SensorDataPublisher();

    bool init();
    void publish(const SensorData& data);
    void run();

private:
    class PubListener : public eprosima::fastdds::dds::DataWriterListener
    {
    public:
        PubListener() : matched(0) {}
        virtual ~PubListener() override {}

        virtual void on_publication_matched(
                eprosima::fastdds::dds::DataWriter* writer,
                const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

        int matched;
    };

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataWriter* writer_;
    eprosima::fastdds::dds::TypeSupport type_;
    PubListener listener_;
    int m_id;
};


#endif // SENSORDATAPUBLISHER_H
