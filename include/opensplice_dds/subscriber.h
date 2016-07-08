
#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

#include <iostream>
#include <string.h>
#include "ccpp_dds_dcps.h"
#include "check_status.h"
#include "ccpp_MSFPPacket.h"
#include "example_main.h"
#include "MSFPPacket_listener.h"

using namespace DDS;

namespace micros_swarm_framework{
    class Subscriber
    {
        private:
            //int robot_id_;
            DomainId_t  domain;
            ReturnCode_t  status;
            
            const char *topic_name_;
            
            char  *MSFPPacketTypeName;
            
            //Generic DDS entities
            DomainParticipantFactory_var  dpf;
            DomainParticipant_var  participant;
            Topic_var  MSFPPacketTopic;
            Subscriber_var  subscriber_;
            DataReader_ptr  parentReader;

            //Type-specific DDS entities
            MSFPPacketTypeSupport_var  MSFPPacketTS;
            MSFPPacketDataReader_var  MSFPPacketDR;
            MSFPPacketSeq_var  packetSeq;
            SampleInfoSeq_var  infoSeq;

            //QosPolicy holders
            TopicQos  topic_qos;
            SubscriberQos  sub_qos;
            DDS::StringSeq  parameterList;

            //Others
            bool  terminated;
            Duration_t timeout;
            
        public:
            Subscriber(std::string topic_name);
            void subscribe(void (*callBack)(const MSFPPacket& packet));
            void subscribe2();
            ~Subscriber();
    };
};

#endif

