
#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include <string>
#include <iostream>
#include <vector>
#include "ccpp_dds_dcps.h"
#include "check_status.h"
#include "ccpp_MSFPPacket.h"
#include "example_main.h"

using namespace DDS;

namespace micros_swarm_framework{
    class Publisher
    {
        private:
            unsigned int robot_id_;
            DomainId_t  domain;
            
            const char *topic_name_;
            
            char  *MSFPPacketTypeName;
            
            DomainParticipantFactory_var  dpf;
            DomainParticipant_var  participant;
            Topic_var  MSFPPacketTopic;
            Publisher_var  publisher_;
            DataWriter_ptr  parentWriter;
            TopicQos  topic_qos;
            PublisherQos  pub_qos;
            DataWriterQos  dw_qos;

            InstanceHandle_t  userHandle;
            ReturnCode_t  status;

            MSFPPacketTypeSupport_var  MSFPPacketTS;
            MSFPPacketDataWriter_var  MSFPPacketDW;

            //Sample definitions
            MSFPPacket  *packet_;
            
        public:
            Publisher(std::string topic_name, unsigned int robot_id);
            void publish(MSFPPacket packet);
            ~Publisher();
    };
};

#endif
