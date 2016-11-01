/**
Software License Agreement (BSD)
\file      publisher.cpp
\authors Xuefeng Chang <changxuefengcn@163.com>
\copyright Copyright (c) 2016, the micROS Team, HPCL (National University of Defense Technology), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of micROS Team, HPCL, nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include "ccpp_dds_dcps.h"
#include "opensplice_dds/check_status.h"
#include "opensplice_dds/ccpp_MSFPPacket.h"
#include "opensplice_dds/example_main.h"
#include "opensplice_dds/publisher.h"

using namespace DDS;

namespace micros_swarm_framework{
    
    Publisher::Publisher(const std::string& topic_name)
    {
        domain = 0;
        topic_name_ = topic_name.data();
        MSFPPacketTypeName = NULL;  
        
        //Create a DomainParticipantFactory and a DomainParticipant, using Default QoS settings
        dpf = DomainParticipantFactory::get_instance ();
        checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
        participant = dpf->create_participant(domain, PARTICIPANT_QOS_DEFAULT, NULL, STATUS_MASK_NONE);
        checkHandle(participant.in(), "DDS::DomainParticipantFactory::create_participant");

        //Register the required datatype
        MSFPPacketTS = new MSFPPacketTypeSupport();
        checkHandle(MSFPPacketTS.in(), "new MSFPPacketTypeSupport");
        MSFPPacketTypeName = MSFPPacketTS->get_type_name();
        status = MSFPPacketTS->register_type(
            participant.in(),
            MSFPPacketTypeName);
        checkStatus(status, "micros_swarm_framework::MSFPPacketTypeSupport::register_type");

        //Set the ReliabilityQosPolicy to BEST_EFFORT_RELIABILITY
        status = participant->get_default_topic_qos(topic_qos);
        checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
        topic_qos.reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
        //topic_qos.reliability.kind = RELIABLE_RELIABILITY_QOS;
        topic_qos.durability_service.history_kind = KEEP_LAST_HISTORY_QOS;
        //topic_qos.durability_service.history_depth = 4000;

        //Make the tailored QoS the new default
        status = participant->set_default_topic_qos(topic_qos);
        checkStatus(status, "DDS::DomainParticipant::set_default_topic_qos");

        //Use the changed policy when defining the MSFPPacket topic
        MSFPPacketTopic = participant->create_topic(
            topic_name_,
            MSFPPacketTypeName,
            topic_qos,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(MSFPPacketTopic.in(), "DDS::DomainParticipant::create_topic (MSFPPacket)");

        //Adapt the default PublisherQos to write into the "micros_swarm_framework_default_partion" Partition
        status = participant->get_default_publisher_qos (pub_qos);
        checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
        pub_qos.partition.name.length(1);
        pub_qos.partition.name[0] = "micros_swarm_framework_partion";

        //Create a Publisher for the application
        publisher_ = participant->create_publisher(pub_qos, NULL, STATUS_MASK_NONE);
        checkHandle(publisher_.in(), "DDS::DomainParticipant::create_publisher");
        
        status = publisher_->get_default_datawriter_qos(dw_qos);
        dw_qos.destination_order.kind = BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS;
        dw_qos.history.kind = KEEP_ALL_HISTORY_QOS;
        //dw_qos.history.kind = KEEP_LAST_HISTORY_QOS;
        //dw_qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        dw_qos.durability.kind = VOLATILE_DURABILITY_QOS;

        //Create a DataWriter for the MSFPPacket Topic (using the appropriate QoS)
        parentWriter = publisher_->create_datawriter(
            MSFPPacketTopic.in(),
            dw_qos,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(parentWriter, "DDS::Publisher::create_datawriter (MSFPPacket)");

        //Narrow the abstract parent into its typed representative
        MSFPPacketDW = MSFPPacketDataWriter::_narrow(parentWriter);
        checkHandle(MSFPPacketDW.in(), "micros_swarm_framework::MSFPPacketDataWriter::_narrow");
    }
    
    void Publisher::publish(const MSFPPacket& packet)
    {
        status = MSFPPacketDW->write(packet, DDS::HANDLE_NIL);
        checkStatus(status, "micros_swarm_framework::MSFPPacketDataWriter::write");
    }
    
    Publisher::~Publisher()
    {
        //Remove the DataWriters 
        status = publisher_->delete_datawriter(MSFPPacketDW.in() );
        checkStatus(status, "DDS::Publisher::delete_datawriter (MSFPPacketDW)");

        //Remove the Publisher
        status = participant->delete_publisher(publisher_.in() );
        checkStatus(status, "DDS::DomainParticipant::delete_publisher");

        status = participant->delete_topic( MSFPPacketTopic.in() );
        checkStatus(status, "DDS::DomainParticipant::delete_topic (MSFPPacketTopic)");

        //Remove the type-names
        string_free(MSFPPacketTypeName);

        //Remove the DomainParticipant
        status = dpf->delete_participant( participant.in() );
        checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");

        //cout << "Completed publisher" << endl;
    }
};
