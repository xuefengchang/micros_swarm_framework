/**
Software License Agreement (BSD)
\file      subscriber.cpp
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

#include <iostream>
#include <string>
#include "ccpp_dds_dcps.h"
#include "opensplice_dds_broker/check_status.h"
#include "opensplice_dds_broker/ccpp_GSDFPacket.h"
#include "opensplice_dds_broker/example_main.h"
#include "opensplice_dds_broker/GSDFPacket_listener.h"

#include "opensplice_dds_broker/subscriber.h"

using namespace DDS;

namespace opensplice_dds_broker{
    
    Subscriber::Subscriber(const std::string& topic_name)
    {
        domain = 0;
        topic_name_ = topic_name.data();
        GSDFPacketTypeName = NULL;
    
        //Create a DomainParticipantFactory and a DomainParticipant (using Default QoS settings)
        dpf = DomainParticipantFactory::get_instance();
        checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
        participant = dpf->create_participant (
            domain,
            PARTICIPANT_QOS_DEFAULT,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(participant, "DDS::DomainParticipantFactory::create_participant");
        
        //Register the required datatype for GSDFPacket
        GSDFPacketTS = new GSDFPacketTypeSupport();
        checkHandle(GSDFPacketTS.in(), "new GSDFPacketTypeSupport");
        GSDFPacketTypeName = GSDFPacketTS->get_type_name();
        status = GSDFPacketTS->register_type(
            participant.in(),
            GSDFPacketTypeName);
        checkStatus(status, "NetworkPartitionsData::GSDFPacketTypeSupport::register_type");

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

        //Use the changed policy when defining the GSDFPacket topic
        GSDFPacketTopic = participant->create_topic(
            topic_name_,
            GSDFPacketTypeName,
            topic_qos,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(GSDFPacketTopic.in(), "DDS::DomainParticipant::create_topic (GSDFPacket)");

        //Adapt the default SubscriberQos to read from the "micros_swarm_framework_partion" Partition
        status = participant->get_default_subscriber_qos (sub_qos);
        checkStatus(status, "DDS::DomainParticipant::get_default_subscriber_qos");
        sub_qos.partition.name.length(1);
        std::string partition_name="micros_swarm_framework_partion";
        sub_qos.partition.name[0] = partition_name.data();

        //Create a Subscriber for the MessageBoard application
        subscriber_ = participant->create_subscriber(sub_qos, NULL, STATUS_MASK_NONE);
        checkHandle(subscriber_.in(), "DDS::DomainParticipant::create_subscriber");
        
        status = subscriber_->get_default_datareader_qos(dr_qos);
        dr_qos.history.kind = KEEP_ALL_HISTORY_QOS;
        //dr_qos.history.kind = KEEP_LAST_HISTORY_QOS;
        dr_qos.destination_order.kind = BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS;
        //dr_qos.durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        dr_qos.durability.kind = VOLATILE_DURABILITY_QOS;

        //Create a DataReader for the NamedMessage Topic (using the appropriate QoS)
        parentReader = subscriber_->create_datareader(
            GSDFPacketTopic.in(),
            dr_qos,
            NULL,
            STATUS_MASK_NONE);
        checkHandle(parentReader, "DDS::Subscriber::create_datareader");

        //Narrow the abstract parent into its typed representative
        GSDFPacketDR = GSDFPacketDataReader::_narrow(parentReader);
        checkHandle(GSDFPacketDR.in(), "NetworkPartitionsData::GSDFPacketDataReader::_narrow");
    }
    
    void Subscriber::subscribe(void (*callBack)(const GSDFPacket& packet))
    {
        GSDFPacketListener *myListener = new GSDFPacketListener();
        myListener->callBack_ = callBack;  //set callBack function
        //myListener->callBack_ = boost::bind(callBack, _1);  //set callBack function
        //myListener->GSDFPacketDR_ = GSDFPacketDataReader::_narrow(GSDFPacketDR.in());
        //checkHandle(myListener->GSDFPacketDR_.in(), "GSDFPacketDataReader::_narrow");

        //DDS::StatusMask mask = DDS::DATA_AVAILABLE_STATUS | DDS::REQUESTED_DEADLINE_MISSED_STATUS;
        DDS::StatusMask mask = DDS::DATA_AVAILABLE_STATUS;
        //myListener->GSDFPacketDR_->set_listener(myListener, mask);
        GSDFPacketDR->set_listener(myListener, mask);
    }
    
    void Subscriber::subscribe(boost::function<void(const GSDFPacket&)> callBack)
    {
        GSDFPacketListener *myListener = new GSDFPacketListener();
        myListener->callBack_ = callBack;  //set callBack function
        //myListener->GSDFPacketDR_ = GSDFPacketDataReader::_narrow(GSDFPacketDR.in());
        //checkHandle(myListener->GSDFPacketDR_.in(), "GSDFPacketDataReader::_narrow");

        //DDS::StatusMask mask = DDS::DATA_AVAILABLE_STATUS | DDS::REQUESTED_DEADLINE_MISSED_STATUS;
        DDS::StatusMask mask = DDS::DATA_AVAILABLE_STATUS;
        //myListener->GSDFPacketDR_->set_listener(myListener, mask);
        GSDFPacketDR->set_listener(myListener, mask);
    }
    
    Subscriber::~Subscriber()
    {
        //Remove the DataReade
        status = subscriber_->delete_datareader(GSDFPacketDR.in());
        checkStatus(status, "DDS::Subscriber::delete_datareader");

        //Remove the Subscriber
        status = participant->delete_subscriber(subscriber_.in());
        checkStatus(status, "DDS::DomainParticipant::delete_subscriber");

        //Remove the Topic
        status = participant->delete_topic(GSDFPacketTopic.in());
        checkStatus(status, "DDS::DomainParticipant::delete_topic (GSDFPacketTopic)");

        //De-allocate the type-names
        string_free(GSDFPacketTypeName);

        //Remove the DomainParticipant
        status = dpf->delete_participant(participant.in());
        checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");
        
        //cout << "Completed subscriber" << endl;
    }
};

